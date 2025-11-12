// -----------------------------------------------------------------------------
// control.c — Hydro block control (pump PWM + flow sensor sampling + erroring)
// -----------------------------------------------------------------------------
//
// Responsibilities of this module:
//   • Drive the pump H-bridge inputs with a fixed, low-duty PWM (I1A=PWM, I1B=LOW).
//   • Count rising edges from a hall-effect flow sensor (e.g., YF-S201).
//   • Periodically compute flow rate in L/min from pulse counts.
//   • Detect "dry run" (pump on but measured flow below a threshold for N seconds).
//   • Surface live telemetry (flow_x100, err) via shared_* accessors and BLE signal.
//   • Allow an optional sink callback for debugging/telemetry fan-out.
//
// Concurrency model & safety notes:
//   • Flow pulses arrive in GPIO IRQ context and increment s_pulses (volatile).
//   • sample_cb() runs from sleeptimer context and snapshots s_pulses with IRQs
//     temporarily disabled to avoid torn reads.
//   • All other state is accessed in task context (enable/disable).
//
// Hardware assumptions:
//   • PUMP_PIN_LOW is held LOW (I1B=0) while I1A is PWM’d => one-quadrant drive.
//   • PWM output is routed via TIMER0 CC0 to PUMP_PIN_PWM.
//   • FLOW_PIN is configured with pull + filter; interrupt on rising edge.
//
// Watch outs / TODOs:
//   • `seconds_since_on` increment scale implicitly depends on sampling period.
//     If you change the period, revisit dry-run timing.
//
// -----------------------------------------------------------------------------

#include "control.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "sl_sleeptimer.h"
#include "app_log.h"
#include <stdbool.h>
#include <stdint.h>
#include "gpiointerrupt.h"
#include "em_timer.h"

#include "app.h"


// ---- Pin layout ------------------------------------------------------------------
// H-bridge (or driver) inputs: I1A (PWM) and I1B (forced LOW). Route CC0 to I1A.
#define PUMP_PORT         gpioPortD
#define PUMP_PIN_PWM      3   // D3 : I1A  -> PWM
#define PUMP_PIN_LOW      2   // D2 : I1B  -> fix LOW

// Flow sensor input: rising edge counting (debounced by input filter)
#define FLOW_PORT         gpioPortC
#define FLOW_PIN          0   // C0 : Flow_data (rising edge count)

// ---- Flow rate sensor parameter (YF-S201) ----------------------------------------
// Calibration: Q [L/min] = F [Hz] / 5.71
// If your specific sensor/hydraulics differ, adjust FLOW_HZ_PER_LPM accordingly.
#define FLOW_HZ_PER_LPM   5.71   // 5.71 Hz == 1 L/min  (Q[L/min] = F[Hz] / 5.71)

// ---- PWM parameters --------------------------------------------------------------
// Fixed PWM duty = PWM_NUM / PWM_DEN (here 1/16 ≈ 6.25%)
#define PWM_HZ          1000u   // 1 kHz
#define PWM_DEN         16u
#define PWM_NUM         1u      // 1/16 ≈ 6,25% duty
#define PWM_TIMER         TIMER0
#define PWM_TIMER_CLOCK   cmuClock_TIMER0
#define PWM_CC_CH         0          // CC0
#define PWM_FREQ_HZ       1000u

// ---- Internal State --------------------------------------------------------------
// s_pulses: incremented in IRQ
static volatile uint32_t s_pulses = 0;
// Timing/compute scratch
static uint32_t s_last_ticks = 0;
static uint32_t s_last_pulses = 0;
static double    s_lpm = 0.0;

// On/off & error latch (error codes set via dry-run/flow detection logic)
static bool     s_enabled = false;
static uint8_t  s_error   = 0;

// Dry-run thresholds:
static double    s_min_lpm_after = 0.2; // bellow 0.2 L/min we give dry error if...
static uint8_t  s_min_after_s   = 3;    // ...it's been the case for 3 seconds

// Optional sink callback to mirror computed telemetry to user code (debugging)
static hydro_sink_t      s_sink = 0;
static void             *s_sink_user = 0;

// Private timer handles (PWM via TIMER HW; sample via sleeptimer)
static sl_sleeptimer_timer_handle_t s_pwm_tmr;
static sl_sleeptimer_timer_handle_t s_sample_tmr;

// ---- Helper Functions ------------------------------------------------------------

// Configure pump pins. I1B hard-low, I1A initially low (PWM off until enabled).
static void pump_gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_LOW, gpioModePushPull, 0); // I1B = 0
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_PWM, gpioModePushPull, 0); // I1A = 0 (off)
}

// Initialize and start HW PWM on TIMER0 CC0 at PWM_FREQ_HZ with duty = 1/PWM_DEN.
// Chooses the smallest prescale that keeps TOP in 16-bit range.
static void pwm_hw_start(void)
{
  CMU_ClockEnable(PWM_TIMER_CLOCK, true);

  uint32_t clk = CMU_ClockFreqGet(PWM_TIMER_CLOCK);
  TIMER_Prescale_TypeDef ps;
  uint32_t top = 0;

  for (ps = timerPrescale1; ps <= timerPrescale1024; ps++) {
    top = (clk / ((1u << ps) * PWM_FREQ_HZ)) - 1u;
    if (top <= 0xFFFFu) break;
  }

  TIMER_Init_TypeDef ti = TIMER_INIT_DEFAULT;
  ti.prescale = ps;
  ti.enable   = false;
  TIMER_Init(PWM_TIMER, &ti);

  TIMER_InitCC_TypeDef tcc = TIMER_INITCC_DEFAULT;
  tcc.mode = timerCCModePWM;
  TIMER_InitCC(PWM_TIMER, PWM_CC_CH, &tcc);

  TIMER_TopSet(PWM_TIMER, top);
  TIMER_CompareSet(PWM_TIMER, PWM_CC_CH, top / PWM_DEN);

  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_PWM, gpioModePushPull, 0);

#if defined(GPIO_TIMER_ROUTEEN_CC0PEN)
  GPIO->TIMERROUTE[0].CC0ROUTE =
      (PUMP_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
    | (PUMP_PIN_PWM << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[0].ROUTEEN |= GPIO_TIMER_ROUTEEN_CC0PEN;
#else
  // in case of other architecture, where ROUTEEN is not present
#endif

  TIMER_Enable(PWM_TIMER, true);
}

// Stop HW PWM and detach route. Also force output low for safe idle.
static void pwm_hw_stop(void)
{
  TIMER_Enable(PWM_TIMER, false);
#if defined(GPIO_TIMER_ROUTEEN_CC0PEN)
  GPIO->TIMERROUTE[0].ROUTEEN &= ~GPIO_TIMER_ROUTEEN_CC0PEN;
#endif
  GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_PWM);
}

// Pump control helper. When ON: keep I1B low and start PWM on I1A.
// When OFF: stop PWM and force both lines LOW (coast/idle).
static void pump_on(bool on)
{
  if (on) {
    GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_LOW); // I1B=0
    pwm_hw_start();
  } else {
      pwm_hw_stop();
    GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_PWM); // I1A=0
    GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_LOW); // I1B=0
  }
}


// ---- IRQ -------------------------------------------------------------------------

// Flow pulse counter interrupt callback (rising edge).
static void flow_irq_cb(uint8_t pin)
{
  (void)pin;
  s_pulses++;
}

// Configure flow input pin with pull+filter and enable rising-edge IRQ.
static void flow_gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(FLOW_PORT, FLOW_PIN, gpioModeInputPullFilter, 1);

    // Interrupt setup, calling on rising edge
    GPIO_ExtIntConfig(FLOW_PORT, FLOW_PIN, FLOW_PIN, true, false, true);

    GPIO_IntClear(1u << FLOW_PIN);

    GPIOINT_Init();
    GPIOINT_CallbackRegister(FLOW_PIN, flow_irq_cb);
    GPIO_IntEnable(1u << FLOW_PIN);
}

// ---- Callback function to calculate the L/min value and set signals --------------
// Periodic sampler: snapshots pulse counter, computes L/min, updates shared signals,
// checks for dry-run, and optionally fans out via s_sink.
static void sample_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle; (void)data;

  // Safe reading
  __disable_irq();
  uint32_t p = s_pulses;
  __enable_irq();

  double dp = p - s_last_pulses;
  s_last_pulses = p;

  // Convert pulses per sampling period to L/min.
  // Assumes sampling period is 1 second so that dp ≈ Hz.
  // If sampling period != 1 s, scale dp by (1 / period_s)
  s_lpm = dp / FLOW_HZ_PER_LPM;

  // Dry run detection. Every 0.25 seconds, reset counter
  static uint8_t seconds_since_on = 0;
  if (s_enabled) {
    if (seconds_since_on < 250) seconds_since_on++;
  } else {
    seconds_since_on = 0;
  }

  // Give error
  if (s_enabled && seconds_since_on >= s_min_after_s && s_lpm < s_min_lpm_after) {
    // the pump is on and the flow rate is bellow the minimum threshold
    // send the error of dryrun
    shared_set_err(1);
  } else if (!s_enabled) {
    shared_set_err(0);
  } else if (!s_enabled && s_lpm > s_min_lpm_after) {
    // flow detection when disabled
    shared_set_err(2);
  }

  // Report flow scaled by 100 (fixed-point for BLE/transport)
  uint16_t flow_x100 = (uint16_t)(s_lpm * 100.0 + 0.5);
  shared_set_flow_x100(flow_x100);

  // Notify BLE stack via external signal; OR multiple bits if needed.
    uint32_t bits = SIG_FLOW | SIG_ERR;   //in case of more signals, logical OR them
    (void)sl_bt_external_signal(bits);    //send an external signal to the BLE stack to process


  if (s_sink) {
    s_sink(shared_get_flow_x100()/100, p, shared_get_err(), s_sink_user);
  }
}



// ---- PUBLIC ----------------------------------------------------------------------

// One-time init: config pump + flow GPIO and snapshot timebase.
void hydro_init(void)
{
  static bool inited = false;
  if (inited) return;
  pump_gpio_init();
  flow_gpio_init();
  s_last_ticks = sl_sleeptimer_get_tick_count();
  inited = true;
}

// Enable/disable the hydro block. Starts/stops sampling and PWM as needed.
// When enabling, (re)initializes counters and clears error latch.
void hydro_enable(bool on)
{

  if (on == s_enabled) return;

  s_enabled = on;
  pump_on(on);

  if (on) {
      sl_status_t sc;
      // Sample frequency set to 1 Hz
      sc = sl_sleeptimer_start_periodic_timer_ms(&s_sample_tmr, 1000, sample_cb, NULL, 0, 0);
      app_log("SAMPLE timer start: 0x%lx\n", (unsigned long)sc);

      s_last_pulses = s_pulses;
      s_error = 0;
    } else {
        // Stop timers and force PWM pin low to fully disable drive
      (void)sl_sleeptimer_stop_timer(&s_pwm_tmr);
      (void)sl_sleeptimer_stop_timer(&s_sample_tmr);
      GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_PWM);
    }
}

// Lightweight accessors for status/telemetry
bool hydro_is_enabled(void) { return s_enabled; }

float hydro_get_flow_lpm(void) { return s_lpm; }
uint32_t hydro_get_pulse_count(void) { return s_pulses; }

// Register/unregister sink callback to receive live updates from sample_cb.
void hydro_set_sink(hydro_sink_t cb, void *user)
{
  //useful for debugging
  s_sink = cb;
  s_sink_user = user;
}
