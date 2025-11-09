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


// ---- Pin kiosztás ----------------------------------------------------------
#define PUMP_PORT         gpioPortD
#define PUMP_PIN_PWM      3   // D3 : I1A  -> PWM
#define PUMP_PIN_LOW      2   // D2 : I1B  -> fix LOW

#define FLOW_PORT         gpioPortC
#define FLOW_PIN          0   // C0 : Flow_data (rising edge count)

// ---- Átfolyásszenzor konstansok (YF-S201) --------------------------
#define FLOW_HZ_PER_LPM   5.71   // 5.71 Hz == 1 L/min  (Q[L/min] = F[Hz] / 5.71)

// ---- PWM paraméterek -------------------------------------------------------
#define PWM_HZ          1000u   // 1 kHz
#define PWM_DEN         16u
#define PWM_NUM         1u      // 1/16 ≈ 6,25% duty
#define PWM_TIMER         TIMER0
#define PWM_TIMER_CLOCK   cmuClock_TIMER0
#define PWM_CC_CH         0          // CC0
#define PWM_FREQ_HZ       1000u

// ---- Belső állapot ---------------------------------------------------------
static volatile uint32_t s_pulses = 0;
static uint32_t s_last_ticks = 0;
static uint32_t s_last_pulses = 0;
static double    s_lpm = 0.0;

static bool     s_enabled = false;
static uint8_t  s_error   = 0;

static double    s_min_lpm_after = 0.2; // pl. 0.2 L/min alatt hibának vesszük...
static uint8_t  s_min_after_s   = 3;    // ...3 mp-vel bekapcs után

static hydro_sink_t      s_sink = 0;
static void             *s_sink_user = 0;

static sl_sleeptimer_timer_handle_t s_pwm_tmr;
static sl_sleeptimer_timer_handle_t s_sample_tmr;

// ---- Segédfüggvények -------------------------------------------------------
static void pump_gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_LOW, gpioModePushPull, 0); // I1B = 0
  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_PWM, gpioModePushPull, 0); // I1A = 0 (off)
}

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
  TIMER_CompareSet(PWM_TIMER, PWM_CC_CH, top / PWM_DEN);  // 1/32 duty

  GPIO_PinModeSet(PUMP_PORT, PUMP_PIN_PWM, gpioModePushPull, 0);

#if defined(GPIO_TIMER_ROUTEEN_CC0PEN)
  GPIO->TIMERROUTE[0].CC0ROUTE =
      (PUMP_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
    | (PUMP_PIN_PWM << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[0].ROUTEEN |= GPIO_TIMER_ROUTEEN_CC0PEN;
#else
  // ha a te szilíciumodon ROUTELOCx API van, ott állítsd be a helyet és PEN biteket
#endif

  TIMER_Enable(PWM_TIMER, true);
}

static void pwm_hw_stop(void)
{
  TIMER_Enable(PWM_TIMER, false);
#if defined(GPIO_TIMER_ROUTEEN_CC0PEN)
  GPIO->TIMERROUTE[0].ROUTEEN &= ~GPIO_TIMER_ROUTEEN_CC0PEN;
#endif
  GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_PWM);
}


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

// C0 -> even line (0)
// ---- IRQ ----

static void flow_irq_cb(uint8_t pin)   // <- ÚJ: driver callback
{
  (void)pin;
  s_pulses++;
}

static void flow_gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(FLOW_PORT, FLOW_PIN, gpioModeInputPullFilter, 1);

    // EXTINT: C0 -> rising edge
    GPIO_ExtIntConfig(FLOW_PORT, FLOW_PIN, FLOW_PIN, true, false, true);

    GPIO_IntClear(1u << FLOW_PIN);

    GPIOINT_Init();
    GPIOINT_CallbackRegister(FLOW_PIN, flow_irq_cb);
    GPIO_IntEnable(1u << FLOW_PIN);
}

// ---- kiszámolja a L/perc-et és meghívja a sink-et ----
static void sample_cb(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle; (void)data;

  // biztonságos olvasás
  __disable_irq();
  uint32_t p = s_pulses;
  __enable_irq();

  double dp = p - s_last_pulses;
  s_last_pulses = p;

  s_lpm = dp / FLOW_HZ_PER_LPM;

  // egyszerű dry-run hiba: csak a bekapcsolás utáni n. másodperctől figyeljük
  static uint8_t seconds_since_on = 0;
  if (s_enabled) {
    if (seconds_since_on < 250) seconds_since_on++;
  } else {
    seconds_since_on = 0;
  }

  if (s_enabled && seconds_since_on >= s_min_after_s && s_lpm < s_min_lpm_after) {
    s_error = 1; // dry
    shared_set_err(1);
  } else if (!s_enabled) {
    s_error = 0;
    shared_set_err(0);
  }

  uint16_t flow_x100 = (uint16_t)(s_lpm * 100.0 + 0.5);
  shared_set_flow_x100(flow_x100);


    uint32_t bits = SIG_FLOW | SIG_ERR;
    //g_sig_bits |= bits;                  // ha több minta jön, össze-OR-oljuk
    (void)sl_bt_external_signal(bits);   // csak jelezni szabad ISR-ből


  if (s_sink) {
    s_sink(s_lpm, p, s_error, s_sink_user);
  }
}



// ---- PUBLIC ----
void hydro_init(void)
{
  static bool inited = false;
  if (inited) return;
  pump_gpio_init();
  flow_gpio_init();
  s_last_ticks = sl_sleeptimer_get_tick_count();
  inited = true;
}


void hydro_enable(bool on)
{

  if (on == s_enabled) return;

  s_enabled = on;
  pump_on(on);

  if (on) {
      sl_status_t sc;
      // Mintavétel: 1000 ms
      sc = sl_sleeptimer_start_periodic_timer_ms(&s_sample_tmr, 1000, sample_cb, NULL, 0, 0);
      app_log("SAMPLE timer start: 0x%lx\n", (unsigned long)sc);

      switch (sc) {
        case SL_STATUS_NO_MORE_RESOURCE:
          app_log(" SL_STATUS_NO_MORE_RESOURCE");
          break;
        case SL_STATUS_ALREADY_EXISTS:
                 app_log("SL_STATUS_ALREADY_EXISTS");
                 break;
        case SL_STATUS_INVALID_STATE:
                 app_log("SL_STATUS_INVALID_STATE");
                 break;
        default:
          app_log("Default case");
          break;
      }



      s_last_pulses = s_pulses;
      s_error = 0;
    } else {
      (void)sl_sleeptimer_stop_timer(&s_pwm_tmr);
      (void)sl_sleeptimer_stop_timer(&s_sample_tmr);
      GPIO_PinOutClear(PUMP_PORT, PUMP_PIN_PWM);
    }
}

bool hydro_is_enabled(void) { return s_enabled; }

float hydro_get_flow_lpm(void) { return s_lpm; }
uint32_t hydro_get_pulse_count(void) { return s_pulses; }

void hydro_set_sink(hydro_sink_t cb, void *user)
{
  s_sink = cb;
  s_sink_user = user;
}
