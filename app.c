/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "control.h"
#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;


// Updates the Pump Enable characteristic.
sl_status_t update_pump_enable_characteristic(uint8_t data_send);
// Sends notification of the Flow State characteristic.
sl_status_t send_flow_rate_notification(uint16_t data_send);
// Sends notification of the Error characteristic.
sl_status_t send_error_state_notification(uint8_t data_send);

volatile uint16_t g_flow_x100 = 0;
volatile uint8_t  g_err = 0;

bool ntf_flow_enabled = false;
bool ntf_err_enabled  = false;

uint16_t shared_get_flow_x100(void) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  uint16_t v = g_flow_x100;
  CORE_EXIT_CRITICAL();
  return v;
}
void shared_set_flow_x100(uint16_t v) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  g_flow_x100 = v;
  CORE_EXIT_CRITICAL();
}

uint8_t shared_get_err(void) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  uint8_t v = g_err;
  CORE_EXIT_CRITICAL();
  return v;
}
void shared_set_err(uint8_t v) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  g_err = v;
  CORE_EXIT_CRITICAL();
}

static void hydro_ble_sink(float lpm, uint32_t pulses, uint8_t error_code, void *user)
{
  (void)user;

  app_log("Flow: %.2f L/min, pulses=%lu, err=%u\r\n",
                    (double)lpm, (unsigned long)pulses, (unsigned)error_code);


  // -- Flowrate: L/min * 100 → u16 (LE) --
  /*uint16_t lpm_x100 = (uint16_t)(lpm * 100.0f + 0.5f);
  sl_status_t sc;*/

  /*  sc = sl_bt_gatt_server_write_attribute_value(gattdb_flow_rate, 0,
                                                 sizeof(lpm_x100), (uint8_t*)&lpm_x100);
  if (sc == SL_STATUS_OK) {
    (void)sl_bt_gatt_server_notify_all(gattdb_flow_rate,
                                       sizeof(lpm_x100), (uint8_t*)&lpm_x100);
  } else {
     app_log("GATT write error (%d) on attr %u\r\n", (int)sc, (int)gattdb_flow_rate);
  }*/

  //send_flow_rate_notification();

  // -- Error: u8 --
  /*sc = sl_bt_gatt_server_write_attribute_value(gattdb_send_error, 0,
                                               sizeof(error_code), &error_code);
  if (sc == SL_STATUS_OK) {
    (void)sl_bt_gatt_server_notify_all(gattdb_send_error,
                                       sizeof(error_code), &error_code);
  } else {
      app_log("GATT write error (%d) on attr %u\r\n", (int)sc, (int)gattdb_send_error);
   }*/

  //send_error_state_notification();

  // Ha a pulses-t is publikálnád külön, itt megteheted:
  (void)pulses;
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{


  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  hydro_init();
  hydro_set_sink(hydro_ble_sink, NULL);   // beregisztrálod az 1 soros küldődet

  app_log("handles: flow=%u err=%u\r\n",
          (unsigned)gattdb_flow_rate, (unsigned)gattdb_send_error);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{



  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);

      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);

      // Check the pump enable state, then update the characteristic and
      // send notification.
      sc = update_pump_enable_characteristic(0);
      app_log_status_error(sc);

      if (sc == SL_STATUS_OK) {
        sc = send_flow_rate_notification(0);
        app_log_status_error(sc);
      }
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("Connection opened.\r\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed.\r\n");

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.
    case sl_bt_evt_gatt_server_attribute_value_id:
      // The value of the gattdb_pumo_enable characteristic was changed.
      if (gattdb_pump_enable == evt->data.evt_gatt_server_attribute_value.attribute) {
        uint8_t data_recv;
        size_t data_recv_len;

        // Read characteristic value.
        sc = sl_bt_gatt_server_read_attribute_value(gattdb_pump_enable,
                                                    0,
                                                    sizeof(data_recv),
                                                    &data_recv_len,
                                                    &data_recv);
        (void)data_recv_len;
        app_log_status_error(sc);

        if (sc != SL_STATUS_OK) {
          break;
        }

        // Toggle LED.
        hydro_enable(data_recv);
        app_log("Calling hydro_enable with %d\r\n", data_recv);

      }
      break;

    // -------------------------------
    // This event occurs when the remote device enabled or disabled the
    // notification.
    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (gattdb_flow_rate == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        // A local Client Characteristic Configuration descriptor was changed in
        // the gattdb_flow_rate characteristic.
        app_log("flow_rate event\r\n");
        if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
            & sl_bt_gatt_notification) {
          // The client just enabled the notification. Send notification of the
          // current flow rate stored in the local GATT table.
          app_log("Notification enabled for flow_rate characteristics.\r\n");

          sc = send_flow_rate_notification(0);
          app_log("Sending initial Flow rate\r\n");
          app_log_status_error(sc);
          ntf_flow_enabled =1;

        } else {
          app_log("Notification disabled for flow_rate.\r\n");
          ntf_flow_enabled = 0;
        }
      }
      if (gattdb_send_error == evt->data.evt_gatt_server_characteristic_status.characteristic){
                  // A local Client Characteristic Configuration descriptor was changed in
                          // the gattdb_send_error characteristic.
                          app_log("send_error event\r\n");
                          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
                              & sl_bt_gatt_notification) {
                            // The client just enabled the notification. Send notification of the
                            // current error state stored in the local GATT table.
                            app_log("Notification enabled for send_error characteristics.\r\n");

                            sc = send_error_state_notification(0);
                            app_log("Sending initial Error state\r\n");
                            app_log_status_error(sc);
                            ntf_err_enabled = 1;
              } else {
                  app_log("Notification disabled for send_error.\r\n");
                            ntf_err_enabled = 0;
              }
      }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_system_external_signal_id: {
          app_log("External sig arrived\r\n");
          uint32_t sig = evt->data.evt_system_external_signal.extsignals;
          if (sig & SIG_SAMPLE) {
            // Olvasd ki a cache-elt értékeket és KÜLDJ
            uint16_t flow = shared_get_flow_x100();
            uint8_t  err  = shared_get_err();
            app_log("ntf_flow_enable: %d   ntf_err_enable: %d\r\n",ntf_flow_enabled, ntf_err_enabled);
            if (ntf_flow_enabled) {
                sl_status_t sc = send_flow_rate_notification(flow);
              if (sc) app_log("notify flow sc=%lu (0x%04lx)\r\n",(unsigned long)sc,(unsigned long)sc);
            }
            if (ntf_err_enabled) {
              sl_status_t sc = send_error_state_notification(err);
              if (sc) app_log("notify err sc=%lu (0x%04lx)\r\n",(unsigned long)sc,(unsigned long)sc);
            }
          }
        } break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}


/***************************************************************************//**
 * Updates the Pump enable characteristic.
 *
 * Checks the current button state and then writes it into the local GATT table.
 ******************************************************************************/
sl_status_t update_pump_enable_characteristic(uint8_t data_send)
{
  sl_status_t sc;


  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_pump_enable,
                                               0,
                                               sizeof(data_send),
                                               &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_info("Attribute written(pump_enable): 0x%02x\r\n", (int)data_send);
  }

  return sc;
}

/***************************************************************************//**
 * Updates the Send Error characteristic.
 *
 * Checks the current button state and then writes it into the local GATT table.
 ******************************************************************************/
sl_status_t update_send_error_characteristic(uint8_t data_send)
{
  sl_status_t sc;


  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_send_error,
                                               0,
                                               sizeof(data_send),
                                               &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_info("Attribute written(send_error): 0x%02x\r\n", (int)data_send);
  }

  return sc;
}

/***************************************************************************//**
 * Updates the Flow Rate characteristic.
 *
 * Checks the current button state and then writes it into the local GATT table.
 ******************************************************************************/
sl_status_t update_flow_rate_characteristic(uint16_t data_send)
{
  sl_status_t sc;

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_flow_rate,
                                               0,
                                               sizeof(data_send),
                                               &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_info("Attribute written(flow_rate): 0x%02x\r\n", (int)data_send);
  }

  return sc;
}

/***************************************************************************//**
 * Sends notification of the Flow rate characteristic.
 *
 * Reads the current button state from the local GATT database and sends it as a
 * notification.
 ******************************************************************************/
sl_status_t send_flow_rate_notification(uint16_t data_send)
{
  sl_status_t sc;
  app_log("data to send (flow_rate): %d", data_send);
  size_t data_len = sizeof(data_send);

  // Read flow rate characteristic stored in local GATT database.
  /*sc = sl_bt_gatt_server_read_attribute_value(gattdb_flow_rate,
                                              0,
                                              sizeof(data_send),
                                              &data_len,
                                              &data_send);
  if (sc != SL_STATUS_OK) {
    app_log("Cannot read gattdb_flow_rate.\r\n");
    return sc;
  }*/
  app_log("gattdb_flow_rate vaules: len:%d data:%d.\r\n",data_len, data_send);
  // Send characteristic notification.
  sc = sl_bt_gatt_server_notify_all(gattdb_flow_rate,
                                    sizeof(data_send),
                                    &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_append(" Notification sent (Flow rate): 0x%02x\r\n", (int)data_send);
  }else {
      app_log("Cannot send gattdb_flow_rate.\r\n   sc = %d", sc);
  }
  return sc;
}

/***************************************************************************//**
 * Sends notification of the Error characteristic.
 *
 * Reads the current button state from the local GATT database and sends it as a
 * notification.
 ******************************************************************************/
sl_status_t send_error_state_notification(uint8_t data_send)
{
  sl_status_t sc;
  size_t data_len = sizeof(data_send);

  app_log("data to send (send_error): %d", data_send);

  // Error error state characteristic stored in local GATT database.
 /* sc = sl_bt_gatt_server_read_attribute_value(gattdb_send_error,
                                              0,
                                              sizeof(data_send),
                                              &data_len,
                                              &data_send);
  if (sc != SL_STATUS_OK) {
      app_log("Cannot read gattdb_send_error.\r\n");
    return sc;
  }*/

  // Send characteristic notification.
  sc = sl_bt_gatt_server_notify_all(gattdb_send_error,
                                    sizeof(data_send),
                                    &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_append(" Notification sent (Error state): 0x%02x\r\n", (int)data_send);
    app_log("gattdb_send_error vaules: len:%d data:%d.\r\n",data_len, data_send);
  }else {
      app_log("Cannot send gattdb_send_error.\r\n  sc = %d\r\n", sc);
      app_log("notify(send_error) sc=%lu (0x%04lx)\r\n",
              (unsigned long)sc, (unsigned long)sc);
  }
  return sc;
}
