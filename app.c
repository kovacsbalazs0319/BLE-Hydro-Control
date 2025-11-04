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

static bool report_button_flag = false;

// Updates the Report Button characteristic.
static sl_status_t update_pump_enable_characteristic(void);
// Sends notification of the Report Button characteristic.
static sl_status_t send_flow_rate_notification(void);



static void hydro_ble_sink(float lpm, uint32_t pulses, uint8_t error_code, void *user)
{
  (void)user;

  app_log("Flow: %.2f L/min, pulses=%lu, err=%u\n",
                    (double)lpm, (unsigned long)pulses, (unsigned)error_code);

  // -- Flowrate: L/min * 100 → u16 (LE) --
  uint16_t lpm_x100 = (uint16_t)(lpm * 100.0f + 0.5f);
  sl_status_t sc;

  sc = sl_bt_gatt_server_write_attribute_value(gattdb_flow_rate, 0,
                                               sizeof(lpm_x100), (uint8_t*)&lpm_x100);
  if (sc == SL_STATUS_OK) {
    (void)sl_bt_gatt_server_notify_all(gattdb_flow_rate,
                                       sizeof(lpm_x100), (uint8_t*)&lpm_x100);
  }

  // -- Error: u8 --
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_send_error, 0,
                                               sizeof(error_code), &error_code);
  if (sc == SL_STATUS_OK) {
    (void)sl_bt_gatt_server_notify_all(gattdb_send_error,
                                       sizeof(error_code), &error_code);
  }

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

      // Button events can be received from now on.
      sl_button_enable(SL_SIMPLE_BUTTON_INSTANCE(0));

      // Check the report button state, then update the characteristic and
      // send notification.
      sc = update_pump_enable_characteristic();
      app_log_status_error(sc);

      if (sc == SL_STATUS_OK) {
        sc = send_flow_rate_notification();
        app_log_status_error(sc);
      }
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("Connection opened.\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed.\n");

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
        // the gattdb_report_button characteristic.
        if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
            & sl_bt_gatt_notification) {
          // The client just enabled the notification. Send notification of the
          // current button state stored in the local GATT table.
          app_log_info("Notification enabled.");

          sc = send_flow_rate_notification();
          app_log("Sending Flow rate\r\n");
          app_log_status_error(sc);
        } else {
          app_log_info("Notification disabled.\n");
        }
      }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/***************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 ******************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (SL_SIMPLE_BUTTON_INSTANCE(0) == handle) {
    report_button_flag = true;
  }
}

/***************************************************************************//**
 * Updates the Report Button characteristic.
 *
 * Checks the current button state and then writes it into the local GATT table.
 ******************************************************************************/
static sl_status_t update_pump_enable_characteristic(void)
{
  sl_status_t sc;
  uint8_t data_send;

  switch (sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(0))) {
    case SL_SIMPLE_BUTTON_PRESSED:
      data_send = (uint8_t)SL_SIMPLE_BUTTON_PRESSED;
      break;

    case SL_SIMPLE_BUTTON_RELEASED:
      data_send = (uint8_t)SL_SIMPLE_BUTTON_RELEASED;
      break;

    default:
      // Invalid button state
      return SL_STATUS_FAIL; // Invalid button state
  }

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_pump_enable,
                                               0,
                                               sizeof(data_send),
                                               &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_info("Attribute written: 0x%02x", (int)data_send);
  }

  return sc;
}

/***************************************************************************//**
 * Sends notification of the Report Button characteristic.
 *
 * Reads the current button state from the local GATT database and sends it as a
 * notification.
 ******************************************************************************/
static sl_status_t send_flow_rate_notification(void)
{
  sl_status_t sc;
  uint8_t data_send;
  size_t data_len;

  // Read report button characteristic stored in local GATT database.
  sc = sl_bt_gatt_server_read_attribute_value(gattdb_flow_rate,
                                              0,
                                              sizeof(data_send),
                                              &data_len,
                                              &data_send);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  // Send characteristic notification.
  sc = sl_bt_gatt_server_notify_all(gattdb_flow_rate,
                                    sizeof(data_send),
                                    &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_append(" Notification sent (Flow rate): 0x%02x\n", (int)data_send);
  }
  return sc;
}
