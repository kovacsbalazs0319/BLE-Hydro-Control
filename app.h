/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
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

#ifndef APP_H
#define APP_H

#include "em_common.h"
#include "em_cmu.h"

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

/**************************************************************************//**
 * GATT functions.
 *****************************************************************************/
// Updates the Pump Enable characteristic.
sl_status_t update_pump_enable_characteristic(uint8_t data_send);
// Sends notification of the Flow State characteristic.
sl_status_t send_flow_rate_notification(uint16_t data_send);
// Sends notification of the Error characteristic.
sl_status_t send_error_state_notification(uint8_t data_send);
// Update the Send Error characteristic.
sl_status_t update_send_error_characteristic(uint8_t data_send);
// Update the Flow Rate characteristic.
sl_status_t update_flow_rate_characteristic(uint16_t data_send);

uint16_t shared_get_flow_x100(void);
void shared_set_flow_x100(uint16_t v);
uint8_t shared_get_err(void);
void shared_set_err(uint8_t v);


#define SIG_FLOW  (1u << 0)
#define SIG_ERR   (1u << 1)

extern volatile uint16_t g_flow_x100;
extern volatile uint8_t  g_err;


#define SIG_SAMPLE  (1u << 0)   // 0x00000001
#endif // APP_H
