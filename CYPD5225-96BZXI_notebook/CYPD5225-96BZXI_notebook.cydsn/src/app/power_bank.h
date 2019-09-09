/**
 * @file power_bank.h
 *
 * @brief
 *
 *******************************************************************************
 * @copyright
 *
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */
#ifndef _POWER_BANK_H_
#define _POWER_BANK_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/* PWM I GPIO port pin number. */
#define PWM_I_GPIO                              (GPIO_PORT_1_PIN_0)

/* Period value of PWM I. */
#define PWM_I_PERIOD                            (500)

/* Comapre value used to set PWM duty cycle for Max currrent value. */
#define PWM_MAX_CURRENT_COMP_VALUE              (PWM_I_PERIOD)

/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/

/*
 * @typedef pb_battery_state_t
 * @brief List of possible battery states.
 */
typedef enum
{
    PB_BATTERY_STATE_NORMAL =               0,  /* Battery in normal/charged state. */
    PB_BATTERY_STATE_DB_TYPEC_CHARGING =    1,  /* Battery is weak and getting charged over TYPE-C. */
    PB_BATTERY_STATE_DB_NOT_CHARGING =      2,  /* Battery is weak and nothing connected. */
    PB_BATTERY_STATE_DB_TYPEC_WAIT_DC =     3,  /* Battery is weak, TYPE-C port is source and waiting for disconnect. */
    PB_BATTERY_SATTE_MAX =                  4,
} pb_battery_state_t;

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief Sets current limit in sink mode using PWM.
 *
 * @param cur_10mA Desired current value in 10mA units.
 *
 * @return None
 */
void pb_typec_set_current (uint16_t cur_10mA);

/**
 * @brief This function handles events from dpm.
 *
 * @param port PD port index.
 * @param evt PD event.
 *
 * @return None
 */
void pb_event_handler(uint8_t port, uint32_t evt);

/**
 * @brief This function initializes PB battery monitor task.
 *
 * @param Null
 * @param Null
 *
 * @return None
 */
void pb_task_init(void);

/**
 * @brief Battery voltage monitor for Power bank
 *
 * @param None
 *
 * @return None
 */
void pb_bat_monitor(void);

#endif /* _POWER_BANK_H_ */

/* End of File */
