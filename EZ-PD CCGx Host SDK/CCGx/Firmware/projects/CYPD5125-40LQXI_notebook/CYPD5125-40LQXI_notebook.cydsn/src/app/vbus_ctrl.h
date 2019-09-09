/**
 * @file vbus_ctrl.h
 *
 * @brief @{VBUS Control header file.@}
 */

/*
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

#ifndef _VBUS_CTRL_H_
#define _VBUS_CTRL_H_

#include <stdbool.h>
#include <stdint.h>
#include <status.h>

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief The function computes PWM duty cycle needed to generate requested VBUS.
 *
 * @param vbus The voltage to set to in mV units
 * @param min_volt Minimum VBUS as per design in mv
 * @param max_volt Maximum VBUS as per design in mv
 * @param pwm_period Period of the PWM signal.
 *
 * @return PWM duty cycle in terms of PWM on time.
 */
uint16_t pwm_get_duty_cycle(uint16_t vbus, uint16_t min_volt, uint16_t max_volt,
    uint16_t pwm_period);

/**
 * @brief The function enables the PWM signal that turns VBus ON.
 * @param port Port index
 * @return None
 */
void vbus_ctrl_pwm_turn_on(uint8_t port);

/**
 * @brief The function disables the PWM signal thus turning VBus OFF.
 * @param port Port index
 * @return None
 */
void vbus_ctrl_pwm_turn_off(uint8_t port);

/**
 * @brief The function sets the Type-C VBUS voltage to requested level using PWM.
 *
 * @param port TYPE-C Port index
 * @param volt_mV The voltage to set to in mV units.
 * @return None
 */
void vbus_ctrl_pwm_set_volt(uint8_t port, uint16_t volt_mV);

/**
 * @brief The function enables the Type-C VBUS voltage feedback logic.
 *
 * @param port TYPE-C Port index
 * @return None
 */
void vbus_ctrl_fb_enable(uint8_t port);

/**
 * @brief The function disables the Type-C VBUS voltage feedback logic.
 *
 * @param port TYPE-C Port index
 * @return None
 */
void vbus_ctrl_fb_disable(uint8_t port);

/**
 * @brief The function sets the Type-C VBUS voltage to requested level using FB (feedback).
 *
 * @param port TYPE-C Port index
 * @param volt_mV The voltage to set to in mV units.
 * @return None
 */
void vbus_ctrl_fb_set_volt(uint8_t port, uint16_t volt_mV);

#endif /* _VBUS_CTRL_H_ */

/* End of File */
