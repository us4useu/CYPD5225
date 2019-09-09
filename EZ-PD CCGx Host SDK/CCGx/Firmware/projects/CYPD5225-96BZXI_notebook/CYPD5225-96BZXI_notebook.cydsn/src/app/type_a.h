/**
 * @file type_a.h
 *
 * @brief @{TYPE-A port header file.@}
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

#ifndef _TYPE_A_H_
#define _TYPE_A_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/**
 * @typedef bc_param_t
 * @brief Struct to handle TYPE-A port status
 */
typedef struct
{
    /* Is TYPEA port enabled? */
    bool type_a_enabled;

    /* Is TYPEA port connected? */
    bool type_a_connected;

    /* Current TYPE-A VBUS in mV units. */
    uint16_t cur_vbus_a;

    /* Whether to perform current based detach detection. */
    bool detach_det;

    /* Type-A current sense deboucne active. */
    bool cur_sense_debounce_active;

    /* SDP Mode enabled. */
    bool sdp_mode_enabled;

    /* The current PWM duty cycle programmed. */
    uint16_t pwm_duty;

    /* The new PWM duty cycle to be programmed. */
    uint16_t new_pwm_duty;

} type_a_status_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Returns status of TYPE-A port.
 *
 * @param None
 *
 * @return TYPE-A port status structure.
 */
type_a_status_t* type_a_get_status(void);

/**
 * @brief Returns state of TYPE-A port.
 *
 * @param None
 *
 * @return TYPE-A port state.
 */
bool type_a_is_idle(void);

/**
 * @brief Configures TYPE-A as SDP.
 *
 * @param Null
 *
 * @return Null
 */
void type_a_enable_sdp(void);

/**
 * @brief Updates TYPE-A port's connection status. The detach_det parameter
 * indicates whether detach detection based on current draw needs to be done.
 * This is required for BC 1.2 and Apple charger modes.
 *
 * @param true if connected detected, false otherwise
 * @param true if detach detection needs to be done
 *
 * @return None.
 */
void type_a_update_status(bool is_connect, bool detach_det);

/**
 * @brief Controls TYPE-A VBUS.
 *
 * @param volt_mV Expected TYPE-A VBUS
 *
 * @return None
 */
void type_a_set_voltage(uint16_t volt_mV);

/**
 * @brief Turns on VBUS for TYPE-A port by enabling DC-DC controller.
 *
 * @param None
 *
 * @return None
 */
void type_a_enable_vbus(void);

/**
 * @brief Turns off VBUS for TYPE-A port by disabling DC-DC controller.
 *
 * @param None
 *
 * @return None
 */
void type_a_disable_vbus(void);

/**
 * @brief Disables TYPE-A port.
 *
 * @param None
 *
 * @return None.
 */
void type_a_port_disable(void);

/**
 * @brief Enables TYPE-A port.
 *
 * @param None
 *
 * @return None.
 */
void type_a_port_enable(void);

/**
 * @brief Detects TYPE-A port partner disconnect using current sense.
 *
 * @param None
 *
 * @return None.
 */
void type_a_detect_disconnect(void);

#endif /* _TYPE_A_H_ */

/* End of File */
