/**
 * @file pd_policy_engine.h
 *
 * @brief @{USB-PD policy engine header file.@}
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

#ifndef _PD_POLICY_ENGINE_H_
#define _PD_POLICY_ENGINE_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function initializes the policy engine. This function also initializes
 * the pd protocol module.
 * @param port port index
 */
void pe_init(uint8_t port);

/**
 * @brief This function runs pe state machine.
 * @param port port index
 */
void pe_fsm(uint8_t port);

/**
 * @brief This function initialize the internal variables of policy engine to
 * known state and start the pd phy
 * @param port port index
 */
void pe_start(uint8_t port);

/**
 * @brief Completely stops the policy engine. This will stop pd phy as well.
 * @param port port index
 */
void pe_stop(uint8_t port);

/**
 * @brief Checks if policy engine is busy in some task.
 * @param port port index
 * @return Returns true if busy else return false
 */
bool pe_is_busy(uint8_t port);

/**
 * @brief Clears hard reset count
 * @param port port index
 * @return None
 */
void pe_clear_hard_reset_count(uint8_t port);

/**
 * @brief Checks if spec rev bit in dpm_status register is valid.
 * @param port port index
 * @return Returns true if spec rev is determined else return false
 */
bool get_spec_rev_determined(uint8_t port);

/**
 * @brief Disables the policy engine. PD port is limited to receiving hard resets.
 * @param port port index
 */
void pe_disabled(uint8_t port);

/**
 * @brief This function pushes current Policy Engine state to the FSM history buffer.
 * @param port port index
 * @param state Next state to be pushed into the buffer.
 */
void pe_push_to_buf(uint8_t port, uint8_t state);

/**
 * @brief This function returns the Policy Engine FSM history buffer.
 * @param port port index
 */
uint8_t* get_pe_state_buf(uint8_t port);

/**
 * @brief This function returns the PPS status.
 * @param port port index
 * @param pps_status PPS status data as defined by USB-PD specification.
 */
void pe_get_pps_status(uint8_t port, uint8_t *pps_status);

#endif /* _PD_POLICY_ENGINE_H_ */
/* End of File */

