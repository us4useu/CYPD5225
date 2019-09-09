/**
 * @file typec_manager.h
 *
 * @brief @{Type-C manager header file.@}
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

#ifndef _TYPEC_MANAGER_H_
#define _TYPEC_MANAGER_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <pd.h>

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
/**
 * @brief This function initializes the Type C manager.
 * This function must be called once during system init.
 * @param port Port index.
 */
void typec_init(uint8_t port);

/**
 * @brief This function starts the Type C functionality in the PD hardware
 * block.
 * @param port Port index.
 */
void typec_start(uint8_t port);

/**
 * @brief This function completely disables the Type C FSM and the Type C
 * functionality in the PD hardware block.
 * @param port Port index.
 */
void typec_stop(uint8_t port);

/**
 * @brief This function checks if the Type C FSM is busy.
 * @param port Port index.
 * @return Returns true if busy, false otherwise.
 */
bool typec_is_busy(uint8_t port);

/**
 * @brief This function synchronizes toggling of both ports so as to avoid
 * frequent wakeup. Synchronization happens only if both ports are toggling.
 */
void typec_sync_toggle(void);

/**
 * @brief This function configures Type C functionality in the PD block when
 * entering deepsleep.
 * @param port Port index.
 */
void typec_deepsleep(uint8_t port);

/**
 * @brief This function configures Type C functionality in the PD block when
 * coming out of deepsleep.
 * @param port Port index.
 */
void typec_wakeup(void);

/**
 * @brief This function implements the Type C state machine.
 * @param port Port index.
 */
void typec_fsm(uint8_t port);

/**
 * @brief This function asserts Rp and deasserts Rd for the specified CC
 * line.
 * @param port Port index.
 * @param channel CC line.
 */
void typec_assert_rp(uint8_t port, uint8_t channel);

/**
 * @brief This function asserts Rd and deasserts Rp for the specified CC
 * line.
 * @param port Port index.
 * @param channel CC line.
 */
void typec_assert_rd(uint8_t port, uint8_t channel);
/**
 * @brief This function changes Rp. If port is in connected state Rp will changed
 * only on active channel. If port is not connected then Rp get updated on both cc
 * channels.
 * @param port Port index.
 * @param rp Desired Rp value.
 */
void typec_change_rp(uint8_t port, rp_term_t rp);

#if DPM_DEBUG_SUPPORT
/**
 * @brief This function pushes current Type-C state to the FSM history buffer
 * @param state Next state to be pushed into the buffer.
 * @param port Port index
 */
void typec_push_to_buf(uint8_t port, uint8_t state);

/**
 * @brief This function returns the Type-C FSM history buffer
 * @param port Port index
 */
uint8_t* get_typec_state_buf(uint8_t port);
#endif /* DPM_DEBUG_SUPPORT */

#endif /* _TYPEC_MANAGER_H_ */

/* End of file */

