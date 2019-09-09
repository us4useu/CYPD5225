/**
 * @file alt_mode_hw.h
 *
 * @brief @{Hardware control for Alternate mode implementation.@}
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

#ifndef _ALT_MODE_HW_H_
#define _ALT_MODE_HW_H_

/*****************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <config.h>

/*****************************************************************************
 * MACRO Definition
 *****************************************************************************/

/**
  @brief Zero data.
 */
#define NO_DATA                                 (0u)
/**
  @brief Enable HPD application command value.
 */
#define HPD_ENABLE_CMD                          (0u)
/**
  @brief Disable HPD application command value.
 */
#define HPD_DISABLE_CMD                         (5u)

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef alt_mode_hw_t
  @brief Application HW type values which are used in alt_mode_hw_evt_t structure.
 */
typedef enum
{
    ALT_MODE_MUX = 1,                    /**< HW type - MUX. */
    ALT_MODE_HPD,                        /**< HW type - HPD transceiver/receiver. */

}alt_mode_hw_t;

/**
  @ typedef mux_select_t
  @ brief Possible settings for the Type-C Data MUX.
  @ note This type should be extended to cover all possible modes for the MUX.
 */
typedef enum
{
    MUX_CONFIG_ISOLATE,                  /**< Isolate configuration. */
    MUX_CONFIG_2_0,                      /**< USB 2.0 configuration. */
    MUX_CONFIG_SS_ONLY,                  /**< USB SS configuration. */
    MUX_CONFIG_DP_2_LANE,                /**< Two lane DP configuration. */
    MUX_CONFIG_DP_4_LANE,                /**< Four lane DP configuration. */
    MUX_CONFIG_RIDGE_CUSTOM,             /**< Alpine/Titan Ridge custom configuration. */
    MUX_CONFIG_INIT,                     /**< Enables MUX functionality. */
    MUX_CONFIG_DEINIT,                   /**< Disables MUX functionality. */

} mux_select_t;

/**
  @ typedef mux_status_t
  @ brief Possible states for the MUX handler.
 */
typedef enum
{
    MUX_STATE_IDLE,                      /**< MUX idle state. */
    MUX_STATE_FAIL,                      /**< MUX switch failed. */
	MUX_STATE_BUSY,                      /**< MUX is busy. */
    MUX_STATE_SUCCESS                    /**< MUX switched successfully. */
} mux_poll_status_t;

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
  @union alt_mode_hw_evt_t
  @brief Alt mode HW application event/command structure.
 */
typedef union
{

    uint32_t val;                       /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing alternate modes HW event/command . */
    struct ALT_MODE_HW_EVT
    {
        uint32_t evt_data    : 16;      /**< Current event/command data. */
        uint32_t hw_type     : 8;       /**< HW type event/command related to. */
        uint32_t data_role   : 8;       /**< Current data role. */
    }hw_evt;                            /**< Union containing the application HW event/command value. */

}alt_mode_hw_evt_t;

/**
  @typedef alt_mode_hw_cmd_cbk_t
  @brief Callback type used for notifications about ALT. MODE Hardware Commands.
 */
typedef void (*alt_mode_hw_cmd_cbk_t) (uint8_t port, uint32_t command);

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function registers an ALT. MODE hardware callback function.
 *
 * @param port Port for which the callback is registered.
 * @param cbk Callback function for ALT. MODE hardware events.
 *
 * @return None.
 */
void alt_mode_hw_set_cbk(uint8_t port, alt_mode_hw_cmd_cbk_t cbk);

/**
 * @brief This function deinits all HW related to alt modes.
 *
 * @param port Port index the function is performed for.
 *
 * @return None.
 */
void alt_mode_hw_deinit(uint8_t port);

/**
 * @brief This function evaluates received application HW command.
 *
 * @param port Port index the function is performed for.
 * @param cmd_param Pointer to received application HW command data.
 *
 * @return true if APP command passed successful, false if APP command is invalid
 * or contain unacceptable fields.
 */
bool eval_app_alt_hw_cmd(uint8_t port, uint8_t *cmd_param);

/**
 * @brief This function evaluates received HPD application command.
 *
 * @param port Port index the function is performed for.
 * @param cmd Pointer to received HPD application command data.
 *
 * @return true if HPD APP command passed successful, false if HPD APP command
 * is invalid or contain unacceptable fields.
 */
bool eval_hpd_cmd(uint8_t port, uint32_t cmd);

/**
 * @brief This function evaluates received MUX application command.
 *
 * @param port Port index the function is performed for.
 * @param cmd Pointer to received MUX application command data.
 *
 * @return true if MUX APP command passed successful, false if MUX APP command
 * is invalid or contain unacceptable fields.
 */
bool eval_mux_cmd(uint8_t port, uint32_t cmd);

/**
 * @brief This function sex appropriate MUX configuration based on the function parameters.
 *
 * @param port Port index the function is performed for.
 * @param cfg Required MUX configuration.
 * @param custom_data Additional data in case of custom AR MUX configuration.
 *
 * @return true if MUX set passed successful, false if MUX setting is invalid or
 * contain unacceptable fields.
 */
bool set_mux(uint8_t port, mux_select_t cfg, uint32_t custom_data);

/**
 * @brief Get the current state of the MUX.
 * @param port PD port index.
 * @return Active MUX setting.
 */
mux_select_t get_mux_state(uint8_t port);

/**
 * @brief Check whether the ALT. MODE hardware block is idle.
 *
 * This function is part of the deep-sleep entry checks for the CCG device, and checks whether
 * there are any pending ALT. MODE hardware commands that require the device to be active.
 *
 * @param port Port on which the hardware status is to be checked.
 * @return true if the block is idle, false otherwise.
 */
bool alt_mode_hw_is_idle(uint8_t port);

/**
 * @brief Prepare the Alt. Mode hardware state for device deep-sleep.
 * @param port The port whose hardware state is to be configured.
 * @return None.
 */
void alt_mode_hw_sleep(uint8_t port);

/**
 * @brief Restore Alt. Mode hardware state after device resumes from deep-sleep.
 * @param port The port whose hardware state is to be restored.
 * @return None
 */
void alt_mode_hw_wakeup(uint8_t port);

/**
 * @brief Return HPD state based on HPD Queue events.
 * @param port DP port index
 * @return true if HPD is connected, false otherwise
 */
bool dp_snk_get_hpd_state(uint8_t port);

#endif /* _ALT_MODE_HW_H_ */

/* [] END OF FILE */
