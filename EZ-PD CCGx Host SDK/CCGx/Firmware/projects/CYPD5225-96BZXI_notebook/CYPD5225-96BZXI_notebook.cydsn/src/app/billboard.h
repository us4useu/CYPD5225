/**
 * @file billboard.h
 *
 * @brief @{Billboard control interface header file.@}
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

/* This file lists all the APIs / data structures
 * for the USB device block. */

#ifndef _BILLBOARD_H_
#define _BILLBOARD_H_

#include "config.h"
#include "stdint.h"
#include "stdbool.h"
#include "status.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/**
 * @brief Maximum number of alternate modes supported by the billboard module.
 */
#define BB_MAX_ALT_MODES                (8)

/**
 * @brief Initialisation value for mode status register.
*/
#define BB_ALT_MODE_STATUS_INIT_VAL     (0x5555)

/**
 * @brief Alternate mode status mask for each mode.
 */
#define BB_ALT_MODE_STATUS_MASK         (0x03)

/**
 * @brief Billboard OFF timer maximum interval. One second is used to ensure
 * that timer module goes through only one interrupt per request.
 */
#define BB_OFF_TIMER_MAX_INTERVAL       (1000)

/**
 * @brief Timeout special value for not disabling the billboard interface.
 */
#define BB_OFF_TIMER_NO_DISABLE         (0xFFFF)

/**
 * @brief Minimum timeout value in allowed in seconds = 60s.
 */
#define BB_OFF_TIMER_MIN_VALUE          (60)

/**
 * @brief Maximum buffering for EP0 transactions inside the billboard module.
 *
 * This definition is valid only for internal billboard implementation.
 */
#define BB_MAX_EP0_XFER_SIZE            (256)

/**
 * @brief Additional failure information due to lack of power as per the
 * billboard capability descriptor.
 */
#define BB_CAP_ADD_FAILURE_INFO_PWR     (0x01)

/**
 * @brief Additional failure information due to USBPD communication failure
 * as per billboard capability descriptor.
 */
#define BB_CAP_ADD_FAILURE_INFO_PD      (0x02)

/**
 * @brief Disconnect BB Status for BB connect event
 */
#define BB_DISCONNECT_STAT              (0xFF)

/**
 * @brief Connect BB Status for BB connect event
 */
#define BB_CONNECT_STAT                 (0x01)

/**
 * @brief Maximum number of alternate modes supported.
 */
#define BB_MAX_SVID                     (2u)

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/

/**
 * @brief Billboard implementation model.
 */
typedef enum
{
    BB_TYPE_NONE,               /**< No billboard device */
    BB_TYPE_EXTERNAL,           /**< External billboard device */
    BB_TYPE_INTERNAL,           /**< Device supports internal USB module to implement billboard */
    BB_TYPE_EXT_CONFIGURABLE    /**< External billboard device which supports configuration through PD controller. */
} bb_type_t;

/**
 * @brief Billboard module states.
 */
typedef enum
{
    BB_STATE_DEINITED,          /**< Module is not initialized */
    BB_STATE_DISABLED,          /**< Module is initialized but not enabled */
    BB_STATE_BILLBOARD,         /**< Module is active with billboard enumeration */
    BB_STATE_LOCKED,            /**< Module is active with additional functionality */
    BB_STATE_FLASHING,          /**< Module is active with flashing mode enumeration */

} bb_state_t;

/**
 * @brief USB Billboard cause for enumeration.
 *
 * The enumeration lists all the supported causes for billboard enumeration.
 */
typedef enum
{
    BB_CAUSE_AME_TIMEOUT,       /**< AME timeout event */
    BB_CAUSE_AME_SUCCESS,       /**< Successful alternate mode entry */
    BB_CAUSE_AME_FAILURE,       /**< Failed alternate mode entry */
    BB_CAUSE_PWR_FAILURE        /**< Failed to get sufficient power */

} bb_cause_t;

/**
 * @brief USB Billboard alternate mode status.
 *
 * The enumeration lists the different status values as defined by
 * the billboard class specification.
 */
typedef enum
{
    BB_ALT_MODE_STAT_ERROR,             /**< Undefined error / alternate mode does not exist */
    BB_ALT_MODE_STAT_NOT_ATTEMPTED,     /**< Alternate mode entry not attempted */
    BB_ALT_MODE_STAT_UNSUCCESSFUL,      /**< Alternate mode entry attempted but failed */
    BB_ALT_MODE_STAT_SUCCESSFUL         /**< Alternate mode entry succeeded */

} bb_alt_mode_status_t;

/**
 * @brief USB Billboard string descriptor indices.
 *
 * The enumeration lists the different string indices used in the billboard
 * implementation.
 */
typedef enum
{
    BB_LANG_ID_STRING_INDEX,            /**< Language ID index */
    BB_MFG_STRING_INDEX,                /**< Manufacturer string index */
    BB_PROD_STRING_INDEX,               /**< Product string index */
    BB_SERIAL_STRING_INDEX,             /**< Serial string index */
    BB_CONFIG_STRING_INDEX,             /**< Configuration string index */
    BB_BB_INF_STRING_INDEX,             /**< Billboard interface string index */
    BB_HID_INF_STRING_INDEX,            /**< HID interface string index */
    BB_URL_STRING_INDEX,                /**< Additional info URL string index */
    BB_ALT_MODE1_STRING_INDEX,          /**< Alternate mode 1 string index */
    BB_ALT_MODE2_STRING_INDEX,          /**< Alternate mode 2 string index */
    BB_ALT_MODE3_STRING_INDEX,          /**< Alternate mode 3 string index */
    BB_ALT_MODE4_STRING_INDEX,          /**< Alternate mode 4 string index */
    BB_ALT_MODE5_STRING_INDEX,          /**< Alternate mode 5 string index */
    BB_ALT_MODE6_STRING_INDEX,          /**< Alternate mode 6 string index */
    BB_ALT_MODE7_STRING_INDEX,          /**< Alternate mode 7 string index */
    BB_ALT_MODE8_STRING_INDEX           /**< Alternate mode 8 string index */

} bb_usb_string_index_t;

/**
 * @brief Internal billboard module handle structure.
 *
 * No explicit structure for the handle is expected to be created outside of
 * the billboard module implementation.
 */
typedef struct bb_handle
{
    /* Common billboard fields. */
    bb_type_t type;                     /**< Billboard implementation type */
    bb_state_t state;                   /**< Billboard current state */
    uint8_t num_alt_modes;              /**< Number of valid alternate modes */
    uint8_t bb_add_info;                /**< AdditionalFailureInfo field in billboard
                                             capability descriptor. */
    uint32_t alt_status;                /**< Current alternate mode status -
                                             The status can hold a maximum of 16
                                             alternate modes (2bits each). */
    uint32_t timeout;                   /**< Pending timeout count in ms for billboard
                                             interface disable. */

    /* Internal billboard fields. */
    uint8_t *ep0_buffer;                /**< EP0 data buffer pointer in case of
                                             internal billboard implementation. */
    bool usb_configured;                /**< Internal flag indicating whether the
                                             configuration is enabled or not. This is valid
                                             only for internal billboard implementation. */
    uint8_t usb_port;                   /**< USB port index used in case the device
                                             has multiple USB ports. */
    bool flashing_mode;                 /**< USB flashing mode state. Valid only for
                                             internal billboard implementation. */
    bool usb_i2cm_mode;                 /**< USB I2C master bridge mode state. Valid only
                                             for internal billboard implementation
                                             supporting I2C master bridge interface. */
    bool queue_enable;                  /**< Billboard interface enable request
                                             queued. Valid only for internal
                                             billboard implementation. */
    bool queue_disable;                 /**< Billboard interface disable request
                                             queued. Valid only for internal
                                             billboard implementation. */
    bool queue_i2cm_enable;             /**< USB-I2C master mode enable request.
                                             Valid only for internal billboard
                                             implementation. */

} bb_handle_t;

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief Initialize the billboard module.
 *
 * The API initializes the billboard module. This is mainly a software state
 * machine initialization. The USB device is not enabled at this point. The API
 * helps to cleanup previous state information.
 *
 * @param port Port index of port to initialize. Caller is expected to
 *      ensure that only valid port index is provided.
 *
 * @return Status of the call.
 */
ccg_status_t bb_init(uint8_t port);

/**
 * @brief The function queues a billboard enumeration / re-enumeration.
 *
 * The API queues the billboard enumeration as per the configuration information
 * provided. Enumeration details are retreived from the configuration table.
 *
 * The function can be called multiple times applications to trigger a
 * re-enumeration without explicit disable call.
 *
 * For internal implementation of billboard, the USB module is controlled from
 * the bb_task() and this function only queues the request. It should be noted
 * that only one pending request is honoured. If more than one request is
 * queued, only the latest is handled. This request is failed if the flashing
 * mode is in progress.
 *
 * @param port Index of port. Caller is expected to ensure that
 *      only valid port index is provided.
 * @param cause Cause for billboard enumeration.
 *
 * @return Status of the call.
 */
ccg_status_t bb_enable(uint8_t port, bb_cause_t cause);

/**
 * @brief The function queues a billboard interface disable.
 *
 * The API disables the billboard device and disconnects the terminations.
 * For internal implementation of billboard, the USB module is controlled
 * from the bb_task() and this function only queues the request. It should be
 * noted that only one pending request is honoured. If more than one request is
 * queued, only the latest is handled. A disable call clears any pending enable.
 *
 * @param port Index of port. Caller is expected to ensure that
 *      only valid port index is provided.
 * @param force Whether to force a disable. false = Interface not disabled when
 *      in flashing mode. true = Interface disabled regardless of the operation mode.
 *
 * @return Status of the call.
 */
ccg_status_t bb_disable(uint8_t port, bool force);

/**
 * @brief The function updates the alternate mode status.
 *
 * The function updates the alternate mode status information for the specified
 * alternate mode index. The altnerate mode index should match the DISCOVER_SVID
 * and DISCOVER_MODES response for the device.
 *
 * @param port Index of port. Caller is expected to ensure that only valid
 *      port index is provided.
 * @param mode_index Index of the mode as defined by the alternate mode manager.
 * @param alt_status Current altenate mode status.
 *
 * @return Status of the call.
 */
ccg_status_t bb_update_alt_status(uint8_t port, uint8_t mode_index,
        bb_alt_mode_status_t alt_status);

/**
 * @brief The function updates the alternate mode status for all modes.
 *
 * The current billboard implementation supports a maximum of 8 alternate modes
 * and the each modes as defined in the order of BOS descriptor has two bit
 * status. Bit 1:0 indicates status of alt_mode0, Bit 3:2 indicates status of
 * alt_mode1 etc. This function should be only used when the billboard status
 * needs to be re-initialized to a specific value. In individual entry / exit
 * cases, bb_update_alt_status() should be used.
 *
 * @param port Index of port. Caller is expected to ensure that only valid
 *      port index is provided.
 * @param status Status data for all alternate modes.
 *
 * @return Status of the call.
 */
ccg_status_t bb_update_all_status(uint8_t port, uint32_t status);

/**
 * @brief Checks whether a billboard interface is present.
 *
 * The function checks the configuration information and identifies if a billboard
 * device exists.
 *
 * @param port Index of port. Caller is expected to ensure that only valid
 *      port index is provided.
 *
 * @return Returns true if billboard is present and false if absent.
 */
bool bb_is_present(uint8_t port);

/**
 * @brief Billboard module task function.
 *
 * The function implements the billboard state machine and needs to be invoked
 * in the main task loop. The task handler allows deferring interrupts and
 * improves interrupt latency of the system.
 *
 * @param port Index of port. Caller is expected to ensure that
 *      only valid port index is provided.
 *
 * @return None
 */
void bb_task(uint8_t port);

/**
 * @brief Function requests for flashing mode operation.
 *
 * The function enables / disables the billboard interface in USB flashing mode.
 * This function. This should be invoked only if the solution requires to start
 * the flashing mode. This request shall prevent the billboard expiry on enable
 * and re-start the billboard expiry on disable as required.
 *
 * @param port Index of port. Caller is expected to ensure that only valid
 *      port index is provided.
 * @param enable Enable / disable control. true = Enable, false = Disable.
 *
 * @return Status of the call.
 */
ccg_status_t bb_flashing_ctrl(uint8_t port, bool enable);

/**
 * @brief Function returns whether the billboard module is idle or not.
 *
 * This function indicates whether there are any pending tasks or not. This
 * function can be invoked before device enters sleep mode to check if it is
 * allowed. But idle condition does not allow deep sleep entry. For this,
 * the application is expected to use the bb_enter_deep_sleep() function.
 *
 * @param port Index of the port. Caller is expected to ensure that only valid
 *      port index is provided.
 *
 * @return true = Billboard module is idle, false = billboard module is busy.
 */
bool bb_is_idle(uint8_t port);

/**
 * @brief Function puts the module into deep sleep.
 *
 * The function first checks if the deep sleep mode can be supported at this
 * time and enables the module to wakeup from deep sleep. In this case, the
 * wakeup source is USB. Deep sleep is allowed only when the USB bus is in
 * suspend state. So it is better to check for the USB state first before
 * this call. Failure on this call does not require any special action to be
 * taken by the caller other than not to enter the deep sleep mode until a
 * subsequent call passes successfully.
 *
 * @param port Index of the port. Caller is expected to ensure that only valid
 *      port index is provided.
 *
 * @return true = Successful deep sleep entry, false = Failed deep sleep entry.
 */
bool bb_enter_deep_sleep(uint8_t port);

/**
 * @brief Function controls the bridge mode internal to the billboard module.
 *
 * The function is valid only for the internal billboard implementation supporting
 * the bridge mode extension. The function should only be called from the
 * solution module from inside the usb_i2cm_bridge_ctrl() function. This call
 * causes re-enumeration.
 *
 * @param port Index of the port. Caller is expected to ensure that only valid
 *      port index is provided.
 *
 */
ccg_status_t bb_bridge_ctrl(uint8_t port, bool enable);

/**
 * @brief Function that returns the Billboard firmware version information.
 *
 * @return Pointer to buffer containing billboard firmware version.
 */
uint8_t *bb_get_version();

#endif /* _BILLBOARD_H_ */

/*[]*/

