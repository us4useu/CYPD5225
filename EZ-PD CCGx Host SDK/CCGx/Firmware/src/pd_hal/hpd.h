/**
 * @file hpd.h
 *
 * @brief @{Hotplug Detect (HPD) driver header file.@}
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

#ifndef _HPD_H_
#define _HPD_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <ccgx_regs.h>
#include <status.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
 * @brief HPD Input direction is used when sensing HPD output
 * from a monitor. This is used in applications like video dongles and
 * monitors.
 */
#define HPD_IP_DIRECTION                        (0)

/**
 * @brief HPD Output direction is used when driving HPD output to a DP source.
 * This is used in applications like PD port controllers in PC/Mobile platforms.
 */
#define HPD_OP_DIRECTION                        (PDSS_CTRL_HPD_DIRECTION)

/**
 * @brief HPD HW register event mask.
 */
#define HPD_EVENT_MASK                          (3u)
/**
 * @brief HPD Event 0 bit position
 */
#define HPD_EVENT_0_POS                         (0)
/**
 * @brief HPD Event 1 bit position
 */
#define HPD_EVENT_1_POS                         (2)
/**
 * @brief HPD Event 2 bit position
 */
#define HPD_EVENT_2_POS                         (4)
/**
 * @brief HPD Event 3 bit position
 */
#define HPD_EVENT_3_POS                         (6)
/**
 * @brief Get the first HPD event from queue status.
 */
#define HPD_GET_EVENT_0(hpd_queue)              (((hpd_queue) >> HPD_EVENT_0_POS) & HPD_EVENT_MASK)
/**
 * @brief Get the second HPD event from queue status.
 */
#define HPD_GET_EVENT_1(hpd_queue)              (((hpd_queue) >> HPD_EVENT_1_POS) & HPD_EVENT_MASK)
/**
 * @brief Get the third HPD event from queue status.
 */
#define HPD_GET_EVENT_2(hpd_queue)              (((hpd_queue) >> HPD_EVENT_2_POS) & HPD_EVENT_MASK)
/**
 * @brief Get the fourth HPD event from queue status.
 */
#define HPD_GET_EVENT_3(hpd_queue)              (((hpd_queue) >> HPD_EVENT_3_POS) & HPD_EVENT_MASK)

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
 * @typedef hpd_event_type_t
 * @brief List of HPD events detected by USBPD block. The UNPLUG, PLUG and
 * IRQ events are used in the case of a DisplayPort Sink implementation, and
 * the COMMAND_DONE event is used in the case of a DP Source implementation.
 */
typedef enum
{
    HPD_EVENT_NONE = 0,         /**< No event. */
    HPD_EVENT_UNPLUG,           /**< DP Unplug event. */
    HPD_EVENT_PLUG,             /**< DP Plug event. */
    HPD_EVENT_IRQ,              /**< DP IRQ event. */
    HPD_COMMAND_DONE            /**< Requested HPD command is complete. */
} hpd_event_type_t;

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @typedef hpd_event_cbk_t
 * @brief HPD event callback function. This function is used by the HAL
 * layer to notify the DisplayPort alternate mode layer about HPD related
 * events that are detected.
 */
typedef void (*hpd_event_cbk_t)(
        uint8_t port,           /**< Port on which HPD event occurred. */
        hpd_event_type_t event  /**< Type of HPD event. */
        );

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Enable the HPD-Receive functionality for the specified PD port.
 * This function shall be used in DP Sink designs and should not be combined
 * with the hpd_transmit_init call.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @param cbk callback function to be called when there is an HPD event.
 * @return  Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t hpd_receive_init(uint8_t port, hpd_event_cbk_t cbk);

/**
 * @brief Enable the HPD-Transmit functionality for the specified PD port.
 * This function shall be used in DP source designs and should not be combined
 * with the hpd_receive_init call.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @param cbk callback to be used for command completion event.
 * @return Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t hpd_transmit_init(uint8_t port, hpd_event_cbk_t cbk);

/**
 * @brief Disable the HPD functionality on the specified PD port. This
 * is used for HPD receive as well transmit de-initialization.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @return Return CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t hpd_deinit(uint8_t port);

/**
 * @brief Returns the current status of the HPD input signal. This function
 * can only be used if the HPD receive block has been enabled using the
 * hpd_receive_init function.
 *
 * @param port pd port index.
 * @return Current HPD input signal status.
 */
bool hpd_receive_get_status(uint8_t port);

/**
 * @brief Send the desired HPD event out through the HPD GPIO. Only
 * the HPD_EVENT_UNPLUG, HPD_EVENT_UNPLUG and HPD_EVENT_IRQ events
 * should be requested.
 *
 * @param port Port on which HPD event is to be sent.
 * @param evtype Type of HPD event to be sent.
 * @param wait Whether function should wait until command is complete.
 * @return Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t hpd_transmit_sendevt(uint8_t port, hpd_event_type_t evtype, bool wait);

/**
 * @brief Prepare the HPD transmit block for deep-sleep. This function changes
 * the IO mode of the HPD signal to GPIO, so as to avoid glitches on the signal
 * during the active - sleep - active power mode transitions of the CCG device.
 *
 * @param port Port whose HPD output is to be updated.
 * return None
 */
void hpd_sleep_entry(uint8_t port);

/**
 * @brief Restore the HPD Tx signal driver after CCG wakes from deep-sleep.
 *
 * @param port Port whose HPD signal is to be restored.
 * @param value Startup value for the HPD signal. The APP layer is expected
 * to identify the desired (PLUG/UNPLUG) state of the signal and pass it to
 * the HAL.
 * @return None
 */
void hpd_wakeup(uint8_t port, bool value);

/**
 * @brief Prepare the HPD RX block for deep-sleep.
 *
 * This routine implements a firmware workaround to retain HPD signal state before
 * entering deep sleep. All details of the workaround are captured in function
 * definition.
 *
 * @param port Port whose HPD RX is active. Caller should ensure to provide only valid values.
 * @param hpd_state Connection status of HPD
 * return None
 */
void hpd_rx_sleep_entry(uint8_t port, bool hpd_state);

/**
 * @brief Restore the HPD RX block function after CCG wakes up from deep sleep.
 * Implements the wakeup portion of the work-around to retail HPD signal state.
 *
 * @param port Port whose HPD RX module settings have to be restored.
 * @return None
 */
void hpd_rx_wakeup(uint8_t port);

/**
 * @brief Returns the state of HPD RX activity timer. This check should be
 * used to prevent device entry into a low power sleep state while an HPD
 * transition is in progress.
 *
 * @param port Port whose HPD RX state is to be checked.
 * @return Status of timer. true if timer is active, false otherwise.
 */
bool is_hpd_rx_state_idle(uint8_t port);

#endif /* _HPD_H_ */

/* [] END OF FILE */
