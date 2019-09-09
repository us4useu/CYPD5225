/**
 * @file timer.h
 *
 * @brief @{Soft timer header file.@}
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

#ifndef TIMER_H_
#define TIMER_H_

#include "config.h"
#include "stdbool.h"

/*****************************************************************************
 * MACRO Definition
 *****************************************************************************/

/**
   @brief Number of soft timers supported per instance of the timer module. The
   number of instances depends on the number of USB-PD ports supported by
   the device.
 */
#define TIMER_NUM_TIMERS                (31u)

/**
   @brief Maximum timeout value in milliseconds supported by the module.
 */
#define TIMER_MAX_TIMEOUT               (0xFFFF)

/**
   @brief Invalid timer ID value.
 */
#define TIMER_INVALID_ID                (0xFFu)

/**
   @brief Invalid timer instance.
 */
#define TIMER_INVALID_INDEX             (0xFFu)

/*****************************************************************************
 * Data Type Definition
 *****************************************************************************/

/**
 * @typedef timer_id_t
 * @brief Timer ID type definition.
 *
 * This type definition is used to identify software timer objects. The timer ID
 * needs to be unique for each soft timer instance and need to be maintained in
 * the application. To maintain uniqueness, the following timer ID allocation
 * rule is expected to be followed.
 *
 * PD and Type-C stack (pd.h)           :   0 -  29 : 30 timers
 * Base application stack (app.h)       :  30 - 127 : 98 timers
 * HPI module (hpi.h)                   : 128 - 135 :  8 timers
 * I2C module (i2c.h)                   : 136 - 143 :  8 timers
 * USB module (usb.h)                   : 144 - 151 :  8 timers
 * IECS module (iecs.h)                 : 152 - 159 :  8 timers
 * Reserved                             : 160 - 191 : 32 timers
 * Solution (project directory)         : 192 - 254 : 63 timers
 */
typedef uint8_t timer_id_t;

/**
 * @typedef timer_cb_t
 * @brief Timer callback function.
 *
 * This callback function is invoked on timer expiry and
 * should be treated as interrupt.
 */
typedef void (*timer_cb_t)(
        uint8_t instance,       /**< Timer module instance number. */
        timer_id_t id           /**< Timer ID for which callback is being generated. */
        );

/**
 * @brief Structure encapsulating information relating to a software timer.
 *
 * This structure encapsulates all information related to a software timer
 * object. The timer module maintains a list of these objects corresponding to
 * the active software timers at any given point of time.
 */
typedef struct
{
    uint32_t count;             /**< The actual count value. */
    timer_cb_t cb;              /**< The callback function pointer for the timer object. */
    uint16_t period;            /**< Period of operation for the timer instance. */
    timer_id_t id;              /**< The ID handle associated with the timer object. */
} ccg_timer_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Initialize the software timer module.
 *
 * This function initializes the software timer module. This function
 * initializes the data structures for timer management and enables the hardware
 * timer used for the soft timer implementation.
 *
 * @return None
 */
void timer_init(void);

/**
 * @brief Start a soft timer
 *
 * Start a specific soft timer. All soft timers are one-shot timers which will
 * run until the specified period has elapsed. The timer expiration callback
 * will be called at the end of the period, if one is provided.
 *
 * @param instance Timer instance number for which we are using the timer.
 * @param id Unique timer id
 * @param period Timer period in milliseconds.
 * @param cb Timer expiration callback. Can be NULL.
 *
 * @return true if the timer is started, false if timer start fails.
 */
bool timer_start(uint8_t instance, timer_id_t id, uint16_t period, timer_cb_t cb);

/**
 * @brief Start a soft timer
 *
 * Start a specific soft timer. All soft timers are one-shot timers which will
 * run until the specified period has elapsed.
 *
 * @param instance Timer instance number for which we are using the timer.
 * @param id Unique timer id
 * @param period Timer period in milliseconds.
 *
 * @return true if the timer is started, false if timer start fails.
 */
bool timer_start_wocb(uint8_t instance, timer_id_t id, uint16_t period);

/**
 * @brief Stop a soft timer.
 *
 * Stop a soft timer which is currently running.
 *
 * @param instance Instance number for which we are using the timer.
 * @param id Unique timer id.
 *
 * @return None
 */
void timer_stop(uint8_t instance, timer_id_t id);

/**
 * @brief Check whether the specified soft timer is currently running.
 *
 * @param instance Instance number for which we are using the timer.
 * @param id Unique timer id.
 *
 * @return true if the timer is running, false otherwise.
 */
bool timer_is_running (uint8_t instance, timer_id_t id);

/**
 * @brief Returns the time remaining for timer expiration.
 *
 * Returns the time remaining for timer expiration.
 *
 * @param instance Instance number for which we are using the timer.
 * @param id Unique timer id.
 *
 * @return Time remaining for expiration of the soft timer.
 */
uint16_t timer_get_count (uint8_t instance, timer_id_t id);

/**
 * @brief Stops all soft timers associated with the instance.
 *
 * @param instance Instance number on which timers are to be stopped.
 *
 * @return None
 */
void timer_stop_all(uint8_t instance);

/**
 * @brief This function stops all soft timers with ids in the specified range.
 *
 * @param instance Instance number on which soft timers are to be stopped.
 * @param start Starting timer ID. The value is inclusive.
 * @param stop Ending timer ID. The value is inclusive.
 *
 * @return None
 */
void timer_stop_range(uint8_t instance, timer_id_t start, timer_id_t stop);

/**
 * @brief Returns number of active timers.
 *
 * @param instance Instance number to query.
 *
 * @return Number of active timers on this instance.
 */
uint8_t timer_num_active(uint8_t instance);

/**
 * @brief Prepare the timer hardware for entering deep sleep.
 *
 * This function prepares the timer module and the hardware timer for entering
 * device deep sleep. This must be called prior to entering deep sleep mode.
 *
 * @return None
 */
void timer_enter_sleep(void);

/**
 * @brief Get the number of LF clock ticks required per ms.
 * @return Returns the number of LF clock ticks per ms.
 */
uint16_t timer_get_multiplier(void);

#endif /* TIMER_H_ */

/* EOF */
