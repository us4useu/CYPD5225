/**
 * @file instrumentation.c
 *
 * @brief @{Source file containing application level instrumentation code.
 *
 * This USB-PD port controller application supports high level instrumentation
 * to track the task execution latencies and runtime stack usage. This source
 * file contains the code associated with these functions.@}
 */

/*
 * Copyright (2016-2018), Cypress Semiconductor Corporation or a subsidiary of
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

#include <project.h>
#include <flash_config.h>
#include <system.h>
#include <app.h>
#include <timer.h>
#include <hpi.h>
#include <boot.h>
#include <flash.h>
#include <status.h>
#include <utils.h>
#include <gpio.h>

/* Run-time stack lower limit defined in linker script. */
extern int __cy_stack_limit;

#if RESET_ON_ERROR_ENABLE

/* RAM based signature and offset used to check whether reset data is valid. */
#define RESET_DATA_VALID_OFFSET         (0)
#define RESET_DATA_VALID_SIG            (0xDEADBEEF)

/* RAM based signature and offset used to check whether watchdog reset has happened. */
#define WATCHDOG_RESET_OFFSET           (1)
#define WATCHDOG_RESET_SIG              (0xC003300C)

/* RAM offset where the watchdog reset count is maintained. */
#define RESET_COUNT_OFFSET              (2)

/* Size of the reset tracking data structure in DWORDs. */
#define RESET_DATA_STRUCT_SIZE          (3)

/* Address of the run-time instrumentation data structure. */
         uint32_t *gRunTimeDataAddr = (uint32_t *)CYDEV_SRAM_BASE;

/* Variable used to identify whether main loop has been run. */
volatile uint32_t gMainLoopDelay = 0;

/* Margin (in ms) available until watchdog reset. */
volatile uint16_t gMinResetMargin = WATCHDOG_RESET_PERIOD_MS;

/* Timer callback to reset device if main loop has not been run as expected. */
void watchdog_timer_cb (
    uint8_t port,
    timer_id_t id)
{
    /*
     * It is possible that this timer is the only reason for the device to wake from sleep.
     * Hence allow two consecutive timer expiries before resetting the device.
     */
    gMainLoopDelay++;
    if (gMainLoopDelay >= 2)
    {
        /* Store the reset signature into RAM. */
        gRunTimeDataAddr[WATCHDOG_RESET_OFFSET] = WATCHDOG_RESET_SIG;
        CySoftwareReset ();
    }

    /* Start the timer again. */
    timer_start (0, WATCHDOG_TIMER_ID, WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
}

#endif /* RESET_ON_ERROR_ENABLE */

#if STACK_USAGE_CHECK_ENABLE
    
/*
 * Minimum run-time stack usage value. This many bytes at the top of the stack
 * will not be tracked for usage.
 */
#define MIN_STACK_USAGE     (128u)

/* Signature used to track stack usage. */
#define STACK_UNUSED_SIG    (0x00555500)

/* Address of the bottom location of the run-time stack. */
         uint32_t *gStackBottom  = (uint32_t *)CYDEV_SRAM_BASE;
volatile uint16_t gMinStackMargin   = 0;

#endif /* STACK_USAGE_CHECK_ENABLE */

void instrumentation_init(void)
{
    uint32_t wdr_cnt = 0;
    (void)wdr_cnt;

#if STACK_USAGE_CHECK_ENABLE
    uint32_t *addr_p;

    /* Store the stack bottom location. */
    gStackBottom = (uint32_t *)&__cy_stack_limit;
#if RESET_ON_ERROR_ENABLE
    /* If we have watchdog reset tracking enabled, the lowest twelve bytes of stack cannot be used. */
    gStackBottom += RESET_DATA_STRUCT_SIZE;
#endif /* RESET_ON_ERROR_ENABLE */

    /* Fill the stack memory with unused signature. */
    for (addr_p = gStackBottom; addr_p < (uint32_t *)((CYDEV_SRAM_BASE + CYDEV_SRAM_SIZE) - MIN_STACK_USAGE); addr_p++)
    {
        *addr_p = STACK_UNUSED_SIG;
    }
    
    /* Initialize the stack margin value. */
    gMinStackMargin = (uint16_t)((uint32_t)addr_p - (uint32_t)gStackBottom);

#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_37 (gMinStackMargin);
#endif /* CCG_HPI_ENABLE */

#endif /* STACK_USAGE_CHECK_ENABLE */

#if RESET_ON_ERROR_ENABLE
    gRunTimeDataAddr = (uint32_t *)&__cy_stack_limit;
    if (gRunTimeDataAddr[RESET_DATA_VALID_OFFSET] == RESET_DATA_VALID_SIG)
    {
        wdr_cnt = gRunTimeDataAddr[RESET_COUNT_OFFSET];
        if (gRunTimeDataAddr[WATCHDOG_RESET_OFFSET] == WATCHDOG_RESET_SIG)
            wdr_cnt++;
    }

    /*
     * Store the reset data valid signature and current reset count.
     * Also clear the reset detected signature.
     */
    gRunTimeDataAddr[RESET_DATA_VALID_OFFSET] = RESET_DATA_VALID_SIG;
    gRunTimeDataAddr[WATCHDOG_RESET_OFFSET]   = 0;
    gRunTimeDataAddr[RESET_COUNT_OFFSET]      = wdr_cnt;
    
    gMinResetMargin = 2 * WATCHDOG_RESET_PERIOD_MS;

#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_35 (gMinResetMargin);
#endif /* CCG_HPI_ENABLE */

#endif /* RESET_ON_ERROR_ENABLE */

#if ((CCG_HPI_ENABLE) && (HPI_WATCHDOG_RESET_ENABLE))
    /* Store the reset count into the HPI register. */
    if (wdr_cnt > 0xFFu)
        wdr_cnt = 0xFFu;
    hpi_set_reset_count (wdr_cnt);
#endif /* CCG_HPI_ENABLE */
}

void instrumentation_start(void)
{
#if RESET_ON_ERROR_ENABLE
    /* Start the timer used for watchdog reset. */
    timer_start (0, WATCHDOG_TIMER_ID, WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
#endif /* RESET_ON_ERROR_ENABLE */
}

void instrumentation_task(void)
{
#if STACK_USAGE_CHECK_ENABLE
    uint32_t *addr_p = gStackBottom;
#endif /* STACK_USAGE_CHECK_ENABLE */

#if RESET_ON_ERROR_ENABLE

    /* Clear the variable to indicate main loop has been run. */
    gMainLoopDelay = 0;

#if CALCULATE_TIMER_MARGINS
    /* Calculate how much time is left before device could get reset. */
    gMinResetMargin = GET_MIN(gMinResetMargin,
            ((1 - gMainLoopDelay) * WATCHDOG_RESET_PERIOD_MS) + timer_get_count(0, WATCHDOG_TIMER_ID));
#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_35 (gMinResetMargin);
#endif /* CCG_HPI_ENABLE */

    /* Stop and restart the timer. */
    timer_stop (0, WATCHDOG_TIMER_ID);
    timer_start (0, WATCHDOG_TIMER_ID, WATCHDOG_RESET_PERIOD_MS, watchdog_timer_cb);
#endif /* CALCULATE_TIMER_MARGINS */

#endif /* RESET_ON_ERROR_ENABLE */

#if STACK_USAGE_CHECK_ENABLE
    for (addr_p = gStackBottom; addr_p < (uint32_t *)((CYDEV_SRAM_BASE + CYDEV_SRAM_SIZE) - MIN_STACK_USAGE); addr_p++)
    {
        if (*addr_p != STACK_UNUSED_SIG)
        {
            break;
        }
    }
    
    /* Calculate the minimum stack availability margin and update debug register. */
    gMinStackMargin = GET_MIN(gMinStackMargin, ((uint32_t)addr_p - (uint32_t)gStackBottom));
#if CCG_HPI_ENABLE
    hpi_set_reserved_reg_37(gMinStackMargin);
#endif /* CCG_HPI_ENABLE */

#endif /* STACK_USAGE_CHECK_ENABLE */
}

void CyBoot_IntDefaultHandler_Exception_EntryCallback(void)
{
#if RESET_ON_ERROR_ENABLE
    /* Store the reset signature into RAM. */
    gRunTimeDataAddr[WATCHDOG_RESET_OFFSET] = WATCHDOG_RESET_SIG;
    CySoftwareReset ();
#endif /* RESET_ON_ERROR_ENABLE */
}

/* End of file */
