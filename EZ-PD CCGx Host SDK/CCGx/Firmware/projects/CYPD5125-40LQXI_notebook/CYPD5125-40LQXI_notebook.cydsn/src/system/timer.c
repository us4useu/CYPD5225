/**
 * @file timer.c
 *
 * @brief @{Soft timer source file.@}
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

#include "config.h"
#include "stdint.h"
#include "stdbool.h"
#include "ccgx_regs.h"
#include "timer.h"

#include "CyLib.h"

/*********************** Configuration checks ********************************/

#if ((TIMER_TYPE != TIMER_TYPE_WDT) && \
        (TIMER_TYPE != TIMER_TYPE_SYSTICK))
#error "Invalid TIMER_TYPE option. Only TIMER_TYPE_WDT and TIMER_TYPE_SYSTICK supported."
#endif /* (TIMER_TYPE) */

#if ((TIMER_TICKLESS_ENABLE != 0) && (TIMER_TYPE != TIMER_TYPE_WDT))
#error "Tickless timer supported only for WDT based implementation."
#endif /* ((TIMER_TICKLESS_ENABLE != 0) && (TIMER_TYPE != TIMER_TYPE_WDT)) */

/*****************************************************************************/

#if ((TIMER_TYPE == TIMER_TYPE_WDT))

/* WDT timer interrupt vector number */
#define WDT_RESET_DISABLE_KEY           ((uint32_t)(0xACED8865u))
#ifdef CCG2
#define WDT_INTERRUPT_VECTOR_NUMBER     (5u)
#elif (defined(CCG3)) || (defined(DMC))
#define WDT_INTERRUPT_VECTOR_NUMBER     (7u)
#elif (defined(CCG4))
#define WDT_INTERRUPT_VECTOR_NUMBER     (8u)
#elif (defined(CCG3PA))
#define WDT_INTERRUPT_VECTOR_NUMBER     (6u)
#elif (defined(CCG5))
#define WDT_INTERRUPT_VECTOR_NUMBER     (9u)
#elif (defined(CCG3PA2))
#define WDT_INTERRUPT_VECTOR_NUMBER     (6u)
#else /* Not supported */
#error "Device not supported by the timer module."
#endif /* Device check */

/*
 * Maximum timeout value allowed. This value comprehends the maximum interrupt
 * latencies so as not to overrun. Providing for a worst case number, 64 ticks
 * is used.
 */
#define TIMER_HW_MAX_TIMEOUT            (0xFFC0u)

/*
 * This time covers for the worst case interrupt latency
 * for the timer interrupt handler. Considering the worst
 * case timing of 100us
 */
#define TIMER_OVERRUN_THRESHOLD         (5u)

/* Timer counter size in number of ticks. */
#define TIMER_NUM_TICK_COUNTS           (0x10000u)

/* Multiplier for 1ms interval */
static uint16_t gl_multiplier;

#endif /* ((TIMER_TYPE == TIMER_TYPE_WDT)) */

#if (TIMER_TICKLESS_ENABLE != 0)
/* Tick time at timer start. */
static uint16_t volatile gl_start_tick;
/* Tick time from the start to match. */
static uint16_t volatile gl_tick_time;
/* Match register content. */
static uint16_t volatile gl_match;
#endif /* (TIMER_TICKLESS_ENABLE != 0) */

/* The HF_CLK frequency should match the actual hardware HF_CLK frequency. */
#define HF_CLK_FREQENCY                 (CYDEV_BCLK__HFCLK__HZ)
/* The HF counter theshold for 1ms timeout. Division is expected
 * to be optimized out by the compiler. */
#define HF_1MS_COUNT_VALUE              (HF_CLK_FREQENCY / 1000)
/* The SYS_CLK frequency should match the actual hardware SYS_CLK frequency. */
#define SYSCLK_FREQUENCY                (CYDEV_BCLK__SYSCLK__HZ)
/* The SYS_CLK counter theshold for 1ms timeout. Division is expected
 * to be optimized out by the compiler. */
#define SYSCLK_1MS_COUNT_VALUE          (SYSCLK_FREQUENCY / 1000)
/* SYS_TICK enable bit. */
#define CM0_SYSTICK_CSR_ENABLE          (1)
/* SYS_TICK interrupt enable. */
#define CM0_SYSTICK_CSR_TICKINT         (1 << 1)
/* SYS_TICK clock source selection for SYS_CLK */
#define CM0_SYSTICK_CSR_CLKSOURCE       (1 << 2)
/* SYS_TICK COUNTFLAG indicator */
#define CM0_SYSTICK_CSR_COUNTFLAG       (1 << 16)

/* Handle for software timers. */
static ccg_timer_t volatile gl_timer_handle[TIMER_NUM_INSTANCES][TIMER_NUM_TIMERS];

/* Number of active timers. */
static uint8_t volatile gl_num_active[TIMER_NUM_INSTANCES];
static uint8_t volatile gl_total_active;

/* Flag indicating whether invoked from timer callback. */
static bool volatile gl_is_callback;

/* Forward declaration for the internal interrupt handler. */
#if (TIMER_TYPE == TIMER_TYPE_WDT)
static void wdt_interrupt_handler(void);
#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */
static void systick_interrupt_handler(void);
#endif /* (TIMER_TYPE) */

#if (TIMER_TYPE == TIMER_TYPE_WDT)
static void wdt_calibrate(void)
{
    uint32_t start_count, end_count;
    /* Ensure that the timer is disabled. */
    CM0->syst_csr = 0;
    /* Clear the counter. */
    CM0->syst_cvr = 0;
    /* Load 1ms timeout. */
    CM0->syst_rvr = (SYSCLK_1MS_COUNT_VALUE - 1);

    /* Wait for the WDT counter to change once so that we ensure the
     * full calibration time. */
    start_count = (SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK);
    while (start_count == (SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK));
    /* Now note the WDT count value. */
    start_count = (SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK);

    CM0->syst_csr = (CM0_SYSTICK_CSR_ENABLE | CM0_SYSTICK_CSR_CLKSOURCE);
    while ((CM0->syst_csr & CM0_SYSTICK_CSR_COUNTFLAG) == 0);

    end_count = (SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK);

    /* Now calculate the actual calibration value. */
    if (start_count > end_count)
    {
        /* The counter had rolled over. */
        end_count |= 0x10000;
    }
    /* Store the value for reload. */
    gl_multiplier = end_count - start_count;

    CM0->syst_csr = 0;
}
#endif /* (TIMER_TYPE == TIMER_TYPE_WDT) */

/*
 * The function initializes the hardware timer. It should be noted that
 * the timer does not functioning at this point.
 */
static void hw_timer_init(void)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)

    /* Disable the WDT timer interrupts. */
    CyIntDisable (WDT_INTERRUPT_VECTOR_NUMBER);

    /* Disable WDT reset generation */
    SRSSLT->wdt_disable_key = WDT_RESET_DISABLE_KEY;
    /* Forward the interrupt to the CPU */
    SRSSLT->srss_intr_mask |= SRSS_INTR_MASK_WDT_MATCH;
    /* Start ILO */
    SRSSLT->clk_ilo_config |= CLK_ILO_CONFIG_ENABLE;
    /* Enable the watchdog timer interrupt */
    SRSSLT->srss_intr = SRSS_INTR_WDT_MATCH;

    /* Timer interrupt handling */
    (void)CyIntSetVector (WDT_INTERRUPT_VECTOR_NUMBER,
            &wdt_interrupt_handler);

    /* Calibrate the WDT timer. */
    wdt_calibrate ();

#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */

    /* Disable the timer. */
    CM0->syst_csr = 0;
    CM0->syst_rvr = (SYSCLK_1MS_COUNT_VALUE - 1);
    /* Register interrupt handler. */
    CyIntSetSysVector(CY_INT_SYSTICK_IRQN, systick_interrupt_handler);

#endif /* (TIMER_TYPE) */
}

/*
 * The function starts the hardware timer. The timer is expected to start
 * interrupt at configured time.
 */
static void hw_timer_start(void)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)

#if (TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    SRSSLT->wdt_match = (uint16_t)((SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK)
            + gl_multiplier);
    /* Need to wait for atleast 3 LF cycles before the register write completes. */
    CyDelayUs (100);
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Enable the WDT interrupt. */
    CyIntEnable (WDT_INTERRUPT_VECTOR_NUMBER);

#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */

    /* Clear the counter */
    CM0->syst_cvr = 0;
    /* Start the timer and enable the interrupt. */
    CM0->syst_csr = (CM0_SYSTICK_CSR_ENABLE | CM0_SYSTICK_CSR_CLKSOURCE |
            CM0_SYSTICK_CSR_TICKINT);

#endif /* (TIMER_TYPE) */
}

/*
 * The function stops the hardware timer. The timer is expected to
 * disable interrupts when stopped.
 */
static void hw_timer_stop(void)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)
    /* Disable the WDT interrupt */
    CyIntDisable (WDT_INTERRUPT_VECTOR_NUMBER);
#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */
    /* Disable the SYSTICK interrupt. */
    CM0->syst_csr = 0;
#endif /* (TIMER_TYPE) */
}

#if (TIMER_TICKLESS_ENABLE != 0)
/* The function retrieves the current hardware count reading. */
static uint16_t hw_timer_get_count(void)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)
    return (uint16_t)(SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK);
#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */
    /* TODO needs implementation. */
    return 0;
#endif /* (TIMER_TYPE) */
}

/* The function loads the hardware period for interrupt. */
static void hw_timer_load_period(uint16_t period)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)
    if ((SRSSLT->wdt_match & WDT_MATCH_MATCH_MASK) != gl_match)
    {
        CyDelayUs (100);
    }
    SRSSLT->wdt_match = period;
    gl_match = period;
#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */
    /* TODO needs implementation. */
#endif /* (TIMER_TYPE) */
}

static uint16_t hw_timer_get_tick_interval(uint16_t start, uint16_t current)
{
    uint32_t interval;

    if (start < current)
    {
        interval = (current - start);
    }
    else
    {
        interval = (TIMER_NUM_TICK_COUNTS - start) + current;
    }

    return (uint16_t)interval;
}
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

void timer_init(void)
{
    uint8_t i, j;

    /* Invalidate all timers. */
    for (i = 0; i < TIMER_NUM_INSTANCES; i++)
    {
        for (j = 0; j < TIMER_NUM_TIMERS; j++)
        {
            gl_timer_handle[i][j].id = TIMER_INVALID_ID;
        }

        gl_num_active[i] = 0;
    }

    gl_total_active = 0;

    /* Initialize the hardware timer. */
    hw_timer_init();
}

/* Internal function to scan the instances to identify the allocated timer. */
static uint8_t timer_get_index(uint8_t instance, timer_id_t id)
{
    uint8_t index;

    if (id == TIMER_INVALID_ID)
    {
        return TIMER_INVALID_INDEX;
    }

    for (index = 0; index < TIMER_NUM_TIMERS; index++)
    {
        if (gl_timer_handle[instance][index].id == id)
        {
            return index;
        }
    }

    return TIMER_INVALID_INDEX;
}

bool timer_start(uint8_t instance, timer_id_t id, uint16_t period, timer_cb_t cb)
{
    int8_t i;
    uint8_t state;
    bool status = false;
    uint8_t index = TIMER_INVALID_INDEX;

    if ((id == TIMER_INVALID_ID) || (period == 0))
    {
        return false;
    }

    /* Enter critical section. */
    state = CyEnterCriticalSection();

    /*
     * Allocates a free instance. Two checks need to be done. First we need
     * to identify if the ID is already active. If so, we should re-use the
     * same ID. If the timer ID is not active, then we should look for a 
     * free slot. Here we are combining the two loops to improve performance.
     * So to make the allocation happen from zero up, the loop is run from
     * the highest to lowest.
     */
    for (i = (TIMER_NUM_TIMERS - 1); i >= 0; i--)
    {
        /* Pick a free instance. */
        if (gl_timer_handle[instance][i].id == TIMER_INVALID_ID)
        {
            index = i;
        }

        /*
         * Scan again if the same timer ID has been previously allocated.
         * If so return the previously allocated instance.
         */
        if (gl_timer_handle[instance][i].id == id)
        {
            index = i;
            /*
             * We are going to re-allocate an already running timer.
             * This means that the previous instance is disabled.
             */
            gl_num_active[instance]--;
            gl_total_active--;
            break;
        }
    }

    if (index != TIMER_INVALID_INDEX)
    {
        gl_timer_handle[instance][index].period = period;
        gl_timer_handle[instance][index].cb = cb;
        gl_timer_handle[instance][index].id = id;

#if (TIMER_TICKLESS_ENABLE != 0)

        /* Multiply the required time with the multiplier. */
        gl_timer_handle[instance][index].count = period * gl_multiplier;

        /*
         * Only start or adjust timer if outside of the timer callback.
         * When inside timer callback, the timer is already started and
         * no adjustment is required.
         */
        if (gl_is_callback == false)
        {
            uint32_t count;
            uint16_t cur_count;

            cur_count = hw_timer_get_count ();
            /*
             * If the timer is already running, then the count has
             * to be adjusted to take care of partial update.
             */
            if (gl_total_active != 0)
            {
                count = hw_timer_get_tick_interval (gl_start_tick,
                        cur_count);
                gl_timer_handle[instance][index].count += count;

                if (gl_timer_handle[instance][index].count < gl_tick_time)
                {
                    /*
                     * It is safe to load the timer here as we have
                     * confirmed that the timer is not going to fire
                     * immediately. Also we know for sure that the
                     * count is less than the maximum value allowed.
                     */
                    gl_tick_time = gl_timer_handle[instance][index].count;
                    hw_timer_load_period ((uint16_t) (gl_start_tick +
                                gl_tick_time));
                }

            }
            else
            {
                /* Need to start the timer if not already running. */
                gl_start_tick = cur_count;
                count = gl_timer_handle[instance][index].count;
                if (count > TIMER_HW_MAX_TIMEOUT)
                {
                    count = TIMER_HW_MAX_TIMEOUT;
                }
                gl_tick_time = count;
                hw_timer_load_period ((uint16_t) (count + cur_count));
                hw_timer_start ();
            }
        }

#else /* (TIMER_TICKLESS_ENABLE == 0) */

        gl_timer_handle[instance][index].count = period;
        /*
         * Turn on hardware timer if not running. Do this only
         * if invoked outside of timer callback.
         */
        if ((gl_total_active == 0) && (gl_is_callback == false))
        {
            hw_timer_start ();
        }

#endif /* (TIMER_TICKLESS_ENABLE) */

        gl_num_active[instance]++;
        gl_total_active++;
        status = true;
    }

    /* Exit critical section. */
    CyExitCriticalSection(state);

    return status;
}

bool timer_start_wocb(uint8_t instance, timer_id_t id, uint16_t period)
{
    return timer_start(instance, id, period, NULL);
}    

/*
 * The function should not be called other than from timer_stop functions.
 * The function expects that the call is being made from inside a critical
 * section and that the timer instance and the index into the timer handle
 * is valid.
 */
static void timer_stop_internal(uint8_t instance, uint8_t index)
{
    /* Stop timer and deallocate it */
    gl_timer_handle[instance][index].id = TIMER_INVALID_ID;
    gl_num_active[instance]--;
    gl_total_active--;

    /*
     * Turn off hardware timer if no timer is active. Turn off should
     * be done only if not being invoked from inside a timer callback.
     * If this is from timer callback, the interrupt handler deals
     * with this at the end of all timer checks.
     */
    if ((gl_total_active == 0) && (gl_is_callback == false))
    {
        hw_timer_stop();
    }
}

void timer_stop(uint8_t instance, timer_id_t id)
{
    uint8_t index, state;

    /* Enter critical section. */
    state = CyEnterCriticalSection ();

    /* Get the timer index from the handle */
    index = timer_get_index (instance, id);
    if (index != TIMER_INVALID_INDEX)
    {
        timer_stop_internal(instance, index);
    }

    /* Exit critical section. */
    CyExitCriticalSection (state);
}

void timer_stop_all(uint8_t instance)
{
    uint8_t index, state;

    /* Enter critical section. */
    state = CyEnterCriticalSection ();

    for (index = 0; index < TIMER_NUM_TIMERS; index++)
    {
        /* Stop the timer only for a valid id. */
        if (gl_timer_handle[instance][index].id != TIMER_INVALID_ID)
        {
            timer_stop_internal(instance, index);
        }
    }

    /* Exit critical section. */
    CyExitCriticalSection (state);
}

void timer_stop_range(uint8_t instance, timer_id_t start, timer_id_t end)
{
    uint8_t index, state;

    /*
     * We should not allow the stop call to happen if the range is invalid.
     * Here we are assuming that the TIMER_INVALID_ID is the largest possible
     * value for the ID. If not we would have to explicitly check for validity
     * of start parameter as well.
     */
    if ((start >= end) || (end == TIMER_INVALID_ID))
    {
        return;
    }

    /* Enter critical section. */
    state = CyEnterCriticalSection ();

    for (index = 0; index < TIMER_NUM_TIMERS; index++)
    {
        timer_id_t id = gl_timer_handle[instance][index].id;
        /* Stop the timer only for the specified ID range. */
        if ((id >= start) && (id <= end))
        {
            timer_stop_internal(instance, index);
        }
    }

    /* Exit critical section. */
    CyExitCriticalSection (state);
}

bool timer_is_running(uint8_t instance, timer_id_t id)
{
    uint8_t index;

    index = timer_get_index (instance, id);
    if (index != TIMER_INVALID_INDEX)
    {
        return true;
    }

    return false;
}

uint8_t timer_num_active(uint8_t instance)
{
    return gl_num_active[instance];
}

uint16_t timer_get_count(uint8_t instance, timer_id_t id)
{
    uint8_t index, state;
    uint32_t count = 0;

    /* Enter critical section. */
    state = CyEnterCriticalSection ();

    index = timer_get_index (instance, id);
    if(index != TIMER_INVALID_INDEX)
    {
#if (TIMER_TICKLESS_ENABLE != 0)
        uint32_t temp_count = gl_timer_handle[instance][index].count;

        count = hw_timer_get_tick_interval(gl_start_tick, hw_timer_get_count());
        if (temp_count > count)
        {
            count = ((temp_count - count) / gl_multiplier);
        }
        else
        {
            count = 0;
        }
#else /* (TIMER_TICKLESS_ENABLE == 0) */
        count = gl_timer_handle[instance][index].count;
#endif /* (TIMER_TICKLESS_ENABLE) */
    }

    /* Exit critical section. */
    CyExitCriticalSection (state);

    return count;
}

uint16_t timer_get_multiplier(void)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)
    return(gl_multiplier);
#else
    /* Value is not known as we have not run calibration. Return a nominal value assuming LFCLK of 40 KHz. */
    return(25);
#endif /* TIMER_TYPE */
}

void timer_enter_sleep(void)
{
#if (TIMER_TYPE == TIMER_TYPE_WDT)
    if (gl_total_active != 0)
    {
#if (TIMER_TICKLESS_ENABLE != 0)
        if ((SRSSLT->wdt_match & WDT_MATCH_MATCH_MASK) != gl_match)
#endif /* (TIMER_TICKLESS_ENABLE != 0) */
        {
            /* To allow WDT registers to synchronize */
            CyDelayUs(50);
        }        
    }
#endif /* (TIMER_TYPE) */
}

/*
 * The timer ISR handler is generated whenever the timer expires. The
 * ISR scans all running instances and decrements the count appropriately.
 *
 * Timer ISR is called every 1ms (or configured period for tickless). If there
 * is any soft timer ON In the ISR all active timer instances are decremented
 * and then checked for timeouts. In case of timeout the timer is stopped and
 * de-allocated. In case, a timer needs to be re-started, it can be donefrom
 * the callback. On timer expire events are also raised here for various modules.
 */
static void timer_interrupt_handler(void)
{
    uint8_t index, instance;
    timer_id_t id;
#if (TIMER_TICKLESS_ENABLE != 0)
    uint16_t cur_count;
    uint32_t interval, period;
#endif /* (TIMER_TICKLESS_ENABLE != 0) */

    /* Indicate being inside the interrupt handler. */
    gl_is_callback = true;

    do
    {
#if (TIMER_TICKLESS_ENABLE != 0)
        cur_count = hw_timer_get_count ();
        interval = hw_timer_get_tick_interval (gl_start_tick, cur_count);

        gl_start_tick = cur_count;
#endif /* (TIMER_TICKLESS_ENABLE != 0) */

        for (instance = 0; instance < TIMER_NUM_INSTANCES; instance++)
        {
            /* Loop through all soft timers */
            for (index = 0; index < TIMER_NUM_TIMERS; index++)
            {
                id = gl_timer_handle[instance][index].id;
                if (id != TIMER_INVALID_ID)
                {
#if (TIMER_TICKLESS_ENABLE != 0)
                    /* Decrement the counters. */
                    if (gl_timer_handle[instance][index].count >
                            (interval + TIMER_OVERRUN_THRESHOLD))
                    {
                        gl_timer_handle[instance][index].count -= interval;
                    }
                    else
                    {
                        gl_timer_handle[instance][index].count = 0;
                    }
#else /* (TIMER_TICKLESS_ENABLE == 0) */
                    gl_timer_handle[instance][index].count--;
#endif /* (TIMER_TICKLESS_ENABLE) */
                    if (gl_timer_handle[instance][index].count == 0)
                    {
                        gl_timer_handle[instance][index].id = TIMER_INVALID_ID;
                        gl_num_active[instance]--;
                        gl_total_active--;
                        if (gl_timer_handle[instance][index].cb != NULL)
                        {
                            gl_timer_handle[instance][index].cb (instance, id);
                        }
                    }
                }
            }
        }

#if (TIMER_TICKLESS_ENABLE != 0)
        period = TIMER_HW_MAX_TIMEOUT;
        interval = 0;
        if (gl_total_active != 0)
        {
            /* Now identify the next timer setting. */
            for (instance = 0; instance < TIMER_NUM_INSTANCES; instance++)
            {
                for (index = 0; index < TIMER_NUM_TIMERS; index++)
                {
                    if (gl_timer_handle[instance][index].id != TIMER_INVALID_ID)
                    {
                        if (gl_timer_handle[instance][index].count < period)
                        {
                            period = gl_timer_handle[instance][index].count;
                        }
                    }
                }
            }

            /*
             * Since there is a remote possiblilty of us missing the timer
             * window while in the callback, re-run to avoid. The design
             * of the software should be to have only small callback
             * handlers. There is a possibility that if the callbacks are
             * large, firmware will not get any time to execute main task.
             */
            interval = hw_timer_get_tick_interval (gl_start_tick,
                    hw_timer_get_count());
        }

    } while (period < (interval + TIMER_OVERRUN_THRESHOLD));

    /*
     * Load the timer with the required period. The value is intentionally
     * type-casted to 16 bit so that it takes care of wrap around.
     */
    gl_tick_time = period;
    hw_timer_load_period ((uint16_t)(period + gl_start_tick));

#else /* (TIMER_TICKLESS_ENABLE == 0) */
    } while (0);
#endif /* (TIMER_TICKLESS_ENABLE) */

    /* Indicate that we are done with the interrupt handler. */
    gl_is_callback = false;

    /* If no timer is runing turn off the hardware timer */
    if(gl_total_active == 0)
    {
        hw_timer_stop();
    }
}

#if (TIMER_TYPE == TIMER_TYPE_WDT)
/* WDT interrupt handler timer expiry interrupt handler. */
static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    SRSSLT->srss_intr = SRSS_INTR_WDT_MATCH;

#if (TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    SRSSLT->wdt_match = (uint16_t)((SRSSLT->wdt_counter & WDT_COUNTER_COUNTER_MASK)
            + gl_multiplier);
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    timer_interrupt_handler ();
}
#else /* (TIMER_TYPE == TIMER_TYPE_SYSTICK) */
/* SYSTICK timer expiry interrupt handler. */
static void systick_interrupt_handler(void)
{
    /* Clear the count flag by reading the CM0->syst_csr register. */
    if (CM0->syst_csr & CM0_SYSTICK_CSR_COUNTFLAG)
    {
        /* Invoke the timer handler. */
        timer_interrupt_handler ();
    }
}
#endif /* (TIMER_TYPE) */

/* EOF */

