/**
 * @file bb_external.c
 *
 * @brief @{External billboard control functions.@}
 *
 *******************************************************************************
 *
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

#include <config.h>
#include <app.h>
#include <dpm.h>
#include <hpi.h>
#include <hpi_internal.h>
#include <timer.h>
#include <billboard.h>

#if (CCG_BB_ENABLE != 0)

/**< Billboard connect state variable for BILLBOARD_UPDATE_EVENT. */    
static uint32_t bb_connect_stat[NO_OF_TYPEC_PORTS];           
static volatile bb_handle_t gl_bb[NO_OF_TYPEC_PORTS];

static void bb_set_conn_stat(uint8_t port, uint8_t conn_stat)
{
    bb_connect_stat[port] = (uint32_t)conn_stat;
    /* Send BB notifications to the solution */
    sln_pd_event_handler (port, APP_EVT_BB, &(bb_connect_stat[port]));
    
    return;
}

ccg_status_t bb_init(uint8_t port)
{  
#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    uint32_t data;
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    if (gl_bb[port].state != BB_STATE_DEINITED)
    {
        return CCG_STAT_BUSY;
    }

    /* Verify if the configuration allows billboard. */
    if(
        (pd_get_ptr_bb_tbl(port)->bb_enable != BB_TYPE_EXTERNAL) &&
        (pd_get_ptr_bb_tbl(port)->bb_enable != BB_TYPE_EXT_CONFIGURABLE)
    )
    {
        return CCG_STAT_FAILURE;
    }

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    /* Init alt mode status register by default values */
    data = BB_ALT_MODE_STATUS_INIT_VAL;
    hpi_bb_reg_update (HPI_DEV_BB_ALT_MODE_STATUS, &data);
    /* Set BB operation model for "EC + Reset" */
    data = HPI_BB_OPER_MODEL;
    hpi_bb_reg_update (HPI_DEV_BB_OPER_MODEL, &data);
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    gl_bb[port].type       = BB_TYPE_EXTERNAL;
    gl_bb[port].state      = BB_STATE_DISABLED;
    gl_bb[port].alt_status = BB_ALT_MODE_STATUS_INIT_VAL;

    return CCG_STAT_SUCCESS;
}

/* Billboard OFF timer callback. */
static void bb_off_timer_cb(uint8_t port, timer_id_t id)
{
    (void)id;

    if(gl_bb[port].timeout == 0)
    {
        /* Disable the billboard interface. */
        bb_disable(port, false);
    }
    else
    {
        /* Continue the timer until the required delay is achieved. */
        uint32_t timeout = gl_bb[port].timeout;

        if(timeout > BB_OFF_TIMER_MAX_INTERVAL)
        {
            timeout = BB_OFF_TIMER_MAX_INTERVAL;
        }
        gl_bb[port].timeout -= timeout;

        timer_start(port, APP_BB_OFF_TIMER, (uint16_t)timeout, bb_off_timer_cb);
    }
}

ccg_status_t bb_enable(uint8_t port, bb_cause_t cause)
{
    uint32_t timeout = 0;

    /* Check if BB is already running */
    if(gl_bb[port].state != BB_STATE_BILLBOARD)
    {
        if (cause == BB_CAUSE_AME_TIMEOUT) 
        {
            /* Send Connect BB event */
            bb_set_conn_stat(port, BB_CONNECT_STAT);
        }
        /* Queue an enable only if the block is initialized and not in flashing mode. */
        if ((gl_bb[port].state == BB_STATE_DEINITED) ||
                (gl_bb[port].state == BB_STATE_FLASHING))
        {
            return CCG_STAT_NOT_READY;
        }

        /* Allow billboard enumeration only if the configuration allows. */
        if ((cause == BB_CAUSE_AME_SUCCESS) &&
                (pd_get_ptr_bb_tbl(port)->bb_always_on == false))
        {
            return CCG_STAT_NOT_READY;
        }

        /* Ensure that the OFF_TIMER is disabled. */
        timer_stop(port, APP_BB_OFF_TIMER);

        /* Update the state. */
        gl_bb[port].state = BB_STATE_BILLBOARD;

        /* Disable the billboard interface after the specified timeout in case of SUCCESS status. */
        if(
                (cause == BB_CAUSE_AME_SUCCESS) &&
                (pd_get_ptr_bb_tbl(port)->bb_timeout != BB_OFF_TIMER_NO_DISABLE)
          )
        {
            timeout = pd_get_ptr_bb_tbl(port)->bb_timeout;
            if (timeout < BB_OFF_TIMER_MIN_VALUE)
            {
                timeout = BB_OFF_TIMER_MIN_VALUE;
            }

            /* Convert time to milliseconds. */
            timeout *= 1000;
            gl_bb[port].timeout = timeout;

            /* Ensure that the timeout does not exceed parameter boundary. */
            if(timeout > BB_OFF_TIMER_MAX_INTERVAL)
            {
                timeout = BB_OFF_TIMER_MAX_INTERVAL;
            }
            gl_bb[port].timeout -= timeout;

            timer_start(port, APP_BB_OFF_TIMER, (uint16_t)timeout, bb_off_timer_cb);
        }
        else
        {
            gl_bb[port].timeout = 0;
        }
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t bb_update_alt_status(uint8_t port, uint8_t mode_index,
        bb_alt_mode_status_t alt_status)
{
    uint32_t status;

    /* Allow update of alternate mode only if initialized and not in flashing */
    if ((gl_bb[port].state == BB_STATE_DEINITED) ||
            (gl_bb[port].state == BB_STATE_FLASHING))
    {
        return CCG_STAT_NOT_READY;
    }

    if (mode_index >= BB_MAX_ALT_MODES)
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }
    
    status = (gl_bb[port].alt_status & ~(BB_ALT_MODE_STATUS_MASK << (mode_index << 1)));
    status |= (alt_status << (mode_index << 1));
    gl_bb[port].alt_status = status;

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))

    /* Check if status was changed from the previous time */
    if (hpi_bb_get_reg(HPI_DEV_BB_ALT_MODE_STATUS) != status)
    {
        /* Send Connect BB event */
        bb_set_conn_stat(port, BB_CONNECT_STAT);
        /* Change BB Alt mode status */
        hpi_bb_reg_update (HPI_DEV_BB_ALT_MODE_STATUS, &status);
    }
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    return CCG_STAT_SUCCESS;
}

ccg_status_t bb_update_all_status(uint8_t port, uint32_t status)
{
    /* Allow update of alternate mode only if initialized and not in flashing */
    if ((gl_bb[port].state == BB_STATE_DEINITED) ||
            (gl_bb[port].state == BB_STATE_FLASHING))
    {
        return CCG_STAT_NOT_READY;
    }

    gl_bb[port].alt_status = status;

#if ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE))
    /* Check if status was changed from the previous time */
    if (hpi_bb_get_reg(HPI_DEV_BB_ALT_MODE_STATUS) != status)
    {
        hpi_bb_reg_update (HPI_DEV_BB_ALT_MODE_STATUS, &status);
    }
#endif /* ((CCG_HPI_BB_ENABLE) && (CCG_HPI_ENABLE)) */

    return CCG_STAT_SUCCESS;
}

ccg_status_t bb_disable(uint8_t port, bool force)
{
    /* Allow disable during flashing only if force parameter is true. */
    if (((force != false) && (gl_bb[port].state <= BB_STATE_DISABLED)) ||
            ((force == false) && (gl_bb[port].state != BB_STATE_BILLBOARD)))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Check if BB is already disdabled */
    if(gl_bb[port].state != BB_STATE_DISABLED)
    {
        gl_bb[port].state = BB_STATE_DISABLED;
        /* Send Disconnect BB event */
        bb_set_conn_stat(port, BB_DISCONNECT_STAT);
    }
    return CCG_STAT_SUCCESS;
}

bool bb_is_present(uint8_t port)
{
    if(gl_bb[port].state == BB_STATE_DEINITED)
    {
        return false;
    }

    return true;
}

ccg_status_t bb_flashing_ctrl(uint8_t port, bool enable)
{
    (void)port;
    (void)enable;

    /* Nothing to do as external Billboard flashing is not supported. */
    return CCG_STAT_SUCCESS;
}

bool bb_is_idle(uint8_t port)
{
    (void)port;
    return true;
    /* Dummy function as there are no billboard tasks. */
}

bool bb_enter_deep_sleep(uint8_t port)
{
    (void)port;

    /* Sleep checks are handled in HPI layer. */
    return true;
}

void bb_task(uint8_t port)
{
    /* Dummy function as there are no billboard tasks. */
    (void)port;

    return;
}

uint8_t *bb_get_version()
{
    /* No version information available. */
    return NULL;
}

#endif /* (CCG_BB_ENABLE != 0) */

/* [] END OF FILE */
