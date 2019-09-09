/**
 * @file psink.c
 *
 * @brief @{Power Sink (Consumer) manager source file.@}
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

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <psink.h>
#include <app.h>
#include <timer.h>


void app_psnk_vbus_ovp_cbk(uint8_t port, bool comp_out);

void sink_fet_off(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_VBUS_SNK_FET_OFF_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SNK_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

void sink_fet_on(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_VBUS_SNK_FET_ON_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SNK_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

#if VBUS_OVP_ENABLE

void app_psnk_vbus_ovp_cbk(uint8_t port, bool comp_out)
{
    /*OVP fault*/
    sink_fet_off(port);

    /* Set alert message */
    pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ovp = true;
    dpm_set_alert(port, alert);

    /*Enqueue HPI OVP fault event*/
    app_event_handler(port, APP_EVT_VBUS_OVP_FAULT, NULL);
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
void snk_vbus_uvp_cbk (uint8_t port, bool comp_out)
{
    /*UVP fault*/
    sink_fet_off(port);

    /*Enqueue HPI OVP fault event*/
    app_event_handler(port, APP_EVT_VBUS_UVP_FAULT, NULL);
}
#endif /* VBUS_UVP_ENABLE */

void psnk_set_voltage (uint8_t port, uint16_t volt_mV)
{
    app_status_t* app_stat = app_get_status(port);
    app_stat->psnk_volt = volt_mV;

    /* Disable VBus discharge when starting off as a SINK device. */
    vbus_discharge_off(port);

#if VBUS_OVP_ENABLE
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5))
    app_ovp_enable(port, volt_mV, CCG_SNK_FET, app_psnk_vbus_ovp_cbk);
#else /* CCG4 */
    app_ovp_enable(port, volt_mV, false, app_psnk_vbus_ovp_cbk);
#endif /* CCGx */
#endif /* VBUS_OVP_ENABLE */
}

void psnk_set_current (uint8_t port, uint16_t cur_10mA)
{
    app_status_t* app_stat = app_get_status(port);
    app_stat->psnk_cur = cur_10mA;
    
    /*
     * There are no implementation to update the current settings as of today.
     * This implementation needs to be updated when the CCGx solution has
     * capability to control the sink current capability.
     */

#if (POWER_BANK == 1)
    /* This is the implementation for CCG3 and CCG3PA Power banks. */

    if (port == TYPEC_PORT_0_IDX)
    {
        APP_SINK_SET_CURRENT_P1(cur_10mA);
    }
#if (CCG_PD_DUALPORT_ENABLE == 1)
    else
    {
        APP_SINK_SET_CURRENT_P2(cur_10mA);
    }
#endif /* (CCG_PD_DUALPORT_ENABLE == 1) */
#endif /* (POWER_BANK == 1) */
}

void psnk_enable (uint8_t port)
{
    uint8_t intr_state;
    intr_state = CyEnterCriticalSection();

    /* Make sure discharge path is disabled at this stage. */
    vbus_discharge_off(port);

    /* Turn on FETs only if dpm is enabled and there is no active fault condition. */
    if ((dpm_get_info(port)->dpm_enabled) && (dpm_get_info(port)->fault_active == false))
    {
        sink_fet_on(port);
    }

    CyExitCriticalSection(intr_state);
}

/*Timer Callback*/
static void app_psnk_tmr_cbk(uint8_t port, timer_id_t id)
{
    app_status_t* app_stat = app_get_status(port);

    switch(id)
    {
        case APP_PSINK_DIS_TIMER:
            timer_stop(port, APP_PSINK_DIS_MONITOR_TIMER);
            vbus_discharge_off(port);
            break;

        case APP_PSINK_DIS_MONITOR_TIMER:
            if(vbus_is_present(port, VSAFE_5V, 0) == false)
            {
                timer_stop(port, APP_PSINK_DIS_TIMER);
                vbus_discharge_off(port);
                app_stat->snk_dis_cbk(port);
            }
            else
            {
                /*Start Monitor Timer again*/
                timer_start(port, APP_PSINK_DIS_MONITOR_TIMER, APP_PSINK_DIS_MONITOR_TIMER_PERIOD, app_psnk_tmr_cbk);
            }
            break;
        default:
            break;
    }
}

void psnk_disable (uint8_t port, sink_discharge_off_cbk_t snk_discharge_off_handler)
{
    uint8_t intr_state;
    app_status_t* app_stat = app_get_status(port);

    intr_state = CyEnterCriticalSection();

#if VBUS_OVP_ENABLE
#ifdef CCG3
    app_ovp_disable (port, CCG_SNK_FET);
#else
    app_ovp_disable (port, false);
#endif /* CCG3 */
#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
    app_uvp_disable (port, CCG_SNK_FET);
#endif /* VBUS_UVP_ENABLE */

    sink_fet_off(port);

    vbus_discharge_off(port);
    timer_stop_range(port, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_MONITOR_TIMER);

    if ((snk_discharge_off_handler != NULL) && (dpm_get_info(port)->dpm_enabled))
    {
        vbus_discharge_on(port);

        app_stat->snk_dis_cbk = snk_discharge_off_handler;

        /* Start Power source enable and monitor timer. */
        timer_start(port, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_TIMER_PERIOD, app_psnk_tmr_cbk);
        timer_start(port, APP_PSINK_DIS_MONITOR_TIMER, APP_PSINK_DIS_MONITOR_TIMER_PERIOD, app_psnk_tmr_cbk);
    }
#if ((VBUS_IN_DISCHARGE_EN) && (POWER_BANK))
    else
    {
        /* It does not make sense to extra discharge even if we are not Type-C attached */
        if ((dpm_get_info(port)->attach == true) && (app_get_status(port)->psnk_volt > VSAFE_5V))
        {
            /*
             * If the same FET controls both source and sink operation, the VBUS_IN
             * side of the FET needs to be discharged to less than 5V. This operation
             * needs to be done. It is expected that this discharge shall happen 
             * before the tSrcTransition period. A fixed delay is used for this. It
             * can be extended to monitor VBUS based on the design as required.
             */
            pd_internal_vbus_in_discharge_on(port);
            timer_start(port, APP_PSINK_DIS_TIMER, APP_PSINK_DIS_VBUS_IN_DIS_PERIOD, app_psnk_tmr_cbk);
        }
    }
#endif /* ((VBUS_IN_DISCHARGE_EN) && (POWER_BANK)) */

    /* Update the psnk_volt data structure so that we do not have stale value
     * till the next sink attach */
    app_stat->psnk_volt = VSAFE_5V;
    
    CyExitCriticalSection(intr_state);
}

/* End of File */

