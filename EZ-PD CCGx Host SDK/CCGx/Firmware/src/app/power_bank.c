/**
 * @file power_bank.c
 *
 * @brief
 *
 *******************************************************************************
 * @copyright
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

#include <pd.h>
#include <gpio.h>
#include <dpm.h>
#include <pdss_hal.h>
#include <app.h>
#include <type_a.h>
#include <power_bank.h>
#include <timer.h>

/* VBATT_MON GPIO. */
#define VBATT_MON_GPIO                         (GPIO_PORT_2_PIN_1)

/* Power bank's battery status. */
pb_battery_state_t gl_pb_battery_state;

/* Flag to register intiate PR SWAP request. */
bool pb_initiate_pr_swap = false;

static uint8_t gl_pb_debounce_count = 0;
static bool gl_pb_debounce_active = false;

void pb_typec_set_current (uint16_t cur_10mA)
{
    uint32_t pwm_duty;
    uint32_t p_inp;
    uint16_t volt;
    uint16_t i_bat;

    /*
     * Set the battery charging current limit. This is how the current is determined:
     * Current Contract defines the Input Power (P_INP).
     * Using this and VBATT_MAX we can calculate the max battery charging current (I_BAT):
     * I_BAT = P_INP / VBATT_MAX
     * But battery has a max charging current limit as well (I_BAT_MAX). So we will
     * use minimum of I_BAT and I_BAT_MAX and then use PWM to set the battery charging current.
     */

    /* Get input power (in mW) based on type of contract.*/
    volt = app_get_status(TYPEC_PORT_0_IDX)->psnk_volt;
    p_inp = (volt * (cur_10mA * PD_CUR_PER_UNIT));
    /* Get output current in mA units. */
    i_bat = (p_inp / (pd_get_ptr_bat_chg_tbl(TYPEC_PORT_0_IDX)->vbatt_max_volt));

    /* Now take minimum of i_bat calculated and I_BAT_MAX from config table. */
    i_bat = GET_MIN (i_bat, pd_get_ptr_bat_chg_tbl(TYPEC_PORT_0_IDX)->vbatt_max_cur);

    /* Calculate PWM duty cycle for max battery charging current calculated above. */
    pwm_duty = ((PWM_MAX_CURRENT_COMP_VALUE  * i_bat) / pd_get_ptr_bat_chg_tbl(TYPEC_PORT_0_IDX)->vbatt_max_cur);
    /* Start PWM block. */
    PWMI_C_Start ();
    PWMI_C_WriteCompare (pwm_duty);
    gpio_set_drv_mode (PWM_I_GPIO, GPIO_DM_STRONG);
}

static void pb_typec_configure_sink(uint8_t port)
{
    /* Configure TYPE-C port as sink because battery is weak. */
    dpm_stop (port);
    dpm_update_port_config (port, PRT_ROLE_SINK, PRT_ROLE_SINK, false, false);
    dpm_start (port);
    /* Update battery status to dead battery and not charging. */
    gl_pb_battery_state = PB_BATTERY_STATE_DB_NOT_CHARGING;
}

void pb_event_handler(uint8_t port, uint32_t evt)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    /*
     * See if we are waiting for disconnect as source when battery is weak.
     * If yes, configure TYPE-C port as sink.
     */
    if (evt == APP_EVT_DISCONNECT)
    {
        if (gl_pb_battery_state == PB_BATTERY_STATE_DB_TYPEC_WAIT_DC)
        {
           pb_typec_configure_sink (port);
        }

        /* Ensure there is no pending initiate PR SWAP request in system. */
        pb_initiate_pr_swap = false;
    }
    /*
     * If PB locked as as sink on TYPE-C port then initiate a PR SWAP if battery is
     * in normal mode and port partner is DRP and not externally powered.
     */
    else if (evt == APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE)
    {
        if ((dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK) && (gl_pb_battery_state
            == PB_BATTERY_STATE_NORMAL))
        {
            /* Check if port partner is DRP and not externally powered. */
            if ((dpm_stat->src_cap_p != NULL) &&
                (dpm_stat->src_cap_p->dat[0].fixed_src.dual_role_power == true) &&
                (dpm_stat->src_cap_p->dat[0].fixed_src.ext_powered == false))
            {
                pb_initiate_pr_swap = true;
            }
        }
    }
}

static void pb_vbus_dis_cb (uint8_t port)
{
    /* Do nothing */
}

static uint16_t pb_get_battery_voltage(void)
{
    /* Measure voltage on VBATT_MON GPIO. */
    uint8_t level;
    uint16_t vbatt;

    hsiom_set_config (VBATT_MON_GPIO, HSIOM_MODE_AMUXA);

    /* Take ADC sample. */
    level = pd_adc_sample (0, PD_ADC_ID_1, PD_ADC_INPUT_AMUX_A);

    /* Convert level to VBUS using 1/11 divider. */
    vbatt = pd_adc_level_to_volt (0, PD_ADC_ID_1, level);
    hsiom_set_config (VBATT_MON_GPIO, HSIOM_MODE_GPIO);
    /* TODO: Value on kits board is not 1/11 but around 1/12. */
    return (vbatt * 12);
}

void pb_task_init(void)
{
    /* PowerBank task initialization routine. */

    /*
     * At init, Battery can be in following states:
     * 1) Battery is normal and powering the system.
     * 2) Battery is dead and TYPE-C VBUS is powering the system.
     */
    if (dpm_get_info(TYPEC_PORT_0_IDX)->dead_bat == true)
    {
        gl_pb_battery_state = PB_BATTERY_STATE_DB_TYPEC_CHARGING;
        /* Disable TYPE-A port because battery is dead. */
#if CCG_TYPE_A_PORT_ENABLE
        /* Disable TYPE-A port. */
        type_a_port_disable ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
    else
    {
        /* TODO: What if battery is powering the system but is in weak condition? */
        gl_pb_battery_state = PB_BATTERY_STATE_NORMAL;
    }
}

static void pb_pr_swap_cb(uint8_t port, resp_status_t resp,  const pd_packet_t* pkt_ptr)
{
    /*
     * If PR SWAP successfully transmitted, clear the flag.
     * Note that there is no retry mechanism if port partner REJECTS PR SWAP
     * request. Code can be updated here if retry is needed.
     */
    if (resp == RES_RCVD)
    {
        /* Clear PR SWAP request flag. */
        pb_initiate_pr_swap = false;
    }
}

static void pb_debounce_cb(uint8_t port, timer_id_t id)
{
    uint16_t vbatt = 0;

    if(PB_DEBOUNCE_TIMER_ID == id)
    {
        vbatt = pb_get_battery_voltage ();
        
        if (vbatt <= pd_get_ptr_bat_chg_tbl(port)->vbatt_cutoff_volt)
        {
            gl_pb_debounce_count++;
            if(gl_pb_debounce_count <= APP_PB_VBATT_DEBOUNCE_IN_MS)
            {
                /* Continue VBATT debouncing for cutoff */
                timer_start (port, PB_DEBOUNCE_TIMER_ID, PB_DEBOUNCE_PERIOD, pb_debounce_cb);
            }
            else
            {
                /*
                 * VBATT was less than CUT_OFF VBATT for the entire
                 * debounce duration. Will catch this in task context in
                 * pb_bat_monitor().
                 * Note: gl_pb_debounce_count not reseted to 0. This will
                 * help in catching it in task context.
                 */
                gl_pb_debounce_active = false;
            }
        }
        else
        {
            /* VBATT sample found > than cut off voltage */
            gl_pb_debounce_count = 0;
            gl_pb_debounce_active = false;
        }
    }
} /* End of pb_debounce_cb() */

void pb_bat_monitor(void)
{
    uint16_t vbatt;

    /* See if a PR SWAP request is pending. If yes, initiate PR SWAP. */
    if (pb_initiate_pr_swap == true)
    {
        dpm_pd_command (TYPEC_PORT_0_IDX, DPM_CMD_SEND_PR_SWAP, NULL, pb_pr_swap_cb);
        return;
    }

    /* Measure VBATT. */
    vbatt = pb_get_battery_voltage ();

    /* Handle power bank state based on battery status. */
    switch (gl_pb_battery_state)
    {
        case PB_BATTERY_STATE_NORMAL:

            if((gl_pb_debounce_active == false) && (gl_pb_debounce_count == 0))
            {
                /*
                 * Code flow will come here when the Debounce timer has not
                 * been started at all or VBATT was NOT less than CUT_OFF
                 * VBATT for the entire debounce duration.
                 */
                gl_pb_debounce_active = true;
                timer_start (TYPEC_PORT_0_IDX, PB_DEBOUNCE_TIMER_ID, PB_DEBOUNCE_PERIOD, pb_debounce_cb);
            }
            else if((gl_pb_debounce_active == false) && (gl_pb_debounce_count > APP_PB_VBATT_DEBOUNCE_IN_MS))
            {
                /* Valid VBATT < than CUT_OFF detected */
                gl_pb_debounce_count = 0;
#if CCG_TYPE_A_PORT_ENABLE
                /* Disable TYPE-A port. */
                type_a_port_disable ();
#endif /* CCG_TYPE_A_PORT_ENABLE */

                /* If TYPE-C port is not connected to anything,
                 * update it's role to sink only mode from DRP mode. */
                if (dpm_get_info (TYPEC_PORT_0_IDX)->attach == false)
                {
                    pb_typec_configure_sink (TYPEC_PORT_0_IDX);
                }
                /* If TYPE-C port is sourcing power, disable PD,
                 * turn off VBUS and wait for disconnect */
                else if (dpm_get_info (TYPEC_PORT_0_IDX)->cur_port_role == PRT_ROLE_SOURCE)
                {
                    app_conf_for_faulty_dev_removal (TYPEC_PORT_0_IDX);
                    /* Turn off VBUS. */
                    dpm_get_info(TYPEC_PORT_0_IDX)->app_cbk->psrc_disable(TYPEC_PORT_0_IDX,
                        pb_vbus_dis_cb);
                    /* Now wait for disconnect. on disconnect, configure port as Sink only. */
                    gl_pb_battery_state = PB_BATTERY_STATE_DB_TYPEC_WAIT_DC;
                }
                /* TYPE-C port is sink and charging the battery. */
                else
                {
                    gl_pb_battery_state = PB_BATTERY_STATE_DB_TYPEC_CHARGING;
                }
            }
            break;

        case PB_BATTERY_STATE_DB_TYPEC_CHARGING:
            /* Battery is weak. TYPE-C port is charging the battery. */

            /*
             * Battery is weak but TYPE-C port is providing power to system. Check if VBATT is more than
             * RESTART range. if yes, disable internal regulator.
             */
            if (vbatt >= pd_get_ptr_bat_chg_tbl(TYPEC_PORT_0_IDX)->vbatt_dischg_en_volt)
            {
                pd_hal_disable_vreg (TYPEC_PORT_0_IDX);
            }

            /*
             * If we TYPE-C port is now no longer connected then we can check the battery voltage again.
             * If VBATT is more than RESTART voltage, we can go back to normal state.
             * If VBATT is less than RESTART voltage, stay in dead battery mode and udpdate the state.
             */
            if (dpm_get_info (TYPEC_PORT_0_IDX)->attach == false)
            {
                if (vbatt >= pd_get_ptr_bat_chg_tbl(TYPEC_PORT_0_IDX)->vbatt_dischg_en_volt)
                {
                    /* Configure TYPE-C port as DRP again. */
                    dpm_stop (TYPEC_PORT_0_IDX);
                    dpm_update_port_config (TYPEC_PORT_0_IDX, PRT_DUAL, PRT_ROLE_SOURCE,
                        true, false);
                    dpm_start (TYPEC_PORT_0_IDX);
#if CCG_TYPE_A_PORT_ENABLE
                    /* Re-enable TYPE-A port. */
                    type_a_port_enable ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
                    gl_pb_battery_state = PB_BATTERY_STATE_NORMAL;
                }
                else
                {
                    if((gl_pb_debounce_active == false) && (gl_pb_debounce_count == 0))
                    {
                        gl_pb_debounce_active = true;
                        timer_start (TYPEC_PORT_0_IDX, PB_DEBOUNCE_TIMER_ID, PB_DEBOUNCE_PERIOD, pb_debounce_cb);
                    }
                    else if((gl_pb_debounce_active == false) && (gl_pb_debounce_count > APP_PB_VBATT_DEBOUNCE_IN_MS))
                    {
                         gl_pb_debounce_count = 0;
                        /*
                         * Ensure TYPE-A port is disabled as battery is weak and no one is charging battery
                         * over TYPE-C port.
                         */
#if CCG_TYPE_A_PORT_ENABLE
                        /* Disable TYPE-A port. */
                        type_a_port_disable ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
                        /* Ensure TYPE-C port is configured as sink. */
                        pb_typec_configure_sink (TYPEC_PORT_0_IDX);
                    }
                }
            }
            /*
             * TYPE-C port is charging the battery. While battery is getting charged, we can enable
             * TYPE-A port as a SDP only.
             */
            else
            {
#if CCG_TYPE_A_PORT_ENABLE
                type_a_enable_sdp ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
            }
            break;

        case PB_BATTERY_STATE_DB_NOT_CHARGING:
            /* Battery is weak. We have disabled TYPE-A port and TYPE-C port is in sink mode.
             * When TYPE-C port is connected to a power source, update the state. */
             if (dpm_get_info (TYPEC_PORT_0_IDX)->attach == true)
            {
                gl_pb_battery_state = PB_BATTERY_STATE_DB_TYPEC_CHARGING;
            }
            break;

        case PB_BATTERY_STATE_DB_TYPEC_WAIT_DC:
            /* Nothing to be handled in this state. */
            break;

        default:
            break;
    }
}

 /* End of File */
