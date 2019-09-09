/**
 * @file app.c
 *
 * @brief @{PD application handler source file.@}
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
#include <pd.h>
#include <dpm.h>
#include <psource.h>
#include <psink.h>
#include <pdo.h>
#include <swap.h>
#include <vdm.h>
#include <app.h>
#include <vdm_task_mngr.h>
#include <timer.h>
#include <hpi.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <hal_ccgx.h>
#include <gpio.h>

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#include <intel_ridge.h>
#endif /* RIDGE_SLAVE_ENABLE */

#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if DP_UFP_SUPP
#include <hpd.h>
#endif /* DP_UFP_SUPP */

#if BATTERY_CHARGING_ENABLE
#include <battery_charging.h>
#endif /* BATTERY_CHARGING_ENABLE */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
#include <type_a.h>
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if CCG_UCSI_ENABLE
#include <ucsi.h>
#endif /* CCG_UCSI_ENABLE */

ovp_settings_t* pd_get_ptr_ovp_tbl(uint8_t port)
{
    /* Update the OVP settings from the configuration table. */
    return ((ovp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->ovp_tbl_offset));
}

ocp_settings_t* pd_get_ptr_ocp_tbl(uint8_t port)
{
    /* Update the VBus OCP settings from the configuration table */
    return ((ocp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->ocp_tbl_offset));
}

uvp_settings_t* pd_get_ptr_uvp_tbl(uint8_t port)
{
    /* Update the VBus UVP settings from the configuration table */
    return ((uvp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->uvp_tbl_offset));
}

scp_settings_t* pd_get_ptr_scp_tbl(uint8_t port)
{
    /* Update the VBus SCP settings from the configuration table */
    return ((scp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->scp_tbl_offset));
}

vconn_ocp_settings_t* pd_get_ptr_vconn_ocp_tbl(uint8_t port)
{
    /* Update the Vcon OCP settings from the configuration table */
    return ((vconn_ocp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->vconn_ocp_tbl_offset));
}

otp_settings_t* pd_get_ptr_otp_tbl(uint8_t port)
{
    /* Update the OTP settings from the configuration table */
    return ((otp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->otp_tbl_offset));
}

pwr_params_t* pd_get_ptr_pwr_tbl(uint8_t port)
{
    /* Update the power parameters from the configuration table */
    return ((pwr_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->pwr_tbl_offset));
}

chg_cfg_params_t* pd_get_ptr_chg_cfg_tbl(uint8_t port)
{
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Return parameters for TYPE-A port. */
        return pd_get_ptr_type_a_chg_cfg_tbl (0);
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    /* Update the legacy charging parameters from the configuration table */
    return ((chg_cfg_params_t *)((uint8_t *)(get_pd_config ()) +
        get_pd_port_config(port)->chg_cfg_tbl_offset));
}

bat_chg_params_t* pd_get_ptr_bat_chg_tbl(uint8_t port)
{
    /* Update the battery charging parameterss from the configuration table */
    return ((bat_chg_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->bat_chg_tbl_offset));
}

pwr_params_t* pd_get_ptr_type_a_pwr_tbl(uint8_t port)
{
    /* Update the power parameters of Type-A port from the configuration table */
    return ((pwr_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->type_a_pwr_tbl_offset));
}

typeA_chg_cfg_params_t* pd_get_ptr_type_a_chg_cfg_tbl(uint8_t port)
{
    /* Update the legacy charging parameters from the configuration table */
    return ((typeA_chg_cfg_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->type_a_chg_tbl_offset));
}

bb_settings_t* pd_get_ptr_bb_tbl(uint8_t port)
{
    /* Update the Billboard settings from the configuration table*/
    return ((bb_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->bb_tbl_offset));
}

#if (CCG_TYPE_A_PORT_ENABLE)
app_status_t app_status[2];
#else
app_status_t app_status[NO_OF_TYPEC_PORTS];
#endif /* CCG_TYPE_A_PORT_ENABLE */

enum {
    FAULT_TYPE_VBUS_OVP = 0,    /* 0 */
    FAULT_TYPE_VBUS_UVP,        /* 1 */
    FAULT_TYPE_VBUS_OCP,        /* 2 */
    FAULT_TYPE_VBUS_SCP,        /* 3 */
    FAULT_TYPE_CC_OVP,          /* 4 */
    FAULT_TYPE_VCONN_OCP,       /* 5 */
    FAULT_TYPE_SBU_OVP,         /* 6 */
    FAULT_TYPE_OTP,             /* 7 */
    FAULT_TYPE_COUNT            /* 8 */
};

/* Number of retries defined by user for each fault type. */
static uint8_t gl_app_fault_retry_limit[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] = {
    0
};

/* Number of times each fault condition has been detected during current connection. */
static volatile uint8_t gl_app_fault_count[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] = {
    0
};

/* Flag to indicate that activity timer timed out. */
static volatile bool ccg_activity_timer_timeout = false;

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)

/* Check whether any fault count has exceeded limit for the specified PD port. */
static bool app_port_fault_count_exceeded(uint8_t port)
{
    uint32_t i;
    bool     retval = false;

    /*
     * We can safely check for all fault types as the retry limits for all non-applicable
     * error types would have been set to 0.
     */
    for (i = 0; i < FAULT_TYPE_COUNT; i++)
    {
        if ((gl_app_fault_retry_limit[port][i] != 0) &&
                (gl_app_fault_count[port][i] >= gl_app_fault_retry_limit[port][i]))
        {
            retval = true;
            break;
        }
    }

    return (retval);
}

/* Clear all fault counters associated with the specified port. */
static void app_clear_fault_counters(uint8_t port)
{
#if VBUS_OCP_ENABLE
    gl_app_fault_count[port][FAULT_TYPE_VBUS_OCP] = 0;
#endif /* VBUS_OCP_ENABLE */
#if VBUS_SCP_ENABLE
    gl_app_fault_count[port][FAULT_TYPE_VBUS_SCP] = 0;
#endif /* VBUS_SCP_ENABLE */
#if VBUS_OVP_ENABLE
    /* Reset OVP Retry count on disconnect. */
    gl_app_fault_count[port][FAULT_TYPE_VBUS_OVP] = 0;
#endif /* VBUS_OVP_ENABLE */
#if VBUS_UVP_ENABLE
    /* Reset UVP Retry count on disconnect. */
    gl_app_fault_count[port][FAULT_TYPE_VBUS_UVP] = 0;
#endif /* VBUS_UVP_ENABLE */
}

#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */

/* Generic routine that notifies the stack about recovery actions for a fault. */
static uint32_t app_handle_fault(uint8_t port, uint32_t fault_type)
{
    uint8_t reason = PD_HARDRES_REASON_VBUS_OVP;

    if (fault_type == FAULT_TYPE_VBUS_OCP)
    {
        reason = PD_HARDRES_REASON_VBUS_OCP;
    }

    /* Not checking for validity of port or fault_type as all calls to this function are internal. */
    dpm_set_fault_active(port);

    if (gl_app_fault_count[port][fault_type] < gl_app_fault_retry_limit[port][fault_type])
    {
        dpm_clear_hard_reset_count(port);
#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
        /*
         * If current foldback mode is enabled, then we should recover from the
         * failure as the behaviour is expected. But we should still continue to
         * handle the fault with hard reset. So, we do not let the fault count 
         * to be incremented.
         */
        if ((fault_type != FAULT_TYPE_VBUS_UVP) ||
                (app_get_status(port)->cur_fb_enabled == false))
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
        {
            gl_app_fault_count[port][fault_type]++;
        }

        /*
         * Try a Hard Reset to recover from fault.
         * If not successful (not in PD contract), try Type-C error recovery.
         */
        if (dpm_send_hard_reset (port, reason) != CCG_STAT_SUCCESS)
        {
            dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
        }
    }
    else
    {
        app_conf_for_faulty_dev_removal(port);
    }

    /* Return the remaining number of retries for the current fault type. */
    return ((uint32_t)(gl_app_fault_retry_limit[port][fault_type] - gl_app_fault_count[port][fault_type]));
}

bool app_validate_configtable_offsets()
{
    uint8_t  port;

    for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
#if (defined(CCG3PA) || defined(CCG3PA2))
        if(!pd_get_ptr_pwr_tbl(port))
        {
            return false;
        }

        if(VBUS_CTRL_TYPE_P1 != pd_get_ptr_pwr_tbl(port)->fb_type)
        {
            return false;
        }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if VBUS_OVP_ENABLE
        if(!pd_get_ptr_ovp_tbl(port))
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OVP] = pd_get_ptr_ovp_tbl(port)->retry_cnt;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_OCP_ENABLE
        if(!pd_get_ptr_ocp_tbl(port))
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OCP] = pd_get_ptr_ocp_tbl(port)->retry_cnt;
#endif /* VBUS_OCP_ENABLE */

#if VBUS_UVP_ENABLE
        if(!pd_get_ptr_uvp_tbl(port))
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_UVP] = pd_get_ptr_uvp_tbl(port)->retry_cnt;
#endif /* VBUS_UVP_ENABLE */

#if VBUS_SCP_ENABLE
        if(!pd_get_ptr_scp_tbl(port))
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_SCP] = pd_get_ptr_scp_tbl(port)->retry_cnt;
#endif /* VBUS_SCP_ENABLE */

#if VCONN_OCP_ENABLE
        if(!pd_get_ptr_vconn_ocp_tbl(port))
        {
            return false;
        }

        /* No retries for VCONN OCP. */
#endif /* VCONN_OCP_ENABLE */

#if OTP_ENABLE
        if(!pd_get_ptr_otp_tbl(port))
        {
            return false;
        }

        /* No retries for OTP. */
#endif /* VBUS_OTP_ENABLE */

#if BATTERY_CHARGING_ENABLE
        if(!pd_get_ptr_chg_cfg_tbl(port))
        {
            return false;
        }
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
        if(!pd_get_ptr_bat_chg_tbl(port))
        {
            return false;
        }
#endif /* POWER_BANK */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(port)->type_a_enable)
        {
            if ((!pd_get_ptr_type_a_pwr_tbl(port)) ||
                (!pd_get_ptr_type_a_chg_cfg_tbl(port)))
            {
                return false;
            }
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (CCG_BB_ENABLE != 0)
        if(!pd_get_ptr_bb_tbl(port))
        {
            return false;
        }
#endif /* (CCG_BB_ENABLE != 0) */
    }

    return true;
}

#if OTP_ENABLE

/* Globals to keep track of OTP condition. */
static uint8_t g1_otp_therm_type[NO_OF_TYPEC_PORTS] = {APP_THERMISTOR_TYPE_ERROR};
static uint8_t gl_otp_debounce_count[NO_OF_TYPEC_PORTS] = {0};
static bool    gl_otp_debounce_active[NO_OF_TYPEC_PORTS] = {false};
static bool    gl_otp_port_disable[NO_OF_TYPEC_PORTS] = {false};

void app_otp_enable(uint8_t port)
{
    /*
     * If port is < NO_OF_TYPEC_PORTS, individual port specific data structure
     * instances are updated. Port = NO_OF_TYPEC_PORTS is used for updating
     * data strutures for alll port at 1 shot. This is used during
     * initialization.
     */
    if(NO_OF_TYPEC_PORTS == port)
    {
        /* This has been called from initialization. We need to do the data
           structure initialization for all port instances. */
        for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            gl_otp_port_disable[port]    = false;
            gl_otp_debounce_active[port] = false;
            gl_otp_debounce_count[port]  = 0;
            g1_otp_therm_type[port]      = APP_THERMISTOR_TYPE_ERROR;
        }
    }
    else
    {
        /* Initialization for port specific instance. */
        gl_otp_port_disable[port]    = false;
        gl_otp_debounce_active[port] = false;
        gl_otp_debounce_count[port]  = 0;
        g1_otp_therm_type[port]      = APP_THERMISTOR_TYPE_ERROR;
    }
}

uint16_t app_otp_get_therm_volt (uint8_t port)
{
    uint8_t level = 0;
    uint16_t therm_volt = 0;

    if (0x0 == (pd_get_ptr_otp_tbl(port)->therm_type))
    {
        /* NTC Thermistor */
        g1_otp_therm_type[port] = APP_THERMISTOR_TYPE_NTC;
    }
    else if(0x01 == ((pd_get_ptr_otp_tbl(port)->therm_type) & 0x01))
    {
        /* PTC Thermistor */
        g1_otp_therm_type[port] = APP_THERMISTOR_TYPE_PTC;
    }
    else
    {
        /* Error in configuration */
        g1_otp_therm_type[port] = APP_THERMISTOR_TYPE_ERROR;
    }

    /* Configure GPIO. */
    hsiom_set_config(OTP_THERM_GPIO, HSIOM_MODE_AMUXA);
    /* Take ADC sample. */
    level = pd_adc_sample (port, PD_ADC_ID_1, PD_ADC_INPUT_AMUX_A);
    therm_volt = pd_adc_level_to_volt (port, PD_ADC_ID_1, level);
    return therm_volt;
}

static void otp_debounce_cb(uint8_t port, timer_id_t id)
{
    uint32_t therm_volt;

    (void)id;

    /* Get thermistor voltage. */
    therm_volt = app_otp_get_therm_volt(port);

    /* If OT still exists. */
    if (
            ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_NTC) &&
             (therm_volt <= pd_get_ptr_otp_tbl(port)->cutoff_volt)) ||
            (((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_PTC) &&
              (therm_volt >= pd_get_ptr_otp_tbl(port)->cutoff_volt)))
       )
    {
        if ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
        {
            /* OT detected. Check if it has crossed configured debounce time */
            gl_otp_debounce_count[port]++;
            if(gl_otp_debounce_count[port] > pd_get_ptr_otp_tbl(port)->debounce)
            {
                /* Valid OT detected. Disable the port now. */
                dpm_stop (port);
                gl_otp_port_disable[port] = true;
                gl_otp_debounce_active[port] = false;
            }
            else
            {
                /* Start debounce timer again. */
                timer_start (port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
        else
        {
            /* OTP feature can get disabled while debouncing is in progress.
             * There is no point continuing OTP checking in this case.
             * This is equivalent to no OTP condition.
             */
            app_otp_enable (port);
        }
    }
    else
    {
        /* OT condition doesn't exist anymore. Restart OTP detection. */
        app_otp_enable (port);
    }
}

void app_otp_check_temp(uint8_t port)
{
    uint16_t therm_volt;

    /*
     * This function will be called in a polling fashion after every expiry of
     * activity timer. Proceed and do necessary steps only if OTP protection
     * is enabled for this PD port.
     */
    if ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
    {
        /* Check thermistor voltage and see if it is below cut off voltage. */
        therm_volt = app_otp_get_therm_volt(port);

        /* If port is disabled then look for restart voltage. */
        if (gl_otp_port_disable[port] == true)
        {
            if (
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_NTC) &&
                     (therm_volt >= pd_get_ptr_otp_tbl(port)->restart_volt)) ||
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_PTC) &&
                     (therm_volt <= pd_get_ptr_otp_tbl(port)->restart_volt))
               )
            {
                /* OT condition doesn't exist anymore. Re-enable the port. */
                dpm_start(port);
                /* Reset the port specific data structures. */
                app_otp_enable(port);
            }
        }
        /* If port not disabled, look for cut off voltage. */
        else if (gl_otp_debounce_active[port] == false)
        {
            /*
             * Compare therm volt and cut off voltage and start OT debounce,
             * if required. Thermistor type value will decide the comparison
             * algorithm.
             */
            if (
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_NTC) &&
                     (therm_volt <= pd_get_ptr_otp_tbl(port)->cutoff_volt)) ||
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_PTC) &&
                     (therm_volt >= pd_get_ptr_otp_tbl(port)->cutoff_volt))
               )
            {
                gl_otp_debounce_active[port] = true;
                gl_otp_debounce_count[port] = 0;
                /* Start OTP debounce timer. */
                timer_start (port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
    }
}
#endif /* OTP_ENABLE */

#ifdef CCG3PA

bool ccg_app_is_idle(void)
{
    /*
     * If activity timer timeout event is not pending, CCG is idle and system can
     * enter low power mode.
     */
    return !ccg_activity_timer_timeout;
}

void ccg_activity_timer_cb(uint8_t instance, timer_id_t id)
{
    (void)instance;
    (void)id;
    /*
     * Activity timer expired. Generate an event so that CCG periodic checks
     * can run.
     */
    ccg_activity_timer_timeout = true;
}

void ccg_app_task(uint8_t port)
{
    /* Check VBATT, OTP and TYPE-A current consumption if activity timer has timed out. */
    if (ccg_activity_timer_timeout == true)
    {
#if (POWER_BANK == 1)
        pb_bat_monitor ();
#endif /* (POWER_BANK == 1) */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(0)->type_a_enable)
        {
            type_a_detect_disconnect ();
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (OTP_ENABLE == 1)
        app_otp_check_temp (port);
#endif /* OTP_ENABLE */

#if (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1)
        ccg_activity_timer_timeout = false;
        timer_start (0, CCG_ACTIVITY_TIMER_ID, CCG_ACTIVITY_TIMER_PERIOD,
                ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ ENABLE == 1) */
    }
}

void ccg_app_task_init(void)
{
#if (POWER_BANK == 1)
    pb_task_init ();
#endif /* (POWER_BANK == 1) */

#if OTP_ENABLE
    /* Enable OTP. */
    app_otp_enable (NO_OF_TYPEC_PORTS);
#endif /* OTP_ENABLE */

    /*
     * Start CCG activity timer. This timer is periodically used to monitor
     * battrey voltage (in power bank application), TYPE-A current consumption
     * and OTP.
     */
#if (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1)
    ccg_activity_timer_timeout = false;
    timer_start (0, CCG_ACTIVITY_TIMER_ID, CCG_ACTIVITY_TIMER_PERIOD,
            ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) */
}

#endif /* CCG3PA */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static bool app_is_vdm_task_ready(uint8_t port)
{
    /* Assume cable discovery finished when device is UFP. */
    bool retval = true;

#if DFP_ALT_MODE_SUPP

    const dpm_status_t *dpm_stat = dpm_get_info (port);

    /* This check only makes sense for DFP. */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        /*
         * Set the cable discovered flag if:
         * 1. Cable discovery is disabled.
         * 2. EMCA present flag in DPM is set.
         */
        if ((dpm_stat->cbl_dsc == false) || (dpm_stat->emca_present != false))
        {
            app_get_status(port)->cbl_disc_id_finished = true;
        }

        /* Return the status of Cable discovered flag. */
        retval = app_get_status(port)->cbl_disc_id_finished;
    }

#endif /* DFP_ALT_MODE_SUPP */

    return retval;
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

/* Timer used to re-enable the PD port after a fault. */
static void fault_recovery_timer_cb(uint8_t port, timer_id_t id)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;

    if (
            (vbus_is_present(port, VSAFE_0V, 0) == false)
       )
    {
        if ((app_status[port].fault_status & APP_PORT_VBUS_DROP_WAIT_ACTIVE) != 0)
        {
            app_status[port].fault_status &= ~APP_PORT_VBUS_DROP_WAIT_ACTIVE;

            /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
            pd_typec_rd_enable (port);
            period = APP_FAULT_RECOVERY_MAX_WAIT;
        }
        else
        {
            /*
             * If VBus is not detected, we can re-enable the PD port.
             */
            app_status[port].fault_status &= ~APP_PORT_DISABLE_IN_PROGRESS;
            dpm_clear_fault_active(port);

            pd_typec_dis_rd(port, CC_CHANNEL_1);
            pd_typec_dis_rd(port, CC_CHANNEL_2);
            dpm_start(port);

            /* Return without restarting the timer. */
            return;
        }
    }

    /* Restart the timer to check VBus and Rp status again. */
    timer_start (port, APP_FAULT_RECOVERY_TIMER, period, fault_recovery_timer_cb);
}

/* Callback used to get notification that PD port disable has been completed. */
static void app_port_disable_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;

    if (
            (vbus_is_present(port, VSAFE_0V, 0) == false)
       )
    {
        /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
        pd_typec_rd_enable (port);
        period = APP_FAULT_RECOVERY_MAX_WAIT;
    }
    else
    {
        /* VBus has not been removed. Start a task which waits for VBus removal. */
        app_status[port].fault_status |= APP_PORT_VBUS_DROP_WAIT_ACTIVE;
    }

    /* Provide a delay to allow VBus turn-on by port partner and then enable the port. */
    timer_start (port, APP_FAULT_RECOVERY_TIMER, period, fault_recovery_timer_cb);
}

ccg_status_t app_disable_pd_port(uint8_t port, dpm_typec_cmd_cbk_t cbk)
{
    ccg_status_t retval = CCG_STAT_SUCCESS;

    if (timer_is_running (port, APP_FAULT_RECOVERY_TIMER))
    {
        /* If the HPI Master is asking us to disable the port, make sure all fault protection state is cleared. */
        app_status[port].fault_status &= ~(
                APP_PORT_VBUS_DROP_WAIT_ACTIVE | APP_PORT_SINK_FAULT_ACTIVE | APP_PORT_DISABLE_IN_PROGRESS);
        timer_stop(port, APP_FAULT_RECOVERY_TIMER);
        pd_typec_dis_rd(port, CC_CHANNEL_1);
        pd_typec_dis_rd(port, CC_CHANNEL_2);
        cbk(port, DPM_RESP_SUCCESS);
    }
    else
    {
        /* Just pass the call on-to the stack. */
        if (dpm_get_info(port)->dpm_enabled)
        {
            retval = dpm_typec_command(port, DPM_CMD_PORT_DISABLE, cbk);
        }
        else
        {
            cbk(port, DPM_RESP_SUCCESS);
        }
    }

    return retval;
}

uint8_t app_task(uint8_t port)
{
    /*
     * If SINK fault handling is pending, queue a port disable command.
     */
    if ((app_status[port].fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
    {
        if (dpm_typec_command (port, DPM_CMD_PORT_DISABLE, app_port_disable_cb) != CCG_STAT_BUSY)
        {
            app_status[port].fault_status &= ~APP_PORT_SINK_FAULT_ACTIVE;
            app_status[port].fault_status |= APP_PORT_DISABLE_IN_PROGRESS;
        }
    }

#if BC_1_2_SRC_ENABLE
    /* Perform any CDP state machine tasks required. */
    ccg_bc_cdp_sm(port);
#endif /* BC_1_2_SRC_ENABLE */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* If VDM processing is allowed */
    if (app_get_status(port)->vdm_task_en != false)
    {
        /* Wait for cable discovery completion before going on Alt. Modes. */
        if (app_is_vdm_task_ready (port))
        {
            vdm_task_mngr (port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (CCG_BB_ENABLE != 0)
    if (bb_is_present(port) != false)
    {
        bb_task(port);
    }
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
    uint8_t i;
    for (i=0; i < NO_OF_BC_PORTS; i++)
    {
        bc_fsm (i);
    }
#endif /* BATTERY_CHARGING_ENABLE */

#if RIDGE_SLAVE_ENABLE
    ridge_slave_task();
#endif /* RIDGE_SLAVE_ENABLE */

#ifdef CCG3PA
    /* Run polling tasks of CCG. */
    ccg_app_task (port);
#endif /* CCG3PA */

    return true;
}

#define AMUX_CTRL_EA_TOP_REFGEN_SEL7_EN     (0x02u);

void app_type_c_enter_sleep(uint8_t port)
{
#if ((defined(CCG3PA)) || (defined(CCG3PA2)))
    /*
     * Configure Refgen block to use Deepsleep Reference input instead of Bandgap
     * reference which is not available in deep sleep.
     */
    PDSS->refgen_0_ctrl &= ~(PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
        PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL);

    /* Switch to using refgen_2_ctrl SEL7 for EA shunt regulator reference. */
    PDSS->amux_ctrl |= AMUX_CTRL_EA_TOP_REFGEN_SEL7_EN;
#endif /* ((defined(CCG3PA)) || (defined(CCG3PA2))) */
}

void app_type_c_wakeup()
{
#if ((defined(CCG3PA)) || (defined(CCG3PA2)))
    /* Switch to using bandgap for EA shunt regulator reference. */
    PDSS->amux_ctrl &= ~AMUX_CTRL_EA_TOP_REFGEN_SEL7_EN;
    /*
     * Configure Refgen block to use Bandgap Reference input instead of Deep sleep
     * reference.
     */
    PDSS->refgen_0_ctrl |= (PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
        PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL);
#endif /* ((defined(CCG3PA)) || (defined(CCG3PA2))) */
}

bool app_type_c_sleep_allowed(void)
{
    bool out = true;
    uint8_t i = 0;

#if (defined(CCG3PA) || defined(CCG3PA2))
    const dpm_status_t *dpm;

    /*
     * Deepsleep mode operation can only be supported when un-attached.
     * When attached, the references and the CSA block requires to be
     * active.
     */
    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        dpm = dpm_get_info(i);

        if (dpm->attach == true)
        {
            out = false;
            break;
        }
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    if (out == true)
    {
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
        {
            app_type_c_enter_sleep (i);
        }
    }

    return out;
}

bool app_sleep(void)
{
    bool stat = true;
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_BB_ENABLE
        if (!bb_enter_deep_sleep(port))
        {
            stat = false;
            break;
        }
#endif /* CCG_BB_ENABLE */

#ifdef CCG3PA
        /*
         * Check if CCG polling tasks are not pending to be serviced and system can enter
         * low power mode.
         */
        if (ccg_app_is_idle () == false)
        {
            stat = false;
            break;
        }
#endif /* CCG3PA */

        /* Don't go to sleep while CC/SBU fault handling is pending. */
        if ((app_status[port].fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
        {
            stat = false;
            break;
        }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        if (!is_vdm_task_idle(port))
        {
            stat = false;
            break;
        }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (DP_UFP_SUPP) && (CCG_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Check if HPD RX Activity timer is running.
         * If yes, don't enter deep sleep. */
        if (!is_hpd_rx_state_idle (port))
        {
            stat = false;
            break;
        }
#endif /* DP_UFP_SUPP && CCG_HPD_RX_ENABLE */

#if ((defined(CCG5)) && (BC_1_2_SRC_ENABLE))
        /* Cannot go to sleep while we have VDM_SRC enabled. */
        if (ccg_is_cdp_sm_busy (port))
        {
            stat = false;
            break;
        }
#endif /* ((defined(CCG5)) && (BC_1_2_SRC_ENABLE)) */
    }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    if (stat)
    {
        for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
        {
            /* Prepare for deep-sleep entry. */
            alt_mode_mngr_sleep(port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

    return stat;
}

void app_wakeup(void)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        alt_mode_mngr_wakeup (port);
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

#if (CCG_BB_ENABLE != 0)
/* Alternate mode entry timeout callback function. */
static void ame_tmr_cbk(uint8_t port, timer_id_t id)
{
    (void)id;

    /* Alternate modes are reset in vdm_task_mngr_deinit(). */
    bb_enable(port, BB_CAUSE_AME_TIMEOUT);
}
#endif /* (CCG_BB_ENABLE != 0) */

#if CCG_PD_REV3_ENABLE

void extd_msg_cb(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr)
{
    static pd_ams_type ams_type[NO_OF_TYPEC_PORTS];
    (void)pkt_ptr;
    if(resp == RES_RCVD){
        dpm_set_chunk_transfer_running(port, ams_type[port]);
    }
    if(resp == CMD_SENT){
        ams_type[port] = dpm_get_info(port)->non_intr_response;
    }
}

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

static void app_extd_msg_handler(uint8_t port, pd_packet_extd_t *pd_pkt_p)
{
    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.data_size >
               ((pd_pkt_p->hdr.hdr.chunk_no + 1) * MAX_EXTD_MSG_LEGACY_LEN)))
    {
        dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmd_sop = pd_pkt_p->sop;
        extd_dpm_buf.extd_type = pd_pkt_p->msg;
        extd_dpm_buf.extd_hdr.val = 0;
        extd_dpm_buf.extd_hdr.extd.chunked = true;
        extd_dpm_buf.extd_hdr.extd.request = true;
        extd_dpm_buf.extd_hdr.extd.chunk_no = pd_pkt_p->hdr.hdr.chunk_no + 1;
        extd_dpm_buf.dat_ptr = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;

        /* Send next chunk request */
        dpm_pd_command_ec(port, DPM_CMD_SEND_EXTENDED,
                &extd_dpm_buf, extd_msg_cb);
    }
    else
    {
        /* Send Not supported message */
        dpm_pd_command_ec(port, DPM_CMD_SEND_NOT_SUPPORTED, NULL, NULL);
    }
}
#endif /* CCG_PD_REV3_ENABLE */

/* This function stops PD operation and configures type c
 * to look for detach of faulty device */
void app_conf_for_faulty_dev_removal(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    if ((!dpm_stat->attach) || (dpm_stat->cur_port_role == PRT_ROLE_SINK))
    {
        /* Set flag to trigger port disable sequence. */
        app_status[port].fault_status |= APP_PORT_SINK_FAULT_ACTIVE;
    }

    /* Stop PE */
    dpm_pe_stop(port);

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Make sure any alternate mode related state is cleared. */
    vdm_task_mngr_deinit (port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

void app_update_bc_src_support(uint8_t port, uint8_t enable)
{
#if BC_1_2_SRC_ENABLE
    app_status[port].bc_12_src_disabled = (bool)(!enable);
    if (!enable)
    {
        ccg_bc_dis(port);
    }
#endif /* BC_1_2_SRC_ENABLE */
}

void app_update_sys_pwr_state(uint8_t state)
{
#if BC_1_2_SRC_ENABLE
    uint8_t i = 0;

    /* System power state change and BC 1.2 is enabled. */
    if (
            (state != hpi_get_sys_pwr_state()) &&
            (app_status[i].bc_12_src_disabled == 0)
       )
    {
        /*
         * Do Type-C error recovery on any ports where the CCGx device is power source and alternate modes are not active.
         * This will allow BC 1.2 state update (CDP->DCP and DCP->CDP) to be detected by the sink.
         */
#if CCG_PD_DUALPORT_ENABLE
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
#endif /* CCG_PD_DUALPORT_ENABLE */
        {
            if (
                    (dpm_get_info(i)->attach) &&
                    (dpm_get_info(i)->cur_port_role == PRT_ROLE_SOURCE) &&
                    ((alt_mode_get_status(i) & 0x7F) == 0)
               )
            {
                dpm_typec_command (i, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
            }
        }
    }
#endif /* BC_1_2_SRC_ENABLE */
}

#if BC_1_2_SRC_ENABLE

/* Function to start the CCG5 BC 1.2 source state machine. */
static void app_bc_12_sm_start(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /* Start CDP state machine if enabled and we are source. */
    if ((dpm_stat->cur_port_role == PRT_ROLE_SOURCE) && (!app_status[port].bc_12_src_disabled))
    {
        /* Enable CDP is system power state is S0, otherwise enable DCP. */
#if CCG_HPI_ENABLE
        if (hpi_get_sys_pwr_state () == 0)
#endif /* CCG_HPI_ENABLE */
        {
            ccg_bc_cdp_en(port);
        }
#if CCG_HPI_ENABLE
        else
        {
            ccg_bc_dcp_en(port);
        }
#endif /* CCG_HPI_ENABLE */
    }
}

#endif /* BC_1_2_SRC_ENABLE */

uint32_t get_bat_status[NO_OF_TYPEC_PORTS];

#ifdef CCG5
static void src_disable_cbk(uint8_t port)
{
    /* Dummy callback used to ensure VBus discharge happens on CC/SBU OVP. */
}
#endif /* CCG5 */

static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

void app_event_handler(uint8_t port, app_evt_t evt, const void* dat)
{
    const app_req_status_t* result;
    const pd_contract_info_t* contract_status;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint32_t fault_arg = 0;
#if CCG_PD_REV3_ENABLE
    pd_do_t alert_ado;
#endif /* CCG_PD_REV3_ENABLE */

    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
#if (!(CCG_SOURCE_ONLY))
#ifdef CCG3
            /* Sink FET need to be enabled in dead battery to power CCG3 board. */
            if(dpm_stat->dead_bat == true)
            {
                psnk_enable(port);
            }
#endif /* CCG3 */
#endif /* (!(CCG_SOURCE_ONLY)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
#if (POWER_BANK == 1)
            /*
             * In power bank case, if device is powered by external VDDD (i.e not
             * in dead battery), disable internal VBUS regulator.
             */
            if (dpm_stat->dead_bat == false)
            {
                pd_hal_disable_vreg (port);
            }
#endif /* (POWER_BANK == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

            /* Initialize the MUX to its default settings (isolate). */
            mux_ctrl_init (port);
            app_get_status(port)->vdm_prcs_failed = false;
            break;

        case APP_EVT_TYPEC_ATTACH:
            /* This will also enable the USB (DP/DM) MUX where required. */
            set_mux (port, MUX_CONFIG_SS_ONLY, 0);

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (dpm_stat->polarity != gl_app_previous_polarity[port])
            {
                app_clear_fault_counters(port);
            }
#endif /* (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE) */
            gl_app_previous_polarity[port] = dpm_stat->polarity;

#if BC_1_2_SRC_ENABLE
            /* Start the BC 1.2 state machine where applicable. */
            app_bc_12_sm_start(port);
#endif /* BC_1_2_SRC_ENABLE */
            break;

        case APP_EVT_CONNECT:
            app_get_status(port)->vdm_prcs_failed = false;
            app_get_status(port)->cbl_disc_id_finished = false;

#if (CCG_BB_ENABLE != 0)
            /* Enable the AME timer on attach if in sink mode. */
            if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
            {
                timer_start(port, APP_AME_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
            }
#endif /* (CCG_BB_ENABLE != 0) */
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            hardreset_cplt = true;
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
            typec_only = ((dpm_stat->pd_connected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:

#if (defined(CCG3PA) || defined(CCG3PA2))
#if (POWER_BANK == 1)
            /*
             * In power bank case, if device is powered by external VDDD, VDDD gets
             * shorted to VBUS_IN line. This shall result connecting VDDD to the
             * Type-C VBUS line. This also includes cases where we start as dead
             * dead battery device and then get charged. So if any time VBUS has to
             * be removed in course of PD / Type-C state machine, ensure that internal
             * VBUS regulator is disabled. In event of dead battery, this shall lead
             * to device reset. This is the safest recovery path. CDT 276535.
             *
             * This code can be removed if the VBATT monitoring can be done
             * continously. But this code can still be in place to avoid any
             * corner case handling.
             */

            /* Do this only on disconnect and type-C error recovery. */
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_TYPE_C_ERROR_RECOVERY))
            {
                pd_hal_disable_vreg(port);
            }
#endif /* (POWER_BANK == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            vdm_task_mngr_deinit (port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

            /*
             * Re-enable MUX in USB mode if hard reset has been completed.
             */
            if (hardreset_cplt)
            {
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            }
            else
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
                    set_mux (port, MUX_CONFIG_ISOLATE, 0);
                    timer_stop (port, APP_AME_TIMEOUT_TIMER);
                }
            }

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(port))
                {
                    if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                    {
                        dpm_clear_fault_active(port);
                    }
                }
            }
#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
                /* Cleanup the PD block states on disconnect. */
                pd_hal_cleanup(port);

                if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                {
                    dpm_clear_fault_active(port);
                }

        #if VCONN_OCP_ENABLE
                /* Clear the VConn fault status. */
                app_status[port].fault_status &= ~APP_PORT_VCONN_FAULT_ACTIVE;
        #endif /* VCONN_OCP_ENABLE */

        #if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
                app_clear_fault_counters(port);
        #endif /* (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE) */

        #if BC_1_2_SRC_ENABLE
                ccg_bc_dis(port);
        #endif /* BC_1_2_SRC_ENABLE */

        #if RIDGE_SLAVE_ENABLE
                /* Clear the error status. */
                ridge_slave_update_ocp_status(port, false);
        #endif /* RIDGE_SLAVE_ENABLE */
            }

            /* Disconnect and Port Disable events are handled above. */
            if (evt == APP_EVT_HARD_RESET_SENT)
            {
                /* Ensure Fault active condition is cleared, if one was detected. */
                if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                {
                    dpm_clear_fault_active(port);
                }
            }
            break;

        case APP_EVT_EMCA_DETECTED:
        case APP_EVT_EMCA_NOT_DETECTED:
            app_get_status(port)->cbl_disc_id_finished = true;
            app_get_status(port)->vdm_prcs_failed = false;
            break;

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const app_req_status_t*)dat ;
            if(*result == REQ_ACCEPT)
            {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                vdm_task_mngr_deinit (port);

                /* Device data role changed. Enable the VDM task manager for alt. mode support. */
                enable_vdm_task_mngr(port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (CCG_BB_ENABLE != 0)
                /* Start tAME Timer to enable BB functionality */
                if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                {
                    timer_start(port, APP_AME_TIMEOUT_TIMER , APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
                }
                else
                {
                    timer_stop (port, APP_AME_TIMEOUT_TIMER);
                }
#endif /* (CCG_BB_ENABLE != 0) */
            }
            break;

        case APP_EVT_VENDOR_RESPONSE_TIMEOUT:
            /* If the APP layer is going to retry the VDM, do not send the event. */
            if (app_status[port].vdm_retry_pending)
                skip_soln_cb = true;
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            /* Set VDM version based on active PD revision. */
#if CCG_PD_REV3_ENABLE
            if (dpm_stat->spec_rev_sop_live >= PD_REV3)
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV3;
            }
            else
#endif /* CCG_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV2;
            }

            contract_status = (pd_contract_info_t*)dat;
            if ((contract_status->status == PD_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == PD_CONTRACT_CAP_MISMATCH_DETECTED))
            {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                /*
                 * Contract established.  Enable VDM task manager for Alt. Mode support.
                 * This function will have no effect if the Alt. Modes are already running.
                 */
                if (
                        (gl_dpm_port_type[port] == PRT_TYPE_UFP) ||
                        (app_get_status(port)->vdm_prcs_failed == false)
                   )
                {
                    enable_vdm_task_mngr(port);
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
            }

#if (CCG_BB_ENABLE != 0)
            if (
                    (contract_status->status != PD_CONTRACT_NEGOTIATION_SUCCESSFUL) &&
                    (dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK) &&
                    (gl_dpm_port_type[port] == PRT_TYPE_UFP)
               )
            {
                bb_enable(port, BB_CAUSE_PWR_FAILURE);
            }
#endif /* (CCG_BB_ENABLE != 0) */

#if BC_1_2_SRC_ENABLE
#if (!CCG_BC_12_IN_PD_ENABLE)
            ccg_bc_dis (port);
#endif /* (!CCG_BC_12_IN_PD_ENABLE) */
#endif /* BC_1_2_SRC_ENABLE */
            break;

        case APP_EVT_VBUS_OCP_FAULT:
#if VBUS_OCP_ENABLE

    #if RIDGE_SLAVE_ENABLE
            /* Update the OCP status. */
            ridge_slave_update_ocp_status(port, true);
    #endif /* RIDGE_SLAVE_ENABLE */

            /* Let the application know the current retry state of the port. */
            fault_arg = app_handle_fault(port, FAULT_TYPE_VBUS_OCP);
            dat = &fault_arg;
#endif /* VBUS_OCP_ENABLE */
            break;

        case APP_EVT_VBUS_SCP_FAULT:
#if VBUS_SCP_ENABLE
            /* Let the application know the current retry state of the port. */
            fault_arg = app_handle_fault(port, FAULT_TYPE_VBUS_SCP);
            dat = &fault_arg;
#endif /* VBUS_SCP_ENABLE */
            break;

        case APP_EVT_VBUS_OVP_FAULT:
#if VBUS_OVP_ENABLE
            /* Let the application know the current retry state of the port. */
            fault_arg = app_handle_fault(port, FAULT_TYPE_VBUS_OVP);
            dat = &fault_arg;
#endif /* VBUS_OVP_ENABLE */
            break;

        case APP_EVT_VBUS_UVP_FAULT:
#if VBUS_UVP_ENABLE
            /* Let the application know the current retry state of the port. */
            fault_arg = app_handle_fault(port, FAULT_TYPE_VBUS_UVP);
            dat = &fault_arg;
#endif /* VBUS_UVP_ENABLE */
            break;

        case APP_EVT_VCONN_OCP_FAULT:
#if VCONN_OCP_ENABLE
            /* Store the VConn fault status. */
            app_status[port].fault_status |= APP_PORT_VCONN_FAULT_ACTIVE;

            /* Exit any active alternate modes. */
            if (gl_dpm_port_type[port] == PRT_TYPE_DFP)
            {
                alt_mode_mngr_exit_all (port);
            }
#endif /* VCONN_OCP_ENABLE */
            break;

#if CCG_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
#if (CCG_HPI_PD_ENABLE)
            /* Handle the extended message locally if forwarding to EC is not enabled. */
            if (hpi_is_extd_msg_ec_ctrl_enabled (port) == false)
#endif
            {
                app_extd_msg_handler(port, (pd_packet_extd_t *)dat);
            }
            skip_soln_cb  = true;
            break;
        case  APP_EVT_ALERT_RECEIVED:
            /* Respond to ALERT message only if there is the number of object is one. */
            if (((pd_packet_t*)dat)->len == 1)
            {
                alert_ado = ((pd_packet_t*)dat)->dat[0];
                if(alert_ado.ado_alert.bat_status_change == false)
                {
                    dpm_pd_command(port, DPM_CMD_GET_STATUS, NULL, NULL);
                }
                else
                {
                    uint8_t i = alert_ado.ado_alert.fixed_bats |
                        (alert_ado.ado_alert.hot_swap_bats << 4);
                    dpm_pd_cmd_buf_t cmd;

                    /* Identify the first battery for which the change is intended. */
                    get_bat_status[port] = 0;
                    while ((i != 0) && ((i & 0x01) == 0))
                    {
                        get_bat_status[port]++;
                        i >>= 1;
                    }

                    cmd.cmd_sop = SOP;
                    cmd.extd_hdr.val = 0x1;
                    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
                    cmd.extd_type = EXTD_MSG_GET_BAT_STATUS;
                    cmd.dat_ptr = (uint8_t*)&get_bat_status[port];
                    dpm_pd_command(port, DPM_CMD_SEND_EXTENDED, &cmd, NULL);
                }
            }
            break;
#endif /* CCG_PD_REV3_ENABLE */

#if REGULATOR_REQUIRE_STABLE_ON_TIME
    case APP_EVT_TYPEC_ATTACH_WAIT:
        REGULATOR_ENABLE();
        break;
    case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
        REGULATOR_DISABLE();
        break;
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

#ifdef CCG5
    case APP_EVT_CC_OVP:
    case APP_EVT_SBU_OVP:
        {
            /* Make sure SOURCE/SINK FETs and VConn supply are turned OFF. */
            vconn_disable(port, dpm_stat->rev_pol);
            if ((dpm_stat->attach) && (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
            {
                /* Remove the Rp termination and notify the HAL that OVP is pending. */
                pd_typec_dis_rp(port, dpm_stat->polarity);
                pd_hal_set_cc_ovp_pending(port);

                psrc_disable(port, src_disable_cbk);
            }
            else
            {
#if CCG_HW_DRP_TOGGLE_ENABLE
                /* Abort auto toggle if enabled. */
                pd_hal_abort_auto_toggle(port);
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */
#if (!(CCG_SOURCE_ONLY))
                psnk_disable(port, 0);
#endif /* (!(CCG_SOURCE_ONLY)) */
            }

            /* No need to take new action as long as previous fault handling is still pending. */
            if ((app_status[port].fault_status & (APP_PORT_SINK_FAULT_ACTIVE | APP_PORT_DISABLE_IN_PROGRESS)) == 0)
            {
                app_handle_fault(port, FAULT_TYPE_CC_OVP);
            }
            else
            {
                skip_soln_cb = true;
            }

            dat = &fault_arg;
        }
        break;
#endif /* CCG5 */

#if 0
        /* Default handlers are sufficient for these cases. */
        case APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS:
        case APP_EVT_RP_CHANGE:
        case APP_EVT_PKT_RCVD:
        case APP_EVT_PR_SWAP_COMPLETE:
        case APP_EVT_VCONN_SWAP_COMPLETE:
        case APP_EVT_SENDER_RESPONSE_TIMEOUT:
        case APP_EVT_SOFT_RESET_SENT:
        case APP_EVT_CBL_RESET_SENT:
#endif
        default:
            /* Nothing to do. */
            break;
    }

#if BATTERY_CHARGING_ENABLE
    bc_pd_event_handler(port,evt);
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
    pb_event_handler (port, evt);
#endif /* POWER_BANK */

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
        sln_pd_event_handler(port, evt, dat);
    }
}

app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].app_resp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

#ifdef CCG5
/* Callback used to receive fault notification from the HAL and to pass it on to the event handler. */
static void app_cc_sbu_fault_handler(uint8_t port, bool fault_type)
{
    app_event_handler(port, (fault_type ? APP_EVT_SBU_OVP : APP_EVT_CC_OVP), 0);
}
#endif /* CCG5 */

void app_init(void)
{
    uint8_t port;

#ifdef CCG5
    /* Register a callback for notification of CC/SBU faults. */
    ccg_set_fault_cb(app_cc_sbu_fault_handler);
#endif /* CCG5 */

    /* For now, only the VDM handlers require an init call. */
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        vdm_data_init(port);

#if (CCG_BB_ENABLE != 0)
        /*
         * Initialize the billboard interface. The billboard
         * interface shall not get initialized if it is not
         * enabled in configuration table.
         */
        bb_init(port);
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
        bc_init(port);
#endif /* BATTERY_CHARGING_ENABLE */
    }

    /* For systems with TYPEA port. */
#if (CCG_TYPE_A_PORT_ENABLE == 1)
    if (get_pd_port_config(0)->type_a_enable)
    {
        type_a_port_enable();
    }
    else
    {
        /* Ensure that the state machine variables are initialized correctly. */
        type_a_port_disable();
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */
}

/* Implements CCG deep sleep functionality for power saving. */
bool system_sleep(void)
{
    uint8_t intr_state;
    bool dpm_slept = false;
    bool app_slept = false;
    bool app_type_c_slept = false;
    bool retval = false;
#if BATTERY_CHARGING_ENABLE
    bool bc_slept = false;
#endif /* BATTERY_CHARGING_ENABLE */

    intr_state = CyEnterCriticalSection();

    /*
     * We have to check the application layer, HPI and the Device Policy
     * Manager (DPM) to see if all of these modules are ready for sleep.
     * CCG can only enter deep sleep if all of these blocks are in an idle
     * state.
     *
     * Note: The respective sleep functions might be performing some
     * state updates as part of the idle check function; and therefore
     * the corresponding wakeup function needs to be called if they have
     * returned true to indicate that sleep is allowed.
     */
    if (app_sleep())
    {
        app_slept = true;

#if BATTERY_CHARGING_ENABLE
        if(bc_sleep() == true)
        {
            bc_slept = true;
#endif /* BATTERY_CHARGING_ENABLE */

            if (
#if CCG_HPI_ENABLE
                    (hpi_sleep_allowed()) &&
#endif /* CCG_HPI_ENABLE */

#if CCG_UCSI_ENABLE
                    (ucsi_sleep_allowed()) &&
#endif /* CCG_UCSI_ENABLE */    
                    (dpm_deepsleep())
               )
            {
                dpm_slept = true;
                /*
                 * Check connection status of TYPE-A and TYPE-C ports to determine if deepsleep
                 * entry is allowed. If not,enter sleep mode to save power.
                 */
                if (
    #if CCG_TYPE_A_PORT_ENABLE
                    (type_a_is_idle() == true) &&
    #endif /* CCG_TYPE_A_PORT_ENABLE */
                    (app_type_c_sleep_allowed() == true)
                    )
                {
                    app_type_c_slept = true;
                    timer_enter_sleep();

                    /*
                     * CDT 224642: The I2C IDLE check needs to be done as the last step
                     * before device enters into sleep. Otherwise, the device may fail
                     * to wake up when there is an address match on the I2C interface.
                     */
#if ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE))
                    if (
    #if CCG_HPI_ENABLE
                            (hpi_sleep())
    #else
                            (1)
    #endif /* CCG_HPI_ENABLE */
                            &&
    #if RIDGE_SLAVE_ENABLE
                            (ridge_slave_sleep())
    #else
                            (1)
    #endif /* RIDGE_SLAVE_ENABLE */
                       )
#endif /* ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE)) */
                    {
                        /* Device sleep entry. */
                        CySysPmDeepSleep();
                        retval = true;
                    }
                }
                else
                {
#if (defined(CCG3PA) || defined(CCG3PA2))
                    /* Enter Sleep mode to save power. */
                    CySysPmSleep();
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
                }
            }
#if BATTERY_CHARGING_ENABLE
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

    CyExitCriticalSection(intr_state);

    if (app_type_c_slept)
    {
        app_type_c_wakeup();
    }

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if(dpm_slept)
    {
        dpm_wakeup();
    }

    /* Call app_wakeup() if app_sleep() had returned true. */
    if(app_slept)
    {
        app_wakeup();
    }

#if BATTERY_CHARGING_ENABLE
    if(bc_slept)
    {
        bc_wakeup();
    }
#endif /* BATTERY_CHARGING_ENABLE */

    return retval;
}

#if VCONN_OCP_ENABLE
void app_vconn_ocp_cbk(uint8_t port, bool comp_out)
{
    /* Disable VConn since we hit a fault. */
    vconn_disable(port, dpm_get_info(port)->rev_pol);

    /* Notify application layer about fault. */
    app_event_handler(port, APP_EVT_VCONN_OCP_FAULT, NULL);
}
#endif /* VCONN_OCP_ENABLE */

void vconn_enable(uint8_t port, uint8_t channel)
{
#if VCONN_OCP_ENABLE

    /* Do not attempt to enable VConn if fault was detected in present connection. */
    if ((app_status[port].fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
    {
        return;
    }

#if CCG_VCONN_MON_WITH_ADC

    /* If Vconn current is being monitored through ADC, configure the associated IOs. */
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P1,APP_VCONN_MON_AMUX_INPUT_P1);
#if CCG_PD_DUALPORT_ENABLE
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P2,APP_VCONN_MON_AMUX_INPUT_P2);
#endif /* CCG_PD_DUALPORT_ENABLE */

    /*
     * 120us delay required as a settling time after HSIOM config to get a stable
     * ADC reading.
     */
    CyDelayUs(120);

#endif /* CCG_VCONN_MON_WITH_ADC */

    system_vconn_ocp_en(port, app_vconn_ocp_cbk);
#endif /* VCONN_OCP_ENABLE */

    pd_vconn_enable(port, channel);

    /* Reset RX Protocol for cable */
    dpm_prot_reset_rx(port, SOP_PRIME);
    dpm_prot_reset_rx(port, SOP_DPRIME);
}

void vconn_disable(uint8_t port, uint8_t channel)
{
    pd_vconn_disable(port, channel);

#if VCONN_OCP_ENABLE
    system_vconn_ocp_dis(port);
#endif /* VCONN_OCP_ENABLE */
}

bool vconn_is_present(uint8_t port)
{
    return pd_is_vconn_present( port, dpm_get_info(port)->rev_pol);
}

bool vbus_is_present(uint8_t port, uint16_t volt, int8 per)
{
    uint8_t level;
    uint8_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling.
     * To avoid false output on OVP Trip pin when VBUS is polled
     * OVP trip pin is disconnected from OVP comp output and last
     * value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4*/

    /*
     * Re-run caliberation every time to ensure that VDDD or the measurement
     * does not break.
     */
    pd_adc_calibrate (port, APP_VBUS_POLL_ADC_ID);
    level = pd_get_vbus_adc_level(port, APP_VBUS_POLL_ADC_ID, volt, per);
    retVal = pd_adc_comparator_sample (port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT, level);

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

uint16_t vbus_get_value(uint8_t port)
{
    uint16_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling. To avoid false output
     * on OVP Trip pin when VBUS is polled OVP trip pin is disconnected from
     * OVP comp output and last value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4 */

    /* Measure the actual VBUS voltage. */
    retVal = pd_hal_measure_vbus(port);

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

#if VCONN_OCP_ENABLE
bool system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk)
{
    if (cbk == NULL)
    {
        return false;
    }

#ifdef CCG5

    /* Request the HAL to enable OCP detection with the appropriate debounce period. */
    pd_hal_vconn_ocp_enable(port, pd_get_ptr_vconn_ocp_tbl(port)->debounce, cbk);

#else /* !CCG5 */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr_1_cfg |= CYVAL_USBPD_V5V_CFG_POS_EDG_DIS_NEG_EDG_EN << PDSS_INTR_1_CFG_V5V_CFG_POS;
    pd->intr1_mask |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Configure ADC to detect VConn Over-Current condition. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, APP_VCONN_OCP_ADC_INPUT,
            APP_VCONN_TRIGGER_LEVEL, PD_ADC_INT_RISING, cbk);
#endif /* CCG4_DOCK */
#endif /* CCG5 */
    return true;
}

void system_vconn_ocp_dis(uint8_t port)
{
#ifdef CCG5

    /* Disable VConn OCP detection in the HAL. */
    pd_hal_vconn_ocp_disable(port);

#else /* !CCG5 */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl &= ~PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Disable the ADC used for VConn OCP detection. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, 0, 0, 0, NULL);
#endif /* CCG4_DOCK */
#endif /* CCG5 */
}

#endif /* VCONN_OCP_ENABLE */

#if VBUS_OVP_ENABLE
/* Configure Over-Voltage Protection checks based on parameters in config table. */
void app_ovp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T ovp_cb)
{
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
    uint8_t level;
    uint8_t threshold;
#else
    uint8_t debounce;
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

    uint8_t intr_state;

    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection();

#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        threshold = pd_get_ptr_ovp_tbl(port)->threshold;

#if ADC_FALSE_OVP_FIX
        /* Make sure threshold is set to a suitable value at low voltages to avoid false OVP trips. */
        if ((volt_mV + ((volt_mV / 100) * threshold)) < ADC_VBUS_MIN_OVP_LEVEL)
        {
            volt_mV   = ADC_VBUS_MIN_OVP_LEVEL;
            threshold = 0;
        }
#endif /* ADC_FALSE_OVP_FIX */

        /* Set OVP threshold. */
        level = pd_get_vbus_adc_level(port, APP_OVP_ADC_ID, volt_mV, threshold);
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, APP_OVP_ADC_INPUT, level, PD_ADC_INT_FALLING, ovp_cb);

#else /* (VBUS_OVP_MODE != VBUS_OVP_MODE_ADC) */

        /* Convert debounce period from us to clock cycles assuming a 500 KHz filter clock. */
        debounce = pd_get_ptr_ovp_tbl(port)->debounce;
        debounce = (debounce > 0x40) ? 0x20 : ((debounce + 1) >> 1);

#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5))
        pd_internal_vbus_ovp_en(port, volt_mV, pd_get_ptr_ovp_tbl(port)->threshold, ovp_cb,
                pfet, VBUS_OVP_MODE, debounce);
#endif /* defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5) */

#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

        CyExitCriticalSection(intr_state);
    }
}

void app_ovp_disable(uint8_t port, bool pfet)
{
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OVP_EN_MASK)
    {
        /* Disable OVP. */
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, 0, 0, 0, NULL);
#else
#if defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5)
        pd_internal_vbus_ovp_dis(port, pfet);
#endif /* defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5)) */
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */
    }
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE

/* Configure Under-Voltage Protection checks based on parameters in config table. */
void app_uvp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T uvp_cb)
{
    uint8_t intr_state;
    uint8_t debounce;

    debounce = pd_get_ptr_uvp_tbl(port)->debounce;
    /* Assumming 500KHz clock */
    debounce = (debounce > 0x40) ? 0x20 : ((debounce+1) >> 1);

    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection ();

#if CCG_PPS_SRC_ENABLE
        if (dpm_get_info(port)->src_sel_pdo.fixed_src.supply_type == PDO_AUGMENTED)
        {
            /*
             * In PPS mode operation, UVP is not an unrecoverable event. It
             * needs to be dealt with a simple hardreset. Configure for non-
             * hardware cutoff operation. NOTE: The threshold for operation
             * can be overridden to set the cut-off based on system requirement.
             * Currently using the lowest UVP point for this.
             */
            volt_mV = dpm_get_info(port)->src_sel_pdo.pps_src.min_volt * 100;
            pd_internal_vbus_uvp_en (port, volt_mV, pd_get_ptr_uvp_tbl(port)->threshold,
                    uvp_cb, pfet, VBUS_UVP_MODE_INT_COMP, debounce);
        }
        else
#endif /* CCG_PPS_SRC_ENABLE */
        {
            pd_internal_vbus_uvp_en (port, volt_mV, pd_get_ptr_uvp_tbl(port)->threshold,
                    uvp_cb, pfet, VBUS_UVP_MODE, debounce);
        }

        CyExitCriticalSection (intr_state);
    }
}

void app_uvp_disable(uint8_t port, bool pfet)
{
    /* Disable UVP. */
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
    {
        pd_internal_vbus_uvp_dis (port, pfet);
    }
}

#endif /* VBUS_UVP_ENABLE */

/* End of File */
