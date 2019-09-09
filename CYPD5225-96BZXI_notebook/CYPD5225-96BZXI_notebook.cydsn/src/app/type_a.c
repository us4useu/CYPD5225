/**
 * @file type_a.c
 *
 * @brief@{TYPE-A port source file.@}
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

#include <stdbool.h>
#include <config.h>
#include <stack_params.h>
#include <status.h>
#include <pd.h>
#include <app.h>
#include <pdss_hal.h>
#include <battery_charging.h>
#include <vbus_ctrl.h>
#include <psource.h>
#include <gpio.h>
#include <timer.h>
#include <chgb_hal.h>
#include <type_a.h>

#if (CCG_TYPE_A_PORT_ENABLE == 1)

/* Number of counts configured as PWM period. */
#define VBUS_A_PWM_PERIOD                       (PWM_A_PWM_PERIOD_VALUE)
/* Maximum change in percentage in PWM duty cycle allowed in one step. */
#define TYPE_A_PWM_STEP_MAX_PER                 (10)
/* Maximum change in PWM duty cycle in terms of PWM counts. */
#define TYPE_A_PWM_STEP_MAX_SIZE                ((int)(VBUS_A_PWM_PERIOD) * \
        (TYPE_A_PWM_STEP_MAX_PER) / 100)

/* GPIO connected to TYPE-A Current sense. */
#define TYPE_A_CUR_SENSE_GPIO                           (GPIO_PORT_2_PIN_3)

/* TYPE-A Disconnect Vsense in mV. */
/*
 * TYPE-A Rsense is 20m Ohm and gain is 200. By default we are looking for
 * 200mA as detach threshold. This translates to 800mV Vsense.
 */
#define TYPE_A_DISCON_VSENSE                            (800)

/* TYPE-A port status. */
type_a_status_t gl_type_a_status;

type_a_status_t* type_a_get_status(void)
{
    return &gl_type_a_status;
}

bool type_a_is_idle(void)
{
    /*
     * Return true if TYPE-A port is not connected so that system can enter low
     * power mode. If TYPE-A port is connected, then PWM is being used for supplying
     * VBSU and hence system can't enter low power mode.
     * Note: It is expected that legacy charging state machine checks for TYPE-A port
     * are done outside of this function for determining IDLE status.
     */
    return !gl_type_a_status.type_a_connected;
}

void type_a_enable_sdp(void)
{
    if (gl_type_a_status.sdp_mode_enabled == false)
    {
        /* Enable SDP termiantions and apply 5V VBUS using low power regulator. */
#if BATTERY_CHARGING_ENABLE
        chgb_apply_src_term(BC_PORT_1_IDX, CHGB_SRC_TERM_SDP);
#endif /* BATTERY_CHARGING_ENABLE */
        TYPE_A_VBUS_EN_Write(0);
        TYPE_A_VBUS_EN_SetDriveMode(TYPE_A_VBUS_EN_DM_STRONG);
        gl_type_a_status.sdp_mode_enabled = true;
    }
}

static void type_a_enable_boost(void)
{
     BUCK_BOOST_EN_A_Write(0);
}

static void type_a_disable_boost(void)
{
    BUCK_BOOST_EN_A_Write(1);
    /* Turn off PWM. */
    PWM_A_WriteCompare(1);
    PWM_A_Stop();
    PWM_OUT_A_SetDriveMode(PWM_OUT_A_DM_ALG_HIZ);
    timer_stop(0, TYPE_A_PWM_STEP_TIMER_ID);
    gl_type_a_status.pwm_duty = 0;
}

void type_a_reg_switch_timer_cb(uint8_t instance, timer_id_t id)
{
    /* Disable low power regulator here. */
    TYPE_A_VBUS_EN_SetDriveMode(TYPE_A_VBUS_EN_DM_ALG_HIZ);
}

void type_a_update_status(bool is_connect, bool detach_det)
{
    if (is_connect == true)
    {
#ifdef TYPE_A_DUAL_REG_ENABLE
        /* TYPE-A port is in connect state. Enable buck-boost regulator with 5V. */
        psrc_set_voltage(TYPE_A_PORT_ID, VSAFE_5V);
        type_a_enable_boost();

        /*
         * We have to give some time before turning off low power regulator.
         * This gives time to buck boost regulator to sustain loaded conditions.
         * Using a timer for this.
         */
        timer_start(0, TYPE_A_REG_SWITCH_TIMER_ID, TYPE_A_REG_SWITCH_TIMER_PERIOD,
            type_a_reg_switch_timer_cb);
#endif /* TYPE_A_DUAL_REG_ENABLE */
    }
    else
    {
#ifdef TYPE_A_DUAL_REG_ENABLE
        /*
         * TYPE-A port is no longer connected. Disable buck-boost regulator and switch
         * to 5V VBUS from LDO.
         */
        type_a_disable_boost();

        timer_stop(0, TYPE_A_REG_SWITCH_TIMER_ID);
        /* Enable VBUS from low power regulator. */
        TYPE_A_VBUS_EN_Write(0);
        TYPE_A_VBUS_EN_SetDriveMode(TYPE_A_VBUS_EN_DM_STRONG);
#endif /* TYPE_A_DUAL_REG_ENABLE */
    }
    
    /* Ensure TYPE-A current sense debounce timer is not running. */
    gl_type_a_status.cur_sense_debounce_active = false;
    timer_stop(0, TYPE_A_CUR_SENSE_TIMER_ID);

    gl_type_a_status.type_a_connected = is_connect;
    gl_type_a_status.detach_det = detach_det;
}

void type_a_set_volt_timer_cbk(uint8_t port, timer_id_t id)
{
    int32_t tmp;

    (void)port;
    (void)id;

    tmp = (gl_type_a_status.new_pwm_duty - gl_type_a_status.pwm_duty);

    /* Adjust to the maximum step size allowed. */
    if (tmp > 0)
    {
        tmp = GET_MIN(tmp, TYPE_A_PWM_STEP_MAX_SIZE);
    }
    else
    {
        tmp = GET_MAX(tmp, (-1 * (int32_t)(TYPE_A_PWM_STEP_MAX_SIZE)));
    }

    gl_type_a_status.pwm_duty += tmp;
    PWM_A_WriteCompare(gl_type_a_status.pwm_duty);

    if (gl_type_a_status.pwm_duty != gl_type_a_status.new_pwm_duty)
    {
        /* Start the timer if the PWM needs further adjustment. */
        timer_start(0, TYPE_A_PWM_STEP_TIMER_ID, TYPE_A_PWM_STEP_TIMER_PERIOD,
                type_a_set_volt_timer_cbk);
    }
}

void type_a_set_voltage(uint16_t volt_mV)
{
    timer_stop(0, TYPE_A_PWM_STEP_TIMER_ID);

    /* Set up PWM for TYPE-A VBUS. */
    gl_type_a_status.new_pwm_duty = pwm_get_duty_cycle(volt_mV, 
            pd_get_ptr_type_a_pwr_tbl(0)->vbus_min_volt,
        pd_get_ptr_type_a_pwr_tbl(0)->vbus_max_volt, VBUS_A_PWM_PERIOD);

    PWM_A_Start();
    /* Invoke the callback directly to avoid losing time. */
    type_a_set_volt_timer_cbk(0, TYPE_A_PWM_STEP_TIMER_ID);
    PWM_OUT_A_SetDriveMode(PWM_OUT_A_DM_STRONG);

    gl_type_a_status.cur_vbus_a = volt_mV;
}

void type_a_enable_vbus(void)
{
#ifndef TYPE_A_DUAL_REG_ENABLE
    /* Enable VBUS_A high-voltage buck-boost VBUS regulator. */
    type_a_enable_boost();
#else
    /*
     * Do nothing because either buck-boost is already enabled or low power
     * regulator is providing default 5V VBUS.
     */
#endif /* TYPE_A_DUAL_REG_ENABLE */
}

void type_a_disable_vbus(void)
{
    /* Disable VBUS_A buck-boost VBUS regulator. */
    type_a_disable_boost();

#ifdef TYPE_A_DUAL_REG_ENABLE
    /* Disable VBUS from low-power regulator as well. */
    TYPE_A_VBUS_EN_SetDriveMode(TYPE_A_VBUS_EN_DM_ALG_HIZ);
#endif /* TYPE_A_DUAL_REG_ENABLE */

    gl_type_a_status.cur_vbus_a = 0;
}

void type_a_port_disable(void)
{
    /* Disbale TYPE-A port. Don't provide any VBUS. */
    type_a_disable_vbus();
#if BATTERY_CHARGING_ENABLE
    /* Disable Legacy charging. */
    bc_stop(BC_PORT_1_IDX);
#endif /* BATTERY_CHARGING_ENABLE */

    gl_type_a_status.type_a_enabled = false;
    gl_type_a_status.type_a_connected = false;

    /* Ensure TYPE-A current sense debounce timer is not running. */
    gl_type_a_status.cur_sense_debounce_active = false;
    gl_type_a_status.sdp_mode_enabled = false;
    timer_stop(0, TYPE_A_CUR_SENSE_TIMER_ID);
}

void type_a_port_enable(void)
{
    /* Enable TYPE-A port.*/

    /* Turn on VBUS. */
#ifndef TYPE_A_DUAL_REG_ENABLE
    /* Enabling Buck Boost here for 5V VBUS as 5V from LDO is not available. */
    type_a_set_voltage(VSAFE_5V);
    type_a_enable_vbus();
#endif /* TYPE_A_DUAL_REG_ENABLE */

    /* Initialize legacy charging. */
#if BATTERY_CHARGING_ENABLE
    bc_init(BC_PORT_1_IDX);
    bc_start(BC_PORT_1_IDX, BC_PORT_SOURCE);
#endif /* BATTERY_CHARGING_ENABLE */

    gl_type_a_status.type_a_enabled = true;
    gl_type_a_status.detach_det = false;
    gl_type_a_status.cur_sense_debounce_active = false;
    gl_type_a_status.sdp_mode_enabled = false;
}

void type_a_detect_disconnect(void)
{
    /*
     * Run through detach detection if TYPE-A port is already connected to a
     * BC 1.2 or Apple device. These devices require detach detection based on
     * current drawn from the port.
     */
    if ((gl_type_a_status.type_a_connected == true) &&
        (gl_type_a_status.detach_det == true))
    {
        bool comp_out;

        uint8_t dac_level;
        hsiom_set_config(TYPE_A_CUR_SENSE_GPIO, HSIOM_MODE_AMUXA);

        /* Convert Vsense to DAC level. */
        dac_level = pd_adc_volt_to_level(0, PD_ADC_ID_1, TYPE_A_DISCON_VSENSE);

        /* Get comparator status. */
        comp_out = pd_adc_comparator_sample(0, PD_ADC_ID_1,
                PD_ADC_INPUT_AMUX_A, dac_level);
        hsiom_set_config(TYPE_A_CUR_SENSE_GPIO, HSIOM_MODE_GPIO);

        /*
         * If comparator output is high, TYPE-A port is in attached state.
         * Otherwise it is disconnected state.
         */
        if (comp_out == false)
        {
            /*
             * We need to debounce this condition for 30 seconds before
             * switching to low power regulator.
             */
            if (gl_type_a_status.cur_sense_debounce_active == false)
            {
                /* Start debounce timer. */
                timer_start(0, TYPE_A_CUR_SENSE_TIMER_ID,
                        TYPE_A_CUR_SENSE_TIMER_PERIOD, NULL);
                gl_type_a_status.cur_sense_debounce_active = true;
            }
            else
            {
                /* Debounce active. Check if debounce time has expired. */
                if (!(timer_is_running(0, TYPE_A_CUR_SENSE_TIMER_ID)))
                {
                    gl_type_a_status.cur_sense_debounce_active = false;
                    /* Indicate TYPE-A disconnect. */
                    type_a_update_status(false, false);
                }
            }
        }
        else
        {
            /* Don't debounce disconnect condition. */
            gl_type_a_status.cur_sense_debounce_active = false;
            timer_stop(0, TYPE_A_CUR_SENSE_TIMER_ID);
        }
    }
}
#endif /* CCG_TYPE_A_PORT_ENABLE */

/* End of File */
