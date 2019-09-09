/**
 * @file vbus_ctrl.c
 *
 * @brief@{VBUS Control PWM source file.@}
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
#include <pdss_hal.h>
#include <type_a.h>
#include <vbus_ctrl.h>
#include <dpm.h>
#include <app.h>
#include <battery_charging.h>
#include <timer.h>

/************************* PWM based VBUS Control. *****************************/

#define VBUS_C_PWM_PERIOD                               (480u)

uint16_t pwm_get_duty_cycle(uint16_t vbus, uint16_t min_volt, uint16_t max_volt,
    uint16_t pwm_period)
{
    uint32_t pwm_duty = 0;

    /*
     * Formula for Vout based on PWM Duty cycle is:
     * Vout = VBUS_MAX * (1/6 + 5/6 * PWM_DUTY_CYCLE)
     * Substituting PWM Period value and compare value (pwm_duty) in this formula
     * Vout = VBUS_MAX * (1/6 + 5/6 * pwm_duty/PWM_PERIOD)
     * Solving for pwm_duty
     * pwm_duty = PWM_PERIOD * ((Vout * 6 - VBUS_MAX)/VBUS_MAX * 5)
     */

    /* Ensure VBUS requested is not less than VBUS_MIN and more than VBUS_MAX. */
    if((vbus >= min_volt) && (vbus <= max_volt))
    {
        /* Applying formula as above. */
        pwm_duty = (vbus * 6) - max_volt;
        pwm_duty *= pwm_period;
        pwm_duty /=  max_volt * 5;
    }

    if(pwm_duty == 0)
    {
        pwm_duty = 1;
    }

    return pwm_duty;
}

void vbus_ctrl_pwm_turn_on(uint8_t port)
{
    if (port == TYPEC_PORT_0_IDX)
    {
#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_PWM)
        DIR_CTRL_C_Write (1);
        /* Need to determine the FET to be used. */
        if (CCG_SRC_FET)
        {
            pd_internal_pfet_on(0, false);
        }
        else
        {
            pd_internal_cfet_on(0, false);
        }
        BUCK_BOOST_EN_C_Write (0);
#endif /* VBUS_CTRL_TYPE_P1 */
    }
    else if (port == TYPEC_PORT_1_IDX)
    {
#if (CCG_TYPE_A_PORT_ENABLE == 1)
        type_a_enable_vbus ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
}

void vbus_ctrl_pwm_turn_off(uint8_t port)
{
    /* Turn off PWM. */
    if (port == TYPEC_PORT_0_IDX)
    {
#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_PWM)
        BUCK_BOOST_EN_C_Write(1);
        /* Need to determine the FET to be used. */
        if (CCG_SRC_FET)
        {
            pd_internal_pfet_off(0, false);
        }
        else
        {
            pd_internal_cfet_off(0, false);
        }
        PWM_C_WriteCompare (1);
        PWM_C_Stop ();
        PWM_OUT_C_SetDriveMode (PWM_OUT_C_DM_ALG_HIZ);
#endif /* VBUS_CTRL_TYPE_P1 */
    }
    else if (port == TYPEC_PORT_1_IDX)
    {
#if (CCG_TYPE_A_PORT_ENABLE == 1)
        type_a_disable_vbus ();
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
}

void vbus_ctrl_pwm_set_volt(uint8_t port, uint16_t volt_mV)
{
    /* Invoke PWM Control routine. */
    if (port == TYPEC_PORT_0_IDX)
    {
#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_PWM)
        uint16_t pwm_duty;
        pwm_duty = pwm_get_duty_cycle (volt_mV, pd_get_ptr_pwr_tbl(TYPEC_PORT_0_IDX)->vbus_min_volt,
            pd_get_ptr_pwr_tbl(TYPEC_PORT_0_IDX)->vbus_max_volt, VBUS_C_PWM_PERIOD);

        PWM_C_Start ();
        PWM_C_WriteCompare (pwm_duty);
        PWM_OUT_C_SetDriveMode (PWM_OUT_C_DM_STRONG);
#endif /* VBUS_CTRL_TYPE_P1 */
    }
    else if (port == TYPEC_PORT_1_IDX)
    {
#if (CCG_TYPE_A_PORT_ENABLE == 1)
        type_a_set_voltage (volt_mV);
#endif /* CCG_TYPE_A_PORT_ENABLE */
    }
}

/************************* FB based VBUS Control. *****************************/

typedef enum
{
    VBUS_CTRL_STATE_ENTRY = 0,      /* Entry state for VBUS change. */
    VBUS_CTRL_STATE_REACH_CLOSER,   /* State updates voltage to the requested level. */
    VBUS_CTRL_STATE_SLOPE_CHECK,    /* State waits for the voltage to stabilize. */
    VBUS_CTRL_STATE_FINE_TUNE,      /* State adjusts voltage based on ADC reading. */
    VBUS_CTRL_STATE_VBUS_SETTLE,    /* State provides required hysterisis wait time. */
    VBUS_CTRL_STATE_OVER            /* Voltage control completed state.*/

}vbus_ctrl_state_t;

/* 
 * This variable is used by the voltage update state machine. It holds whether to 
 * skip slope check or not after adjusting the voltage. This should not be modified
 * outside of the state machine logic.
 */
static bool gl_skip_slope_check;
/* This flag holds whether the feedback control logic is enabled or not. */
static bool gl_vbus_ctrl_enable;
/*
 * This variable holds the current iDAC setting loaded in hardware. This variable
 * avoids reading from the hardware registers everytime. It should always remain
 * in sync with hardware setting.
 */
static int16_t gl_idac;
/*
 * This variable is used by the voltage update state machine. It holds the required
 * change in steps of iDAC change pending to reach the requested voltage. IT should
 * not be modified outside of the state machine.
 */
static int16_t gl_idac_change;
/*
 * This variable is the current voltage level set. This value corresponds to the
 * iDAC setting done. It should not be modified outside of the state machine.
 */
static uint16_t gl_cur_volt;
/*
 * This variable is used by the slope check state machine to store the previously
 * read data. This variable should not be used as actual voltage data and should
 * not be used outside of the state machine.
 */
static uint16_t gl_prev_volt;
/*
 * This variable is used by the slope check state machine to store the number of
 * cycles of stable voltage attained. This variable should not be modified outside
 * of the state machine.
 */
static uint8_t gl_vbus_settle_cycle_count;

/* Current state of the VBUS control state machine. */
static vbus_ctrl_state_t gl_vbus_ctrl_state;

/*
 * The macro holds the VBUS change per change in iDAC setting. This voltage assumes
 * that the feedback divider resistance (R1) is 200KOhms. It needs to be 200KOhm
 * for supporting PPS operation. This value is used in signed number operation
 * and should be left as signed.
 */
#define VBUS_CHANGE_PER_DAC_BIT             (20)

/*
 * This macro holds the accuracy requirement for voltage control. This parameter
 * is used to determine whether we should apply voltage correction algorithm or
 * not. This value is used in signed number operation and should be left as signed.
 */
#define VBUS_CLOSE_PERCENT                  (5)
/*
 * This macro holds the limit after which we need to do slope detection for opto
 * isolator based designs. Since opto-isolator based designs are slow in voltage
 * correction, the slope check is done only beyond the threshold. Any voltage
 * change below this, is expected to meet PPS timings and should be covered by
 * the default timing provided. This value is used in signed number operation
 * and should be left as signed.
 */
#define VBUS_CTRL_OPTO_SLOPE_THRESHOLD      (500)

/* 
 * This macro is used by the slope detection algorithm to determine stable voltage.
 * If the variation in voltage is below this threshold, it is considered as stable.
 * This value is used in signed number operation and should be left as signed.
 */
#define VBUS_STABLE_MAX_VAR_MV              (200)

/* 
 * These macros hold the maximum change in iDAC steps allowed for the system.
 * These values are used in signed number operations and should be left as signed.
 */
#define VBUS_CTRL_MAX_SNK_STEP  (((int16_t)VBUS_CTRL_ABOVE_5V_MAX_STEP) / (VBUS_CHANGE_PER_DAC_BIT))
#define VBUS_CTRL_MAX_SRC_STEP  (((int16_t)VBUS_CTRL_BELOW_5V_MAX_STEP) / (VBUS_CHANGE_PER_DAC_BIT))

extern void app_psrc_tmr_cbk(uint8_t port, timer_id_t id);

/*
 * This function returns the signed iDAC step change required to reach the new
 * voltage specified from the current voltage specified. It does not rely on
 * the actual voltage global variables but the parameters to the function. The
 * calculation is done without any voltage accuracy improvement algorithm and
 * is used by the control algorithms to reach the required voltage.
 */
static int16_t vbus_ctrl_fb_get_idac_step(uint16_t new_volt, uint16_t cur_volt)
{
    return (((int16_t)new_volt - (int16_t)cur_volt) / VBUS_CHANGE_PER_DAC_BIT);
}

/*
 * This function returns the ideal voltage matching the providing iDAC value.
 * It does not apply any accuracy improvement algorithm and is used by the state
 * machine to achieve the required accuracy.
 */
static int16_t vbus_ctrl_fb_get_volt(int16_t idac)
{
    return ((int16_t)VSAFE_5V + (idac * VBUS_CHANGE_PER_DAC_BIT));
}

#if (VBUS_CTRL_TRIM_ADJUST_ENABLE)

/** Hardware register address information. Should not be modified. **/

/* Sink DAC trim - 5V */
#define BG_ISNK_DAC_CTRL_0_5V (*(volatile uint8_t *)(0x0FFFF28E))
#define BG_ISNK_DAC_CTRL_1_5V (*(volatile uint8_t *)(0x0FFFF28F))
#define BG_ISNK_DAC_CTRL_COMBINED_5V (BG_ISNK_DAC_CTRL_0_5V | \
        (BG_ISNK_DAC_CTRL_1_5V << 8))
/* Sink DAC trim - 20V */
#define BG_ISNK_DAC_CTRL_0_20V (*(volatile uint8_t *)(0x0FFFF291))
#define BG_ISNK_DAC_CTRL_1_20V (*(volatile uint8_t *)(0x0FFFF292))
#define BG_ISNK_DAC_CTRL_COMBINED_20V (BG_ISNK_DAC_CTRL_0_20V | \
        (BG_ISNK_DAC_CTRL_1_20V << 8))
/* Source DAC trim - 3V */
#define BG_ISRC_DAC_CTRL_0_3V (*(volatile uint8_t *)(0x0FFFF2A6))
/* Source DAC trim - 5V */
#define BG_ISRC_DAC_CTRL_0_5V (*(volatile uint8_t *)(0x0FFFF290))
/* Source DAC trim - 20V */
#define BG_ISRC_DAC_CTRL_0_20V (*(volatile uint8_t *)(0x0FFFF293))

/*
 * This function returns the iDAC value to be used for attaining the voltage.
 * It calculates the value based on the TRIM setting loaded in SFLASH and is
 * applicable only for opto-isolator based designs.
 */
static int16_t vbus_ctrl_get_trim_idac(uint16_t volt_mv)
{
	int16_t trim_code = 0;
    int16_t src_adj = BG_ISRC_DAC_CTRL_0_3V;

	if (volt_mv >= VSAFE_5V)
	{
        /* Sink iDAC will be used */
        trim_code = ((((BG_ISNK_DAC_CTRL_COMBINED_20V - 
                            (BG_ISNK_DAC_CTRL_COMBINED_5V - BG_ISRC_DAC_CTRL_0_5V)) * 
                        vbus_ctrl_fb_get_idac_step(volt_mv, VSAFE_5V)) +
                    ((BG_ISNK_DAC_CTRL_COMBINED_5V - BG_ISRC_DAC_CTRL_0_5V) * 750)) / 750);
    }
    else
    {
        /* Source iDac will be used */
        if(BG_ISRC_DAC_CTRL_0_3V == 0)
        {
            src_adj = 100;
        }
        trim_code = -((((src_adj + 
                            (BG_ISNK_DAC_CTRL_COMBINED_5V - BG_ISRC_DAC_CTRL_0_5V)) *
                        vbus_ctrl_fb_get_idac_step(VSAFE_5V, volt_mv)) -
                    ((BG_ISNK_DAC_CTRL_COMBINED_5V - BG_ISRC_DAC_CTRL_0_5V) * 100)) / 100);
    }

    return trim_code;
}
#endif /* (VBUS_CTRL_TRIM_ADJUST_ENABLE) */

/*
 * This function updates the iDAC values within the maximum change restrictions.
 * It takes in the signed iDAC change required and identifies the maximum change
 * allowed. It updates the hardware iDAC setting to this and returns the actual
 * change done. This function is repeatedly called by the state machine until 
 * all the required change is completed. The function does not do any error checks
 * and should not be called with incorrect or zero parameter values.
 */
static int16_t vbus_ctrl_fb_reach_closer(int16_t idac_change)
{
    int16_t step;

    if ((gl_idac > 0) || ((gl_idac == 0) && (idac_change > 0)))
    {
        /* We are using the sink iDAC. */
        if (idac_change > 0)
        {
            /*
             * We are increasing the voltage. But we can only do it within 
             * the maximum voltage step.
             */
            step = GET_MIN((uint16_t)idac_change, VBUS_CTRL_MAX_SNK_STEP);
        }
        else
        {
            /*
             * We are decreasing the voltage. We can only do this till zero
             * or the maximum threshold level allowed.
             */
            step = idac_change * -1;
            if ((uint16_t)step > VBUS_CTRL_MAX_SNK_STEP)
            {
                step = VBUS_CTRL_MAX_SNK_STEP;
            }
            if (gl_idac != 0)
            {
                step = GET_MIN(gl_idac, step);
            }

            /* Step needs to be signed. */
            step = (step * -1);
        }
    }
    else
    {
        /* We are using the source iDAC. */
        if (idac_change > 0)
        {
            /*
             * We are increasing the voltage. But we can only do it within 
             * the maximum voltage step. Also, we cannot cross zero.
             */
            step = idac_change;
            if ((gl_idac != 0) && ((gl_idac + idac_change) > 0))
            {
                step = gl_idac * -1;
            }
            step = GET_MIN((uint16_t)step, VBUS_CTRL_MAX_SRC_STEP);
        }
        else
        {
            /*
             * We are decreasing the voltage. We can only do this for maximum
             * step size allowed.
             */
            step = (idac_change  * -1);
            if ((uint16_t)step > VBUS_CTRL_MAX_SRC_STEP)
            {
                step = VBUS_CTRL_MAX_SRC_STEP;
            }

            /* Step needs to be signed. */
            step = (step * -1);
        }
    }

    /* Now we know the actual step size, we can apply it. */
    gl_idac += step;
    pd_hal_set_fb_dac(gl_idac);

    return step;
}

#if (VBUS_CTRL_STEP_US_ENABLE)
static void vbus_ctrl_fb_set_volt_cbk(uint8_t port, timer_id_t id);

static void vbus_ctrl_timer_isr(void)
{
    /* Clear the interrupt and invoke the voltage control callback. */
    VBUS_CTRL_TIMER_ClearInterrupt(VBUS_CTRL_TIMER_INTR_MASK_TC);
    vbus_ctrl_fb_set_volt_cbk(TYPEC_PORT_0_IDX, APP_PSOURCE_VBUS_SET_TIMER);
}
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */

/*
 * This function runs the voltage control state machine. This function is expected
 * to be called from the VBUS transition control timer running every 1ms until
 * the voltage transition is completed. The callback function itself determines
 * when to re-start the timer and should not be invoked outside of the current
 * vbus_ctrl_fb_set_volt() function.
 */
static void vbus_ctrl_fb_set_volt_cbk(uint8_t port, timer_id_t id)
{
    int16_t diff;
    uint16_t volt_mV, close_volt = 0;
    uint32_t tmp;
#if ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE))
    uint16_t abs_diff, thres;
#endif /* ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE)) */
    app_status_t* app_stat = app_get_status(port);

    (void)id;

    volt_mV = app_stat->psrc_volt;

    switch(gl_vbus_ctrl_state)
    {
        case VBUS_CTRL_STATE_ENTRY:        
#if ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE))
            diff = (int16_t)volt_mV - (int16_t)gl_cur_volt;
            abs_diff = (diff < 0) ? (diff * -1) : diff;
            thres = ((volt_mV * VBUS_CLOSE_PERCENT) / 100);

            /* 
             * gl_skip_slope_check can be true only when re-entering here from
             * the VBUS_CTRL_STATE_FINE_TUNE state. In this case, do not try to
             * again break the transition; continue with a single load.
             */
            if ((abs_diff > thres) && (gl_skip_slope_check == false))
            {
#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
                /*
                 * For direct feedback applications, apply ADC correction. This requires
                 * that we setup voltage to within 5% of the requested voltage and then
                 * correct the required level. In case of current limited state of PPS
                 * operation, we may not be able to measure via ADC. In this case, we
                 * may have to adjust the voltage based on absolute value. This will
                 * require us to store the voltage which was used for this adjustment
                 * as well.
                 */
                if (diff > 0)
                {
                    close_volt = volt_mV - thres;
                }
                else
                {
                    close_volt = volt_mV + thres;
                }

                /*
                 * If we are in current limit mode, then do not try to adjust 
                 * the voltage based on ADC. Just jump based on the iDAC step.
                 */
                if (dpm_get_info(port)->cur_fb)
                {
                    gl_idac_change = vbus_ctrl_fb_get_idac_step(volt_mV, gl_cur_volt);
                    close_volt = 0;
                }
                else
                {
                    gl_idac_change = vbus_ctrl_fb_get_idac_step(close_volt, gl_cur_volt);
                }
#else /* VBUS_CTRL_TRIM_ADJUST_ENABLE */
                /*
                 * For trim based application, we need to calculate the TRIM value
                 * required for getting to the relavant voltage.
                 */
                gl_idac_change = (vbus_ctrl_get_trim_idac(volt_mV) - gl_idac);
                /*
                 * Since opto based designs are slow, we cannot impose same slope
                 * rules for all voltage changes and still meet PPS timings. To
                 * avoid this, the minimum time is adjusted to account for all
                 * transitions within 500mV. Anything above shall go with the
                 * slope check.
                 */
                if (abs_diff < VBUS_CTRL_OPTO_SLOPE_THRESHOLD)
                {
                    gl_skip_slope_check = true;
                }
#endif /* VBUS_CTRL_ADC_ADJUST_ENABLE */
            }
            else
#endif /* ((VBUS_CTRL_ADC_ADJUST_ENABLE) || (VBUS_CTRL_TRIM_ADJUST_ENABLE)) */
            {
                /* Take direct value. */
                gl_idac_change = vbus_ctrl_fb_get_idac_step(volt_mV, gl_cur_volt);
#if (VBUS_CTRL_TRIM_ADJUST_ENABLE)
                gl_skip_slope_check = true;
#endif /* (VBUS_CTRL_TRIM_ADJUST_ENABLE) */
            }

            /* Update the current voltage variable. */
            if (close_volt == 0)
            {
                gl_cur_volt = volt_mV;
            }
            else
            {
                gl_cur_volt = close_volt;
            }

            gl_vbus_ctrl_state = VBUS_CTRL_STATE_REACH_CLOSER;
            /* Intentional fall through */

        case VBUS_CTRL_STATE_REACH_CLOSER:
            /*
             * Keep loading the iDAC in steps until final value is reached.
             * The maximum step size allowed depends on the system.
             */
            gl_idac_change -= vbus_ctrl_fb_reach_closer(gl_idac_change);
            if (gl_idac_change != 0)
            {
                break;
            }

            gl_prev_volt = 0;
            gl_vbus_settle_cycle_count = 0;
            if (gl_skip_slope_check)
            {
                gl_vbus_ctrl_state = VBUS_CTRL_STATE_VBUS_SETTLE;
                gl_skip_slope_check = false;
            }
            else
            {
                gl_vbus_ctrl_state = VBUS_CTRL_STATE_SLOPE_CHECK;
            }
            break;

        case VBUS_CTRL_STATE_SLOPE_CHECK:
            tmp = vbus_get_value(port);
            if (gl_prev_volt > tmp)
            {
                diff = gl_prev_volt - tmp;
            }
            else
            {
                diff = tmp - gl_prev_volt;
            }

            /*
             * If the slope for change is voltage has gone below the maximum,
             * variation in stable condition, continue to the next phase.
             */
            if (diff > (int16_t)VBUS_STABLE_MAX_VAR_MV)
            {
                /* Now store the current reading as previous. */
                gl_prev_volt = tmp;
                gl_vbus_settle_cycle_count = 0;
                break;
            }

            /*
             * We now need to debounce the slope check as the change can happen
             * slower than what we can measure. This debounce does an extended
             * slope check. gl_vbus_settle_cycle_count is being used for this
             * debounce.
             */
            gl_vbus_settle_cycle_count++;
            if (gl_vbus_settle_cycle_count < VBUS_CTRL_SLOPE_DEBOUNCE_PERIOD)
            {
                break;
            }

            /*
             * If the final voltage is not yet acheived, then we need to proceed
             * with fine tuning the voltage based on ADC reading. Else proceed
             * to wait for settling time.
             */
            if (gl_cur_volt != volt_mV)
            {
                gl_vbus_ctrl_state = VBUS_CTRL_STATE_FINE_TUNE;
            }
            else
            {
                gl_vbus_ctrl_state = VBUS_CTRL_STATE_VBUS_SETTLE;
            }
            break;

#if (VBUS_CTRL_ADC_ADJUST_ENABLE)
        case VBUS_CTRL_STATE_FINE_TUNE:
            /*
             * This means that we have not attained the final voltage. Check
             * again to ensure that we are not in CF mode. If we are, then do
             * not adjust. Just update based on the stored voltage value.
             */
            if (dpm_get_info(port)->cur_fb == false)
            {
                gl_cur_volt = vbus_get_value(port);
            }

            /* Now adjust the voltage as required. */
            gl_skip_slope_check = true;
            gl_vbus_ctrl_state = VBUS_CTRL_STATE_ENTRY;
            break;
#endif /* (VBUS_CTRL_ADC_ADJUST_ENABLE) */

        case VBUS_CTRL_STATE_VBUS_SETTLE:
            /*
             * If this is an active connection, then trigger the hysteris timer.
             * In case of non-PPS PDO, we need to wait for voltage to attain the
             * required voltage. In case of PPS PDO, we need to enforce a minimum
             * settling time as there is no guaranteed voltage to look for.
             */
            if (timer_is_running(port, APP_PSOURCE_EN_TIMER))
            {
                uint16_t delay;

                if (app_stat->cur_fb_enabled)
                {
                    delay = (APP_PSOURCE_EN_TIMER_PERIOD - 
                            timer_get_count(port, APP_PSOURCE_EN_TIMER));
                    if (delay < VBUS_CTRL_SETTLE_TIME_PERIOD)
                    {
                        delay = (APP_PSOURCE_EN_HYS_TIMER_PERIOD + 
                                (VBUS_CTRL_SETTLE_TIME_PERIOD - delay));
                    }
                    else
                    {
                        delay = APP_PSOURCE_EN_HYS_TIMER_PERIOD;
                    }
                }
                else
                {
                    /* Wait for the voltage to be within range. */
                    if (!(((app_stat->psrc_rising == true) &&
                                (vbus_is_present(port, app_stat->psrc_volt, VBUS_TURN_ON_MARGIN) == true)) ||
                            ((app_stat->psrc_rising == false) &&
                             (vbus_is_present(port, app_stat->psrc_volt, VBUS_DISCHARGE_MARGIN) == false))))
                    {
                        break;
                    }

                    delay = APP_PSOURCE_EN_HYS_TIMER_PERIOD;
                }

                timer_start(port, APP_PSOURCE_EN_HYS_TIMER, delay, app_psrc_tmr_cbk);
            }
            gl_vbus_ctrl_state = VBUS_CTRL_STATE_OVER;
            /* Intentional fall through */

        case VBUS_CTRL_STATE_OVER:
            /* Do nothing. Stay in this state until a new request comes. */
            break;

        default:
            /* Will never come here. */
            break;
    }

    /* 
     * Restart the timer if VBUS transition is not completed.The timer is expected
     * to be running at 1ms. If the voltage transition steps needs to be done
     * at a smaller step size than this, then the timer call can be updated to
     * use a TCPWM timer when in VBUS_CTRL_STATE_REACH_CLOSER state.
     * All other states still requires timer to be operating at 1ms step size.
     */
#if (VBUS_CTRL_STEP_US_ENABLE)
    if (gl_vbus_ctrl_state == VBUS_CTRL_STATE_REACH_CLOSER)
    {
        /* 
         * Start a one shot TCPWM timer to trigger a voltage change. The timer
         * block is expected to be configured with the required period and so
         * just need to be started.
         */
        VBUS_CTRL_TIMER_ISR_StartEx(vbus_ctrl_timer_isr);
        VBUS_CTRL_TIMER_Start();
    }
    else
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */
    {
        if (gl_vbus_ctrl_state != VBUS_CTRL_STATE_OVER)
        {
            timer_start(port, APP_PSOURCE_VBUS_SET_TIMER,
                    APP_PSOURCE_VBUS_SET_TIMER_PERIOD, vbus_ctrl_fb_set_volt_cbk);
        }
    }
}

void vbus_ctrl_fb_enable(uint8_t port)
{
    (void)port;

    /* Do this only if the control is disabled. */
    if (gl_vbus_ctrl_enable == false)
    {
        pd_hal_enable_cv();
        gl_cur_volt = VSAFE_5V;
        gl_idac = 0;
        gl_vbus_ctrl_enable = true;
    }
}

void vbus_ctrl_fb_disable(uint8_t port)
{
    (void)port;

    timer_stop(port, APP_PSOURCE_VBUS_SET_TIMER);
#if (VBUS_CTRL_STEP_US_ENABLE)
    VBUS_CTRL_TIMER_Stop();
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */

    pd_hal_disable_cv();
    gl_cur_volt = VSAFE_5V;
    gl_idac = 0;
    gl_vbus_ctrl_state = VBUS_CTRL_STATE_OVER;
    gl_vbus_ctrl_enable = false;
}

void vbus_ctrl_fb_set_volt(uint8_t port, uint16_t volt_mV)
{
    /* Stop any previous state machine execution on receiving a new request. */
    timer_stop(port, APP_PSOURCE_VBUS_SET_TIMER);
#if (VBUS_CTRL_STEP_US_ENABLE)
    VBUS_CTRL_TIMER_Stop();
#endif /* (VBUS_CTRL_STEP_US_ENABLE) */

    /* If a request for 0V comes, then do nothing. Disable the FB block and exit. */
    if (volt_mV <= VSAFE_0V)
    {
        vbus_ctrl_fb_disable(port);
        return;
    }

    /*
     * First calculate the new iDAC setting required for the operation. Simplest
     * is to use absolute based on the iDAC step size. However, using this can
     * result in loss of accuracy. In opto-isolator based designs, we can apply
     * a TRIM based calibration to calculate the required iDAC setting. In case
     * of direct feedback implementation, the voltage correction needs to be done
     * with ADC.
     * In all cases, if the voltage step size is too small, we do not apply
     * improvement algorithm as the PPS specification expects voltage change on
     * every request.
     */
    if (gl_cur_volt <= VSAFE_0V)
    {
        /* Ensure default values are correct. */
        gl_cur_volt = VSAFE_5V;
        gl_idac = 0;
    }
    /*
     * If the timer was stopped before completion, the gl_cur_volt is
     * wrong and needs to be fixed. This check needs to be taken up.
     */
    if (gl_vbus_ctrl_state != VBUS_CTRL_STATE_OVER)
    {
        gl_cur_volt = vbus_ctrl_fb_get_volt(gl_idac);
    }
    
    /* Ensure that the default values are loaded correctly. */
    gl_skip_slope_check = false;


    /* If the requested voltage is same as previous, then just hold-off for minimum time. */
    if (gl_cur_volt == volt_mV)
    {
        /*
         * TODO: Evaluate if we need to adjust for missed ADC correction during
         * current foldback mode. This seems to make things worse and so is not
         * done for now.
         */
        gl_vbus_ctrl_state = VBUS_CTRL_STATE_VBUS_SETTLE;
        /*
         * Start the timer so that we give time for APP_PSOURCE_EN_TIMER to get
         * started where applicable. This is so that we can trigger the callback
         * locally without having another monitor timer running from the psrc_enable()
         * function.
         */
        timer_start(port, APP_PSOURCE_VBUS_SET_TIMER, 
            APP_PSOURCE_VBUS_SET_TIMER_PERIOD, vbus_ctrl_fb_set_volt_cbk);
    }
    else
    {
        gl_vbus_ctrl_state = VBUS_CTRL_STATE_ENTRY;
        /* Now invoke the timer callback directly to avoid any delay. */
        vbus_ctrl_fb_set_volt_cbk(port, APP_PSOURCE_VBUS_SET_TIMER);
    }
}

/* [] */

