/**
 * @file pdo.c
 *
 * @brief @{PDO evaluation and handling functions.@}
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
#include <pdo.h>
#include <app.h>

#if (!(CCG_SOURCE_ONLY))
static uint32_t max_min_cur_pwr[NO_OF_TYPEC_PORTS];
static uint32_t contract_power[NO_OF_TYPEC_PORTS];
static uint32_t contract_voltage[NO_OF_TYPEC_PORTS];
static uint32_t op_cur_power[NO_OF_TYPEC_PORTS];

static uint32_t calc_power(uint32_t voltage, uint32_t current)
{
    /*
       Voltage is expressed in 50 mV units.
       Current is expressed in 10 mA units.
       Power should be expressed in 250 mW units.
     */
    return (DIV_ROUND_UP(voltage * current, 500));
}

#if !DISABLE_PDO_BATTERY
static uint32_t calc_current(uint32_t power, uint32_t voltage)
{
    /*
       Power is expressed in 250 mW units.
       Voltage is expressed in 50 mV units.
       Current should be expressed in 10 mA units.
     */
    return (DIV_ROUND_UP(power * 500, voltage));
}
#endif /* DISABLE_PDO_BATTERY */

/**
 * Checks if SRC pdo is acceptable for SNK pdo.
 * @param pdo_src pointer to current SRC PDO
 * @param pdo_snk pointer to current SNK PDO
 * @return True if current src pdo is acceptable for current snk pdo
 */
static bool is_src_acceptable_snk(uint8_t port, pd_do_t* pdo_src, uint8_t snk_pdo_idx)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    pd_do_t* pdo_snk = (pd_do_t*)&dpm->cur_snk_pdo[snk_pdo_idx];
    uint32_t snk_supply_type = pdo_snk->fixed_snk.supply_type;
    uint32_t fix_volt;
    uint32_t max_volt;
    uint32_t min_volt;
    uint32_t out = false;
    uint32_t max_min_temp, compare_temp;
#if !DISABLE_PDO_BATTERY    
    uint32_t oper_cur_pwr;
#endif /* DISABLE_PDO_BATTERY */

    max_min_temp = dpm->cur_snk_max_min[snk_pdo_idx] & SNK_MIN_MAX_MASK;

    switch(pdo_src->fixed_src.supply_type)
    {
        case PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
            fix_volt = pdo_src->fixed_src.voltage;
            max_volt = DIV_ROUND_UP(fix_volt, 20);
            min_volt = fix_volt - max_volt;
            max_volt = fix_volt + max_volt;

            switch(snk_supply_type)  /* Checking sink PDO type */
            {
                case PDO_FIXED_SUPPLY:
                    if(fix_volt == pdo_snk->fixed_snk.voltage)
                    {
                        compare_temp = GET_MAX (max_min_temp, pdo_snk->fixed_snk.op_current);
                        if (pdo_src->fixed_src.max_current >= compare_temp)
                        {
#if (!POWER_BANK)
                            op_cur_power[port] = pdo_snk->fixed_snk.op_current;
#else /* (POWER_BANK) */
                            op_cur_power[port] = pdo_src->fixed_src.max_current;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case PDO_VARIABLE_SUPPLY:
                    if ((min_volt >= pdo_snk->var_snk.min_voltage) && (max_volt <= pdo_snk->var_snk.max_voltage))
                    {
                        compare_temp = GET_MAX (max_min_temp, pdo_snk->var_snk.op_current);
                        if (pdo_src->fixed_src.max_current >= compare_temp)
                        {
#if (!POWER_BANK)
                            op_cur_power[port] = pdo_snk->var_snk.op_current;
#else /* (POWER_BANK) */
                            op_cur_power[port] = pdo_src->fixed_src.max_current;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;
#if !DISABLE_PDO_BATTERY
                case PDO_BATTERY:
                    if ((min_volt >= pdo_snk->bat_snk.min_voltage) && (max_volt <= pdo_snk->bat_snk.max_voltage))
                    {
                        fix_volt = min_volt;

                        /* Calculate the operating current and min/max current values. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.op_power, min_volt);
                        max_min_temp = calc_current(max_min_temp, min_volt);

                        /* Make sure the source can supply the maximum current that may be required. */
                        compare_temp = GET_MAX(max_min_temp, oper_cur_pwr);
                        if (pdo_src->fixed_src.max_current >= compare_temp)
                        {
                            op_cur_power[port] = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;
#endif /* DISABLE_PDO_BATTERY */
                default:
                    break;
            }

            if (out)
            {
                contract_voltage[port] = fix_volt;
                contract_power[port]   = calc_power (fix_volt, op_cur_power[port]);
                max_min_cur_pwr[port]  = max_min_temp;
            }
            break;
#if !DISABLE_PDO_BATTERY
        case PDO_BATTERY:   /* SRC is a battery */
            max_volt = pdo_src->bat_src.max_voltage;
            min_volt = pdo_src->bat_src.min_voltage;

            switch(snk_supply_type)
            {
                case PDO_FIXED_SUPPLY:
                    /* Battery cannot supply fixed voltage
                     * Battery voltage changes with time
                     * This contract if permitted can be un-reliable */
                    break;

                case PDO_VARIABLE_SUPPLY:
                    if((min_volt >= pdo_snk->var_snk.min_voltage) && (max_volt <= pdo_snk->var_snk.max_voltage))
                    {
                        /* Calculate the expected operating power and maximum power requirement. */
                        oper_cur_pwr = calc_power(max_volt, pdo_snk->var_snk.op_current);
                        max_min_temp = calc_power(max_volt, max_min_temp);

                        compare_temp = GET_MAX (max_min_temp, oper_cur_pwr);
                        if (pdo_src->bat_src.max_power >= compare_temp)
                        {
#if (!POWER_BANK)
                            op_cur_power[port] = oper_cur_pwr;
#else /* (POWER_BANK) */
                            op_cur_power[port] = pdo_src->bat_src.max_power;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case PDO_BATTERY:
                    /* Battery connected directly to a battery
                     * This combination is unreliable */
                    if((min_volt >= pdo_snk->bat_snk.min_voltage) && (max_volt <= pdo_snk->bat_snk.max_voltage))
                    {
                        compare_temp = GET_MAX (max_min_temp, pdo_snk->bat_snk.op_power);
                        if (pdo_src->bat_src.max_power >= compare_temp)
                        {
                            op_cur_power[port] = pdo_snk->bat_snk.op_power;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                contract_voltage[port] = max_volt;
                max_min_cur_pwr[port]  = max_min_temp;
                contract_power[port]   = op_cur_power[port];
            }
            break;
#endif /* DISABLE_PDO_BATTERY */
        case PDO_VARIABLE_SUPPLY:   /* Variable supply PDO */
            max_volt = pdo_src->var_src.max_voltage;
            min_volt = pdo_src->var_src.min_voltage;

            switch (snk_supply_type) /* Checking sink PDO type */
            {
                case PDO_FIXED_SUPPLY:
                    /* This connection is not feasible
                     * A variable source cannot provide a fixed voltage */
                    break;

                case PDO_VARIABLE_SUPPLY:
                    if((min_volt >= pdo_snk->var_snk.min_voltage) && (max_volt <= pdo_snk->var_snk.max_voltage))
                    {
                        compare_temp = GET_MAX (pdo_snk->var_snk.op_current, max_min_temp);
                        if (pdo_src->var_src.max_current >= compare_temp)
                        {
#if (!POWER_BANK)
                            contract_power[port] = calc_power(min_volt, pdo_snk->var_snk.op_current);
                            op_cur_power[port]   = pdo_snk->var_snk.op_current;
#else /* (POWER_BANK) */
                            contract_power[port] = calc_power(min_volt, pdo_src->var_src.max_current);
                            op_cur_power[port]   = pdo_src->var_src.max_current;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;
#if !DISABLE_PDO_BATTERY
                case PDO_BATTERY:
                    if((min_volt >= pdo_snk->bat_snk.min_voltage) && (max_volt <= pdo_snk->bat_snk.max_voltage))
                    {
                        /* Convert from power to current. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.op_power, min_volt);
                        max_min_temp = calc_current(max_min_temp, min_volt);

                        compare_temp = GET_MAX (oper_cur_pwr, max_min_temp);
                        if (pdo_src->var_src.max_current >= compare_temp)
                        {
                            contract_power[port] = pdo_snk->bat_snk.op_power;
                            op_cur_power[port]   = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;
#endif /* DISABLE_PDO_BATTERY */
                default:
                    break;
            }

            if (out)
            {
                contract_voltage[port] = max_volt;
                max_min_cur_pwr[port]  = max_min_temp;
            }
            break;

        default:
            break;
    }

    return out;
}

static pd_do_t form_rdo(uint8_t port, uint8_t pdo_no, bool capMisMatch, bool giveBack)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    pd_do_t snk_rdo;

    snk_rdo.val = 0u;
    snk_rdo.rdo_gen.no_usb_suspend = dpm->snk_usb_susp_en;
    snk_rdo.rdo_gen.usb_comm_cap = dpm->snk_usb_comm_en;
    snk_rdo.rdo_gen.cap_mismatch = capMisMatch;
    snk_rdo.rdo_gen.obj_pos = pdo_no;
    snk_rdo.rdo_gen.give_back_flag = (capMisMatch) ? false : giveBack;
    snk_rdo.rdo_gen.op_power_cur = op_cur_power[port];
    snk_rdo.rdo_gen.min_max_power_cur = max_min_cur_pwr[port];
    if( (snk_rdo.rdo_gen.give_back_flag == false) &&
            (snk_rdo.rdo_gen.op_power_cur > snk_rdo.rdo_gen.min_max_power_cur))
    {
        snk_rdo.rdo_gen.min_max_power_cur = snk_rdo.rdo_gen.op_power_cur;
    }

#if CCG_PD_REV3_ENABLE
    if(dpm->spec_rev_sop_live >= PD_REV3)
    {
        snk_rdo.rdo_gen.unchunk_sup = true;
    }
#endif /* CCG_PD_REV3_ENABLE */

    return snk_rdo;
}

/*
 * This function can be used to ask EC to evaluate a src cap message
 * For now evaluating here and executing the callback in this function itself
 */
void eval_src_cap(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    uint8_t src_pdo_index, snk_pdo_index;
    uint8_t num_src_pdo = src_cap->len;
    pd_do_t* snk_pdo = (pd_do_t*)&dpm->cur_snk_pdo[0];
    uint16_t src_vsafe5_cur = src_cap->dat[0].fixed_src.max_current; /* Source max current for first PDO */
    pd_do_t snk_rdo;
    uint32_t highest_contract_power = 0u;

    bool match = false;
    bool high_cap = snk_pdo[0].fixed_snk.high_cap;

    for(snk_pdo_index = 0u; snk_pdo_index < dpm->cur_snk_pdo_count; snk_pdo_index++)
    {
        for(src_pdo_index = 0u; src_pdo_index < num_src_pdo; src_pdo_index++)
        {
            if(is_src_acceptable_snk(port, (pd_do_t*)(&src_cap->dat[src_pdo_index]), snk_pdo_index))
            {
                /* contract_power is calculated in is_src_acceptable_snk() */
                if(contract_power[port] >= highest_contract_power)
                {
                    /* Check if sink needs higher capability */
                    if(high_cap && (contract_voltage[port] == (VSAFE_5V/PD_VOLT_PER_UNIT)))
                    {
                        /* 5V contract isn't acceptable with high_cap = 1 */
                        continue;
                    }

                    highest_contract_power = contract_power[port];
                    snk_rdo = form_rdo(port, (src_pdo_index + 1u), false,
                            (dpm->cur_snk_max_min[snk_pdo_index] & GIVE_BACK_MASK));
                    match = true;
                }
            }
        }
    }

    if(match == false)
    {
        /* No match happened */
        /* Capability mismatch */
        /* Ask for vsafe5v PDO with CapMismatch */
        contract_voltage[port] = snk_pdo[0].fixed_snk.voltage;
        op_cur_power[port] = snk_pdo[0].fixed_snk.op_current;
        contract_power[port] = DIV_ROUND_UP(contract_voltage[port] * op_cur_power[port], 500u);

        if(src_vsafe5_cur < op_cur_power[port])
        {
            /* SNK operation current can't be bigger than SRC max_current */
            op_cur_power[port] = src_vsafe5_cur;
        }

        max_min_cur_pwr[port] = dpm->cur_snk_max_min[0];
        snk_rdo = form_rdo(port, 1u, true, false);
    }

    app_get_resp_buf(port)->resp_do = snk_rdo;
    app_resp_handler(port, app_get_resp_buf(port));
}
#endif /* (!(CCG_SOURCE_ONLY)) */

/*
 * This function can be used to ask EC to evaluate a request message
 * For now evaluating here and executing the callback in this function itself
 */
void eval_rdo(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler)
{
    if(dpm_is_rdo_valid(port, rdo) == CCG_STAT_SUCCESS)
    {
        app_get_resp_buf(port)->req_status = REQ_ACCEPT;
    }
    else
    {
        app_get_resp_buf(port)->req_status = REQ_REJECT;
    }
    app_resp_handler(port, app_get_resp_buf(port));
}

 /* End of File */

