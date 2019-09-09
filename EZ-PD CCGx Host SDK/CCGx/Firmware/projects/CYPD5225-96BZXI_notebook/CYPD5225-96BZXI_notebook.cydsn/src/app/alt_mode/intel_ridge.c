/**
 * @file intel_ridge.c
 *
 * @brief @{Intel Thunderbolt Controller (ridge) control interface source file.@}
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
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR/TR PURPOSE. Cypress reserves the
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
#include <intel_ridge.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <dpm.h>
#include <timer.h>
#include <app.h>

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#endif /* RIDGE_SLAVE_ENABLE */

typedef struct
{
    ridge_reg_t                 ridge_stat;         /* Status register value reported to the Ridge. */
    ridge_reg_t                 cmd_reg;            /* Command register value provided by the Ridge. */
    uint8_t                     polarity;           /* Connection polarity. */
    hpd_event_cbk_t             hpd_cbk;            /* HPD callback used when using virtual HPD. */
    bool                        hpd_update_req;     /* Status of virtual HPD update. */
    ridge_ctrl_change_cb_t      ctrl_change_cb;     /* Callback to called when control register is changed. */
}ridge_t;

ridge_t ridge[NO_OF_TYPEC_PORTS];

/* Sets AR/TR lines as isolate */
static bool ridge_set_isolate(uint8_t port);
/* Sets AR/TR lines as USB 2.0 */
static bool ridge_set_2_0(uint8_t port);
/* Sets AR/TR lines as USB SS */
static bool ridge_set_usb_ss(uint8_t port);
/* Sets AR/TR as custom TBT configuration */
static bool ridge_set_custom(uint8_t port,uint32_t ridge_cfg);

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
/* Sets AR/TR lines as DP 2 lanes */
static bool ridge_set_dp_2_lane(uint8_t port);
/* Sets AR/TR lines as DP 4 lanes */
static bool ridge_set_dp_4_lane(uint8_t port, uint32_t pin_assign);
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

/************************** Function definitions *****************************/

static bool ridge_set_isolate(uint8_t port)
{
    ridge_reg_t*     reg = &ridge[port].ridge_stat;

    /* No connection active. */
    reg->val = NO_DATA;
    /* If data connection present. */
    if (dpm_get_info(port)->attach)
    {
        reg->val |= RIDGE_SAFE_STATE_MASK;
    }

    return ridge_set_custom(port, reg->val);
}

static bool ridge_set_2_0(uint8_t port)
{
    ridge_reg_t*     reg = &ridge[port].ridge_stat;

    /* Set Data Connection bit and USB 2.0 bit. */
    reg->val = RIDGE_DEBUG_MODE_MASK;

    return ridge_set_custom(port, reg->val);
}

static bool ridge_set_usb_ss(uint8_t port)
{
    ridge_reg_t *reg = &ridge[port].ridge_stat;
    pd_do_t   cbl_vdo;

    /* We don't support USB 3.0 connection as UFP. */
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        return false;
    }

    reg->val &= ~RIDGE_DATA_CONN_MASK;
    reg->val |= RIDGE_USB_STATE_MASK;

    /* Check if no USB 2.0 cable */
    cbl_vdo.val = dpm_get_info(port)->cbl_vdo.val;
    if ((cbl_vdo.val != NO_DATA) && (cbl_vdo.std_cbl_vdo.usb_ss_sup == USB_2_0_SUPP))
    {
        return ridge_set_2_0(port);
    }

    /* Set GEN2 speed only if the cable supports it. */
    if ((cbl_vdo.val == NO_DATA) || (cbl_vdo.std_cbl_vdo.usb_ss_sup == USB_GEN_2_SUPP))
    {
        reg->ridge_stat.usb3_speed = true;
    }

    /* Set AR/TR */
    return ridge_set_custom(port, reg->val);
}

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static bool ridge_set_dp_2_lane(uint8_t port)
{
    ridge_reg_t *reg = &ridge[port].ridge_stat;
    pd_do_t   cbl_vdo;

    reg->val &= ~RIDGE_DATA_CONN_MASK;
    reg->val |= RIDGE_DP_2_LANE_MASK; /* 2-lane connection. */
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        reg->ridge_stat.dp_role = true;
    }
    /* Set GEN2 speed only if the cable supports it. */
    cbl_vdo.val = dpm_get_info(port)->cbl_vdo.val;
    if ((cbl_vdo.val == NO_DATA) || (cbl_vdo.std_cbl_vdo.usb_ss_sup == USB_GEN_2_SUPP))
    {
        reg->ridge_stat.usb3_speed = true;
    }

    /* Set AR/TR */
    return ridge_set_custom(port, reg->val);
}

static bool ridge_set_dp_4_lane(uint8_t port,  uint32_t pin_assign)
{
    ridge_reg_t*     reg = &ridge[port].ridge_stat;

    reg->val &= ~RIDGE_DATA_CONN_MASK;
    reg->val |= RIDGE_DP_4_LANE_MASK; /* 4-lane connection. */
    /* CDT 286795 - trigger config bit if DP C pin assignment */
    if (pin_assign == DP_PIN_CONFIG_C)
    {
        reg->ridge_stat.dp_pin_assign = true;
    }
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        reg->ridge_stat.dp_role = true;
    }

    /* Set AR/TR */
    return ridge_set_custom(port, reg->val);
}
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

static bool ridge_set_custom(uint8_t port, uint32_t ridge_cfg)
{
    ridge_reg_t*     reg = &ridge[port].ridge_stat;

    /* Copy ar config to AR/TR struct */
    reg->val = ridge_cfg;
    /* Update polarity field. */
    reg->ridge_stat.conn_orien = ridge[port].polarity;

    /* Update data role field. */
    reg->ridge_stat.usb_dr = false;
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        reg->ridge_stat.usb_dr = true;
    }

#if RIDGE_SLAVE_ENABLE
    /* Update the status register content and raise an interrupt to the Alpine/Titan Ridge. */
    ridge_slave_status_update (port, reg->val, false);
#endif /* RIDGE_SLAVE_ENABLE */

    return true;
}

bool ridge_set_mux(uint8_t port, mux_select_t cfg, uint8_t polarity, uint32_t ridge_cfg)
{
    ridge[port].polarity = polarity;

    switch (cfg)
    {
        case MUX_CONFIG_ISOLATE:
            return ridge_set_isolate(port);
        case MUX_CONFIG_2_0:
            return ridge_set_2_0(port);
        case MUX_CONFIG_SS_ONLY:
            return ridge_set_usb_ss(port);

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
        case MUX_CONFIG_DP_2_LANE:
            return ridge_set_dp_2_lane(port);
        case MUX_CONFIG_DP_4_LANE:
            return ridge_set_dp_4_lane(port, ridge_cfg);
#endif /* ((DP_DFP_SUPP) || (DP_UFP_SUPP)) */

        case MUX_CONFIG_RIDGE_CUSTOM:
            return ridge_set_custom(port, ridge_cfg);
        default:
            break;
    }

    return false;
}

#if RIDGE_I2C_HPD_ENABLE

ccg_status_t tr_hpd_init(uint8_t port, hpd_event_cbk_t cbk)
{
    ccg_status_t     stat =  CCG_STAT_FAILURE;

    ridge[port].hpd_update_req = false;
    if (cbk != NULL)
    {
        ridge[port].hpd_cbk = cbk;
        stat = CCG_STAT_SUCCESS;
    }

    return stat;
}

void tr_hpd_deinit(uint8_t port)
{
    ridge[port].hpd_update_req = false;
    ridge[port].hpd_cbk = NULL;
}

void initiate_int_for_clear_bit(uint8_t port, timer_id_t id)
{
    ridge_reg_t*     reg          = &ridge[port].ridge_stat;
    reg->ridge_stat.irq_ack = false;
    ridge_slave_status_update(port, reg->val, true);
}

ccg_status_t tr_hpd_sendevt(uint8_t port, hpd_event_type_t evtype)
{
    ccg_status_t     stat         =  CCG_STAT_SUCCESS;
    ridge_reg_t*     reg          = &ridge[port].ridge_stat;

    ridge[port].hpd_update_req = true;
     /* Update HPD-out as required. */
    switch (evtype)
    {
        case HPD_EVENT_UNPLUG:
            reg->ridge_stat.hpd_lvl = false;
            reg->ridge_stat.hpd_irq = false;
            break;

        case HPD_EVENT_PLUG:
            ridge[port].hpd_update_req = false;
            if (reg->ridge_stat.hpd_lvl == false)
            {
                reg->ridge_stat.hpd_lvl = true;
                ridge[port].hpd_update_req = true;
            }
            break;

        case HPD_EVENT_IRQ:
            reg->ridge_stat.hpd_irq = true;
            reg->ridge_stat.hpd_lvl = true;
            break;

        case HPD_COMMAND_DONE:
            ridge[port].hpd_update_req = false;
            /*
             * Update IRQ_ACKfmPD bit when Sink to notify TR that Attention
             * command was sent successfully
             */
            if (dpm_get_info(port)->cur_port_type == PRT_TYPE_UFP)
            {
                reg->ridge_stat.irq_ack = true;
            }
            break;

        default:
            return CCG_STAT_BAD_PARAM;
    }

    /* Update Titan Ridge status */
    if (ridge[port].hpd_update_req != false)
    {
        ridge_slave_status_update (port, reg->val, true);
    }
    /* If HPD status update not required then go to next queue */
    else if (ridge[port].hpd_cbk != NULL)
    {
        ridge[port].hpd_cbk(port, HPD_COMMAND_DONE);
    }

    return stat;
}

bool tr_is_hpd_change(uint8_t port)
{
    return ridge[port].hpd_update_req;
}

#endif /* RIDGE_I2C_HPD_ENABLE */

void ridge_eval_cmd(uint8_t port, uint32_t stat, uint32_t stat_mask)
{
    /* Notify the APP layer about the control register change. */
    if (ridge[port].ctrl_change_cb != NULL)
    {
        ridge[port].ctrl_change_cb(port);
    }

    /* Go through all bits to find bits which were changed from the last time. */

    /* Check TBT conn bit */
    if (stat_mask & TBT_HOST_CONN_MASK)
    {
    }

    /* Check USB conn bit */
    if (stat_mask & TR_USB_HOST_CONN_MASK)
    {
    }

    /* Check DP conn bit */
    if (stat_mask & TR_DP_HOST_CONN_MASK)
    {
    }

#if ((RIDGE_I2C_HPD_ENABLE) && (DP_UFP_SUPP))
    if (ridge[port].hpd_cbk != NULL)
    {
        /* Check HPD level conn bit */
        if (stat_mask & TR_HPD_LVL_MASK)
        {
            if (stat & TR_HPD_LVL_MASK)
            {
                /* Set HPD High */
                ridge[port].hpd_cbk(port, HPD_EVENT_PLUG);
            }
            else
            {
                /* Set HPD Low */
                ridge[port].hpd_cbk(port, HPD_EVENT_UNPLUG);
            }
        }

        /* Check HPD IRQ bit */
        if ((stat_mask & TR_HPD_IRQ_MASK) && (stat & TR_HPD_IRQ_MASK))
        {
            ridge[port].hpd_cbk(port, HPD_EVENT_IRQ);
        }

        if (( (stat & TR_HPD_IRQ_MASK) == 0) && (stat_mask & TR_HPD_IRQ_MASK))
        {
            timer_start(port, APP_INITIATE_SEND_IRQ_CLEAR_ACK, APP_INITIATE_SEND_IRQ_CLEAR_ACK_PERIOD, initiate_int_for_clear_bit);
        }
    }
#endif /* ((RIDGE_I2C_HPD_ENABLE) && (DP_UFP_SUPP)) */
}

void ridge_set_ctrl_change_cb(uint8_t port, ridge_ctrl_change_cb_t cb)
{
    ridge[port].ctrl_change_cb = cb;
}

/* [] END OF FILE */
