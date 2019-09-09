/**
 * @file dp_sid.c
 *
 * @brief @{DisplayPort alternate mode handler source file.@}
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
#include <dp_sid.h>
#include <hpd.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>

typedef struct
{
    alt_mode_info_t info;
    pd_do_t vdo[MAX_DP_VDO_NUMB];
    dp_state_t state;
    mux_select_t dp_mux_cfg;
    pd_do_t config_vdo;
    uint8_t dp_active_flag;
    /*CCG's DP pin supported mask*/
    uint8_t ccg_dp_pins_supp;
    /*Port partner's DP pin supported mask*/
    uint8_t partner_dp_pins_supp;
    uint8_t dp_2_lane_active;
    uint16_t hpd_state;
    uint8_t queue_read_index;
    pd_do_t status_vdo;
#if DP_DFP_SUPP
    bool dp_exit;
    uint8_t dp_4_lane;
    uint8_t dp_2_lane;
    uint8_t dp_cfg_ctrl;
    uint8_t max_sop_supp;
    uint8_t cable_config_supp;
    const atch_tgt_info_t* tgt_info_ptr;
    bool dp_act_cbl_supp;
    pd_do_t cable_vdo[MAX_DP_VDO_NUMB];
#endif /* DP_DFP_SUPP */

}dp_status;

dp_status dp[NO_OF_TYPEC_PORTS];

#if DP_DFP_SUPP
/* Analyses DP DISC MODE info for DFP */    
static alt_mode_info_t* reg_dp_dfp(uint8_t port, alt_mode_reg_info_t *reg);
/* Analyse receved DP sink DISC MODE VDO */
static dp_state_t analyse_dp_sink_vdo(uint8_t port, pd_do_t svid_vdo);
/* Evaluates DP sink Status Update/Attention VDM */
static dp_state_t eval_ufp_status(uint8_t port);
/* Inits DP DFP alt mode */
static void init_dp(uint8_t port);
/* Checks if cable supports DP handling */
static bool is_cable_dp_capable(uint8_t port, const atch_tgt_info_t *atch_tgt_info);
/* HPD callback function */
static void dp_src_hpd_resp_cbk(uint8_t port, uint32_t event);
/* Add reseived DP source HPD status to queue */
static void dp_dfp_enqueue_hpd(uint8_t port, uint32_t status);
/* Dequues next HPD status */
static void dp_dfp_dequeue_hpd(uint8_t port);
/* Main DP Source alt mode function */
static void dp_dfp_run(uint8_t port);
/* Analyses received status update VDO */
static void analyse_status_update_vdo(uint8_t port);
/* Analyse if DFP and UFP DP modes consistent */
static bool is_prtnr_ccg_consistent(uint8_t port,uint8_t config);
/* Analyses DP DISC MODE info for the cable */    
static alt_mode_info_t* reg_dp_cbl(uint8_t port, alt_mode_reg_info_t *reg);    
/* Check if cable contains DP SVID */
static bool is_cable_dp_svid(const atch_tgt_info_t *atch_tgt_info);
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
/* HPD UFP callback function */
static void dp_snk_hpd_resp_cbk(uint8_t port, uint32_t cmd);
/* Main DP Sink alt mode function */
static void dp_ufp_run(uint8_t port);
/* Updates DP Sink Status */
static bool dp_ufp_update_status_field (uint8_t port, dp_stat_bm_t bit_pos, bool status);
/* Add reseived DP sink HPD status to queue */
static void dp_ufp_enqueue_hpd(uint8_t port, hpd_event_type_t status);
/* Dequues DP Sink HPD status */
static void dp_ufp_dequeue_hpd(uint8_t port);
/* Verifies is input configuration valid */
static bool is_config_correct(uint8_t port);
#endif /* DP_UFP_SUPP */

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
/* Composes VDM for sending by alt mode manager */
static void send_cmd(uint8_t port);
/* Returns pointer to DP alt mode cmd info structure */
static alt_mode_info_t* dp_info(uint8_t port);
/* Evaluates DP sink Gonfiguration command response */
static mux_select_t dp_eval_config(uint8_t port);
/* Exits DP alt mode */
static void dp_exit(uint8_t port);
/* Inits HPD functionality */
static void init_hpd(uint8_t port, uint8_t port_role);
/* Returns pointer to DP VDO buffer */
static pd_do_t* dp_get_vdo(uint8_t     port);
/* Resets DP variables when exits DP alt mode */
static void reset_dp_var(uint8_t port);
/* Stores given VDO in DP VDO buffer for sending */
static void dp_assign_vdo(uint8_t port, uint32_t vdo);
/* Evaluates received command from application */
static bool dp_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data);
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */

#if DP_DFP_SUPP
static void dp_set_app_evt(uint8_t port, uint32_t evtype, uint32_t evdata)
{
    dp[port].info.app_evt_needed = true;
    dp_info(port)->app_evt_data.alt_mode_event_data.evt_type = evtype;
    dp_info(port)->app_evt_data.alt_mode_event_data.evt_data = evdata;
}
#endif

/* Workaround for CCG4 Dock. This info is needed to enable DP config E */
#if ((CCG4_DOCK) && (DP_UFP_SUPP))
uint8_t get_dp_config(uint8_t port)
{
    return dp[port].dp_4_lane;
}
#endif /* CCG4_DOCK && DP_UFP_SUPP */

/********************* Alt modes manager register function ********************/

alt_mode_info_t* reg_dp_modes(uint8_t port, alt_mode_reg_info_t* reg_info)
{
    alt_mode_info_t* ret_info = NULL;
    reg_info->alt_mode_id = MODE_NOT_SUPPORTED;
    
    /* Check if DP SVID discover mode received VDO relates to DP alt mode */
    if (reg_info->svid_vdo.std_dp_vdo.rsvd == DP_ALT_MODE_ID)
    {
#if DP_UFP_SUPP
        /* If DP sink */
        if (reg_info->data_role == PRT_TYPE_UFP)
        {
            /* Reset DP info struct */
            reset_alt_mode_info(&(dp[port].info));
            reset_dp_var(port);
            reg_info->alt_mode_id = DP_ALT_MODE_ID;
            dp_info(port)->cbk = dp_ufp_run;
            dp_info(port)->VDM_HDR.svid = DP_SVID;
            dp_info(port)->vdo[SOP] = dp[port].vdo;
            dp_info(port)->vdo_max_numb = MAX_DP_VDO_NUMB;
            dp_info(port)->eval_app_cmd = dp_eval_app_cmd;
            /* Set application event */
            reg_info->app_evt = AM_EVT_ALT_MODE_SUPP;
            /* Get supported DP sink pin assigment. */
            if (reg_info->svid_vdo.std_dp_vdo.recep)
            {
                dp[port].ccg_dp_pins_supp = reg_info->svid_vdo.std_dp_vdo.ufp_d_pin;
            }
            else
            {
                dp[port].ccg_dp_pins_supp = reg_info->svid_vdo.std_dp_vdo.dfp_d_pin;
            }

            return &(dp[port].info);
        }
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
        /* If DP source */
        if (reg_info->atch_type == ATCH_TGT)
        {
            /* Analyze DFP DP Disc mode response */
           ret_info = reg_dp_dfp(port, reg_info);
        }
        else if (reg_info->atch_type == CABLE)
        {
            /* Analyze DP cable Disc mode response */
           ret_info = reg_dp_cbl(port, reg_info);
        }
#endif /* DP_DFP_SUPP */
    }

    return ret_info;
}

#if DP_DFP_SUPP
static alt_mode_info_t* reg_dp_dfp(uint8_t port, alt_mode_reg_info_t *reg)
{
    /* Reset DP info struct */
    reset_alt_mode_info(&(dp[port].info));
    reg->alt_mode_id = DP_ALT_MODE_ID;
    reset_dp_var(port);
    /* Set config ctrl as false at the start */
    dp[port].dp_cfg_ctrl = false;
    /* Check if cable discovered and if active cable has no fixed SS lines */
    if (is_cable_dp_capable(port, reg->atch_tgt_info) == false)
    {
        /* Set application event */
        reg->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
        return NULL;
    }
    /* Analyse sink VDO */
    dp[port].state = analyse_dp_sink_vdo(port, reg->svid_vdo);
    if (dp[port].state == DP_STATE_EXIT)
    {
        /* Set application event */
        reg->app_evt = AM_EVT_NOT_SUPP_PARTNER_CAP;
        return NULL;
    }

    /* Check if cable is active and supports PD 3.0 */
    if (dpm_get_info(port)->cbl_type == PROD_TYPE_ACT_CBL)
    {
        /* Check if cable has DP SVID */
        if (is_cable_dp_svid(reg->atch_tgt_info) == false)
        {
            /* SOP' disc svid is not needed */
            reg->cbl_sop_flag = SOP_INVALID;
            /* Set application event */
            reg->app_evt = AM_EVT_ALT_MODE_SUPP;
        }
        else
        {
            /* SOP' disc svid  needed */
            reg->cbl_sop_flag = SOP_PRIME;
        }
    }
    /* Copy cable, device/AMA info pointer */
    dp[port].tgt_info_ptr       = reg->atch_tgt_info;    
    dp[port].max_sop_supp       = SOP;

#if CCG_UCSI_ENABLE
    /* New event to notify the UCSI module */
    reg->app_evt = AM_EVT_SUPP_ALT_MODE_CHNG;
#endif /*CCG_UCSI_ENABLE*/    

    /* Notify application which DP MUX config is supported by both DFP and UFP */
    dp_set_app_evt(port, DP_ALLOWED_MUX_CONFIG_EVT,
            (dp[port].partner_dp_pins_supp & dp[port].ccg_dp_pins_supp));

    dp_info(port)->vdo_max_numb = MAX_DP_VDO_NUMB;
    /* Prepare DP Enter command */
    dp_info(port)->mode_state   = ALT_MODE_STATE_INIT;
    dp_dfp_run(port);

    return &(dp[port].info);
}
    
static alt_mode_info_t* reg_dp_cbl(uint8_t port, alt_mode_reg_info_t *reg)
{
    reg->alt_mode_id          = DP_ALT_MODE_ID;
    uint8_t cbl_supp_pin_asgn = reg->svid_emca_vdo.std_dp_vdo.dfp_d_pin | 
                reg->svid_emca_vdo.std_dp_vdo.ufp_d_pin;
    
    if (reg->cbl_sop_flag != SOP_INVALID)
    {
        if (
                (is_prtnr_ccg_consistent(port, cbl_supp_pin_asgn) &&
                (cbl_supp_pin_asgn & (DP_DFP_D_CONFIG_C | DP_DFP_D_CONFIG_D)))
            )
        {
            /* SOP' VDM needed */
            dp[port].max_sop_supp               = SOP_PRIME;
            /* Allow send SOP' packets */
            dp_info(port)->sop_state[SOP_PRIME] = ALT_MODE_STATE_SEND;
            /* Allow SOP'/SOP" DP handling */
            dp[port].dp_act_cbl_supp = true;
            /* Save supported cable pin assignment */
            dp[port].cable_config_supp = cbl_supp_pin_asgn;

            /* If SOP'' controller present (active cables only) allow SOP'' VDM */
            if ((dp[port].tgt_info_ptr->cbl_vdo->val != NO_DATA) &&
                    (dpm_get_info(port)->cbl_type == PROD_TYPE_ACT_CBL) &&
                    (dp[port].tgt_info_ptr->cbl_vdo->std_cbl_vdo.sop_dp == 1))
            {
                dp_info(port)->sop_state[SOP_DPRIME]   = ALT_MODE_STATE_SEND;
                dp[port].max_sop_supp                  = SOP_DPRIME;
            }

            /* Return SOP'' disc svid not needed */
            reg->cbl_sop_flag = SOP_INVALID;
            /* Set application event */
            reg->app_evt = AM_EVT_ALT_MODE_SUPP;
        }
        else
        {
            /* Set SOP'/SOP" failed */
            reg->cbl_sop_flag = SOP_INVALID;
        }
    }

    return &(dp[port].info);
}
#endif /* DP_DFP_SUPP */

/************************* DP Source Functions definitions ********************/

#if DP_DFP_SUPP
static void dp_dfp_run(uint8_t port)
{
    alt_mode_state_t dp_mode_state = dp_info(port)->mode_state;
    
    switch (dp_mode_state)
    {
        case ALT_MODE_STATE_INIT:
            /* Enable DP functionality */
            init_dp(port);
            /* Send Enter Mode command */
            send_cmd(port);
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            dp[port].state = (dp_state_t)dp_info(port)->VDM_HDR.cmd;
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            switch (dp[port].state)
            {
                case DP_STATE_ENTER:
                    /* Init HPD */
                    init_hpd(port, PRT_TYPE_DFP);
                    /* Go to Status update */
                    dp[port].state = DP_STATE_STATUS_UPDATE;
                    dp_assign_vdo(port, STATUS_UPDATE_VDO);
                    break;

                case DP_STATE_STATUS_UPDATE:
                    /* Analyse received VDO */
                    analyse_status_update_vdo(port);
                    if (dp[port].dp_cfg_ctrl == DP_MUX_CTRL_CMD)
                    {
                        return;
                    }
                    break;

                case DP_STATE_CONFIG:
                    /* If App controls DP MUX Config - send ACK to App */
                    if (dp[port].dp_cfg_ctrl == DP_MUX_CTRL_CMD)
                    {
                        dp_set_app_evt(port, DP_STATUS_UPDATE_EVT, DP_CFG_CMD_ACK_MASK);
                    }

                    /* Set MUX */
                    dp[port].dp_mux_cfg = dp_eval_config(port);
                    set_mux(port, dp[port].dp_mux_cfg, dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt);
                    /* Handle HPD queue */
                    if (dp[port].config_vdo.val == EMPTY_VDO)
                    {
                        dp_dfp_enqueue_hpd(port, EMPTY_VDO);
                    }
                    else
                    {
                        /* Add HPD to queue*/
                        dp_dfp_enqueue_hpd(port, dp[port].status_vdo.val);
                    }
                    
                    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                    /* CDT 251518 fix - set HPD to LOW when USB config is set */
                    /* Exit DP required */
                    if (dp[port].dp_exit != false)
                    {
                        dp[port].state = DP_STATE_EXIT;
                        dp[port].dp_exit = false;
                        break;
                    }
                    return;

                case DP_STATE_EXIT:
                    dp_exit(port);
                    return;

                default:
                    return;
            }
            send_cmd(port);
            break;

        case ALT_MODE_STATE_IDLE:
            /* Verify if input message is Attention */
            if ((dp_state_t)dp_info(port)->VDM_HDR.cmd == DP_STATE_ATT)
            {
                /* Handle HPD status */
                if (dp[port].dp_active_flag != false)
                {
                    /* Add HPD to queue*/
                    dp_dfp_enqueue_hpd(port, (*dp_get_vdo(port)).val);
                }
                /* Analyse received VDO */
                analyse_status_update_vdo(port);
                if (dp[port].dp_cfg_ctrl == DP_MUX_CTRL_CMD)
                {
                    return;
                }
                send_cmd(port);
            }
            break;

        case ALT_MODE_STATE_FAIL:
            if ((dp[port].state == DP_STATE_ENTER) || (dp[port].state == DP_STATE_EXIT))
            {
                dp_exit(port);
            }
            else if ((dp[port].state == DP_STATE_CONFIG) || (dp[port].state == DP_STATE_STATUS_UPDATE))
            {
                dp[port].state = DP_STATE_EXIT;
                send_cmd(port);
            }
            else
            {
                dp[port].state = DP_STATE_IDLE;
                dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            }
            break;

        case ALT_MODE_STATE_EXIT:
            dp[port].state = DP_STATE_CONFIG;
            dp[port].dp_exit = true;

            dp_assign_vdo(port, DP_USB_SS_CONFIG);
            send_cmd(port);
            break;

        default:
            break;
    }
}

static void analyse_status_update_vdo(uint8_t port)
{
    /* If App layer controls DP MUX Config */
    if (dp[port].dp_cfg_ctrl == DP_MUX_CTRL_CMD)
    {
        dp_set_app_evt(port, DP_STATUS_UPDATE_EVT, (*dp_get_vdo(port)).val);

        /* Set DP to idle and wait for config cmd from App */
        dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
        return;
    }

    /* CCG handles Status Update in auto mode. */
    dp[port].state = eval_ufp_status(port);
    if (dp[port].state == DP_STATE_CONFIG)
    {
        set_mux(port, MUX_CONFIG_ISOLATE, NO_DATA);
        if (dp[port].config_vdo.val != DP_USB_SS_CONFIG)
        {
            dp[port].config_vdo.dp_cfg_vdo.sign = DP_1_3_SIGNALING;
            dp[port].config_vdo.dp_cfg_vdo.sel_conf = DP_CONFIG_SELECT;
            if (dp[port].dp_act_cbl_supp != false)
            {
                /* Check if DP active cable could provide config pin assignment */
                if ((dp[port].cable_config_supp & dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt) == NO_DATA)
                {
                    dp[port].state =  DP_STATE_EXIT;
                    return;
                }
            }
        }
        dp_assign_vdo(port,dp[port].config_vdo.val);
    }
}

static dp_state_t analyse_dp_sink_vdo(uint8_t port, pd_do_t svid_vdo)
{
    dp[port].dp_4_lane = NO_DATA;
    dp[port].dp_2_lane = NO_DATA;
    
    if ((svid_vdo.std_dp_vdo.port_cap == DP_PORT_CAP_UFP_D) ||
            (svid_vdo.std_dp_vdo.port_cap == DP_PORT_CAP_BOTH))
    {
        /* Read DP_MODES_SUPPORTED in DP structure. */
        dp[port].ccg_dp_pins_supp = get_pd_port_config(port)->dp_config_supported;

        /* If DP Interface is presented on a USB TYPE C receptacle,
         * compare with UFP_D Pin Assignments advertised by UFP. */
        if (svid_vdo.std_dp_vdo.recep)
        {
            dp[port].partner_dp_pins_supp = svid_vdo.std_dp_vdo.ufp_d_pin;
        }
        else
        {
            dp[port].partner_dp_pins_supp = svid_vdo.std_dp_vdo.dfp_d_pin;
        }

        /* Check if either C/E is supported by both UFP and CCG. If yes.
         * we are good to go. */
        if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_C))
        {
            dp[port].dp_4_lane = DP_DFP_D_CONFIG_C;
        }
        else if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_E))
        {
            dp[port].dp_4_lane = DP_DFP_D_CONFIG_E;
        }
        else if (dp[port].partner_dp_pins_supp & DP_DFP_D_CONFIG_D)
        {
            dp[port].dp_2_lane = DP_DFP_D_CONFIG_D;
        }
        else
        {
            return DP_STATE_EXIT;
        }
    }
    else
    {
        /* UFP does not support UFP_D mode. DP not possible. */
        return DP_STATE_EXIT;
    }

    /* See if 2 lane Pin Configuration is supported by both devices.
     * If yes, store the preference. */
    if (dp[port].partner_dp_pins_supp & DP_DFP_D_CONFIG_D)
    {
        dp[port].dp_2_lane = DP_DFP_D_CONFIG_D;
    }
    else if (is_prtnr_ccg_consistent(port, DP_DFP_D_CONFIG_F))
    {
        dp[port].dp_2_lane = DP_DFP_D_CONFIG_F;
    }
    else
    {
        /* Multi Function not possible. */
        dp[port].dp_2_lane = DP_INVALID_CFG;
    }

    return DP_STATE_ENTER;
}

static dp_state_t eval_ufp_status(uint8_t port)
{
    dp_state_t ret = DP_STATE_CONFIG;
    bool       dp_mode = false;
    uint32_t   dp_asgn = DP_INVALID_CFG;

    pd_do_t status = *dp_get_vdo(port);
    dp[port].status_vdo = status;

    dp[port].config_vdo.val = 0;

    /* Handle the STATUS UPDATE sent by UFP. */
    if ((status.dp_stat_vdo.usb_cfg != 0) || (status.dp_stat_vdo.exit != 0))
    {
        dp[port].config_vdo.val = DP_USB_SS_CONFIG;
        /* If Exit request bit is set then send Exit cmd */
        if (status.dp_stat_vdo.exit != 0)
        {
            dp[port].dp_exit = true;
        }
    }
    else if (dp[port].dp_active_flag == false)
    {
        /* USB Configuration mode. */
        /* Check if UFP_D is connected. */
        if (status.dp_stat_vdo.dfp_ufp_conn > DP_CONN_DFP_D)
        {
            /* Check if Multi Function requested and CCG can support 2 lane. */
            if ((status.dp_stat_vdo.mult_fun != false) &&
                    (dp[port].dp_2_lane != DP_INVALID_CFG))
            {
                dp_asgn = dp[port].dp_2_lane;
            }
            else
            {
                dp_asgn = dp[port].dp_4_lane;
            }

            dp_mode = true;
        }
        else
        {
            ret = DP_STATE_IDLE;
        }
    }
    /* DP Config is active. */
    else
    {
        /* DP Config mode. */
        /* Check if UFP_D no longer connected. */
        if (status.dp_stat_vdo.dfp_ufp_conn < DP_CONN_UFP_D)
        {
            /* Move to USB Config. */
            dp[port].config_vdo.val = DP_USB_SS_CONFIG;
        }
        /* Check if DP sink is requesting for 2 lane and current
         * config is not 2 DP lane. */
        else if ((status.dp_stat_vdo.mult_fun != false) && (dp[port].dp_2_lane_active == false)
                && (dp[port].dp_2_lane != DP_INVALID_CFG))
        {
            /* Move to DP 2 lane. */
            dp_mode = true;
            dp_asgn = dp[port].dp_2_lane;
        }
        /* Check if DP Sink is requesting for NON-MULTI Funtion Mode
         * and current mode is MULTI FUNCTION. In this case switch to
         * 4 lane DP. */
        else if ((dp[port].dp_2_lane_active != false) && (status.dp_stat_vdo.mult_fun == false))
        {
            /* Move to DP 4 lane. */
            dp_mode = true;
            dp_asgn = dp[port].dp_4_lane;
        }
        else
        {
            ret = DP_STATE_IDLE;
        }
    }

    if (dp_mode)
    {
        dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt = dp_asgn;
    }

    return ret;
}

static bool is_cable_dp_capable(uint8_t port, const atch_tgt_info_t* atch_tgt_info)
{
    
    if ((atch_tgt_info->cbl_vdo != NULL) && (atch_tgt_info->cbl_vdo->val != NO_DATA))
    {

        /* DP cannot be supported if the USB mode supported is not 3.1 Gen1/Gen2; or
           if this is an ACTIVE CABLE and the directionality bits are not supported. */
        if (
                (atch_tgt_info->cbl_vdo->std_cbl_vdo.usb_ss_sup == USB_2_0_SUPP) || 
                (0
                // Active Cable WorkAround
                //    (dpm_get_info(port)->cbl_type == PROD_TYPE_ACT_CBL) 
                //&&((atch_tgt_info->cbl_vdo->val & CBL_DIR_SUPP_MASK) != CBL_DIR_SUPP_MASK)
                )
           )
        {
            return false;
        }
    }

    /* Allow DP mode to go through if no cable marker is found. */
    return true;
}

static bool is_cable_dp_svid(const atch_tgt_info_t* atch_tgt_info)
{
    uint8_t idx = 0;

    while (atch_tgt_info->cbl_svid[idx] != 0)
    {
        /* If Cable Intel SVID found */
        if (atch_tgt_info->cbl_svid[idx] == DP_SVID)
        {
            return true;
        }
        idx++;
    }

    return false;
}

static void init_dp(uint8_t port)
{
    dp_info(port)->eval_app_cmd = dp_eval_app_cmd;
    dp_info(port)->vdo_max_numb = MAX_DP_VDO_NUMB;
    dp_info(port)->vdo[SOP] = dp[port].vdo;

    /* Goto enter state */
    dp[port].state = DP_STATE_ENTER;

    /* No need to put MUX into SAFE state at mode entry as this is managed in CONFIG state. */
    dp_info(port)->set_mux_isolate = false;
    dp_info(port)->cbk = dp_dfp_run;

    dp_info(port)->vdo[SOP_PRIME]  = dp[port].cable_vdo;
    dp_info(port)->vdo[SOP_DPRIME] = dp[port].cable_vdo;
}

void dp_dfp_enqueue_hpd(uint8_t port, uint32_t status)
{
    uint8_t  prev_queue_idx = dp[port].queue_read_index;
    uint16_t hpd_state      = dp[port].hpd_state;
    uint8_t  hpd_idx        = dp[port].queue_read_index;

    /* Check whether IRQ queue is full */
    if (dp[port].queue_read_index < DP_QUEUE_FULL_INDEX)
    {
        switch(GET_HPD_IRQ_STAT(status))
        {
            case HPD_LOW_IRQ_LOW:
                /* Empty queue */
                hpd_idx = DP_QUEUE_STATE_SIZE;
                hpd_state = (uint16_t)HPD_EVENT_UNPLUG;
                prev_queue_idx = DP_QUEUE_EMPTY_INDEX; 
                break;
                
            case HPD_LOW_IRQ_HIGH:
                hpd_state <<= DP_QUEUE_STATE_SIZE;
                hpd_state |= (uint16_t) HPD_EVENT_IRQ;
                hpd_idx += DP_QUEUE_STATE_SIZE;
                break;
                
            case HPD_HIGH_IRQ_LOW:
                /* Add to queue High HPD state */
                hpd_state <<= DP_QUEUE_STATE_SIZE;
                hpd_state |= (uint16_t) HPD_EVENT_PLUG;
                hpd_idx += DP_QUEUE_STATE_SIZE;
                break;
                
            case HPD_HIGH_IRQ_HIGH:
                /*
                 * Add to queue HPD HIGH if previous HPD level was LOW or it's
                 * a first HPD transaction. In other cases add only HPD IRQ
                 * event to queue. 
                 */
                if 
                (
                    (prev_queue_idx == DP_QUEUE_EMPTY_INDEX) ||
                    ((hpd_event_type_t)(DP_HPD_STATE_MASK &
                        (hpd_state >> (hpd_idx - DP_QUEUE_STATE_SIZE))) == HPD_EVENT_UNPLUG)
                )
                {
                    hpd_state <<= DP_QUEUE_STATE_SIZE;
                    hpd_state |= (uint32_t) HPD_EVENT_PLUG;
                    hpd_idx += DP_QUEUE_STATE_SIZE;
                }

                hpd_state <<= DP_QUEUE_STATE_SIZE;
                hpd_state |= (uint32_t) HPD_EVENT_IRQ;
                hpd_idx += DP_QUEUE_STATE_SIZE;
                break;
                
            default:
                break;
        }
        dp[port].hpd_state = hpd_state;
        dp[port].queue_read_index = hpd_idx;
        /* Allow dequeue only if no HPD dequeueing in the process */
        if (prev_queue_idx == DP_QUEUE_EMPTY_INDEX)
        {
            dp_dfp_dequeue_hpd(port);
        }
    }
}

void dp_dfp_dequeue_hpd(uint8_t port)
{
    hpd_event_type_t hpd_evt = HPD_EVENT_NONE;
    uint16_t hpd_state       = dp[port].hpd_state;
    uint8_t  hpd_idx         = dp[port].queue_read_index;
    
    if (hpd_idx != DP_QUEUE_EMPTY_INDEX)
    {
        hpd_evt = (hpd_event_type_t)(DP_HPD_STATE_MASK &
                      (hpd_state >> (hpd_idx - DP_QUEUE_STATE_SIZE)));
        eval_hpd_cmd (port, (uint32_t)hpd_evt);
    }
}

static void dp_src_hpd_resp_cbk(uint8_t port, uint32_t event)
{
    if (((event & 0xFFFF) == HPD_COMMAND_DONE) && (dp_info(port)->is_active == true))
    {        
        if (dp[port].queue_read_index != DP_QUEUE_EMPTY_INDEX)
        {
            dp[port].queue_read_index -= DP_QUEUE_STATE_SIZE;
            dp_dfp_dequeue_hpd(port);
        }
    }
}
#endif /* DP_DFP_SUPP */

/************************* DP Sink Functions definitions **********************/

#if DP_UFP_SUPP

#if DP_GPIO_CONFIG_SELECT

/* Global to hold DP Pin configuration when GPIO based selection is enabled. */
uint8_t gl_dp_sink_config = 0;

void dp_sink_set_pin_config(uint8_t dp_config)
{
    /* Store DP Configuration supported as DP Sink. */
    gl_dp_sink_config = dp_config;
}

uint8_t dp_sink_get_pin_config(void)
{
    return gl_dp_sink_config;
}
#endif /* DP_GPIO_CONFIG_SELECT */

static void dp_ufp_run(uint8_t port)
{
    uint8_t dp_config;

    /* Get alt modes state and VDM command */
    alt_mode_state_t dp_mode_state = dp_info(port)->mode_state;
    dp[port].state = (dp_state_t)dp_info(port)->VDM_HDR.cmd;
    /* If exit all modes cmd received */
    if (dp_info(port)->VDM_HDR.cmd == EXIT_ALL_MODES)
    {
        dp[port].state = DP_STATE_EXIT;
    }
    switch (dp_mode_state)
    {
        case ALT_MODE_STATE_IDLE:

            switch(dp[port].state)
            {
                case DP_STATE_ENTER:
                    /* Enable HPD receiver */
                    init_hpd (port, PRT_TYPE_UFP);
                    /* Fill Status Update with appropiate bits */
                    dp[port].status_vdo.val = EMPTY_VDO;
                    /* Update UFP Enabled status field */
                    dp_ufp_update_status_field(port, DP_STAT_EN, true);
#if !DP_UFP_DONGLE
                    /* Update UFP connection status */
                    dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
#endif /* DP_UFP_MONITOR */

                    /* Check if Multi-function allowed */
                    if (((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_D) != NO_DATA) ||
                            ((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_F) != NO_DATA))
                    {
                        dp_ufp_update_status_field(port, DP_STAT_MF, true);
                    }
                    /* No VDO is needed in response */
                    dp_assign_vdo(port, NONE_VDO);
                    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                    return;

                case DP_STATE_STATUS_UPDATE:
                    /* Update current HPD status */
                    dp_ufp_update_status_field(port, DP_STAT_HPD, dp_snk_get_hpd_state(port));
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /*
                     * Update UFP Connected status based on DP Pin configuration. For NON-DP
                     * configuration, always report connected.
                     */
                    if (dp_sink_get_pin_config () == DP_DFP_D_CONFIG_C)
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, dp_snk_get_hpd_state(port));
                    }
#else
                    /*
                     * Update UFP Connected status based on DP Pin configuration. For NON-DP
                     * configuration, always report connected.
                     */
                    if (((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_C) != NO_DATA) ||
                            ((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_D) != NO_DATA))
                    {
                        /* Always report UFP connected. */
                        dp_ufp_update_status_field (port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        dp_ufp_update_status_field (port, DP_STAT_UFP_CONN,
                                dp_snk_get_hpd_state(port));
                    }
#endif /* DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */

                    /* CDT 257059 fix - IRQ bit should be set to zero in Status Update response */
                    dp_ufp_update_status_field (port, DP_STAT_IRQ, false);

                    /* Respond with current DP sink status */
                    dp_assign_vdo(port, dp[port].status_vdo.val);
                    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
                    return;

                case DP_STATE_CONFIG:
                    /* Check if Config VDO is correct */
                    if (
                            /* IF DP configuration */
                            ((dp_get_vdo(port)->dp_cfg_vdo.sel_conf == DP_CONFIG_SELECT) &&
                             (dp_get_vdo(port)->dp_cfg_vdo.sign == DP_1_3_SIGNALING)) ||
                            /* If USB configuration requested */
                            (dp_get_vdo(port)->dp_cfg_vdo.sel_conf == USB_CONFIG_SELECT)
                       )
                    {
                        /* Save port partner pin config */
                        dp[port].partner_dp_pins_supp = dp_get_vdo(port)->dp_cfg_vdo.dfp_asgmnt;

#if DP_GPIO_CONFIG_SELECT
                        /* Pin configuration supported is based on GPIO status. */
                        dp_config = dp_sink_get_pin_config ();
#else
                        /* Get DP Pin configuration supported from config table. */
                        dp_config = get_pd_port_config(port)->dp_config_supported;
#endif /* DP_GPIO_CONFIG_SELECT */

                        /* Check if both UFP and DFP support selected pin configuration */
                        if (
                                ((is_config_correct(port)) &&
                                 (dp_config & dp[port].partner_dp_pins_supp)) ||
                                 (dp_get_vdo(port)->dp_cfg_vdo.sel_conf == USB_CONFIG_SELECT)
                           )
                        {
                            /* Save config VDO */
                            dp[port].config_vdo = *(dp_get_vdo(port));
                            /* Get DP MUX configuration */
                            dp[port].dp_mux_cfg = dp_eval_config(port);

                            /* Set DP MUX */
                            set_mux(port, dp[port].dp_mux_cfg, dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt);
                            dp_assign_vdo(port, NONE_VDO);
                            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;

                            /* If HPD is HIGH then send Attention */
                            if 
                                (
#if (RIDGE_I2C_HPD_ENABLE == 0)
                                    (dp_snk_get_hpd_state(port) != false) &&
#endif /* RIDGE_I2C_HPD_ENABLE == 0 */                                    
                                    (dp[port].queue_read_index == NO_DATA)
                                )
                            {
                                /* If HPD already HIGH then add it to queue */
                                dp[port].hpd_state <<= DP_QUEUE_STATE_SIZE;
                                dp[port].hpd_state |= (uint32_t) HPD_EVENT_PLUG;
                                dp[port].queue_read_index += DP_QUEUE_STATE_SIZE;
                                dp_info(port)->mode_state = ALT_MODE_STATE_RUN; 
                            }   
                            return;
                        }
                    }
                    break;

                case DP_STATE_EXIT:
                    /* Deinit alt mode and reset dp variables */
                    dp_exit(port);
                    dp_assign_vdo(port, NONE_VDO);
                    return;

                default:
                    break;
            }
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            /* Analyse Attention callback */
            if (dp[port].state == DP_STATE_ATT)
            {
                /* Clear Request and IRQ status bits */
                dp_ufp_update_status_field(port, DP_STAT_USB_CNFG, false);
                dp_ufp_update_status_field(port, DP_STAT_EXIT, false);
                dp_ufp_update_status_field(port, DP_STAT_IRQ, false);
                dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;

#if RIDGE_I2C_HPD_ENABLE
                /* CDT 288790 fix */
                if(dp_get_vdo(port)->dp_stat_vdo.hpd_irq == 1)
#endif /* RIDGE_I2C_HPD_ENABLE */
                {                
                     eval_hpd_cmd (port, HPD_COMMAND_DONE);
                }

                /* Check if any HPD event in queue */
                dp_ufp_dequeue_hpd(port);
                return;
            }
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            return;
        case ALT_MODE_STATE_RUN:
            /* Set idle as default */
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            /* Send Attention if HPD is High while DP initialization */
            if (dp[port].hpd_state != NO_DATA)
            {
                dp_ufp_dequeue_hpd(port);
            }
            return;
        default:
            break;
    }

    /* Send NACK */
    dp_info(port)->mode_state = ALT_MODE_STATE_FAIL;
}

static bool is_config_correct(uint8_t port)
{
    bool retval = false;

    /* If input configuration is valid then return true */
    switch (dp[port].partner_dp_pins_supp)
    {
        case DP_DFP_D_CONFIG_C:
        case DP_DFP_D_CONFIG_D:
        case DP_DFP_D_CONFIG_E:
        case DP_DFP_D_CONFIG_F:
            retval = true;
            break;
        default:
            retval = false;
            break;
    }

    return retval;
}

static bool dp_ufp_update_status_field(uint8_t port, dp_stat_bm_t bit_pos, bool status)
{
    uint32_t    prev_status;

    /* Save current DP sink status */
    prev_status = dp[port].status_vdo.val;

    /* Update status field*/
    if (status != false)
    {
        SET_FLAG(dp[port].status_vdo.val, (uint8_t)bit_pos);
    }
    else
    {
        REMOVE_FLAG(dp[port].status_vdo.val, (uint8_t)bit_pos);
    }

    /* Check if status changed */
    return (prev_status != dp[port].status_vdo.val);
}

static void dp_snk_hpd_resp_cbk(uint8_t port, uint32_t  cmd)
{
    hpd_event_type_t event = (hpd_event_type_t)cmd;

    /* Handle hpd only when DP sink was entered */
    if (
            (event > HPD_EVENT_NONE)   &&
            (event < HPD_COMMAND_DONE) &&
            (dp_info(port)->is_active != false)
       )
    {
        /* If HPD received after Status update command was sent then add to HPD queue */
        dp_ufp_enqueue_hpd(port, event);
    }
}

static void dp_ufp_enqueue_hpd(uint8_t port, hpd_event_type_t status)
{
    uint16_t hpd_state       = dp[port].hpd_state;
    uint8_t  hpd_idx         = dp[port].queue_read_index;
    
    /* Check if queue is not full */
    if (hpd_idx <= (DP_UFP_MAX_QUEUE_SIZE * DP_QUEUE_STATE_SIZE))
    {
        switch (status)
        {
            case HPD_EVENT_UNPLUG:
                /* Empty queue */
                hpd_idx = DP_QUEUE_STATE_SIZE;
                hpd_state = (uint32_t) status;
                break;
            case HPD_EVENT_PLUG:
            case HPD_EVENT_IRQ:
                /* Add to queue High HPD state or IRQ */
                hpd_state <<= DP_QUEUE_STATE_SIZE;
                hpd_state |= (uint32_t) status;
                hpd_idx += DP_QUEUE_STATE_SIZE;
                break;

            default:
                return;
        }
        dp[port].hpd_state = hpd_state;
        dp[port].queue_read_index = hpd_idx;
        /* If any Attention already is in sending process then halt dequeue procedure */
        if (dp_info(port)->mode_state == ALT_MODE_STATE_IDLE)
        {
            /* Dequeue HPD events */
            dp_ufp_dequeue_hpd(port);
        }
    }
}

static void dp_ufp_dequeue_hpd(uint8_t port)
{
    hpd_event_type_t hpd_evt = HPD_EVENT_NONE;
    bool    is_att_needed    = false;
    uint16_t hpd_state       = dp[port].hpd_state;
    uint8_t  hpd_idx         = dp[port].queue_read_index;

    if (hpd_idx != (uint8_t)DP_QUEUE_EMPTY_INDEX)
    {
        /* Get queued HPD event */
        hpd_evt = (hpd_event_type_t)(DP_HPD_STATE_MASK & (hpd_state >>
                    (hpd_idx - DP_QUEUE_STATE_SIZE)));
        /* Decrease queue size */
        hpd_idx -= DP_QUEUE_STATE_SIZE;
        /* Get queued HPD event */
        switch(hpd_evt)
        {
            case HPD_EVENT_UNPLUG:
                /* Update DP sink status */
                if (dp_ufp_update_status_field(port, DP_STAT_HPD, false))
                {
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /* For DP Pin configuration, ensure that UFP Connected status bit is updated. */
                    if (dp_sink_get_pin_config () == DP_DFP_D_CONFIG_E)
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, false);
                    }
#else
                    if (((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_C) != NO_DATA) ||
                            ((dp[port].ccg_dp_pins_supp & DP_DFP_D_CONFIG_D) != NO_DATA))
                    {
                        /* Always report UFP connected. */
                        dp_ufp_update_status_field (port, DP_STAT_UFP_CONN, true);
                    }
                    else
                    {
                        /* Update UFP connection status */
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, false);
                    }
#endif /*  DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                /* Zero IRQ status if Unattached */
                dp_ufp_update_status_field(port, DP_STAT_IRQ, false);
                break;
            case HPD_EVENT_PLUG:
                /* Update DP sink status */
                if (dp_ufp_update_status_field(port, DP_STAT_HPD, true))
                {
#if DP_UFP_DONGLE
#if DP_GPIO_CONFIG_SELECT
                    /* For DP Pin configuration, ensure that UFP Connected status bit is updated. */
                    if (dp_sink_get_pin_config () == DP_DFP_D_CONFIG_E)
                    {
                        dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
                    }
#else
                    /* Update UFP connection status */
                    dp_ufp_update_status_field(port, DP_STAT_UFP_CONN, true);
#endif /*  DP_GPIO_CONFIG_SELECT */
#endif /* DP_UFP_DONGLE */
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                /* If next queued event is IRQ we can combine in one Attention */
                if ((((hpd_event_type_t)(DP_HPD_STATE_MASK & (hpd_state >>
                                        (hpd_idx - DP_QUEUE_STATE_SIZE)))) == HPD_EVENT_IRQ) &&
                        (hpd_idx != (uint8_t)DP_QUEUE_EMPTY_INDEX))
                {
                    /* Decrease queue size */
                    hpd_idx -= DP_QUEUE_STATE_SIZE;
                    /* Update IRQ field in status */
                    dp_ufp_update_status_field(port, DP_STAT_IRQ, true);
                    is_att_needed = true;
                }
                else
                {
                    /* Zero IRQ status */
                    dp_ufp_update_status_field(port, DP_STAT_IRQ, false);
                }
                break;
            case HPD_EVENT_IRQ:
                /* Update DP sink status */
                if (dp_ufp_update_status_field(port, DP_STAT_IRQ, true))
                {
                    /* Check flag to send Attention VDM */
                    is_att_needed = true;
                }
                break;
            default:
                break;
        }
        dp[port].hpd_state = hpd_state;
        dp[port].queue_read_index = hpd_idx;
        
        /* If status changed then send attention */
        if (is_att_needed != false)
        {
            /* Copy Status VDO to VDO buffer send Attention VDM */
            dp[port].state = DP_STATE_ATT;
            dp_assign_vdo(port, dp[port].status_vdo.val);
            send_cmd(port);
            return;
        }
        /* 
         * If Attention for current event not needed, but there are some events
         * left in queue then run dequeue 
         */
        else if (hpd_idx != (uint8_t)DP_QUEUE_EMPTY_INDEX)
        {
            dp_ufp_dequeue_hpd(port);
        }
    }
    dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
}
#endif /* DP_UFP_SUPP */

/************************* Common DP Functions definitions ********************/

#if ((DP_DFP_SUPP) || (DP_UFP_SUPP))
static void send_cmd(uint8_t port)
{
    sop_t sop_idx = SOP;
    
    if (dp[port].state != DP_STATE_IDLE)
    {
        dp_info(port)->vdm_header.val = NO_DATA;
        dp_info(port)->VDM_HDR.cmd    = (uint32_t)dp[port].state;
        dp_info(port)->VDM_HDR.svid   = DP_SVID;

#if DP_DFP_SUPP
        for (sop_idx = SOP; sop_idx <= dp[port].max_sop_supp; sop_idx++)
#endif /* DP_DFP_SUPP */
        {   
            dp_info(port)->vdo_numb[sop_idx] = ((dp[port].state == DP_STATE_EXIT) || (dp[port].state == DP_STATE_ENTER)) ? 
                NO_DATA : MAX_DP_VDO_NUMB;
            dp_info(port)->sop_state[sop_idx] = ALT_MODE_STATE_SEND;
            if ((sop_idx > SOP)  && (dp[port].state == DP_STATE_STATUS_UPDATE))
            {
                dp_info(port)->sop_state[sop_idx] = ALT_MODE_STATE_IDLE;
            }
        }
        dp_info(port)->mode_state = ALT_MODE_STATE_SEND;
    }
}

static alt_mode_info_t* dp_info(uint8_t port)
{
    return &(dp[port].info);
}

#if DP_DFP_SUPP
static bool is_prtnr_ccg_consistent(uint8_t port, uint8_t config)
{
    return (
            /* DP MUX pin */
            (dp[port].ccg_dp_pins_supp & dp[port].partner_dp_pins_supp & config) ||
            /* USB config */
            (config == NO_DATA)
       );
}
#endif /* DP_DFP_SUPP */

static mux_select_t dp_eval_config(uint8_t port)
{

    switch (dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt)
    {
        case DP_DFP_D_CONFIG_C:
        case DP_DFP_D_CONFIG_E:
#if CCG4_DOCK
        if(port == TYPEC_PORT_0_IDX)
            dp[port].dp_4_lane = dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt;
#endif
            dp[port].dp_active_flag = true;
            dp[port].dp_2_lane_active = false;
            return MUX_CONFIG_DP_4_LANE;
        case DP_DFP_D_CONFIG_D:
        case DP_DFP_D_CONFIG_F:
            dp[port].dp_active_flag = true;
            dp[port].dp_2_lane_active = true;
            return MUX_CONFIG_DP_2_LANE;
        case DP_USB_SS_CONFIG:
            dp[port].dp_active_flag = false;
            dp[port].dp_2_lane_active = false;
        default:
            break;
    }

    return MUX_CONFIG_SS_ONLY;
}

static void reset_dp_var(uint8_t port)
{
    /* Zeros DP flags */
    dp[port].dp_active_flag = false;
    dp[port].dp_2_lane_active = false;
    dp[port].config_vdo.val = EMPTY_VDO;
    dp[port].status_vdo.val = EMPTY_VDO;
    dp[port].hpd_state = false;
    dp[port].queue_read_index = false;
    dp[port].state = DP_STATE_IDLE;
    dp_assign_vdo(port, EMPTY_VDO);

#if DP_DFP_SUPP
    dp[port].dp_2_lane = false;
    dp[port].dp_4_lane = false;
    dp[port].dp_exit = false;
    dp[port].max_sop_supp = SOP;
    dp[port].tgt_info_ptr = NULL;
    dp[port].dp_act_cbl_supp = false;
#endif /* DP_DFP_SUPP */
}

static void dp_assign_vdo(uint8_t port, uint32_t vdo)
{
    /* No DP VDOs needed while send VDM */
    if (vdo == NONE_VDO)
    {
        dp_info(port)->vdo_numb[SOP]        = 0;
#if DP_DFP_SUPP
        dp_info(port)->vdo_numb[SOP_PRIME]  = 0;
        dp_info(port)->vdo_numb[SOP_DPRIME] = 0;
#endif /* DP_DFP_SUPP */
    }
    else
    {
        /* Include given VDO as part of VDM */
        dp[port].vdo[DP_VDO_IDX].val = vdo;
        dp_info(port)->vdo_numb[SOP] = MAX_DP_VDO_NUMB;

    #if DP_DFP_SUPP
        if (dp[port].dp_act_cbl_supp != false)
        {
            dp[port].cable_vdo[DP_VDO_IDX].val = vdo;
            dp_info(port)->vdo_numb[SOP_PRIME] = MAX_DP_VDO_NUMB;
            if (dp[port].max_sop_supp == SOP_DPRIME)
            {
                dp_info(port)->vdo_numb[SOP_DPRIME] = MAX_DP_VDO_NUMB;
            }
        }
    #endif /* DP_DFP_SUPP */
    }
}

static pd_do_t* dp_get_vdo(uint8_t port)
{
    return &(dp[port].vdo[DP_VDO_IDX]);
}

static void dp_exit(uint8_t port)
{
    reset_dp_var(port);
    eval_hpd_cmd(port, HPD_DISABLE_CMD);
    alt_mode_hw_set_cbk(port, NULL);
    dp_info(port)->mode_state = ALT_MODE_STATE_EXIT;
}

static void init_hpd(uint8_t port, uint8_t port_role)
{
    eval_hpd_cmd (port, HPD_ENABLE_CMD);
    dp[port].hpd_state = 0;
    dp[port].queue_read_index = 0;
    
#if DP_DFP_SUPP
    if (port_role != PRT_TYPE_UFP)
    {
        /* Register a HPD callback and send the HPD_ENABLE command. */
        alt_mode_hw_set_cbk (port, dp_src_hpd_resp_cbk);
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    if (port_role == PRT_TYPE_UFP)
    {
        /* Register a HPD callback and send the HPD_ENABLE command. */
        alt_mode_hw_set_cbk (port, dp_snk_hpd_resp_cbk);
    }
#endif /* DP_UFP_SUPP */
}

/****************************HPI command evaluation***************************/

static bool dp_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data)
{
#if ((DP_DFP_SUPP != 0) || (DP_UFP_SUPP != 0))
    uint32_t data = cmd_data.alt_mode_event_data.evt_data;
#endif /* ((DP_DFP_SUPP != 0) || (DP_UFP_SUPP != 0)) */

#if DP_DFP_SUPP
    uint32_t config = NO_DATA;
#endif /* DP_DFP_SUPP */
#if DP_UFP_SUPP
    bool is_att_needed = false;
#endif /* DP_UFP_SUPP */

#if DP_DFP_SUPP
    /* mux configuration command */
    if (cmd_data.alt_mode_event_data.evt_type == DP_MUX_CTRL_CMD)
    {
        dp[port].dp_cfg_ctrl = (data == DP_MUX_CTRL_CMD);
        return true;
    }
    /* DP config command received */
    else if ((cmd_data.alt_mode_event_data.evt_type == DP_APP_CFG_CMD) &&
            (dp[port].dp_cfg_ctrl != false))
    {
        if (data == DP_APP_CFG_USB_IDX)
        {
            dp[port].state = DP_STATE_CONFIG;

            dp[port].config_vdo.val = DP_USB_SS_CONFIG;
            dp_assign_vdo(port, dp[port].config_vdo.val);
            send_cmd(port);
        }
        else if(data <= DP_APP_CFG_CMD_MAX_NUMB)
        {
            /* Check if received DP config is possible */
            SET_FLAG(config, data);
            /* Check pin assigment compatibility */
            if ((is_prtnr_ccg_consistent(port, config)) &&
                    (dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt != config))
            {
                dp[port].state = DP_STATE_CONFIG;

                /* Prepare Config cmd and send VDM */
                dp[port].config_vdo.val = 0;
                dp[port].config_vdo.dp_cfg_vdo.dfp_asgmnt = config;
                dp[port].config_vdo.dp_cfg_vdo.sign = DP_1_3_SIGNALING;
                dp[port].config_vdo.dp_cfg_vdo.sel_conf = DP_CONFIG_SELECT;
                dp_assign_vdo(port, dp[port].config_vdo.val);
                send_cmd(port);
            }
        }
        else
        {
            return false;
        }
    }
#endif /* DP_DFP_SUPP */

#if DP_UFP_SUPP
    /* mux configuration command */
    if (cmd_data.alt_mode_event_data.evt_type == DP_SINK_CTRL_CMD)
    {
        /* Check if received command is Exit/USB Request */
        if ((data < DP_STAT_HPD) && (data > DP_STAT_MF))
        {
            /* Update DP sink status */
            if (dp_ufp_update_status_field(port, (dp_stat_bm_t)data, true))
            {
                /* Set send Attention flag */
                is_att_needed = true;
            }
        }

        /* Check if received command is enabling of Multi-function */
        if (data == DP_STAT_MF)
        {
            /* Check if MF is supported by CCG */
            if (
                    (dp[port].ccg_dp_pins_supp & (DP_DFP_D_CONFIG_D | DP_DFP_D_CONFIG_F)) &&
                    /* If status changed from previous */
                    (dp_ufp_update_status_field(port, DP_STAT_MF, true) != false)
               )
            {
                is_att_needed = true;
            }
        }

        /* Check if received command is disabling of Multi-function */
        if (
                (data == (DP_STAT_MF - 1)) &&
                (dp_ufp_update_status_field(port, DP_STAT_MF, false) != false)
           )
        {
            is_att_needed = true;
        }

        /* If status changed then send attention */
        if (is_att_needed != false)
        {
            /* Copy Status VDO to VDO buffer send Attention VDM */
            dp[port].state = DP_STATE_ATT;
            dp_assign_vdo(port, dp[port].status_vdo.val);
            send_cmd(port);
            return true;
        }
        else
        {
            dp_info(port)->mode_state = ALT_MODE_STATE_IDLE;
        }
    }
#endif /* DP_UFP_SUPP */

    return false;
}
#endif /* DP_DFP_SUPP || DP_UFP_SUPP */

/* [] END OF FILE */
