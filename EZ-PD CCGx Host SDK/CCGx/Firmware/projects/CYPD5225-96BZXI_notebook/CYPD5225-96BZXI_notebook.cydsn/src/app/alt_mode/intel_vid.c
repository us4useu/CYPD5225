/**
 * @file intel_vid.c
 *
 * @brief @{Thunderbolt (Intel VID) alternate mode handler source file.@}
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

#include <alt_modes_mngr.h>
#include <intel_vid.h>
#include <alt_mode_hw.h>
#include <dpm.h>
#include <config.h>
#include <intel_ridge.h>
    
#if ((TBT_DFP_SUPP) || (TBT_UFP_SUPP))

typedef struct
{
    alt_mode_info_t info;
    tbt_state_t state;
    pd_do_t vdo[MAX_TBT_VDO_NUMB];
    pd_do_t enter_mode_vdo;
    ridge_reg_t ridge_status;
    uint8_t max_sop_supp;

#if TBT_DFP_SUPP
    const atch_tgt_info_t* tgt_info_ptr;
#endif /* TBT_DFP_SUPP */
}tbt_status;

tbt_status tbt[NO_OF_TYPEC_PORTS];

#if TBT_DFP_SUPP
/* Analyses TBT DISC MODE info for DFP */    
static alt_mode_info_t* reg_tbt_dfp(uint8_t port, alt_mode_reg_info_t *reg);
/* Analyses TBT DISC MODE info for the cable */    
static alt_mode_info_t* reg_tbt_cbl(uint8_t port, alt_mode_reg_info_t *reg);
/* Generaze enter mode VDO */
static void generate_enter_mode_vdo(uint8_t port, pd_do_t svid_vdo, pd_do_t cable_vdo);
/* Composes VDM for sending by alt mode manager */
static void send_cmd(uint8_t port);
/* Inits TBT DFP alt mode */
static void init_tbt(uint8_t port);
/* Exits TBT alt mode */
static void tbt_exit(uint8_t port);
/* Main TBT source handling functions */
static void tbt_dfp_run(uint8_t port);
/* Evaluates receved TBT attention VDM */
static tbt_state_t eval_tbt_attention(uint8_t port);
/* Stores given VDO in TBT VDO buffer for sending */
void tbt_assign_vdo(uint8_t port, uint32_t vdo);
/* Returns pointer to TBT VDO buffer */
pd_do_t* tbt_get_vdo(uint8_t port);
/* Checks if cable supports TBT handling */
static bool is_cable_tbt_capable(uint8_t port, const atch_tgt_info_t *atch_tgt_info);
/* Check if cable contains TBT SVID */
static bool is_cable_tbt_svid(const atch_tgt_info_t *atch_tgt_info);
#endif /* TBT_DFP_SUPP */

#if TBT_UFP_SUPP
/* Main TBT sink handling functions */
static void tbt_ufp_run(uint8_t port);
#endif /* TBT_UFP_SUPP */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
/* Sets Ridge bits depending on current TBT state */
static bool tbt_set_regs_enter(uint8_t port);
/* Evaluates command received from application */
static bool tbt_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data);
/* Returns pointer to alt mode info structure */
static alt_mode_info_t* tbt_info(uint8_t port);
#endif

/************************** Function definitions *****************************/

alt_mode_info_t* reg_intel_modes(uint8_t port, alt_mode_reg_info_t* reg_info)
{
    if (reg_info->svid_vdo.tbt_vdo.intel_mode == TBT_ALT_MODE_ID)
    {
#if TBT_UFP_SUPP
        /* If TBT sink */
        if (reg_info->data_role == PRT_TYPE_UFP)
        {
            /* Reset TBT info struct */
            reset_alt_mode_info(&(tbt[port].info));
            reg_info->app_evt            = AM_EVT_ALT_MODE_SUPP;
            reg_info->alt_mode_id        = TBT_ALT_MODE_ID;
            tbt_info(port)->cbk          = tbt_ufp_run;
            tbt_info(port)->VDM_HDR.svid = INTEL_VID;
            tbt_info(port)->vdo[SOP]     = tbt[port].vdo;
            tbt_info(port)->vdo_max_numb = MAX_TBT_VDO_NUMB;
            tbt_info(port)->eval_app_cmd = tbt_eval_app_cmd;

            return &(tbt[port].info);
        }
#endif /* TBT_UFP_SUPP */

#if TBT_DFP_SUPP
        /* If TBT source */
        if (reg_info->atch_type == ATCH_TGT)
        {
            /* Analyze DFP TBT Disc mode response */
           return reg_tbt_dfp(port, reg_info);
        }
        else if (reg_info->atch_type == CABLE)
        {
            /* Analyze TBT cable Disc mode response */
           return reg_tbt_cbl(port, reg_info);
        }
#endif /* TBT_DFP_SUPP */
    }

    reg_info->alt_mode_id = MODE_NOT_SUPPORTED;
    return NULL;
}

#if TBT_DFP_SUPP

static alt_mode_info_t* reg_tbt_dfp(uint8_t port, alt_mode_reg_info_t *reg)
{
     /* Reset TBT info struct */
    reset_alt_mode_info(&(tbt[port].info));
    reg->alt_mode_id = TBT_ALT_MODE_ID;
    /* Check if cable discovered and if active cable has no fixed SS lines */
    if (is_cable_tbt_capable(port, reg->atch_tgt_info) == false)
    {
        /* Set application event */
        reg->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
        return NULL;
    }
    /* Check if cable has INTEL SVID */
    if (is_cable_tbt_svid(reg->atch_tgt_info) == false)
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
    /* Copy cable, device/AMA info pointer */
    tbt[port].tgt_info_ptr = reg->atch_tgt_info;

    /* Generate TBT enter mode VDO based on disc mode resp */
    generate_enter_mode_vdo(port, reg->svid_vdo, reg->svid_emca_vdo);
    tbt[port].max_sop_supp = SOP;
    /* Init TBT */
    tbt_info(port)->mode_state   = ALT_MODE_STATE_INIT;
    tbt_info(port)->vdo_max_numb = MAX_TBT_VDO_NUMB;
    tbt_dfp_run(port);
    
    return &tbt[port].info;
}
    
static alt_mode_info_t* reg_tbt_cbl(uint8_t port, alt_mode_reg_info_t *reg)
{
    if (reg->cbl_sop_flag != SOP_INVALID)
    {
        /* Generate TBT enter mode VDO based on disc mode resp */
        generate_enter_mode_vdo(port, reg->svid_vdo, reg->svid_emca_vdo);
        /* SOP' VDM needed */
        tbt[port].max_sop_supp               = SOP_PRIME;
        /* Allow send SOP' packets */
        tbt_info(port)->sop_state[SOP_PRIME] = ALT_MODE_STATE_SEND;

        /* If SOP'' controller present (active cables only) allow SOP'' VDM */
        if ((tbt[port].tgt_info_ptr->cbl_vdo->val != NO_DATA) &&
                (dpm_get_info(port)->cbl_type == PROD_TYPE_ACT_CBL) &&
                (tbt[port].tgt_info_ptr->cbl_vdo->std_cbl_vdo.sop_dp == 1))
        {
            tbt_info(port)->sop_state[SOP_DPRIME] = ALT_MODE_STATE_SEND;
            tbt[port].max_sop_supp                = SOP_DPRIME;
        }

        /* Init TBT */
        tbt_info(port)->mode_state = ALT_MODE_STATE_INIT;
        tbt_dfp_run(port);
    }
    else
    {
        /* If cable is active */
        if ((tbt[port].tgt_info_ptr->cbl_vdo->val != NO_DATA) &&
                (tbt[port].tgt_info_ptr->cbl_vdo->std_cbl_vdo.cbl_term >=
                 CBL_TERM_ONE_ACT_ONE_PAS_VCONN_REQ))
        {
            /* Set application event */
            reg->app_evt = AM_EVT_CBL_NOT_SUPP_ALT_MODE;
            /* Disable TBT functionality */
            tbt_info(port)->mode_state = ALT_MODE_STATE_DISABLE;
            
            return NULL;
        }
    }
    /* Return SOP'' disc svid not needed */
    reg->cbl_sop_flag = SOP_INVALID;
    /* Set application event */
    reg->app_evt = AM_EVT_ALT_MODE_SUPP;

    return (&tbt[port].info);
}
    
    
static void tbt_dfp_run(uint8_t port)
{
    switch (tbt_info(port)->mode_state)
    {
        case ALT_MODE_STATE_INIT:
            init_tbt(port);
            send_cmd(port);
            break;

        case ALT_MODE_STATE_WAIT_FOR_RESP:
            tbt[port].state = (tbt_state_t)tbt_info(port)->VDM_HDR.cmd;
            tbt_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            switch (tbt[port].state)
            {
                case TBT_STATE_ENTER:
                    tbt_set_regs_enter(port);
                    tbt[port].state = TBT_STATE_IDLE;
                    return;
                    
                case TBT_STATE_EXIT:
                    tbt_exit(port);
                    return;
                    
                default:
                    return;
            }
            break;

        case ALT_MODE_STATE_IDLE:
            /* Verify if input message is Attention */
            if ((tbt_state_t)tbt_info(port)->VDM_HDR.cmd == TBT_STATE_ATT)
            {
                tbt[port].state = eval_tbt_attention(port);
                send_cmd(port);
            }
            break;

        case ALT_MODE_STATE_FAIL:
            switch (tbt[port].state)
            {
                case TBT_STATE_ENTER:
                    /* Cbl enter was done */
                    if ((tbt_info(port)->sop_state[SOP] == ALT_MODE_STATE_FAIL) &&
                            (tbt[port].max_sop_supp != SOP))
                    {
                        tbt[port].state = TBT_STATE_EXIT;
                        send_cmd(port);
                        tbt_info(port)->sop_state[SOP] = ALT_MODE_STATE_DISABLE;
                    }
                    else
                    {
                        tbt_exit(port);
                    }
                    return;
                    
                case TBT_STATE_EXIT:
                    tbt_exit(port);
                    return;
                    
                default:
                    return;
            }
            break;

        case ALT_MODE_STATE_EXIT:
            tbt[port].state = TBT_STATE_EXIT;
            send_cmd(port);
            break;

        default:
            break;
    }
}

static void generate_enter_mode_vdo(uint8_t port, pd_do_t svid_vdo, pd_do_t cable_vdo)
{
    tbt[port].enter_mode_vdo.val                = NO_DATA;
    tbt[port].enter_mode_vdo.tbt_vdo.intel_mode = TBT_ALT_MODE_ID;
    tbt[port].enter_mode_vdo.tbt_vdo.leg_adpt   = GET_LEGACY_TBT_ADAPTER(svid_vdo.val);

    if (cable_vdo.val == NO_DATA)
    {
        /*
         * If cable does not provide a MODE VDO, we have to fill the parameters
         * with fields from the cable D_ID response. In case cable discovery is
         * turned off, indicate a simple 3.0 EMCA passive cable.
         */
        const dpm_status_t *dpm_ptr = dpm_get_info(port);

        if (dpm_ptr->cbl_dsc != false)
        {
            tbt[port].enter_mode_vdo.tbt_vdo.cbl_speed = dpm_ptr->cbl_vdo.std_cbl_vdo.usb_ss_sup;
            tbt[port].enter_mode_vdo.tbt_vdo.cbl       = (dpm_ptr->cbl_vdo.std_cbl_vdo.cbl_term >> 1);
        }
        else
        {
            /*
             * Since the cable discovery is turned off, we have to assume that
             * the product has a type-C plug and the cable supports all the
             * characteristics supported by the product. Indicate highest value.
             */
            tbt[port].enter_mode_vdo.tbt_vdo.cbl_speed = USB_GEN_2_SUPP;
        }
    }
    else
    {
        tbt[port].enter_mode_vdo.tbt_vdo.cbl_speed     = cable_vdo.tbt_vdo.cbl_speed;
        tbt[port].enter_mode_vdo.tbt_vdo.cbl_gen       = cable_vdo.tbt_vdo.cbl_gen;
        tbt[port].enter_mode_vdo.tbt_vdo.cbl_type      = cable_vdo.tbt_vdo.cbl_type;
        tbt[port].enter_mode_vdo.tbt_vdo.cbl           = cable_vdo.tbt_vdo.cbl;
        tbt[port].enter_mode_vdo.tbt_vdo.link_training = cable_vdo.tbt_vdo.link_training;
    }
}

static tbt_state_t eval_tbt_attention(uint8_t port)
{
    tbt_state_t  stat = TBT_STATE_IDLE;
    pd_do_t     *att  = tbt_get_vdo(port);

    /* If Exit is needed */
    if (TBT_EXIT(att->val))
    {
        stat = TBT_STATE_EXIT;
    }
    /* Restore TBT settings */
    else if ((BB_STATUS(att->val) == false) && (USB2_ENABLE(att->val) == false))
    {
        /* Set AR/TR MUX */
        set_mux(port, MUX_CONFIG_RIDGE_CUSTOM, tbt[port].ridge_status.val);
    }
    else if (USB2_ENABLE(att->val))
    {
        /* Set to USB 2.0 */
        set_mux(port, MUX_CONFIG_2_0, NO_DATA);
    }

    return stat;
}

static void init_tbt(uint8_t port)
{
    sop_t     idx;

    for (idx = 0; idx <= tbt[port].max_sop_supp; idx++)
    {
        tbt_info(port)->sop_state[idx] = ALT_MODE_STATE_SEND;
        tbt_info(port)->vdo[idx]       = tbt[port].vdo;
        /* TBT not uses VDO for cable while entered */
        if (idx > SOP)
        {
            tbt_info(port)->vdo_numb[idx] = NO_DATA;
        }
    }

    tbt_info(port)->cbk          = tbt_dfp_run;
    tbt_info(port)->eval_app_cmd = tbt_eval_app_cmd;

    /* Not setting MUX into safe state for TBT mode as it is causing failures when using Active Cables. */
    tbt_info(port)->set_mux_isolate = false;

    /* Update the VDO to be sent for enter mode. */
    tbt_assign_vdo(port, tbt[port].enter_mode_vdo.val);
    /* Set TBT state as enter */
    tbt[port].state = TBT_STATE_ENTER;
}

static void tbt_exit(uint8_t port)
{
    tbt_info(port)->mode_state = ALT_MODE_STATE_EXIT;
}

static bool is_cable_tbt_capable(uint8_t port, const atch_tgt_info_t* atch_tgt_info)
{
    /* Check for cable characteristics. */
    if (atch_tgt_info->cbl_vdo->val != NO_DATA)
    {
        /* TBT mode can be entered if an active cable with TBT mode support reports only USB 2.0 support. */
        if (
                (atch_tgt_info->cbl_vdo->std_cbl_vdo.usb_ss_sup == USB_2_0_SUPP) &&
                (!is_cable_tbt_svid(atch_tgt_info))
           )
        {
            return false;
        }

        return true;
    }

    return false;
}

static bool is_cable_tbt_svid(const atch_tgt_info_t* atch_tgt_info)
{
    uint8_t idx = 0;

    while (atch_tgt_info->cbl_svid[idx] != 0)
    {
        /* If Cable Intel SVID found */
        if (atch_tgt_info->cbl_svid[idx] == INTEL_VID)
        {
            return true;
        }
        idx++;
    }

    return false;
}
#endif /* TBT_DFP_SUPP */

void tbt_assign_vdo(uint8_t port, uint32_t vdo)
{
    /* No TBT VDOs needed while send VDM */
    if (vdo == NONE_VDO)
    {
        tbt_info(port)->vdo_numb[SOP] = NO_DATA;
    }
    else
    {
        /* Include given VDO as part of VDM */
        tbt[port].vdo[TBT_VDO_IDX].val = vdo;
        tbt_info(port)->vdo_numb[SOP] = MAX_TBT_VDO_NUMB;
    }
}

pd_do_t* tbt_get_vdo(uint8_t port)
{
    return &(tbt[port].vdo[TBT_VDO_IDX]);
}

static alt_mode_info_t* tbt_info(uint8_t port)
{
    return &(tbt[port].info);
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static void send_cmd(uint8_t port)
{
    if (tbt[port].state != TBT_STATE_IDLE)
    {
        sop_t sop_idx;
        tbt_info(port)->vdm_header.val = NO_DATA;
        tbt_info(port)->VDM_HDR.cmd    = (uint32_t)tbt[port].state;
        tbt_info(port)->VDM_HDR.svid   = INTEL_VID;
        if (tbt[port].state == TBT_STATE_EXIT)
        {
            /* Set isolate before send Exit cmd */
            set_mux(port, MUX_CONFIG_ISOLATE, NO_DATA);
            tbt_assign_vdo(port, NONE_VDO);
        }
        for (sop_idx = SOP; sop_idx <= tbt[port].max_sop_supp; sop_idx++)
        {
            tbt_info(port)->sop_state[sop_idx] = ALT_MODE_STATE_SEND;
        }
        tbt_info(port)->mode_state = ALT_MODE_STATE_SEND;
    }
}

static bool tbt_set_regs_enter(uint8_t port)
{
    ridge_reg_t* reg   = &tbt[port].ridge_status;
    pd_do_t* tbt_param = &tbt[port].enter_mode_vdo;

    /* Clear bits */
    reg->val                       = RIDGE_TBT_MODE_MASK;
    reg->ridge_stat.active_cbl     = tbt_param->tbt_vdo.cbl;
    reg->ridge_stat.cbl_type       = tbt_param->tbt_vdo.cbl_type;
    reg->ridge_stat.tbt_type       = tbt_param->tbt_vdo.leg_adpt;
    reg->ridge_stat.act_link_train = tbt_param->tbt_vdo.link_training;
    reg->ridge_stat.tbt_cbl_spd    = tbt_param->tbt_vdo.cbl_speed;
    reg->ridge_stat.tbt_cbl_gen    = tbt_param->tbt_vdo.cbl_gen;

    /* Set MUX */
    return set_mux(port, MUX_CONFIG_RIDGE_CUSTOM, tbt[port].ridge_status.val);

}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

/**********************TBT SINK ALT MODE FUNCTIONS****************************/

#if TBT_UFP_SUPP
static void tbt_ufp_run(uint8_t port)
{
    /* Get VDM command */
    tbt[port].state = (tbt_state_t)tbt_info(port)->VDM_HDR.cmd;
    /* If exit all modes cmd received */
    if (tbt_info(port)->VDM_HDR.cmd == EXIT_ALL_MODES)
    {
        tbt[port].state = TBT_STATE_EXIT;
    }
    switch(tbt_info(port)->mode_state)
    {
        case ALT_MODE_STATE_IDLE:
            switch(tbt[port].state)
            {
                case TBT_STATE_ENTER:
                    /* Check enter mode VDO */
                    if (tbt_get_vdo(port)->tbt_vdo.intel_mode == TBT_ALT_MODE_ID)
                    {
                        /* Copy Enter VDO to TBT struct */
                        tbt[port].enter_mode_vdo = *tbt_get_vdo(port);
                        /* Set AR MUX */
                        if (tbt_set_regs_enter(port))
                        {
                            tbt_info(port)->vdo_numb[SOP] = NO_DATA;
                            tbt_info(port)->mode_state    = ALT_MODE_STATE_IDLE;
                            return;
                        }
                    }
                    break;
                case TBT_STATE_EXIT:
                    tbt_assign_vdo(port, NONE_VDO);
                    tbt_info(port)->mode_state = ALT_MODE_STATE_EXIT;
                    return;
                default:
                    break;
                    /* Send NACK */
                    tbt_info(port)->mode_state = ALT_MODE_STATE_FAIL;
            }
        case ALT_MODE_STATE_WAIT_FOR_RESP:
            tbt_info(port)->mode_state = ALT_MODE_STATE_IDLE;
            return;
        default:
            break;
    }
    /* Send NACK */
    tbt_info(port)->mode_state = ALT_MODE_STATE_FAIL;
}
#endif /* TBT_UFP_SUPP */

/******************* TBT application Related Functions ************************/

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))

/* Activates only when application TBT command is received */
static bool tbt_eval_app_cmd(uint8_t port, alt_mode_evt_t cmd_data)
{
#if TBT_UFP_SUPP
    /* Save HPI attention cmd and send Attention VDM */
    tbt[port].state = TBT_STATE_ATT;
    tbt_assign_vdo(port, cmd_data.val);
    send_cmd(port);
    /* Att HPI command has the same structure as Att VDM in TBT spec */
    /* Restore TBT settings */
    if ((BB_STATUS(cmd_data.val) == false) && (USB2_ENABLE(cmd_data.val) == false))
    {
        /* Set AR MUX */
        set_mux(port, MUX_CONFIG_RIDGE_CUSTOM, tbt[port].ridge_status.val);
    }
    if (USB2_ENABLE(cmd_data.val))
    {
        /* Set to USB 2.0 */
        set_mux(port, MUX_CONFIG_2_0, NO_DATA);
    }
#endif /* TBT_UFP_SUPP */
    return true;
}

#endif /* DFP_ALT_MODE_SUPP || UFP_ALT_MODE_SUPP */

#endif /* ((TBT_DFP_SUPP) || (TBT_UFP_SUPP)) */

/* [] END OF FILE */
