/**
 * @file alt_modes_mngr.c
 *
 * @brief @{Alternate Mode Manager source file.@}
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

#include "config.h"
#include <alt_modes_mngr.h>
#include <alt_modes_config.h>
#include <pd.h>
#include <dpm.h>
#include <app.h>
#include <vdm.h>
#include <timer.h>
#include <vdm_task_mngr.h>
#include <vdm.h>

#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if ((MAX_SUPP_ALT_MODES < MAX_SVID_SUPP) && ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)))  
#error "The number of user alternate modes exceed maximum supported alt modes. \
	    Please increase MAX_SUPP_ALT_MODES"
#endif
/**
 * @typedef alt_mode_status_t
 * @brief struct to hold alt modes manager status
 */
typedef struct
{
    /* Holds info when register alt mode */
    alt_mode_reg_info_t   reg_info;
    /* Supported alternate modes. */
    uint32_t              am_supported_modes;
    /* Exited alternate modes. */
    uint32_t              am_exited_modes;
    /* Active alternate modes. */
    uint32_t              am_active_modes;
    /* Pointers to each alt mode info structure */
    alt_mode_info_t*      alt_mode_info[MAX_SUPP_ALT_MODES];
    /* Number of existed alt modes */
    uint8_t               alt_modes_numb;
    /* Buffer to hold VDM */
    dpm_pd_cmd_buf_t      vdm_buf;
    /* Holds application event data */
    uint32_t              app_evt_data[ALT_MODE_EVT_SIZE];
    /* Current alt modes mngr status */
    alt_mode_mngr_state_t state;
    /* Pointer to vdm_msg_info_t struct in vdm task mngr */
    vdm_msg_info_t       *vdm_info;
    /* Hold current SVID index for discovery mode command */
    uint8_t               svid_idx;
    /* Check whether the device is a PD 3.0 supporting UFP. */
    uint8_t               pd3_ufp;
    
}alt_mode_mngr_status_t;

/*Main structure to hold alt modes manager status */
alt_mode_mngr_status_t alt_mode[NO_OF_TYPEC_PORTS];

#if (!UCSI_ALT_MODE_ENABLED)
/* Composes alt mode info to vdm_msg_info_t struct before sending */
static uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type);
/* Parces received VDM info and moves it to alt mode info struct */
static void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* alt_mode_vdm, sop_t sop_type);
#endif /*UCSI_ALT_MODE_ENABLED*/    

#if DFP_ALT_MODE_SUPP
/* Find next avaliable alt mode for processing if previous exited */
static uint8_t get_next_alt_mode(uint8_t port);
#endif /* DFP_ALT_MODE_SUPP   */

#if UFP_ALT_MODE_SUPP
/* This function verifies possibility of entering of corresponding UFP alt mode */
static bool is_mode_activated(uint8_t port, const pd_packet_t *vdm);
/* UFP function for alt modes processing */
static bool ufp_reg_alt_mode(uint8_t port, const pd_packet_t *vdm);
#endif /* UFP_ALT_MODE_SUPP */

#if (!UCSI_ALT_MODE_ENABLED)
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))    
/* Returns alt mode index for given svid */
static uint8_t get_alt_mode_svid_idx(uint8_t port, uint16_t svid);
/* Returns alt mode index in compatibility table */
static uint8_t get_alt_mode_tbl_idx(uint8_t port, uint16_t svid, uint8_t id);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */    

/* Resets alt mode mngr info */
static void reset_mngr_info(uint8_t port);

/* Fast access functions */
alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx);

#endif /*UCSI_ALT_MODE_ENABLED*/ 

/* Alt modes mngr AMS Prototypes */
static vdm_task_t run_disc_mode(uint8_t port);
static vdm_task_t eval_disc_mode(uint8_t port);
static vdm_task_t disc_mode_fail(uint8_t port);
static vdm_task_t monitor_alt_modes(uint8_t port);
static vdm_task_t eval_alt_modes(uint8_t port);
static vdm_task_t fail_alt_modes(uint8_t port);
static vdm_task_t alt_mode_mngr_deinit(uint8_t port);

/*State Table*/
static vdm_task_t (*const alt_mode_ams_table [ALT_MODE_MNGR_STATE_PROCESS + 1]
        [VDM_EVT_EXIT + 1]) (uint8_t port) = {
    {
        /* Send next discovery svid */
        run_disc_mode,
        /* Evaluate disc svid response */
        eval_disc_mode,
        /* Process failed disc svid response */
        disc_mode_fail,
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit
    },
    {
        /* Monitor if any changes appears in modes  */
        monitor_alt_modes,
        /* Evaluate alt mode response */
        eval_alt_modes,
        /* Process failed alt modes response */
        fail_alt_modes,
        /* Exit from alt mode manager */
        alt_mode_mngr_deinit
    }
};

/************************* DFP Related Function definitions *******************/

vdm_task_t reg_alt_mode_mngr(uint8_t port, atch_tgt_info_t* atch_tgt_info, vdm_msg_info_t* vdm_msg_info)
{
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_live = (bool)(dpm_get_info(port)->spec_rev_sop_live >= PD_REV3);
#endif

    alt_mode[port].alt_modes_numb = get_alt_mode_numb(port);
    alt_mode[port].vdm_info       = vdm_msg_info;

    /* Check device role to start with. */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        if (atch_tgt_info->tgt_svid[0] == NO_DATA)
        {
            /* UFP does not support any of the SVIDs of interest. Exit VDM manager. */
            return VDM_TASK_EXIT;
        }
        else
        {
            if (get_alt_mode_numb(port) != false)
            {
                /* Register pointers to VDM mngr info */
                alt_mode[port].reg_info.atch_tgt_info  = atch_tgt_info;
                /* Set alt modes mngr state to Discovery Mode process */
                alt_mode[port].reg_info.data_role      = PRT_TYPE_DFP;
                alt_mode[port].state                   = ALT_MODE_MNGR_STATE_DISC_MODE;
                /* Set alt mode trigger based on config */
                app_get_status(port)->alt_mode_trig_mask = get_pd_port_config(port)->dp_mode_trigger;
                return VDM_TASK_ALT_MODE;
            }
        }
    }
    else
    {
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        alt_mode[port].pd3_ufp = pd3_live;
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

        if (atch_tgt_info->tgt_svid[0] == NO_DATA)
        {
            /* If we have no SVIDs to evaluate by UFP then go to regular monitoring */
            alt_mode[port].state    = ALT_MODE_MNGR_STATE_PROCESS;
            return VDM_TASK_ALT_MODE;
        }
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        else
        {
            if (pd3_live)
            {
                /* If PD spec revision is 3.0, we can start with discover mode process. */
                return VDM_TASK_ALT_MODE;
            }
        }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
    }

    return VDM_TASK_EXIT;
}

bool is_alt_mode_mngr_idle(uint8_t port)
{
    bool    is_idle = true;
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    uint8_t           alt_mode_idx;

    if ((alt_mode[port].state == ALT_MODE_MNGR_STATE_DISC_MODE) ||
        (!alt_mode_hw_is_idle(port)))
        return false;

    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            /* If alt mode not Idle then return current alt mode state */
            if ((get_mode_info(port, alt_mode_idx)->mode_state != ALT_MODE_STATE_IDLE) ||
                (get_mode_info(port, alt_mode_idx)->app_evt_needed != false))
            {
                is_idle = false;
            }
        }
    }
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

    return (is_idle);
}

void alt_mode_mngr_sleep(uint8_t port)
{
    alt_mode_hw_sleep (port);
}

void alt_mode_mngr_wakeup(uint8_t port)
{
    alt_mode_hw_wakeup (port);
}

vdm_task_t vdm_task_mngr_alt_mode_process(uint8_t port, vdm_evt_t vdm_evt)
{
    /* Run alt modes manager ams table */
    return alt_mode_ams_table[alt_mode[port].state][vdm_evt](port);
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

static void set_disc_mode_params(uint8_t port, sop_t sop)
{
    vdm_msg_info_t *vdm_p = alt_mode[port].vdm_info;

    vdm_p->vdm_header.val   = NO_DATA;
    vdm_p->VDM_HDR.svid     = alt_mode[port].reg_info.atch_tgt_info->tgt_svid[alt_mode[port].svid_idx];
    vdm_p->VDM_HDR.cmd      = VDM_CMD_DSC_MODES;
    vdm_p->VDM_HDR.obj_pos  = NO_DATA;
    vdm_p->sop_type         = sop;
    vdm_p->vdo_numb         = NO_DATA;
    vdm_p->VDM_HDR.vdm_type = VDM_TYPE_STRUCTURED;
}

static void send_sln_event_nodata(uint8_t port, uint16_t svid, uint8_t am_id, alt_mode_app_evt_t evtype)
{
    sln_pd_event_handler (port, APP_EVT_ALT_MODE,
            form_alt_mode_event (port, svid, am_id, evtype, NO_DATA)
            );
}

static void send_sln_app_evt(uint8_t port, uint32_t data)
{
    sln_pd_event_handler (port, APP_EVT_ALT_MODE,
            form_alt_mode_event (port, alt_mode[port].vdm_info->VDM_HDR.svid,
                alt_mode[port].reg_info.alt_mode_id,
                    (data == NO_DATA) ? alt_mode[port].reg_info.app_evt : AM_EVT_DATA_EVT,
                        data)
            );
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static vdm_task_t run_disc_mode(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    uint16_t cur_svid;

    /* Set cable sop flag not needed at start*/
    alt_mode[port].reg_info.cbl_sop_flag = SOP_INVALID;

    /* Search for next SVID until svid array is not empty */
    while ((cur_svid = alt_mode[port].reg_info.atch_tgt_info->tgt_svid[alt_mode[port].svid_idx]) != 0)
    {
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        /* Check is current port date role UFP */
        if (alt_mode[port].pd3_ufp)
        {
            /* Send Disc Mode cmd */
            set_disc_mode_params (port, SOP);
            return VDM_TASK_SEND_MSG;
        }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

#if DFP_ALT_MODE_SUPP
        if (is_svid_supported(cur_svid, port) != false)
        {
            /* Send notifications to the solution. */
            send_sln_event_nodata (port, cur_svid, 0, AM_EVT_SVID_SUPP);

            /* If SVID is supported send Disc Mode cmd */
            set_disc_mode_params (port, SOP);
            return VDM_TASK_SEND_MSG;
        }

#if (SAVE_SUPP_SVID_ONLY == 0)
        /* Send notifications to the solution. */
        send_sln_event_nodata (port, cur_svid, 0, AM_EVT_SVID_NOT_SUPP);
#endif /* SAVE_SUPP_SVID_ONLY */

        /* If svid not supported */
        alt_mode[port].svid_idx++;
#endif /* DFP_ALT_MODE_SUPP */
    }

    if (alt_mode[port].am_supported_modes == NONE_MODE_MASK)
    {
        /* No supp modes */
        return VDM_TASK_EXIT;
    }

    /* Send SVID discovery finished notification to the solution. */
    send_sln_event_nodata (port, 0, 0, AM_EVT_DISC_FINISHED);

#if DFP_ALT_MODE_SUPP
    /* Goto alt mode process */
    get_next_alt_mode(port);
#endif /* DFP_ALT_MODE_SUPP */
    alt_mode[port].state = ALT_MODE_MNGR_STATE_PROCESS;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
static void handle_cbl_disc_mode(uint8_t port, bool failed)
{
    uint8_t               tbl_svid_idx, tbl_alt_mode_idx, vdo_idx;
    alt_mode_info_t      *am_info_p;
    vdm_msg_info_t       *vdm_p = alt_mode[port].vdm_info;
    alt_mode_reg_info_t  *reg   = &(alt_mode[port].reg_info);
    /*
       Get index of function in alt_mode_config related to receive SVID.
       This is expected to be valid as we would already have gone through
       Discover MODE for the UFP.
     */
    tbl_svid_idx = get_alt_mode_svid_idx(port, vdm_p->VDM_HDR.svid);
    reg->atch_type = CABLE;

    /* Analyse all received VDOs. */
    for (vdo_idx = 0; ((failed) || (vdo_idx < vdm_p->vdo_numb)); vdo_idx++)
    {
        if (failed)
        {
            reg->cbl_sop_flag = SOP_INVALID;
        }
        else
        {
            /* Save current VDO and its position in svid structure */
            reg->svid_emca_vdo = vdm_p->vdo[vdo_idx];
        }

        /* Check if DFP support attached target alt mode */
        am_info_p = is_alt_mode_allowed[tbl_svid_idx] (port, reg);
        if (am_info_p == NULL)
        {
            /* Get index of alt mode in the compatibility table */
            tbl_alt_mode_idx = get_alt_mode_tbl_idx(port, vdm_p->VDM_HDR.svid, reg->alt_mode_id);
            if (tbl_alt_mode_idx != MODE_NOT_SUPPORTED)
            {
                /* Remove pointer to alt mode info struct */
                alt_mode[port].alt_mode_info[tbl_alt_mode_idx] = NULL;
                /* Remove flag that alt mode could be runned */
                REMOVE_FLAG(alt_mode[port].am_supported_modes, tbl_alt_mode_idx);
                reg->cbl_sop_flag = SOP_INVALID;
            }
        }
        else
        {
            if (!failed)
                am_info_p->cbl_obj_pos = (vdo_idx + VDO_START_IDX);
        }

        if (reg->app_evt != AM_NO_EVT)
        {
            /* Send notifications to the solution. */
            send_sln_app_evt (port, NO_DATA);
            reg->app_evt = AM_NO_EVT;
        }

        if (failed)
            break;
    }
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static vdm_task_t eval_disc_mode(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
    uint8_t               vdo_idx, tbl_svid_idx, tbl_alt_mode_idx;
    alt_mode_info_t      *am_info_p;
    vdm_msg_info_t       *vdm_p = alt_mode[port].vdm_info;
    alt_mode_reg_info_t  *reg   = &(alt_mode[port].reg_info);
    
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (alt_mode[port].pd3_ufp)
    {
        /* Goto next SVID */
        alt_mode[port].svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    /* Evaluate SOP response */
    if (vdm_p->sop_type == (uint8_t)SOP)
    {
        /* Get index of function in alt_mode_config related to receive SVID */
        tbl_svid_idx = get_alt_mode_svid_idx(port, vdm_p->VDM_HDR.svid);
        /* Additional check if we support current SVID operations */
        if (tbl_svid_idx != MODE_NOT_SUPPORTED)
        {
            reg->atch_type = ATCH_TGT;
            /* Analyse all rec VDOs */
            for (vdo_idx = 0; vdo_idx < vdm_p->vdo_numb; vdo_idx++)
            {
                /* Save current VDO and its position in svid structure */
                reg->svid_vdo = vdm_p->vdo[vdo_idx];
                /* Check if DFP support attached target alt mode */
                am_info_p = is_alt_mode_allowed[tbl_svid_idx](port, reg);
                /* If VDO relates to any of supported alt modes */
                if (reg->alt_mode_id != MODE_NOT_SUPPORTED)
                {
                    if (reg->app_evt != AM_NO_EVT)
                    {
                        /* Send notifications to the solution. */
                        send_sln_app_evt(port, NO_DATA);
                        reg->app_evt = AM_NO_EVT;
                    }
                    /* If alternate modes discovered and could be runned */
                    if (am_info_p != NULL)
                    {
                        /* Save alt mode ID and object position */
                        am_info_p->alt_mode_id = reg->alt_mode_id;
                        am_info_p->obj_pos     = (vdo_idx + VDO_START_IDX);
                        if (am_info_p->app_evt_needed != false)
                        {
                            /* Send notifications to the solution. */
                            send_sln_app_evt (port, am_info_p->app_evt_data.val);
                            am_info_p->app_evt_needed = false;
                        }
                        /* Get index of alt mode in the compatibility table */
                        tbl_alt_mode_idx = get_alt_mode_tbl_idx(port, vdm_p->VDM_HDR.svid,
                                reg->alt_mode_id);
                        if (tbl_alt_mode_idx != MODE_NOT_SUPPORTED)
                        {
                            /* Save pointer to alt mode info struct */
                            alt_mode[port].alt_mode_info[tbl_alt_mode_idx] = am_info_p;
                            /* Set flag that alt mode could be runned */
                            SET_FLAG(alt_mode[port].am_supported_modes, tbl_alt_mode_idx);
                        }
                    }
                }
            }

            /* If cable DISC Mode is needed - send VDM */
            if (reg->cbl_sop_flag != SOP_INVALID)
            {
                set_disc_mode_params (port, SOP_PRIME);
                return VDM_TASK_SEND_MSG;
            }
        }
    }
    /* Evaluate cable response: Packet type will be SOP' or SOP'' here. */
    else
    {
        handle_cbl_disc_mode(port, false);

        /* If cable DISC Mode is needed - send VDM */
        if (reg->cbl_sop_flag != SOP_INVALID)
        {
            set_disc_mode_params (port, SOP_DPRIME);
            return VDM_TASK_SEND_MSG;
        }
    }

    /* If no any result goto next SVID */
    alt_mode[port].svid_idx++;

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    return VDM_TASK_ALT_MODE;
}

static vdm_task_t disc_mode_fail(uint8_t port)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check is current port date role UFP */
    if (alt_mode[port].pd3_ufp)
    {
        /* Goto next SVID */
        alt_mode[port].svid_idx++;
        return VDM_TASK_ALT_MODE;
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

    if (alt_mode[port].vdm_info->sop_type == (uint8_t)SOP_PRIME)
    {
        handle_cbl_disc_mode(port, true);
    }
    /* If Disc Mode cmd fails goto next SVID */
    alt_mode[port].svid_idx++;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

    return VDM_TASK_ALT_MODE;
}

static vdm_task_t monitor_alt_modes(uint8_t port)
{
    uint8_t          alt_mode_idx;
    alt_mode_state_t alt_mode_state;
    vdm_task_t       stat = VDM_TASK_ALT_MODE;
    sop_t            sop_state;
    alt_mode_info_t  *am_info_p  = NULL;

    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info(port, alt_mode_idx);
            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            switch (alt_mode_state)
            {
            case ALT_MODE_STATE_SEND:
                /* This case activates when VDM sequence for given alt mode */
                /* was interrupted by alt mode with higher priority */
            case ALT_MODE_STATE_WAIT_FOR_RESP:
                /* Check if Trigger mask is set */
                if (am_info_p->VDM_HDR.cmd == VDM_CMD_ENTER_MODE)
                {
                    if (app_get_status(port)->alt_mode_trig_mask & (1 << alt_mode_idx))
                    {
                        /* 
                         * If enter EC trigger for alt mode is set then wait 
                         * for the EC alt mode Enter command 
                         */
                        continue;
                    }
                }

                /* Check if SOP' is needed */
                if (am_info_p->sop_state[SOP_PRIME] == ALT_MODE_STATE_SEND)
                {
                    sop_state = SOP_PRIME;
                }
                else if (am_info_p->sop_state[SOP_DPRIME] == ALT_MODE_STATE_SEND)
                {
                    sop_state = SOP_DPRIME;
                }
                else
                {
                    sop_state = SOP;
                }

                /* Check if MUX should and could be set*/
                if (
                        ((am_info_p->VDM_HDR.cmd == VDM_CMD_ENTER_MODE) ||
                         (am_info_p->VDM_HDR.cmd == VDM_CMD_EXIT_MODE)) &&
                        (am_info_p->set_mux_isolate != false)          &&
                        ((alt_mode[port].am_active_modes & (~(1 << alt_mode_idx))) == false)
                   )
                {
                    set_mux(port, MUX_CONFIG_ISOLATE, NO_DATA);
                }

                if ((am_info_p->is_active) || (am_info_p->VDM_HDR.cmd == VDM_CMD_ENTER_MODE))
                {
                    /* Copy to vdm info and send vdm */
                    move_to_vdm_info(port, am_info_p, sop_state);
                    am_info_p->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
                }

                return VDM_TASK_SEND_MSG;

            case ALT_MODE_STATE_EXIT:
#if DFP_ALT_MODE_SUPP
                if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
                {
                    /* Remove flag from active mode */
                    REMOVE_FLAG(alt_mode[port].am_active_modes, alt_mode_idx);
                    /* Set alt mode as disabled */
                    am_info_p->mode_state = ALT_MODE_STATE_DISABLE;
                    /* Set flag as exited mode */
                    SET_FLAG(alt_mode[port].am_exited_modes, alt_mode_idx);

                    /* If any active modes are present */
                    if (alt_mode[port].am_active_modes == NONE_MODE_MASK)
                    {
                        /* Notify APP layer that ALT mode has been exited. */
                        app_get_status(port)->alt_mode_entered = false;
                        /* Set MUX to SS config */
                        set_mux(port, MUX_CONFIG_SS_ONLY, NO_DATA);
                        /* Get next alt mode if avaliable */
                        get_next_alt_mode(port);
                    }
                }
#endif /* DFP_ALT_MODE_SUPP */

#if UFP_ALT_MODE_SUPP
                if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                {
                    if (alt_mode[port].am_active_modes != NONE_MODE_MASK)
                    {
                        /* Notify APP layer that ALT mode has been exited. */
                        app_get_status(port)->alt_mode_entered = false;
                    }

                    /* Send notifications to the solution if alt mode was entered. */
                    sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                            form_alt_mode_event (port,
                                am_info_p->VDM_HDR.svid,
                                am_info_p->alt_mode_id,
                                AM_EVT_ALT_MODE_EXITED, NO_DATA));

                    /* Set alt mode not active */
                    am_info_p->is_active = false;
                    /* Remove flag that alt mode could be processed */
                    REMOVE_FLAG(alt_mode[port].am_active_modes, alt_mode_idx);
                }
#endif /* UFP_ALT_MODE_SUPP   */
                break;

            case ALT_MODE_STATE_IDLE:
                /* If alt modes need to send app event data */
                if (am_info_p->app_evt_needed != false)
                {
                    /* Send notifications to the solution. */
                    sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                            form_alt_mode_event (port,
                                am_info_p->VDM_HDR.svid,
                                am_info_p->alt_mode_id,
                                AM_EVT_DATA_EVT, 
                                am_info_p->app_evt_data.val));
                    am_info_p->app_evt_needed = false;
                }
                break;

            case ALT_MODE_STATE_RUN:
                /* Run ufp evaluation function */
                am_info_p->cbk(port);
                break;

            default:
                break;

            }    
        }
    }

    return stat;
}

static vdm_task_t eval_alt_modes(uint8_t port)
{
    uint8_t            alt_mode_idx;
    alt_mode_state_t   alt_mode_state;
    vdm_task_t         stat = VDM_TASK_ALT_MODE;
    alt_mode_app_evt_t appevt_type = AM_NO_EVT;
    alt_mode_info_t    *am_info_p  = NULL;
    uint32_t           appevt_data = NO_DATA;
    vdm_msg_info_t     *vdm_p = alt_mode[port].vdm_info;
    
    /* Look through all alt modes  */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info(port, alt_mode_idx);

            /* If pointer to cbk is NULL then return */
            if (am_info_p->cbk == NULL) 
                break;

            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            switch (alt_mode_state)
            {
                case ALT_MODE_STATE_WAIT_FOR_RESP:
                    /* Set flag that send transaction passed successful */
                    am_info_p->sop_state[vdm_p->sop_type] = ALT_MODE_STATE_IDLE;
                    /* Copy received resp to alt mode info struct */
                    get_vdm_info_vdo(port, am_info_p, vdm_p->sop_type);
                    /* If received VDM is SOP */
                    if (vdm_p->sop_type == SOP)
                    {
                        /* Run alt mode analisys function */
                        am_info_p->cbk(port);
                        /* If UVDM command then break */
                        if (vdm_p->VDM_HDR.vdm_type != VDM_TYPE_STRUCTURED)
                        {
                            break;
                        }
                        /* If alt mode entered */
                        if (vdm_p->VDM_HDR.cmd == VDM_CMD_ENTER_MODE)
                        {
                            /* Notify APP layer that ALT mode has been entered. */
                            app_get_status(port)->alt_mode_entered = true;
                            /* Set mode as active */
                            am_info_p->is_active = true;
                            /* Queue notifications to the solution. */
                            appevt_type = AM_EVT_ALT_MODE_ENTERED;
                        }

                        if (vdm_p->VDM_HDR.cmd == VDM_CMD_EXIT_MODE)
                        {
                            /* Set mode as not active */
                            am_info_p->is_active = false;
                            /* Queue notifications to the solution. */
                            appevt_type = AM_EVT_ALT_MODE_EXITED;
                        }

                        /* If alt modes need to send app event data */
                        if (am_info_p->app_evt_needed != false)
                        {
                            /* Queue notifications to the solution. */
                            appevt_type = AM_EVT_DATA_EVT;
                            appevt_data = am_info_p->app_evt_data.val;
                            am_info_p->app_evt_needed = false;
                        }

                        /* Send event to solution space, if required. */
                        if (appevt_type != AM_NO_EVT)
                        {
                            sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                                    form_alt_mode_event (port,
                                        am_info_p->VDM_HDR.svid,
                                            am_info_p->alt_mode_id,
                                                appevt_type, appevt_data));
                        }
                    }
                    else
                    {
                        /* If received VDM is SOP' type, check if SOP'' is needed.
                           If received VDM is SOP'', the check on SOP_DPRIME will fail trivially. */
                        if (am_info_p->sop_state[SOP_DPRIME] == ALT_MODE_STATE_SEND)
                        {
                            /* If needed send SOP'' VDM */
                            move_to_vdm_info(port,am_info_p, SOP_DPRIME);
                            stat = VDM_TASK_SEND_MSG;
                        }
                        else if (am_info_p->sop_state[SOP] == ALT_MODE_STATE_SEND)
                        {
                            /* If SOP'' not needed - send SOP VDM */
                            move_to_vdm_info(port,am_info_p, SOP);
                            stat = VDM_TASK_SEND_MSG;
                        }
                        else
                        {
                            am_info_p->cbk(port);
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }

    return stat;
}

static vdm_task_t fail_alt_modes(uint8_t port)
{
    uint8_t             alt_mode_idx;
    alt_mode_state_t    alt_mode_state;
    alt_mode_app_evt_t  appevt_type = AM_EVT_CBL_RESP_FAILED;
    alt_mode_info_t     *am_info_p = NULL;
    vdm_msg_info_t      *vdm_p = alt_mode[port].vdm_info;
    
    /* Look through all alt modes */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is active */
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info(port, alt_mode_idx);
            /* Get alt mode state */
            alt_mode_state = am_info_p->mode_state;
            /* If mode waits for response */
            if (alt_mode_state == ALT_MODE_STATE_WAIT_FOR_RESP)
            {
                /* Change status to fail */
                am_info_p->sop_state[vdm_p->sop_type] = ALT_MODE_STATE_FAIL;
                /* Set Failure code at the object pos field */
                am_info_p->VDM_HDR.obj_pos = vdm_p->VDM_HDR.obj_pos;

                if (vdm_p->sop_type == SOP)
                {
                    appevt_type = AM_EVT_SOP_RESP_FAILED;
                }

                /* Send notifications to the solution. */
                sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                        form_alt_mode_event (port,
                            am_info_p->VDM_HDR.svid,
                                am_info_p->alt_mode_id, appevt_type, NO_DATA));

                /* Run alt mode analisys function */
                am_info_p->mode_state = ALT_MODE_STATE_FAIL;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port);
                }
            }
        }
    }

    return VDM_TASK_ALT_MODE;
}

#if DFP_ALT_MODE_SUPP
uint8_t is_svid_supported(uint16_t svid, uint8_t port)
{
    uint8_t idx, am_numb;
    const comp_tbl_t* tbl_addr = NULL;
    
    am_numb = get_alt_mode_numb(port);

    /* Look through all alt modes */
    for (idx = 0; idx < am_numb; idx++)
    {
#if ALT_MODE_DIFF_CONFIG        
        if (port == TYPEC_PORT_0_IDX)
#endif /* ALT_MODE_DIFF_CONFIG */            
        {
            tbl_addr = &dfp_compatibility_mode_table[idx][idx];
        }
#if ALT_MODE_DIFF_CONFIG
        else
        {
            tbl_addr = &dfp_compatibility_mode_table_p1[idx][idx];
        }
#endif /* ALT_MODE_DIFF_CONFIG */    
        /* if alt mode with given index is supported by CCG */
        if (tbl_addr->svid == svid)
        {
            return true;
        }
    }

    return false;
}

static uint8_t get_next_alt_mode(uint8_t port)
{
    uint32_t idx1, idx2;
    uint32_t am_numb    = alt_mode[port].alt_modes_numb;
    uint32_t aval_modes = (alt_mode[port].am_supported_modes) & (~alt_mode[port].am_exited_modes);
    const comp_tbl_t* tbl_addr = NULL;
    
    /* Set active modes bit map to zero */
    alt_mode[port].am_active_modes = NONE_MODE_MASK;
    /* Start looking for next supported alt modes */
    for (idx1 = 0; idx1 < am_numb; idx1++)
    {
        /* If mode supported by CCG and not processed yet */
        if (IS_FLAG_CHECKED((aval_modes), idx1))
        {
            /* Look for the alt modes which could be runned */
            for (idx2 = 0; idx2 < am_numb; idx2++)
            {
#if ALT_MODE_DIFF_CONFIG        
                if (port == TYPEC_PORT_0_IDX)
#endif /* ALT_MODE_DIFF_CONFIG */    
                {
                    tbl_addr = &dfp_compatibility_mode_table[idx1][idx2];
                }
#if ALT_MODE_DIFF_CONFIG
                else
                {
                    tbl_addr = &dfp_compatibility_mode_table_p1[idx1][idx2];
                }
#endif /* ALT_MODE_DIFF_CONFIG */    
                /* Check if this entry has a valid SVID. */
                if (tbl_addr->svid != NO_DATA)
                {
                    SET_FLAG(alt_mode[port].am_active_modes, idx2);
                }
            }

            return true;
        }
    }

    return false;
}

#endif /* DFP_ALT_MODE_SUPP   */

#if (!UCSI_ALT_MODE_ENABLED)
static uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t *info, sop_t sop_type)
#else
uint8_t move_to_vdm_info(uint8_t port, alt_mode_info_t *info, sop_t sop_type)
#endif /*UCSI_ALT_MODE_ENABLED*/
{
    vdm_msg_info_t *vdm_p = alt_mode[port].vdm_info;
    
    vdm_p->sop_type   = sop_type;
    vdm_p->vdo_numb   = NO_DATA;
    vdm_p->vdm_header = info->vdm_header;    
    if ((info->vdo[sop_type] != NULL) && (info->vdo_numb[sop_type] != NO_DATA))
    {
        vdm_p->vdo_numb = info->vdo_numb[sop_type];
        /* Save received VDO */
        memcpy(vdm_p->vdo, info->vdo[sop_type],(info->vdo_numb[sop_type]) << 2);
    }
    if (
            (info->uvdm_supp == false) 
    
    /* If SVDM */
#if UVDM_SUPP        
        ||  ((info->VDM_HDR.vdm_type == VDM_TYPE_STRUCTURED) 
        &&   (info->uvdm_supp))
#endif /* UVDM_SUPP */        
        )
    {
        vdm_p->VDM_HDR.vdm_type = VDM_TYPE_STRUCTURED;
        vdm_p->VDM_HDR.obj_pos = info->obj_pos;
        /* Set object position if custom Attention should be send */
        if ((info->VDM_HDR.cmd == VDM_CMD_ATTENTION) && (info->custom_att_obj_pos))
        {
            vdm_p->VDM_HDR.obj_pos = info->VDM_HDR.obj_pos;
        }
        if (sop_type != SOP)
        {
            vdm_p->VDM_HDR.obj_pos  = info->cbl_obj_pos;
        }
    }
    
    return true;
}

#if (!UCSI_ALT_MODE_ENABLED)
static void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* info, sop_t sop_type)
#else
void get_vdm_info_vdo(uint8_t port, alt_mode_info_t* info, sop_t sop_type)
#endif/*UCSI_ALT_MODE_ENABLED*/
{
    vdm_msg_info_t *vdm_p = alt_mode[port].vdm_info;
    uint8_t         vdo_numb;

#if UVDM_SUPP        
    if (info->VDM_HDR.vdm_type == VDM_TYPE_STRUCTURED)
#endif /* UVDM_SUPP */        
    {
        /* Copy object position to the VDM Header */
        info->VDM_HDR.obj_pos = vdm_p->VDM_HDR.obj_pos;
    }
    vdo_numb = vdm_p->vdo_numb;
    /* Copy received VDO to alt mode info */
    if ((vdo_numb != 0) && (info->vdo[sop_type] != NULL))
    {
        info->vdo_numb[sop_type] = vdm_p->vdo_numb;
        /* Save Rec VDO */
        if (vdm_p->vdo_numb <= info->vdo_max_numb)
        {
            /* Save received VDO */
            memcpy(info->vdo[sop_type], vdm_p->vdo,(vdm_p->vdo_numb) << 2);
        }
    }
}

static vdm_task_t alt_mode_mngr_deinit(uint8_t port)
{
    uint8_t             alt_mode_idx;
    alt_mode_info_t     *am_info_p = NULL;

    /* Find and reset alt modes info structures */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        am_info_p = get_mode_info(port, alt_mode_idx);
        if (am_info_p != NULL)
        {
            /* If current data role is UFP - set mode to idle */
            if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
            {
                /* CDT 232310 fix */
                am_info_p->mode_state = ALT_MODE_STATE_IDLE;
            }
            else
            {
                /* If DFP - wait for response */
                am_info_p->mode_state = ALT_MODE_STATE_WAIT_FOR_RESP;
            }

            /* Exit from alt mode */
            am_info_p->VDM_HDR.cmd = VDM_CMD_EXIT_MODE;
            am_info_p->cbk(port);

            /* Reset alt mode info */
            reset_alt_mode_info(am_info_p);
        }
    }

    /* Reset alt mode mngr info */
    reset_mngr_info(port);

#if (CCG_BB_ENABLE != 0)
    bb_disable(port, true);
    bb_update_all_status(port, BB_ALT_MODE_STATUS_INIT_VAL);
#endif /* (CCG_BB_ENABLE != 0) */

    return VDM_TASK_EXIT;
}


/******************** Common ALT Mode DFP and UFP functions *******************/

uint8_t get_alt_mode_numb(uint8_t port)
{
    uint8_t am_numb = NO_DATA;    

#if DFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        am_numb = DFP_MAX_SVID_SUPP;
#if ALT_MODE_DIFF_CONFIG
        if (port == TYPEC_PORT_1_IDX) 
        {
            am_numb = DFP_MAX_SVID_SUPP_P1;
        }
#endif /* ALT_MODE_DIFF_CONFIG */    
    }
#endif /* DFP_ALT_MODE_SUPP */

#if UFP_ALT_MODE_SUPP
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
    {
        am_numb = UFP_MAX_SVID_SUPP;
#if ALT_MODE_DIFF_CONFIG
        if (port == TYPEC_PORT_1_IDX)
        {
            am_numb = UFP_MAX_SVID_SUPP_P1;
        }
#endif /* ALT_MODE_DIFF_CONFIG */    
    }
#endif /* UFP_ALT_MODE_SUPP */

    return am_numb;
}

uint8_t is_alt_modes_available(uint8_t port)
{
    uint8_t stat = false;

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))    
    uint8_t idx;
    const comp_tbl_t* tbl_addr = NULL;
    port_type_t port_type      = gl_dpm_port_type[port];

    for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
    {
#if DFP_ALT_MODE_SUPP
        if (port_type != PRT_TYPE_UFP)
        {
            tbl_addr = &dfp_compatibility_mode_table[idx][idx];
        }
#endif

#if UFP_ALT_MODE_SUPP
        if (port_type == PRT_TYPE_UFP)
        {
            tbl_addr = &ufp_compatibility_mode_table[idx][idx];
        }
#endif

        /* if any alt mode is avaliable */
        if ((tbl_addr != NULL) && (tbl_addr->svid != NO_DATA))
        {
            if (port_type == PRT_TYPE_UFP)
            {
                /* Set svid idx to zero to start Disc Mode process */
                alt_mode[port].svid_idx = 0;
            }
            stat = true;
            break;
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */    

    return stat;
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#if (!UCSI_ALT_MODE_ENABLED)
static uint8_t get_alt_mode_tbl_idx(uint8_t port, uint16_t svid, uint8_t id)
#else
uint8_t get_alt_mode_tbl_idx(uint8_t port, uint16_t svid, uint8_t id)
#endif /*UCSI_ALT_MODE_ENABLED*/
{
    uint8_t idx;   
    const comp_tbl_t* tbl_addr = NULL;

    for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
    {
#if DFP_ALT_MODE_SUPP
        if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
        {
            tbl_addr = &dfp_compatibility_mode_table[idx][idx];
#if ALT_MODE_DIFF_CONFIG        
            if (port == TYPEC_PORT_1_IDX) 
            {
                tbl_addr = &dfp_compatibility_mode_table_p1[idx][idx];;
            }
#endif /* ALT_MODE_DIFF_CONFIG */                
        }
#endif
#if UFP_ALT_MODE_SUPP
        if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
        {
            tbl_addr = &ufp_compatibility_mode_table[idx][idx];
#if ALT_MODE_DIFF_CONFIG        
            if (port == TYPEC_PORT_1_IDX) 
            {
                tbl_addr = &ufp_compatibility_mode_table_p1[idx][idx];;
            }
#endif /* ALT_MODE_DIFF_CONFIG */    
        }
#endif
        /* if any alt mode is avaliable */
        if ((tbl_addr != NULL) && (tbl_addr->svid == svid) && (tbl_addr->alt_mode_id == id))
        {
            return idx;
        }
    }

    return MODE_NOT_SUPPORTED;
}

#if (!UCSI_ALT_MODE_ENABLED)
static uint8_t get_alt_mode_svid_idx(uint8_t port, uint16_t svid)
#else 
uint8_t get_alt_mode_svid_idx(uint8_t port, uint16_t svid)
#endif /*UCSI_ALT_MODE_ENABLED*/
{
    uint8_t idx;

    /* Look through all alt modes */
    for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
    {
        /* if alt mode with given index is supported by CCG */
        if (supp_svid_tbl[idx] == svid)
        {
            return idx;
        }
    }
    
    return MODE_NOT_SUPPORTED;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

void reset_alt_mode_info(alt_mode_info_t *info)
{
    memset(info, 0, sizeof(*info));
}

void reset_mngr_info(uint8_t port)
{
    memset(&alt_mode[port], 0, sizeof(alt_mode[port]));
}

dpm_pd_cmd_buf_t* get_vdm_buff(uint8_t port)
{
    return &(alt_mode[port].vdm_buf);
}

/******************* ALT MODE Solution Related Functions ****************************/

/* Function to exit all active alternate modes. */
void alt_mode_mngr_exit_all(uint8_t port)
{
#if DFP_ALT_MODE_SUPP
    uint8_t             alt_mode_idx;
    alt_mode_info_t     *am_info_p = NULL;

    if (gl_dpm_port_type[port] != PRT_TYPE_DFP)
    {
        /* Nothing to do if the port is not DFP. */
        return;
    }

    /* For each alternate mode, initiate mode exit. */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info (port, alt_mode_idx);
            if (am_info_p != NULL)
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk (port);
                }
            }
        }
    }
#endif /* DFP_ALT_MODE_SUPP */
}

#if ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE))

bool eval_app_alt_mode_cmd(uint8_t port, uint8_t *cmd, uint8_t *data)
{
    alt_mode_evt_t  cmd_info, cmd_data;
    alt_mode_info_t *am_info_p = NULL;
    uint8_t         alt_mode_idx;
    bool            found = false;

    /* Convert received cmd bytes as info and data */
    cmd_info = (alt_mode_evt_t)MAKE_DWORD(cmd[3], cmd[2], cmd[1], cmd[0]);
    cmd_data = (alt_mode_evt_t)MAKE_DWORD(data[3], data[2], data[1], data[0]);

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    /* Check if received app command is start discover process for UFP when PD 3.0 supported */
    if (
            (alt_mode[port].pd3_ufp) &&
            (cmd_info.alt_mode_event.data_role == PRT_TYPE_UFP) &&
            (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_RUN_UFP_DISC)
       )
    {
        /* Try to start Discovery process if VDM manager is not busy  */
        return is_ufp_disc_started(port);
    }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE)) */

    /* Look for the alternate mode entry which matches the SVID and alt mode id. */
    for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
    {
        /* If mode is supported. */
        if (IS_FLAG_CHECKED(alt_mode[port].am_supported_modes, alt_mode_idx))
        {
            am_info_p = get_mode_info (port, alt_mode_idx);
            if ((am_info_p->VDM_HDR.svid == cmd_info.alt_mode_event.svid) &&
                (am_info_p->alt_mode_id  == cmd_info.alt_mode_event.alt_mode))
            {
                found = true;
                break;
            }
        }
    }

#if DFP_ALT_MODE_SUPP
    if ((gl_dpm_port_type[port] != PRT_TYPE_UFP) && (cmd_info.alt_mode_event.data_role != PRT_TYPE_UFP))
    {
		if (cmd_info.alt_mode_event.alt_mode_evt == AM_SET_TRIGGER_MASK)
        {
			app_get_status(port)->alt_mode_trig_mask = (uint8_t)cmd_data.alt_mode_event_data.evt_data;
            return true;
        }
        /* Check if Enter command and trigger is set */
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_ENTER)
        {
            /* If we found an alternate mode entry for this {SVID, ID} pair. */
            if (found)
            {
                if ((am_info_p->cbk != NULL) && (am_info_p->is_active == false))
                {
                    /* Set flag as active mode */
                    SET_FLAG(alt_mode[port].am_active_modes, alt_mode_idx);
                    REMOVE_FLAG(alt_mode[port].am_exited_modes, alt_mode_idx);

                    /* Inits alt mode */
                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                    am_info_p->cbk(port);

                    /* Goto alt mode processing */
                    alt_mode[port].state = ALT_MODE_MNGR_STATE_PROCESS;
                    return true;
                }
            }
            return false;
        }
    }
#endif /* DFP_ALT_MODE_SUPP */

    if ((found) && (am_info_p->is_active == true) && (am_info_p->mode_state == ALT_MODE_STATE_IDLE) &&
         (cmd_info.alt_mode_event.data_role == gl_dpm_port_type[port]))
    {
        /* If received cmd is specific alt mode command */
        if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_SPEC)
        {
            return (am_info_p->eval_app_cmd(port, cmd_data));
        }
#if DFP_ALT_MODE_SUPP
        else if (cmd_info.alt_mode_event.alt_mode_evt == AM_CMD_EXIT)
        {
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                if (am_info_p->cbk != NULL)
                {
                    am_info_p->cbk(port);
                    return true;
                }
            }
        }
#endif /* DFP_ALT_MODE_SUPP */
    }

    return false;
}

#endif /* ((CCG_HPI_ENABLE) || (APP_ALTMODE_CMD_ENABLE)) */

const uint32_t* form_alt_mode_event(uint8_t port, uint16_t svid, uint8_t am_idx, alt_mode_app_evt_t evt, uint32_t data)
{
    alt_mode_evt_t temp;

    temp.alt_mode_event.svid                           = (uint32_t)svid;
    temp.alt_mode_event.alt_mode                       = (uint32_t)am_idx;
    temp.alt_mode_event.alt_mode_evt                   = (uint32_t)evt;
    temp.alt_mode_event.data_role                      = (uint32_t)gl_dpm_port_type[port];
    alt_mode[port].app_evt_data[ALT_MODE_EVT_IDX]      = temp.val;
    alt_mode[port].app_evt_data[ALT_MODE_EVT_DATA_IDX] = NO_DATA;

    if (data != NO_DATA)
    {
        alt_mode[port].app_evt_data[ALT_MODE_EVT_DATA_IDX] = data;
    }

    return alt_mode[port].app_evt_data;
}

alt_mode_info_t* get_mode_info(uint8_t port, uint8_t alt_mode_idx)
{
    return alt_mode[port].alt_mode_info[alt_mode_idx];
}

#if ( CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED )

const atch_tgt_info_t* get_atch(uint8_t port)
{
    return alt_mode[port].reg_info.atch_tgt_info;
}

vdm_msg_info_t* get_vdm_info(uint8_t port)
{
    return alt_mode[port].vdm_info;
}

alt_mode_state_t get_alt_mode_state(alt_mode_info_t* alt_mode_info_ptr)
{
    return (alt_mode_state_t)alt_mode_info_ptr->mode_state;
}

alt_mode_reg_info_t get_alt_mode_reg_info(uint8_t port)
{
    return alt_mode[port].reg_info;   
}    
    
uint32_t get_alt_mode_id(uint8_t port)
{
    return alt_mode[port].reg_info.alt_mode_id;
}

const comp_tbl_t get_tbl_info(uint8_t port, uint8_t idx)
{
#if UFP_ALT_MODE_SUPP  
    if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
        return ufp_compatibility_mode_table[idx][idx];
#endif
    return dfp_compatibility_mode_table[idx][idx];
}

uint32_t get_tbl_mode_info(uint8_t port, uint8_t idx) 
{
    return idx;
}

uint8_t is_alt_mode_active(uint8_t port, uint8_t alt_mode_idx)
{
    return IS_FLAG_CHECKED(alt_mode[port].am_supported_modes & (~alt_mode[port].am_exited_modes), alt_mode_idx);
}
 
void init_alt_mode(uint8_t port_idx, uint8_t alt_mode_idx)
{                
    alt_mode_info_t* am_info_p = get_mode_info(port_idx, alt_mode_idx);

	/* Set flag as active mode */
    SET_FLAG(alt_mode[port_idx].am_active_modes, alt_mode_idx);
    REMOVE_FLAG(alt_mode[port_idx].am_exited_modes, alt_mode_idx);

    /* Inits alt mode */
    am_info_p->mode_state = ALT_MODE_STATE_INIT;
    am_info_p->cbk(port_idx);

    /* Goto alt mode processing */
    alt_mode[port_idx].state = ALT_MODE_MNGR_STATE_PROCESS;
}

uint32_t get_supp_alt_modes(uint8_t port)
{
    return alt_mode[port].am_supported_modes;
}

void set_alt_mode_state(uint8_t port_idx, uint8_t alt_mode_idx)
{
	/* Set flag as active mode */
    SET_FLAG(alt_mode[port_idx].am_active_modes, alt_mode_idx);
    REMOVE_FLAG(alt_mode[port_idx].am_exited_modes, alt_mode_idx);
}
#endif /*CCG_UCSI_ENABLE && UCSI_ALT_MODE_ENABLED*/

#if UFP_ALT_MODE_SUPP

static bool ufp_reg_alt_mode(uint8_t port, const pd_packet_t *vdm)
{
    uint8_t              svid_idx, vdo_numb, alt_mode_idx = 0;
    pd_do_t             *dobj;
    alt_mode_info_t     *am_info_p = NULL;
    alt_mode_reg_info_t *reg       = &(alt_mode[port].reg_info);
    uint32_t             svid      = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid;
    uint32_t             obj_pos   = vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.obj_pos;

    /* Check if any alt mode is supported by DFP */
    if (get_alt_mode_numb(port) != false)
    {
        /* Get index of related svid register function */
        svid_idx = get_alt_mode_svid_idx(port, svid);
        /* If SVID supported by CCG */
        if (svid_idx != MODE_NOT_SUPPORTED)
        {
            reg->data_role = PRT_TYPE_UFP;
            /* Get SVID related VDOs and number of VDOs */
            if (get_modes_vdo_info (port, svid, &dobj, &vdo_numb) == false)
            {
                return false;
            }
            /* Check if object position is not greather than VDO numb */
            if ((obj_pos < vdo_numb) && (svid == supp_svid_tbl[svid_idx]))
            {
                /* Save Disc mode VDO and object position */
                reg->svid_vdo = dobj[obj_pos];
                /* Check if UFP support attached target alt mode */
                am_info_p = is_alt_mode_allowed[svid_idx](port, reg);
                /* If VDO relates to any of supported alt modes */
                if ((alt_mode[port].reg_info.alt_mode_id != MODE_NOT_SUPPORTED) && (am_info_p != NULL))
                {
                    /* If alternate modes discovered and could be runned */
                    /* Get index of alt mode in the compatibility table */
                    alt_mode_idx = get_alt_mode_tbl_idx(port, svid, reg->alt_mode_id);
                    if (alt_mode_idx != MODE_NOT_SUPPORTED)
                    {
                        /* Save alt mode ID and obj position */
                        am_info_p->alt_mode_id = reg->alt_mode_id;
                        am_info_p->obj_pos = obj_pos;
                        /* Save pointer to alt mode info struct */
                        alt_mode[port].alt_mode_info[alt_mode_idx] = am_info_p;
                        /* Set flag that alt mode could be runned */
                        SET_FLAG(alt_mode[port].am_supported_modes, alt_mode_idx);
                        return true;
                    }
                }
            }
#if (CCG_BB_ENABLE != 0)               
            else if (bb_is_present(port) != false)
            {
                /* Go throught all alt modes */
                for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
                {
                    am_info_p = get_mode_info(port, alt_mode_idx);
                    /* If Alt mode with corresponded SVID already active then don't notify BB device */
                    if (
                            (am_info_p != NULL) && 
                            (am_info_p->VDM_HDR.svid == vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                            (am_info_p->is_active == true)
                       )
                    {
                        return false;
                    }                     
                    /*
                     * If Alt mode with corresponded SVID not activated and 
                     * Enter Mode command has object position which not supported 
                     * by CCG then enumerate BB
                     */
                    /* Disable AME timeout timer. */
                    timer_stop(port, APP_AME_TIMEOUT_TIMER);
                    /* Update BB alt mode status register as Error */
                    bb_update_alt_status(port, svid_idx, BB_ALT_MODE_STAT_UNSUCCESSFUL);
                    /* Enable BB controller */
                    bb_enable(port, BB_CAUSE_AME_FAILURE);
                }
            }
#endif /* (CCG_BB_ENABLE != 0) */            
        }
    }

    return false;
}

static bool is_mode_activated(uint8_t port, const pd_packet_t *vdm)
{
    uint8_t      idx;

    /* If any alt mode already registered */
    if (alt_mode[port].am_supported_modes != NONE_MODE_MASK)
    {
        for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
        {
            /* Try to find alt mode among supported alt modes */
            if (
                    (alt_mode[port].alt_mode_info[idx] != NULL) &&
                    (get_mode_info(port, idx)->VDM_HDR.svid ==
                     vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid) &&
                    (get_mode_info(port, idx)->obj_pos ==
                     vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.obj_pos)
               )
            {
                /* return true if ufp alt mode already registered in alt mode mngr */
                return true;
            }
        }
    }

    /* Register alt mode and check possibility of alt mode entering
     * CDT 237168 */
    return ufp_reg_alt_mode(port, vdm);
}

#endif /* UFP_ALT_MODE_SUPP */

bool eval_rec_vdm(uint8_t port, const pd_packet_t *vdm)
{
    uint8_t         idx;
#if UFP_ALT_MODE_SUPP
    uint8_t         idx2;
    bool            enter_flag   = false;
    const comp_tbl_t* tbl_addr   = NULL;
#endif /* UFP_ALT_MODE_SUPP */
    alt_mode_info_t *am_info_p   = NULL;
    pd_do_t         vdm_header   = vdm->dat[VDM_HEADER_IDX];
    vdm_resp_t*     vdm_response = &(app_get_status(port)->vdm_resp);


    if (VDM_HDR.vdm_type == VDM_TYPE_STRUCTURED)
    {
        /* Discovery commands not processed by alt modes manager */
        if (VDM_HDR.cmd < VDM_CMD_ENTER_MODE)
        {
            return false;
        }
        /* Save number of available alt modes */
        alt_mode[port].alt_modes_numb = get_alt_mode_numb(port);

#if UFP_ALT_MODE_SUPP
        /* If Enter mode cmd */
        if (
                (VDM_HDR.cmd == VDM_CMD_ENTER_MODE) &&
                (gl_dpm_port_type[port] == PRT_TYPE_UFP)
           )
        {
            if (is_mode_activated(port, vdm) == true)
            {
                enter_flag = true;
            }
        }
#endif /* UFP_ALT_MODE_SUPP */
    }
    /* Go throught all alt modes */
    for (idx = 0; idx < alt_mode[port].alt_modes_numb; idx++)
    {
        am_info_p = get_mode_info(port, idx);

        /* Check if received command processing allowed */
        if (
                (am_info_p != NULL) && (am_info_p->VDM_HDR.svid == VDM_HDR.svid) &&
                (
                    (am_info_p->obj_pos == VDM_HDR.obj_pos) || 
                    (am_info_p->custom_att_obj_pos == true) || 
                    ((am_info_p->uvdm_supp == true) && 
                    (VDM_HDR.vdm_type == VDM_TYPE_UNSTRUCTURED))
                )
            )
        {
#if UFP_ALT_MODE_SUPP
            /* If Enter mode cmd */
            if (enter_flag == true)
            {
                /* If alt mode already active then ACK */
                if (am_info_p->is_active == true)
                {
                    return true;
                }

                /* If all alt modes not active and just entered  */
                if (alt_mode[port].am_active_modes == NONE_MODE_MASK)
                {
                    /* Save Header */
                    am_info_p->vdm_header    = vdm_header;
                    am_info_p->vdo_numb[SOP] = vdm->len - VDO_START_IDX;

                    if ((vdm->len > VDO_START_IDX) && (vdm->len <= (am_info_p->vdo_max_numb + VDO_START_IDX)))
                    {
                        /* Save received VDO */
                        memcpy(am_info_p->vdo[SOP], &(vdm->dat[VDO_START_IDX]),
                              (am_info_p->vdo_numb[SOP]) * PD_WORD_SIZE);
                    }

                    /* Run ufp alt mode cbk */
                    am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                    am_info_p->cbk(port);

                    if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
                    {
                        /* Set mode as active */
                        am_info_p->is_active = true;

                        /* Notify APP layer that ALT mode has been entered. */
                        app_get_status(port)->alt_mode_entered = true;

                        /* Send notifications to the solution if alt mode entered. */
                        sln_pd_event_handler (port, APP_EVT_ALT_MODE,
                                form_alt_mode_event (port,
                                    am_info_p->VDM_HDR.svid,
                                        am_info_p->alt_mode_id,
                                            AM_EVT_ALT_MODE_ENTERED, NO_DATA));

#if (CCG_BB_ENABLE != 0)
                        if (bb_is_present(port) != false)
                        {
                            /* Disable AME timeout timer. */
                            timer_stop(port, APP_AME_TIMEOUT_TIMER);
                            /* Enable BB controller */
                            bb_enable(port, BB_CAUSE_AME_SUCCESS);
                            /* Update BB alt mode status register as successful config */
                            bb_update_alt_status(port, idx, BB_ALT_MODE_STAT_SUCCESSFUL);
                        }
#endif /* (CCG_BB_ENABLE != 0) */

                        /* Set flag that alt mode could be processed */
                        SET_FLAG(alt_mode[port].am_active_modes, idx);
                        return true;
                    }

#if (CCG_BB_ENABLE != 0)
                    if (bb_is_present(port) != false)
                    {
                        /* Disable AME timeout timer. */
                        timer_stop(port, APP_AME_TIMEOUT_TIMER);
                        /* Enable BB controller */
                        bb_enable(port, BB_CAUSE_AME_FAILURE);
                        /* Update BB alt mode status register as Error */
                        bb_update_alt_status(port, idx, BB_ALT_MODE_STAT_UNSUCCESSFUL);
                    }
#endif /* (CCG_BB_ENABLE != 0) */
                    return false;
                }
                else
                {
                    /* Check alt mode in ufp consistent table  */
                    for (idx2 = 0; idx2 < alt_mode[port].alt_modes_numb; idx2++)
                    {
                        /* find table index of any active alt mode */
                        if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, idx2))
                        {
                            tbl_addr = &ufp_compatibility_mode_table[idx][idx2];
#if ALT_MODE_DIFF_CONFIG    
                            if (port == TYPEC_PORT_1_IDX)
                            {
                                tbl_addr = &ufp_compatibility_mode_table_p1[idx][idx2];
                            }
#endif /* ALT_MODE_DIFF_CONFIG */                 
                            /* If alt mode consistent with active alt mode */
                            if (tbl_addr->svid != NO_DATA)
                            {
                                /* Save cmd */
                                am_info_p->VDM_HDR.cmd = VDM_HDR.cmd;
                                am_info_p->vdo_numb[SOP] = vdm->len - VDO_START_IDX;
                                /* Save Rec VDO */
                                if ((vdm->len > VDO_START_IDX) && (vdm->len <= (am_info_p->vdo_max_numb + VDO_START_IDX)))
                                {
                                    /* Copy received VDO */
                                    memcpy(am_info_p->vdo[SOP], &(vdm->dat[VDO_START_IDX]),
                                            (am_info_p->vdo_numb[SOP]) * PD_WORD_SIZE);
                                }
                                /* Run ufp alt mode cbk */
                                am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                                am_info_p->cbk(port);
                                if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
                                {
                                    /* Set mode as active */
                                    am_info_p->is_active = true;
                                    /* Set flag that alt mode could be processed */
                                    SET_FLAG(alt_mode[port].am_active_modes, idx);
                                    return true;
                                }
                                return false;
                            }
                        }
                    }
                    return false;
                }
            }
            /* Any other received command */
            else
#endif /* UFP_ALT_MODE_SUPP */
            {
                /* Check if alt mode is active */
                if (am_info_p->is_active == false)
                {
                    return false;
                }

                /* Save cmd */
                am_info_p->vdm_header    = vdm_header;
                am_info_p->vdo_numb[SOP] = vdm->len - VDO_START_IDX;
                /* Save custom attention object position if needed */
                if (
                        (VDM_HDR.cmd == VDM_CMD_ATTENTION) && 
                        (am_info_p->custom_att_obj_pos)    &&
                        (VDM_HDR.vdm_type == VDM_TYPE_STRUCTURED)
                    )
                {
                    am_info_p->VDM_HDR.obj_pos = VDM_HDR.obj_pos;
                }

                /* Save Rec VDO */
                if ((vdm->len > VDO_START_IDX) &&
                        (vdm->len <= (am_info_p->vdo_max_numb + VDO_START_IDX)))
                {
                    /* Save received VDO */
                    memcpy(am_info_p->vdo[SOP], &(vdm->dat[VDO_START_IDX]),
                            ((vdm->len - VDO_START_IDX) * PD_WORD_SIZE));
                }

                /* Set alt mode state as idle */
                am_info_p->mode_state = ALT_MODE_STATE_IDLE;
                /* Run alt mode cbk */
                am_info_p->cbk(port);

                /* If command processed successful */
                if (am_info_p->mode_state != ALT_MODE_STATE_FAIL)
                {
                    /* Copy VDM header to respond buffer if Unstructured*/
                    if (am_info_p->VDM_HDR.vdm_type == VDM_TYPE_UNSTRUCTURED)
                    {
                        vdm_response->resp_buf[VDM_HEADER_IDX] = am_info_p->vdm_header;
                    }
                    /* Set number of data objects */
                    vdm_response->do_count = am_info_p->vdo_numb[SOP] + VDO_START_IDX;
                    if (am_info_p->vdo_numb[SOP] != NO_DATA)
                    {
                        /* If VDO resp is needed */
                        memcpy( &(vdm_response->resp_buf[VDO_START_IDX]),
                                am_info_p->vdo[SOP],
                                (am_info_p->vdo_numb[SOP]) * PD_WORD_SIZE);

                    }
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }

        /* If Exit all modes */
        if (
                (VDM_HDR.vdm_type == VDM_TYPE_STRUCTURED)   &&
                (VDM_HDR.obj_pos == EXIT_ALL_MODES)         &&
                (VDM_HDR.cmd == VDM_CMD_EXIT_MODE)          &&
                (am_info_p->VDM_HDR.svid == VDM_HDR.svid)   &&
                (am_info_p->is_active)
            )
        {
            /* Save cmd */
            am_info_p->VDM_HDR.cmd = EXIT_ALL_MODES;
            /* Run ufp alt mode cbk */
            am_info_p->cbk(port);
            return true;
        }
    }

    return false;
}

uint8_t alt_mode_get_status(uint8_t port)
{
    uint8_t ret = NO_DATA;
#if DFP_ALT_MODE_SUPP
    alt_mode_info_t *am_info_p = NULL;
    uint8_t          alt_mode_idx;

    if (alt_mode[port].state == ALT_MODE_MNGR_STATE_PROCESS)
    {
        /* Check each alternate mode */
        for (alt_mode_idx = 0; alt_mode_idx < alt_mode[port].alt_modes_numb; alt_mode_idx++)
        {
            if (IS_FLAG_CHECKED(alt_mode[port].am_active_modes, alt_mode_idx))
            {
                am_info_p = get_mode_info (port, alt_mode_idx);
                if (am_info_p != NULL)
                {
                    if (am_info_p->is_active != false)
                    {
                        /* Set alt modes which were entered */
                        SET_FLAG(ret, alt_mode_idx);
                    }
                }
            }
        }
        /* Mode discovery complete. */
        ret |= 0x80;
    }

#endif /* DFP_ALT_MODE_SUPP */

    return ret;
}

/* [] END OF FILE */

