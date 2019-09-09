/**
 * @file vdm_task_mngr.c
 *
 * @brief @{VDM task manager source file.@}
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

#include <vdm.h>
#include <pd.h>
#include <dpm.h>
#include <timer.h>
#include <app.h>
#include <alt_mode_hw.h>
#include <hpi.h>
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
#include <vdm_task_mngr.h>
#include <alt_modes_mngr.h>

/**
 * @typedef vdm_task_status_t
 * @brief struct to hold vdm manager status
 */
typedef struct
{
    /* Current VDM manager state */
    vdm_task_t      vdm_task;
    /* Current VDM manager event */
    vdm_evt_t       vdm_evt;
    /* Info about cable, device/AMA */
    atch_tgt_info_t atch_tgt;
    /* Struct with received/sent VDM info */
    vdm_msg_info_t  vdm_msg;
    /* Sent/received VDM retry counters */
    uint8_t         rec_retry_cnt;
    /* Holds svid index if we have more than one Disc SVID response */
    uint8_t         svid_idx;
    /* Holds count of D_SVID requests sent. */
    uint8_t         dsvid_cnt;
}vdm_task_status_t;

#if VDM_RESP_QUERY_SUPPORTED

#define MAX_DISC_ID_RESP_LEN    (MAX_NO_OF_DO)
#define MAX_DISC_SVID_RESP_LEN  (18u)

/* Array to store the D_ID response received from the port partner. */
static uint32_t vdm_disc_id_resp[NO_OF_TYPEC_PORTS][MAX_DISC_ID_RESP_LEN];
static uint8_t  vdm_disc_id_resp_len[NO_OF_TYPEC_PORTS];

/* Array to store the D_SVID response received from the port partner. */
static uint32_t vdm_disc_svid_resp[NO_OF_TYPEC_PORTS][MAX_DISC_SVID_RESP_LEN];
static uint8_t  vdm_disc_svid_resp_len[NO_OF_TYPEC_PORTS];

#endif /* VDM_RESP_QUERY_SUPPORTED */

/* Main structure to hold variables for VDM task manager */
vdm_task_status_t vdm_stat[NO_OF_TYPEC_PORTS];

/* Init VDM task mngr */
static vdm_task_t vdm_task_mngr_init(uint8_t port);

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
/* Discover ID cmd handler */
static vdm_task_t vdm_task_mng_disc_id(uint8_t port, vdm_evt_t vdm_evt);
/* Discovery SVID cmd handler */
static vdm_task_t vdm_task_mng_disc_svid(uint8_t port, vdm_evt_t vdm_evt);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

/* Sets recevied VDM response command as failed */
static void set_vdm_failed(uint8_t port, fail_status_t fail_stat);
/* Check VDM retries */
static bool vdm_retry_check(uint8_t port, fail_status_t fail_stat);
/* Parses received VDM resp to vdm_msg_info_t struct */
static uint8_t parse_vdm(uint8_t port, const pd_packet_t* rec_vdm);
/* Callback function for sent VDM */
static void vdm_rec_cbk(uint8_t port, resp_status_t status, const pd_packet_t *rec_vdm);
/* Function to send VDM */
static vdm_task_t send_vdm(uint8_t port);
/* Checks if VDM mngr is enabled */
static bool is_vdm_mngr_enabled(uint8_t port);
/* Returns pointer to vdm_msg_info_t struct */
static vdm_msg_info_t* get_msg(uint8_t port);
/* Composes vdm_msg_info_t data to dpm_pd_cmd_buf_t struct */
static uint8_t compose_vdm(uint8_t port);
/* Reset vdm mngr info */
static void reset_vdm_mngr(uint8_t port);

/* VDM field set/get functions. */
vdm_evt_t get_evt(uint8_t port);
vdm_task_t get_task(uint8_t port);
void set_task(uint8_t port, vdm_task_t task);
void set_evt(uint8_t port, vdm_evt_t evt);

/**
   @typedef cable_dp_reset_state_t
   @brief Enumeration tracking current cable (SOP'') soft reset state.
 */
typedef enum {
    CABLE_DP_RESET_IDLE = 0,    /**< No Cable (SOP'') reset state. */
    CABLE_DP_RESET_WAIT,        /**< Waiting for response from cable (SOP'') soft reset. */
    CABLE_DP_RESET_RETRY,       /**< Cable soft reset (SOP'') to be attempted. */
    CABLE_DP_RESET_DONE         /**< Cable soft reset (SOP'') has been completed. */
} cable_dp_reset_state_t;

/* Maximum number of EMCA SOFT_RESET attempts to be made. */
#define MAX_EMCA_DP_RESET_COUNT         (3u)

static cable_dp_reset_state_t vdm_emca_rst_state[NO_OF_TYPEC_PORTS] = {
    CABLE_DP_RESET_IDLE
#if (NO_OF_TYPEC_PORTS > 1)
    ,
    CABLE_DP_RESET_IDLE
#endif /* (NO_OF_TYPEC_PORTS > 1) */
};

static uint8_t vdm_emca_rst_count[NO_OF_TYPEC_PORTS] = {
    0
#if (NO_OF_TYPEC_PORTS > 1)
    ,
    0
#endif /* (NO_OF_TYPEC_PORTS > 1) */
};

void enable_vdm_task_mngr(uint8_t port)
{
    /* If the VDM Task is already running, do nothing. */
    if (app_get_status(port)->vdm_task_en == false)
    {
        set_task(port, VDM_TASK_INIT);
        app_get_status(port)->vdm_task_en = true;
        app_get_status(port)->vdm_prcs_failed = false;
    }
}

static vdm_task_t vdm_task_mngr_init(uint8_t port)
{
    vdm_task_t ret = VDM_TASK_EXIT;

    /* Reset vdm mngr info */
    reset_vdm_mngr(port);
    set_evt(port, VDM_EVT_RUN);

#if CCG4_DOCK
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {
        /* in CCG4 Dock US is UFP only, so it should not init D_ID */
        if(port == 0u) return VDM_TASK_EXIT;
    }
#endif /* CCG4_DOCK */

    if (get_alt_mode_numb(port) != false)
    {
        /* Check if current data role is DFP */
        if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
        {
            ret = VDM_TASK_DISC_ID;
    }
        else
        {
            ret = VDM_TASK_REG_ATCH_TGT_INFO;
        }
    }

    return ret;

}

bool is_vdm_task_idle(uint8_t port)
{
    /*
       If VDM manager is enabled, check
       1. Whether BUSY timer is running.
       2. Whether the ALT mode tasks are idle.
     */
    return !(is_vdm_mngr_enabled (port) &&
            ((timer_is_running (port, APP_VDM_BUSY_TIMER)) ||
             (get_task(port) != VDM_TASK_ALT_MODE) ||
             (is_alt_mode_mngr_idle(port) == false))
            );
}

void vdm_task_mngr(uint8_t port)
{
    /* The VDM task enabled check is performed by the caller. */
    if (
            (timer_is_running(port, APP_VDM_BUSY_TIMER) == true)
#if MUX_DELAY_EN			
			|| (app_get_status(port)->is_mux_busy != false)
#endif /* MUX_DELAY_EN */			
       )
    {
        return;
    }

    /* Get current vdm task */
    switch (get_task(port))
    {
        case VDM_TASK_WAIT:
            break;

        case VDM_TASK_INIT:
            set_task(port, vdm_task_mngr_init(port));
            break;

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
        case VDM_TASK_DISC_ID:
            set_task(port, vdm_task_mng_disc_id(port, get_evt(port)));
            break;

        case VDM_TASK_DISC_SVID:
            set_task(port, vdm_task_mng_disc_svid(port, get_evt(port)));
            break;
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

        case VDM_TASK_SEND_MSG:
            compose_vdm(port);
            set_task(port, send_vdm(port));
            break;

        case VDM_TASK_REG_ATCH_TGT_INFO:
            set_task(port, reg_alt_mode_mngr(port, &vdm_stat[port].atch_tgt, get_msg(port)));
            set_evt(port, VDM_EVT_RUN);
            break;

        case VDM_TASK_ALT_MODE:
            set_task(port, vdm_task_mngr_alt_mode_process(port, get_evt(port)));
            if ((get_task(port) == VDM_TASK_SEND_MSG) || (get_task(port) == VDM_TASK_ALT_MODE))
            {
                set_evt(port, VDM_EVT_RUN);
            }
            break;

        case VDM_TASK_EXIT:
            vdm_task_mngr_deinit(port);
            app_get_status(port)->vdm_prcs_failed = true;
            break;

        default:
            return;

    }
}

static bool vdm_retry_check(uint8_t port, fail_status_t fail_stat)
{
    (vdm_stat[port].rec_retry_cnt)++;

    /* If we don't receive response more than rec_retry_cnt notify that fail */
    if (vdm_stat[port].rec_retry_cnt > MAX_RETRY_CNT)
    {
        set_vdm_failed(port, fail_stat);
        return false;
    }
    /* Try to send msg again */
    set_task(port, VDM_TASK_SEND_MSG);
    /* Start a timer to delay the retry attempt. */
    timer_start(port, APP_VDM_BUSY_TIMER, APP_VDM_FAIL_RETRY_PERIOD, NULL);
    return true;
}

static void set_vdm_failed(uint8_t port, fail_status_t fail_stat)
{
    /* Reset retry counters */
    vdm_stat[port].rec_retry_cnt = 0;
    set_evt(port, VDM_EVT_FAIL);
    /* Set failed and save failed command */
    get_msg(port)->VDM_HDR.cmd = get_vdm_buff(port)->cmd_do->std_vdm_hdr.cmd;
    /* Save Failure code in the object position field */
    get_msg(port)->VDM_HDR.obj_pos = (uint32_t)fail_stat;
}

/* Received VDM callback */
static void vdm_rec_cbk(uint8_t port, resp_status_t status, const pd_packet_t* rec_vdm)
{
    uint32_t response;
    bool     run_task_flag = false;

    /* If response ACK */
    if (status == RES_RCVD)
    {
        /* Handle Standard VDM */
        if (rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.vdm_type == VDM_TYPE_STRUCTURED)
        {
            response = rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd_type;

#if CCG_PD_REV3_ENABLE
            if (rec_vdm->sop == SOP)
            {
                app_get_status(port)->vdm_version =
                    GET_MIN (app_get_status(port)->vdm_version, rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.st_ver);
            }
#endif /* CCG_PD_REV3_ENABLE */

            /* Actual response received. */
            switch (response)
            {
                case CMD_TYPE_RESP_ACK:
                    {
                        /* Check if received response is expected. */
                        if ((rec_vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd) ==
                                (get_vdm_buff(port)->cmd_do[VDM_HEADER_IDX].std_vdm_hdr.cmd))
                        {
                            parse_vdm(port, rec_vdm);
                            set_evt(port, VDM_EVT_EVAL);

                            /* Reset timer counter when ACK. */
                            vdm_stat[port].rec_retry_cnt = 0;
                        }
                        else
                        {
                            /* Set failed and save failed command */
                            set_vdm_failed(port, TIMEOUT);
                        }

                        /* Continue the state machine operation. */
                        run_task_flag = true;
                    }
                    break;

                case CMD_TYPE_RESP_BUSY:
                    /* Target is BUSY. */
                    {
                        /* Check if command should be retried. We always have to continue task execution. */
                        vdm_stat[port].rec_retry_cnt++;
                        if (vdm_stat[port].rec_retry_cnt > MAX_RETRY_CNT)
                        {
                            set_vdm_failed(port, BUSY);
                            run_task_flag = true;
                        }
                        else
                        {
                            /* Try to send msg again */
                            set_task(port, VDM_TASK_SEND_MSG);
                            /* Start a timer. Command will be retried when timer expires. */
                            timer_start(port, APP_VDM_BUSY_TIMER, APP_VDM_BUSY_TIMER_PERIOD, NULL);
                        }
                    }
                    break;
                default:
                    /* Target NAK-ed the message. */
                    {
                        /* Notify manager with failed event */
                        set_vdm_failed(port, NACK);
                        run_task_flag = true;
                    }
                    break;
            }
        }
        /* Handle UVDM */
        else
        {
            parse_vdm(port, rec_vdm);
            set_evt(port, VDM_EVT_EVAL);
            /* Reset timer counter when ACK */
            vdm_stat[port].rec_retry_cnt = 0;
            run_task_flag = true;
        }
    }
    /* Attention related handler */
    else if (get_msg(port)->VDM_HDR.cmd == VDM_CMD_ATTENTION)
    {
        /* This statement need to notify alt mode that Attention VDM was successfuly sent */
        if ((status == CMD_SENT) && (get_task(port) == VDM_TASK_WAIT))
        {
            /* Start a timer. Command will be retried when timer expires. */
            timer_stop(port, APP_VDM_BUSY_TIMER);
            set_evt(port, VDM_EVT_EVAL);
            /* Reset retry counter */
            vdm_stat[port].rec_retry_cnt = 0;
            /* Continue the state machine operation. */
            run_task_flag = true;
        }
        else if (status == SEQ_ABORTED)
        {
            /* Try to send msg again */
            run_task_flag = true;
        }
    }
    else
    {
        /* Good CRC not received or no response (maybe corrupted packet). */
        if ((status == CMD_FAILED) || (status == RES_TIMEOUT))
        {
            /* Check for retries. If failure persists after all retries, go to exit. */
            if (vdm_retry_check (port, (fail_status_t)status) == false)
            {
                run_task_flag = true;
            }
        }
        else
        {
            if (status == SEQ_ABORTED)
            {
                /* Try to send msg again */
                set_task(port, VDM_TASK_SEND_MSG);
            }
        }
    }

    /* Check if we need run any task */
    if (run_task_flag == true)
    {
#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))
        switch (vdm_stat[port].vdm_msg.VDM_HDR.cmd)
        {
            case VDM_CMD_DSC_IDENTITY:
                set_task(port, VDM_TASK_DISC_ID);
                break;
            case VDM_CMD_DSC_SVIDS:
                set_task(port, VDM_TASK_DISC_SVID);
                break;
            default:
                set_task(port, VDM_TASK_ALT_MODE);
                break;
        }
#else /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
        set_task(port, VDM_TASK_ALT_MODE);
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */
    }
}

#if ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE))

static vdm_task_t vdm_task_mng_disc_id(uint8_t port, vdm_evt_t vdm_evt)
{
    vdm_msg_info_t *msg_p  = get_msg (port);
    vdm_task_t ret         = VDM_TASK_EXIT;
    const dpm_status_t *dpm_stat = dpm_get_info(port);

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_get_info(port)->spec_rev_sop_live >= PD_REV3) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP));
#endif

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
            /* Form Discover ID VDM packet. */
            msg_p->vdm_header.val   = NO_DATA;
            msg_p->VDM_HDR.svid     = STD_SVID;
            msg_p->VDM_HDR.cmd      = VDM_CMD_DSC_IDENTITY;
            msg_p->vdo_numb         = NO_DATA;
            msg_p->sop_type         = SOP;
            msg_p->VDM_HDR.vdm_type = VDM_TYPE_STRUCTURED;
            ret                     = VDM_TASK_SEND_MSG;
            break;

            /* Evaluate received response */
        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
#if VDM_RESP_QUERY_SUPPORTED
                /* Store the D_ID response received. */
                vdm_disc_id_resp[port][0] = msg_p->vdm_header.val;
                memcpy ((uint8_t *)&vdm_disc_id_resp[port][1], (uint8_t *)msg_p->vdo, msg_p->vdo_numb * sizeof (uint32_t));
                vdm_disc_id_resp_len[port] = msg_p->vdo_numb + 1;
#endif /* VDM_RESP_QUERY_SUPPORTED */

                /* Set svid idx to zero before start DISC SVID */
                vdm_stat[port].svid_idx  = NO_DATA;
                vdm_stat[port].dsvid_cnt = 0;

                /* Copy ID header to info struct */
                vdm_stat[port].atch_tgt.ama_vdo.val = msg_p->vdo[PD_DISC_ID_AMA_VDO_IDX].val;
                /* Copy AMA VDO to info struct */
                vdm_stat[port].atch_tgt.tgt_id_header.val = msg_p->vdo[VDO_START_IDX - 1].val;

                /* If AMA and cable (if present) do not need Vconn, disable VConn. */
                if (
                        (dpm_stat->vconn_retain == 0) &&
                        ((msg_p->vdo[VDO_START_IDX - 1].std_id_hdr.prod_type != PROD_TYPE_AMA) ||
                            (msg_p->vdo[PD_DISC_ID_AMA_VDO_IDX].std_ama_vdo.vcon_req == 0)) &&
                        ((dpm_stat->emca_present == false) ||
                         (dpm_stat->cbl_vdo.std_cbl_vdo.cbl_term == CBL_TERM_BOTH_PAS_VCONN_NOT_REQ))
                   )
                {
                    vconn_disable(port, dpm_stat->rev_pol);
                }

                /* If alt modes not supported. */
                if (msg_p->vdo[VDO_START_IDX - 1].std_id_hdr.mod_support == false)
                {
                    break;
                }

                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
        case VDM_EVT_FAIL:
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_DISC_SVID;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

        default:
            break;
    }

    return ret;
}

/* Checks if input SVID already saved. */
static bool is_svid_stored(uint16_t *svid_arr, uint16_t svid)
{
    uint8_t  idx;

    /* Go through all SVIDs and compare with input SVID */
    for (idx = 0; idx < MAX_SVID_VDO_SUPP; idx++)
    {
        /* If input SVID already saved */
        if (svid_arr[idx] == svid)
        {
            return true;
        }
    }

    return false;
}

/*
   This function saves received Discover SVID resp,
   returns true if a NULL SVID was received.
 */
static bool save_svids(uint8_t port, uint16_t *svid_arr, uint8_t max_svid)
{
    uint8_t         idx, vdo_count;
    uint16_t        svid;
    uint8_t         svid_idx = vdm_stat[port].svid_idx;
    bool            retval   = false;
    vdm_msg_info_t  *msg_p   = get_msg (port);

    /* Compare received SVIDs with supported SVIDs */
    vdo_count = msg_p->vdo_numb;

    /* Stop further discovery if this response does not have the maximum no. of DOs. */
    if (vdo_count < (MAX_NO_OF_DO - 1))
    {
        retval = true;
    }

    for (idx = 0; idx < (vdo_count * 2); idx++)
    {
        if ((idx & 1) == 0)
        {
            /* Upper half of the DO. */
            svid = msg_p->vdo[idx >> 1].val >> 16u;
        }
        else
        {
            /* Lower half of the DO. */
            svid = msg_p->vdo[idx >> 1].val & 0xFFFFu;
        }

        if (svid_idx < (max_svid - 1))
        {
            /* Stop on NULL SVID. */
            if (svid == NO_DATA)
            {
                retval = true;
            }
            else
            {
                /* If SVID not saved previously then save */
                if (is_svid_stored(svid_arr, svid) == false)
                {
#if SAVE_SUPP_SVID_ONLY
                    if (is_svid_supported(svid, port) != false)
#endif /* SAVE_SUPP_SVID_ONLY */
                    {
                        svid_arr[svid_idx] = svid;
                        svid_idx++;
                    }
                }
            }
        }
        else
        {
            /* Cannot continue as we have no more space. */
            retval = true;
            break;
        }
    }

    /* Set zero after last SVID in info */
    svid_arr[svid_idx] = NO_DATA;
    vdm_stat[port].svid_idx = svid_idx;

    /* Terminate discovery after MAX_DISC_SVID_COUNT attempts. */
    vdm_stat[port].dsvid_cnt++;
    if (vdm_stat[port].dsvid_cnt >= MAX_DISC_SVID_COUNT)
        retval = true;

    return retval;
}

static void set_disc_svid_param (uint8_t port, uint8_t sop)
{
    vdm_msg_info_t *msg_p = get_msg (port);

    msg_p->vdm_header.val   = NO_DATA;
    msg_p->VDM_HDR.svid     = STD_SVID;
    msg_p->VDM_HDR.cmd      = VDM_CMD_DSC_SVIDS;
    msg_p->VDM_HDR.obj_pos  = NO_DATA;
    msg_p->sop_type         = sop;
    msg_p->vdo_numb         = NO_DATA;
    msg_p->VDM_HDR.vdm_type = VDM_TYPE_STRUCTURED;
}

static vdm_task_t vdm_task_mng_disc_svid(uint8_t port, vdm_evt_t vdm_evt)
{
    vdm_task_t ret               = VDM_TASK_EXIT;
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    vdm_msg_info_t *msg_p        = get_msg (port);
#if VDM_RESP_QUERY_SUPPORTED
    uint32_t tmp;
#endif /* VDM_RESP_QUERY_SUPPORTED */

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
    bool pd3_ufp = ((bool)(dpm_stat->spec_rev_sop_live >= PD_REV3) &&
            (gl_dpm_port_type[port] == PRT_TYPE_UFP));
#endif

    switch (vdm_evt)
    {
        case VDM_EVT_RUN:
            /* Form Discover SVID VDM packet */
            set_disc_svid_param (port, SOP);
            ret = VDM_TASK_SEND_MSG;
            break;

        case VDM_EVT_EVAL:
            /* Check is current port date role DFP */
            if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
            {
                /* For attached target response */
                if (msg_p->sop_type == (uint8_t)SOP)
                {
#if VDM_RESP_QUERY_SUPPORTED
                    if (vdm_stat[port].svid_idx == 0)
                    {
                        /* Save the DISC_SVID response. */
                        vdm_disc_svid_resp[port][0] = msg_p->vdm_header.val;
                        memcpy ((uint8_t *)&vdm_disc_svid_resp[port][1], (uint8_t *)msg_p->vdo, msg_p->vdo_numb * sizeof (uint32_t));
                        vdm_disc_svid_resp_len[port] = msg_p->vdo_numb + 1;
                    }
                    else
                    {
                        /* Save the incremental DISC_SVID response. */
                        tmp = GET_MIN (msg_p->vdo_numb, MAX_DISC_SVID_RESP_LEN - vdm_disc_svid_resp_len[port]);
                        memcpy ((uint8_t *)&vdm_disc_svid_resp[port][vdm_disc_svid_resp_len[port]], (uint8_t *)msg_p->vdo,
                                tmp * sizeof (uint32_t));
                        vdm_disc_svid_resp_len[port] += tmp;
                    }
#endif /* VDM_RESP_QUERY_SUPPORTED */

                    if (msg_p->vdo[0].val == NONE_VDO)
                    {
                        /* No SVID supported */
                        break;
                    }

                    /* Save received SVIDs and check if a NULL SVID was received. */
                    if (save_svids (port, vdm_stat[port].atch_tgt.tgt_svid, MAX_SVID_VDO_SUPP) != false)
                    {
                        /* If cable was detected and supports alternate modes, send SOP' disc svid */
                        if ((dpm_stat->emca_present != false) &&
                                (dpm_stat->cbl_mode_en != false))
                        {
                            if (
                                    (dpm_stat->vconn_logical)
                            #if VCONN_OCP_ENABLE
                                    && ((app_get_status(port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) == 0)
                            #endif /* VCONN_OCP_ENABLE */
                               )
                            {
                                if (!vconn_is_present (port))
                                {
                                    /*
                                     * We are VConn source and VConn is off. Enable and apply a delay to let
                                     * the EMCA power up.
                                     */
                                    vconn_enable (port, dpm_stat->rev_pol);

                                    /* Start a timer to delay the retry attempt. */
                                    timer_start(port, APP_VDM_BUSY_TIMER, APP_CABLE_POWER_UP_DELAY, NULL);
                                }

                                vdm_stat[port].svid_idx  = 0;
                                vdm_stat[port].dsvid_cnt = 0;
                                set_disc_svid_param (port, SOP_PRIME);
                                ret = VDM_TASK_SEND_MSG;
                            }
                            else
                            {
                                /*
                                 * We are not VConn source. This should not happen as stack will ensure
                                 * we are VConn source. Skip SOP' checks in the unlikely case where this
                                 * happens.
                                 */
                                set_evt(port, VDM_EVT_RUN);
                                ret = VDM_TASK_REG_ATCH_TGT_INFO;
                            }
                        }
                        else
                        {
                            /* Goto register info in alt modes mngr */
                            set_evt(port, VDM_EVT_RUN);
                            ret = VDM_TASK_REG_ATCH_TGT_INFO;
                        }
                    }
                    else
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        set_disc_svid_param (port, SOP);
                        ret = VDM_TASK_SEND_MSG;
                    }
                }
                /* For cable response */
                else
                {
                    /* If the EMCA returned any DOs, save the content. */
                    if ((msg_p->vdo[VDO_START_IDX - 1].val != NONE_VDO) &&
                            (save_svids(port, vdm_stat[port].atch_tgt.cbl_svid, MAX_CABLE_SVID_SUPP) == false))
                    {
                        /* If not all SVID received, ask for the next set of SVIDs. */
                        set_disc_svid_param (port, SOP_PRIME);
                        ret = VDM_TASK_SEND_MSG;
                    }
                    else
                    {
                        /*
                           If EMCA did not support any SVIDs of interest and does not require VConn for operation,
                           we can disable VConn.
                         */
                        if (vdm_stat[port].atch_tgt.cbl_svid[0] == NO_DATA)
                        {
                            if (
                                    (dpm_stat->vconn_retain == 0) &&
                                    (dpm_stat->cbl_vdo.std_cbl_vdo.cbl_term == CBL_TERM_BOTH_PAS_VCONN_NOT_REQ)
                               )
                            {
                                vconn_disable(port, dpm_stat->rev_pol);
                            }
                        }

                        /* Move to the next step. */
                        set_evt(port, VDM_EVT_RUN);
                        ret = VDM_TASK_REG_ATCH_TGT_INFO;
                    }
                }
            }

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
            /* Check is current port date role UFP */
            if (pd3_ufp)
            {
                /* Send Disc SVID cmd */
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */
            break;

        case VDM_EVT_FAIL:
            /* If cable SVID fails */
            if (msg_p->sop_type == (uint8_t)SOP_PRIME)
            {
                set_evt(port, VDM_EVT_RUN);
                ret = VDM_TASK_REG_ATCH_TGT_INFO;
            }
            break;

        default:
            break;
    }

    return ret;
}

#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_MODE_DISC_ENABLE)) */

static void reset_vdm_mngr(uint8_t port)
{
    vdm_stat[port].rec_retry_cnt = NO_DATA;
    vdm_stat[port].svid_idx      = NO_DATA;
    vdm_stat[port].dsvid_cnt     = NO_DATA;
    /* Clear arrays which hold SVIDs */
    memset(&vdm_stat[port].atch_tgt, NO_DATA, sizeof(atch_tgt_info_t));
    /* Store the pointer to the cable VDO discovered by PD stack. */
    vdm_stat[port].atch_tgt.cbl_vdo = &(dpm_get_info(port)->cbl_vdo);
    /* Clear the SOP'' reset state. */
    vdm_emca_rst_state[port] = CABLE_DP_RESET_IDLE;
    vdm_emca_rst_count[port] = 0;

    /* Stop the VDM task delay timer. */
    timer_stop(port, APP_VDM_BUSY_TIMER);

#if VDM_RESP_QUERY_SUPPORTED
    vdm_disc_id_resp_len[port] = 0;
    memset ((uint8_t *)&vdm_disc_id_resp[port], 0, MAX_NO_OF_DO * sizeof (uint32_t));
    vdm_disc_svid_resp_len[port] = 0;
    memset ((uint8_t *)&vdm_disc_svid_resp[port], 0, MAX_DISC_SVID_RESP_LEN * sizeof (uint32_t));
#endif /* VDM_RESP_QUERY_SUPPORTED */
}

void vdm_task_mngr_deinit(uint8_t port)
{
    /* CDT 247011 re-fix */
    app_get_status(port)->vdm_prcs_failed = false;
    /* If VDM task is not active, no need to go through the rest of the steps. */
    if (app_get_status(port)->vdm_task_en != false)
    {
        reset_vdm_mngr(port);
        app_get_status(port)->vdm_task_en = false;
        app_get_status(port)->alt_mode_entered = false;
        /* Exit from alt mode manager */
        vdm_task_mngr_alt_mode_process(port, VDM_EVT_EXIT);
        /* Deinit App HW */
        alt_mode_hw_deinit(port);
    }
}

static void sop_dp_soft_reset_cb(uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    switch (resp)
    {
        case CMD_SENT:
            /* Do nothing. */
            break;

        case RES_RCVD:
            /* Proceed with rest of alternate mode state machine. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_DONE;
            break;

        default:
            /* Retry the cable SOFT_RESET. */
            vdm_emca_rst_count[port]++;
            if (vdm_emca_rst_count[port] >= MAX_EMCA_DP_RESET_COUNT)
            {
                vdm_emca_rst_state[port] = CABLE_DP_RESET_DONE;
            }
            else
            {
                vdm_emca_rst_state[port] = CABLE_DP_RESET_RETRY;
            }
            break;
    }
}

static void send_sop_dp_soft_reset(uint8_t port)
{
    dpm_pd_cmd_buf_t dpm_cmd_param;
    ccg_status_t     api_stat;

    dpm_cmd_param.cmd_sop      = SOP_DPRIME;
    dpm_cmd_param.no_of_cmd_do = 0;
    dpm_cmd_param.dat_ptr      = 0;
    dpm_cmd_param.timeout      = 15;

    api_stat = dpm_pd_command (port, DPM_CMD_SEND_SOFT_RESET_EMCA, &dpm_cmd_param, sop_dp_soft_reset_cb);
    switch (api_stat)
    {
        case CCG_STAT_SUCCESS:
            /* Wait for the cable SOFT_RESET response. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_WAIT;
            break;

        case CCG_STAT_BUSY:
        case CCG_STAT_NOT_READY:
            /* Need to retry the command. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_RETRY;
            break;

        default:
            /* Connection state changed. Abort the Cable SOFT_RESET process. */
            vdm_emca_rst_state[port] = CABLE_DP_RESET_DONE;
            break;
    }
}

/* Fill DPM cmd buffer with properly VDM info */
static uint8_t compose_vdm(uint8_t port)
{
    uint8_t           idx;
    dpm_pd_cmd_buf_t  *vdm_buf = get_vdm_buff(port);
    vdm_msg_info_t    *msg_p   = get_msg(port);

    vdm_buf->cmd_sop                = msg_p->sop_type;
    vdm_buf->no_of_cmd_do           = msg_p->vdo_numb + VDO_START_IDX;
    vdm_buf->cmd_do[VDM_HEADER_IDX] = msg_p->vdm_header;

    if (msg_p->VDM_HDR.vdm_type == VDM_TYPE_STRUCTURED)
    {
        msg_p->VDM_HDR.cmd_type    = CMD_TYPE_INITIATOR;

        if (vdm_buf->cmd_sop == SOP)
        {
            vdm_buf->cmd_do[VDM_HEADER_IDX].std_vdm_hdr.st_ver = app_get_status(port)->vdm_version;
        }
        else
        {
            vdm_buf->cmd_do[VDM_HEADER_IDX].std_vdm_hdr.st_ver = dpm_get_info(port)->cbl_vdm_version;
        }

        /* Set exceptions for Enter/Exit mode cmd period */
        switch (msg_p->VDM_HDR.cmd)
        {
            case VDM_CMD_ENTER_MODE:
                vdm_buf->timeout = PD_VDM_ENTER_MODE_RESPONSE_TIMER_PERIOD;
                break;
            case VDM_CMD_EXIT_MODE:
                vdm_buf->timeout = PD_VDM_EXIT_MODE_RESPONSE_TIMER_PERIOD;
                break;
            case VDM_CMD_ATTENTION:
                /* No timeout required for attention messages. */
                vdm_buf->timeout = NO_DATA;
                break;
            default:
                vdm_buf->timeout = PD_VDM_RESPONSE_TIMER_PERIOD;
                break;
        }
    }
    /* Handle UVDM */
    else
    {
        vdm_buf->timeout = PD_VDM_RESPONSE_TIMER_PERIOD;
    }

    /* Copy VDOs to send buffer */
    if (msg_p->vdo_numb > NO_DATA)
    {
        for (idx = 0; idx < msg_p->vdo_numb; idx++)
        {
            vdm_buf->cmd_do[idx + VDO_START_IDX].val = msg_p->vdo[idx].val;
        }
    }

    return true;
}

/* Parse received VDM and save info in  */
static uint8_t parse_vdm(uint8_t port, const pd_packet_t* rec_vdm)
{
    uint8_t vdo_idx;
    vdm_msg_info_t *msg_p  = get_msg (port);

    msg_p->vdm_header = rec_vdm->dat[VDM_HEADER_IDX];
    msg_p->vdo_numb   = (rec_vdm->len - VDO_START_IDX);
    msg_p->sop_type   = rec_vdm->sop;

    /* If VDO is present in received VDM */
    if (rec_vdm->len > VDO_START_IDX)
    {
        for (vdo_idx = 0; vdo_idx < rec_vdm->len; vdo_idx++)
        {
            msg_p->vdo[vdo_idx].val = rec_vdm->dat[vdo_idx + VDO_START_IDX].val;
        }
    }

    return true;
}

static vdm_task_t send_vdm(uint8_t port)
{
    if ((gl_dpm_port_type[port] == PRT_TYPE_DFP) && (get_vdm_buff(port)->cmd_sop == SOP_DPRIME))
    {
        switch (vdm_emca_rst_state[port])
        {
            /* If SOP'' SOFT RESET has not been done or a retry is pending, try to send it. */
            case CABLE_DP_RESET_IDLE:
            case CABLE_DP_RESET_RETRY:
                send_sop_dp_soft_reset(port);
                /* Intentional fall-through. */

            case CABLE_DP_RESET_WAIT:
                return VDM_TASK_SEND_MSG;

            default:
                /* EMCA SOFT_RESET done. We can proceed. */
                break;
        }
    }

    app_get_status(port)->vdm_retry_pending = (vdm_stat[port].rec_retry_cnt != MAX_RETRY_CNT);
    if (dpm_pd_command (port, DPM_CMD_SEND_VDM, get_vdm_buff(port), vdm_rec_cbk) == CCG_STAT_SUCCESS)
    {
        return VDM_TASK_WAIT;
    }

    /* If fails - try to send VDM again */
    return VDM_TASK_SEND_MSG;
}

#if ((CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE))
bool is_ufp_disc_started(uint8_t port)
{
    if (is_vdm_task_idle(port) == true)
    {
        /* Set VDM Task to send DISC ID VDM */
        set_task(port, VDM_TASK_DISC_ID);
        set_evt(port, VDM_EVT_RUN);
        return true;
    }

    return false;
}
#endif /* (CCG_PD_REV3_ENABLE) && (UFP_ALT_MODE_SUPP) && (UFP_MODE_DISC_ENABLE) */

bool is_vdm_mngr_enabled(uint8_t port)
{
    return (bool)app_get_status(port)->vdm_task_en;
}

vdm_evt_t get_evt(uint8_t port)
{
    return vdm_stat[port].vdm_evt;
}

vdm_task_t get_task(uint8_t port)
{
    return vdm_stat[port].vdm_task;
}

void set_task(uint8_t port, vdm_task_t task)
{
    vdm_stat[port].vdm_task = task;
}

void set_evt(uint8_t port, vdm_evt_t evt)
{
    vdm_stat[port].vdm_evt = evt;
}

static vdm_msg_info_t* get_msg(uint8_t port)
{
    return &vdm_stat[port].vdm_msg;
}
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

uint8_t *vdm_get_disc_id_resp(uint8_t port, uint8_t *resp_len_p)
{
    uint8_t *ptr = NULL;

    /* Check for bad pointer argument. */
    if (resp_len_p == NULL)
        return NULL;

    /* Set response length to zero by default. */
    *resp_len_p = 0;

#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
    if ((port < NO_OF_TYPEC_PORTS) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
    {
        *resp_len_p = vdm_disc_id_resp_len[port];
        ptr         = (uint8_t *)vdm_disc_id_resp[port];
    }
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */

    return (ptr);
}

uint8_t *vdm_get_disc_svid_resp(uint8_t port, uint8_t *resp_len_p)
{
    uint8_t *ptr = NULL;

    /* Check for bad pointer argument. */
    if (resp_len_p == NULL)
        return NULL;

    /* Set response length to zero by default. */
    *resp_len_p = 0;

#if ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED))
    if ((port < NO_OF_TYPEC_PORTS) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
    {
        *resp_len_p = vdm_disc_svid_resp_len[port];
        ptr         = (uint8_t *)vdm_disc_svid_resp[port];
    }
#endif /* ((DFP_ALT_MODE_SUPP) && (VDM_RESP_QUERY_SUPPORTED)) */

    return (ptr);
}
/* [] END OF FILE */
