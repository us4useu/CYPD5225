/**
 * @file vdm.c
 *
 * @brief @{Vendor Defined Message (VDM) handler source file.@}
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
#include <timer.h>
#include <vdm.h>
#include <pd.h>
#include <dpm.h>
#include <app.h>
#include <vdm_task_mngr.h>
#include <alt_modes_mngr.h>
#include <intel_vid.h>
#include <hpi.h>
#if DP_GPIO_CONFIG_SELECT
#include <dp_sid.h>
#endif /* DP_GPIO_CONFIG_SELECT */

#if (FLASHING_MODE_PD_ENABLE == 1)
#include <flash.h>
#include <uvdm.h>
#endif /* FLASHING_MODE_PD_ENABLE */

/* Stores Discover ID response VDO count */
static uint8_t  gl_vdm_id_vdo_cnt[NO_OF_TYPEC_PORTS];

/* Stores Discover SVID response VDO count */
static uint8_t  gl_vdm_svid_vdo_cnt[NO_OF_TYPEC_PORTS];

/* Stores Discover Modes response VDO count */
static uint16_t gl_vdm_mode_data_len[NO_OF_TYPEC_PORTS];

/* Stores pointer to Discover ID response data */
static pd_do_t *gl_vdm_id_vdo_p[NO_OF_TYPEC_PORTS];

/* Stores pointer to Discover SVID response data */
static pd_do_t *gl_vdm_svid_vdo_p[NO_OF_TYPEC_PORTS];

/* Stores pointer to Discover Modes response data */
static uint8_t *gl_vdm_mode_data_p[NO_OF_TYPEC_PORTS];

#if (FLASH_ENABLE_NB_MODE == 1) && (FLASHING_MODE_PD_ENABLE == 1)
vdm_resp_cbk_t gl_vdm_resp_cbk = NULL;
/*
 * In Non-Blocking Flash Write Scenario, this is the callback
 * invoked by Flash Module after Flash row write completes.
 * This should be used to send back the VDM response.
 *
 * @param NONE
 * @param NONE
 */
void uvdm_nb_flash_write_cb(flash_write_status_t write_status)
{
    /* Non-Blocking Flashing is supported only for CCG3. Hence the port index is
     * fixed to 0.*/
    app_status_t* app_stat = app_get_status(0);

    /* Response should be sent out only if Flash write was not aborted. */
    if (write_status == FLASH_WRITE_COMPLETE)
    {
        app_stat->vdm_resp.resp_buf[UVDM_HEADER_INDEX].val =
            ((get_pd_config()->flashing_vid << 16) |
            (CMD_TYPE_RESP_ACK << 11) | (uvdm_get_cur_nb_cmd ()));
        app_stat->vdm_resp.resp_buf[UVDM_RESPONSE_VDO_INDEX].val =
            (CCG_STAT_SUCCESS + CCG_STATUS_CODE_OFFSET);

        app_stat->vdm_resp.do_count = 2;
        /* Ignore Unstructured VDM */
        app_stat->vdm_resp.no_resp = VDM_AMS_RESP_READY;
        gl_vdm_resp_cbk (0, &app_stat->vdm_resp);
    }

    uvdm_reset_nb_cmd_state ();
}
#else
void *uvdm_nb_flash_write_cb = NULL;
#endif /* (FLASH_ENABLE_NB_MODE == 1) && (FLASHING_MODE_PD_ENABLE == 1) */

/* Store VDM information from the config table in the RAM variables. */
void vdm_data_init(uint8_t port)
{
    uint16_t size = 0;

    /* Calculate the number of VDOs in the D_ID response. */
    size = get_pd_port_config(port)->id_vdm_length;
    /* Subtract 4 bytes for the header and reduce to number of VDOs. */
    if (size > 4)
        gl_vdm_id_vdo_cnt[port] = (uint8_t)((size - 4) >> 2);
    else
        gl_vdm_id_vdo_cnt[port] = 0;

    /* Update the D_ID response pointer. */
    gl_vdm_id_vdo_p[port] = (pd_do_t *)(((uint8_t *)get_pd_config ()) +
            get_pd_port_config(port)->id_vdm_offset + 4);

    /* Calculate the number of VDOs in the D_SVID response. */
    size = get_pd_port_config(port)->svid_vdm_length;
    /* Subtract 4 bytes for the header and reduce to number of VDOs. */
    if (size > 4)
        gl_vdm_svid_vdo_cnt[port] = (uint8_t)((size - 4) >> 2);
    else
        gl_vdm_svid_vdo_cnt[port] = 0;

    /* Update the D_SVID response pointer. */
    gl_vdm_svid_vdo_p[port] = (pd_do_t *)(((uint8_t *)get_pd_config ()) +
            get_pd_port_config(port)->svid_vdm_offset + 4);

    /* Store the D_MODE response length from configuration table. */
    gl_vdm_mode_data_len[port] = get_pd_port_config(port)->mode_vdm_length;

    /* Store pointer to the D_MODE responses. */
    gl_vdm_mode_data_p[port] = (uint8_t *)(get_pd_config ()) +
        get_pd_port_config(port)->mode_vdm_offset;
}

void vdm_update_svid_resp(uint8_t port, uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p)
{
    if (port < NO_OF_TYPEC_PORTS)
    {
        gl_vdm_svid_vdo_cnt[port]  = svid_vdo_cnt;
        gl_vdm_svid_vdo_p[port]    = (pd_do_t *)svid_vdo_p;
    }
}

void vdm_update_data(uint8_t port, uint8_t id_vdo_cnt, uint8_t *id_vdo_p,
        uint8_t svid_vdo_cnt, uint8_t *svid_vdo_p, uint16_t mode_resp_len, uint8_t *mode_resp_p)
{
    if (port < NO_OF_TYPEC_PORTS)
    {
        gl_vdm_id_vdo_cnt[port]    = id_vdo_cnt;
        gl_vdm_id_vdo_p[port]      = (pd_do_t *)id_vdo_p;
        gl_vdm_svid_vdo_cnt[port]  = svid_vdo_cnt;
        gl_vdm_svid_vdo_p[port]    = (pd_do_t *)svid_vdo_p;
        gl_vdm_mode_data_len[port] = mode_resp_len;
        gl_vdm_mode_data_p[port]   = mode_resp_p;
    }
}

bool get_modes_vdo_info(uint8_t port, uint16_t svid, pd_do_t **temp_p, uint8_t *no_of_vdo)
{
    uint8_t d_mode_resp_size_total;
    uint8_t d_mode_resp_size;
    pd_do_t *header;
    uint8_t* resp_p = gl_vdm_mode_data_p[port];

    /* Parse all responses based on SVID */
    d_mode_resp_size_total = gl_vdm_mode_data_len[port];

    /* If size is less than or equal to 4, return NACK. */
    if (d_mode_resp_size_total <= 4)
    {
        return false;
    }

    while (d_mode_resp_size_total)
    {
        /* Get the size of packet. */
        d_mode_resp_size = *resp_p;

        /* Read the SVID from header. */
        header = (pd_do_t *)(resp_p + 4);
        if (header->std_vdm_hdr.svid == svid)
        {
            *no_of_vdo = ((d_mode_resp_size-4) >> 2);
            *temp_p = header;
            return true;
        }
        /* Move to next packet. */
        resp_p += d_mode_resp_size;
        d_mode_resp_size_total -= d_mode_resp_size;
    }
    return false;
}

void eval_vdm(uint8_t port, const pd_packet_t *vdm, vdm_resp_cbk_t vdm_resp_handler)
{
    app_status_t* app_stat = app_get_status(port);
    pd_do_t* dobj;
    uint8_t i, count;
    bool eval_rslt ;
#if CCG_PD_REV3_ENABLE
    bool pd3_live = false;
#endif

#if (FLASHING_MODE_PD_ENABLE == 1)
#if (FLASH_ENABLE_NB_MODE == 1)
    gl_vdm_resp_cbk = vdm_resp_handler;
#endif /* FLASH_ENABLE_NB_MODE */
    uvdm_response_state_t response_state = UVDM_NOT_HANDLED;
#endif /* FLASHING_MODE_PD_ENABLE */

#if CCG_PD_REV3_ENABLE
    if (dpm_get_info(port)->spec_rev_sop_live >= PD_REV3)
        pd3_live = true;
#endif

    /* By Default assume we will respond */
    app_stat->vdm_resp.no_resp = VDM_AMS_RESP_READY;

    if (
            (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.vdm_type == VDM_TYPE_STRUCTURED) &&
            (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd_type == CMD_TYPE_INITIATOR)
       )
    {
        /* Copy received VDM Header data to VDM response Header*/
        app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].val = vdm->dat[VDM_HEADER_IDX].val;

#if CCG_PD_REV3_ENABLE
        /* Use the minimum VDM version from among the partner's revision and the live revision. */
        app_stat->vdm_version = GET_MIN (app_stat->vdm_version, vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.st_ver);
#endif

        /* Set a NAK response by default. */
        app_stat->vdm_resp.do_count = 1 ;
        app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_NAK;
#if MUX_DELAY_EN
        /* Save pointer to VDM response handler and set appropriate flag */
        app_stat->is_vdm_pending = true;
        app_stat->vdm_resp_cbk = vdm_resp_handler;
#endif /* MUX_DELAY_EN */
#if (!CCG_PD_REV3_ENABLE)
        if(gl_dpm_port_type[port] == PRT_TYPE_UFP)
#else
        if ((gl_dpm_port_type[port] == PRT_TYPE_UFP) || (pd3_live))
#endif /* !CCG_PD_REV3_ENABLE */
        {
            /* VDM Commands (D_ID -- EXIT_MODE) should be NAKd if VDO Count in VDM
             * command is more than one. */
            if (vdm->len == 1)
            {
                switch(vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd)
                {
                    case VDM_CMD_DSC_IDENTITY:
                        count = gl_vdm_id_vdo_cnt[port];
                        if((vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid ==  STD_SVID) && (count != 0))
                        {
                            app_stat->vdm_resp.do_count = count;
                            dobj = gl_vdm_id_vdo_p[port];
                            for(i = 0 ; i < count; i++)
                            {
                                app_stat->vdm_resp.resp_buf[i] = dobj[i];
                            }

#if (CCG_PD_REV3_ENABLE)
                            /* Mask Product Type (DFP) field when VDM version is 1.0. */
                            if (app_stat->vdm_version == 0)
                            {
                                app_stat->vdm_resp.resp_buf[VDO_START_IDX].std_id_hdr.prod_type_dfp = 0;
                            }
#else /* !CCG_PD_REV3_ENABLE */
                            app_stat->vdm_resp.resp_buf[VDO_START_IDX].std_id_hdr.rsvd1 = 0;
#endif /* CCG_PD_REV3_ENABLE */

                            /* Set VDM Response ACKed */
                            app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
                        }
                        break;
                    case VDM_CMD_DSC_SVIDS:
                        count = gl_vdm_svid_vdo_cnt[port];
                        if((vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid == STD_SVID) && (count != 0))
                        {
                            app_stat->vdm_resp.do_count = count;
                            dobj = gl_vdm_svid_vdo_p[port];
                            for(i = 0 ; i < count; i++)
                            {
                                app_stat->vdm_resp.resp_buf[i] = dobj[i];
                            }
                            /* Set VDM Response ACKed */
                            app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
                        }
                        break;
                    case VDM_CMD_DSC_MODES:
                        eval_rslt = get_modes_vdo_info (port, vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid,
                            &dobj, &count);
                        if (eval_rslt == true)
                        {
                            app_stat->vdm_resp.do_count = count;
                            for (i = 0; i < count; i++)
                            {
                                app_stat->vdm_resp.resp_buf[i] = dobj[i];
                            }
#if DP_GPIO_CONFIG_SELECT
                            /*
                             * For GPIO based DP Pin configuration selection, update the
                             * response based on GPIO status.
                             */
                            if (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid == DP_SVID)
                            {
                                app_stat->vdm_resp.resp_buf[1].std_dp_vdo.dfp_d_pin = dp_sink_get_pin_config ();
                            }
#endif /* DP_GPIO_CONFIG_SELECT */
                            /* Set VDM Response ACKed */
                            app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
                        }
                        break;
                    case VDM_CMD_ENTER_MODE:
#if CCG_HPI_PD_ENABLE
                        if (hpi_is_vdm_ec_ctrl_enabled (port))
                        {
                            app_stat->vdm_resp.no_resp = VDM_AMS_RESP_FROM_EC;
                        }
#endif /* CCG_HPI_PD_ENABLE */
                        break;

                    case VDM_CMD_EXIT_MODE:
#if CCG_HPI_PD_ENABLE
                        if (hpi_is_vdm_ec_ctrl_enabled (port))
                        {
                            app_stat->vdm_resp.no_resp = VDM_AMS_RESP_FROM_EC;
                        }
#endif /* CCG_HPI_PD_ENABLE */
                        break;

                    case VDM_CMD_ATTENTION:
                        /* Ignore Attention VDM */
                        app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_REQ;
                        break;
                    default:
                        break;
                }
            }

            /* Under PD2, handle VDMs based on VDM manager functionality. */
#if CCG_PD_REV3_ENABLE
            if (!pd3_live)
#endif /* CCG_PD_REV3_ENABLE */
            {
#if UFP_ALT_MODE_SUPP
                if (
                        (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid != STD_SVID) &&
                        (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd != VDM_CMD_ATTENTION)
                   )
                {
                    /* If DFP cmd processed success */
                    if (eval_rec_vdm(port, vdm))
                    {
                        /* Set VDM Response ACKed */
                        app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
                    }
                }
#endif /* UFP_ALT_MODE_SUPP */
            }
#if CCG_PD_REV3_ENABLE
            else
            {
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
                {
                    if ((vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_ENTER_MODE) ||
                            (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_EXIT_MODE))
                    {
                        app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_REQ;
                    }
                    else
                    {
                        if (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_ATTENTION)
                        {
                            app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_REQ;
                        }
                        eval_rec_vdm(port, vdm);
                    }
                }
                else
                {
                    /* If received cmd processed success */
                    if (eval_rec_vdm(port, vdm) == true)
                    {
                        /* Set VDM Response ACKed */
                        app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_ACK;
                    }
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
            }
#endif /* CCG_PD_REV3_ENABLE */
        }
        else
        {
#if DFP_ALT_MODE_SUPP
            if ((vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.cmd == VDM_CMD_ATTENTION) && (app_stat->vdm_task_en == true))
            {
                /* evaluate attention VDM */
                eval_rec_vdm(port, vdm);
            }
#endif /* DFP_ALT_MODE_SUPP */

            /* No response to VDMs received on PD 2.0 connection while in DFP state. */
            app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_REQ;
        }

        /* Set the VDM version for the response. */
        app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.st_ver = app_stat->vdm_version;
    }
    else
    {
#if (FLASHING_MODE_PD_ENABLE == 1)
        /* Pass the VDM to U_VDM handler */
        response_state = uvdm_handle_cmd ((uint32_t *)(&vdm->hdr), &dobj, &count,
            uvdm_nb_flash_write_cb);
        /* Response is ready. Notify PD. */
        if (response_state == UVDM_HANDLED_RESPONSE_READY)
        {
            app_stat->vdm_resp.do_count = count;
            for (i = 0; i < count; i++)
            {
                app_stat->vdm_resp.resp_buf[i] = dobj[i];
            }
        }
        /* No response to UVDM. */
        else if (response_state == UVDM_HANDLED_NO_RESPONSE)
        {
           /* Ignore Unstructured VDM response. */
            app_stat->vdm_resp.no_resp = VDM_AMS_RESP_FROM_EC;
        }
        /* UVDM not recognized by default UVDM handler. */
        else if (response_state == UVDM_NOT_HANDLED)
        {
            /*
             * This is the default implementation to handle UVDM commands which are
             * not recognized by CY UVDM handler. User can update this with custom
             * handlers.
             */

            /* If SVID is Flashing VID, respond with NAK. */
            if (vdm->dat[VDM_HEADER_IDX].std_vdm_hdr.svid ==
                get_pd_config()->flashing_vid)
            {
                app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].val =
                    vdm->dat[VDM_HEADER_IDX].val;
                app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].ustd_vdm_hdr.cmd_type
                    = CMD_TYPE_RESP_NAK;
                app_stat->vdm_resp.do_count = 1;
            }
            else
            {
                /* Invoke non-CY UVDM handlers here. */
#if QC_4_0_ENABLE
                /* QC 4.0 UVDM handling. */
                response_state = uvdm_qc_4_0_handler(port, (uint32_t *)(&vdm->hdr),
                    &dobj, &count);
                if (response_state == UVDM_HANDLED_RESPONSE_READY)
                {
                    app_stat->vdm_resp.do_count = count;
                    for (i = 0; i < count; i++)
                    {
                        app_stat->vdm_resp.resp_buf[i] = dobj[i];
                    }
                }
                /* No response to UVDM. */
                else if (response_state == UVDM_HANDLED_NO_RESPONSE)
                {
                   /* Ignore Unstructured VDM response. */
                    app_stat->vdm_resp.no_resp = VDM_AMS_RESP_FROM_EC;
                }
#endif /* QC_4_0_ENABLE */

                if (response_state == UVDM_NOT_HANDLED)
                {

                    /*
                     * SVID is not Recognized. Ignore if PD contract is PD 2.0.
                     * Send Not_Supported if Contract is PD 3.0.
                     */

                    /* No VDM response required. */
                    app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_REQ;
#if CCG_PD_REV3_ENABLE
                    if (pd3_live)
                    {
                        /* PD 3.0 Contract: Respond with Not_Supported. */
                        app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_SUPP;
                    }
#endif /* CCG_PD_REV3_ENABLE */
                }
            }
        }
        /* UVDM handled but response will be sent later. */
        else
        {
            /* Ignore. */
            return;
        }
#else /* (FLASHING_MODE_PD_ENABLE != 1) */
  #if UVDM_SUPP
        if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
        {
            if (eval_rec_vdm(port, vdm) == false)
            {
                /* If UVDM not successed then ignore this UVDM */
                app_stat->vdm_resp.no_resp = VDM_AMS_RESP_FROM_EC;
            }
        }
  #else /* !UVDM_SUPP */

        /* Set response type as Wait for EC to allow the response to be sent at a later time. */
        app_stat->vdm_resp.no_resp = VDM_AMS_RESP_FROM_EC;

    #if (CCG_PD_REV3_ENABLE)
      #if (CCG_HPI_PD_ENABLE)
        if (!hpi_is_vdm_ec_ctrl_enabled (port))
      #endif /* (CCG_HPI_PD_ENABLE) */
        {
            if (pd3_live)
            {
                /* PD 3.0 Contract: Respond with Not_Supported. */
                app_stat->vdm_resp.no_resp = VDM_AMS_RESP_NOT_SUPP;
            }
        }
    #endif /* (CCG_PD_REV3_ENABLE) */

  #endif /* UVDM_SUPP */
#endif /* FLASHING_MODE_PD_ENABLE */
    }

#if MUX_DELAY_EN
    /* Send VDM response only when MUX is idle or alt mode response is NACK */
    if (
            (app_stat->vdm_resp.resp_buf[VDM_HEADER_IDX].std_vdm_hdr.cmd_type != CMD_TYPE_RESP_ACK) ||
            (app_stat->is_mux_busy == false)
       )
    {
        vdm_resp_handler(port, &app_stat->vdm_resp);
        app_stat->is_vdm_pending = false;
    }
#else
    vdm_resp_handler(port, &app_stat->vdm_resp);
#endif /* MUX_DELAY_EN */
}

/* End of File */
