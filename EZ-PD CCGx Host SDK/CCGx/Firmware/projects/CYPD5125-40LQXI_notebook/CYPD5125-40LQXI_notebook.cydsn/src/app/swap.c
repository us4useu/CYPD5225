/**
 * @file swap.c
 *
 * @brief @{Swap request (PR_SWAP, DR_SWAP, VCONN_SWAP) handlers.@}
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
#include <swap.h>
#include <app.h>

static app_req_status_t get_response(uint8_t port, uint8_t raw_resp)
{
#if CCG_PD_REV3_ENABLE
    const dpm_status_t* dpm = dpm_get_info(port);
#endif /* CCG_PD_REV3_ENABLE */

    app_req_status_t retVal;
    switch(raw_resp)
    {
    case APP_RESP_ACCEPT:
        retVal = REQ_ACCEPT;
        break;
    case APP_RESP_WAIT:
        retVal =  REQ_WAIT;
        break;
#if CCG_PD_REV3_ENABLE
    case APP_RESP_NOT_SUPPORTED:
        retVal = REQ_NOT_SUPPORTED;
        break;
#endif /* CCG_PD_REV3_ENABLE */
    default:
        retVal = REQ_REJECT;
        break;
    }

#if CCG_PD_REV3_ENABLE
    if(dpm->spec_rev_sop_live <= PD_REV2)
    {
        if(retVal == REQ_NOT_SUPPORTED)
        {
            retVal = REQ_REJECT;
        }
    }
#endif /* CCG_PD_REV3_ENABLE */
    return retVal;
}

void eval_dr_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    app_req_status_t result = REQ_REJECT;

    if (app_get_status(port)->alt_mode_entered == true)
    {
        result = REQ_SEND_HARD_RESET;
    }
    else
    {
        result = get_response(port, GET_DR_SWAP_RESP(dpm->swap_response));
    }

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));
}

void eval_pr_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    app_req_status_t result = REQ_REJECT;
#if (!(CCG_SOURCE_ONLY))
    uint8_t pdo_mask;

    if(dpm->cur_port_role == PRT_ROLE_SOURCE)
    {
        pdo_mask = dpm->src_pdo_mask;
    }
    else
    {
        pdo_mask = dpm->snk_pdo_mask;
    }
#endif /* (!(CCG_SOURCE_ONLY)) */

    /*
     * Default response shall be NOT_SUPPORTED instead of REJECT if current mode
     * is PD REV3 and port role is source/sink only.
     */
#if CCG_PD_REV3_ENABLE
    if ((dpm_get_info(port)->spec_rev_sop_live >= PD_REV3) &&
        (dpm->port_role != PRT_DUAL))
    {
        result = REQ_NOT_SUPPORTED;
    }
#endif /* CCG_PD_REV3_ENABLE */

#if (!(CCG_SOURCE_ONLY))
    if ((dpm->dead_bat == false) && (dpm->port_role == PRT_DUAL) &&
            (((pdo_mask & (0x1 << PD_EXTERNALLY_POWERED_BIT_POS)) == 0) || (dpm->cur_port_role == PRT_ROLE_SINK)))
    {
        result = get_response(port, GET_PR_SWAP_RESP(dpm->swap_response));
    }
#endif /* (!(CCG_SOURCE_ONLY)) */

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));
}

void eval_vconn_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    const dpm_status_t* dpm = dpm_get_info(port);
    app_req_status_t result = REQ_REJECT;

    result = get_response(port, GET_VCONN_SWAP_RESP(dpm->swap_response));
    if((result == REQ_REJECT) || (result == REQ_NOT_SUPPORTED))
    {
        /* Allow other port to become VCONN source if we are not able to. */
        if(dpm->vconn_logical == true)
        {
            result = REQ_ACCEPT;
        }
    }

#if VCONN_OCP_ENABLE
    /* Do not allow VCONN_SWAP to become VConn source if fault is active. */
    if (
            (dpm->vconn_logical == false) &&
            ((app_get_status(port)->fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
       )
    {
        result = REQ_REJECT;
    }
#endif /* VCONN_OCP_ENABLE */

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));
}

#if (!(CCG_SOURCE_ONLY))
#if CCG_PD_REV3_ENABLE

void eval_fr_swap (uint8_t port, app_resp_cbk_t app_resp_handler)
{
    /* Always accept, FRS support is enabled/disabled by separate bit in config table */
    app_req_status_t result = REQ_ACCEPT;

    app_get_resp_buf(port)->req_status = result;
    app_resp_handler(port, app_get_resp_buf(port));

}

#endif /* CCG_PD_REV3_ENABLE */
#endif /* (!(CCG_SOURCE_ONLY)) */

 /* End of File */

