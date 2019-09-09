/**
 * @file dpm_intern.h
 *
 * @brief @{Stack internal definitions for the Device Policy Manager (DPM).@}
 */

/*
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

#ifndef _DPM_INTERN_H_
#define _DPM_INTERN_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <pd.h>
#include <status.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/
/**
 * @brief Size (in bytes) of the config table data that needs to be loaded in
 * RAM at runtime.
 */
#define PORT_INFO_COPY_SIZE   (102u)

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function returns the DPM status of the specified port.
 * @param port Port index.
 * @return Returns pointer to the DPM status structure.
 */
dpm_status_t* dpm_get_status(uint8_t port);

/**
 * @brief This function refreshes the source cap at runtime. Source cap may be
 * changed due to Embedded Controller (EC) updates.
 * @param port Port index.
 * @return Returns true if successful, false otherwise.
 */
bool dpm_refresh_src_cap(uint8_t port);

/**
 * @brief This function refreshes the sink cap at runtime. Sink cap may be
 * changed due to Embedded Controller (EC) updates.
 * @param port Port index.
 * @return Returns true if successful, false otherwise.
 */
bool dpm_refresh_snk_cap(uint8_t port);

/**
 * @brief This function checks if the previous contract is valid. Previous
 * contract may become invalid due to runtime update of source caps by the
 * Embedded Controller (EC).
 * @param port Port index.
 * @return CCG_STAT_SUCCESS if rdo is valid, CCG_STAT_BAD_PARAM
 * if port index is not correct, CCG_STAT_FAILURE if rdo is not valid.
 */
ccg_status_t dpm_is_prev_contract_valid(uint8_t port);

/**
 * @brief This function sets a particular pe event.
 * @param port Port index.
 * @param evt_mask Event Mask
 * @return None
 */
void dpm_set_pe_evt(uint8_t port, uint32_t evt_mask);

/**
 * @brief This function clears a particular pe event.
 * @param port Port index.
 * @param evt_mask Event Mask
 * @return None
 */
void dpm_clear_pe_evt(uint8_t port, uint32_t evt_mask);

/**
 * @brief This function sets a particular typec event.
 * @param port Port index.
 * @param evt_mask Event Mask
 * @return None
 */
void dpm_set_typec_evt(uint8_t port, uint32_t evt_mask);

/**
 * @brief This function clears a particular typec event.
 * @param port Port index.
 * @param evt_mask Event Mask
 * @return None
 */
void dpm_clear_typec_evt(uint8_t port, uint32_t evt_mask);


/**
 * @brief A wrapper for app events
 * @param port PD Port index.
 * @param evt App event
 * @return 
 */
void dpm_notify_app(uint8_t port, app_evt_t evt);

/**
 * @brief A wrapper for app event with data.
 * @param port PD port index
 * @param evt App event
 * @param dat Data pointer
 * @return 
 */
void dpm_notify_app_wdata(uint8_t port, app_evt_t evt, const void* dat);

#if CCG_PD_REV3_ENABLE

/**
 * @brief This function returns maximum PD spec rev supported by device.
 * @param port Port index.
 * @return PD spec rev
 */
pd_rev_t dpm_get_default_pd_rev(uint8_t port);

#if CCG_FRS_RX_ENABLE
    
/**
 * @brief This function returns if FRS swap receive is supported by this port
 * @param port Port index.
 * @return Returns true if supported, false otherwise.
 */
bool dpm_is_frs_rx_supported(uint8_t port);   

#endif /* CCG_FRS_RX_ENABLE */ 

#if CCG_FRS_TX_ENABLE
    
/**
 * @brief This function returns if FRS swap transmit is supported by this port
 * @param port Port index.
 * @return Returns true if supported, false otherwise.
 */
bool dpm_is_frs_tx_supported(uint8_t port);   

#endif /* CCG_FRS_TX_ENABLE */ 


#endif /* CCG_PD_REV3_ENABLE */

#endif /* _DPM_INTERN_H_ */

/* End of file */

