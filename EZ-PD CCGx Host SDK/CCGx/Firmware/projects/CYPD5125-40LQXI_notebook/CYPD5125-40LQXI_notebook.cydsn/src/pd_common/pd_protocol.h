/**
 * @file pd_protocol.h
 *
 * @brief @{USB-PD protocol layer header file.@}
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

#ifndef _PD_PROTOCOL_H_
#define _PD_PROTOCOL_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <pd.h>
#include <status.h>
/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function sets the clock and necessary registers for PD hw ip to
 * function and initializes the PD protocol layer.
 * @param port port index
 * @param cbk pd event handler callback
 * @return ccg_status_t
 */
ccg_status_t pd_prot_init(uint8_t port, pd_cbk_t cbk);

/**
 * @brief This function starts the protocol layer and configures pd phy as per
 * current port role /data role/contract status of port.
 * This API does not enable the receiver
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_start(uint8_t port);

/**
 * @brief This function configures pd phy as per current port role /data role/contract status of port.
 * This API does not enable the receiver
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_refresh_roles(uint8_t port);

/**
 * @brief This function completely stops the pd hw and put it in lowest power state.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_stop(uint8_t port);

/**
 * @brief This function enables the bmc receiver.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_rx_en(uint8_t port);

/**
 * @brief This function disables the bmc receiver.
 * @param port port index
 * @param hard_reset_en When 0 means rx is completely disabled, When 1 means only
 * hard reset can be received.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_rx_dis(uint8_t port, uint8_t hard_reset_en);

/**
 * @brief This function checks if protocol layer is busy
 * @param port port index
 * @return Return true when busy else return false
 */
bool pd_prot_is_busy(uint8_t port);

/**
 * @brief This function resets the protocol layer(TX and RX) counters for each sop type.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_reset_all(uint8_t port);

/**
 * @brief This function resets protocol layer(TX and RX) counter for a specific sop type.
 * @param port port index
 * @param sop sop type
 * @return ccg_status_t
 */
ccg_status_t pd_prot_reset(uint8_t port, sop_t sop);

/**
 * @brief This function resets protocol layer RX only counter for a specific sop type.
 * @param port port index
 * @param sop sop type
 * @return ccg_status_t
 */
ccg_status_t pd_prot_reset_rx(uint8_t port, sop_t sop);

/**
 * @brief This function sends a control message. Results will be known to caller via callback
 * function registered in pd_prot_int() if this function returns success.
 * This function returns after registering the request.
 * @param port port index
 * @param sop sop type
 * @param msg_type control message type.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_ctrl_msg(
        uint8_t port,
        sop_t sop,
        ctrl_msg_t msg_type);

/**
 * @brief This function sends a data message. Results will be known to caller via callback
 * function registered in pd_phy_init() this this function returns success.
 * This function returns after registering the request.
 * @param port port index
 * @param sop sop type
 * @param msg_type data message type
 * @param count data objects count
 * @param dobj pointer to data objects
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_data_msg(
        uint8_t port,
        sop_t sop,
        data_msg_t msg_type,
        uint8_t count,
        pd_do_t *dobj);

#if CCG_PD_REV3_ENABLE

/**
 * @brief This function sends an extended message. Results will be known to caller via callback
 * function registered in pd_phy_init() if this function returns sucsess.
 * This function returns after registering the request.
 * @param port port index
 * @param sop sop type
 * @param msg_type data message type
 * @param ext_hdr 16 bit extended header
 * @param dobj pointer to data
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_extd__msg(uint8_t port,
                       sop_t sop,
                       extd_msg_t msg_type,
                       pd_extd_hdr_t ext_hdr,
                       uint8_t* dobj);

#endif /* CCG_PD_REV3_ENABLE */

/**
 * @brief This function sends a hard reset. Results will be known to caller via callback
 * function registered in pd_phy_init() if this function returns success.
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_hard_reset(uint8_t port);

/**
 * @brief This function sends a cable reset. Results will be known to caller via callback
 * function registered in pd_phy_init() if this fucntion returns sucsess.
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_send_cable_reset(uint8_t port);

/**
 * @brief This function enables sending bist carrier mode 2. There is no call back for this function
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_en_bist_cm2(uint8_t port);

/**
 * @brief This function disable sending bist carrier mode 2. There is no call back for this function
 * This function returns after registering the request.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_dis_bist_cm2(uint8_t port);

/**
 * @brief This function puts the receiver in bist test data mode.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_en_bist_test_data(uint8_t port);

/**
 * @brief This function disables the bist test data mode
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_dis_bist_test_data(uint8_t port);

/**
 * @brief This function can be used by higher layers to avoid retry on CRC expire
 * for a particular message. When this flag is set before calling
 * pd_prot_send_data_msg() or pd_send_crtl_msg() APIs, retry is avoided on CRC failure.
 * This is one time only. Flag is automatically cleared after
 * transmission(success or fail).
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t pd_prot_set_avoid_retry(uint8_t port);

/**
 * @brief This function returns pointer to received pd packet
 * @param port port index
 * @return Returns pointer to received data if port param is not correct return null
 */
pd_packet_extd_t* pd_prot_get_rx_packet(uint8_t port);

#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
    
/**
 * @brief This function enables the fast role swap receive functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_rx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap receive functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_rx_disable(uint8_t port);

#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
    
/**
 * @brief This function enables the fast role swap transmit functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_tx_enable(uint8_t port);

/**
 * @brief This function disables the fast role swap transmit functionality.
 *
 * @param port: Port index.
 * @return ccg_status_t
 */
ccg_status_t pd_prot_frs_tx_disable(uint8_t port);

#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */

#endif /* _PD_PROTOCOL_H_ */

/* End of File */

