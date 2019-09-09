/**
 * @file uvdm.h
 *
 * @brief @{Unstructured VDM handler header file.@}
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

#ifndef _UVDM_H
#define _UVDM_H

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/*
 * @brief MAX Number of VDOs in U_VDM response.
 */
#define UVDM_RESPONSE_MAX_NO_OF_VDO                             (0x07)

/*
 * @brief VDM Header index in U_VDM Response..
 */
#define UVDM_HEADER_INDEX                                       (0x00)

/*
 * @brief U_VDM commands signature byte offset.
 */
#define UVDM_SIGNATURE_BYTE_OFFSET                              (0x00)

/*
 * @brief Response U_VDM DEVICE_MODE VDO Index.
 */
#define UVDM_DEVICE_MODE_VDO_INDEX                              (0x01)

/*
 * @brief BOOT LAST ROW VDO Index.
 */
#define UVDM_BOOT_LAST_ROW_VDO_INDEX                            (0x02)

/*
 * @brief Response U_VDM BOOT VERSION VDO INDEX.
 */
#define UVDM_BOOT_VERSION_VDO_INDEX                             (0x01)

/*
 * @brief Response U_VDM FW Image 1 VERSION VDO INDEX.
 */
#define UVDM_IMG1_VERSION_VDO_INDEX                             (0x03)

/*
 * @brief Response U_VDM FW Image 2 VERSION VDO INDEX.
 */
#define UVDM_IMG2_VERSION_VDO_INDEX                             (0x05)

/*
 * @brief Version number size in bytes.
 */
#define UVDM_VERSION_NUM_SIZE_BYTES                             (0x08)

/*
 * @brief Number of VDOs in GET_VERSION U_VDM Response.
 */
#define UVDM_GET_VERSION_U_VDM_NO_OF_VDO                        (0x06)

/*
 * @brief GET_SILICON_ID Command size in bytes.
 */
#define UVDM_GET_SILICON_ID_CMD_SIZE                            (0x04)

/*
 * @brief GET_SILICON_ID Command Signature : "S"..
 */
#define UVDM_GET_SILICON_ID_CMD_SIG                             (0x53)

/*
 * @brief Response UVDM_FW1_START_ADDR VDO_INDEX.
 */
#define UVDM_FW1_START_ADDR_VDO_INDEX                           (0x01)

/*
 * @brief Response UVDM_FW2_START_ADDR VDO_INDEX.
 */
#define UVDM_FW2_START_ADDR_VDO_INDEX                           (0x02)

/*
 * @brief Number of VDOs in GET_FW_START_ADDRESS_UVDM Response.
 */
#define UVDM_GET_FW_START_ADDR_UVDM_NO_OF_VDO                   (0x02)

/*
 * @brief SILICON_ID VDO INDEX in U_VDM Response.
 */
#define UVDM_SILICON_ID_VDO_INDEX                               (0x02)

/*
 * @brief DEVICE_RESET Command size in bytes.
 */
#define UVDM_DEVICE_RESET_CMD_SIZE                              (0x04)

/*
 * @brief Device RESET Command Signature : "R"..
 */
#define UVDM_DEVICE_RESET_CMD_SIG                               (0x52)

/*
 * @brief JUMP_TO_BOOT Command size in bytes.
 */
#define UVDM_JUMP_TO_BOOT_CMD_SIZE                              (0x04)

/*
 * @brief JUMP_TO_BOOT Command Signature : "J".
 */
#define UVDM_JUMP_TO_BOOT_CMD_SIG                                ('J')

/**
 * @brief Signature value used to request a JUMP_TO_ALT_FW operation.
 */
#define UVDM_JUMP_TO_ALT_FW_SIG                                  ('A')

/*
 * @brief ENTER_FLASHING_MODE Command size in bytes.
 */
#define UVDM_ENTER_FLASHING_MODE_CMD_SIZE                       (0x04)

/*
 * @brief ENTER FLASHING MODE Command Signature : "J".
 */
#define UVDM_ENTER_FLASHING_MODE_CMD_SIG                        (0x50)

/*
 * @brief Flash WRITE and Read command size.
 */
#define UVDM_FLASH_READ_WRITE_CMD_SIZE                          (0x04)

/*
 * @brief FLASH_READ_WRITE Command Signature : "F".
 */
#define UVDM_FLASH_READ_WRITE_CMD_SIG                           (0x46)

/*
 * @brief Flash Write and Read command row num LSB offset.
 */
#define UVDM_FLASH_ROW_NUM_LSB_OFFSET                           (0x01)

/*
 * @brief Flash Write and Read command row num MSB offset.
 */
#define UVDM_FLASH_ROW_NUM_MSB_OFFSET                           (0x02)

/*
 * @brief READ_DATA Response VDO index.
 */
#define UVDM_READ_DATA_RESPONSE_VDO_INDEX                       (0x01)

/*
 * @brief U_VDM Response VDO index.
 */
#define UVDM_RESPONSE_VDO_INDEX                                 (0x01)

/*
 * @brief READ_DATA Response no of VDOs in error case.
 */
#define UVDM_READ_DATA_NO_OF_VDO_ERROR_CASE                     (0x02)

/*
 * @brief VALIDATE_FW command size.
 */
#define UVDM_VALIDATE_FW_CMD_SIZE                               (0x04)

/*
 * @brief VALIDATE_FW Register FW MODE offset.
 */
#define UVDM_VALIDATE_FW_MODE_INDEX                             (0x00)

/*
 * @brief REASON_FOR_BOOT_MODE VDO index.
 */
#define UVDM_REASON_FOR_BOOT_MODE_VDO_INDEX                     (0x01)

/*
 * @brief GET_CHECKSUM CMD size.
 */
#define UVDM_GET_CHECKSUM_CMD_SIZE                              (0x08)

/*
 * @brief GET_CHECKSUM CMD FLASH ADDR LSB offset.
 */
#define UVDM_FLASH_ADDR_LSB_OFFSET                              (0x00)

/*
 * @brief GET_CHECKSUM CMD FLASH SIZE LSB offset.
 */
#define UVDM_FLASH_SIZE_LSB_OFFSET                              (0x04)

/*
 * @brief GET_CHECKSUM response checksum VDO index.
 */
#define UVDM_CHEKCSUM_VDO_INDEX                                 (0x01)

/*
 * @brief SET_APP_PRIORITY command size.
 */
#define UVDM_SET_APP_PRIORITY_CMD_SIZE                          (0x04)

/*
 * @brief SET_APP_PRIORITY FW Image priority offset.
 */
#define UVDM_SET_APP_PRIORITY_INDEX                             (0x00)

/*
 * @brief Send Siganture UVDM Valid Sequence Number 1.
 */
#define UVDM_SEND_SIGN_SEQUENCE_1                               (0x01)

/*
 * @brief Send Siganture UVDM Valid Sequence Number 2.
 */
#define UVDM_SEND_SIGN_SEQUENCE_2                               (0x02)

/*
 * @brief Send Siganture UVDM Valid Sequence Number 3.
 */
#define UVDM_SEND_SIGN_SEQUENCE_3                               (0x03)

/*
 * @brief Send Signature UVDM Sections Size in Bytes.
 */
#define UVDM_SEND_SIGN_SEC_1_2_SIZE                             (0x18)

/*
 *  @brief Send Signature UVDM Sections Size in Bytes.
 */
#define UVDM_SEND_SIGN_SEC_3_SIZE                               (0x10)

/*
 * @brief GET_CUSTOMER_INFO VDM Sequence Number 1.
 */
#define UVDM_GET_CUSTOMER_INFO_SEQ_1                             (0x01)

/*
 * @brief GET_CUSTOMER_INFO VDM Sequence Number 2.
 */
#define UVDM_GET_CUSTOMER_INFO_SEQ_2                             (0x02)

/*
 * @brief GET_CUSTOMER_INFO Response Size in bytes.
 */
#define UVDM_GET_CUSTOMER_INFO_RESPONSE_SIZE                     (0x10)

/*
 * @brief GET_CUSTOMER_INFO number of VDOs in response.
 */
#define UVDM_GET_CUSTOMER_INFO_RESPONSE_VDO_NUM                  (0x04)


/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/
/*
 * @typedef uvdm_cmd_opcode_t
 *
 * @brief List of opcodes of CY Flashing Alternate Mode U_VDMs.
 */
typedef enum
{
    UVDM_CMD_RESERVED,                      /**< Reserved. */
    UVDM_CMD_GET_DEVICE_MODE_OPCODE,        /**< Get active mode of device. */
    UVDM_CMD_GET_DEVICE_VERSION_OPCODE,     /**< Get version information of all images. */
    UVDM_CMD_GET_SILICON_ID_OPCODE,         /**< Get silicon ID of CCG. */
    UVDM_CMD_DEVICE_RESET_OPCODE,           /**< Reset CCG. */
    UVDM_CMD_JUMP_TO_BOOT_OPCDOE,           /**< Jump to boot mode. */
    UVDM_CMD_ENTER_FLASHING_MODE_OPCODE,    /**< Enable flash access mode. */
    UVDM_CMD_SEND_DATA_OPCODE,              /**< Collect flash row data for write. */
    UVDM_CMD_FLASH_WRITE_OPCODE,            /**< Program flash row. */
    UVDM_CMD_READ_DATA_OPCODE,              /**< Collect flash row data for read. */
    UVDM_CMD_FLASH_READ_OPCODE,             /**< Read flash row. */
    UVDM_CMD_VALIDATE_FW_OPCODE,            /**< Validate FW image. */
    UVDM_CMD_REASON_FOR_BOOT_MODE,          /**< Get reason for boot mode. */
    UVDM_CMD_GET_CHECKSUM,                  /**< Get checksum of specified flash section. */
    UVDM_CMD_GET_FW_START_ADDRESS_OPCODE,   /**< Get start address of FW images. */
    UVDM_CMD_SET_APP_PRIORITY_OPCODE,       /**< Set APP priority value. */
    UVDM_CMD_RESERVED_16,                   /**< Reserved. */
    UVDM_CMD_SEND_SIGNATURE_OPCODE,         /**< Program FW image ECDSA signature. */
    UVDM_CMD_RESERVED_18,                   /**< Reserved. */
    UVDM_CMD_GET_BOOT_TYPE,                 /**< Get information related to boot loader type. */
    UVDM_CMD_GET_CUSTOMER_INFO              /**< Get custom data. */
} uvdm_cmd_opcode_t;

/*
 * @typedef uvdm_response_state_t
 *
 * @brief List of possible stateso of UVDM Response.
 */
typedef enum
{
    UVDM_NOT_HANDLED,                       /**< UVDM not recognised, can't be handled. */
    UVDM_HANDLED_RESPONSE_READY,            /**< UVDM handled and response is ready to be sent. */
    UVDM_HANDLED_NO_RESPONSE,               /**< UVDM handled but no response required. */
    UVDM_HANDLED_RESPONSE_NOT_READY         /**< UVDM Handled but response will be sent later, potential
                                                 non-blocking command. */
} uvdm_response_state_t;

/*
 * @typedef uvdm_qc_4_0_cmd_t
 *
 * @brief List of opcodes of QC 4.0 UVDM Commands.
 */
typedef enum
{
    UVDM_QC_GET_CASE_TEMP = 0x1003,
    UVDM_QC_GET_CONNECTOR_TEMP = 0x0B03,
    UVDM_QC_GET_CONNECTOR_VOLT = 0x0603,
    UVDM_QC_GET_CHARGER_TYPE = 0x0C03,
    UVDM_QC_GET_CHARGER_VERSION = 0x0E03
} uvdm_qc_4_0_cmd_t;

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/*
 * @brief Returns current non blocking command opcode.
 *
 * Description
 * This function returns the opcode of current Non Blocking UVDM Command
 * which can be used to form the response VDM.
 *
 * @param None
 * @return uvdm_cmd_opcode_t UVDM Command Opcode
 */
uvdm_cmd_opcode_t uvdm_get_cur_nb_cmd(void);

/*
 * @brief Resets internal state and trackers related to Non Blokcing flash write
 * operation at the end of write operation.
 *
 * @param None
 * @return None
 */
void uvdm_reset_nb_cmd_state(void);

/*
 * @brief CY Flashing Mode Entry Handler
 *
 * Description
 * This function is called from PD Stack when CCG enters CY Flashing Altenrate
 * mode. CY UVDM commands are handled only when CCG has entered CY Flashing
 * Alternate mode.
 * @return None
 */
void uvdm_enter_cy_alt_mode(void);

/*
 * @brief CY Flashing Mode Exit Handler
 *
 * Description
 * This function is called from PD Stack when CCG exits CY Flashing Altenrate
 * mode.
 * @return None
 */
void uvdm_exit_cy_alt_mode(void);

/*
 * @brief Return CY Flashing Mode Status
 *
 * Description
 * This function returns the current state of CY Flashing Alternate Mode.
 * @return TRUE if CY Flashing Mode active,FALSE Otherwise
 */
bool uvdm_get_flashing_mode(void);

/*
 * @brief Handle Device Reset Command
 *
 * Description
 * This function handles device Reset command. Device Reset handling is
 * application specific and hence this routine shall be implemented at Solution
 * level.
 *
 * @param reset_sig Type of Reset: Device Reset, Jump to boot or Jump to Alt image
 *
 * @param ccg_status_t
 */
ccg_status_t uvdm_handle_device_reset(uint32_t reset_sig);

/*
 * @brief CY Flashing Alternate Mode UVDM Command Handler
 *
 * Description
 * Handles all CY Flashing Alternate Mode UVDM Commands.
 *
 * @param rx_pkt Pointer to VDM Command
 * @param vdm_rspn_pkt Pointer to the VDM response packet
 * @param vdo_count Number of VDOs in VDM response, 0 if no VDM Response
 * @param flash_cb Callback function for Non Blocking Flash Write
 *
 * @return uvdm_response_state_t
 */
uvdm_response_state_t uvdm_handle_cmd(uint32_t *rx_pkt, pd_do_t **vdm_rspn_pkt,
    uint8_t *vdo_count, flash_cbk_t flash_cb);

/*
 * @brief QC 4.0 UVDM Handler
 *
 * Description
 * Handles all QC 4.0 Protocol UVDM Commands.
 *
 * @port Index of TYPE-C port
 * @param rx_pkt Pointer to VDM Command
 * @param vdm_rspn_pkt Pointer to the VDM response packet
 * @param vdo_count Number of VDOs in VDM response, 0 if no VDM Response
 *
 * @return uvdm_response_state_t
 */
uvdm_response_state_t uvdm_qc_4_0_handler(uint8_t port, uint32_t *rx_pkt,
    pd_do_t **vdm_rspn_pkt, uint8_t *vdo_count);

#endif /* _UVDM_H */

/* [] END OF FILE */
