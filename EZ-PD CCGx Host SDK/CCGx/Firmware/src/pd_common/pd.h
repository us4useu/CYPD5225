/**
 * @file pd_common\pd.h
 *
 * @brief @{Common definitions and structures used in the CCG USB-PD stack.
 *
 * Note: Changing values in this header file are not likely to have an impact
 * on the final application behavior; as the pre-defined values would have been
 * compiled into the stack libraries.
 * @}
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

#ifndef _PD_H_
#define _PD_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <utils.h>

/*******************************************************************************
 * MACRO Definitions
 ******************************************************************************/

/********************************* PD macros **********************************/

#define PD_EXTERNALLY_POWERED_BIT_POS       (7u)
/**< Externally powered bit position in Source PDO mask. */

#define BC_SINK_1_2_MODE_ENABLE_MASK        (0x01)
/**< BC 1.2 sink mode enable mask for config table parameter. */

#define BC_SINK_APPLE_MODE_ENABLE_MASK      (0x02)
/**< Apple sink mode enable mask for config table parameter. */

#define BC_SRC_1_2_MODE_ENABLE_MASK         (0x01)
/**< BC 1.2 Source mode enable mask for config table parameter. */

#define BC_SRC_APPLE_MODE_ENABLE_MASK       (0x02)
/**< Apple source mode enable mask for config table parameter. */

#define BC_SRC_QC_MODE_ENABLE_MASK          (0x04)
/**< QC source mode enable mask for config table parameter. */

#define BC_SRC_AFC_MODE_ENABLE_MASK         (0x08)
/**< AFC source mode enable mask for config table parameter. */

#define BC_SRC_QC_4_0_MODE_ENABLE_MASK      (0x10)
/**< QC 4.0 mode enable mask for config table parameter. */

#define BC_SRC_QC_VER_2_CLASS_A_VAL         (0u)
/**< QC source Version and class mask for config table parameter. */

#define BC_SRC_QC_VER_2_CLASS_B_VAL         (1u)
/**< QC source Version and class mask for config table parameter. */

#define BC_SRC_QC_VER_3_CLASS_A_VAL         (2u)
/**< QC source Version and class mask for config table parameter. */

#define BC_SRC_QC_VER_3_CLASS_B_VAL         (3u)
/**< QC source Version and class mask for config table parameter. */

#define PD_MAX_SRC_CAP_TRY                  (6u)
/**< MAX SRC CAP retry count used to determine if the device connected is PD Capable or not. */

#define GIVE_BACK_MASK                      (0x8000u)
/**< Mask for the give-back supported bit in the snk_pdo_max_min_current_pwr field of the configuration table. */

#define SNK_MIN_MAX_MASK                    (0x3FFu)
/**< Mask to extract the actual min/max current value from the snk_pdo_max_min_current_pwr
     field of the configuration table. */

#define GET_DR_SWAP_RESP(resp)              ((resp) & 0x3u)
/**< Macro to extract the default DR_SWAP command response from the swap_response field
     in the configuration table. */

#define GET_PR_SWAP_RESP(resp)              (((resp) & 0xCu) >> 2u)
/**< Macro to extract the default PR_SWAP command response from the swap_response field in the configuration table. */

#define GET_VCONN_SWAP_RESP(resp)           (((resp) & 0x30u) >> 4u)
/**< Macro to extract the default VCONN_SWAP command response from the swap_response field
     in the configuration table. */

#define MAX_SRC_CAP_COUNT                   (50u)
/**< Maximum retries of Source Capability messages. */

#define MAX_HARD_RESET_COUNT                (3u)
/**< Maximum hard reset retry count. */

#define MAX_CBL_DSC_ID_COUNT                (20u)
/**< Maximum number of cable discovery DISCOVER_IDENTITY messages that should be sent out. */

#define MAX_NO_OF_DO                        (7u)
/**< Maximum number of DOs in a packet. Limited by PD message definition. */

#define MAX_NO_OF_PDO                       (MAX_NO_OF_DO)
/**< Maximum number of PDOs in a packet. Limited by PD message definition. */

#define MAX_NO_OF_VDO                       (MAX_NO_OF_DO)
/**< Maximum number of VDOs in a packet. Limited by PD message definition. */

#define VDM_HEADER_IDX                      (0u)
/**< Index of VDM header data object in a received message. */

#define BDO_HDR_IDX                         (0u)
/**< Index of BIST header data object in a received message. */

#define ID_HEADER_IDX                       (1u)
/**< Index of ID_HEADER data object in a received VDM. */

#define RDO_IDX                             (0u)
/**< Index of Request data object in a received message. */

#define MAX_EXTD_PKT_SIZE                   (260u)
/**< Maximum extended message size in bytes. */

#define MAX_EXTD_PKT_WORDS                  (65u)
/**< Maximum extended message 32-bit words. Each word is 32 bit. */

#define MAX_EXTD_MSG_LEGACY_LEN             (26u)
/**< Maximum legacy Extended message size in bytes. */

#define MAX_MESSAGE_ID                      (7u)
/**< Maximum message id value in PD Header. */

#define MAX_SOP_TYPES                       (3)
/**< Max SOP types excluding hard reset, cable reset, SOP_PDEBUG and SOP_DPDEBUG. */

#define STD_SVID                            (0xFF00u)
/**< Standard SVID defined by USB-PD specification. */

#define DP_SVID                             (0xFF01u)
/**< Displayport SVID defined by VESA specification. */

#define TBT_SVID                            (0x8087u)
/**< Thunderbolt SVID defined by Intel specification. */

#define CY_VID                              (0x04B4u)
/**< Cypress VID defined by Cypress for field upgrades. */

#define STD_VDM_VERSION_IDX                 (13u)
/**< Position of VDM version field in structured VDM header. */

#define STD_VDM_VERSION_REV3                (1u)
/**< VDM version 2.0. Used under USB-PD Revision 3.0. */

#define STD_VDM_VERSION_REV2                (0u)
/**< VDM version 1.0. Used under USB-PD Revision 2.0. */

#define STD_VDM_VERSION                     (0u)
/**< Default VDM version used. Corresponds to VDM version 1.0. */

#define VSAFE_0V                            (800u)
/**< Vbus voltage = 0.8 V */

#define VSAFE_3_6V                          (3600u)
/**< Vbus voltage = 3.6 V */

#define VSAFE_5V                            (5000u)
/**< Vbus voltage = 5.0 V */

#define VSAFE_9V                            (9000u)
/**< Vbus voltage = 9.0 V */

#define VSAFE_12V                           (12000u)
/**< Vbus voltage = 12.0 V */

#define VSAFE_13V                           (13000u)
/**< Vbus voltage = 13.0 V */

#define VSAFE_15V                           (15000u)
/**< Vbus voltage = 15.0 V */

#define VSAFE_19V                           (19000u)
/**< Vbus voltage = 19.0 V */

#define VSAFE_20V                           (20000u)
/**< Vbus voltage = 20.0 V */

#define VSAFE_0V_PR_SWAP_SNK_SRC            (3000u)
/**< Maximum voltage allowed when PS_RDY is received during a SNK to SRC PR_SWAP. This is set to 3.0 V. */

#define VSAFE_0V_HARD_RESET                 (3000u)
/**< Maximum voltage allowed at the end of a Hard Reset when CCGx is SNK. This is set to 3.0 V. */

#define PD_VOLT_PER_UNIT                    (50u)
/**< Voltage unit used in PDOs. */

#define ISAFE_0A                            (0u)
/**< VBus current usage = 0 A. */

#define ISAFE_DEF                           (50u)
/**< VBus current usage = 0.5 A. */

#define I_1A                                (100)
/**< VBus current usage = 1.0 A. */

#define I_1P5A                              (150)
/**< VBus current usage = 1.5 A. */

#define I_2A                                (200)
/**< VBus current usage = 2.0 A. */

#define I_3A                                (300)
/**< VBus current usage = 3.0 A. */

#define I_5A                                (500)
/**< VBus current usage = 5.0 A. */

#define PD_CUR_PER_UNIT                     (10u)
/**< Current unit used in PDOs. */

#define SNK_DETACH_VBUS_POLL_COUNT      (5u)
/**< Number of VBus checks used to detect detach as sink. */

#define DRP_TOGGLE_PERIOD                   (75u)
/**< Minimum DRP toggling period, in ms. See Table 4-16 of the Type-C spec Rev1. */

#define SRC_DRP_MIN_DC                      (30)
/**< Minimum percentage of DRP period for a source. See Table 4-16 of the Type-C spec Rev1. */

#define CC_CHANNEL_1                        (0u)
/**< Reference to CC_1 pin in the Type-C connector. */

#define CC_CHANNEL_2                        (1u)
/**< Reference to CC_2 pin in the Type-C connector. */

#define TYPEC_FSM_NONE                      (0x00000000u)
/**< Type-C state machine inactive mode. */

#define TYPEC_FSM_GENERIC                   (0x00000001u)
/**< Type-C state machine active mode. */

#define HPD_RX_ACTIVITY_TIMER_PERIOD_MIN    (5u)
/**< Minimum HPD receiver timer period in ms. */

#define HPD_RX_ACTIVITY_TIMER_PERIOD_MAX    (105u)
/**< Maximum HPD receiver timer period in ms. */

#define PD_NO_RESPONSE_TIMER_PERIOD         (5000u)
/**< PD No response timer period in ms. See Section 6.5.7 of USBPD Spec Rev2 v1.2 */

#define PD_CBL_DSC_ID_TIMER_PERIOD          (49u)
/**< Cable discovery timer period in ms. See Section 6.5.15 of USBPD Spec Rev2 v1.2 */

#define PD_CBL_DSC_ID_START_TIMER_PERIOD    (43u)
/**< Cable discovery start period in ms. See Section 6.5.15 of USBPD Spec Rev2 v1.2 */

#define PD_CBL_DELAY_TIMER_PERIOD           (2u)
/**< Cable delay timer period in ms. See Section 6.5.14 of USBPD Spec Rev2 v1.2 */

#define PD_PHY_BUSY_TIMER_PERIOD            (15u)
/**< Period of timer used internally by stack to prevent PHY lockup in TX state. */

#define PD_HARD_RESET_TX_TIMER_PERIOD       (20u)
/**< Hard reset transmit timer period in ms. See Section 6.3.13 of USBPD Spec Rev2 v1.2 */

#define PD_VCONN_SWAP_INITIATOR_TIMER_PERIOD (110u)
/**< This timer used by stack to do auto retry VCONN swap before PR swap (if DUT is sink).
 * Minimum gap between VCONN swap request shall be a minimum 100ms, to be safe 110ms
 * is used.
 */

#define PD_VBUS_TURN_ON_TIMER_PERIOD        (275u)
/**< VBus ON timer period in ms. See Table 7-22 of USBPD Spec Rev2 v1.2 */

#define PD_VBUS_TURN_OFF_TIMER_PERIOD       (625u)
/**< VBus OFF timer period in ms. See Table 7-22 of USBPD Spec Rev2 v1.2 */

#define PD_PS_SRC_TRANS_TIMER_PERIOD        (400u)
/**< Src.Trans timer period in ms. See Section 6.5.6.1 of USBPD Spec Rev2 v1.2 */

#define PD_PS_SRC_OFF_TIMER_PERIOD          (900u)
/**< Src. off timer period in ms. See Section 6.5.6.2 of USBPD Spec Rev2 v1.2 */

#define PD_PS_SRC_ON_TIMER_PERIOD           (450u)
/**< Src. on timer period in ms. See Section 6.5.6.3 of USBPD Spec Rev2 v1.2 */

#define PD_PS_SNK_TRANSITION_TIMER_PERIOD   (500u)
/**< Snk. transition period in ms. See Section 6.5.6.1 of USBPD Spec Rev2 v1.2 */

#define PD_SRC_RECOVER_TIMER_PERIOD         (800u)
/**< Src.Recover timer period in ms. See Section 7.6.1 of USBPD Spec Rev2 v1.2 */

#define PD_SENDER_RESPONSE_TIMER_PERIOD     (27u)
/**< Sender response timeout period in ms. See Section 6.5.2 of USBPD Spec Rev2 v1.2 */

#define PD_RECEIVER_RESPONSE_TIMER_PERIOD   (15u)
/**< Receiver response timeout period in ms. See Section 6.5.2 of USBPD Spec Rev2 v1.2 */

#define PD_SINK_WAIT_CAP_TIMER_PERIOD       (400u)
/**< Snk wait cap period in ms. See Section 6.5.4.2 of USBPD Spec Rev2 v1.2 */

#define PD_SRC_CAP_TIMER_PERIOD             (150u)
/**< Src cap timer period in ms. See Section 6.5.4.1 of USBPD Spec Rev2 v1.2 */

#define PD_SWAP_SRC_START_TIMER_PERIOD      (55u)
/**< Src start (during PR_SWAP) period in ms. See Section 6.5.9.2 of USBPD Spec Rev2 v1.2 */

#define PD_SOURCE_TRANSITION_TIMER_PERIOD   (28u)
/**< Source voltage transition timer period in ms. See Table 7-22 of USBPD Spec Rev2 v1.2 */

#define PD_VCONN_OFF_TIMER_PERIOD           (25u)
/**< Vconn off timer period in ms. See Section 6.5.13 of USBPD Spec Rev2 v1.2 */

#define PD_VCONN_ON_TIMER_PERIOD            (100u)
/**< VConn on timer period in ms. */

#define PD_VCONN_TURN_ON_TIMER_PERIOD       (10u)
/**< Period of VConn monitoring checks done internally. */

#define PD_CBL_READY_TIMER_PERIOD           (50u)
/**< This timer is the delay between PD startup and sending cable Discover ID request
 * to ensure cable is ready to respond. */

#define PD_VDM_RESPONSE_TIMER_PERIOD        (25u)
/**< VDM response timer period in ms. See Section 6.5.12.1 of USBPD Spec Rev2 v1.2 */

#define PD_VDM_ENTER_MODE_RESPONSE_TIMER_PERIOD (45u)
/**< Enter mode response timeout period in ms. See Section 6.5.12.2 of USBPD Spec Rev2 v1.2 */

#define PD_VDM_EXIT_MODE_RESPONSE_TIMER_PERIOD  (45u)
/**< Exit mode response timeout period in ms. See Section 6.5.12.3 of USBPD Spec Rev2 v1.2 */

#define PD_DPM_RESP_REC_RESP_PERIOD         (20u)
/**< VDM receiver response timer period in ms. See Section 6.5.12.1 of USBPD Spec Rev2 v1.2.
 * This timer is slightly relaxed from the spec value.
 */

#define PD_BIST_CONT_MODE_TIMER_PERIOD      (55u)
/**< BIST continuous mode period in ms. See Section 6.5.8.4 of USBPD Spec Rev2 v1.2 */

#define PD_SINK_VBUS_TURN_OFF_TIMER_PERIOD  (750u)
/**< Time in ms allowed for VBus turn off during hard reset. */

#define PD_SINK_VBUS_TURN_ON_TIMER_PERIOD   (1300u)
/**< Time in ms allowed for VBus to turn on during hard reset. */

#define PD_PS_HARD_RESET_TIMER_PERIOD       (27u)
/**< Hard reset timer period in ms. See Section 6.5.11.2 of USBPD Spec Rev2 v1.2 */

#define PD_COLLISION_SRC_COOL_OFF_TIMER_PERIOD      (5u)
/**< Time for which PD 3.0 source will keep Rp as SinkTxNG after returning to Ready state. */

#define PD_SINK_TX_TIMER_PERIOD                     (18u)
/**< Delay between AMS initiation attempts by PD 3.0 sink while Rp = SinkTxNG. */

#define PD_PPS_SRC_TIMER_PERIOD                     (14000u)
/**< PPS timer period in ms. */

#define TYPEC_CC_DEBOUNCE_TIMER_PERIOD              (140u)
/**< CC debounce period in ms from Type-C spec. */

#define TYPEC_PD_DEBOUNCE_TIMER_PERIOD              (11u)
/**< PD debounce period in ms. */

#define TYPEC_RD_DEBOUNCE_TIMER_PERIOD              (12u)
/**< Rd debounce period (detach detection) in ms. */

#define TYPEC_SRC_DETACH_DEBOUNCE_PERIOD            (2u)
/**< Debounce period used to detect detach when we are source. */

#define TYPEC_PD3_RPCHANGE_DEBOUNCE_PERIOD          (2u)
/**< Period in ms used to detect Rp change in a PD 3.0 contract. */

#define TYPEC_ERROR_RECOVERY_TIMER_PERIOD           (50u)
/**< Type-C error recovery timer period in ms. */

#define TYPEC_DRP_TRY_TIMER_PERIOD                  (110u)
/**< Type-C Try DRP timer period in ms. */

#define CCG_FRS_TX_ENABLE_MASK                      (0x02u)
/**< FRS transmit enable flag in config table setting. */

#define CCG_FRS_RX_ENABLE_MASK                      (0x01u)
/**< FRS receive enable flag in config table setting. */

#define CCG_PD_EXT_SRCCAP_SIZE                      (24u)
/**< Size of extended source capabilities message in bytes. */

#define CCG_PD_EXT_SRCCAP_INP_INDEX                 (21u)
/**< Index of Source Inputs field in extended source caps. */

#define CCG_PD_EXT_SRCCAP_INP_UNCONSTRAINED         (0x02u)
/**< Mask for unconstrained source input in extended source caps. */

#define CCG_PD_EXT_SRCCAP_PDP_INDEX                 (23u)
/**< Index of PDP field in extended source caps. */

#define CCG_PD_EXT_STATUS_SIZE                      (5u)
/**< Size of status extended message in bytes. */

#define CCG_PD_EXT_PPS_STATUS_SIZE                  (4u)
/**< Size of PPS status extended message in bytes. */

#define CCG_PD_FIX_SRC_PDO_MASK_REV2                (0xFE3FFFFFu)
/**< Mask to be applied on Fixed Supply Source PDO for PD Rev 2.0 */

#define CCG_PD_FIX_SRC_PDO_MASK_REV3                (0xFF3FFFFFu)
/**< Mask to be applied on Fixed Supply Source PDO for PD Rev 3.0 */

#define CCG_PD_FLAG_CONTRACT_NEG_ACTIVE             (1u)
/**< Status field indicating that contract negotiation is in progress. */

#define CCG_PD_FLAG_EXPLICIT_CONTRACT               (2u)
/**< Status field indicating that explicit contract is present. */

#define CCG_PD_FLAG_SRC_READY                       (4u)
/**< Status field indicating that source is ready. */

#define CCG_PD_FLAG_POWER_SINK                      (8u)
/**< Status field indicating that the port is currently a sink. */

#define CCG_CC_STAT_ZOPEN                           (0u)
/**< CC line status: ZOpen. */

#define CCG_CC_STAT_DRP_TOGGLE                      (1u)
/**< CC line status: DRP toggle in progress. */

#define CCG_CC_STAT_RD_PRESENT                      (2u)
/**< CC line status: Rd is applied. */

#define CCG_CC_STAT_RP_PRESENT                      (4u)
/**< CC line status: Rp is applied. */

#define CCG_CC_STAT_VCONN_ACTIVE                    (8u)
/**< CC line status: VConn is applied. */

#define CCG_DPM_ERROR_NONE                          (0u)
/**< No additional DPM error information available. */

#define CCG_DPM_ERROR_NO_VCONN                      (1u)
/**< DPM command failed due to absence of VConn. */

/** @cond DOXYGEN_HIDE */

/* Macros used internally in the PD stack. Exclude from API guide. */

#define LENGTH_CHECK_SKIP                   (0xFFFF)    /* Macro to skip length check in pd_is_msg function. */
#define CBL_VDO_INDEX                       (4u)        /* Index of cable VDO in received VDM. */
#define CBL_VDO_COUNT                       (5u)        /* Number of expected data objects in D_ID response VDM. */
#define PD_REV_POS                          (6u)        /* Position of PD revision field in message header. */

#define PD_REV_V1_0                         (0u)        /* PD revision 1.0: Not supported by CCGx. */
#define PD_REV_V2_0                         (1u)        /* PD revision 2.0: Default supported revision. */
#define PD_REV_V3_0                         (2u)        /* PD revision 3.0. */
#define PD_REV_RESERVED                     (3u)        /* Reserved/undefined revision. */

#define TX_SOP_GD_CRC_HDR_DFLT              ((PD_REV_V2_0 << PD_REV_POS) | 0x0001u)
#define TX_SOP_PRIME_DPRIME_GD_CRC_HDR_DFLT ((TX_SOP_GD_CRC_HDR_DFLT <<16u) |TX_SOP_GD_CRC_HDR_DFLT)

#define MAX_PD_PKT_WORDS                    (8u)        /* Maximum PD words in one packet. */
#define PD_WORD_SIZE                        (4u)        /* Size of each PD word in bytes. */
#define MAX_PD_PKT_BYTES                    (MAX_PD_PKT_WORDS * PD_WORD_SIZE)

/* Create PD 2.0 header given message type, id and count. */
#define PD_HEADER(type, id, cnt)            \
    ((type) | (PD_REV_V2_0 << 6) | ((id) << 9) | ((cnt) << 12))

/* Create PD 3.0 header given message type, id, count and extended message indication. */
#define PD_HEADER_REV3(type, id, cnt, ext)  \
    ((type) | ((id) << 9) | ((cnt) << 12) | ((ext) << 15))

/* Macro to update DR and PR role in PD header. */
#define PD_DR_PR_ROLE(data_role,pwr_role)   (((pwr_role) << 8) | ((data_role) << 5))

/* Return message count from header. */
#define GET_PD_HDR_CNT(header)              (((header) >> 12) & 0x7)

/* Return PR role bit from header. */
#define GET_PD_HDR_PR_ROLE(header)          (((header) >> 8) & 0x1)

/* Return Cable Plug bit from header. */
#define GET_PD_HDR_CBL_PLUG(header)         (((header) >> 8) & 0x1)

/* Return spec revision from header. */
#define GET_PD_HDR_SPEC_REV(header)         (((header) >> 6) & 0x3)

/* Return DR role bit from header. */
#define GET_PD_HDR_DR_ROLE(header)          (((header) >> 5) & 0x1)

/* Return message ID from header. */
#define GET_PD_HDR_ID(header)               (((header) >> 9) & 0x7)

/* Return Message Type from header. */
#define GET_PD_HDR_TYPE(header)             ((header) & 0xF)

/* PD header mask. */
#define PD_HDR_MASK                         (0x0000FFFFu)

/* Message ID mask. */
#define MSG_ID_MASK                         ((0x7u) << 9u)

/* Return complete PD header Rx buffer[0]. */
#define GET_PD_HDR(buf0)                    ((buf0) & PD_HDR_MASK)

#define GET_RDO_OBJ_POS(rdo)                (((rdo) >> 28) & 0x7)       /* Get object position from RDO. */
#define GET_RDO_GV_BACK(rdo)                (((rdo) >> 27) & 0x1)       /* Get GiveBack flag from RDO. */
#define GET_RDO_CAP_MIS(rdo)                (((rdo) >> 26) & 0x1)       /* Get Cap. mismatch flag from RDO. */
#define GET_RDO_USB_COM(rdo)                (((rdo) >> 25) & 0x1)       /* Get USB comm. support flag from RDO. */
#define GET_RDO_NO_SSPND(rdo)               (((rdo) >> 24) & 0x1)       /* Get USB suspend support flag from RDO. */
#define GET_RDO_OP_CUR(rdo)                 (((rdo) >> 10) & 0x3FF)     /* Get Op. current field from RDO. */
#define GET_RDO_OP_PWR(rdo)                 (((rdo) >> 10) & 0x3FF)     /* Get Op. power field from battery RDO. */
#define GET_RDO_MAX_OP_CUR(rdo)             (((rdo)) & 0x3FF)           /* Get max. op. current from RDO. */
#define GET_RDO_MIN_OP_CUR(rdo)             (((rdo)) & 0x3FF)           /* Get min. op. current from RDO. */
#define GET_RDO_MAX_OP_PWR(rdo)             (((rdo)) & 0x3FF)           /* Get max. op. power from battery RDO. */
#define GET_RDO_MIN_OP_PWR(rdo)             (((rdo)) & 0x3FF)           /* Get min. op. power from battery RDO. */

#define GET_VID(vdm_hdr)                    ((vdm_hdr) >> 16)           /* Get VID from VDM header. */
#define GET_VDM_TYPE(vdm_hdr)               (((vdm_hdr) >> 15) & 0x1)   /* Get VDM type from VDM header. */
#define GET_SVDM_VDM_VER(vdm_hdr)           (((vdm_hdr) >> 13) & 0x3)   /* Get VDM version from VDM header. */
#define GET_SVDM_OBJ_POS(vdm_hdr)           (((vdm_hdr) >> 8) & 0x7)    /* Get object position from VDM header. */
#define GET_SVDM_CMD_TYPE(vdm_hdr)          (((vdm_hdr) >> 6) & 0x3)    /* Get command type from VDM header. */
#define GET_SVDM_CMD(vdm_hdr)               (((vdm_hdr)) & 0x1F)        /* Get command code from VDM header. */

/* VDM header for D_ID command. */
#define STD_VDM_HEADER_IDENTITY_REQ                                             \
                                            (                                   \
                                                (STD_SVID << 16) |              \
                                                (VDM_TYPE_STRUCTURED << 15) |   \
                                                (VDM_CMD_DSC_IDENTITY)          \
                                            )

/* VDM header for D_SVID command. */
#define STD_VDM_HEADER_SVID_REQ                                                 \
                                            (                                   \
                                                (STD_SVID << 16) |              \
                                                (VDM_TYPE_STRUCTURED << 15) |   \
                                                (VDM_CMD_DSC_SVIDS)             \
                                            )

#define GET_BIST_MODE(bist_hdr)             ((bist_hdr) >> 28)          /* Get BIST mode from BIST header. */

/* Message masks for control messages. */
#define CTRL_MSG_RSRVD_MASK                 (0x1 << CTRL_MSG_RSRVD)
#define CTRL_MSG_GOOD_CRC_MASK              (0x1 << CTRL_MSG_GOOD_CRC)
#define CTRL_MSG_GO_TO_MIN_MASK             (0x1 << CTRL_MSG_GO_TO_MIN)
#define CTRL_MSG_ACCEPT_MASK                (0x1 << CTRL_MSG_ACCEPT)
#define CTRL_MSG_REJECT_MASK                (0x1 << CTRL_MSG_REJECT)
#define CTRL_MSG_PING_MASK                  (0x1 << CTRL_MSG_PING)
#define CTRL_MSG_PS_RDY_MASK                (0x1 << CTRL_MSG_PS_RDY)
#define CTRL_MSG_GET_SOURCE_CAP_MASK        (0x1 << CTRL_MSG_GET_SOURCE_CAP)
#define CTRL_MSG_GET_SINK_CAP_MASK          (0x1 << CTRL_MSG_GET_SINK_CAP)
#define CTRL_MSG_DR_SWAP_MASK               (0x1 << CTRL_MSG_DR_SWAP)
#define CTRL_MSG_PR_SWAP_MASK               (0x1 << CTRL_MSG_PR_SWAP)
#define CTRL_MSG_VCONN_SWAP_MASK            (0x1 << CTRL_MSG_VCONN_SWAP)
#define CTRL_MSG_WAIT_MASK                  (0x1 << CTRL_MSG_WAIT)
#define CTRL_MSG_SOFT_RESET_MASK            (0x1 << CTRL_MSG_SOFT_RESET)
#define CTRL_MSG_NOT_SUPPORTED_MASK         (0x1 << CTRL_MSG_NOT_SUPPORTED)
#define CTRL_MSG_GET_SRC_CAP_EXTD_MASK      (0x1 << CTRL_MSG_GET_SRC_CAP_EXTD)
#define CTRL_MSG_GET_STATUS_MASK            (0x1 << CTRL_MSG_GET_STATUS)
#define CTRL_MSG_FR_SWAP_MASK               (0x1 << CTRL_MSG_FR_SWAP)

/* Message masks for data messages. */
#define DATA_MSG_SRC_CAP_MASK               (0x1 << DATA_MSG_SRC_CAP)
#define DATA_MSG_REQUEST_MASK               (0x1 << DATA_MSG_REQUEST)
#define DATA_MSG_BIST_MASK                  (0x1 << DATA_MSG_BIST)
#define DATA_MSG_SNK_CAP_MASK               (0x1 << DATA_MSG_SNK_CAP)
#define DATA_MSG_BAT_STATUS_MASK            (0x1 << DATA_MSG_BAT_STATUS)
#define DATA_MSG_SRC_ALERT_MASK             (0x1 << DATA_MSG_ALERT)
#define DATA_MSG_VDM_MASK                   (0x1 << DATA_MSG_VDM)

/* Message masks for extended data messages. */
#define EXTD_MSG_SRC_CAP_EXTD_MASK          (0x1 << EXTD_MSG_SRC_CAP_EXTD)
#define EXTD_MSG_STATUS_MASK                (0x1 << EXTD_MSG_STATUS)
#define EXTD_MSG_GET_BAT_CAP_MASK           (0x1 << EXTD_MSG_GET_BAT_CAP)
#define EXTD_MSG_GET_BAT_STATUS_MASK        (0x1 << EXTD_MSG_GET_BAT_STATUS)
#define EXTD_MSG_BAT_CAP_MASK               (0x1 << EXTD_MSG_BAT_CAP)
#define EXTD_MSG_GET_MANU_INFO_MASK         (0x1 << EXTD_MSG_GET_MANU_INFO)
#define EXTD_MSG_MANU_INFO_MASK             (0x1 << EXTD_MSG_MANU_INFO)
#define EXTD_MSG_SECURITY_REQ_MASK          (0x1 << EXTD_MSG_SECURITY_REQ)
#define EXTD_MSG_SECURITY_RESP_MASK         (0x1 << EXTD_MSG_SECURITY_RESP)
#define EXTD_MSG_FW_UPDATE_REQ_MASK         (0x1 << EXTD_MSG_FW_UPDATE_REQ)
#define EXTD_MSG_FW_UPDATE_RESP_MASK        (0x1 << EXTD_MSG_FW_UPDATE_RESP)

#define CBL_CAP_0A                          (0u)        /* Cable current capability: 0 A. */
#define CBL_CAP_3A                          (300u)      /* Cable current capability: 3 A. */
#define CBL_CAP_5A                          (500u)      /* Cable current capability: 5 A. */

/*
 * Default cable current capability assumed by the stack. Please use
 * dpm_update_def_cable_cap() to change this value.
 */
#define DEFAULT_CBL_CAP                     (CBL_CAP_3A)

#define VSAFE_0V_SNK_MARGIN                 (0)         /* Allowed margin (%) for VSafe_0 measurement as sink. */
#define VSAFE_0V_PR_SWAP_SNK_SRC_MARGIN     (0)         /* Allowed margin (%) on VBus measurement during PR_SWAP. */
#define VSAFE_0V_HARD_RESET_MARGIN          (0)         /* Allowed margin (%) on VBus measurement during HARD RESET. */
#define VSAFE_0V_SRC_MARGIN                 (-50)
#define VSAFE_0V_SRC_TURN_ON_MARGIN         (0)
#define VSAFE_5V_SNK_TURN_ON_MARGIN         (-20)
#define VSAFE_5V_SNK_TURN_OFF_MARGIN        (-27)
#define VSAFE_SNK_TURN_OFF_MARGIN           (-20)
#define VSAFE_5V_SRC_TURN_ON_MARGIN         (-20)
#define VSAFE_5V_FRS_SWAP_RX_MARGIN         (10)
#define VSAFE_5V_FRS_SWAP_TX_MARGIN         (10)

/* Masks for Policy Engine related events. */
#define PE_EVT_NONE                         (0x00000000u)       /* No Policy Engine events pending. */
#define PE_EVT_HARD_RESET_RCVD              (0x00000001u)       /* Hard Reset received event pending. */
#define PE_EVT_SOFT_RESET_RCVD              (0x00000002u)       /* Soft Reset received event pending. */
#define PE_EVT_ENTRY                        (0x00000004u)       /* Entry to new state machine. */
#define PE_EVT_TX_SUCCESS                   (0x00000008u)       /* PD message transmission completed. */
#define PE_EVT_TX_DISCARDED                 (0x00000010u)       /* PD message discarded due to collission. */
#define PE_EVT_TX_FAIL                      (0x00000020u)       /* PD message transmission failed. */
#define PE_EVT_PKT_RCVD                     (0x00000040u)       /* New PD message received. */
#define PE_EVT_PWR_RDY                      (0x00000080u)       /* Power ready (SRC/SNK transition complete) state. */
#define PE_EVT_TIMEOUT                      (0x00000100u)       /* Timeout event. */
#define PE_EVT_DPM_CMD_RCVD                 (0x00000200u)       /* DPM command received from APP layer. */
#define PE_EVT_APP_RESP_RCVD                (0x00000400u)       /* Response for PD message received from APP layer. */
#define PE_EVT_VDM_RESP_RCVD                (0x00000800u)       /* Response for VDM received from APP layer. */
#define PE_EVT_CABLE_TIMEOUT                (0x00001000u)       /* Cable access timed out. */
#define PE_EVT_NO_RESPONSE_TIMEOUT          (0x00002000u)       /* Response timeout event. */
#define PE_EVT_FR_SIGNAL_RCVD               (0x00004000u)       /* FR_Swap signalling received. */
#define PE_EVT_FR_SIGNAL_SENT               (0x00008000u)       /* FR_Swap signalling sent. */
#define PE_EVT_PPS_TIMEOUT                  (0x00010000u)       /* PPS timeout occurred. */
#define PE_EVT_ALL_MASK                     (0xFFFFFFFFu)       /* Mask to extract all pending events. */

/* Rp combinations macros. */
#define RP_CC1_OPEN_CC2_OPEN                (((RP_OPEN) << 8)|RP_OPEN) /* CCG Rp: No external termination on both CC. */
#define RP_CC1_OPEN_CC2_RD                  (((RP_RD) << 8)|RP_OPEN)   /* CCG Rp: external Rd on CC2. */
#define RP_CC1_OPEN_CC2_RA                  (((RP_RA) << 8)|RP_OPEN)   /* CCG Rp: external Ra on CC2. */
#define RP_CC1_RD_CC2_OPEN                  (((RP_OPEN) << 8)|RP_RD)   /* CCG Rp: external Rd on CC1. */
#define RP_CC1_RA_CC2_OPEN                  (((RP_OPEN) << 8)|RP_RA)   /* CCG Rp: external Ra on CC1. */
#define RP_CC1_RA_CC2_RD                    (((RP_RD) << 8)|RP_RA)     /* CCG Rp: Rd on CC2, Ra on CC1. */
#define RP_CC1_RA_CC2_RA                    (((RP_RA) << 8)|RP_RA)     /* CCG Rp: Ra on both CC. */
#define RP_CC1_RD_CC2_RA                    (((RP_RA) << 8)|RP_RD)     /* CCG Rp: Rd on CC1, Ra on CC2. */
#define RP_CC1_RD_CC2_RD                    (((RP_RD) << 8)|RP_RD)     /* CCG Rp: Rd on both CC. */

/* Rd combinations macros. */
#define RD_CC1_RA_CC2_USB                   (((RD_USB) << 8)|RD_RA)    /* CCG Rd: External Default Rp on CC2. */
#define RD_CC1_RA_CC2_1_5A                  (((RD_1_5A) << 8)|RD_RA)   /* CCG Rd: 1.5A Rp on CC2. */
#define RD_CC1_RA_CC2_3A                    (((RD_3A) << 8)|RD_RA)     /* CCG Rd: 3A Rp on CC2. */
#define RD_CC1_USB_CC2_RA                   (((RD_RA) << 8)|RD_USB)    /* CCG Rd: External Default Rp on CC1. */
#define RD_CC1_1_5A_CC2_RA                  (((RD_RA) << 8)|RD_1_5A)   /* CCG Rd: 1.5A Rp on CC1. */
#define RD_CC1_3A_CC2_RA                    (((RD_RA) << 8)|RD_3A)     /* CCG Rd: 3A Rp on CC1. */
#define RD_CC1_RA_CC2_RA                    (((RD_RA) << 8)|RD_RA)     /* CCG Rd: No Rp on either CC. */
#define RD_CC1_ERR_CC2_ERR                  (((RD_ERR) << 8)|RD_ERR)   /* CCG Rd: Error state. */

/* Type-C events macros. */
#define TYPEC_EVT_NONE                      (0x00000000u)               /* No pending Type-C events. */
#define TYPEC_EVT_ERR_RECOVERY              (0x00000001u)               /* Type-C error recovery state. */
#define TYPEC_EVT_ENTRY                     (0x00000002u)               /* Entry to new Type-C state. */
#define TYPEC_EVT_DETACH                    (0x00000004u)               /* Detach event detected. */
#define TYPEC_EVT_ATTACH                    (0x00000008u)               /* Attach event detected. */
#define TYPEC_EVT_PWR_RDY                   (0x00000010u)               /* Power Ready (SRC/SNK transition complete) */
#define TYPEC_EVT_TIMEOUT1                  (0x00000020u)               /* Timeout event #1 */
#define TYPEC_EVT_TIMEOUT2                  (0x00000040u)               /* Timeout event #2 */
#define TYPEC_EVT_DPM_CMD_RCVD              (0x00000080u)               /* DPM command received from APP layer. */
#define TYPEC_EVT_ALL_MASK                  (0xFFFFFFFFu)               /* Mask to get all events. */

/* This is an internal timer used to prevent RX lock up */
#define PD_GOOD_CRC_TX_TIMER_PERIOD         (3u)

#define PD_FRS_SRC_SNK_MAX_WAIT_FOR_FR_SWAP (20u)
#define PD_FRS_SNK_SRC_MAX_WAIT_FOR_RP      (14u)

/* TYPEC_SNK_TRY_TIMER_PERIOD must be more than TYPEC_PD_DEBOUNCE_TIMER_PERIOD */
#define TYPEC_SNK_TRY_TIMER_PERIOD          (18u)
#define TYPEC_DRP_TIMER_PERIOD              (37u)
#define TYPEC_VBUS_DISCHARGE_TIMER_PERIOD   (50u)
#define VBUS_TURN_ON_TIMER_PERIOD           (250u)
#define SRC_DISCONNECT_WAIT_PERIOD          (1u)
#define VCONN_TURN_ON_TIMER_PERIOD          (10u)
#define TYPEC_SINK_VBUS_DISCHARGE_PERIOD    (275u)
#define TYPEC_ACTIVITY_TIMER_PERIOD         (20u)
#define TYPEC_SYNC_TOGGLE_PERIOD            (30u)

#define PD_EXTD_STATUS_EVT_CF_POS           (4u)
#define PD_EXTD_PPS_STATUS_EVT_CF_POS       (3u)

/** @endcond */

/*******************************************************************************
 * Enumerated Data Definitions
 ******************************************************************************/

/**
 * @typedef pd_err_recov_reason_t
 * @brief Enumeration of reasons for error recovery entry.
 */
typedef enum {
    ERR_RECOV_REASON_NONE = 0,          /**< Error recovery is not active. */
    ERR_RECOV_HR_FAIL,                  /**< Error recovery due to hard reset failure. */
    ERR_RECOV_PROTECT_FAULT,            /**< Error recovery due to protection (OVP/OCP) fault. */
    ERR_RECOV_POWER_FAULT,              /**< Error recovery due to voltage fault. */
    ERR_RECOV_BAD_DATA_ROLE,            /**< Error recovery due to bad data role in incoming PD message. */
    ERR_RECOV_FRS_FAIL                  /**< Error recovery due to Fast Role Swap error. */
} pd_err_recov_reason_t;

/**
 * @typedef pd_emca_sr_reason_t
 * @brief Enumeration of possible reasons to issue an EMCA (SOP' or SOP'') soft reset.
 */
typedef enum {
    EMCA_SR_REASON_NONE = 0,            /**< No EMCA soft-reset in progress. */
    EMCA_SR_CABLE_DISC,                 /**< EMCA soft-reset due to cable discovery. */
    EMCA_SR_ALT_MODE_DISC               /**< EMCA soft-reset due to alt mode discovery. */
} pd_emca_sr_reason_t;

/**
 * @typedef pd_cable_reset_reason_t
 * @brief Enumeration of reasons for issuing a Cable Reset.
 */
typedef enum {
    EMCA_CABLE_RES_NONE = 0,            /**< No Cable Reset performed. */
    EMCA_CABLE_RES_SR_TIMEOUT           /**< SOP' or SOP'' SoftReset timed out. */
} pd_cable_reset_reason_t;

/**
 * @typedef pd_hard_reset_reason_t
 * @brief Enumeration of reasons for issuing a hard reset.
 */
typedef enum {
    PD_HARDRES_REASON_NONE = 0,         /**< HardReset not issued. */
    PD_HARDRES_REASON_NO_SRC_CAP,       /**< No Source Capability messages received. */
    PD_HARDRES_REASON_HOSTCONN,         /**< TBT Host Connect state change. */
    PD_HARDRES_REASON_SR_ERROR,         /**< SoftReset failed. */
    PD_HARDRES_REASON_CONTRACT_ERROR,   /**< Power contract failed. */
    PD_HARDRES_REASON_DRSWAP,           /**< DR Swap received while in Alternate Mode. */
    PD_HARDRES_REASON_VBUS_OVP,         /**< Over-Voltage condition detected. */
    PD_HARDRES_REASON_VBUS_OCP,         /**< Over-Current condition detected. */
    PD_HARDRES_REASON_AMS_ERROR,        /**< PD Atomic Message Sequence error. */
} pd_hard_reset_reason_t;

/**
 * @typedef pd_soft_reset_reason_t
 * @brief Enumeration of reasons for issuing a soft reset.
 */
typedef enum {
    PD_SOFTRES_REASON_NONE = 0,         /**< SoftReset not issued. */
    PD_SOFTRES_REASON_SRCNEG_ERROR,     /**< Contract negotiation error when CCGx is source. */
    PD_SOFTRES_REASON_SNKNEG_ERROR,     /**< Contract negotiation error when CCGx is sink. */
    PD_SOFTRES_REASON_AMS_ERROR         /**< PD protocol error. */
} pd_soft_reset_reason_t;

/**
 * @typedef pd_timer_id_t
 * @brief Soft timer ID assignments for various Type-C, USB-PD and Application level periodic tasks.
 * These timer IDs should not be changed; and new timer IDs should be chosen from free ranges.
 *
 * Please refer to timer definitions in app.h to avoid conflicts with timer IDs used in default App
 * layer code.
 */
typedef enum {
    PD_TIMERS_START_ID            = 0,                  /**< 000: Start index for USB-PD stack timers. */
    PD_CABLE_TIMER                = 1u,                 /**< 001: Timer used for cable capability check. */
    PD_NO_RESPONSE_TIMER          = 2u,                 /**< 002: Response timer. */
    PD_CBL_DSC_ID_TIMER           = 3u,                 /**< 003: Timer used for cable discovery state machine. */
    PD_CBL_DELAY_TIMER            = 4u,                 /**< 004: Timer used to enforce cable delay. */
    PD_PHY_BUSY_TIMER             = 5u,                 /**< 005: Timer used to handle PHY busy status. */
    PD_GOOD_CRC_TX_TIMER          = 6u,                 /**< 006: GoodCRC timer. */
    PD_HARD_RESET_TX_TIMER        = 7u,                 /**< 007: Hard reset transmit timer. */
    PD_VCONN_SWAP_INITIATOR_TIMER = 8u,                 /**< 008: VConn swap initiator timer. */
    PD_GENERIC_TIMER              = 9u,                 /**< 009: Generic AMS timer. */
    PD_PPS_TIMER                  = 10u,                /**< 010: PPS related timer. */
    PD_SINK_TX_TIMER              = 11u,                /**< 011: PD 3.0 sink Rp flow control timer. */
    PD_TIMERS_END_ID              = 14u,                /**< 014: End index (inclusive) for USB-PD stack timers. */

    TYPEC_TIMERS_START_ID         = 15u,                /**< 015: Start index for Type-C timers. */
    TYPEC_CC1_DEBOUNCE_TIMER      = 15u,                /**< 015: Timer used for CC1 debounce. */
    TYPEC_CC2_DEBOUNCE_TIMER      = 16u,                /**< 016: Timer used for CC2 debounce. */
    TYPEC_RD_DEBOUNCE_TIMER       = 17u,                /**< 017: Timer used for Rd debounce. */
    TYPEC_VBUS_DISCHARGE_TIMER    = 18u,                /**< 018: VBus discharge timer id. */
    TYPEC_ACTIVITY_TIMER          = 19u,                /**< 019: Type-C activity timer id. */
    TYPEC_GENERIC_TIMER1          = 20u,                /**< 020: Generic Type-C state machine timer #1. */
    TYPEC_GENERIC_TIMER2          = 21u,                /**< 021: Generic Type-C state machine timer #2. */
    TYPEC_TIMERS_END_ID           = 24u,                /**< 024: End index (inclusive) for Type-C timers. */

    PD_OCP_DEBOUNCE_TIMER         = 25u,                /**< 025: Timer used for FW debounce of VBus OCP. */
    HPD_RX_ACTIVITY_TIMER_ID      = 26u,                /**< 026: Timer used for HPD receive handling. */
    PD_VCONN_OCP_DEBOUNCE_TIMER   = 27u,                /**< 027: Timer used for FW debounce of VConn OCP. */
    VBUS_DISCHARGE_SCHEDULE_TIMER = 29u,                /**< 029: Timer used to monitor VBus discharge operation. */

    APP_TIMERS_START_ID           = 30u                 /**< 030: Start index for Application level timers. */
} pd_timer_id_t;

/**
 * @typedef app_fault_mask_t
 *
 * @brief Fault handling enable/disable masks for use in the configuration table.
 * The respective fault handling will only be enabled if the corresponding bit is set in the
 * protection_enable bit in the configuration table.
 */
typedef enum {
    CFG_TABLE_OVP_EN_MASK       = 0x01u, /**< VBUS Over-Voltage fault handling enable mask. */
    CFG_TABLE_OCP_EN_MASK       = 0x02u, /**< VBUS Over-Current fault handling enable mask. */
    CFG_TABLE_UVP_EN_MASK       = 0x04u, /**< VBUS Under-Voltage fault handling enable mask. */
    CFG_TABLE_SCP_EN_MASK       = 0x08u, /**< VBUS Short-Circuit fault handling enable mask. */
    CFG_TABLE_VCONN_OCP_EN_MASK = 0x10u, /**< VCONN Over-Current fault handling enable mask. */
    CFG_TABLE_OTP_EN_MASK       = 0x20u  /**< Over-Temperature fault handling enable mask. */
} app_fault_mask_t;

/**
 * @typedef app_swap_resp_t
 *
 * @brief Possible responses to various USB-PD swap requests from the application layer.
 * The PD stack hands the requests up to the application for handling and gets directions
 * on handling the request in the form of these response codes.
 */
typedef enum {
    APP_RESP_ACCEPT         = 0u,       /**< Swap request should be accepted. */
    APP_RESP_REJECT,                    /**< Swap request should be rejected. */
    APP_RESP_WAIT,                      /**< Swap request handling should be delayed (send wait response). */
    APP_RESP_NOT_SUPPORTED              /**< Swap request is not supported. */
} app_swap_resp_t;

/**
 * @typedef pd_rev_t
 * @brief Enumeration of the PD spec revisions.
 */
typedef enum
{
    PD_REV1 = 0,                        /**< USB-PD spec revision 1.0. Not supported. */
    PD_REV2,                            /**< USB-PD spec revision 2.0. */
    PD_REV3,                            /**< USB-PD spec revision 3.0. */
    PD_REV_RSVD                         /**< Undefined USB-PD spec revision. */
} pd_rev_t;

/**
 * @typedef msg_class_t
 * @brief Enum of the PD message types.
 */
typedef enum
{
    PD_CTRL_MSG = 0,                    /**< Control message. */
    PD_DATA_MSG,                        /**< Data message. */
    PD_EXTD_MSG,                        /**< Extended data message. */
    PD_CABLE_RESET,                     /**< Cable reset message. */
    PD_MSG_RSVD                         /**< Undefined message type. */
} pd_msg_class_t;

/**
 * @typedef rdo_type_t
 * @brief Enum of the RDO types.
 */
typedef enum
{
    FIXED_VAR_RDO = 0,                  /**< Fixed or variable supply request data object. */
    BAT_RDO                             /**< Battery request data object. */
} rdo_type_t;

/**
 * @typedef ctrl_msg_t
 * @brief Enum of the control message types.
 */
typedef enum
{
    CTRL_MSG_RSRVD = 0,                 /**< 0x00: Reserved message code. */
    CTRL_MSG_GOOD_CRC,                  /**< 0x01: GoodCRC message. */
    CTRL_MSG_GO_TO_MIN,                 /**< 0x02: GotoMin message. */
    CTRL_MSG_ACCEPT,                    /**< 0x03: Accept message. */
    CTRL_MSG_REJECT,                    /**< 0x04: Reject message. */
    CTRL_MSG_PING,                      /**< 0x05: Ping message. */
    CTRL_MSG_PS_RDY,                    /**< 0x06: PS_RDY message. */
    CTRL_MSG_GET_SOURCE_CAP,            /**< 0x07: Get_Source_Cap message. */
    CTRL_MSG_GET_SINK_CAP,              /**< 0x08: Get_Sink_Cap message. */
    CTRL_MSG_DR_SWAP,                   /**< 0x09: DR_Swap message. */
    CTRL_MSG_PR_SWAP,                   /**< 0x0A: PR_Swap message. */
    CTRL_MSG_VCONN_SWAP,                /**< 0x0B: VCONN_Swap message. */
    CTRL_MSG_WAIT,                      /**< 0x0C: Wait message. */
    CTRL_MSG_SOFT_RESET,                /**< 0x0D: Soft_Reset message. */
    CTRL_MSG_NOT_SUPPORTED = 16,        /**< 0x10: Not_Supported message. */
    CTRL_MSG_GET_SRC_CAP_EXTD,          /**< 0x11: Get_Source_Cap_Extended message. */
    CTRL_MSG_GET_STATUS,                /**< 0x12: Get_Status message . */
    CTRL_MSG_FR_SWAP,                   /**< 0x13: FR_Swap message. */
    CTRL_MSG_GET_PPS_STATUS,            /**< 0x14: Get_PPS_Status message. */
    CTRL_MSG_GET_COUNTRY_CODES          /**< 0x15: Get_Country_Codes message. */
} ctrl_msg_t;

/**
 * @typedef data_msg_t
 * @brief Enum of the data message types.
 */
typedef enum
{
    DATA_MSG_SRC_CAP = 1,               /**< 0x01: Source_Capabilities message. */
    DATA_MSG_REQUEST,                   /**< 0x02: Request message. */
    DATA_MSG_BIST,                      /**< 0x03: BIST message. */
    DATA_MSG_SNK_CAP,                   /**< 0x04: Sink_Capabilities message. */
    DATA_MSG_BAT_STATUS,                /**< 0x05: Battery_Status message. */
    DATA_MSG_ALERT,                     /**< 0x06: Alert message. */
    DATA_MSG_GET_COUNTRY_INFO,          /**< 0x07: Get_Country_Info message. */
    DATA_MSG_VDM = 15                   /**< 0x0F: Vendor_Defined message. */
} data_msg_t;

/**
 * @typedef extd_msg_t
 * @brief Enum of the extended data message types.
 */
typedef enum
{
    EXTD_MSG_SRC_CAP_EXTD = 1,          /**< 0x01: Source_Capabilities_Extended message. */
    EXTD_MSG_STATUS,                    /**< 0x02: Status message. */
    EXTD_MSG_GET_BAT_CAP,               /**< 0x03: Get_Battery_Cap message. */
    EXTD_MSG_GET_BAT_STATUS,            /**< 0x04: Get_Battery_Status message. */
    EXTD_MSG_BAT_CAP,                   /**< 0x05: Battery_Capabilities message. */
    EXTD_MSG_GET_MANU_INFO,             /**< 0x06: Get_Manufacturer_Info message. */
    EXTD_MSG_MANU_INFO,                 /**< 0x07: Manufacturer_Info message. */
    EXTD_MSG_SECURITY_REQ,              /**< 0x08: Security_Request message. */
    EXTD_MSG_SECURITY_RESP,             /**< 0x09: Security_Response message. */
    EXTD_MSG_FW_UPDATE_REQ,             /**< 0x0A: Firmware_Update_Request message. */
    EXTD_MSG_FW_UPDATE_RESP,            /**< 0x0B: Firmware_Update_Response message. */
    EXTD_MSG_PPS_STATUS,                /**< 0x0C: PPS_Status message. */
    EXTD_MSG_COUNTRY_INFO,              /**< 0x0D: Country_Info message. */
    EXTD_MSG_COUNTRY_CODES              /**< 0x0E: Country_Codes message. */
} extd_msg_t;

/**
 * @typedef pdo_t
 * @brief Enum of the PDO types.
 */
typedef enum
{
    PDO_FIXED_SUPPLY = 0,               /**< Fixed (voltage) supply power data object. */
    PDO_BATTERY,                        /**< Battery based power data object. */
    PDO_VARIABLE_SUPPLY,                /**< Variable (voltage) supply power data object. */
    PDO_AUGMENTED                       /**< Augmented power data object. */
} pdo_t;

/**
 * @typedef apdo_t
 * @brief Enum of the Augmented PDO types.
 */
typedef enum
{
    APDO_PPS = 0,                       /**< Programmable Power Supply PDO. */
    APDO_RSVD1,                         /**< Reserved for future use. */
    APDO_RSVD2,                         /**< Reserved for future use. */
    APDO_RSVD3                          /**< Reserved for future use. */
} apdo_t;

/**
 * @typedef peak_cur_cap_t
 * @brief Enum of Peak Current Capability levels.
 */
typedef enum
{
    IMAX_EQ_IOC = 0,                    /**< Peak current equal to operating current. */
    IMAX_EQ_130_IOC,                    /**< Peak current is 1.3x operating current. */
    IMAX_EQ_150_IOC,                    /**< Peak current is 1.5x operating current. */
    IMAX_EQ_200_IOC                     /**< Peak current is 2x operating current. */
} peak_cur_cap_t;

/**
 * @typedef bist_mode_t
 * @brief Enum of the BIST modes.
 */
typedef enum
{
    BIST_RX_MODE = 0,                   /**< BIST receiver mode. */
    BIST_TX_MODE,                       /**< BIST transmit mode. */
    BIST_RETURN_COUNTERS_MODE,          /**< Send Returned BIST counters response. */
    BIST_CARRIER_MODE_0,                /**< BIST carrier mode 0. */
    BIST_CARRIER_MODE_1,                /**< BIST carrier mode 1. */
    BIST_CARRIER_MODE_2,                /**< BIST carrier mode 2. */
    BIST_CARRIER_MODE_3,                /**< BIST carrier mode 3. */
    BIST_EYE_PATTERN_MODE,              /**< BIST eye pattern. */
    BIST_TEST_DATA_MODE                 /**< BIST test data mode. */
} bist_mode_t;

/**
 * @typedef sop_t
 * @brief Enum of the SOP (Start Of Frame) types.
 */
typedef enum
{
    SOP = 0,                            /**< SOP: Used for communication with port partner. */
    SOP_PRIME,                          /**< SOP': Cable marker communication. */
    SOP_DPRIME,                         /**< SOP'': Cable marker communication. */
    SOP_P_DEBUG,                        /**< SOP'_Debug */
    SOP_DP_DEBUG,                       /**< SOP''_Debug */
    HARD_RESET,                         /**< Hard Reset */
    CABLE_RESET,                        /**< Cable Reset */
    SOP_INVALID                         /**< Undefined ordered set. */
} sop_t;

/**
 * @typedef port_role_t
 * @brief Enum of the PD port roles.
 */
typedef enum
{
    PRT_ROLE_SINK = 0,                  /**< Power sink */
    PRT_ROLE_SOURCE,                    /**< Power source */
    PRT_DUAL                            /**< Dual Role Power device: can be source or sink. */
} port_role_t;

/**
 * @typedef port_type_t
 * @brief Enum of the PD port types.
 */
typedef enum
{
    PRT_TYPE_UFP = 0,                   /**< Upstream facing port. USB device or Alternate mode accessory. */
    PRT_TYPE_DFP,                       /**< Downstream facing port. USB host or Alternate mode controller. */
    PRT_TYPE_DRP                        /**< Dual Role data device: can be UFP or DFP. */
} port_type_t;

/**
 * @typedef pd_fr_swap_supp_t
 * @brief Enum to hold FR swap options in sink capabilities
 */
typedef enum
{
    FR_SWAP_NOT_SUPPORTED = 0,          /**< FR_Swap is not supported. */
    FR_SWAP_DEF_USB,                    /**< Device will sink less than 900 mA of current after FR_Swap. */
    FR_SWAP_1_5A,                       /**< Device will sink less than 1.5 A of current after FR_Swap. */
    FR_SWAP_3A                          /**< Device will sink less than 3 A of current after FR_SWAP. */
}fr_swap_supp_t;

/**
 * @typedef app_req_status_t
 * @brief Enum of the PD Request results. Enum fields map to the control
 * message field in the PD spec.
 */
typedef enum
{
    REQ_SEND_HARD_RESET = 1,            /**< Invalid message. Send Hard Reset. */
    REQ_ACCEPT = 3,                     /**< Send Accept message. */
    REQ_REJECT = 4,                     /**< Send Reject message. */
    REQ_WAIT = 12,                      /**< Send Wait message. */
    REQ_NOT_SUPPORTED = 16              /**< Send Not_Supported message. Will translate to Reject message under PD 2.0 */
} app_req_status_t;

/**
 * @typedef resp_status_t
 * @brief Enum of the response status to DPM commands.
 */
typedef enum
{
    SEQ_ABORTED = 0,                    /**< PD AMS aborted. */
    CMD_FAILED,                         /**< PD AMS failed. */
    RES_TIMEOUT,                        /**< No response received. */
    CMD_SENT,                           /**< PD command has been sent. Response wait may be in progress. */
    RES_RCVD                            /**< Response received. */
} resp_status_t;

/**
 * @typedef dpm_pd_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) command types.
 */
typedef enum
{
    DPM_CMD_SRC_CAP_CHNG = 0,           /**< Source Caps changed notification. Can be used to trigger fresh contract. */
    DPM_CMD_SNK_CAP_CHNG,               /**< Sink Caps changed notification. Can be used to trigger fresh contract. */
    DPM_CMD_SEND_GO_TO_MIN,             /**< Send GotoMin message to port partner. */
    DPM_CMD_GET_SNK_CAP,                /**< Send Get_Sink_Cap message to port partner. */
    DPM_CMD_GET_SRC_CAP,                /**< Send Get_Source_Cap message to port partner. */
    DPM_CMD_SEND_HARD_RESET,            /**< Send Hard Reset. */
    DPM_CMD_SEND_SOFT_RESET,            /**< Send Soft Reset to port partner. */
    DPM_CMD_SEND_CABLE_RESET,           /**< Send Cable Reset. */
    DPM_CMD_SEND_SOFT_RESET_EMCA,       /**< Send Soft Reset to cable marker. */
    DPM_CMD_SEND_DR_SWAP,               /**< Send DR_Swap request. */
    DPM_CMD_SEND_PR_SWAP,               /**< Send PR_Swap request. */
    DPM_CMD_SEND_VCONN_SWAP,            /**< Send VCONN_Swap request. */
    DPM_CMD_SEND_VDM,                   /**< Send VDM message. */
    DPM_CMD_SEND_EXTENDED,              /**< Send extended data message. */
    DPM_CMD_GET_SRC_CAP_EXTENDED,       /**< Send Get_Source_Cap_Extended message. */
    DPM_CMD_GET_STATUS,                 /**< Send Get_Status message. */
    DPM_CMD_SEND_BATT_STATUS,           /**< Send Battery_Status data message. */
    DPM_CMD_SEND_ALERT,                 /**< Send Alert message. */
    DPM_CMD_SEND_NOT_SUPPORTED,         /**< Send Not_Supported message. */
    DPM_CMD_SEND_INVALID = 0xFFu        /**< Invalid command code. */
} dpm_pd_cmd_t;

/**
 * @typedef pd_devtype_t
 * @brief Enum of the attached device type.
 */
typedef enum
{
    DEV_SNK = 1,                        /**< Power sink device is attached. */
    DEV_SRC,                            /**< Power source device is attached. */
    DEV_DBG_ACC,                        /**< Debug accessory is attached. */
    DEV_AUD_ACC,                        /**< Audio accessory is attached. */
    DEV_PWRD_ACC,                       /**< Powered accessory is attached. */
    DEV_UNSUPORTED_ACC                  /**< Unsupported device type is attached. */
} pd_devtype_t;

/**
 * @typedef vdm_type_t
 * @brief Enum of the VDM types.
 */
typedef enum
{
    VDM_TYPE_UNSTRUCTURED = 0,          /**< Unstructured VDM. */
    VDM_TYPE_STRUCTURED                 /**< Structured VDM. */
} vdm_type_t;

/**
 * @typedef std_vdm_cmd_t
 * @brief Enum of the standard VDM commands.
 */
typedef enum
{
    VDM_CMD_DSC_IDENTITY = 1,           /**< Discover Identity command. */
    VDM_CMD_DSC_SVIDS,                  /**< Discover SVIDs command. */
    VDM_CMD_DSC_MODES,                  /**< Discover Modes command. */
    VDM_CMD_ENTER_MODE,                 /**< Enter Mode command. */
    VDM_CMD_EXIT_MODE,                  /**< Exit Mode command. */
    VDM_CMD_ATTENTION,                  /**< Attention message. */
    VDM_CMD_DP_STATUS_UPDT = 16,        /**< DisplayPort Status Update message. */
    VDM_CMD_DP_CONFIGURE = 17           /**< DisplayPort Configure command. */
} std_vdm_cmd_t;

/**
 * @typedef std_vdm_cmd_type_t
 * @brief Enum of the standard VDM command types.
 */
typedef enum
{
    CMD_TYPE_INITIATOR = 0,             /**< VDM sent by command initiator. */
    CMD_TYPE_RESP_ACK,                  /**< ACK response. */
    CMD_TYPE_RESP_NAK,                  /**< NAK response. */
    CMD_TYPE_RESP_BUSY                  /**< BUSY response. */
} std_vdm_cmd_type_t;

/**
 * @typedef std_vdm_prod_t
 * @brief Enum of the standard VDM product types.
 */
typedef enum
{
    PROD_TYPE_UNDEF = 0,                /**< Undefined device type. */
    PROD_TYPE_HUB,                      /**< Hub device type. */
    PROD_TYPE_PERI,                     /**< Peripheral device type. */
    PROD_TYPE_PAS_CBL,                  /**< Passive Cable. */
    PROD_TYPE_ACT_CBL,                  /**< Active Cable. */
    PROD_TYPE_AMA                       /**< Alternate Mode Accessory. */
} std_vdm_prod_t;

/**
 * @typedef std_vdm_ver_t
 * @brief Enum for the standard VDM version.
 */
typedef enum
{
    STD_VDM_VER1 = 0,                   /**< VDM version 1.0 */
    STD_VDM_VER2,                       /**< VDM version 2.0 */
    STD_VDM_VER3,                       /**< VDM version 3.0 */
    STD_VDM_VER4                        /**< VDM version 4.0 */
} std_vdm_ver_t;

/**
 * @typedef cbl_vbus_cur_t
 * @brief Enum of the cable current levels.
 */
typedef enum
{
    CBL_VBUS_CUR_0A = 0,                /**< Cable does not conduct VBus power through. */
    CBL_VBUS_CUR_3A,                    /**< Cable can support a maximum of 3A. */
    CBL_VBUS_CUR_5A                     /**< Cable can support a maximum of 5A. */
} cbl_vbus_cur_t;

/**
 * @typedef cbl_term_t
 * @brief Enum of the cable termination types.
 */
typedef enum
{
    CBL_TERM_BOTH_PAS_VCONN_NOT_REQ = 0,        /**< Passive cable, VConn not required. */
    CBL_TERM_BOTH_PAS_VCONN_REQ,                /**< Passive cable, VConn required. */
    CBL_TERM_ONE_ACT_ONE_PAS_VCONN_REQ,         /**< One end active, one end passive, VConn required. */
    CBL_TERM_BOTH_ACT_VCONN_REQ                 /**< Both ends of cable are active, VConn required. */
} cbl_term_t;

/**
 * @typedef pe_cbl_state_t
 * @brief Enum of the Policy Engine cable discovery states.
 */
typedef enum
{
    CBL_FSM_DISABLED = 0,                       /**< Cable state machine is inactive. */
    CBL_FSM_ENTRY,                              /**< Cable state machine starting up. */
    CBL_FSM_SEND_SOFT_RESET,                    /**< Cable state machine sending Soft Reset to cable marker. */
    CBL_FSM_SEND_DSC_ID                         /**< Cable state machine waiting for cable response. */
} pe_cbl_state_t;

/**
 * @typedef rp_cc_status_t
 * @brief Enum of the Rp status when Rp is asserted.
 */
typedef enum
{
    RP_RA  = 0,                         /**< CCG has applied Rp. External Ra present.  */
    RP_RD,                              /**< CCG has applied Rp. External Rd present.  */
    RP_OPEN,                            /**< CCG has applied Rp. No external pulldown. */
} rp_cc_status_t;

/**
 * @typedef rd_cc_status_t
 * @brief Enum of the Rd status when Rd is asserted.
 */
typedef enum
{
    RD_RA = 0,                          /**< CCG has applied Rd. No external Rp. */
    RD_USB,                             /**< CCG has applied Rd. Default Rp present. */
    RD_1_5A,                            /**< CCG has applied Rd. 1.5A Rp present. */
    RD_3A,                              /**< CCG has applied Rd. 3A Rp present. */
    RD_ERR                              /**< CCG has applied Rd. Error state. */
} rd_cc_status_t;

/**
 * @typedef rp_term_t
 * @brief Enum of the CC termination current levels.
 */
typedef enum
{
    RP_TERM_RP_CUR_DEF = 0,             /**< Use default Rp. */
    RP_TERM_RP_CUR_1_5A,                /**< Use 1.5 A Rp. */
    RP_TERM_RP_CUR_3A                   /**< Use 3A Rp. */
} rp_term_t;

/**
 * @typedef try_src_snk_t
 * @brief Enum of the Try Source/ Try Sink options.
 */
typedef enum
{
    TRY_SRC_TRY_SNK_DISABLED = 0,       /**< Try.SRC and Try.SNK disabled. */
    TRY_SRC_ENABLED,                    /**< Try.SRC enabled. */
    TRY_SNK_ENABLED                     /**< Try.SNK enabled. */
} try_src_snk_t;

/**
 * @typedef dpm_typec_cmd_t
 * @brief Enum of the DPM (Device Policy Manager) command types that can be initiated through
 * the dpm_typec_command API.
 * @see dpm_typec_command
 */
typedef enum
{
    DPM_CMD_SET_RP_DFLT = 0,            /**< Command to select Default Rp. */
    DPM_CMD_SET_RP_1_5A,                /**< Command to select 1.5 A Rp. */
    DPM_CMD_SET_RP_3A                   /**< Command to select 3 A Rp. */,
    DPM_CMD_PORT_DISABLE,               /**< Command to disable the USB-PD port. */
    DPM_CMD_TYPEC_ERR_RECOVERY,         /**< Command to initiate Type-C error recovery. */
    DPM_CMD_TYPEC_INVALID               /**< Invalid command type. */
} dpm_typec_cmd_t;

/**
 * @typedef dpm_typec_cmd_resp_t
 * @brief Enum of the DPM (Device Policy Manager) response types.
 */
typedef enum
{
    DPM_RESP_FAIL = 0,                  /**< Command failed. */
    DPM_RESP_SUCCESS                    /**< Command succeeded. */
} dpm_typec_cmd_resp_t;

/**
 * @typedef typec_fsm_state_t
 * @brief Enum of the Type-C FSM states. This is for internal stack usage.
 * @warning The ordering of elements must not be altered unless the state table
 * is also updated to match.
 */
typedef enum
{
    TYPEC_FSM_DISABLED = 0,             /**< Type-C state machine is disabled. */
    TYPEC_FSM_ERR_RECOV,                /**< Error Recovery state. */
    TYPEC_FSM_ATTACH_WAIT,              /**< AttachWait.SRC or AttachWait.SNK state. */
#if (!(CCG_SOURCE_ONLY))
    TYPEC_FSM_TRY_SRC,                  /**< Try.SRC state. */
    TYPEC_FSM_TRY_WAIT_SNK,             /**< TryWait.SNK state. */
    TYPEC_FSM_TRY_SNK,                  /**< Try.SNK state. */
    TYPEC_FSM_TRY_WAIT_SRC,             /**< TryWait.SRC state. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    TYPEC_FSM_UNATTACHED_SRC,           /**< Unattached.SRC state. */
#if (!(CCG_SOURCE_ONLY))
    TYPEC_FSM_UNATTACHED_SNK,           /**< Unattached.SNK state. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    TYPEC_FSM_AUD_ACC,                  /**< AudioAccessory state. */
    TYPEC_FSM_DBG_ACC,                  /**< DebugAccessory state. */
    TYPEC_FSM_ATTACHED_SRC,             /**< Attached.SRC state. */
#if (!(CCG_SOURCE_ONLY))
    TYPEC_FSM_ATTACHED_SNK,             /**< Attached.SNK state. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    TYPEC_FSM_MAX_STATES                /**< Invalid Type-C state. */
} typec_fsm_state_t;

/**
 * @enum pe_fsm_state_t
 * @brief Enumeration of Policy Engine states for a USB-PD port. This is for internal stack usage.
 * @warning The ordering of elements must not be altered unless the state table in the stack
 * source is also updated.
 */
typedef enum
{
    PE_FSM_OFF = 0,                             /**< Policy Engine not started. */
    PE_FSM_HR_SEND,                             /**< Send HardReset */
    PE_FSM_HR_SRC_TRANS_DFLT,                   /**< PE_SRC_Transition_to_default */
    PE_FSM_HR_SRC_RECOVER,                      /**< Policy Engine waiting for recovery before enabling VBus. */
    PE_FSM_HR_SRC_VBUS_ON,                      /**< Policy Engine enabling VBus after Hard Reset completion. */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_HR_SNK_TRANS_DFLT,                   /**< PE_SNK_Transition_to_default */
    PE_FSM_HR_SNK_WAIT_VBUS_OFF,                /**< Policy Engine waiting for VBus turning off. */
    PE_FSM_HR_SNK_WAIT_VBUS_ON,                 /**< Policy Engine waiting for VBus to turn back on. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    PE_FSM_BIST_TEST_DATA,                      /**< BIST test data state. */
    PE_FSM_BIST_CM2,                            /**< PE_BIST_Carrier_Mode */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_SNK_STARTUP,                         /**< PE_SNK_Startup */
    PE_FSM_SNK_WAIT_FOR_CAP,                    /**< PE_SNK_Wait_for_Capabilities */
    PE_FSM_SNK_EVAL_CAP,                        /**< PE_SNK_Evaluate_Capability */
    PE_FSM_SNK_SEL_CAP,                         /**< PE_SNK_Select_Capability */
#endif /* (!(CCG_SOURCE_ONLY)) */
    PE_FSM_SRC_STARTUP,                         /**< PE_SRC_Startup */
    PE_FSM_SRC_WAIT_NEW_CAP,                    /**< PE_SRC_Wait_New_Capabilities */
    PE_FSM_SRC_SEND_CBL_SR,                     /**< PE_CBL_Soft_Reset */
    PE_FSM_SRC_SEND_CBL_DSCID,                  /**< PE_CBL_Get_Identity */
    PE_FSM_SRC_SEND_CAP,                        /**< PE_SRC_Send_Capabilities */
    PE_FSM_SRC_DISCOVERY,                       /**< PE_SRC_Discovery */
    PE_FSM_SRC_NEG_CAP,                         /**< PE_SRC_Negotiate_Capability */
    PE_FSM_SRC_TRANS_SUPPLY,                    /**< PE_SRC_Transition_Supply */
    PE_FSM_SRC_SEND_PS_RDY,                     /**< Policy Engine waiting to send PS_RDY. */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_SNK_TRANS,                           /**< PE_SNK_Transition_Sink */
#endif /* (!(CCG_SOURCE_ONLY)) */
    PE_FSM_SR_SEND,                             /**< Policy Engine sending Soft Reset. */
    PE_FSM_SR_RCVD,                             /**< Policy Engine received Soft Reset. */
    PE_FSM_VRS_VCONN_ON,                        /**< Policy Engine waiting for VConn to turn on. */
    PE_FSM_VRS_VCONN_OFF,                       /**< Policy Engine waiting for VConn to turn off. */
    PE_FSM_SWAP_EVAL,                           /**< Evaluate received swap command. */
    PE_FSM_SWAP_SEND,                           /**< Waiting to send swap command. */
    PE_FSM_DRS_CHANGE_ROLE,                     /**< Change data role. */
#if (!(CCG_SOURCE_ONLY))
    PE_FSM_PRS_SRC_SNK_TRANS,                   /**< Source to Sink PR_Swap transition start. */
    PE_FSM_PRS_SRC_SNK_VBUS_OFF,                /**< Initial source waiting for VBus turning off. */
    PE_FSM_PRS_SRC_SNK_WAIT_PS_RDY,             /**< Initial source waiting for PS_RDY. */
    PE_FSM_PRS_SNK_SRC_WAIT_PS_RDY,             /**< Initial sink waiting for PS_RDY. */
    PE_FSM_PRS_SNK_SRC_VBUS_ON,                 /**< Initial sink turning VBus ON. */
    PE_FSM_FRS_CHECK_RP,                        /**< Initial sink checking Rp to send FR_Swap message. */
    PE_FSM_FRS_SRC_SNK_CC_SIGNAL,               /**< Initial source sending FR_Swap signal. */
#endif /* (!(CCG_SOURCE_ONLY)) */
    PE_FSM_READY,                               /**< PE_Ready state. */
    PE_FSM_SEND_MSG,                            /**< Policy Engine sending new AMS. */
    PE_FSM_MAX_STATES                           /**< Invalid Policy Engine state. */
}pe_fsm_state_t;

/**
 * @typedef pd_contract_status_t
 * @brief Enum of possible PD contract negotiation scenarios that are used to
 * signal the application event handler. This status will be reported in byte 0
 * of the event data passed along with the APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE
 * event. Bytes 3:1 of the event data are not used; and bytes 7:4 will report
 * the RDO where applicable.
 */
typedef enum
{
    PD_CONTRACT_NEGOTIATION_SUCCESSFUL      = 0x01,     /**< PD contract negotiation successful. */
    PD_CONTRACT_CAP_MISMATCH_DETECTED       = 0x03,     /**< PD contract negotiated, but capability mismatch
                                                             is present. */
    PD_CONTRACT_REJECT_CONTRACT_VALID       = 0x00,     /**< Contract rejected by CCG, but previous contract
                                                             is still valid. */
    PD_CONTRACT_REJECT_CONTRACT_NOT_VALID   = 0x04,     /**< Contract rejected by CCG and previous contract
                                                             became invalid. */
    PD_CONTRACT_REJECT_NO_CONTRACT          = 0x08,     /**< Contract rejected by CCG and there was no previous
                                                             contract. */
    PD_CONTRACT_REJECT_EXPLICIT_CONTRACT    = 0x0C,     /**< Request rejected by port partner while in previous
                                                             explicit contract. */
    PD_CONTRACT_REJECT_NO_EXPLICIT_CONTRACT = 0x10,     /**< Request rejected by port partner with no previous
                                                             explicit contract. */
    PD_CONTRACT_PS_READY_NOT_RECEIVED       = 0x14,     /**< Failed to receive PS_RDY after Accept. */
    PD_CONTRACT_PS_READY_NOT_SENT           = 0x18      /**< Failed to send PS_RDY after Accept. */
} pd_contract_status_t;

/**
 * @typedef app_evt_t
 * @brief Enum of events that are signalled to the application.
 */
typedef enum
{
    APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS,         /**< 0x00: Unexpected high voltage seen on VBus. */
    APP_EVT_TYPE_C_ERROR_RECOVERY,              /**< 0x01: Type-C error recovery initiated. */
    APP_EVT_CONNECT,                            /**< 0x02: Type-C connect detected. */
    APP_EVT_DISCONNECT,                         /**< 0x03: Type-C disconnect(detach) detected. */
    APP_EVT_EMCA_DETECTED,                      /**< 0x04: Cable (EMCA) discovery successful. */
    APP_EVT_EMCA_NOT_DETECTED,                  /**< 0x05: Cable (EMCA) discovery timed out. */
    APP_EVT_ALT_MODE,                           /**< 0x06: Alternate mode related event. */
    APP_EVT_APP_HW,                             /**< 0x07: MUX control related event. */
    APP_EVT_BB,                                 /**< 0x08: Billboard status change. */
    APP_EVT_RP_CHANGE,                          /**< 0x09: Rp termination change detected. */
    APP_EVT_HARD_RESET_RCVD,                    /**< 0x0A: Hard Reset received. */
    APP_EVT_HARD_RESET_COMPLETE,                /**< 0x0B: Hard Reset processing completed. */
    APP_EVT_PKT_RCVD,                           /**< 0x0C: New PD message received. */
    APP_EVT_PR_SWAP_COMPLETE,                   /**< 0x0D: PR_SWAP process completed. */
    APP_EVT_DR_SWAP_COMPLETE,                   /**< 0x0E: DR_SWAP process completed. */
    APP_EVT_VCONN_SWAP_COMPLETE,                /**< 0x0F: VConn_SWAP process completed. */
    APP_EVT_SENDER_RESPONSE_TIMEOUT,            /**< 0x10: Sender response timeout occurred. */
    APP_EVT_VENDOR_RESPONSE_TIMEOUT,            /**< 0x11: Vendor message response timeout occurred. */
    APP_EVT_HARD_RESET_SENT,                    /**< 0x12: Hard Reset sent by CCG. */
    APP_EVT_SOFT_RESET_SENT,                    /**< 0x13: Soft Reset sent by CCG. */
    APP_EVT_CBL_RESET_SENT,                     /**< 0x14: Cable Reset sent by CCG. */
    APP_EVT_PE_DISABLED,                        /**< 0x15: PE.Disabled state entered. */
    APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE,   /**< 0x16: Contract negotiation completed. */
    APP_EVT_VBUS_OVP_FAULT,                     /**< 0x17: VBus Over Voltage fault detected. */
    APP_EVT_VBUS_OCP_FAULT,                     /**< 0x18: VBus Over Current fault detected. */
    APP_EVT_VCONN_OCP_FAULT,                    /**< 0x19: VConn Over Current fault detected. */
    APP_EVT_VBUS_PORT_DISABLE,                  /**< 0x1A: PD port disable completed. */
    APP_EVT_TYPEC_STARTED,                      /**< 0x1B: PD port enable (start) completed. */
    APP_EVT_FR_SWAP_COMPLETE,                   /**< 0x1C: FR_SWAP process completed. */
    APP_EVT_TEMPERATURE_FAULT,                  /**< 0x1D: Over Temperature fault detected. */
    APP_EVT_HANDLE_EXTENDED_MSG,                /**< 0x1E: Extended message received and needs to be handled. */
    APP_EVT_VBUS_UVP_FAULT,                     /**< 0x1F: VBus Under Voltage fault detected. */
    APP_EVT_VBUS_SCP_FAULT,                     /**< 0x20: VBus Short Circuit fault detected. */
    APP_EVT_TYPEC_ATTACH_WAIT,                  /**< 0x21: Type-C AttachWait state entered. For internal use only. */
    APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED,    /**< 0x22: Type-C transition from AttachWait to Unattached. For
                                                           internal use only. */
    APP_EVT_TYPEC_ATTACH,                       /**< 0x23: Type-C attach event. */
    APP_EVT_CC_OVP,                             /**< 0x24: Over Voltage on CC/VConn line detected. */
    APP_EVT_SBU_OVP,                            /**< 0x25: Over Voltage on SBU1/SBU2 line detected. */
    APP_EVT_ALERT_RECEIVED,                     /**< 0x26: Alert message received. For internal use only. */
    APP_EVT_SRC_CAP_TRIED_WITH_NO_RESPONSE,     /**< 0x27: Src Cap tried with no response. For internal use only. */
    APP_EVT_PD_SINK_DEVICE_CONNECTED            /**< 0x28: Sink device connected. For internal use only. */
} app_evt_t;

typedef enum
{
    PD_AMS_NONE = 0,                            /**< No AMS active. */
    PD_AMS_NON_INTR,                            /**< Non-interruptible AMS is active. */
    PD_AMS_INTR                                 /**< Interruptible AMS is active. */
}pd_ams_type;

/**
 * @typedef vdm_ams_t
 * @brief Enumeration of application responses to policy manager.
 */
typedef enum
{
    VDM_AMS_RESP_READY = 0,                     /**< Response is ready */
    VDM_AMS_RESP_NOT_REQ,                       /**< No response required */
    VDM_AMS_RESP_FROM_EC,                       /**< Response will come from EC */
    VDM_AMS_RESP_NOT_SUPP                       /**< Send a NOT_SUPPORTED response. */
} vdm_ams_t;

/*****************************************************************************
 * Data Structure Definitions
 ****************************************************************************/

/**
 * @brief Union to hold CC status.
 */
typedef union cc_state
{
    uint16_t state;                     /**< Combined status of CC1 and CC2. */
    uint8_t  cc[2];                     /**< Individual status of CC1(cc[0]) and CC2(cc[1]). */
} cc_state_t;

/**
 * @brief Union to hold the PD header defined by the USB-PD specification. Lower 16 bits hold the message
 * header and the upper 16 bits hold the extended message header (where applicable).
 */
typedef union
{
    uint32_t val;                               /**< Header expressed as a 32-bit word. */
    struct PD_HDR
    {
        uint32_t msg_type   : 5;                /**< Bits 04:00 - Message type. */
        uint32_t data_role  : 1;                /**< Bit     05 - Data role. */
        uint32_t spec_rev   : 2;                /**< Bits 07:06 - Spec revision. */
        uint32_t pwr_role   : 1;                /**< Bit     08 - Power role. */
        uint32_t msg_id     : 3;                /**< Bits 11:09 - Message ID. */
        uint32_t len        : 3;                /**< Bits 14:12 - Number of data objects. */
        uint32_t extd       : 1;                /**< Bit     15 - Extended message. */
        uint32_t data_size  : 9;                /**< Bits 24:16 - Extended message size in bytes. */
        uint32_t rsvd1      : 1;                /**< Bit     25 - Reserved. */
        uint32_t request    : 1;                /**< Bit     26 - Chunk request. */
        uint32_t chunk_no   : 4;                /**< Bits 30:27 - Chunk number. */
        uint32_t chunked    : 1;                /**< Bit     31 - Chunked message. */
    } hdr;                                      /**< Header broken down into respective fields. */
} pd_hdr_t;

/**
 * @brief Union to hold the PD extended header.
 */
typedef union
{
    uint16_t val;                               /**< Extended header expressed as 2-byte integer value. */
    struct
    {
        uint16_t data_size  : 9;                /**< Bits 08:00 - Extended message size in bytes. */
        uint16_t rsvd1      : 1;                /**< Bit     09 - Reserved. */
        uint16_t request    : 1;                /**< Bit     10 - Chunk request. */
        uint16_t chunk_no   : 4;                /**< Bits 14:11 - Chunk number. */
        uint16_t chunked    : 1;                /**< Bit     15 - Chunked message. */
    } extd;                                     /**< Extended header broken down into respective fields. */
} pd_extd_hdr_t;

/**
 * @union pd_do_t
 * @brief Union to hold a PD data object. All USB-PD data objects are 4-byte values which are interpreted
 * according to the message type, length and object position. This union represents all possible interpretations
 * of a USB-PD data object.
 */
typedef union
{

    uint32_t val;                                   /**< Data object interpreted as an unsigned integer value. */

    struct BIST_DO
    {
        uint32_t rsvd1                      : 16;   /**< Reserved field. */
        uint32_t rsvd2                      : 12;   /**< Reserved field. */
        uint32_t mode                       : 4;    /**< BIST mode. */
    } bist_do;                                      /**< DO interpreted as a BIST data object. */

    struct FIXED_SRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 100mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t pk_current                 : 2;    /**< Peak current. */
        uint32_t reserved                   : 2;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t dr_swap                    : 1;    /**< Data Role Swap supported. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t ext_powered                : 1;    /**< Externally powered. */
        uint32_t usb_suspend_sup            : 1;    /**< USB suspend supported. */
        uint32_t dual_role_power            : 1;    /**< Dual role power support. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_src;                                    /**< DO interpreted as a Fixed Supply PDO - Source. */

    struct VAR_SRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 10mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_src;                                      /**< DO interpreted as a Variable Supply PDO - Source. */

    struct BAT_SRC
    {
        uint32_t max_power                  : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_src;                                      /**< DO interpreted as a Battery Supply PDO - Source. */

    struct SRC_GEN
    {
        uint32_t max_cur_power              : 10;   /**< Maximum current in 10 mA or power in 250 mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type. */
    } src_gen;                                      /**< Generic source DO. */

    struct FIXED_SNK
    {
        uint32_t op_current                 : 10;   /**< Operational current in 10mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t rsrvd                      : 3;    /**< Reserved field. */
        uint32_t fr_swap                    : 2;    /**< FR swap support. */
        uint32_t dr_swap                    : 1;    /**< Data Role Swap supported. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t ext_powered                : 1;    /**< Externally powered. */
        uint32_t high_cap                   : 1;    /**< Higher capability possible. */
        uint32_t dual_role_power            : 1;    /**< Dual role power support. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_snk;                                    /**< DO interpreted as a Fixed Supply PDO - Sink. */

    struct VAR_SNK
    {
        uint32_t op_current                 : 10;   /**< Operational current in 10mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_snk;                                      /**< DO interpreted as a Variable Supply PDO - Sink. */

    struct BAT_SNK
    {
        uint32_t op_power                   : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_snk;                                      /**< DO interpreted as a Battery Supply PDO - Sink. */

    struct RDO_FIXED_VAR
    {
        uint32_t max_op_current             : 10;   /**< Maximum operating current in 10mA units. */
        uint32_t op_current                 : 10;   /**< Operating current in 10mA units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 0. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_fix_var;                                  /**< Fixed or Variable Request Data Object. */

    struct RDO_FIXED_VAR_GIVEBACK
    {
        uint32_t min_op_current             : 10;   /**< Minimum operating current in 10mA units. */
        uint32_t op_current                 : 10;   /**< Operating current in 10mA units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 1. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_fix_var_gvb;                              /**< Fixed or Variable Request Data Object with GiveBack. */

    struct RDO_BAT
    {
        uint32_t max_op_power               : 10;   /**< Maximum operating power in 250mW units. */
        uint32_t op_power                   : 10;   /**< Operating power in 250mW units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 0. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_bat;                                      /**< Battery Request Data Object. */

    struct RDO_BAT_GIVEBACK
    {
        uint32_t min_op_power               : 10;   /**< Minimum operating power in 250mW units. */
        uint32_t op_power                   : 10;   /**< Operating power in 250mW units. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack flag = 1. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_bat_gvb;                                  /**< Battery Request Data Object with GiveBack. */

    struct RDO_GEN
    {
        uint32_t min_max_power_cur          : 10;   /**< Min/Max power or current requirement. */
        uint32_t op_power_cur               : 10;   /**< Operating power or current requirement. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack supported flag = 0. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_gen;                                      /**< Generic Request Data Object representation. */

    struct RDO_GEN_GVB
    {
        uint32_t max_power_cur              : 10;   /**< Min/Max power or current requirement. */
        uint32_t op_power_cur               : 10;   /**< Operating power or current requirement. */
        uint32_t rsrvd1                     : 3;    /**< Reserved field. */
        uint32_t unchunk_sup                : 1;    /**< Unchunked extended messages supported. */
        uint32_t no_usb_suspend             : 1;    /**< No USB suspend. */
        uint32_t usb_comm_cap               : 1;    /**< USB communication capability. */
        uint32_t cap_mismatch               : 1;    /**< Capability mismatch. */
        uint32_t give_back_flag             : 1;    /**< GiveBack supported flag = 1. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsrvd2                     : 1;    /**< Reserved field. */
    } rdo_gen_gvb;                                  /**< Generic Request Data Object with GiveBack representation. */

    struct STD_VDM_HDR
    {
        uint32_t cmd                        : 5;    /**< VDM command id. */
        uint32_t rsvd1                      : 1;    /**< Reserved field. */
        uint32_t cmd_type                   : 2;    /**< VDM command type. */
        uint32_t obj_pos                    : 3;    /**< Object position. */
        uint32_t rsvd2                      : 2;    /**< Reserved field. */
        uint32_t st_ver                     : 2;    /**< Structured VDM version. */
        uint32_t vdm_type                   : 1;    /**< VDM type = Structured. */
        uint32_t svid                       : 16;   /**< SVID associated with VDM. */
    } std_vdm_hdr;                                  /**< Structured VDM header data object. */

    struct USTD_VDM_HDR
    {
        uint32_t cmd                        : 5;    /**< Command id. */
        uint32_t seq_num                    : 3;    /**< Sequence number. */
        uint32_t rsvd1                      : 3;    /**< Reserved field. */
        uint32_t cmd_type                   : 2;    /**< Command type. */
        uint32_t vdm_ver                    : 2;    /**< VDM version. */
        uint32_t vdm_type                   : 1;    /**< VDM type = Unstructured. */
        uint32_t svid                       : 16;   /**< SVID associated with VDM. */
    } ustd_vdm_hdr;                                 /**< Unstructured VDM header data object as defined by Cypress. */

    struct USTD_QC_4_0_HDR
    {
        uint32_t cmd_0                      : 8;    /**< Command code #0. */
        uint32_t cmd_1                      : 7;    /**< Command code #1. */
        uint32_t vdm_type                   : 1;    /**< VDM type = Unstructured. */
        uint32_t svid                       : 16;   /**< SVID associated with message. */
    } ustd_qc_4_0_hdr;                              /**< Unstructured VDM header data object as defined by QC 4.0 spec. */

    struct QC_4_0_DATA_VDO
    {
        uint32_t data_0                     : 8;    /**< Command data #0. */
        uint32_t data_1                     : 8;    /**< Command data #1. */
        uint32_t data_2                     : 8;    /**< Command data #2. */
        uint32_t data_3                     : 8;    /**< Command data #3. */
    } qc_4_0_data_vdo;                              /**< Unstructured VDM data object as defined by QC 4.0 spec. */

    struct STD_VDM_ID_HDR
    {
        uint32_t usb_vid                    : 16;   /**< 16-bit vendor ID. */
#if CCG_PD_REV3_ENABLE
        uint32_t rsvd1                      : 7;    /**< Reserved field. */
        uint32_t prod_type_dfp              : 3;    /**< Product type as DFP. */
#else
        uint32_t rsvd1                      : 10;   /** Reserved field. */
#endif /* CCG_PD_REV3_ENABLE */
        uint32_t mod_support                : 1;    /**< Whether alternate modes are supported. */
        uint32_t prod_type                  : 3;    /**< Product type as UFP. */
        uint32_t usb_dev                    : 1;    /**< USB device supported. */
        uint32_t usb_host                   : 1;    /**< USB host supported. */
    } std_id_hdr;                                   /**< Standard ID_HEADER VDO. */

    struct STD_CERT_VDO
    {
        uint32_t usb_xid                    : 32;   /**< 32-bit XID value. */
    } std_cert_vdo;                                 /**< Cert Stat VDO structure. */

    struct STD_PROD_VDO
    {
        uint32_t bcd_dev                    : 16;   /**< 16-bit bcdDevice value. */
        uint32_t usb_pid                    : 16;   /**< 16-bit product ID. */
    } std_prod_vdo;                                 /**< Product VDO structure. */

    struct STD_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t sop_dp                     : 1;    /**< Whether SOP'' controller is present. */
        uint32_t vbus_thru_cbl              : 1;    /**< Whether cable allows VBus power through. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t ssrx2                      : 1;    /**< Whether SSRX2 has configurable direction. */
        uint32_t ssrx1                      : 1;    /**< Whether SSRX1 has configurable direction. */
        uint32_t sstx2                      : 1;    /**< Whether SSTX2 has configurable direction. */
        uint32_t sstx1                      : 1;    /**< Whether SSTX1 has configurable direction. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t typec_plug                 : 1;    /**< Whether cable has a plug: Should be 0. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd1                      : 4;    /**< Reserved field. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } std_cbl_vdo;                                  /**< Cable VDO structure as defined in USB-PD r2.0. */

    struct PAS_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t rsvd2                      : 2;    /**< Reserved field. */
        uint32_t max_vbus_volt              : 2;    /**< Max. VBus voltage supported. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t typec_plug                 : 1;    /**< Reserved field. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd3                      : 1;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } pas_cbl_vdo;                                  /**< Passive cable VDO structure as defined by PD 3.0. */

    struct ACT_CBL_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported by the cable. */
        uint32_t sop_dp                     : 1;    /**< Whether SOP'' controller is present. */
        uint32_t vbus_thru_cbl              : 1;    /**< Whether cable conducts VBus through. */
        uint32_t vbus_cur                   : 2;    /**< VBus current supported by the cable. */
        uint32_t rsvd1                      : 2;    /**< Reserved field. */
        uint32_t max_vbus_volt              : 2;    /**< Max. VBus voltage supported. */
        uint32_t cbl_term                   : 2;    /**< Cable termination and VConn power requirement. */
        uint32_t cbl_latency                : 4;    /**< Cable latency. */
        uint32_t typec_plug                 : 1;    /**< Reserved field. */
        uint32_t typec_abc                  : 2;    /**< Cable plug type. */
        uint32_t rsvd2                      : 1;    /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t cbl_fw_ver                 : 4;    /**< Cable firmware version. */
        uint32_t cbl_hw_ver                 : 4;    /**< Cable hardware version. */
    } act_cbl_vdo;                                  /**< Active cable VDO structure as defined by PD 3.0. */

    struct STD_AMA_VDO
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported. */
        uint32_t vbus_req                   : 1;    /**< Whether device requires VBus. */
        uint32_t vcon_req                   : 1;    /**< Whether device requires VConn. */
        uint32_t vcon_pwr                   : 3;    /**< VConn power required. */
        uint32_t ssrx2                      : 1;    /**< Whether SSRX2 has configurable direction. */
        uint32_t ssrx1                      : 1;    /**< Whether SSRX1 has configurable direction. */
        uint32_t sstx2                      : 1;    /**< Whether SSTX2 has configurable direction. */
        uint32_t sstx1                      : 1;    /**< Whether SSTX1 has configurable direction. */
        uint32_t rsvd1                      : 12;   /**< Reserved field. */
        uint32_t ama_fw_ver                 : 4;    /**< AMA firmware version. */
        uint32_t ama_hw_ver                 : 4;    /**< AMA hardware version. */
    } std_ama_vdo;                                  /**< AMA VDO structure as defined by PD 2.0. */

    struct STD_AMA_VDO_PD3
    {
        uint32_t usb_ss_sup                 : 3;    /**< USB signalling supported. */
        uint32_t vbus_req                   : 1;    /**< Whether device requires VBus. */
        uint32_t vcon_req                   : 1;    /**< Whether device requires VConn. */
        uint32_t vcon_pwr                   : 3;    /**< VConn power required. */
        uint32_t rsvd1                      : 13;   /**< Reserved field. */
        uint32_t vdo_version                : 3;    /**< VDO version. */
        uint32_t ama_fw_ver                 : 4;    /**< AMA firmware version. */
        uint32_t ama_hw_ver                 : 4;    /**< AMA hardware version. */
    } std_ama_vdo_pd3;                              /**< AMA VDO structure as defined by PD 3.0. */

    struct STD_SVID_RESP_VDO
    {
        uint32_t svid_n1                    : 16;   /**< SVID #1 */
        uint32_t svid_n                     : 16;   /**< SVID #2 */
    } std_svid_res;                                 /**< Discover_SVID response structure. */

    struct STD_DP_VDO
    {
        uint32_t port_cap                   : 2;    /**< Port capability. */
        uint32_t signal                     : 4;    /**< Signalling supported. */
        uint32_t recep                      : 1;    /**< Whether Type-C connector is plug or receptacle. */
        uint32_t usb2_0                     : 1;    /**< USB 2.0 signalling required. */
        uint32_t dfp_d_pin                  : 8;    /**< DFP_D pin assignments supported. */
        uint32_t ufp_d_pin                  : 8;    /**< UFP_D pin assignments supported. */
        uint32_t rsvd                       : 8;    /**< Reserved field. */
    } std_dp_vdo;                                   /**< DisplayPort Mode VDO as defined by VESA spec. */

    struct DP_STATUS_VDO
    {
        uint32_t dfp_ufp_conn               : 2;    /**< Whether DFP_D/UFP_D is connected. */
        uint32_t pwr_low                    : 1;    /**< Low power mode. */
        uint32_t en                         : 1;    /**< DP functionality enabled. */
        uint32_t mult_fun                   : 1;    /**< Multi-function mode preferred. */
        uint32_t usb_cfg                    : 1;    /**< Request switch to USB configuration. */
        uint32_t exit                       : 1;    /**< Exit DP mode request. */
        uint32_t hpd_state                  : 1;    /**< Current HPD state. */
        uint32_t hpd_irq                    : 1;    /**< HPD IRQ status. */
        uint32_t rsvd                       : 23;   /**< Reserved field. */
    } dp_stat_vdo;                                  /**< DisplayPort status update VDO as defined by VESA spec. */

    struct DP_CONFIG_VDO
    {
        uint32_t sel_conf                   : 2;    /**< Select configuration. */
        uint32_t sign                       : 4;    /**< Signalling for DP protocol. */
        uint32_t rsvd1                      : 2;    /**< Reserved. */
        uint32_t dfp_asgmnt                 : 8;    /**< DFP_D pin assignment. */
        uint32_t ufp_asgmnt                 : 8;    /**< UFP_D pin assignment. */
        uint32_t rsvd2                      : 8;    /**< Reserved field. */
    } dp_cfg_vdo;                                   /**< DisplayPort configure VDO as defined by VESA spec. */

    /** @cond DOXYGEN_HIDE */
    struct PPS_SRC
    {
        uint32_t max_cur                    : 7;
        uint32_t rsvd1                      : 1;
        uint32_t min_volt                   : 8;
        uint32_t rsvd2                      : 1;
        uint32_t max_volt                   : 8;
        uint32_t rsvd3                      : 2;
        uint32_t pps_pwr_limited            : 1;
        uint32_t apdo_type                  : 2;
        uint32_t supply_type                : 2;
    } pps_src;

    struct PPS_SNK
    {
        uint32_t op_cur                     : 7;
        uint32_t rsvd1                      : 1;
        uint32_t min_volt                   : 8;
        uint32_t rsvd2                      : 1;
        uint32_t max_volt                   : 8;
        uint32_t rsvd3                      : 1;
        uint32_t cur_fb                     : 1;
        uint32_t rsvd4                      : 1;
        uint32_t apdo_type                  : 2;
        uint32_t supply_type                : 2;
    } pps_snk;

    struct RDO_PPS
    {
        uint32_t op_cur                     : 7;
        uint32_t rsvd1                      : 2;
        uint32_t out_volt                   : 11;
        uint32_t rsvd2                      : 3;
        uint32_t unchunk_sup                : 1;
        uint32_t no_usb_suspend             : 1;
        uint32_t usb_comm_cap               : 1;
        uint32_t cap_mismatch               : 1;
        uint32_t rsvd3                      : 1;
        uint32_t obj_pos                    : 3;
        uint32_t rsrvd2                     : 1;
    } rdo_pps;

    struct ADO_ALERT
    {
        uint32_t rsvd1                      :16;
        uint32_t hot_swap_bats              :4;
        uint32_t fixed_bats                 :4;
        uint32_t rsvd2                      :1;
        uint32_t bat_status_change          :1;
        uint32_t ocp                        :1;
        uint32_t otp                        :1;
        uint32_t op_cond_change             :1;
        uint32_t src_input_change           :1;
        uint32_t ovp                        :1;
    }ado_alert;

    struct TBT_VDO
    {
        uint32_t intel_mode                 : 16;
        uint32_t cbl_speed                  : 3;
        uint32_t cbl_gen                    : 2;
        uint32_t cbl_type                   : 1;
        uint32_t cbl                        : 1;
        uint32_t link_training              : 1;
        uint32_t leg_adpt                   : 1;
        uint32_t rsvd                       : 7;
    } tbt_vdo;

    struct SLICE_VDO
    {
        uint32_t slice_mode                 : 16;
        uint32_t module_type                : 2;
        uint32_t rsvd                       : 14;
    } slice_vdo;

    struct SLICE_SUBHDR
    {
        uint32_t am_addr                    : 20;
        uint32_t vdo_cnt                    : 3;
        uint32_t multi_part                 : 1;
        uint32_t data_cnt                   : 8;
    } slice_subhdr;
    /** @endcond */

} pd_do_t;

/**
 * @brief PD port status corresponding to the Status Data Block (SSDB)
 * See Table 6-39 of USB-PD R3 specification.
 */
typedef struct
{
    uint8_t  intl_temperature;                  /**< Port's internal temperature. 0 if not supported. */
    uint8_t  present_input;                     /**< Reports current input power status. */
    uint8_t  battery_input;                     /**< Reports the current battery status. */
    uint8_t  event_flags;                       /**< Event flags. */
    uint8_t  temp_status;                       /**< Temperature status. */
    uint8_t  dummy[3];                          /**< Reserved field used for 4 byte alignment. */
} pd_power_status_t;

/**
 * @union pd_port_status_t
 * @brief PD port status as reported to Embedded Controller.
 */
typedef union
{
    uint32_t    val;                                            /**< PD-Status register value. */

    struct PD_PORT_STAT
    {
        uint32_t dflt_data_role         : 2;                    /**< Default data role. */
        uint32_t dflt_data_pref         : 1;                    /**< Preferred data role in case of DRP. */
        uint32_t dflt_power_role        : 2;                    /**< Default power role. */
        uint32_t dflt_power_pref        : 1;                    /**< Preferred power role in case of DRP. */
        uint32_t cur_data_role          : 1;                    /**< Current data role. */
        uint32_t reserved0              : 1;                    /**< Reserved. */
        uint32_t cur_power_role         : 1;                    /**< Current power role. */
        uint32_t min_state              : 1;                    /**< Whether in Min state (due to GotoMin). */
        uint32_t contract_exist         : 1;                    /**< Whether explicit contract exists. */
        uint32_t emca_present           : 1;                    /**< EMCA detected or not. */
        uint32_t vconn_src              : 1;                    /**< Whether CCG is VConn source. */
        uint32_t vconn_on               : 1;                    /**< Whether VConn is actually ON. */
        uint32_t rp_status              : 1;                    /**< Current Rp status. */
        uint32_t pe_rdy                 : 1;                    /**< Whether Policy Engine is in Ready state. */
        uint32_t ccg_spec_rev           : 2;                    /**< USB-PD revision supported by CCG firmware. */
        uint32_t peer_pd3_supp          : 1;                    /**< Whether port partner supports PD 3.0. */
        uint32_t peer_unchunk_supp      : 1;                    /**< Whether port partner supports unchunked messages. */
        uint32_t emca_spec_rev          : 2;                    /**< USB-PD revision supported by cable marker. */
        uint32_t reserved2              : 10;                   /**< Reserved field. */
    } status;                                                   /**< Structure containing status bits. */
} pd_port_status_t;

/**
 *  @brief PD port-specific configuration data from the configuration table.
 */
typedef struct
{
    uint16_t    id_vdm_offset;          /**< Byte 0x00: Two byte offset to the Discover ID Response VDM. */
    uint16_t    id_vdm_length;          /**< Byte 0x02: Two byte length of the Discover ID Response VDM. */
    uint16_t    svid_vdm_offset;        /**< Byte 0x04: Two byte offset to the Discover SVID Response VDM. */
    uint16_t    svid_vdm_length;        /**< Byte 0x06: Two byte length of the Discover SVID Response VDM. */
    uint16_t    mode_vdm_offset;        /**< Byte 0x08: Two byte offset to the Discover Mode Response VDM. */
    uint16_t    mode_vdm_length;        /**< Byte 0x0A: Two byte length of the Discover Mode Response VDM. */
    uint16_t    ext_src_cap_offset;     /**< Byte 0x0C: Two byte offset to the Src. Cap Extended response. */
    uint16_t    ext_src_cap_length;     /**< Byte 0x0E: Two byte length of the Src. Cap Extended response. */
    uint16_t    reserved_0[4];          /**< Byte 0x10: Reserved area for additional VDMs. */
    uint16_t    ra_timeout;             /**< Byte 0x18: Ra removal delay for EMCA applications, measured in ms. */
    uint16_t    reserved_1[3];          /**< Byte 0x1A: Reserved area for EMCA configuration. */
    uint8_t     port_role;              /**< Byte 0x20: PD port role: 0=Sink, 1=Source, 2=Dual Role. */
    uint8_t     default_role;           /**< Byte 0x21: Default port role in case of a dual role port: 0=Sink, 1=Source. */
    uint8_t     current_level;          /**< Byte 0x22: Type-C current level: 0=Default, 1=1.5A, 2=3A. */
    uint8_t     is_src_bat;             /**< Byte 0x23: Whether the power source is connected to a battery. */
    uint8_t     is_snk_bat;             /**< Byte 0x24: Whether the power sink is connected to a battery. */
    uint8_t     snk_usb_susp_en;        /**< Byte 0x25: Whether USB suspend is supported. */
    uint8_t     snk_usb_comm_en;        /**< Byte 0x26: Whether USB communication is supported. */
    uint8_t volatile swap_response;     /**< Byte 0x27: Response to be sent for each USB-PD SWAP command:
                                         *  Bits 1:0 => DR_SWAP response
                                         *  Bits 3:2 => PR_SWAP response
                                         *  Bits 5:4 => VCONN_SWAP response
                                         * Allowed values are: 0=ACCEPT, 1=REJECT, 2=WAIT.
                                        */
    uint8_t     drp_toggle_en;          /**< Byte 0x28: Whether DRP toggle is enabled. */
    uint8_t     src_pdo_cnt;            /**< Byte 0x29: Number of valid source PDOs in the table: Maximum supported value is 7. */
    uint8_t     default_src_pdo_mask;   /**< Byte 0x2A: Default Source PDO enable mask. */
    uint8_t     snk_pdo_cnt;            /**< Byte 0x2B: Number of valid sink PDOs in the table: Maximum supported value is 7. */
    uint8_t     default_sink_pdo_mask;  /**< Byte 0x2C: Default Sink PDO enable mask. */
    uint8_t     rp_supported;           /**< Byte 0x2D: Supported Rp values. Multiple bits can be set.
                                         *   Bit 0 => Default current support.
                                         *   Bit 1 => 1.5A support.
                                         *   Bit 2 => 3A support.
                                         */
    uint8_t     pd_operation_en;        /**< Byte 0x2E: Whether PD operation is supported on the port. */
    uint8_t     try_src_en;             /**< Byte 0x2F: Whether Try.SRC state is supported on the port. */
    uint8_t     cable_disc_en;          /**< Byte 0x30: Whether cable discovery is supported on the port. */
    uint8_t     dead_bat_support;       /**< Byte 0x31: Whether firmware should force Sink operation in Dead Battery condition. */
    uint8_t     err_recovery_en;        /**< Byte 0x32: Whether PD error recovery is enabled. */
    uint8_t volatile port_disable;      /**< Byte 0x33: Port disable flag. */
    uint8_t volatile frs_enable;        /**< Byte 0x34: Fast Role Swap enable flags. */
    uint8_t volatile vconn_retain;      /**< Byte 0x35: Whether VConn should be retained in ON state. */
    uint16_t    reserved_3[5];          /**< Byte 0x36: Reserved words for padding to 4-byte aligned address. */

    uint32_t    src_pdo_list[MAX_NO_OF_PDO];    /**< Byte 0x40: Source PDO data array. */
    uint32_t    snk_pdo_list[MAX_NO_OF_PDO];    /**< Byte 0x5C: Sink PDO data array. */
    uint16_t    snk_pdo_max_min_current_pwr[MAX_NO_OF_PDO]; /**< Byte 0x78: Array of sink PDO parameters.
                                                             *   For each element, the format is as below:
                                                             *     Bit  15   => Give Back support flag
                                                             *     Bits 14:0 => Minimum sink operating current.
                                                             */
    uint16_t    reserved_4;             /**< Byte 0x86: Reserved space for additional port parameters. */
    uint8_t volatile protection_enable; /**< Byte 0x88: Enable field for protection settings
                                         *   Bit0: ovp enable
                                         *   Bit1: ocp enable
                                         *   Bit2: uvp enable
                                         *   Bit3: scp enable
                                         *   Bit4: vconn ocp enable
                                         *   Bit5: otp enable
                                         *   Bit(6:7): Reserved for future use
                                         */
    uint8_t     reserved_8;             /**< Byte 0x89: Reserved for future use. */
    uint16_t    ovp_tbl_offset;         /**< Byte 0x8A: Offset to VBus OVP settings. */
    uint16_t    ocp_tbl_offset;         /**< Byte 0x8C: Offset to VBus OCP settings. */
    uint16_t    uvp_tbl_offset;         /**< Byte 0x8E: Offset to VBus UVP settings. */
    uint16_t    scp_tbl_offset;         /**< Byte 0x90: Offset to VBus SCP settings. */
    uint16_t    vconn_ocp_tbl_offset;   /**< Byte 0x92: Offset to Vcon OCP settings. */
    uint16_t    otp_tbl_offset;         /**< Byte 0x94: Offset to OTP settings. */
    uint16_t    pwr_tbl_offset;         /**< Byte 0x96: Offset to power parameters. */
    uint16_t    chg_cfg_tbl_offset;     /**< Byte 0x98: Offset to legacy charging parameters. */
    uint16_t    bat_chg_tbl_offset;     /**< Byte 0x9A: Offset to battery charging parameters. */
    uint8_t     reserved_9[4];          /**< Byte 0x9C: Reserved. */
    uint8_t volatile dp_config_supported;   /**< Byte 0xA0: Supported Pin configurations for DP modes
                                             *   0b00000000: USB SS only.
                                             *   0b00000001: Reserved for future use (A).
                                             *   0b00000010: Reserved for future use (B).
                                             *   0b00000100: Pin Config C.
                                             *   0b00001000: Pin Config D.
                                             *   0b00010000: Pin Config E.
                                             *   0b00100000: Pin Config F.
                                             */
    uint8_t volatile dp_mux_control;    /**< Byte 0xA1: DP_MUX_CONTROL method:
                                         *   0 => DP MUX Controlled by CCG.
                                         *   1 => DP MUX Controlled by EC.
                                         */
    uint8_t volatile dp_mode_trigger;   /**< Byte 0xA2: DP_MODE_TRIGGER: Trigger for initializing DP modes.
                                         *   0 => CCG initiates DP after contract.
                                         *   1 => CCG waits for a trigger from EC.
                                         */
    uint8_t     dp_oper;                /**< Byte 0xA3: Type of DP operation supported.
                                         *   Bit 0: DP Sink supported
                                         *   Bit 1: DP Source supported.
                                         */
    uint8_t volatile dp_pref_mode;      /**< Byte 0xA4: DP preferred mode.
                                         *   Bit 0:
                                         *          0: CCG as DP Sink prefers 4 lane DP mode only.
                                         *          1: CCG as DP Sink prefers 2 lane DP + USB SS Mode.
                                         *  All other bits are reserved.
                                         */
    uint8_t     reserved_6[3];          /**< Byte 0xA5: Reserved area for future expansion. */
    uint16_t    bb_tbl_offset;          /**< Byte 0xA8: Two byte offset to Billboard settings */
    uint8_t volatile type_a_enable;     /**< Byte 0xAA: Type-A port support. */
    uint8_t     reserved_10;            /**< Byte 0xAB: Reserved. */
    uint16_t    type_a_pwr_tbl_offset;  /**< Byte 0xAC: Offset to power parameters of Type-A port. */
    uint16_t    type_a_chg_tbl_offset;  /**< Byte 0xAE: Offset to battery charging parameters of Type-A port. */
    uint16_t    dock_cfg_tbl_offset;    /**< Byte 0xB0: Offset to dock configuration parameter table. */
    uint16_t    spm_cfg_tbl_offset;     /**< Byte 0xB2: Offset to Source Policy Manager parameter table. */
    uint8_t     reserved_11[60];        /**< Byte 0xB4: Reserved area for future expansion. */
} pd_port_config_t;

/**
 * @brief Struct to hold the OVP settings.
 */
typedef struct
{
    uint8_t table_len;      /**< Table length in bytes */
    uint8_t threshold;      /**< OVP threshold: Excess voltage above expected value in percentage of expected voltage */
    uint8_t debounce;       /**< OVP debounce duration in micro-seconds.
                             *   If a non-zero debounce is specified,
                             *   there can be an error of upto 35 us due to the device being in sleep mode.
                             */
    uint8_t retry_cnt;      /**< Number of consecutive OVP events allowed before the port operation is
                                 suspended by CCG firmware. */

} ovp_settings_t;

/**
 * @brief Struct to hold the OCP settings.
 */
typedef struct
{
    uint8_t table_len;      /**< Table length in bytes */
    uint8_t threshold;      /**< OCP threshold: Excess current in percentage of maximum allowed current. */
    uint8_t debounce;       /**< OCP debounce period in milliseconds. OCP handling is only performed if
                             *   the OCP condition persists for this duration. */
    uint8_t retry_cnt;      /**< Number of consecutive OCP events allowed before the port is suspended by CCG firmware. */
    uint8_t threshold2;     /**< Secondary OCP threshold (corresponding to peak current condition). Set to 0 if not used. */
    uint8_t debounce2;      /**< Debounce period in ms corresponding to secondary threshold. */
    uint8_t sense_res;      /**< Sense Resistor impedance in milli-Ohm units. */
    uint8_t cs_res;         /**< Current sense tuning resistor impedance in 100 Ohm units. */
} ocp_settings_t;

/**
 * @brief Struct to hold the UVP settings.
 */
typedef struct
{
    uint8_t table_len;      /**< Table length in bytes */
    uint8_t threshold;      /**< UVP threshold: Reduced voltage below expected value in percentage of
                                 expected voltage */
    uint8_t debounce;       /**< UVP debounce duration in micro-seconds.
                             *   If a non-zero debounce is specified,
                             *   there can be an error of upto 35 us due to the device being in sleep mode. */
    uint8_t retry_cnt;      /**< Number of consecutive UVP events allowed before the port operation is
                                 suspended by CCG firmware. */
} uvp_settings_t;

/**
 * @brief Struct to hold the SCP settings.
 */
typedef struct
{
    uint8_t table_len;      /**< Table length in bytes */
    uint8_t threshold;      /**< SCP threshold: Reduced voltage below expected value in percentage of
                                 expected voltage */
    uint8_t debounce;       /**< SCP debounce duration in micro-seconds.
                             *   If a non-zero debounce is specified,
                             *   there can be an error of upto 35 us due to the device being in sleep mode. */
    uint8_t retry_cnt;      /**< Number of consecutive SCP events allowed before the port operation is suspended. */
} scp_settings_t;

/**
 * @brief Struct to hold the Vconn OCP settings.
 */
typedef struct
{
    uint8_t table_len;          /**< Table length in bytes */
    uint8_t threshold;          /**< Max. Vconn current allowed in 10 mA units. */
    uint8_t debounce;           /**< Vconn OCP debounce period in ms. */
    uint8_t reserved_0;         /**< Reserved for future use*/
} vconn_ocp_settings_t;

/**
 * @brief Struct to hold the OTP settings.
 */
typedef struct
{
    uint8_t table_len;          /**< Table length in bytes */
    uint8_t therm_type;         /**< Type of thermistor used for temperature sensing:
                                 *   0 = Negative Temperature Coefficient (NTC)
                                 *   1 = Positive Temperature Coefficien
                                 */
    uint16_t cutoff_volt;       /**< Voltage corresponding to the temperature at which OTP cutoff is to be
                                     performed (in mV) */
    uint16_t restart_volt;      /**< Voltage corresponding to the temperature at which system operation
                                     can be resumed (in mV) */
    uint16_t debounce;          /**< OTP debounce duration in ms. */
    uint8_t reserved_0[4];      /**< Reserved for future use */
} otp_settings_t;

/**
 * @brief Struct to hold the port power parameters.
 */
typedef struct
{
    uint8_t table_len;          /**< Power parameters table length in bytes */
    uint8_t fb_type;            /**< Type of power feedback:
                                 *   Bit 0 --> No feedback
                                 *   Bit 1 --> PWM
                                 *   Bit 2 --> Direct feedback
                                 *   Bit 3 --> Opto-isolator based feedback
                                 */
    uint16_t vbus_min_volt;     /**< VBus minimum voltage in mV */
    uint16_t vbus_max_volt;     /**< VBus maximum voltage in mV */
    uint16_t vbus_dflt_volt;    /**< Default VBus supply voltage when feedback control is tri-stated. */
    uint32_t fb_ctrl_r1;        /**< Feedback control circuit: R1 resistance in Ohms */
    uint32_t fb_ctrl_r2;        /**< Feedback control circuit: R2 resistance in Ohms */
} pwr_params_t, typeA_pwr_params_t;

/**
 * @brief Struct to hold the port legacy charging parameters.
 */
typedef struct
{
    uint8_t table_len;          /**< Legacy charging parameter table length in bytes */
    uint8_t src_sel;            /**< Legacy charging source selection bit mask:
                                 *   Bit 0 --> BC 1.2 support
                                 *   Bit 1 --> Apple charger
                                 *   Bit 2 --> Quick charge
                                 *   Bit 3 --> AFC charger
                                 *   Other bits reserved
                                 */
    uint8_t snk_sel;            /**< Legacy charging sink selection bit mask:
                                 *   Bit 0 --> BC 1.2
                                 *   Bit 1 --> Apple brick
                                 *   Other bits reserved
                                 */
    uint8_t reserved_0;         /**< Reserved for future use */
    uint8_t apple_src_id;       /**< Apple charger brick ID to be used as source:
                                 *   Bit 0 --> 1 A
                                 *   Bit 1 --> 2.1 A
                                 *   Bit 2 --> 2.4 A
                                 */
    uint8_t qc_src_type;        /**< Quick charge source type:
                                 *   Bit 0 --> QC 2.0 Class-A
                                 *   Bit 1 --> QC 2.0 Class-B
                                 *   Bit 2 --> QC 3.0 Class-A
                                 *   Bit 3 --> QC 3.0 Class-B
                                 */
    uint8_t reserved_1;         /**< Reserved for future use */
    uint8_t afc_src_cap_cnt;    /**< Number of AFC source voltage-current capabilities supported. */
    uint8_t afc_src_caps[16];   /**< AFC source voltage-current capabilities list. */
} chg_cfg_params_t, typeA_chg_cfg_params_t;

/**
 * @brief Struct to hold the battery charging parameters.
 */
typedef struct
{
    uint8_t table_len;              /**< Battery charger parameter table length in bytes*/
    uint8_t reserved_0;             /**< Reserved for future use*/
    uint16_t vbatt_max_volt;        /**< Maximum battery voltage in mV. */
    uint16_t vbatt_cutoff_volt;     /**< Minimum battery voltage below which device should stop discharging. */
    uint16_t vbatt_dischg_en_volt;  /**< Battery voltage at which discharge can be re-enabled. */
    uint16_t vbatt_max_cur;         /**< Maximum charging current allowed for the battery in mA. */
    uint16_t reserved_1;            /**< Reserved for future use*/
} bat_chg_params_t;

/**
 * @brief Struct to hold the Billboard settings.
 */
typedef struct
{
    uint8_t table_len;              /**< Byte 0x00: Billboard parameter table length in bytes. */
    uint8_t volatile bb_enable;     /**< Byte 0x01: This field indicates whether a billboard device is enabled.
                                     *   The usage and requirement for the billboard device is application
                                     *   specific.
                                     *    0 => No billboard device.
                                     *    1 => External billboard device.
                                     *    2 => Internal billboard.
                                     */
    uint8_t volatile bb_always_on;  /**< Byte 0x02: This field is valid only if the bb_enable is non-zero. The field
                                     *   determines when to enable the billboard interface.
                                     *    0 => Enable the billboard device only on error.
                                     *    1 => Always enable billboard on connection.
                                     */
    uint8_t volatile bb_option;     /**< Byte 0x03: This field provides the various functionalities supported by
                                     *   the device.
                                     *    Bit 0    => 0 = No HID interface, 1 = Enable HID interface.
                                     *    Bits 1:7 => Reserved.
                                     */
    uint16_t    bb_timeout;         /**< Byte 0x04: This field is valid only if bb_enable is non-zero. The field
                                     *   determines how long the billboard interface stays on, in seconds.
                                     *   FFFFh    => Stays on until disconnect.
                                     *   000Ah to FFFEh => Timeout in seconds.
                                     */

    uint8_t volatile bb_ec_present; /**< Byte 0x06: EC present notification to external billboard controller. */
    uint8_t volatile bb_bus_power;  /**< Byte 0x07: This field is valid only for devices that have internal USB
                                     *   support. The field indicates whether the device is bus powered or not.
                                     */
    uint8_t volatile bb_unique_container_id;   /**< Byte 0x08: This field is valid only for devices that have
                                                    internal USB support. The field indicates whether the device
                                                    creates the container ID descriptor or uses what is provided
                                                    in the BOS descriptor.
                                                */
    uint8_t    reserved_1;                      /**< Byte 0x09: Reserved area for future expansion. */
    uint16_t volatile bb_bos_dscr_offset;       /**< Byte 0x0A: This field is valid only for devices that have
                                                     internal USB support. The field provides the offset inside
                                                     the table where the BOS descriptor for the device is located.
                                                     The BOS descriptor is mandatory for billboard support.
                                                 */
    uint16_t    bb_string_dscr_offset[15];      /**< Byte 0x0C: This field is valid only for devices that have
                                                     internal USB support. The field provides the offset inside
                                                     the table where the various string descriptors for the device
                                                     are located. The descriptors are expected to be in a fixed order
                                                     and is mandatory. The following are the usage models for the
                                                     various string indices:
                                                 *     0   => Manufacturer string (mandatory).
                                                 *     1   => Product string (mandatory).
                                                 *     2   => Serial string (optional).
                                                 *            0000h = No serial string descriptor,
                                                 *            FFFFh = Unique serial string generated by device,
                                                 *            Any other offset is treated as a valid serial string.
                                                 *     3   => Configuration string (mandatory).
                                                 *     4   => Billboard interface string (mandatory).
                                                 *     5   => HID interface string (mandatory).
                                                 *     6   => Additional info URL string (mandatory).
                                                 *     7-8 => Alternate mode strings (mandatory for all valid modes).
                                                 */
    uint16_t    bb_vid;                         /**< Byte 0x2A: VID for the Billboard device. */
    uint16_t    bb_pid;                         /**< Byte 0x2C: PID for the Billboard device. */
} bb_settings_t;

/**
 * @brief Struct to hold the PD device configuration.
 */
typedef struct
{
    uint16_t    table_sign;     /**< Byte 0x00: Two byte signature to indicate validity of the configuration. */
    uint8_t     table_type;     /**< Byte 0x02: The table type indicates the type of solution.
                                 *   1 => EMCA
                                 *   2 => DFP
                                 *   3 => UFP
                                 *   4 => DRP
                                 */
    uint8_t     application;    /**< Byte 0x03: This field specifies the type of PD application supported:
                                 *   0 => Notebook
                                 *   1 => Tablet
                                 *   2 => Passive Cable
                                 *   3 => Active Cable
                                 *   4 => Monitor
                                 *   5 => Power Adapter
                                 *   6 => Cable Adapter (Dongle)
                                 */
    uint16_t    table_version;  /**< Byte 0x04: Table version: This contains 4 bit major version, 4 bit minor
                                 *   version and 8 bit patch number.
                                 */
    uint16_t    table_size;     /**< Byte 0x06: Size of the configuration table. Checksum is calculated over
                                 *   bytes 10 to size - 1.
                                 */
    uint8_t     table_checksum; /**< Byte 0x08: One byte configuration checksum. Calculated over bytes 10 to
                                 *   byte (size - 1).
                                 */
    uint8_t     flash_checksum; /**< Byte 0x09: One byte flash checksum. Used to ensure that binary sum of
                                 *   config table is 0.
                                 */
    uint16_t    flashing_vid;   /**< Byte 0x0A: Special VID used for flashing mode in case of CC firmware
                                 *   updates.
                                 */
    uint16_t    flashing_mode;  /**< Byte 0x0C: Special Mode used for flashing in case of CC firmware updates. */
    uint8_t     pd_port_cnt;    /**< Byte 0x0E: Number of PD ports enabled in the configuration. */
    uint8_t     reserved_0;     /**< Byte 0x0F: Reserved byte for 2-byte alignment. */
    uint16_t    mfg_info_offset;/**< Byte 0x10: Offset to manufacturer information in config table. */
    uint8_t     mfg_info_length;/**< Byte 0x12: Length of manufacturer information in config table. */
    uint8_t     reserved_1;     /**< Byte 0x13: Reserved field for 2-byte alignment. */
    uint16_t    reserved_2[6];  /**< Byte 0x14: Reserved fields for future expansion. */

    pd_port_config_t port_conf[NO_OF_TYPEC_PORTS]; /**< Byte 0x20 (0x110): Configuration data for each USB-PD port. */
} pd_config_t;

/**
 * @brief Struct to hold response to policy manager.
 * Note: The app_resp_t structure is only used for responses that have a single DO. This may need
 * to get extended if more DOs are to be supported.
 */
typedef struct
{
    pd_do_t             resp_do;                /**< Response data object. */
    app_req_status_t    req_status;             /**< Request status. */
} app_resp_t;

/**
 * @brief Struct to hold response to policy manager.
 */
typedef struct
{
    pd_do_t     resp_buf[MAX_NO_OF_DO]; /**< Data objects buffer */
    uint8_t     do_count;               /**< Data objects count */
    vdm_ams_t   no_resp;                /**< Response type. */
} vdm_resp_t;

/**
 * @brief Struct to hold PD command buffer.
 * @warning When providing pointer to the extended data make sure original buffer
 * is always 4-byte aligned. i.e, even if 1 byte data is required, 4 bytes should be used to
 * store that data.
 */
typedef struct
{
    sop_t               cmd_sop;                /**< SOP type */
    extd_msg_t          extd_type;              /**< Extended Message Type */
    pd_extd_hdr_t       extd_hdr;               /**< Extended Header */
    uint8_t             no_of_cmd_do;           /**< No of data objects including VDM header */
    uint8_t*            dat_ptr;                /**< Data Pointer in case of extended message only*/
    uint8_t             timeout;                /**< Timeout value, in ms, for a response.
                                                 *   If set to zero, the PD stack will not wait for a VDM response
                                                 *   and jump to ready state after this buffer has been sent.
                                                 */
    pd_do_t             cmd_do[MAX_NO_OF_DO];   /**< Command data objects. */
} dpm_pd_cmd_buf_t;

/**
 * @brief Structure to hold PD Contract information.
 */
typedef struct
{
    uint16_t cur_pwr;           /**< PD contract current/power. */
    uint16_t max_volt;          /**< PD contract max voltage in mV */
    uint16_t min_volt;          /**< PD contract min voltage in mV */
} contract_t;

/**
 * @brief Stucture to hold PD Contract information passed with APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE
 * event to the application.
 */
typedef struct
{
    pd_do_t              rdo;           /**< RDO associated with the contract. */
    pd_contract_status_t status;        /**< Status of the contract. */
} pd_contract_info_t;

/**
 * @brief Struct to hold a PD packet.
 */
typedef struct
{
    uint8_t     sop;                    /**< Packet type. */
    uint8_t     len;                    /**< Length in data objects. */
    uint8_t     msg;                    /**< Message code. */
    uint8_t     data_role;              /**< Data role. */
    pd_hdr_t    hdr;                    /**< Message header. */
    pd_do_t     dat[MAX_NO_OF_DO];      /**< Data objects associated with the message. */
} pd_packet_t;

/**
 * @brief Struct to hold extended PD packets (messages).
 */
typedef struct
{
    uint8_t     sop;                    /**< Packet type. */
    uint8_t     len;                    /**< Length of the message: Unused for unchunked extended messages. */
    uint8_t     msg;                    /**< Message code. */
    uint8_t     data_role;              /**< Data role. */
    pd_hdr_t    hdr;                    /**< Message header, including extended header. */
    pd_do_t     dat[MAX_EXTD_PKT_WORDS];/**< Data associated with the message. */
} pd_packet_extd_t;

/**
 * @brief PD callback prototype.
 * This is a stack internal callback function used by the USB-PD Protocol
 * layer to send events to the Policy Engine. The events notified correspond
 * to Policy Engine events such as HARD RESET or SOFT RESET received.
 *
 * @param port PD port index.
 * @param Type of event being notified.
 */
typedef void (*pd_cbk_t)(uint8_t port, uint32_t event);

/**
 * @brief DPM PD command callback. This is the type of callback function used by the Policy Engine
 * to report results of a command to the application layer.
 *
 * @param port PD port index.
 * @param resp Response code.
 * @param pkt_ptr Pointer to any PD packet associated with the response.
 */
typedef void (*dpm_pd_cmd_cbk_t)(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr);

/**
 * @brief Application response callback. This is the type of callback used by the stack to
 * receive application level responses associated with a PD message such as a SWAP request.
 *
 * @param port PD port index.
 * @param resp Pointer to the structure holding response information.
 */
typedef void (*app_resp_cbk_t)(uint8_t port, app_resp_t* resp);

/**
 * @brief VDM response callback. This is the type of callback used by the stack to receive
 * application level responses to a VDM received from the port partner or cable marker.
 *
 * @param port PD port index.
 * @param resp Pointer to structure holding response information.
 */
typedef void (*vdm_resp_cbk_t)(uint8_t port, vdm_resp_t* resp);

/**
 * @brief Power ready callback. Type of callback used by the stack to receive notification
 * from the power source/sink hardware manager that the requested power transition has
 * been completed.
 *
 * @param port PD port index.
 */
typedef void (*pwr_ready_cbk_t)(uint8_t port);

/**
 * @brief Sink discharge off callback. Callback type used by the stack to receive
 * notification that the sink discharge circuit has been turned off.
 *
 * @param port PD port index.
 */
typedef void (*sink_discharge_off_cbk_t)(uint8_t port);

/**
 * @brief Type C command response callback. Type of callback used by the stack to
 * report results of a dpm_typec_command API call to the application layer.
 *
 * @param port PD port index.
 * @param resp Response code.
 */
typedef void (*dpm_typec_cmd_cbk_t)(uint8_t port, dpm_typec_cmd_resp_t resp);

/**
 * @brief Struct to hold the application interface. The application is expected to
 * fill the structure with pointers to functions that use the on-board circuitry to
 * accomplish tasks like source/sink power turn on/off. All the functions in this
 * structure must be non-blocking and take minimum execution time.
 *
 * @warning The application must check the callback pointer passed by the
 * stack is not NULL.
 */
typedef struct
{
    void (*app_event_handler) (
            uint8_t port,               /**< PD port index. */
            app_evt_t evt,              /**< Type of event. */
            const void* dat             /**< Event related data. */
            );                          /**< App event handler callback. */

    void (*psrc_set_voltage) (
            uint8_t port,               /**< PD port index. */
            uint16_t volt_mV            /**< Target voltage in mV units. */
            );                          /**< Set power source voltage in mV units. */

    void (*psrc_set_current) (
            uint8_t port,               /**< PD port index. */
            uint16_t cur_10mA           /**< Expected operating current in 10 mA units. */
            );                          /**< Set power source current in 10mA units. */

    void (*psrc_enable) (
            uint8_t port,               /**< PD port index. */
            pwr_ready_cbk_t pwr_ready_handler   /**< Function to be called after power enable. */
            );                          /**< Enable the power supply. The pwr_ready_handler, if not NULL, must be
                                         *   called when the power supply is ready.
                                         */

    void (*psrc_disable) (
            uint8_t port,               /**< PD port index. */
            pwr_ready_cbk_t pwr_ready_handler   /**< Function to be called after power disable. */
            );                          /**< Disable the power supply. The pwr_ready_handler, if not NULL,
                                         *   must be called when the power supply has been discharged to Vsafe0V. */

    void (*vconn_enable) (
            uint8_t port,               /**< PD port index. */
            uint8_t channel             /**< CC channel on which to enable VConn. */
            );                          /**< Turn VCONN supply ON. */

    void (*vconn_disable) (
            uint8_t port,               /**< PD port index. */
            uint8_t channel             /**< CC channel on which to disable VConn. */
            );                          /**< Turn VCONN supply OFF. */

    bool (*vconn_is_present) (
            uint8_t port                /**< PD port index. */
            );                          /**< Check whether VConn supply is ON. */

    bool (*vbus_is_present) (
            uint8_t port,               /**< PD port index. */
            uint16_t volt,              /**< Expected voltage in mV units. */
            int8_t per                  /**< Allowed voltage deviation in percentage units. */
            );                          /**< Check whether VBus voltage is within expected range. */

    void (*vbus_discharge_on) (
            uint8_t port                /**< PD port index. */
            );                          /**< Turn on VBUS discharge circuit. */

    void (*vbus_discharge_off) (
            uint8_t port                /**< PD port index. */
            );                          /**< Turn off VBUS discharge circuit. */

#if (!(CCG_SOURCE_ONLY))
    void (*psnk_set_voltage) (
            uint8_t port,               /**< PD port index. */
            uint16_t volt_mV            /**< Target voltage in mV units. */
            );                          /**< Set power sink voltage, in mV units. */

    void (*psnk_set_current) (
            uint8_t port,               /**< PD port index. */
            uint16_t cur_10mA           /**< Operating current in 10 mA units. */
            );                          /**< Set power sink current, in 10mA units. */

    void (*psnk_enable) (
            uint8_t port                /**< PD port index. */
            );                          /**< Enable power sink related circuitry. */

    void (*psnk_disable) (
            uint8_t port,               /**< PD port index. */
            sink_discharge_off_cbk_t snk_discharge_off_handler  /**< Callback to be called after discharge is done. */
            );                          /**< Disable power sink related circuitry. */

    void (*eval_src_cap) (
            uint8_t port,               /**< PD port index. */
            const pd_packet_t* src_cap, /**< Pointer to list of received source caps. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Evaluate received source caps and provide the RDO to be used
                                             to negotiate contract. */
#endif /* (!(CCG_SOURCE_ONLY)) */

    void (*eval_rdo)(
            uint8_t port,               /**< PD port index. */
            pd_do_t rdo,                /**< Received RDO object. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Evaluate sink request message. */

    void (*eval_dr_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handles DR swap request received by port. */
    void (*eval_pr_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handles pr swap request received by port. */

    void (*eval_vconn_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handles VCONN swap request received by port. */

    void (*eval_vdm) (
            uint8_t port,               /**< PD port index. */
            const pd_packet_t *vdm,     /**< Pointer to received VDM. */
            vdm_resp_cbk_t vdm_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handle VDMs (all structured/unstructured VDMs need to be handled) received
                                             by the port. */
#if (!(CCG_SOURCE_ONLY))
    void (*eval_fr_swap) (
            uint8_t port,               /**< PD port index. */
            app_resp_cbk_t app_resp_handler     /**< Return parameter to report response through. */
            );                          /**< Handle FR swap request received by the specified port. */
#endif /* (!(CCG_SOURCE_ONLY)) */

    uint16_t (*vbus_get_value) (
            uint8_t port                /**< PD port index. */
            );                          /**< Get current VBUS value in mV from application. */

} app_cbk_t;

/**
 * @brief PD Device Policy Status structure. This structure holds all of the configuration and status
 * information associated with a port on the CCG device. Members of this structure should not be
 * directly modified by any of the application code.
 *
 * @warning Initial elements of this structure maps directly to config table
 * fields and hence must not be moved around or changed.
 */
typedef struct
{
    uint8_t port_role;          /**< Port role: Sink, Source or Dual. */
    uint8_t dflt_port_role;     /**< Default port role: Sink or Source. */
    uint8_t src_cur_level;      /**< Type C current level in the source role. */
    uint8_t is_src_bat;         /**< Power source is connected to a battery. */
    uint8_t is_snk_bat;         /**< Power sink is connected to a battery. */
    uint8_t snk_usb_susp_en;    /**< USB suspend supported indication. */
    uint8_t snk_usb_comm_en;    /**< USB communication supported indication. */
    uint8_t swap_response;      /**< Response to be sent for each USB-PD SWAP command.
                                     - Bits 1:0 => DR_SWAP response.
                                     - Bits 3:2 => PR_SWAP response.
                                     - Bits 5:4 => VCONN_SWAP response.
                                     Allowed values are: 0=ACCEPT, 1=REJECT, 2=WAIT, 3=NOT_SUPPORTED.
                                 */

    uint8_t toggle;             /**< DRP toggle is enabled. */
    uint8_t src_pdo_count;      /**< Source PDO count from the config table or updated at runtime by the EC. */
    uint8_t src_pdo_mask;       /**< Source PDO mask from the config table or updated at runtime by the EC. */
    uint8_t snk_pdo_count;      /**< Sink PDO count from the config table or updated at runtime by the EC. */
    uint8_t snk_pdo_mask;       /**< Sink PDO mask from the config table or updated at runtime by the EC. */
    uint8_t rp_supported;       /**< Supported Rp values bit mask.
                                     - Bit 0 => Default Rp supported.
                                     - Bit 1 => 1.5 A Rp supported.
                                     - Bit 2 => 3 A Rp supported.
                                 */

    uint8_t pd_support;         /**< USB-PD supported. */
    uint8_t try_src_snk;        /**< Try Source/ Try Sink control knob. */
    uint8_t cbl_dsc;            /**< Cable discovery control knob. */
    uint8_t db_support;         /**< Dead battery support control knob. */
    uint8_t err_recov;          /**< Error recovery control knob.*/
    uint8_t port_disable;       /**< PD port disable flag. */
    uint8_t frs_enable;         /**< FRS enable flags. */
    uint8_t vconn_retain;       /**< Whether VConn should be retained in ON state. */

    uint16_t reserved_3[5];     /**< Reserved words for padding to 4-byte aligned address. */

    pd_do_t src_pdo[MAX_NO_OF_PDO];     /**< Source PDO loaded from the config table or updated at runtime by the EC. */
    pd_do_t snk_pdo[MAX_NO_OF_PDO];     /**< Sink PDO loaded from the config table or updated at runtime by the EC. */
    uint16_t snk_max_min[MAX_NO_OF_PDO];/**< Max min current from the config table or updated at runtime by the EC. */

    port_type_t cur_port_type;  /**< Current port type: UFP or DFP. */
    port_role_t cur_port_role;  /**< Current Port role: Sink or Source. */
    uint8_t volatile snk_cur_level;     /**< Type C current level in sink role. */
    uint8_t volatile bootup;    /**< Flag to indicate chip bootup, used to check dead battery. */
    uint8_t volatile dead_bat;  /**< Flag to indicate dead battery operation. */
    uint8_t drp_period;         /**< Time period for DRP toggling. */
    uint8_t src_period;         /**< Time period for which to stay as a SRC for a DRP device. */
    uint8_t snk_period;         /**< Time period for which to stay as a SNK for a DRP device. */
    uint8_t volatile skip_scan; /**< Skip CC scanning control knob. */

    uint8_t polarity;                   /**< CC channel polarity (CC1=0, CC2=1) */
    uint8_t rev_pol;                    /**< CC channel reverse polarity. */
    uint8_t volatile connect;           /**< Port connected but not debounced yet. */
    uint8_t volatile attach;            /**< Port attached indication. */
    port_role_t role_at_connect;        /**< Port role when the port moved to the attached state. */
    pd_devtype_t attached_dev;          /**< Type of device attached. */
    uint8_t volatile contract_exist;    /**< PD contract exists indication. */
    uint8_t volatile pd_connected;      /**< Ports are PD connected indication. */
    uint8_t volatile pd_disabled;       /**< PD disabled indication. */
    uint8_t volatile ra_present;        /**< Ra present indication. */
    uint8_t volatile emca_present;      /**< EMCA cable present indication. */
    uint8_t volatile bist_cm2_enabled;  /**< BIST Carrier Mode 2 going on */

    std_vdm_prod_t cbl_type;            /**< Stores the cable type. */
    std_vdm_ver_t cbl_vdm_version;      /**< Stores the cable VDM version. */
    uint8_t volatile vconn_logical;     /**< VCONN logical status. */
    uint8_t cur_src_pdo_count;          /**< Source PDO count in the last sent source cap. */
    uint8_t cur_snk_pdo_count;          /**< Sink PDO count in the last sent sink cap. */
    uint8_t cbl_wait;                   /**< Flag to indicate cable discovery is waiting for some event. */
    pe_cbl_state_t cbl_state;           /**< Cable discovery state machine state. */
    uint8_t cbl_soft_reset_tried;       /**< Stores number of cable soft reset tries. */

    typec_fsm_state_t typec_fsm_state;  /**< Type C generic FSM state. */
    dpm_pd_cmd_t dpm_pd_cmd;            /**< Current DPM PD command. */
    uint8_t dpm_pd_cmd_active;          /**< Indicate DPM PD command was registered. */
    uint8_t dpm_typec_cmd_active;       /**< Indicate DPM Type C command was registered. */
    bool dpm_enabled;                   /**< DPM enabled flag. */
    bool dpm_init;                      /**< DPM Initialized flag. */

    uint8_t dpm_safe_disable;           /**< DPM sage disable flag. Used to make sure that the port is disabled
                                             correctly after a fault occurrence. */

    
    dpm_typec_cmd_t dpm_typec_cmd;      /**< Current DPM Type C command. */
    uint8_t src_cur_level_live;         /**< Current Type C current level in the source role. */
    cc_state_t cc_live;                 /**< Live CC status. */
    cc_state_t cc_status;               /**< Current debounced CC status. */
    cc_state_t cc_old_status;           /**< Old CC status. */
    cc_state_t cc_rd_status;            /**< Rd status. */
    pd_rev_t spec_rev_sop_live;         /**< Current spec rev for SOP messages. */

    pd_rev_t spec_rev_sop_prime_live;   /**< Current spec revision for SOP Prime/DPrime messages. */
    pd_rev_t spec_rev_cbl;              /**< Spec revision of the currently connected cable. */
    pd_rev_t spec_rev_peer;             /**< Spec revision of the currently connected peer. */
    bool unchunk_sup_live;              /**< Mutual unchunk support with the currently connected peer. */
    bool unchunk_sup_peer;              /**< Unchunk support of the currently connected peer. */
    bool snk_rp_detach_en;              /**< Flag to indicate sink will detach on Rp removal instead of VBUS removal. */
    bool cur_fb;                        /**< Flag to indicate current foldback is active */
    pd_ams_type non_intr_response;      /**< Flag to indicate stack is waiting for App response to a
                                            non interruptible AMS */

    
    bool fr_rx_disabled;                /**< FRS receive disabled by EC. */
    bool fr_rx_en_live;                 /**< Fast role signal detection enabled */
    bool fr_tx_disabled;                /**< FRS transmit disabled by EC. */
    bool fr_tx_en_live;                 /**< Fast role signal auto send enabled */
    volatile bool fault_active;         /**< Flag to indicate the a fault consition exists. */
    pe_fsm_state_t pe_fsm_state;        /**< Holds the current state of Policy Engine (PE). */
    uint32_t volatile pe_evt;           /**< Stores policy engine events. */
    uint32_t volatile typec_evt;        /**< Stores Type C events. */
    contract_t contract;                /**< Current pd contract. */
    pd_do_t alert;                      /**< Alert status */

    pd_do_t cbl_vdo;                    /**< Stores the last received cable VDO. */
    bool cbl_mode_en;                   /**< Whether cable supports alternate modes. */

    app_cbk_t* app_cbk;                 /**< Application callback pointer. */
    dpm_pd_cmd_cbk_t dpm_pd_cbk;        /**< Pointer to DPM PD callback function. */
    dpm_typec_cmd_cbk_t dpm_typec_cbk;  /**< Pointer to DPM Type C callback function. */
    dpm_pd_cmd_buf_t* cmd_p;            /**< Pointer to DPM command buffer. */
    dpm_pd_cmd_buf_t dpm_cmd_buf;       /**< Local DPM command buffer. */

    uint16_t cur_snk_max_min[MAX_NO_OF_PDO];    /**< Max min current/power of current sink capabilities. */
    pd_do_t cur_src_pdo[MAX_NO_OF_PDO];         /**< Current source PDOs sent in source cap messages. */
    pd_do_t cur_snk_pdo[MAX_NO_OF_PDO];         /**< Current sink PDOs sent in sink cap messages. */
    pd_do_t src_cur_rdo;                        /**< Stores the current rdo received by source */
    pd_do_t src_last_rdo;                       /**< Stores the last rdo received by source */
    pd_do_t src_rdo;                            /**< Last RDO received (when operating as a source) that resulted in
                                                     a contract. */
    pd_do_t snk_rdo;                            /**< Last RDO sent (when operating as a sink) that resulted in
                                                     a contract. */
    pd_do_t snk_sel_pdo;                        /**< Selected PDO which resulted in contract (when sink). */
    pd_do_t src_sel_pdo;                        /**< Selected PDO which resulted in contract (when source). */

    pd_packet_t *src_cap_p;                     /**< Pointer to the current/last source cap message received.
                                                     May be NULL. Data pointed to by this should not be changed. */

    uint32_t padval;                            /**< Fields below need to be properly aligned to 4 byte boundary */
    pd_power_status_t port_status;              /**< Port power status. */
    uint8_t ext_src_cap[CCG_PD_EXT_SRCCAP_SIZE];        /**< Buffer to hold extended source caps. */
    uint8_t pps_status[CCG_PD_EXT_PPS_STATUS_SIZE];     /**< Buffer to hold PPS status. */
    uint8_t dpm_err_info;                       /**< Additional error status for DPM commands. */

#if DPM_DEBUG_SUPPORT
    /* Parameters added for Thunderbolt dock support. */
    pd_do_t active_rdo;                                 /**< RDO that has been accepted last. */
    uint8_t contract_flags;                             /**< Stores the contract flags. */
    uint8_t src_pdo_flags;                              /**< Stores the source PDO flags */
    uint8_t err_recov_reason;                           /**< Reason for port entering Type-C error recovery. */
    uint8_t sopdp_soft_reset_reason;                    /**< Reason for port issuing SOP'' SoftReset. */
    uint8_t sopp_soft_reset_reason;                     /**< Reason for port issuing SOP' SoftReset. */
    uint8_t cable_reset_reason;                         /**< Reason for port issuing Cable Reset. */
    uint8_t hard_reset_reason;                          /**< Reason for port issuing Hard Reset. */
    uint8_t soft_reset_reason;                          /**< Reason for port issuing Soft Reset. */
    uint8_t sopdp_present;                              /**< Whether SOP'' cable controller is present. */
    uint8_t cc_stat[2];                                 /**< Current state of the CC1 and CC2 pins. */

    /* Debug counters  */
    uint8_t  connection_count;                          /**< Number of connections since power-up. */
    uint8_t  fault_count;                               /**< Number of faults in current connection. */
    uint8_t  contr_negotiation_count;                   /**< Number of contracts made during current connection. */
    uint16_t pd_msgs_sent;                              /**< Number of messages sent during current connection. */
    uint16_t pd_msgs_rxd;                               /**< Number of messages received during current connection. */
#endif /* DPM_DEBUG_SUPPORT */
} dpm_status_t;

/*****************************************************************************
 * Global Function Declarations
 *****************************************************************************/

/**
 * @brief This function gets the config table data.
 * @return Returns a pointer to the config table info structure.
 * @warning The information provided by this API must not be altered by the
 * application.
 */
const pd_config_t* get_pd_config(void);

/**
 * @brief This function gets the configuration information for the specified port.
 * @param port Port index.
 * @return Returns a pointer to the port specific config table info structure.
 * @warning The information provided by this API must not be altered by the
 * application.
 */
const pd_port_config_t* get_pd_port_config(uint8_t port);

/**
 * @brief This is a utility function to check if the packet is an expected
 * message of a certain class.
 * @param pkt Packet pointer.
 * @param sop_type Sop type to match. Passing SOP_INVALID will skip this check.
 * @param msg_class Message class: Control, Data or Extended.
 * @param msg_mask Message mask is a 32 bit mask. Each bit corresponds to a
 * message type corresponding to the message class. If mask is 0, then this
 * check is skipped.
 * @param length Length corresponds to the 'Number of Data Objects' field in
 * the PD header. If length is 0xFFFF, this check is skipped.
 * @return Returns true if packet matches all the conditions, else false.
 */
bool pd_is_msg(const pd_packet_t* pkt, sop_t sop_type, pd_msg_class_t msg_class, uint32_t msg_mask, uint16_t length);

/**
 * @brief Retrieve pointer to VBus OVP settings from the configuration table.
 *
 * This function retrieves the VBus OVP settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 * @return Pointer to VBus OVP settings.
 */
ovp_settings_t* pd_get_ptr_ovp_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to VBus OCP settings from the configuration table.
 *
 * This function retrieves the VBus OCP settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 * @return Pointer to VBus OCP settings.
 */
ocp_settings_t* pd_get_ptr_ocp_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to VBus UVP settings from the configuration table.
 *
 * This function retrieves the VBus UVP settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 * @return Pointer to VBus UVP settings.
 */
uvp_settings_t* pd_get_ptr_uvp_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to VBus SCP settings from the configuration table.
 *
 * This function retrieves the VBus SCP settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 * @return Pointer to VBus SCP settings.
 */
scp_settings_t* pd_get_ptr_scp_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to Vconn OCP settings from the configuration table.
 *
 * This function retrieves the Vconn OCP settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 * @return Pointer to Vconn OCP settings.
 */
vconn_ocp_settings_t* pd_get_ptr_vconn_ocp_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to OTP settings from the configuration table.
 *
 * This function retrieves the OTP settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to OTP settings.
 */
otp_settings_t* pd_get_ptr_otp_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to power parameters from the configuration table.
 *
 * This function retrieves the power parameters that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to power parameters.
 */
pwr_params_t* pd_get_ptr_pwr_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to battery charging parameters from the configuration table.
 *
 * This function retrieves the legacy charging parameters that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to legacy charging parameters.
 */
chg_cfg_params_t* pd_get_ptr_chg_cfg_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to battery power parameters from the configuration table.
 *
 * This function retrieves the battery charging parameters that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to battery charging parameters.
 */
bat_chg_params_t* pd_get_ptr_bat_chg_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to power parameters of Type-A port from the configuration table.
 *
 * This function retrieves the power parameters of Type-A port that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to power parameters of Type-A port.
 */
pwr_params_t* pd_get_ptr_type_a_pwr_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to TYPE-A battery charging parameters from the configuration table.
 *
 * This function retrieves the legacy charging parameters that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to legacy charging parameters.
 */
typeA_chg_cfg_params_t* pd_get_ptr_type_a_chg_cfg_tbl(uint8_t port);

/**
 * @brief Retrieve pointer to Billboard settings from the configuration table.
 *
 * This function retrieves the Billboard settings that is stored in
 * the configuration table and stores it in the run-time data structures.
 *
 * @param port USB-PD port for which the data is to be retrieved.
 *
 * @return Pointer to Billboard settings.
 */
bb_settings_t* pd_get_ptr_bb_tbl(uint8_t port);

/*
 * This table contains type c current advertisement in 10mA units for various Rp levels
 * and it maps to enum rd_cc_status_t
 */
extern const uint16_t CUR_TABLE[5];

#endif /* _PD_H_ */

/* End of file */
