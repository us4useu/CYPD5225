/**
 * @file dp_sid.h
 *
 * @brief @{DisplayPort alternate mode handler header file.@}
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

#ifndef _DP_SID_H_
#define _DP_SID_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>
#include <alt_modes_mngr.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
  @brief DisplayPort SVID.
 */
#define DP_SVID                         (0xFF01u)
/**
  @brief DP alternative mode ID.
 */
#define DP_ALT_MODE_ID                     (0u)
/**
  @brief Maximum number of VDO DP uses for the alt mode flow.
 */
#define MAX_DP_VDO_NUMB                    (1u)
/**
  @brief Index of VDO buffer element which uses to handle DP VDO.
 */
#define DP_VDO_IDX                        (0u)
/**
  @brief DP Status Update command VDO when DP is DFP.
 */
#define STATUS_UPDATE_VDO                (0x01u)
/**
  @brief Statndard DP signaling type value in the signaling field for Config VDO.
 */
#define DP_1_3_SIGNALING                (0x0001u)
/**
  @brief Value when DFP set configuration for UFP_U as UFP_D in the configuration
  select field for Config VDO.
 */
#define DP_CONFIG_SELECT                (2u)

/**
  @brief Value when DFP set configuration as USB in the configuration select field
    for Config VDO.
 */
#define USB_CONFIG_SELECT               (0u)

/**
  @brief Pin assigment mask for USB pin assigment in the Configure Pin Assigment
  field for Config VDO.
 */
#define DP_USB_SS_CONFIG                (0b00000000u)
/**
  @brief Pin assigment mask for C pin assigment in the Configure Pin Assigment
  field for Config VDO.
 */
#define DP_DFP_D_CONFIG_C               (0b00000100u)
/**
  @brief Pin assigment mask for D pin assigment in the Configure Pin Assigment
  field for Config VDO.
 */
#define DP_DFP_D_CONFIG_D               (0b00001000u)
/**
  @brief Pin assigment mask for E pin assigment in the Configure Pin Assigment
  field for Config VDO.
 */
#define DP_DFP_D_CONFIG_E               (0b00010000u)
/**
  @brief Pin assigment mask for F pin assigment in the Configure Pin Assigment
  field for Config VDO.
 */
#define DP_DFP_D_CONFIG_F               (0b00100000u)
/**
  @brief Internal DP denotation for invalid pin assigment
 */
#define DP_INVALID_CFG                  (0b11111111u)

/**
  @brief HPD low and IRQ low value getting from UFP Attention/Status Update VDO
 */
#define HPD_LOW_IRQ_LOW                 (0x0u)
/**
  @brief HPD high and IRQ low value getting from UFP Attention/Status Update VDO
 */
#define HPD_HIGH_IRQ_LOW                (0x1u)
/**
  @brief HPD low and IRQ high value getting from UFP Attention/Status Update VDO
 */
#define HPD_LOW_IRQ_HIGH                (0x2u)
/**
  @brief HPD high and IRQ high value getting from UFP Attention/Status Update VDO
 */
#define HPD_HIGH_IRQ_HIGH               (0x3u)
/**
  @brief HPD queue one element size in bits
 */
#define DP_QUEUE_STATE_SIZE             (2u)
/**
  @brief HPD queue one element state mask
 */
#define DP_HPD_STATE_MASK               (0x0003u)
/**
  @brief Flag to indicate an empty queue
 */
#define DP_QUEUE_EMPTY_INDEX            (0x0u)
/**
  @brief Flag to indicate an full queue
 */
#define DP_QUEUE_FULL_INDEX             (7u)
/**
  @brief Flag to indicate DP UFP's full queue
 */
#define DP_UFP_MAX_QUEUE_SIZE           (4u)
/**
  @brief Macross to get HPD state from Attention/Status Update VDO
 */
#define GET_HPD_IRQ_STAT(status)        (((status) >> 7u)& 0x3 )
/**
  @brief DP Sink Control APP command value
 */
#define    DP_SINK_CTRL_CMD           (3u)
/**
  @brief DP MUX Control APP command value
 */
#define    DP_MUX_CTRL_CMD            (1u)
/**
  @brief DP MUX Configure APP command value
 */
#define    DP_APP_CFG_CMD             (2u)
/**
  @brief Bit index of USB configuration in APP DP MUX Configure command data
 */
#define DP_APP_CFG_USB_IDX            (6u)
/**
  @brief Maximum vualue of USB configuration in APP DP MUX Configure command data
 */
#define DP_APP_CFG_CMD_MAX_NUMB       (6u)
/**
  @brief DP Allowed MUX Configuration APP event value
 */
#define DP_ALLOWED_MUX_CONFIG_EVT     (1u)
/**
  @brief DP Status Update APP event value
 */
#define DP_STATUS_UPDATE_EVT          (2u)
/**
  @brief Acked field mask for EC DP MUX Configure command data
 */
#define DP_CFG_CMD_ACK_MASK           (0x200)

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
  @typedef dp_state_t
  @brief This enumeration holds all possible DP states.
 */
typedef enum
{
    DP_STATE_IDLE = 0,              /**< Idle state. */
    DP_STATE_ENTER = 4,             /**< Enter mode state. */
    DP_STATE_STATUS_UPDATE = 16,    /**< DP Status Update state. */
    DP_STATE_CONFIG = 17,           /**< DP Configure state. */
    DP_STATE_ATT = 6,               /**< DP Attention state. */
    DP_STATE_EXIT = 5,              /**< Exit mode state. */

}dp_state_t;

/**
  @typedef dp_port_cap_t
  @brief This enumeration holds possible DP capabilities.
 */
typedef enum
{
    DP_PORT_CAP_RSVD = 0,            /**< Reserved capability. */
    DP_PORT_CAP_UFP_D ,              /**< UFP is UFP_D-capable. */
    DP_PORT_CAP_DFP_D ,              /**< UFP is DFP_D-capable. */
    DP_PORT_CAP_BOTH,                /**< UFP is DFP_D and UFP-D capable. */

}dp_port_cap_t;

/**
  @typedef dp_conn_t
  @brief This enumeration holds possible DFP_D/UFP_D Connected status (Status Update Messege).
 */
typedef enum
{
    DP_CONN_NONE = 0,               /**< Neither DFP_D nor UFP_D is connected. */
    DP_CONN_DFP_D ,                 /**< DFP_D is connected. */
    DP_CONN_UFP_D ,                 /**< UFP_D is connected. */
    DP_CONN_BOTH,                   /**< Both DFP_D and UFP_D are connected. */

}dp_conn_t;

/**
  @typedef dp_stat_bm_t
  @brief This enumeration holds corresponding bit positions of Status Update VDM fields.
 */
typedef enum
{
    DP_STAT_DFP_CONN = 0,           /**< DFP_D is connected field bit position. */
    DP_STAT_UFP_CONN ,              /**< UFP_D is connected field bit position. */
    DP_STAT_PWR_LOW ,               /**< Power Low field bit position. */
    DP_STAT_EN ,                    /**< Enabled field bit position. */
    DP_STAT_MF ,                    /**< Multi-function Prefered field bit position. */
    DP_STAT_USB_CNFG ,              /**< USB Configuration Request field bit position. */
    DP_STAT_EXIT ,                  /**< Exit DP Request field bit position. */
    DP_STAT_HPD ,                   /**< HPD state field bit position. */
    DP_STAT_IRQ ,                   /**< HPD IRQ field bit position. */

}dp_stat_bm_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function analyses Discovery information to find out
 * if further DP alternative mode processing is allowed.
 *
 * @param port Port index the function is performed for.
 * @param reg_info Pointer to structure which holds alt mode register info.
 *
 * @return Pointer to DP alternative mode command structure if analisys passed
 * successful. In other case function returns NULL pointer
 */
alt_mode_info_t* reg_dp_modes(uint8_t port, alt_mode_reg_info_t* reg_info);

#if DP_GPIO_CONFIG_SELECT
/**
 * @brief Store the DP Pin configuration based on GPIO status.
 *
 * @param dp_config DP Pin configuration
 *
 * @return None
 */
void dp_sink_set_pin_config(uint8_t dp_config);

/**
 * @brief Return the DP Pin configuration based on GPIO status.
 *
 * @param None
 *
 * @return DP Pin configuration
 */
uint8_t dp_sink_get_pin_config(void);
#endif /* DP_GPIO_CONFIG_SELECT */

/* Workaround for CCG4 Dock. This info is needed to enable config E */
#if CCG4_DOCK
uint8_t get_dp_config(uint8_t port);
#endif /* CCG4_DOCK */

#endif /* _DP_SID_H_ */

/* [] END OF FILE */
