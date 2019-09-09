/**
 * @file intel_ridge.h
 *
 * @brief @{Intel Alpine/Titan Ridge control interface header file.@}
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

#ifndef _INTEL_RIDGE_H_
#define _INTEL_RIDGE_H_

/*****************************************************************************
 * Header files including
 *****************************************************************************/

#include <alt_mode_hw.h>
#include <status.h>
#include <hpd.h>

#define RIDGE_DISCON_STATE_MASK                  (0x00)
/** @brief Disconnect state bit mask for Status register. */

#define RIDGE_DATA_CONN_MASK                     (0x10171)
/** @brief Data connection bit mask for Status register. */

#define RIDGE_USB_STATE_MASK                     (0x31)
/** @brief USB only state bit mask for  Status register. */

#define RIDGE_SAFE_STATE_MASK                    (0x01)
/** @brief Safe state bit mask for  Status register. */

#define RIDGE_DP_4_LANE_MASK                     (0x111)
/** @brief 4 lane DP alt mode state bit mask for  Status register. */

#define RIDGE_DP_2_LANE_MASK                     (0x531)
/** @brief 2 lane DP alt mode state bit mask for Status register. */

#define DP_PIN_CONFIG_C                          (0b00000100u)
/** @brief Pin assigment mask for C pin assigment. */

#define RIDGE_TBT_MODE_MASK                      (0x10001)
/** @brief TBT alt mode state bit mask for Titan register. */

#define RIDGE_DEBUG_MODE_MASK                    (0x11)
/** @brief Debug alt mode state bit mask for Status register. */

#define TBT_HOST_CONN_MASK                       (0x01)
/** @brief TBT HOST Connected bit mask for Command register. */

#define TR_IRQ_ACK_MASK                          (0x2000)
/** @brief HPD IRQ ACK bit mask for Titan Ridge command register. */

#define TR_HPD_IRQ_MASK                          (0x4000)
/** @brief HPD IRQ bit mask for Titan Ridge command/status register. */

#define TR_HPD_LVL_MASK                          (0x8000)
/** @brief HPD Level bit mask for Titan Ridge command register. */

#define TR_USB_HOST_CONN_MASK                    (0x10)
/** @brief USB HOST Connected bit mask for Titan Ridge command register. */

#define TR_DP_HOST_CONN_MASK                     (0x20)
/** @brief DP HOST Connected bit mask for Titan Ridge command register. */

#define RIDGE_STATUS_OCP_MASK                    (0x08)
/** @brief Over-Current status bit in the status register. */

/*****************************************************************************
 * Data Struct Definition
 *****************************************************************************/

/**
  @union ridge_reg_t
  @brief Union to hold AR/TR Registers
 */
typedef union
{
    uint32_t val;                       /**< Integer field used for direct manipulation of reason code. */

    /** @brief Struct containing USB-PD controller status to be reported to Alpine/Titan Ridge.
        Using the structure definition corresponding to Titan Ridge in all cases.
     */
    struct USBPD_STATUS_REG
    {
        uint32_t data_conn_pres  : 1;   /**< B0: Whether data connection is present. */
        uint32_t conn_orien      : 1;   /**< B1: CC polarity. */
        uint32_t active_cbl      : 1;   /**< B2: Active cable. From B22 of Cable MODE Response. */
        uint32_t ovc_indn        : 1;   /**< B3: Over-Current indication. */
        uint32_t usb2_conn       : 1;   /**< B4: USB 2.0 connection. Set when no ALT MODES are active. */
        uint32_t usb3_conn       : 1;   /**< B5: USB 3.1 connection. Set when no ALT MODES are active. */
        uint32_t usb3_speed      : 1;   /**< B6: USB Gen2/Gen1 speed. From B18-B16 of Cable MODE Response. */
        uint32_t usb_dr          : 1;   /**< B7: Data role. DFP=0, UFP=1. */
        uint32_t dp_conn         : 1;   /**< B8: DP connection status. */
        uint32_t dp_role         : 1;   /**< B9: DP direction. Source=0, Sink=1. */
        uint32_t dp_pin_assign   : 2;   /**< B[11-10]: DP pin assignment. 4-lane='b00 2-lane='b01 */
        uint32_t dbg_acc_mode    : 1;   /**< B12: USB Type-C Debug accessory mode. */
        uint32_t irq_ack         : 1;   /**< B13: Set after receiving GoodCRC from Attention message with IRQ_HPD. */
        uint32_t hpd_irq         : 1;   /**< B14: HPD IRQ received from DP Sink. */
        uint32_t hpd_lvl         : 1;   /**< B15: HPD level received from DP Sink. */
        uint32_t tbt_conn        : 1;   /**< B16: TBT connection status. */
        uint32_t tbt_type        : 1;   /**< B17: TBT type. From B16 of UFP MODE Response. */
        uint32_t cbl_type        : 1;   /**< B18: TBT cable type. From B21 of Cable MODE Response. */
        uint32_t pro_dock_detect : 1;   /**< B19: Reporting of vPro Support. */
        uint32_t act_link_train  : 1;   /**< B20: Active TBT link training. From B23 of Cable MODE Response. */
        uint32_t dbg_alt_m_conn  : 1;   /**< B21: NIDnT Alt mode defined in MIPI SVID = 0xFF03. */
        uint32_t rsvd            : 1;   /**< B22: Reserved. */
        uint32_t force_lsx       : 1;   /**< B23: Set to zero. */
        uint32_t pwr             : 1;   /**< B24: Indicates PWR mismatch (Used only in TBT BPD). */
        uint32_t tbt_cbl_spd     : 3;   /**< B[27-25]: Cable speed. From B18-B16 of Cable MODE Response. */
        uint32_t tbt_cbl_gen     : 2;   /**< B[29-28]: Cable generation. From B20-19 of Cable MODE Response. */
        uint32_t rsvd4           : 1;   /**< B30: Reserved. */
        uint32_t interrupt_ack   : 1;   /**< B31: Interrupt indication (Set by EC when in I2C slave Mode). */
    }ridge_stat;                        /**< PD-controller status. */

    /** @brief Struct containing USB-PD controller command register fields for Alpine/Titan Ridge. */
    struct USBPD_CMD_REG
    {
        uint32_t tbt_host_conn   : 1;   /**< B0: TBT host connected. */
        uint32_t soft_rst        : 1;   /**< B1: Issue USB-PD Controller soft reset. */
        uint32_t i2c_int_ack     : 1;   /**< B2: Alpine/Titan Ridge acknowledge for the interrupt. */
        uint32_t rsvd2           : 1;   /**< B3: Reserved. */
        uint32_t usb_host_conn   : 1;   /**< B4: Indicates that USB Host is connected (TBT device only). */
        uint32_t dp_host_conn    : 1;   /**< B5: Indicates that DP Host is connected (TBT device only). */
        uint32_t rsvd3           : 7;   /**< B[12-6]: Reserved. */
        uint32_t irq_ack         : 1;   /**< B13: IRQ ACK PD Controller to Titan Ridge. */
        uint32_t hpd_irq         : 1;   /**< B14: HPD IRQ when acting as DP Sink. */
        uint32_t hpd_lvl         : 1;   /**< B15: HPD level when acting as DP Sink. */
        uint32_t rsvd4           : 16;  /**< B[31-16]: Reserved. */
    }usbpd_cmd_reg;                     /**< PD-controller command register. */

}ridge_reg_t;

/**
 * @brief Type of callback function used for notification of control register changes.
 */
typedef void (*ridge_ctrl_change_cb_t)(
        uint8_t port                    /**< Port on which control register change happened. */
        );

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function set AR/TR registers in accordance to input parameters
 *
 * @param port Port index the AR/TR settings are performed for.
 * @param mux_cfg MUX configuration.
 * @param polarity Attached target Type-C Polarity.
 * @param cfg Contains AR/TR register settings in case of TBT alt mode is active.
 *
 * @return true if AR/TR was set successful, in the other case - false
 */
bool ridge_set_mux(uint8_t port, mux_select_t mux_cfg, uint8_t polarity, uint32_t cfg);

/**
 * @brief Enables Titan Ridge the HPD functionality for the specified PD port.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @param cbk callback to be used for command completion event.
 * @return Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t tr_hpd_init(uint8_t port, hpd_event_cbk_t cbk);

/**
 * @brief Disables Titan Ridge the HPD functionality for the specified PD port.
 *
 * @param port PD port index. Caller should ensure to provide only valid values.
 * @return None.
 */
void tr_hpd_deinit(uint8_t port);

/**
 * @brief Send the desired HPD event out through the Titan Ridge HPD GPIO. Only
 * the HPD_EVENT_UNPLUG, HPD_EVENT_UNPLUG and HPD_EVENT_IRQ events
 * should be requested.
 *
 * @param port Port on which HPD event is to be sent.
 * @param evtype Type of HPD event to be sent.
 * @return Returns CCG_STAT_SUCCESS in case of success, error code otherwise.
 */
ccg_status_t tr_hpd_sendevt(uint8_t port, hpd_event_type_t evtype);

/**
 * @brief Analyses received ridge command register content.
 *
 * @param port USB-PD port index corresponding to the status update.
 * @param stat Command register value.
 * @param stat_mask Bit mask of TR command register which were changed from the previous time
 *
 * @return true if the interface is idle, false otherwise.
 */
void ridge_eval_cmd(uint8_t port, uint32_t stat, uint32_t stat_mask);

/**
 * @brief Indicates is HPD level changed from the previous state
 *
 * @param port USB-PD port index corresponding to the status update.
 *
 * @return None.
 */
bool tr_is_hpd_change(uint8_t port);

/**
 * @brief Register a callback for notification of data control register changes.
 *
 * @param port PD port index.
 * @param cb Pointer to callback function.
 * @return None
 */
void ridge_set_ctrl_change_cb(uint8_t port, ridge_ctrl_change_cb_t cb);

#endif /*_INTEL_RIDGE_H_ */

/* [] END OF FILE */
