/**
 * @file usb_i2cm.h
 *
 * @brief @{USB I2C master bridge interface header file.@}
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

#ifndef _USB_I2CM_H_
#define _USB_I2CM_H_

#include "stdint.h"
#include "stdbool.h"
#include "status.h"
#include "usb.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/**
 * @brief USB I2CM bridge BULK OUT endpoint.
 */
#define USB_I2CM_OUT_EP_INDEX           (USB_EP_INDEX_EP2)

/**
 * @brief USB I2CM bridge BULK IN endpoint.
 */
#define USB_I2CM_IN_EP_INDEX            (USB_EP_INDEX_EP3)

/**
 * @brief USB I2CM bridge interrupt IN endpoint.
 */
#define USB_I2CM_INT_EP_INDEX           (USB_EP_INDEX_EP4)

/**
 * @brief USB I2CM bridge sub-class code for I2C interface.
 */
#define USB_I2CM_SUB_CLASS              (0x03)

/**
 * @brief USB-I2CM SCB block index.
 */
#define USB_I2CM_SCB_INDEX              (3)

/**
 * @brief USB-I2CM control flags.
 */
#define USB_I2CM_CTRL_STOP_POS          (0)
#define USB_I2CM_CTRL_STOP              (1 << USB_I2CM_CTRL_STOP_POS)
#define USB_I2CM_CTRL_NAK_LAST_POS      (1)
#define USB_I2CM_CTRL_NAK_LAST          (1 << USB_I2CM_CTRL_NAK_LAST_POS)
#define USB_I2CM_CTRL_FLAGS             (0xFF)

/**
 * @brief USB-I2CM status flags.
 */
#define USB_I2CM_STAT_ACTIVE_POS        (0)
#define USB_I2CM_STAT_ACTIVE            (1 << USB_I2CM_STAT_ACTIVE_POS)
#define USB_I2CM_STAT_BUS_BUSY_POS      (1)
#define USB_I2CM_STAT_BUS_BUSY          (1 << USB_I2CM_STAT_BUS_BUSY_POS)
#define USB_I2CM_STAT_TX_UFLOW_POS      (3)
#define USB_I2CM_STAT_TX_UFLOW          (1 << USB_I2CM_STAT_TX_UFLOW_POS)
#define USB_I2CM_STAT_ARB_ERR_POS       (4)
#define USB_I2CM_STAT_ARB_ERR           (1 << USB_I2CM_STAT_ARB_ERR_POS)
#define USB_I2CM_STAT_NAK_POS           (5)
#define USB_I2CM_STAT_NAK               (1 << USB_I2CM_STAT_NAK_POS)
#define USB_I2CM_STAT_BUS_ERR_POS       (6)
#define USB_I2CM_STAT_BUS_ERR           (1 << USB_I2CM_STAT_BUS_ERR_POS)

#define USB_I2CM_STAT_SIZE              (3)

/**
 * @brief USB-I2CM internal scratch buffer size. This should be more than the
 * maximum data that can be loaded into the FIFO or USB EP at a time. For the
 * current design 64 bytes is sufficient.
 */
#define USB_I2CM_BUF_SIZE               (64)

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/

/**
 * @brief Internal USB-I2CM vendor commands enumeration.
 */
typedef enum
{
    USB_I2CM_VDR_RQT_I2C_WRITE = 0xC6, /**< I2C write request. The data is provided
                                         over the bulk out endpoint.
                                         CMD_CODE: 0x40
                                         VALUE:
                                            bit0 - start,
                                            bit1 - stop,
                                            bit3 - start on idle,
                                            bits[14:8] - slave addr
                                         INDEX: I2C write length.
                                         LENGTH: 0.
                                         EP0 DATA: None.
                                         OUT EP DATA: I2C data to be written. */                 
    USB_I2CM_VDR_RQT_I2C_READ,          /**< I2C read request. The data is provided
                                          over the bulk in endpoint.
                                          CMD_CODE: 0x40
                                          VALUE:
                                              bit0 - start,
                                              bit1 - stop,
                                              bit2 - NAK last byte,
                                              bit3 - start on idle,
                                              bits[14:8] - slave addr
                                          INDEX: I2C read length.
                                          LENGTH: 0.
                                          EP0 DATA: None.
                                          IN EP DATA: I2C read data from device. */
    USB_I2CM_VDR_RQT_I2C_GET_STATUS,    /**< I2C status request. The current status
                                          of the I2C transaction.
                                          CMD_CODE: 0xC0
                                          VALUE:
                                              bit0 - 0: TX 1: RX
                                          INDEX: 0.
                                          LENGTH: 3.
                                          EP0 DATA: IN -
                                              byte0:
                                                  bit0 - flag,
                                                  bit1 - bus_state,
                                                  bit2 - SDA state,
                                                  bit3 - TX underflow, 
                                                  bit4 - arbitration err,  
                                                  bit5 - NAK,
                                                  bit6 - bus error
                                              byte(2:1): Data count remaining. */
    USB_I2CM_VDR_RQT_I2C_RESET          /**< I2C block abort / reset request.
                                          CMD_CODE: 0x40
                                          VALUE:
                                              bit0 - 0: TX 1: RX
                                          INDEX: 0.
                                          LENGTH: 0.
                                          EP0 DATA: None. */

} usb_i2cm_vdr_rqt_t;

/**
 * @brief Internal USB-I2CM state enumeration.
 */
typedef enum
{
    USB_I2CM_STATE_DISABLED = 0,        /* Disabled state. */
    USB_I2CM_STATE_IDLE,                /* Interface enabled but not active. */
    USB_I2CM_STATE_WRITE,               /* Active and in write stage. */
    USB_I2CM_STATE_READ,                /* Active and in read stage. */
    USB_I2CM_STATE_ERROR                /* Error state. Need reset. */

}usb_i2cm_state_t;

/**
 * @brief Internal USB-I2CM module handle structure.
 *
 * No explicit structure for the handle is expected to be created outside of
 * the USB-I2CM module implementation.
 */
typedef struct
{
    usb_i2cm_state_t state;             /**< State of the bridge interface. */
    bool preamble_state;                /**< Indicates whether the current request
                                          is in preamble state. */
    uint8_t preamble;                   /**< Preamble to be used for the current
                                          transaction. */
    uint8_t ctrl;                       /**< Control flags for the current transaction. */
    uint8_t status;                     /**< Status of the current transaction. */
    uint16_t count;                     /**< Pending count of the current transaction. */
    uint16_t length;                    /**< Length of the current transaction. */

}usb_i2cm_t;

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief Handler for the USB-I2CM bridge GET_DESCRIPTOR request handler.
 *
 * The function is expected to be called by the USB module or the solution 
 * USB request handler. There should not be any explicit call to this function.
 *
 * @param pkt USB setup packet received for the request
 *
 * @return Status of the call.
 */
ccg_status_t usb_i2cm_get_dscr_rqt_handler(usb_setup_pkt_t *pkt);

/**
 * @brief Handler for the USB-I2CM bridge vendor control requests.
 *
 * The function is expected to be called by the USB module or the solution 
 * USB vendor request handler. There should not be any explicit call to this
 * function.
 *
 * @param pkt USB setup packet received for the request
 *
 * @return Status of the call.
 */
ccg_status_t usb_i2cm_vendor_rqt_handler(usb_setup_pkt_t *pkt);

/**
 * @brief The function enables / disables the USB-I2CM bridge mode.
 *
 * The function is expected to be called by the billboard module to enable / 
 * disable the bridge functionality. The function should be called when USB
 * host configures / un-configures the device.
 *
 * @return Status of the call.
 */
ccg_status_t usb_i2cm_inf_ctrl(bool enable);

/**
 * @brief Task handler for the bridge module.
 *
 * The function is expected to be called by the billboard module to run the 
 * bridge tasks. The function is expected to be called once enabled from the 
 * bb_task() function.
 *
 */
void usb_i2cm_task(void);

/**
 * @brief Get a buffer for USB EP0 request handling.
 *
 * The function is expected to be implemented by the application. This allows a
 * common buffer to be re-used for all EP0 transactions. It is expected that the
 * buffer is always available through out the USB vendor command transaction.
 *
 * The function should always return a valid pointer with sufficient buffering
 * for 64 bytes.
 *
 * @return Pointer to the buffer
 */
uint8_t *usb_i2cm_get_ep0_buffer(void);

/**
 * @brief Return whether the USB I2CM bridge is idle or not.
 *
 * @return Bool: True if idle and false if not.
 */
bool usb_i2cm_is_idle(void);

#endif /* _USB_I2CM_H_ */

/*[]*/

