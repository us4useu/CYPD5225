/**
 * @file usb_hid.h
 *
 * @brief @{USB HID class interface (flashing) header file.@}
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

#ifndef _USB_HID_H_
#define _USB_HID_H_

#include "stdint.h"
#include "stdbool.h"
#include "status.h"
#include "usb.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/**
 * @brief Size of the version information.
 */
#define FW_VERSION_SIZE                 (8)

/**
 * @brief Size of the firmware information report.
 */
#define HID_CY_FW_INFO_SIZE             (63)

/**
 * @brief Size of the firmware information report.
 */
#define HID_CUSTOMER_INFO_SIZE          (32)

/**
 * @brief Report ID mask for GET_REPORT and SET_REPORT requests.
 */
#define HID_GET_SET_REPORT_ID_MASK      (0xFF)

/**
 * @brief Endpoint index for the HID interrupt IN endpoint.
 */
#define USB_HID_INTR_EP_INDEX           (USB_EP_INDEX_EP1)

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/

/**
 * @brief HID vendor request / response report IDs.
 *
 * The reports are aligned to accomodate the report ID as the first byte.
 * The report size does not include this first byte added as part of the
 * protocol. The byte information for each report ID includes this first 
 * byte.
 */
typedef enum
{
    HID_REPORT_ID_CY_FW_INFO = 0xE0,
                                /**< CY_FW_INFO data report. The report returns
                                 * information about the device and firmware.
                                 * Report direction: IN, report size: 63.
                                 * BYTE[0]    : 0xE0
                                 * BYTE[1]    : Reserved
                                 * BYTE[3:2]  :Signature "CY"
                                 * BYTE[4]    : Current operating mode.
                                 *      BIT(1:0) - 0 = Bootloader
                                 *                 1 = FW image 1
                                 *                 2 = FW image 2
                                 * BYTE[5]    : Bootloader information.
                                 *      BIT(0)  - This bit is set if the boot-loader
                                 *      supports security (SHA2 checksum at boot).
                                 *      BIT(1)  - This bit is set if the boot-loader
                                 *      has no flashing interface.
                                 *      BIT(2)  - This bit is set if the boot-loader
                                 *      supports application priority feature.
                                 *      BIT(4:3) - Flash row size information
                                 *              0 = Row size of 128 bytes
                                 *              1 = Row size of 256 bytes
                                 * BYTE[6]    : Boot mode reason
                                 *      BIT(0)  - This bit is set if the firmware
                                 *      requested a jump to boot-loader
                                 *      BIT(1)  - Reserved
                                 *      BIT(2)  - FW image 1 status. Set if invalid.
                                 *      BIT(3)  - FW image 2 status. Set if invalid.
                                 *      BIT(5:4) - Application priority setting
                                 *              0 = Default priority - most recent image.
                                 *              1 = Image1 higher priority.
                                 *              2 = Image2 higher priority.
                                 * BYTE[7]    : Reserved
                                 * BYTE[11:8] : Silicon ID
                                 * BYTE[19:12]: Bootloader version
                                 * BYTE[27:20]: FW image 1 version
                                 * BYTE[35:28]: FW image 2 version
                                 * BYTE[39:36]: FW image 1 start address
                                 * BYTE[43:40]: FW image 2 start address
                                 * BYTE[51:44]: Device UID
                                 * BYTE[63:52]: Reserved */
    HID_REPORT_ID_RQT,
                                /**< HID vendor command report.
                                 * Report direction: OUT, report size: 7.
                                 * BYTE[0]    : 0xE1
                                 * BYTE[1]    : Request CMD
                                 * BYTE[7:2]  : Command parameters. */
    HID_REPORT_ID_FLASH_WRITE,  /**< Flash write command report.
                                 * Report direction: OUT, report size: 131.
                                 * BYTE[0]    : 0xE2
                                 * BYTE[1]    : "F"
                                 * BYTE[3:2]  : Row ID to write data to.
                                 * BYTE[131:4]: Data to write. */
    HID_REPORT_ID_FLASH_READ,   /**< Flash read command report.
                                 * Report direction: IN, report size: 131.
                                 * BYTE[0]    : 0xE3
                                 * BYTE[1]    : "F"
                                 * BYTE[3:2]  : Row ID of the data.
                                 * BYTE[131:4]: Data read from flash. */
    HID_REPORT_ID_CUSTOMER_INFO /**< Customer information data report.
                                 * Report direction: IN, report size: 32.
                                 * BYTE[0]    : 0xE4
                                 * BYTE[32:1] : Customer information data. */

} hid_report_id_t;

/**
 * @brief HID vendor request commands for HID_REPORT_ID_RQT_CMD report ID.
 */
typedef enum
{
    HID_RQT_CMD_RESERVED = 0,   /**< Reserved command ID. */
    HID_RQT_CMD_JUMP,           /**< Jump request.
                                  * PARAM[0]  : Signature
                                  *     'J' = Jump to boot-loader.
                                  *     'R' = Reset device.
                                  *     'A' = Jump to alternate image.
                                  * PARAM[5:1]: Reserved. */
    HID_RQT_CMD_ENTER_FLASHING, /**< Enter flashing mode request.
                                  * PARAM[0]  : Signature
                                  *     'P' = Enable flashing mode.
                                  *     Others = Disable flashing mode.
                                  * PARAM[5:1]: Reserved. */
    HID_RQT_CMD_SET_READ_ROW,   /**< Set flash read row request.
                                  * PARAM[1:0]: Row ID
                                  * PARAM[5:2]: Reserved. */
    HID_RQT_CMD_VALIDATE_FW,    /**< Validate firmware request.
                                  * PARAM[0]  : Firmware image number to validate.
                                  * PARAM[5:1]: Reserved. */
    HID_RQT_CMD_SET_APP_PRIORITY,
                                /**< Set application priority setting.
                                  * PARAM[0]  : Signature 'F'
                                  * PARAM[1]  : Priorty setting (0, 1 or 2).
                                  * PARAM[5:2]: Reserved. */
    HID_RQT_CMD_I2C_BRIDGE_CTRL
                                /**< Control the I2C master bridge mode.
                                  * PARAM[0]  : Signature
                                  *     'B' = Enable bridge mode, else disable
                                  *           bridge mode.
                                  * PARAM[5:1]: Reserved. */

}  hid_rqt_cmd_t;

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief Returns the HID interface and endpoint descriptors.
 *
 * The function is expected to be invoked by the global descriptor request
 * handler and returns the HID interface descriptor and the corresponding
 * endpoint descriptor. This can be placed directly into the configuration
 * descriptor.
 *
 * @param buffer Buffer pointer to copy the descriptor into. It is the 
 *      responsibility of the caller to ensure that sufficient buffering
 *      is available. The HID interface descriptor along with the endpoint
 *      descriptor(s) is always expected to be less than 64 bytes.
 *
 * @param max_length Maximum buffering available
 *
 * @param inf_num Interface number to be used for creating the HID inferface
 *      descriptor.
 *
 * @param inf_str_index String descriptor index to be used when creating the
 *      interface descriptor.
 *
 * @return Actual size of the interface descriptor copied.
 */
uint16_t usb_hid_get_inf_dscr(uint8_t *buffer, uint16_t max_length,
        uint8_t inf_num, uint8_t inf_str_index);

/**
 * @brief Returns the HID report descriptor.
 *
 * The function is expected to be invoked by the global descriptor request
 * handler and returns the HID report descriptor.
 *
 * @param buffer Buffer pointer to copy the descriptor into. It is the 
 *      responsibility of the caller to ensure that sufficient buffering
 *      is available.
 *
 * @param max_length Maximum buffering available
 *
 * @return Actual size of the descriptor copied.
 */
uint16_t usb_hid_get_report_dscr(uint8_t *buffer, uint16_t max_length);

/**
 * @brief USB HID-class request handler for USB flashing requests.
 *
 * The function provides the default HID-class command handler for flashing.
 * This function is expected to be invoked when a USB vendor command is
 * received from the USB module. If there are no additional vendor commands
 * supported by the application, then this function can be directly registered
 * for handling vendor commands with USB module.
 *
 * @param pkt USB setup packet received for the request
 *
 * @return Status of the call
 */
ccg_status_t usb_hid_class_rqt_handler(usb_setup_pkt_t *pkt);

/**
 * @brief USB HID interface control function.
 *
 * The function enables / disables the corresponding interface. The function
 * is expected to be invoked from the global event handler during
 * SET_CONFIGUATION and DISCONNECT / RESET events. The function enables / 
 * disables the interrupt endpoint for the HID interface.
 *
 * @param enable Indicates whether to enable or disable the HID interface.
 *
 * @return Status of the call
 */
ccg_status_t usb_hid_inf_ctrl(bool enable);

/**
 * @brief Get a buffer for USB EP0 request handling.
 *
 * The function is expected to be implemented by the application. This allows a
 * common buffer to be re-used for all EP0 transactions. It is expected that the
 * buffer is always available through out the USB vendor command transaction.
 *
 * The function should always return a valid pointer with sufficient buffering
 * for a single flash row.
 *
 * @return Pointer to the buffer
 */
uint8_t *usb_hid_get_ep0_buffer(void);

/**
 * @brief Enter flashing request handler for the solution.
 *
 * The function is expected to be implemented by the application. This allows
 * the application to control the device behaviour as well as verify if 
 * flashing can be done at the specified time. If a failure is returned, the
 * flashing request from the host shall be rejected. The expectation is that
 * while in flashing the application does not use the USB module for any other
 * purpose and the USB interface is left ON.
 *
 * @return Status of the call
 */
ccg_status_t usb_hid_flashing_rqt(bool enable);

/**
 * @brief Enter I2C master bridge mode for the solution.
 *
 * The function is expected to be implemented by the application. This allows 
 * the application to enable / disable the bridge mode enumeration If a failure
 * is returned, the bridge mode request from the host shall be rejected.
 *
 * @param enable True to enable and false to disable.
 *
 * @return Status of the call
 */
ccg_status_t usb_i2cm_bridge_ctrl(bool enable);

/**
 * @brief Update the firmware locations reported in the HID reports. This function
 * can be used by solution layer code to update the default firmware locations
 * reported by the USB-HID based firmware update code.
 *
 * @param bl_last_row Last flash row which is part of the boot-loader.
 * @param fw1_last_row Last flash row which is part of the FW1 binary.
 * @return None
 */
void usb_hid_set_fw_locations(uint16_t bl_last_row, uint16_t fw1_last_row);

#endif /* _USB_HID_H_ */

/*[]*/

