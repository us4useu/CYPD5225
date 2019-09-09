/**
 * @file usbconst.h
 *
 * @brief @{Constants defined in the USB specification.@}
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

#ifndef _USBCONST_H_
#define _USBCONST_H_

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/** USB Setup Commands Recipient **/

/* The Target mask */
#define USB_TARGET_MASK                 (0x03)
/* The USB Target Device */
#define USB_TARGET_DEVICE               (0x00)
/* The USB Target Interface */
#define USB_TARGET_INTF                 (0x01)
/* The USB Target Endpoint */
#define USB_TARGET_ENDPT                (0x02)
/* The USB Target Other */
#define USB_TARGET_OTHER                (0x03)

/* The standard device request, GET_STATUS device */
#define USB_GS_DEVICE                   (0x80)
/* The standard device request, GET_STATUS interface */
#define USB_GS_INTERFACE                (0x81)
/* The standard device request, GET_STATUS endpoint */
#define USB_GS_ENDPOINT                 (0x82)

/* Direction bit in setup packet. */
#define USB_SETUP_PKT_DIRECTION         (0x80)
/* The request type mask */
#define USB_TYPE_MASK                   (0x60)
/* The USB standard request */
#define USB_STANDARD_RQT                (0x00)
/* The USB class request */
#define USB_CLASS_RQT                   (0x20)
/* The USB vendor request */
#define USB_VENDOR_RQT                  (0x40)
/* The USB reserved request */
#define USB_RESERVED_RQT                (0x60)

/* USB device class */
#define USB_CLASS_HID                   (0x03)
#define USB_CLASS_BILLBOARD             (0x11)
#define USB_CLASS_TYPEC_BRIDGE          (0x12)
#define USB_CLASS_VENDOR                (0xFF)

/* Maximum address that can be assigned. */
#define USB_MAX_DEVICE_ADDR             (127)

/* Number of CRC bytes per packet. */
#define USB_CRC_SIZE                    (2)
/* Size of the setup packet. */
#define USB_SETUP_PKT_SIZE              (8)
/* Size of the USB device descriptor. */
#define USB_DEVICE_DSCR_SIZE            (18)
/* Size of the USB configuration descriptor. */
#define USB_CONFIG_DSCR_SIZE            (9)
/* Size of the USB interface descriptor. */
#define USB_INF_DSCR_SIZE               (9)
/* Size of the USB HID interface descriptor. */
#define USB_HID_DSCR_SIZE               (9)
/* Size of the USB HID interface descriptor. */
#define USB_EP_DSCR_SIZE                (7)
/* Size of the USB BOS descriptor. */
#define USB_BOS_DSCR_SIZE               (5)
/* Size of the USB BOS USB 2.0 extension descriptor. */
#define USB_BOS_USB2_EXTN_DSCR_SIZE     (7)
/* Size of the USB BOS container ID descriptor. */
#define USB_BOS_CONTAINER_ID_DSCR_SIZE  (20)
/* Size of the alternate mode capability descriptor. */
#define USB_ALT_MODE_CAP_DSCR_SIZE      (8)

/* VID field offset in the device descriptor. */
#define USB_DEVICE_DSCR_VID_OFFSET      (8)
/* PID field offset in the device descriptor. */
#define USB_DEVICE_DSCR_PID_OFFSET      (10)
/* bcdDevice field offset in the device descriptor. */
#define USB_DEVICE_DSCR_BCD_DEV_OFFSET  (12)
/* Serial string index field offset in the device descriptor. */
#define USB_DEVICE_DSCR_SERIAL_OFFSET   (16)

/* Number of interfaces field offset in the configuration descriptor. */
#define USB_CONFIG_DSCR_NUM_INF_OFFSET  (4)
/* Attribute field offset in the configuration descriptor. */
#define USB_CONFIG_DSCR_ATTRIB_OFFSET   (7)
/* bMaxPower field offset in the configuration descriptor. */
#define USB_CONFIG_DSCR_MAXPOWER_OFFSET (8)
/* Flag indicating self powered mode in the configuration descriptor. */
#define USB_CONFIG_DSCR_SELF_POWERED    (0x40)

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/

/**
 * @brief Standard device request codes.
 *
 * These are the various standard requests received from the USB host.
 * The device is expected to respond to all these requests.
 */
typedef enum
{
    USB_SC_GET_STATUS = 0x00,     /**< Get status request. */
    USB_SC_CLEAR_FEATURE,         /**< Clear feature. */
    USB_SC_RESERVED,              /**< Reserved command. */
    USB_SC_SET_FEATURE,           /**< Set feature. */
    USB_SC_SET_ADDRESS = 0x05,    /**< Set address. */
    USB_SC_GET_DESCRIPTOR,        /**< Get descriptor. */
    USB_SC_SET_DESCRIPTOR,        /**< Set descriptor. */
    USB_SC_GET_CONFIGURATION,     /**< Get configuration. */
    USB_SC_SET_CONFIGURATION,     /**< Set configuration. */
    USB_SC_GET_INTERFACE,         /**< Get interface (alternate setting). */
    USB_SC_SET_INTERFACE,         /**< Set interface (alternate setting). */
    USB_SC_SYNC_FRAME,            /**< Synch frame. */
    USB_SC_SET_SEL = 0x30,        /**< Set system exit latency. */
    USB_SC_SET_ISOC_DELAY         /**< Set isochronous delay. */

} usb_setup_cmd_t;

/**
 * @brief Enumeration of the endpoint types.
 *
 * There are four types of endpoints. This defines the behaviour of
 * the endpoint. The control endpoint is a compulsory for any device
 * whereas the other endpoints are used as per requirement.
 */
typedef enum
{
    USB_EP_CONTROL = 0,           /**< Control Endpoint Type */
    USB_EP_ISO = 1,               /**< Isochronous Endpoint Type */
    USB_EP_BULK = 2,              /**< Bulk Endpoint Type */
    USB_EP_INTR = 3               /**< Interrupt Endpoint Type */

} usb_ep_type_t;

/**
 * @brief Enumeration of descriptor types.
 *
 * A USB device is identified by the descriptors provided. The
 * following are the standard defined descriptors.
 */
typedef enum
{
    USB_DEVICE_DSCR = 0x01,       /**< Super speed device descriptor  */
    USB_CONFIG_DSCR,              /**< Configuration descriptor */
    USB_STRING_DSCR,              /**< String descriptor */
    USB_INF_DSCR,                 /**< Interface descriptor */
    USB_EP_DSCR,                  /**< Endpoint descriptor */
    USB_DEVQUAL_DSCR,             /**< Device qualifier descriptor */
    USB_OTHERSPEED_DSCR,          /**< Other speed configuration descriptor */
    USB_INF_POWER_DSCR,           /**< Interface power descriptor */
    USB_BOS_DSCR = 0x0F,          /**< BOS descriptor */
    USB_DEVICE_CAPB_DSCR,         /**< Device capability descriptor */
    USB_HID_DSCR = 0x21,          /**< HID descriptor */
    USB_REPORT_DSCR               /**< Report descriptor */

} usb_dscr_type_t;

/**
 * @brief Enumeration of device capability types.
 *
 * The BOS descriptor supports various device capability descriptors.
 * The enumeration contains the various types for capability descriptors.
 */
typedef enum
{
    USB_CAP_TYPE_WIRELESS = 0x01,
                                /**< Wireless capability descriptor */
    USB_CAP_TYPE_USB2_EXTN,
                                /**< USB 2.0 extension descriptor */
    USB_CAP_TYPE_SS_USB,        /**< USB super-speed capability descriptor */
    USB_CAP_TYPE_CONTAINER_ID,  /**< Container ID capability descriptor */
    USB_CAP_TYPE_BILLBOARD = 0x0D
                                /**< Billboard capability descriptor */

} usb_dev_cap_type_t;

/**
 * @brief List of USB feature selector codes.
 *
 * The following are the various features that can be selected
 * using the SetFeature request or cleared using ClearFeature
 * setup request. Refer to the USB specification for more
 * information.
 */
typedef enum
{
    USB_EP_HALT     = 0,        /**< USB Endpoint HALT feature. Sets or clears EP stall.  */
    USB_REMOTE_WAKE = 1,        /**< USB 2.0 Remote Wakeup. */
    USB_TEST_MODE   = 2         /**< USB 2.0 Test mode. */

} usb_feature_select_t;

/**
 * @brief List of USB HID class request codes as per USB HID specification v 1.11.
 */
typedef enum
{
    USB_HID_RQT_GET_REPORT = 0x01,      /* USB HID: Get report request code. */
    USB_HID_RQT_GET_IDLE = 0x02,        /* USB HID: Get idle request code. */
    USB_HID_RQT_GET_PROTOCOL = 0x03,    /* USB HID: Get protocol request code. */
    USB_HID_RQT_SET_REPORT = 0x09,      /* USB HID: Set report request code. */
    USB_HID_RQT_SET_IDLE = 0x0A,        /* USB HID: Set idle request code. */
    USB_HID_RQT_SET_PROTOCOL = 0x0B     /* USB HID: Set protocol request code. */

} usb_hid_rqt_t;

/**
 * @brief USB setup packet format.
 *
 * The structure breaks the setup packet into
 * various fields defined in the USB specification.
 */
typedef struct __attribute__((__packed__))
{
    uint8_t attrib;             /**< Request attributes:
                                     Bit7:   0=Host to device, 1=Device to host
                                     Bits6:5 0=Standard, 1=Class, 2=Vendor,
                                             3=Reserved
                                     Bits4:0 0=Device, 1=Interface, 2=Endpoint,
                                             3=Other, 4-31=Reserved */
    uint8_t cmd;                /**< Request command */
    uint16_t value;             /**< Value parameter for command */
    uint16_t index;             /**< Index parameter for command */
    uint16_t length;            /**< Number of bytes to be transferred in data phase */

} usb_setup_pkt_t;

#endif /* _USBCONST_H_ */

/*[]*/

