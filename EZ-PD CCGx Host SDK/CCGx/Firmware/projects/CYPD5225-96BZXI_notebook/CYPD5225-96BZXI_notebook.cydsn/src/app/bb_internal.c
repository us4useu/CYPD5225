/**
 * @file bb_internal.c
 *
 * @brief @{Internal Billboard control interface for CCG3.@}
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

#include "stdint.h"
#include "stdbool.h"
#include "config.h"
#include "flash_config.h"
#include "ccgx_regs.h"
#include "status.h"
#include "usb.h"
#include "utils.h"
#include "timer.h"
#include "system.h"
#include "boot.h"
#include "billboard.h"
#include "flash.h"
#include "pd.h"
#include "app.h"

#if (FLASHING_MODE_USB_ENABLE != 0)
#include "usb_hid.h"
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */

#if (APP_I2CM_BRIDGE_ENABLE)
#include "usb_i2cm.h"
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

#if (DP_GPIO_CONFIG_SELECT)
#include "dp_sid.h"
#endif /* (DP_GPIO_CONFIG_SELECT) */

#if (CCG_BB_ENABLE != 0)

/*
 * Bit field in bb_option from configuration table which indicates
 * that the device needs to enable HID interface along with billboard.
 */
#define BB_OPTION_HID_ENABLE            (0x01)

/* Number of alternate modes field offset in the BOS descriptor. */
#define USB_BOS_DSCR_NUM_ALT_MODE_OFFET (36)
/* Offset for alternate mode status field offset in the BOS descriptor. */
#define USB_BOS_DSCR_ALT_STATUS_OFFSET  (40)
/* Offset for alternate mode bAdditionalFailureInfo field in the BOS descriptor. */
#define USB_BOS_DSCR_ADD_INFO_OFFSET    (74)
/* Offset for alternate mode0 information offset in the BOS descriptor. */
#define USB_BOS_DSCR_MODE0_INFO_OFFSET  (76)

/* Discover ID response minimum size. */
#define VDM_DID_MIN_SIZE                (20)
/* Discover ID response ID header VDO offset. */
#define VDM_RESP_ID_HEADER_OFFSET       (8)

/* Device descriptor support. */
const static uint8_t gl_device_dscr[] =
{
    USB_DEVICE_DSCR_SIZE,       /* Descriptor size. */
    USB_DEVICE_DSCR,            /* Descriptor type. */
    0x01, 0x02,                 /* USB 2.01. */
    0x00,                       /* Device class - defined at interface. */
    0x00,                       /* Device sub-class - defined at interface. */
    0x00,                       /* Device protocol - defined at interface. */
    0x08,                       /* EP0 max packet size (8 bytes) - hardware limited. */
    0xB4, 0x04,                 /* VID. */
    0x48, 0xF6,                 /* PID = Flashing mode. */
    0x00, 0x00,                 /* Device release number. */
    0x01,                       /* Manufacturer string index. */
    0x02,                       /* Product string index. */
    0x00,                       /* Serial number string index. */
    0x01                        /* Number of configurations. */
};

/* Configuration descriptor support. */
const static uint8_t gl_config_dscr[] =
{
    /* Configuration descriptor. */
    USB_CONFIG_DSCR_SIZE,       /* Descriptor size. */
    USB_CONFIG_DSCR,            /* Descriptor type. */
    USB_CONFIG_DSCR_SIZE, 0x00, /* Total length.
                                   NOTE: Need to calculate based on number of interfaces added. */
    0x00,                       /* Number of interfaces.
                                   NOTE: Need to load the correct number of interfaces. */
    0x01,                       /* Configuration number. */
    BB_CONFIG_STRING_INDEX,     /* Configuration string index. */
    0x80,                       /* Configuration characteristics: Bus powered.
                                   Need to load the correct setting. */
    0x32,                       /* Max power consumption in 2mA unit: 100mA. */
};

/* Billboard interface descriptor. */
const static uint8_t gl_bb_inf_dscr[] =
{
    /* Interface descriptor */
    USB_INF_DSCR_SIZE,          /* Descriptor size */
    USB_INF_DSCR,               /* Interface Descriptor type */
    0x00,                       /* Interface number. Billboard device always comes up as first interface */
    0x00,                       /* Alternate setting number */
    0x00,                       /* Number of endpoints */
    USB_CLASS_BILLBOARD,        /* Interface class */
    0x00,                       /* Interface sub class */
    0x00,                       /* Interface protocol code */
    BB_BB_INF_STRING_INDEX      /* Interface descriptor string index */
};

/* Fallback BOS descriptor. Not expected to be used. */
const static uint8_t gl_bos_dscr[] =
{
    USB_BOS_DSCR_SIZE,          /* Descriptor size */
    USB_BOS_DSCR,               /* Device descriptor type */
    0x4C,0x00,                  /* Length of this descriptor and all sub descriptors:
                                   NOTE: Need to calculate based on number of alternate modes */
    0x03,                       /* Number of device capability descriptors:
                                   USB 2.0 ext, Container ID and Billboard Capability descriptor */

    /* USB 2.0 extension */
    USB_BOS_USB2_EXTN_DSCR_SIZE,/* Descriptor size */
    USB_DEVICE_CAPB_DSCR,       /* Device capability type descriptor */
    USB_CAP_TYPE_USB2_EXTN,     /* USB 2.0 extension capability type */
    0x00,0x00,0x00,0x00,        /* Supported device level features: LPM support  */

    /* Container ID Descriptor */
    USB_BOS_CONTAINER_ID_DSCR_SIZE,
    /* Descriptor size */
    USB_DEVICE_CAPB_DSCR,       /* Device capability type descriptor */
    USB_CAP_TYPE_CONTAINER_ID,  /* ContainerID capability type */
    0x00,                       /* Reserved */
    0x00,0x00,0x00,0x00,        /* 128-bit Container ID. Note: It will be updated based on config */
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,

    /* Billboard capability descriptor */
    0x2C,                       /* Descriptor size.
                                   NOTE: Need to calculate based on number of alternate modes */
    USB_DEVICE_CAPB_DSCR,       /* Device capability type descriptor */
    USB_CAP_TYPE_BILLBOARD,     /* Billboard capability type */
    0x00,                       /* iAdditionalInfoURL string descriptor index */
    0x00,                       /* Number of alternate modes supported. */
    0x00,                       /* Preferred alternate mode index. */
    0x00, 0x00,                 /* VCONN power - 1W and VCONN is required. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                /* bmConfigured - Set to Unspecified error. */
    0x10, 0x01,                 /* bcdVersion = 0x0110 */
    0x00,                       /* bAdditonalFailureInfo */
    0x00,                       /* bReserved */
};

/* The string descriptor usage is fixed.
 * 0 - Language ID
 * 1 - Manufacturer
 * 2 - Product
 * 3 - Serial string if available
 * 4 - Configuration
 * 5 - Billboard interface
 * 6 - HID interface
 * 7 - Additional Info URL
 * 8:15 - Alternate mode strings.
 */
#define USB_NUM_STRING_DSCR     (16)

/* Size of the unique device information */
#define CY_UNIQUE_NUMBER_SIZE   (8)

/* Language ID string descriptor */
const static uint8_t gl_lang_id_dscr[] =
{
    0x04,                       /* Descriptor size. */
    USB_STRING_DSCR,            /* Descriptor type. */
    0x09, 0x04                  /* Supported language ID. */
};

#if (NO_OF_TYPEC_PORTS > 1)
#error "Device does not support two billboard ports."
#endif

#ifndef CCG3
#error "Device not supported"
#endif

/* Billboard interface internal handle structure */
static volatile bb_handle_t gl_bb[NO_OF_TYPEC_PORTS];

/* EP0 buffering for billboard implementation */
static volatile uint8_t gl_ep0_buffer[NO_OF_TYPEC_PORTS][BB_MAX_EP0_XFER_SIZE];

/* TODO: Temporary hack to ensure that port variable is available everywhere. */
static const uint8_t port = 0;

/* Internal function which takes a nibble and converts to ASCII. The caller
 * should ensure that only HEX value is being provided. */
static uint8_t hex_to_ascii(uint8_t nibble)
{
    if (nibble < 10)
    {
        return (0x30 + nibble);
    }
    else
    {
        return (0x37 + nibble);
    }
}

/* Internal function to generate unique serial number string. */
static void generate_unique_serial_string(uint8_t *buffer)
{
    uint8_t i;
    uint8_t *ptr;
    uint32_t unique[CY_UNIQUE_NUMBER_SIZE/(sizeof(uint32_t))];

    CyGetUniqueId((unsigned long *)unique);
    /* Load the size information for the descriptor. The size of the descriptor
     * is number of bytes * 4 (to convert each nibble to unicode) + 2 (to include
     * size and descriptor type bytes) */
    buffer[0] = ((CY_UNIQUE_NUMBER_SIZE + 4) << 2) + 2;
    buffer[1] = USB_STRING_DSCR;

    /* Load the silicon Family ID */
    buffer[2] = hex_to_ascii(CCG_DEV_FAMILY_ID >> 12);
    buffer[3] = 0;
    buffer[4] = hex_to_ascii((CCG_DEV_FAMILY_ID >> 8) & 0xF);
    buffer[5] = 0;
    buffer[6] = hex_to_ascii((CCG_DEV_FAMILY_ID >> 4) & 0xF);
    buffer[7] = 0;
    buffer[8] = hex_to_ascii((CCG_DEV_FAMILY_ID >> 0) & 0xF);
    buffer[9] = 0;

    /* Load the silicon ID */
    buffer[10] = hex_to_ascii(CCG_DEV_SILICON_ID >> 12);
    buffer[11] = 0;
    buffer[12] = hex_to_ascii((CCG_DEV_SILICON_ID >> 8) & 0xF);
    buffer[13] = 0;
    buffer[14] = hex_to_ascii((CCG_DEV_SILICON_ID >> 4) & 0xF);
    buffer[15] = 0;
    buffer[16] = hex_to_ascii((CCG_DEV_SILICON_ID >> 0) & 0xF);
    buffer[17] = 0;

    ptr = (uint8_t *)unique;

    for (i = 18; i < buffer[0]; i += 4)
    {
        buffer[i] = hex_to_ascii(*ptr >> 4);
        buffer[i + 1] = 0;
        buffer[i + 2] = hex_to_ascii(*ptr & 0x0F);
        buffer[i + 3] = 0;
        ptr++;
    }
}

/* GET_DESCRIPTOR request handler function. */
static ccg_status_t usb_get_dscr_rqt_handler(usb_setup_pkt_t *pkt)
{
    uint8_t index, temp, dscr;
    uint8_t *ptr = NULL, *tmp_ptr;
    uint16_t length = 0;
    pd_port_config_t *port_cfg;
    bb_settings_t *bb_cfg;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    dscr = (pkt->value >> 8);
#if (APP_I2CM_BRIDGE_ENABLE)
    /*
     * If we are operating in bridge mode, use the function provided by
     * the bridge interface. If the request is for HID specific response
     * handle it here.
     */
    if ((gl_bb[port].usb_i2cm_mode != false) &&
            (dscr != USB_HID_DSCR) &&
            (dscr != USB_REPORT_DSCR))
    {
        return usb_i2cm_get_dscr_rqt_handler(pkt);
    }
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

    index = (pkt->value & 0xFF);
    port_cfg = (pd_port_config_t *)get_pd_port_config(port);
    bb_cfg   = pd_get_ptr_bb_tbl(port);

    switch (dscr)
    {
        case USB_DEVICE_DSCR:

            ptr = (uint8_t *)gl_bb[port].ep0_buffer;
            length = gl_device_dscr[0];
            memcpy(ptr, gl_device_dscr, length);

            /* Update descriptor based on configuration information. */
            if ((port_cfg->id_vdm_offset != 0) &&
                    (port_cfg->id_vdm_length >= VDM_DID_MIN_SIZE))
            {
                tmp_ptr = (uint8_t *)(get_pd_config()) + port_cfg->id_vdm_offset;
                /* Retreive the VID, PID, and bcdDevice information. */
                tmp_ptr += VDM_RESP_ID_HEADER_OFFSET;
                /* The VID is located in the LS 16 bits of the ID header. */
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_VID_OFFSET] = *tmp_ptr;
                tmp_ptr++;
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_VID_OFFSET + 1] = *tmp_ptr;
                tmp_ptr += 7;
                /* The bcdDevice is in the LS 16 bits of the Product VDO. */
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_BCD_DEV_OFFSET] = *tmp_ptr;
                tmp_ptr++;
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_BCD_DEV_OFFSET + 1] = *tmp_ptr;
                tmp_ptr++;
                /* The PID is in the MS 16 bits of the Product VDO. */
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_PID_OFFSET] = *tmp_ptr;
                tmp_ptr++;
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_PID_OFFSET + 1] = *tmp_ptr;
                tmp_ptr++;
            }

            /* Update the serial string descriptor. */
            if (bb_cfg->bb_string_dscr_offset[BB_SERIAL_STRING_INDEX - 1] != 0)
            {
                gl_bb[port].ep0_buffer[USB_DEVICE_DSCR_SERIAL_OFFSET] =
                    (BB_SERIAL_STRING_INDEX);
            }
            break;

        case USB_CONFIG_DSCR:
            length = 0;
            temp = 0;
            ptr = (uint8_t *)gl_bb[port].ep0_buffer;
            tmp_ptr = ptr;
            memcpy(tmp_ptr, gl_config_dscr, sizeof(gl_config_dscr));
            length += sizeof(gl_config_dscr);
            tmp_ptr += sizeof(gl_config_dscr);

            if (bb_cfg->bb_bus_power == 0)
            {
                ptr[USB_CONFIG_DSCR_ATTRIB_OFFSET] |= USB_CONFIG_DSCR_SELF_POWERED;
            }

            /* Add the billboard and flashing interface as required */
            {
                /* Billboard interface */
                temp++;
                memcpy(tmp_ptr, gl_bb_inf_dscr, sizeof(gl_bb_inf_dscr));
                length += sizeof(gl_bb_inf_dscr);
                tmp_ptr += sizeof(gl_bb_inf_dscr);
                /* Check and add the HID interface as required. */
#if (FLASHING_MODE_USB_ENABLE != 0)
                if (bb_cfg->bb_option & BB_OPTION_HID_ENABLE)
                {
                    uint16_t hid_length;

                    /*
                     * Here the assumption is that the HID descriptor
                     * is going to be valid and shall not overflow.
                     */
                    hid_length = usb_hid_get_inf_dscr(tmp_ptr, 
                            USB_EP_MAX_PKT_SIZE, temp, BB_HID_INF_STRING_INDEX);
                    length += hid_length;
                    tmp_ptr += hid_length;
                    temp++;
                }
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */
            }

            /* Now load the actual size and count to the response. */
            ptr[2] = WORD_GET_LSB(length);
            ptr[3] = WORD_GET_MSB(length);
            ptr[USB_CONFIG_DSCR_NUM_INF_OFFSET] = temp;
            break;

        case USB_BOS_DSCR:
            if (bb_cfg->bb_bos_dscr_offset != 0)
            {
                ptr = (uint8_t *)gl_bb[port].ep0_buffer;
                tmp_ptr = (uint8_t *)(get_pd_config()) + bb_cfg->bb_bos_dscr_offset;
                length = MAKE_WORD(tmp_ptr[3], tmp_ptr[2]);
                memcpy(ptr, tmp_ptr, length);

                /* TODO: Check and create the container ID descriptor if required */

                /*
                 * Update the mode response for DisplayPort alternate mode for
                 * DP dongle solution.
                 */
#if DP_GPIO_CONFIG_SELECT
                /* 
                 * We need to identify the mode index for DP SVID. This is a special
                 * case implementation and we expect the entry to be present.
                 */
                tmp_ptr = tmp_ptr + USB_BOS_DSCR_MODE0_INFO_OFFSET;
                for (temp = 0; temp < gl_bb[port].num_alt_modes; temp++)
                {
                    if (MAKE_WORD(tmp_ptr[temp * 4 + 1], tmp_ptr[temp * 4]) == DP_SVID)
                    {
                        /* Locate the alternate mode capability descriptor for DP and update it. */
                        tmp_ptr     = (uint8_t *)(ptr + length);
                        tmp_ptr    -= ((gl_bb[port].num_alt_modes - temp) * USB_ALT_MODE_CAP_DSCR_SIZE);
                        tmp_ptr[5]  = dp_sink_get_pin_config();
                    }
                }
#endif /* DP_GPIO_CONFIG_SELECT */

                /* Update the status of the alternate modes. */
                memcpy((uint8_t *)(&ptr[USB_BOS_DSCR_ALT_STATUS_OFFSET]),
                        (uint8_t *)(&gl_bb[port].alt_status),
                        sizeof(gl_bb[port].alt_status));

                /* Update the additional failure info data. */
                *((uint8_t *)(&ptr[USB_BOS_DSCR_ADD_INFO_OFFSET])) =
                    gl_bb[port].bb_add_info;
            }
            else
            {
                ptr = (uint8_t *)gl_bos_dscr;
                length = MAKE_WORD(ptr[3],ptr[2]);
            }
            break;

#if (FLASHING_MODE_USB_ENABLE != 0)
        case USB_HID_DSCR:
            ptr = (uint8_t *)gl_bb[port].ep0_buffer;
            length = usb_hid_get_inf_dscr(ptr, USB_EP_MAX_PKT_SIZE,
                    1, BB_HID_INF_STRING_INDEX);
            /*
             * Since only the HID descriptor needs to be returned
             * and not the subsequent endpoint descriptor, return
             * only the HID descriptor.
             */
            ptr += USB_INF_DSCR_SIZE;
            length = GET_MIN(USB_HID_DSCR_SIZE, length);
            break;

        case USB_REPORT_DSCR:
            ptr = (uint8_t *)gl_bb[port].ep0_buffer;
            length = usb_hid_get_report_dscr(ptr, BB_MAX_EP0_XFER_SIZE);
            break;
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */

        case USB_STRING_DSCR:
            if (index == BB_LANG_ID_STRING_INDEX)
            {
                ptr = (uint8_t *)gl_lang_id_dscr;
            }
            else if(index == BB_SERIAL_STRING_INDEX)
            {
                if (bb_cfg->bb_string_dscr_offset[index - 1] == 0xFFFF)
                {
                    /* Create unique serial number. */
                    ptr = (uint8_t *)gl_bb[port].ep0_buffer;
                    generate_unique_serial_string(ptr);
                }
                else if (bb_cfg->bb_string_dscr_offset[index - 1] != 0)
                {
                    ptr = (uint8_t *)(get_pd_config()) + bb_cfg->bb_string_dscr_offset[index - 1];
                }
            }
            else if (index < USB_NUM_STRING_DSCR)
            {
                if (bb_cfg->bb_string_dscr_offset[index - 1] != 0)
                {
                    ptr = (uint8_t *)(get_pd_config()) + bb_cfg->bb_string_dscr_offset[index - 1];
                }
            }
            else
            {
                /* Not supported. Do nothing. */
            }

            /* Retreive the length. */
            if (ptr != NULL)
            {
                length = ptr[0];
            }
            break;

        default:
            /* Do nothing. */
            break;
    }

    if (ptr != NULL)
    {
        length = GET_MIN(length, pkt->length);
        status = usb_ep0_setup_write(ptr, length, true, NULL);
    }

    return status;
}

#if (FLASHING_MODE_USB_ENABLE != 0)
uint8_t *usb_hid_get_ep0_buffer(void)
{
    return gl_bb[port].ep0_buffer;
}

/* The function handles USB events. */
static void usb_event_handler(usb_state_t state, uint32_t data)
{
    (void)data;

    if (state == USB_STATE_CONFIGURED)
    {
        if (gl_bb[port].usb_configured == false)
        {
#if (APP_I2CM_BRIDGE_ENABLE)
            if (gl_bb[port].usb_i2cm_mode != false)
            {
                usb_i2cm_inf_ctrl(true);
            }
            else
#endif /* (APP_I2CM_BRIDGE_ENABLE) */
            {
                usb_hid_inf_ctrl(true);
            }

            gl_bb[port].usb_configured = true;
        }
    }
    else if ((state < USB_STATE_CONFIGURED) ||
            (state == USB_STATE_UNCONFIGURED))
    {
        if (gl_bb[port].usb_configured != false)
        {
#if (APP_I2CM_BRIDGE_ENABLE)
            if (gl_bb[port].usb_i2cm_mode != false)
            {
                usb_i2cm_inf_ctrl(false);
            }
            else
#endif /* (APP_I2CM_BRIDGE_ENABLE) */
            {
                usb_hid_inf_ctrl(false);
            }

            gl_bb[port].usb_configured = false;
        }
    }
}

ccg_status_t usb_hid_flashing_rqt(bool enable)
{
    return bb_flashing_ctrl(port, enable);
}
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */

ccg_status_t bb_init(uint8_t port)
{
    usb_config_t cfg;
    ccg_status_t status = CCG_STAT_SUCCESS;
    bb_settings_t *bb_cfg;

    if (gl_bb[port].state != BB_STATE_DEINITED)
    {
        return CCG_STAT_BUSY;
    }

    /* Verify if the configuration allows billboard. */
    bb_cfg = pd_get_ptr_bb_tbl(port);
    if(bb_cfg->bb_enable != BB_TYPE_INTERNAL)
    {
        return CCG_STAT_FAILURE;
    }

    memset((uint8_t *)&cfg, 0, sizeof(cfg));
    cfg.bus_power = true;
    cfg.get_dscr_cb = usb_get_dscr_rqt_handler;
#if (FLASHING_MODE_USB_ENABLE != 0)
    cfg.class_rqt_cb = usb_hid_class_rqt_handler;
    cfg.event_cb = usb_event_handler;
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */
#if (APP_I2CM_BRIDGE_ENABLE)
    cfg.vendor_rqt_cb = usb_i2cm_vendor_rqt_handler;
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

    /* Retreive the number of alternate modes. */
    if (bb_cfg->bb_bos_dscr_offset != 0)
    {
        gl_bb[port].num_alt_modes = *((uint8_t *)(get_pd_config()) +
            bb_cfg->bb_bos_dscr_offset + USB_BOS_DSCR_NUM_ALT_MODE_OFFET);
    }
    else
    {
        gl_bb[port].num_alt_modes = 0;
    }

    status = usb_init(&cfg);
    if (status == CCG_STAT_SUCCESS)
    {
        gl_bb[port].type = BB_TYPE_INTERNAL;
        gl_bb[port].state = BB_STATE_DISABLED;
        gl_bb[port].usb_port = port;
        gl_bb[port].ep0_buffer = (uint8_t *)gl_ep0_buffer[port];
        bb_update_all_status(port, BB_ALT_MODE_STATUS_INIT_VAL);
    }

    return status;
}

ccg_status_t bb_enable(uint8_t port, bb_cause_t cause)
{
    bb_settings_t *bb_cfg;

    /* Queue an enable only if the block is initialized and not in flashing mode. */
    if ((gl_bb[port].state == BB_STATE_DEINITED) ||
            (gl_bb[port].state == BB_STATE_LOCKED))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Allow billboard enumeration only if the configuration allows. */
    bb_cfg = pd_get_ptr_bb_tbl(port);
    if ((cause == BB_CAUSE_AME_SUCCESS) && (bb_cfg->bb_always_on == false))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Update the additional failure information data. */
    switch (cause)
    {
        case BB_CAUSE_AME_TIMEOUT:
        case BB_CAUSE_AME_FAILURE:
            gl_bb[port].bb_add_info = BB_CAP_ADD_FAILURE_INFO_PD;
            break;

        case BB_CAUSE_PWR_FAILURE:
            gl_bb[port].bb_add_info = BB_CAP_ADD_FAILURE_INFO_PWR;
            break;

        default:
            gl_bb[port].bb_add_info = 0;
            break;
    }

    /* Load the information. */
    gl_bb[port].queue_enable = true;

    return CCG_STAT_SUCCESS;
}

ccg_status_t bb_disable(uint8_t port, bool force)
{
    ccg_status_t status = CCG_STAT_SUCCESS;

    /* Allow disable during flashing only if force parameter is true. */
    if (((force != false) && (gl_bb[port].state <= BB_STATE_DISABLED)) ||
            ((force == false) && ((gl_bb[port].state != BB_STATE_BILLBOARD) ||
                (flash_access_get_status((1 << FLASH_IF_USB_HID)) != false))))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Clear any pending enable request and then queue a disable request. */
    gl_bb[port].queue_enable = false;
    gl_bb[port].queue_disable = true;
#if (APP_I2CM_BRIDGE_ENABLE)
        gl_bb[port].queue_i2cm_enable = false;
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

    return status;
}

ccg_status_t bb_update_alt_status(uint8_t port, uint8_t mode_index,
        bb_alt_mode_status_t alt_status)
{
    uint32_t status;

    /* Allow update of alternate mode only if initialized. */
    if (gl_bb[port].state == BB_STATE_DEINITED)
    {
        return CCG_STAT_NOT_READY;
    }

    if (mode_index >= BB_MAX_ALT_MODES)
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    status = (gl_bb[port].alt_status & ~(BB_ALT_MODE_STATUS_MASK << (mode_index << 1)));
    status |= (alt_status << (mode_index << 1));
    gl_bb[port].alt_status = status;

    return CCG_STAT_SUCCESS;
}

ccg_status_t bb_update_all_status(uint8_t port, uint32_t status)
{
    uint8_t state;

    /* Allow update of alternate mode only if initialized. */
    if (gl_bb[port].state == BB_STATE_DEINITED)
    {
        return CCG_STAT_NOT_READY;
    }

    state = CyEnterCriticalSection();
    /*
     * Mask out the unused alternate modes. The expression does the following:
     * 2 ^ (2 * number of alternate modes) - 1: This is the mask for
     * value status information (two bits per alternate mode).
     */
    gl_bb[port].alt_status = ((1 << (gl_bb[port].num_alt_modes << 1)) - 1) & status;

    CyExitCriticalSection(state);

    return CCG_STAT_SUCCESS;
}

bool bb_is_present(uint8_t port)
{
    if(gl_bb[port].state == BB_STATE_DEINITED)
    {
        return false;
    }

    return true;
}

bool bb_is_idle(uint8_t port)
{
    if ((gl_bb[port].queue_enable != false) ||
            (gl_bb[port].queue_disable != false) ||
            (usb_is_idle() == false))
    {
        return false;
    }

    return true;
}

bool bb_enter_deep_sleep(uint8_t port)
{
    usb_state_t state;

    state = usb_get_state();

    if (((state == USB_STATE_SUSPENDED) || (state <= USB_STATE_DISABLED)) &&
            (bb_is_idle(port) != false))
    {
        return true;
    }

    return false;
}

static void bb_off_timer_cb(uint8_t port, timer_id_t id)
{
    (void)id;

    if(gl_bb[port].timeout == 0)
    {
        /* Indicate that the port needs to be disabled. */
        bb_disable(port, false);
    }
    else
    {
        /* Continue the timer until the required delay is achieved. */
        uint32_t timeout = gl_bb[port].timeout;

        if(timeout > BB_OFF_TIMER_MAX_INTERVAL)
        {
            timeout = BB_OFF_TIMER_MAX_INTERVAL;
        }
        gl_bb[port].timeout -= timeout;

        timer_start(port, APP_BB_OFF_TIMER, (uint16_t)timeout, bb_off_timer_cb);
    }
}

static void bb_off_timer_start(uint8_t port)
{
    bb_settings_t *bb_cfg;
    uint32_t timeout = 0;

    /* Disable the billboard interface after the specified timeout if the alt. mode status is success. */
    bb_cfg = pd_get_ptr_bb_tbl(port);
    if ((gl_bb[port].bb_add_info == 0) && (bb_cfg->bb_timeout != BB_OFF_TIMER_NO_DISABLE))
    {
        timeout = bb_cfg->bb_timeout;
        if (timeout < BB_OFF_TIMER_MIN_VALUE)
        {
            timeout = BB_OFF_TIMER_MIN_VALUE;
        }

        /* Convert time to milliseconds. */
        timeout *= 1000;
        gl_bb[port].timeout = timeout;

        /* Ensure that the timeout does not exceed parameter boundary. */
        if(timeout > BB_OFF_TIMER_MAX_INTERVAL)
        {
            timeout = BB_OFF_TIMER_MAX_INTERVAL;
        }
        gl_bb[port].timeout -= timeout;

        timer_start(port, APP_BB_OFF_TIMER, timeout, bb_off_timer_cb);
    }
    else
    {
        gl_bb[port].timeout = 0;
    }
}

ccg_status_t bb_flashing_ctrl(uint8_t port, bool enable)
{
#if (FLASHING_MODE_USB_ENABLE != 0)
    uint8_t state;

    /* Change only if the block is initialized and not in flashing mode. */
    if ((gl_bb[port].state <= BB_STATE_DISABLED) ||
            (gl_bb[port].flashing_mode == enable))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Load the information and clear any pending enable / disable. */
    state = CyEnterCriticalSection();
    gl_bb[port].queue_enable = false;
    gl_bb[port].queue_disable = false;
    if (enable == false)
    {
        /* Now start the billboard expiry timer if required. */
        bb_off_timer_start(port);
    }
    else
    {
#if (APP_I2CM_BRIDGE_ENABLE)
        /* Do this only if not in bridge mode. */
        if (gl_bb[port].usb_i2cm_mode == false)
#endif /* (APP_I2CM_BRIDGE_ENABLE) */
        {
            gl_bb[port].state = BB_STATE_LOCKED;
            /* Now stop the billboard expiry timer. */
            timer_stop(port, APP_BB_OFF_TIMER);
            gl_bb[port].timeout = 0;
        }
    }
    gl_bb[port].flashing_mode = enable;
    CyExitCriticalSection(state);
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */

    return CCG_STAT_SUCCESS;
}

#if (APP_I2CM_BRIDGE_ENABLE)
ccg_status_t bb_bridge_ctrl(uint8_t port, bool enable)
{
    uint8_t state;
    ccg_status_t status = CCG_STAT_SUCCESS;

    /* Queue an enable only if the block is initialized, not in flashing mode. */
    if ((gl_bb[port].state == BB_STATE_DEINITED) ||
            (gl_bb[port].flashing_mode != false) ||
            (gl_bb[port].usb_i2cm_mode == enable))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Load the information and clear any pending enable / disable. */
    state = CyEnterCriticalSection();
    if (enable == false)
    {
        /* This is no different from the forced disable. */
        bb_disable(port, true);
    }
    else
    {
        gl_bb[port].state = BB_STATE_LOCKED;
        /* Now stop the billboard expiry timer. */
        timer_stop(port, APP_BB_OFF_TIMER);
        gl_bb[port].timeout = 0;
        gl_bb[port].queue_i2cm_enable = true;
        gl_bb[port].queue_disable = true;
    }
    CyExitCriticalSection(state);

    return status;
}

uint8_t *usb_i2cm_get_ep0_buffer(void)
{
    return gl_bb[port].ep0_buffer;
}
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

/*
 * Internal functions do not validate the parameters. Caller is expected
 * to ensure that the parameters are valid. The function does the actual
 * start of the USB module based on the request queued. This function is
 * expected to be called only from the bb_task() function.
 */
static ccg_status_t bb_start(uint8_t port)
{
    bool reg_enable = false;
    uint16_t vbus_mv;
    ccg_status_t status = CCG_STAT_SUCCESS;

    vbus_mv = pd_adc_calibrate(port, APP_VBUS_POLL_ADC_ID);
    if (vbus_mv > USB_VBUS_REG_EN_THRESHOLD)
    {
        reg_enable = true;
    }
    status = usb_enable(reg_enable);
    if (status == CCG_STAT_SUCCESS)
    {
        gl_bb[port].state = BB_STATE_BILLBOARD;
        gl_bb[port].usb_configured = false;

        /* Start the billboard interface disable timer. */
        bb_off_timer_start(port);
    }

    /* Clear the queue flag. */
    gl_bb[port].queue_enable = false;

    return status;
}

/*
 * Internal functions do not validate the parameters. Caller is expected
 * to ensure that the parameters are valid. The function does the actual
 * disable of the USB module based on the request queued. This function is
 * expected to be called only from the bb_task() function.
 */
static ccg_status_t bb_stop(uint8_t port)
{
    ccg_status_t status = CCG_STAT_SUCCESS;

    /* If the interface is enabled, disable it first. */
    if (gl_bb[port].usb_configured != false)
    {
#if (APP_I2CM_BRIDGE_ENABLE)
        if (gl_bb[port].usb_i2cm_mode != false)
        {
            status = usb_i2cm_inf_ctrl(false);
        }
        else
#endif /* (APP_I2CM_BRIDGE_ENABLE) */
        {
            status = usb_hid_inf_ctrl(false);
        }
        
        gl_bb[port].usb_configured = false;
    }

    /* Now disable the USB block. */
    if (status == CCG_STAT_SUCCESS)
    {
        status = usb_disable();
    }
    if (status == CCG_STAT_SUCCESS)
    {
        gl_bb[port].state = BB_STATE_DISABLED;
        gl_bb[port].usb_configured = false;
        gl_bb[port].queue_disable = false;

        /* Start the timer for delaying the next start. */
        timer_start(port, APP_BB_ON_TIMER, APP_BB_ON_TIMER_PERIOD, NULL);
    }

    return status;
}

void bb_task(uint8_t port)
{
    uint8_t state;

    if (gl_bb[port].state == BB_STATE_DEINITED)
    {
        return;
    }

    state = CyEnterCriticalSection();
    if (gl_bb[port].queue_disable != false)
    {
        if (gl_bb[port].state > BB_STATE_DISABLED)
        {
            bb_stop(port);
        }
#if (APP_I2CM_BRIDGE_ENABLE)
        /* Update state to locked state to prevent any race condition. */
        if (gl_bb[port].queue_i2cm_enable != false)
        {
            gl_bb[port].state = BB_STATE_LOCKED;
        }
#endif /* (APP_I2CM_BRIDGE_ENABLE) */
    }
#if (APP_I2CM_BRIDGE_ENABLE)
    /*
     * If this is a I2C bridge mode request, then start the bridge mode
     * after the required re-start delay is imposed.
     */
    if ((gl_bb[port].queue_i2cm_enable != false) &&
            (timer_is_running(port, APP_BB_ON_TIMER) == false))
    {
        /* Start the enumeration. */
        if (bb_start(port) == CCG_STAT_SUCCESS)
        {
            gl_bb[port].state = BB_STATE_LOCKED;
            gl_bb[port].usb_i2cm_mode = true;
            gl_bb[port].queue_i2cm_enable = false;
        }
    }
#endif /* (APP_I2CM_BRIDGE_ENABLE) */
    if (gl_bb[port].queue_enable != false)
    {
        /* Disable the billboard interface to present the update. */
        if (gl_bb[port].state > BB_STATE_DISABLED)
        {
            bb_stop(port);
        }
        /* Wait for the ON delay timer to expire. */
        if (timer_is_running(port, APP_BB_ON_TIMER) == false)
        {
            if (gl_bb[port].state == BB_STATE_DISABLED)
            {
                bb_start(port);
            }
            else
            {
                /* Interface is in another mode and cannot be started. */
                gl_bb[port].queue_enable = false;
            }
        }
    }
    CyExitCriticalSection(state);

#if (APP_I2CM_BRIDGE_ENABLE)
    /* Invoke the bridge tasks. */
    if (gl_bb[port].usb_i2cm_mode != false)
    {
        usb_i2cm_task();
    }
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

    /* Invoke the USB tasks. */
    usb_task();
}

uint8_t *bb_get_version()
{
    /* No version associated with internal billboard. */
    return NULL;
}

#endif /* (CCG_BB_ENABLE != 0) */

/* [] */

