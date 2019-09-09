/**
 * @file usb_hid.c
 *
 * @brief @{USB HID class interface (flashing) source file.@}
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
#include "status.h"
#include "usb.h"
#include "utils.h"
#include "system.h"
#include "boot.h"
#include "flash.h"
#include "usb_hid.h"
#include "ccgx_regs.h"

#if (FLASHING_MODE_USB_ENABLE != 0)

/* Index of last flash row which is part of the USB boot-loader application. */
static uint16_t gl_hid_bl_last_row = CCG_BOOT_LOADER_LAST_ROW;

/* Index of last flash row which is part of the FW1 application. */
static uint16_t gl_hid_fw1_last_row = CCG_IMG1_LAST_FLASH_ROW_NUM;

/* Cypress defined manufacturing report descriptor for CY vendor requests. */
static const uint8_t gl_hid_report_dscr[] =
{
    0x06, 0xEE, 0xFF,           /* Usage Page (Vendor-Defined 1) */
    0x09, 0x01,                 /* Usage (Vendor-Defined 1) */
    0xA1, 0x01,                 /* Collection (Application) */

    0x85, HID_REPORT_ID_CY_FW_INFO,
    /* Report ID (E0) - Get FW info */
    0x09, 0x02,             /* Usage (Vendor-Defined 2) */
    0x15, 0x00,             /* Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /* Logical Maximum (255) */
    0x75, 0x08,             /* Report Size (8) */
    0x95, HID_CY_FW_INFO_SIZE,
    /* Report Count (63) */
    0xB1, 0x02,             /* Feature
                             * (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit) */

    0x85, HID_REPORT_ID_RQT,
    /* Report ID (E1) – Command request */
    0x09, 0x02,             /* Usage (Vendor-Defined 2) */
    0x15, 0x00,             /* Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /* Logical Maximum (255) */
    0x75, 0x08,             /* Report Size (8) */
    0x95, 0x07,
    /* Report Count (7) */
    0x91, 0x02,             /* Output
                             * (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit) */

    0x85, HID_REPORT_ID_FLASH_WRITE,
    /* Report ID (E2) – FLASH_WRITE */
    0x09, 0x02,             /* Usage (Vendor-Defined 2) */
    0x15, 0x00,             /* Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /* Logical Maximum (255) */
    0x75, 0x08,             /* Report Size (8) */
    0x95, CCG_FLASH_ROW_SIZE + 3,
    /* Report Count (131/259) */
    0x91, 0x02,             /* Output
                             * (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit) */

    0x85, HID_REPORT_ID_FLASH_READ,
    /* Report ID (E3) – FLASH_READ */
    0x09, 0x02,             /* Usage (Vendor-Defined 2) */
    0x15, 0x00,             /* Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /* Logical Maximum (255) */
    0x75, 0x08,             /* Report Size (8) */
    0x95, CCG_FLASH_ROW_SIZE + 3,
    /* Report Count (131/259) */
    0xB1, 0x02,             /* Feature
                             * (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit) */

    0x85, HID_REPORT_ID_CUSTOMER_INFO,
    /* Report ID (E4) - Get customer info */
    0x09, 0x02,             /* Usage (Vendor-Defined 2) */
    0x15, 0x00,             /* Logical Minimum (0) */
    0x26, 0xFF, 0x00,       /* Logical Maximum (255) */
    0x75, 0x08,             /* Report Size (8) */
    0x95, HID_CUSTOMER_INFO_SIZE,
    /* Report Count (32) */
    0xB1, 0x02,             /* Feature
                             * (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit) */

    0xC0,                       /* End Collection */
};

/* The interface number offset inside the HID interface descriptor. */
#define HID_DSCR_INF_NUM_OFFSET (2)
/* Interface string index offset inside the HID interface descriptor. */
#define HID_DSCR_INF_STR_OFFSET (8)

/* HID interface descriptor. */
static const uint8_t gl_hid_inf_dscr[] =
{
    /* Interface descriptor. */
    USB_INF_DSCR_SIZE,          /* Descriptor size. */
    USB_INF_DSCR,               /* Descriptor type. */
    0x00,                       /* Interface number - NOTE: Needs to be
                                   updated. */
    0x00,                       /* Alternate setting: None. */
    0x01,                       /* Number of endpoints: 1. */
    USB_CLASS_HID,              /* Interface class - HID interface */
    0x00,                       /* Interface subclass - 0 (none). */
    0x00,                       /* Interface protocol - 0 (none). */
    0x00,                       /* Interface string index - NOTE:
                                   Needs to be updated. */

    /* HID descriptor. */
    USB_HID_DSCR_SIZE,          /* Descriptor size. */
    USB_HID_DSCR,               /* Descriptor type - HID. */
    0x11, 0x01,                 /* bcdHID : version 1.11 */
    0x00,                       /* Target country - None. */
    0x01,                       /* Number of report descriptors. */
    USB_REPORT_DSCR,            /* Descriptor type - Report. */
    WORD_GET_LSB((uint16_t)sizeof(gl_hid_report_dscr)),
    WORD_GET_MSB((uint16_t)sizeof(gl_hid_report_dscr)),
    /* Descriptor size -  Based on report. */

    /* IN Endpoint descriptor. Only to comply with HID spec. */
    USB_EP_DSCR_SIZE,           /* Descriptor size. */
    USB_EP_DSCR,                /* Descriptor type - endpoint. */
    0x81,                       /* Endpoint address - 1 IN. */
    USB_EP_INTR,                /* Endpoint attributes - INT. */
    USB_EP_MAX_PKT_SIZE, 0x00,  /* Maximum endpoint. */
    0xFF                        /* bInterval - 255ms Polling interval is
                                   kept low to reduce host load. */
};

/* Current flash row ID used for FLASH_READ operation. */
static uint16_t gl_flash_read_row;

#if (FLASH_ENABLE_NB_MODE != 0)
/* Variable holds if there is any pending flash write */
static bool gl_flash_write_pending;
#endif

uint16_t usb_hid_get_inf_dscr(uint8_t *buffer, uint16_t max_length,
        uint8_t inf_num, uint8_t inf_str_index)
{
    uint16_t length;

    length = GET_MIN(max_length, sizeof(gl_hid_inf_dscr));
    memcpy(buffer, gl_hid_inf_dscr, length);

    /* Now update the interface number and string index. */
    buffer[HID_DSCR_INF_NUM_OFFSET] = inf_num;
    buffer[HID_DSCR_INF_STR_OFFSET] = inf_str_index;

    return length;
}

uint16_t usb_hid_get_report_dscr(uint8_t *buffer, uint16_t max_length)
{
    uint16_t length;

    length = GET_MIN(max_length, sizeof(gl_hid_report_dscr));
    memcpy(buffer, gl_hid_report_dscr, length);

    return length;
}

void usb_hid_set_fw_locations(uint16_t bl_last_row, uint16_t fw1_last_row)
{
    gl_hid_bl_last_row  = bl_last_row;
    gl_hid_fw1_last_row = fw1_last_row;
}

static ccg_status_t hid_handle_get_report(hid_report_id_t id, usb_setup_pkt_t *pkt)
{
    uint8_t *ptr;
    uint32_t *dptr;
    uint16_t length = 0;
    uint32_t temp;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    ptr = usb_hid_get_ep0_buffer();

    switch (id)
    {
        case HID_REPORT_ID_CY_FW_INFO:
#if (!CCG_BOOT)
            /* Make sure the firmware images are validated. The boot-loader
             * would already have done this at start-up.
             */
            boot_update_fw_status ();
#endif /* !CCG_BOOT */

            /* Load firmware information - Refer header file for more info. */
            memset(ptr, 0, HID_CY_FW_INFO_SIZE + 1);
            ptr[0] = HID_REPORT_ID_CY_FW_INFO;
            ptr[2] = 'C';
            ptr[3] = 'Y';
            ptr[4] = sys_get_device_mode();
            ptr[5] = 0; /* TODO: Retrieve the correct information from boot-loader. */
            ptr[6] = get_boot_mode_reason().val;
            ptr += 8;

            sys_get_silicon_id(&temp);
            memcpy(ptr, &temp, sizeof(temp));
            ptr += sizeof(temp);

            memcpy(ptr, sys_get_boot_version(), FW_VERSION_SIZE);
            ptr += FW_VERSION_SIZE;

            memcpy(ptr, sys_get_img1_fw_version(), FW_VERSION_SIZE);
            ptr += FW_VERSION_SIZE;

            memcpy(ptr, sys_get_img2_fw_version(), FW_VERSION_SIZE);
            ptr += FW_VERSION_SIZE;

            dptr = (uint32_t *)ptr;
            dptr[0] = ((gl_hid_bl_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM);
            dptr[1] = ((gl_hid_fw1_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM);
            ptr += 2 * sizeof(uint32_t);

            /* Expecting 32-bit alignment to avoid copy. */
            CyGetUniqueId((unsigned long *)ptr);

            /* Reset the buffer pointer. */
            ptr = usb_hid_get_ep0_buffer();
            length = GET_MIN((HID_CY_FW_INFO_SIZE + 1), pkt->length);
            status = usb_ep0_setup_write(ptr, length, true, NULL);
            break;

        case HID_REPORT_ID_FLASH_READ:
            ptr[0] = HID_REPORT_ID_FLASH_READ;
            ptr[1] = 'F';
            ptr[2] = WORD_GET_LSB(gl_flash_read_row);
            ptr[3] = WORD_GET_MSB(gl_flash_read_row);
            status = flash_row_read(gl_flash_read_row, &ptr[4]);
            if (status == CCG_STAT_SUCCESS)
            {
                length = GET_MIN((CCG_FLASH_ROW_SIZE + 4), pkt->length);
                status = usb_ep0_setup_write(ptr, length, true, NULL);
            }
            /* Increment the pointer in case of success. */
            if (status == CCG_STAT_SUCCESS)
            {
                gl_flash_read_row++;
            }
            break;

        case HID_REPORT_ID_CUSTOMER_INFO:
            ptr[0] = HID_REPORT_ID_CUSTOMER_INFO;
            memcpy(&ptr[1], (uint8_t *)sys_get_custom_info_addr(),
                    HID_CUSTOMER_INFO_SIZE);
            length = GET_MIN((HID_CUSTOMER_INFO_SIZE + 1), pkt->length);
            status = usb_ep0_setup_write(ptr, length, true, NULL);
            break;

        default:
            /* Do nothing. */
            break;
    }

    return status;
}

#if (FLASH_ENABLE_NB_MODE != 0)
/* Non-blocking flash write callback function */
static void flash_nb_write_cb(flash_write_status_t write_status)
{
    gl_flash_write_pending = false;

    /* Respond on USB HID interface if write successfully completed. */
    if (write_status != FLASH_WRITE_IN_PROGRESS)
    {
        if (write_status == FLASH_WRITE_COMPLETE)
            usb_ep0_send_recv_status();
        else
            usb_ep0_set_stall ();
    }
}
#else
void *flash_nb_write_cb = NULL;
#endif /* (FLASH_ENABLE_NB_MODE != 0) */

/* EP0 callback function for delayed handling for flash access. */
static ccg_status_t hid_flash_write_cb(usb_setup_pkt_t *pkt)
{
    uint8_t *ptr;
    uint16_t row_id;
    ccg_status_t status = CCG_STAT_FAILURE;

    /*
     * Note: This is how the flash write buffer is formatted:
     * Bytes 0 - 3: Reserved and not used.
     * Bytes 4 - 7: Report ID , Signature and Row ID.
     * Bytes 8 - 135: Flash row data
     */
    ptr = usb_hid_get_ep0_buffer();

    /* Validate the received parameters */
    if (
            (ptr[5] == 'F')
#if (FLASH_ENABLE_NB_MODE != 0)
            &&
            (gl_flash_write_pending == false)
#endif /* (FLASH_ENABLE_NB_MODE != 0) */
       )
    {
        row_id = MAKE_WORD(ptr[7], ptr[6]);
        status = flash_row_write(row_id, &ptr[8], flash_nb_write_cb);

        /* In blocking write if flash write is successful, response is Success
         * and in non blocking write if flash write operation starts, response is
         * NO RESPONSE. */
#if (FLASH_ENABLE_NB_MODE != 0)
        if (status == CCG_STAT_NO_RESPONSE)
        {
            gl_flash_write_pending = true;
            status = CCG_STAT_SUCCESS;
        }
#else
        if (status == CCG_STAT_SUCCESS)
        {
            /* Complete the status phase */
            status = usb_ep0_send_recv_status();
        }
#endif /* (FLASH_ENABLE_NB_MODE != 0) */
    }

    /* Stall on error is already being done by the USB module. */

    return status;
}

/*
 * HID jump request handler. The function jumps to the respective firmware image
 * based on the parameter. USB success handling is done inside the function.
 * In case of error, the caller is expected to handle EP0 stall.
 */
static ccg_status_t hid_handle_rqt_jump(uint8_t cmd)
{
    uint32_t addr;
    uint32_t sig = 0;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;
    bool ack = false;

    (void)addr;
    (void)sig;

    switch (cmd)
    {
#if !defined(CCG_BOOT) && (CCG_FIRMWARE_APP_ONLY == 0) && (SECURE_FW_UPDATE == 0)
        case 'J':
            ack = true;
            sig = CCG_BOOT_MODE_RQT_SIG;
            break;
#endif /* !defined(CCG_BOOT) && (CCG_FIRMWARE_APP_ONLY == 0) && (SECURE_FW_UPDATE == 0) */

        case 'R':
            ack = true;
            /* NOTE: Signature will be zero for RESET command. */
            break;

#if !defined(CCG_BOOT) && (CCG_FIRMWARE_APP_ONLY == 0)
        case 'A':
#if (FLASH_ENABLE_NB_MODE == 0)
            if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
            {
                addr = CCG_IMG2_FW_METADATA_ADDR;
                sig = CCG_FW2_BOOT_RQT_SIG;
            }
            else
            {
                addr = CCG_IMG1_FW_METADATA_ADDR;
                sig = CCG_FW1_BOOT_RQT_SIG;
            }

            status = boot_validate_fw((sys_fw_metadata_t *)addr);
            if (status == CCG_STAT_SUCCESS)
            {
                ack = true;
            }
#else
            /*
             * Don't support JUMP_TO_ALT image command if non blocking
             * FW update feature is enabled.
             */
            /* NOTE: NACK flag is already set. So nothing shall be done here. */
#endif /* #if (FLASH_ENABLE_NB_MODE == 0) */
            break;
#endif /* !defined(CCG_BOOT) && (CCG_FIRMWARE_APP_ONLY == 0) */

        default:
            /* Do nothing. */
            break;
    }

    if (ack)
    {
        /* Wait for the ACK to complete */
        status = usb_ep0_send_recv_status();
        if(status == CCG_STAT_SUCCESS)
        {
            status = usb_ep0_wait_for_ack();

            /* 1ms delay added to ACK stage to finish before reset. */
            CyDelay(1);
        }

        if (status == CCG_STAT_SUCCESS)
        {
            /* Disable all interrupts. */
            CyGlobalIntDisable;

#if (CCG_FIRMWARE_APP_ONLY == 0)
            /* NOTE: Using Bootloader component provided variable
             * to store the signature. This makes sure that this value
             * is never overwritten by compiler and it retains the value
             * through resets and jumps. We use lower two bytes of this
             * variable to store the siganture. */
            cyBtldrRunType = sig;
#endif /* CCG_FIRMWARE_APP_ONLY */

            /* Call device reset routine. */
            CySoftwareReset();
        }
    }

    return status;
}

/*
 * Enter flashing mode request handler. The caller is expected to stall EP0
 * on failure cases. Status phase for success case is handled inside.
 */
static ccg_status_t hid_handle_rqt_enter_flashing(uint8_t cmd)
{
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    if (cmd == 'P')
    {
        /* Fail the request if already in flashing mode for another interface. */
        if (flash_access_get_status(~(1 << FLASH_IF_USB_HID)) == false)
        {
            /* Enable flashing mode if this is the first request. */
            if (flash_access_get_status(1 << FLASH_IF_USB_HID) == false)
            {
                status = usb_hid_flashing_rqt(true);
                if (status == CCG_STAT_SUCCESS)
                {
                    /*
                     * TODO: Allocate 8 header bytes before the data buffer so that in-place flash writes
                     * can be performed.
                     */
                    flash_enter_mode(true, FLASH_IF_USB_HID, false);
                }
            }
            else
            {
                /* Flashing mode is already active. Nothing to do here. */
                status = CCG_STAT_SUCCESS;
            }
        }
    }
    else
    {
#if (FLASH_ENABLE_NB_MODE != 0)
        /*
         * Check if a non-blocking flash write is active.
         * If yes, start an abort operation.
         */
        if (gl_flash_write_pending)
        {
            /* Signal Flash module to abort the current flash write operation. */
            flash_non_blocking_write_abort();
        }
#endif /* FLASH_ENABLE_NB_MODE */
        /* Fail the request if already in flashing mode. */
        if (flash_access_get_status(1 << FLASH_IF_USB_HID) != false)
        {
            flash_enter_mode(false, FLASH_IF_USB_HID, false);
            status = usb_hid_flashing_rqt(false);
        }
    }
    if (status == CCG_STAT_SUCCESS)
    {
        status = usb_ep0_send_recv_status();
    }

    return status;
}

/* HID request command callback. */
static ccg_status_t hid_rqt_cb(usb_setup_pkt_t *pkt)
{
    uint8_t *ptr;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    ptr = usb_hid_get_ep0_buffer();

    /* Handle request. */
    switch (ptr[1])
    {
        case HID_RQT_CMD_JUMP:
            status = hid_handle_rqt_jump(ptr[2]);
            break;

        case HID_RQT_CMD_ENTER_FLASHING:
            status = hid_handle_rqt_enter_flashing(ptr[2]);
            break;

        case HID_RQT_CMD_SET_READ_ROW:
            gl_flash_read_row = MAKE_WORD(ptr[3], ptr[2]);
            status = usb_ep0_send_recv_status();
            break;

        case HID_RQT_CMD_VALIDATE_FW:
            status = boot_handle_validate_fw_cmd(ptr[2]);
            if (status == CCG_STAT_SUCCESS)
            {
                status = usb_ep0_send_recv_status();
            }
            break;

#if (APP_PRIORITY_FEATURE_ENABLE != 0) && !defined(CCG_BOOT)
        case HID_RQT_CMD_SET_APP_PRIORITY:
            if (ptr[2] == 'F')
            {
                status = flash_set_app_priority(ptr[3]);
            }
            if (status == CCG_STAT_SUCCESS)
            {
                status = usb_ep0_send_recv_status();
            }
            break;
#endif /* (APP_PRIORITY_FEATURE_ENABLE != 0) && !defined(CCG_BOOT) */

#if (APP_I2CM_BRIDGE_ENABLE)
        case HID_RQT_CMD_I2C_BRIDGE_CTRL:
            status = usb_i2cm_bridge_ctrl((ptr[2] == 'B') ? true : false);
            if (status == CCG_STAT_SUCCESS)
            {
                status = usb_ep0_send_recv_status();
            }
            break;
#endif /* (APP_I2CM_BRIDGE_ENABLE) */

        default:
            /* Do nothing. */
            break;
    }

    return status;
}

static ccg_status_t hid_handle_set_report(hid_report_id_t id, usb_setup_pkt_t *pkt)
{
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    switch (id)
    {
        case HID_REPORT_ID_FLASH_WRITE:
            if (pkt->length == (CCG_FLASH_ROW_SIZE + 4))
            {
                /* Queue a read request. */
                /* Increase request length by 4 only in case of Flash Write,
                 * since the function usb_ep0_setup_read checks for length to be
                 * multiple of 8. This is required because for Flash Write the
                 * USB Packet Size is 132, which is not a multiple of 8.*/

                /* Flash write sequence expects 8 bytes of un-used space before
                 * flash row data buffer. Therefore, setting up read from 4th byte
                 * as first 4 bytes in the report are header and this will provide total
                 * 8 bytes of unused space in the buffer. */
                status = usb_ep0_setup_read((usb_hid_get_ep0_buffer() + 4),
                        pkt->length + 4, false, hid_flash_write_cb);
            }
            break;

        case HID_REPORT_ID_RQT:
            if (pkt->length == 8)
            {
                /* Queue a read request. */

                status = usb_ep0_setup_read(usb_hid_get_ep0_buffer(),
                        pkt->length, false, hid_rqt_cb);
            }
            break;

        default:
            /* Do nothing. */
            break;
    }

    return status;
}

/* USB class request handler for HID requests. */
ccg_status_t usb_hid_class_rqt_handler(usb_setup_pkt_t *pkt)
{
    uint8_t id;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    id = (pkt->value & HID_GET_SET_REPORT_ID_MASK);

    switch (pkt->cmd)
    {
        case USB_HID_RQT_GET_REPORT:
            status = hid_handle_get_report(id, pkt);
            break;

        case USB_HID_RQT_SET_REPORT:
            status = hid_handle_set_report(id, pkt);
            break;

        default:
            /* Do nothing. */
            break;
    }

    return status;
}

ccg_status_t usb_hid_inf_ctrl(bool enable)
{
    ccg_status_t status;

    if (enable)
    {
        status = usb_ep_enable(USB_HID_INTR_EP_INDEX, false);
    }
    else
    {
        status = usb_ep_disable(USB_HID_INTR_EP_INDEX);
    }

    return status;
}

#endif /* (FLASHING_MODE_USB_ENABLE != 0) */

/* [] */

