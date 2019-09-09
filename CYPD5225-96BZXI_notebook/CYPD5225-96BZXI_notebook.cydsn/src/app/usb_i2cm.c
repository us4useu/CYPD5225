/**
 * @file usb_hid.c
 *
 * @brief @{USB I2C master bridge interface source file.@}
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
#include "status.h"
#include "usb.h"
#include "utils.h"
#include "system.h"
#include "usb_i2cm.h"
#include "dpm.h"
#include "usb_hid.h"
#include "ccgx_regs.h"

#if (APP_I2CM_BRIDGE_ENABLE)

/* Device specific definitions. */
#ifdef CCG3
#define I2C_SCB_RX_TRIGGER              (64)
#define I2C_SCB_TX_TRIGGER              (64)
#define I2C_SCB_FIFO_SIZE               (128)
#define SCB0_INT_VECTOR_LOCATION        (8)
#define SCB1_INT_VECTOR_LOCATION        (9)
#define SCB2_INT_VECTOR_LOCATION        (10)
#define SCB3_INT_VECTOR_LOCATION        (11)
#else /* CCG */
#error "Device not supported by this module."
#endif /* CCG */

/* Update the SCB TX FIFO trigger level. */
#define i2c_update_tx_trig_level(scb_index,level) \
    SCB_PRT[(scb_index)]->tx_fifo_ctrl = ((level) & SCB_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK);

/* Update the SCB RX FIFO trigger level. */
#define i2c_update_rx_trig_level(scb_index,level) \
    SCB_PRT[(scb_index)]->rx_fifo_ctrl = ((level) & SCB_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK);

/*
   Enable the SCB transmitter.
   Note: SCBv2 does not have a transmit enable. Use the FIFO clear instead.
 */
#define i2c_scb_tx_enable(scb_index) \
    SCB_PRT[(scb_index)]->tx_fifo_ctrl &= ~SCB_TX_FIFO_CTRL_CLEAR;

/*
   Disable the SCB transmitter.
   Note: SCBv2 does not have a transmit enable. Use the FIFO clear instead.
 */
#define i2c_scb_tx_disable(scb_index) \
    SCB_PRT[(scb_index)]->tx_fifo_ctrl |= SCB_TX_FIFO_CTRL_CLEAR;

/* 
   Enable the SCB receiver.
   Note: SCBv2 does not have a receive enable. Use the FIFO clear instead.
 */
#define i2c_scb_rx_enable(scb_index) \
    SCB_PRT[(scb_index)]->rx_fifo_ctrl &= ~SCB_RX_FIFO_CTRL_CLEAR;

/*
   Disable the SCB receiver.
   Note: SCBv2 does not have a receive enable. Use the FIFO clear instead.
 */
#define i2c_scb_rx_disable(scb_index) \
    SCB_PRT[(scb_index)]->rx_fifo_ctrl |= SCB_RX_FIFO_CTRL_CLEAR;

/* Enable the SCB interrupt vector. */
#define i2c_scb_vic_int_enable(scb_index) \
    CyIntEnable(SCB0_INT_VECTOR_LOCATION + (scb_index))

/* Disable the SCB interrupt vector. */
#define i2c_scb_vic_int_disable(scb_index) \
    CyIntDisable(SCB0_INT_VECTOR_LOCATION + (scb_index))

/* USB string descriptor indices. */
#define USB_I2CM_LANG_ID_INDEX  (0)
#define USB_I2CM_MFG_STR_INDEX  (1)
#define USB_I2CM_PROD_STR_INDEX (2)

/* Single port point index to be used in this file. */
static uint8_t port = 0;

/* SCB register structures for direct access. */
const PSCB_PRT_REGS_T SCB_PRT[4] =
{
    SCB_PRT0,
    SCB_PRT1,
    SCB_PRT2,
    SCB_PRT3
};
    
/* Device descriptor support. */
const static uint8_t gl_i2cm_device_dscr[] =
{
    USB_DEVICE_DSCR_SIZE,       /* Descriptor size. */
    USB_DEVICE_DSCR,            /* Descriptor type. */
    0x00, 0x02,                 /* USB 2.00. */
    0x00,                       /* Device class - defined at interface. */
    0x00,                       /* Device sub-class - defined at interface. */
    0x00,                       /* Device protocol - defined at interface. */
    0x08,                       /* EP0 max packet size (8 bytes) - hardware limited. */
    0xB4, 0x04,                 /* VID. */
    0x50, 0xF6,                 /* PID = Flashing mode. */
    0x00, 0x00,                 /* Device release number. */
    USB_I2CM_MFG_STR_INDEX,     /* Manufacturer string index. */
    USB_I2CM_PROD_STR_INDEX,    /* Product string index. */
    0x00,                       /* Serial number string index. */
    0x01                        /* Number of configurations. */
};

/* Configuration descriptor support. */
const static uint8_t gl_i2cm_config_dscr[] =
{
    /* Configuration descriptor. */
    USB_CONFIG_DSCR_SIZE,       /* Descriptor size. */
    USB_CONFIG_DSCR,            /* Descriptor type. */
    USB_CONFIG_DSCR_SIZE, 0x00, /* Total length.
                                   NOTE: Need to calculate based on number of
                                   interfaces added. */
    0x00,                       /* Number of interfaces.
                                   NOTE: Need to load the correct number of
                                   interfaces. */
    0x01,                       /* Configuration number. */
    0x00,                       /* Configuration string index. */
    0x80,                       /* Configuration characteristics: Bus powered.
                                   Need to load the correct setting. */
    0x32,                       /* Max power consumption in 2mA unit: 100mA. */
};

/* Bridge interface descriptor. */
const static uint8_t gl_i2cm_inf_dscr[] =
{
    /* Interface descriptor */
    USB_INF_DSCR_SIZE,          /* Descriptor size */
    USB_INF_DSCR,               /* Interface Descriptor type */
    0x00,                       /* Interface number. Billboard device always
                                   comes up as first interface */
    0x00,                       /* Alternate setting number */
    0x03,                       /* Number of endpoints = 3 */
    USB_CLASS_VENDOR,           /* Interface class - vendor defined */
    USB_I2CM_SUB_CLASS,         /* Interface sub class */
    0x00,                       /* Interface protocol code */
    0x00,                       /* Interface descriptor string index */

    /* Endpoint Descriptor (BULK-OUT). */
    USB_EP_DSCR_SIZE,           /* Descriptor size. */
    USB_EP_DSCR,                /* Endpoint Descriptor type. */
    (USB_I2CM_OUT_EP_INDEX + 1),/* Endpoint address and description. */
    USB_EP_BULK,                /* Bulk Endpoint Type. */
    0x40,0x00,                  /* Max packet size : 64 bytes */
    0x00,                       /* Servicing Interval for data transfers. */

    /* Endpoint Descriptor (BULK-IN) */
    USB_EP_DSCR_SIZE,           /* Descriptor size */
    USB_EP_DSCR,                /* Endpoint descriptor type */
    0x80 | (USB_I2CM_IN_EP_INDEX + 1),
                                /* Endpoint address and description */
    USB_EP_BULK,                /* Bulk endpoint type */
    0x40,0x00,                  /* Max packet size = 512 bytes */
    0x00,                       /* Servicing interval for data transfers. */

    /* Endpoint Descriptor (INT-IN) */
    USB_EP_DSCR_SIZE,           /* Descriptor size */
    USB_EP_DSCR,                /* Endpoint descriptor type */
    0x80 | (USB_I2CM_INT_EP_INDEX + 1),
                                /* Endpoint address and description */
    USB_EP_INTR,                /* Interrupt endpoint type */
    0x40,0x00,                  /* Max packet size = 64 bytes */
    0x0A                        /* Servicing interval for data transfers */
};

/* Language ID string descriptor */
const static uint8_t gl_i2cm_lang_id_dscr[] =
{
    0x04,                       /* Descriptor size. */
    USB_STRING_DSCR,            /* Descriptor type. */
    0x09, 0x04                  /* Supported language ID. */
};

const static uint8_t gl_i2cm_mfg_str_dscr[] =
{
    0x10,                       /* Descriptor size. */
    USB_STRING_DSCR,            /* Descriptor type. */
    'C', 0x00,                  /* Manufacturer string. */
    'y', 0x00,
    'p', 0x00,
    'r', 0x00,
    'e', 0x00,
    's', 0x00,
    's', 0x00
};

const static uint8_t gl_i2cm_prod_str_dscr[] =
{
    0x16,                       /* Descriptor size. */
    USB_STRING_DSCR,            /* Descriptor type. */
    'U', 0x00,                  /* Product string. */
    'S', 0x00,
    'B', 0x00,
    ' ', 0x00,
    'B', 0x00,
    'R', 0x00,
    'I', 0x00,
    'D', 0x00,
    'G', 0x00,
    'E', 0x00
};

/* Internal state machine handle structure variable. */
static usb_i2cm_t gl_usb_i2cm;

static uint8_t gl_status_buffer[USB_I2CM_STAT_SIZE];

static uint8_t gl_buffer[USB_I2CM_BUF_SIZE];

ccg_status_t usb_i2cm_get_dscr_rqt_handler(usb_setup_pkt_t *pkt)
{
    uint8_t index, temp;
    uint8_t *ptr = NULL, *tmp_ptr;
    uint8_t *buffer = usb_i2cm_get_ep0_buffer();
    uint16_t length = 0;
    bb_settings_t *bb_cfg;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    index  = (pkt->value & 0xFF);
    bb_cfg = pd_get_ptr_bb_tbl(port);

    switch (pkt->value >> 8)
    {
        case USB_DEVICE_DSCR:
            ptr = buffer;
            length = USB_DEVICE_DSCR_SIZE;
            memcpy(ptr, gl_i2cm_device_dscr, length);
            break;

        case USB_CONFIG_DSCR:
            length = 0;
            temp = 0;
            ptr = buffer;
            tmp_ptr = ptr;
            memcpy(tmp_ptr, gl_i2cm_config_dscr, sizeof(gl_i2cm_config_dscr));
            length += sizeof(gl_i2cm_config_dscr);
            tmp_ptr += sizeof(gl_i2cm_config_dscr);

            if (bb_cfg->bb_bus_power == 0)
            {
                ptr[USB_CONFIG_DSCR_ATTRIB_OFFSET] |= USB_CONFIG_DSCR_SELF_POWERED;
            }

            /* Add the USB-I2CM interface */
            temp++;
            memcpy(tmp_ptr, gl_i2cm_inf_dscr, sizeof(gl_i2cm_inf_dscr));
            length += sizeof(gl_i2cm_inf_dscr);
            tmp_ptr += sizeof(gl_i2cm_inf_dscr);

#if (FLASHING_MODE_USB_ENABLE != 0)
            /* Check and add the HID interface as required. */
            if (bb_cfg->bb_option != 0)
            {
                uint16_t hid_length;

                /*
                 * Here the assumption is that the HID descriptor
                 * is going to be valid and shall not overflow.
                 */
                hid_length = usb_hid_get_inf_dscr(tmp_ptr, USB_EP_MAX_PKT_SIZE, temp, 0);
                length += hid_length;
                tmp_ptr += hid_length;
                temp++;
            }
#endif /* (FLASHING_MODE_USB_ENABLE != 0) */

            /* Now load the actual size and count to the response. */
            ptr[2] = WORD_GET_LSB(length);
            ptr[3] = WORD_GET_MSB(length);
            ptr[USB_CONFIG_DSCR_NUM_INF_OFFSET] = temp;
            break;

        case USB_STRING_DSCR:
            switch (index)
            {
                case USB_I2CM_LANG_ID_INDEX:
                    ptr = (uint8_t *)gl_i2cm_lang_id_dscr;
                    length = ptr[0];
                    break;

                case USB_I2CM_MFG_STR_INDEX:
                    ptr = (uint8_t *)gl_i2cm_mfg_str_dscr;
                    length = ptr[0];
                    break;

                case USB_I2CM_PROD_STR_INDEX:
                    ptr = (uint8_t *)gl_i2cm_prod_str_dscr;
                    length = ptr[0];
                    break;

                default:
                    /* Do nothing. */
                    break;
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

/* Forward declaration of static functions. */
static void i2cm_start(uint8_t scb_index);
static void i2cm_stop(uint8_t scb_index);
static void i2cm_scb_write(uint8_t scb_index, uint8_t *buffer, uint8_t size);
static void i2cm_scb_read(uint8_t scb_index, uint8_t *buffer, uint8_t size);
static void i2cm_reset(uint8_t scb_index);
static void i2cm_update_status(uint8_t scb_index);

ccg_status_t usb_i2cm_inf_ctrl(bool enable)
{
    ccg_status_t status = CCG_STAT_SUCCESS;

    if (enable != false)
    {
        /* Enable the USB endpoints. */
        status = usb_ep_enable(USB_I2CM_OUT_EP_INDEX, true);
        if (status == CCG_STAT_SUCCESS)
        {
            status = usb_ep_enable(USB_I2CM_IN_EP_INDEX, false);
        }
        if (status == CCG_STAT_SUCCESS)
        {
            status = usb_ep_enable(USB_I2CM_INT_EP_INDEX, false);
        }
        if (status == CCG_STAT_SUCCESS)
        {
            /* Enable the SCB for USB-I2CM operation. */
            i2cm_start(USB_I2CM_SCB_INDEX);
            gl_usb_i2cm.state = USB_I2CM_STATE_IDLE;
        }
    }
    else
    {
        /* Disable the USB endpoints. */
        usb_ep_disable(USB_I2CM_OUT_EP_INDEX);
        usb_ep_disable(USB_I2CM_IN_EP_INDEX);
        usb_ep_disable(USB_I2CM_INT_EP_INDEX);
        /* Disable bridge mode and enable the SCB for normal operation. */
        i2cm_stop(USB_I2CM_SCB_INDEX);
        gl_usb_i2cm.state = USB_I2CM_STATE_DISABLED;
    }

    return status;
}

bool usb_i2cm_is_idle(void)
{
    if (gl_usb_i2cm.state > USB_I2CM_STATE_IDLE)
    {
        return false;
    }

    return true;
}

ccg_status_t usb_i2cm_rqt_write_handler(usb_setup_pkt_t *pkt)
{
    ccg_status_t status = CCG_STAT_SUCCESS;

    gl_usb_i2cm.preamble = ((pkt->value >> 7) & 0xFE);
    gl_usb_i2cm.status = USB_I2CM_STAT_ACTIVE;
    gl_usb_i2cm.ctrl = (pkt->value & USB_I2CM_CTRL_FLAGS);
    gl_usb_i2cm.count = pkt->index;
    gl_usb_i2cm.length = pkt->index;

    /* Enable the OUT EP to receive data. */
    if (gl_usb_i2cm.length != 0)
    {
        status = usb_ep_queue_read_single(USB_I2CM_OUT_EP_INDEX);
    }

    /* Load the request details. */
    if (status == CCG_STAT_SUCCESS)
    {
        gl_usb_i2cm.state = USB_I2CM_STATE_WRITE;
        gl_usb_i2cm.preamble_state = true;
 
        /* The actual I2C transfer shall begin only on getting the USB data. */
    }

    return status;
}

ccg_status_t usb_i2cm_rqt_read_handler(usb_setup_pkt_t *pkt)
{
    ccg_status_t status = CCG_STAT_SUCCESS;

    gl_usb_i2cm.preamble = (((pkt->value >> 7) & 0xFE) | 0x01);
    gl_usb_i2cm.status = USB_I2CM_STAT_ACTIVE;
    gl_usb_i2cm.ctrl = (pkt->value & USB_I2CM_CTRL_FLAGS);
    gl_usb_i2cm.count = pkt->index;
    gl_usb_i2cm.length = pkt->index;

    /* Load the request details. */
    if (status == CCG_STAT_SUCCESS)
    {
        gl_usb_i2cm.state = USB_I2CM_STATE_READ;
        gl_usb_i2cm.preamble_state = true;

        /* Setup the I2C block. */
        SCB_PRT[USB_I2CM_SCB_INDEX]->intr_m = ~0;

        /* Load the command. */
        SCB_PRT[USB_I2CM_SCB_INDEX]->i2c_m_cmd = SCB_I2C_M_CMD_M_START;
        /* Queue the preamble byte. */
        i2cm_scb_write(USB_I2CM_SCB_INDEX, &gl_usb_i2cm.preamble, 1);
 
        /* The data transfer phase is done in the task handler. */
    }

    return status;
}

ccg_status_t usb_i2cm_vendor_rqt_handler(usb_setup_pkt_t *pkt)
{
    bool handle_status = false;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    switch (pkt->cmd)
    {
        case USB_I2CM_VDR_RQT_I2C_WRITE:
            if (gl_usb_i2cm.state == USB_I2CM_STATE_IDLE)
            {
                /* Handle write request. */
                status = usb_i2cm_rqt_write_handler(pkt);
                handle_status = true;

            }
            break;

        case USB_I2CM_VDR_RQT_I2C_READ:
            if (gl_usb_i2cm.state == USB_I2CM_STATE_IDLE)
            {
                /* Handle read request. */
                status = usb_i2cm_rqt_read_handler(pkt);
                handle_status = true;
            }
            break;

        case USB_I2CM_VDR_RQT_I2C_GET_STATUS:
            if (gl_usb_i2cm.state >= USB_I2CM_STATE_IDLE)
            {
                uint8_t buffer[3];

                buffer[0] = gl_usb_i2cm.status;
                buffer[1] = WORD_GET_LSB(gl_usb_i2cm.count);
                buffer[2] = WORD_GET_MSB(gl_usb_i2cm.count);

                /*
                 * Since there is pending I2C state to be considered,
                 * the count needs to be faked for the last byte.
                 */
                if (gl_usb_i2cm.state != USB_I2CM_STATE_IDLE)
                {
                    buffer[1] |= 1;
                }

                /* Send the status information to EP0. */
                status = usb_ep0_setup_write(buffer, sizeof(buffer), true, NULL);
            }
            break;

        case USB_I2CM_VDR_RQT_I2C_RESET:
            if (gl_usb_i2cm.state >= USB_I2CM_STATE_IDLE)
            {
                /* Handle i2c reset request. */
                i2cm_reset(USB_I2CM_SCB_INDEX);
                gl_usb_i2cm.status = 0;
                gl_usb_i2cm.state = USB_I2CM_STATE_IDLE;
                handle_status = true;
                status = CCG_STAT_SUCCESS;
            }
            break;

        default:
            /* Do nothing. */
            break;
    }

    if (handle_status != false)
    {
        if (status == CCG_STAT_SUCCESS)
        {
            status = usb_ep0_send_recv_status();
        }
        if (status != CCG_STAT_SUCCESS)
        {
            status = usb_ep0_set_stall();
        }

        /* Flush the status endpoint. */
        usb_ep_flush(USB_I2CM_INT_EP_INDEX);
    }

    return status;
}

/**
 * Disables and clear all interrupt of the scb block.
 * @param scb_index SCB block index.
 */
static void i2cm_clear_all_intr(uint8_t scb_index)
{
    PSCB_PRT_REGS_T scb_p = SCB_PRT[scb_index];

    /* Clear and disable all interrupts. It is easier to
     * clear all interrupts than to do it selectively. */
    scb_p->intr_m_mask  = 0;
    scb_p->intr_m       = 0xFFFFFFFFu;
    scb_p->intr_tx_mask = 0;
    scb_p->intr_tx      = 0xFFFFFFFFu;
    scb_p->intr_rx_mask = 0;
    scb_p->intr_rx      = 0xFFFFFFFFu;
}

/**
 * Enable the SCB block as I2C master for bridge operation. The block is expected
 * to be disabled and the IOs configured as I2C before calling this function.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 */
static void i2cm_start(uint8_t scb_index)
{
    /* First enable the I2C interface. */
    uint32_t regVal;
    PSCB_PRT_REGS_T scb_p = SCB_PRT[scb_index];

    scb_p->ctrl = 0;

    /* Configure the I2C block. */
    regVal = (SCB_I2C_CTRL_DEFAULT & (SCB_I2C_CTRL_HIGH_PHASE_OVS_MASK
                | SCB_I2C_CTRL_LOW_PHASE_OVS_MASK));
    regVal |= SCB_I2C_CTRL_M_READY_DATA_ACK;
    regVal |= SCB_I2C_CTRL_MASTER_MODE;
    scb_p->i2c_ctrl = regVal;

    scb_p->i2c_m_cmd = SCB_I2C_M_CMD_DEFAULT;

     /* Clear the TX and RX FIFO. */
    scb_p->tx_fifo_ctrl = SCB_TX_FIFO_CTRL_CLEAR;
    scb_p->rx_fifo_ctrl = SCB_RX_FIFO_CTRL_CLEAR;

    /* Clear and disable all interrupts. */
    i2cm_clear_all_intr(scb_index);

    /* Read the SCB control register value. */
    regVal = scb_p->ctrl;

    /* Enable byte mode I2C operation. */
    regVal |= SCB_CTRL_BYTE_MODE;

    /* Configure the data width and bit order for I2C. */
    scb_p->tx_ctrl = SCB_TX_CTRL_DEFAULT;
    scb_p->rx_ctrl = SCB_RX_CTRL_DEFAULT;

    /* Enable the SCB. */
    scb_p->ctrl = regVal | SCB_CTRL_ENABLED;

    /* Update the trigger levels. */
    i2c_update_tx_trig_level(scb_index, I2C_SCB_TX_TRIGGER);
    i2c_update_rx_trig_level(scb_index, I2C_SCB_RX_TRIGGER);

    /* Enable RX and TX blocks. */
    i2c_scb_rx_enable(scb_index);
    i2c_scb_tx_enable(scb_index);

    /* Disable the interrupt vector. */
    i2c_scb_vic_int_disable(scb_index);
}

/**
 * Disable the SCB block and allow normal SCB operation.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 */
static void i2cm_stop(uint8_t scb_index)
{
    PSCB_PRT_REGS_T scb_p = SCB_PRT[scb_index];

    scb_p->ctrl = 0;

    /* Clear and disable all interrupts. */
    i2cm_clear_all_intr(scb_index);
}

/**
 * @brief I2C block write function.
 *
 * The function writes the data provided into the TX FIFO for transfer.
 * The caller is expected to ensure that there is sufficient FIFO buffering
 * for the write. No parameter validation is done by the function.
 *
 * @param scb_index SCB index for the block.
 * @param src_ptr Source buffer pointer.
 * @param size Size of the copy.
 */
static void i2cm_scb_write(uint8_t scb_index, uint8_t *src_ptr, uint8_t size)
{
    uint8_t index;

    for (index = 0; index < size; index++)
    {
       SCB_PRT[scb_index]->tx_fifo_wr = src_ptr[index];
    }
}

/**
 * @brief I2C block read function.
 *
 * The function reads the data from the RX FIFO for transfer. The caller is 
 * expected to ensure that there is sufficient FIFO buffering for the read.
 * No parameter validation is done by the function.
 *
 * @param scb_index SCB index for the block.
 * @param dest_ptr Destination buffer pointer.
 * @param size Size of the copy.
 */
static void i2cm_scb_read(uint8_t scb_index, uint8_t *dest_ptr, uint8_t size)
{
    uint8_t index;

    /* Read data into the scratch buffer. */
    for (index = 0; index < size; index++)
    {
        dest_ptr[index] = SCB_PRT[scb_index]->rx_fifo_rd;
    }
}

/**
 * @brief I2C block reset to reset the state machine.
 *
 * @param scb_index SCB index for the block.
 */
static void i2cm_reset(uint8_t scb_index)
{
    PSCB_PRT_REGS_T scb_p = SCB_PRT[USB_I2CM_SCB_INDEX];

    scb_p->i2c_m_cmd = 0;
    scb_p->ctrl &= ~SCB_CTRL_ENABLED;
    scb_p->ctrl |= SCB_CTRL_ENABLED;
}

/**
 * @brief Update the I2C state in the bridge status variable.
 *
 * @param scb_index SCB index for the block.
 */
static void i2cm_update_status(uint8_t scb_index)
{
    PSCB_PRT_REGS_T scb_p = SCB_PRT[USB_I2CM_SCB_INDEX];

    if (scb_p->intr_m & SCB_INTR_M_I2C_ARB_LOST)
    {
        gl_usb_i2cm.status |= USB_I2CM_STAT_ARB_ERR;
    }
    if (scb_p->intr_m & SCB_INTR_M_I2C_NACK)
    {
        gl_usb_i2cm.status |= USB_I2CM_STAT_NAK;
    }
    if (scb_p->intr_m & SCB_INTR_M_I2C_BUS_ERROR)
    {
        gl_usb_i2cm.status |= USB_I2CM_STAT_BUS_ERR;
    }
    if (scb_p->i2c_status & SCB_I2C_STATUS_BUS_BUSY)
    {
        gl_usb_i2cm.status |= USB_I2CM_STAT_BUS_BUSY;
    }
    if (scb_p->i2c_status & SCB_I2C_STATUS_BUS_BUSY)
    {
        gl_usb_i2cm.status |= USB_I2CM_STAT_BUS_BUSY;
    }
}

/**
 * @brief Load status data into the interrupt endpoint.
 */
static void usb_i2cm_send_status(void)
{
    ccg_status_t status;

    gl_status_buffer[0] = gl_usb_i2cm.status;
    gl_status_buffer[1] = WORD_GET_LSB(gl_usb_i2cm.count);
    gl_status_buffer[2] = WORD_GET_MSB(gl_usb_i2cm.count);

    status = usb_ep_write_single(USB_I2CM_INT_EP_INDEX, gl_status_buffer,
            sizeof(gl_status_buffer));
    if (status != CCG_STAT_SUCCESS)
    {
        usb_ep_flush(USB_I2CM_INT_EP_INDEX);
    }
}

/**
 * @brief Write task function which handles I2C write state machine. The
 * function should be called only from the bridge task function.
 */
static void usb_i2cm_write_task(void)
{
    PSCB_PRT_REGS_T scb_p = SCB_PRT[USB_I2CM_SCB_INDEX];
    bool ep_state = usb_ep_is_ready(USB_I2CM_OUT_EP_INDEX);
    ccg_status_t status = CCG_STAT_SUCCESS;
    uint8_t fifosize = I2C_SCB_FIFO_SIZE - (scb_p->tx_fifo_status
            & SCB_TX_FIFO_STATUS_USED_MASK);
    uint8_t ep_len;
    uint16_t cur_len;

    if (gl_usb_i2cm.preamble_state != false)
    {
        /* This is the first state. Wait for the USB data. */
        if (ep_state == false)
        {
            return;
        }

        /* Data is now available. Copy the preamble and the data. */
        status = usb_ep_read_single(USB_I2CM_OUT_EP_INDEX, gl_buffer, &ep_len);
        if (status == CCG_STAT_SUCCESS)
        {
            /* Setup the I2C block. */
            scb_p->intr_m = ~0;
            /* First load the preamble byte. */
            i2cm_scb_write(USB_I2CM_SCB_INDEX, &gl_usb_i2cm.preamble, 1);
            /* Now copy the data into the TX FIFO. */
            cur_len = GET_MIN(gl_usb_i2cm.count, ep_len);
            i2cm_scb_write(USB_I2CM_SCB_INDEX, gl_buffer, cur_len);

            /* Now update the state machine flags. */
            gl_usb_i2cm.preamble_state = false;
            gl_usb_i2cm.count -= cur_len;

            /* Check the control information and load the command. */
            scb_p->i2c_m_cmd = SCB_I2C_M_CMD_M_START;

            /*
             * Now queue a subsequent read on EP if there is further data
             * to be received.
             */
            if (gl_usb_i2cm.count != 0)
            {
                status = usb_ep_queue_read_single(USB_I2CM_OUT_EP_INDEX);
            }
        }
    }
    else
    {
        /* Read once to prevent race. */
        uint32_t intr_m_status = scb_p->intr_m;

        /* First check if there is any error in the I2C bus. */
        if (intr_m_status & (SCB_INTR_M_I2C_ARB_LOST | SCB_INTR_M_I2C_NACK |
                    SCB_INTR_M_I2C_BUS_ERROR))
        {
            status = CCG_STAT_FAILURE;
        }

        if (status == CCG_STAT_SUCCESS)
        {
            /* Now check if the transfer is completed successfully. */
            if (gl_usb_i2cm.count == 0)
            {
                /*
                 * Check if the start bit has been sent out. The stop bit
                 * setting should be done only after sending start as hardware
                 * prioritizes the stop bit over the start bit.
                 */
                if (scb_p->i2c_m_cmd == 0)
                {
                    /* Trigger the stop bit if required. */
                    if (gl_usb_i2cm.ctrl & USB_I2CM_CTRL_STOP)
                    {
                        scb_p->i2c_m_cmd = SCB_I2C_M_CMD_M_STOP;
                        /* Remove the flag to prevent race. */
                        gl_usb_i2cm.ctrl &= ~USB_I2CM_CTRL_STOP;
                    }
                }

                /*
                 * The last of the data has been loaded. Now we need to
                 * wait for the I2C bus transfer to complete.
                 */
                if (((scb_p->tx_fifo_status & (SCB_TX_FIFO_STATUS_USED_MASK |
                                    SCB_TX_FIFO_STATUS_SR_VALID)) == 0) &&
                        (((gl_usb_i2cm.ctrl & USB_I2CM_CTRL_STOP) == 0) ||
                            (intr_m_status & SCB_INTR_M_I2C_STOP)) &&
                        (scb_p->i2c_m_cmd == 0))
                {
                    /* The transfer has fully completed. */
                    gl_usb_i2cm.status &= ~USB_I2CM_STAT_ACTIVE;
                    gl_usb_i2cm.state = USB_I2CM_STATE_IDLE;
                    /* Send the status information. */
                    usb_i2cm_send_status();
                }
            }
            else
            {
                /*
                 * If both EP data and FIFO buffering are available, then
                 * load the data from EP buffer into TX FIFO.
                 */
                if ((ep_state != false) && (fifosize > I2C_SCB_TX_TRIGGER))
                {
                    status = usb_ep_read_single(USB_I2CM_OUT_EP_INDEX,
                            gl_buffer, &ep_len);
                    if (status == CCG_STAT_SUCCESS)
                    {
                        /* Now copy the data into the TX FIFO. */
                        cur_len = GET_MIN(gl_usb_i2cm.count, ep_len);
                        i2cm_scb_write(USB_I2CM_SCB_INDEX, gl_buffer, cur_len);
                        gl_usb_i2cm.count -= cur_len;

                        /*
                         * Now queue a subsequent read on EP if there is 
                         * further data to be received.
                         */
                        if (gl_usb_i2cm.count != 0)
                        {
                            status = usb_ep_queue_read_single(USB_I2CM_OUT_EP_INDEX);
                        }
                    }
                }
            }
        }
    }

    /* Handle any error situation. */
    if (status != CCG_STAT_SUCCESS)
    {
        /* If there is pending data to be received, stall the OUT ep. */
        if (gl_usb_i2cm.count != 0)
        {
            usb_ep_set_stall(USB_I2CM_OUT_EP_INDEX);
        }
        /* Now update the device status. */
        i2cm_update_status(USB_I2CM_SCB_INDEX);

        /* Move to error state. */
        gl_usb_i2cm.state = USB_I2CM_STATE_ERROR;

        /* Clean up the I2C state. */
        i2cm_reset(USB_I2CM_SCB_INDEX);

        /* Send the status on the interrupt endpoint. */
        usb_i2cm_send_status();
    }
}

/**
 * @brief Read task function which handles I2C read state machine. The
 * function should be called only from the bridge task function.
 */
static void usb_i2cm_read_task(void)
{
    PSCB_PRT_REGS_T scb_p = SCB_PRT[USB_I2CM_SCB_INDEX];
    bool ep_state = usb_ep_is_ready(USB_I2CM_IN_EP_INDEX);
    ccg_status_t status = CCG_STAT_SUCCESS;
    uint8_t fifosize = (scb_p->rx_fifo_status & SCB_RX_FIFO_STATUS_USED_MASK);
    uint16_t cur_len;
    uint32_t intr_m_status;

    /* Read once to prevent race. */
    intr_m_status = scb_p->intr_m;

    if (gl_usb_i2cm.preamble_state != false)
    {
        /* First check for any error in preamble transfer. */
        if (intr_m_status & (SCB_INTR_M_I2C_ARB_LOST | SCB_INTR_M_I2C_NACK |
                    SCB_INTR_M_I2C_BUS_ERROR))
        {
            status = CCG_STAT_FAILURE;
        }

        /* Now check if preamble transfer completed successfully. */
        if (intr_m_status & SCB_INTR_M_I2C_ACK)
        {
            gl_usb_i2cm.preamble_state = false;
        }
    }
    else
    {
        /* First check if there is any error in the I2C bus. */
        if (intr_m_status & (SCB_INTR_M_I2C_ARB_LOST | SCB_INTR_M_I2C_NACK |
                    SCB_INTR_M_I2C_BUS_ERROR))
        {
            status = CCG_STAT_FAILURE;
        }

        if (status == CCG_STAT_SUCCESS)
        {
            /* Now check if the transfer is completed successfully. */
            if (gl_usb_i2cm.count == 0)
            {
                if ((intr_m_status & SCB_INTR_M_I2C_STOP) &&
                        (scb_p->i2c_m_cmd == 0))
                {
                    /*
                     * The transfer has fully completed. Clean up the receive
                     * FIFO and update the state.
                     */
                    i2c_scb_rx_disable(USB_I2CM_SCB_INDEX);
                    i2c_scb_rx_enable(USB_I2CM_SCB_INDEX);
                    gl_usb_i2cm.status &= ~USB_I2CM_STAT_ACTIVE;
                    gl_usb_i2cm.state = USB_I2CM_STATE_IDLE;
                    /* Send the status information. */
                    usb_i2cm_send_status();
                }
            }
            else
            {
                /*
                 * If both EP data and FIFO buffering are available, then
                 * load the data from RX FIFO to ep buffer.
                 */
                cur_len = GET_MIN(gl_usb_i2cm.count, I2C_SCB_RX_TRIGGER);
                if ((ep_state != false) && (fifosize > cur_len))
                {
                    i2cm_scb_read(USB_I2CM_SCB_INDEX, gl_buffer, cur_len);
                    gl_usb_i2cm.count -= cur_len;
                    /* Check for end of transfer and signal the same. */
                    if (gl_usb_i2cm.count == 0)
                    {
                        /*
                         * Since the functions do not allow continued read,
                         * signal a stop without looking at the control 
                         * information.
                         */
                        scb_p->i2c_m_cmd = (SCB_I2C_M_CMD_M_NACK |
                                SCB_I2C_M_CMD_M_STOP);
                    }
                    status = usb_ep_write_single(USB_I2CM_IN_EP_INDEX,
                            gl_buffer, cur_len);
                }
            }
        }
    }

    /* Handle any error situation. */
    if (status != CCG_STAT_SUCCESS)
    {
        /* If there is pending data to be received, stall the OUT ep. */
        if (gl_usb_i2cm.count != 0)
        {
            usb_ep_set_stall(USB_I2CM_IN_EP_INDEX);
        }

        /* Now update the device status. */
        i2cm_update_status(USB_I2CM_SCB_INDEX);

        /* Move to error state. */
        gl_usb_i2cm.state = USB_I2CM_STATE_ERROR;

        /* Clean up the I2C state. */
        i2cm_reset(USB_I2CM_SCB_INDEX);

        /* Queue the status on the interrupt endpoint. */
        usb_i2cm_send_status();
    }
}

void usb_i2cm_task(void)
{
    switch (gl_usb_i2cm.state)
    {
        case USB_I2CM_STATE_WRITE:
            usb_i2cm_write_task();
            break;

        case USB_I2CM_STATE_READ:
            usb_i2cm_read_task();
            break;

        default:
            /* Do nothing. */
            break;
    }
}

#endif /* (APP_I2CM_BRIDGE_ENABLE) */

/* [] */

