/**
 * @file ridge_slave.c
 *
 * @brief @{Alpine/Titan Ridge I2C slave interface source file.@}
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

#include "i2c.h"
#include "ridge_slave.h"
#include "utils.h"
#include "pd.h"
#include "dpm.h"
#include "psource.h"
#include "intel_ridge.h"
#include "timer.h"
#include "pdss_hal.h"
#include "app.h"

/* SCB used for the Alpine/Titan Ridge Slave interface. */
static uint8_t            ridge_slave_scb_idx = RIDGE_SLAVE_SCB_INDEX;

/* Command register for each PD port. */
static volatile uint32_t  ridge_slave_cmd_reg[NO_OF_TYPEC_PORTS];

/* Status register for each PD port. */
static volatile uint32_t  ridge_slave_stat_reg[NO_OF_TYPEC_PORTS];

/* Scratch buffer used to hold incoming data from Alpine/Titan Ridge. */
static uint8_t            ridge_slave_write_buf[RIDGE_SLAVE_MAX_WRITE_SIZE];

/* Scratch buffer used to provide read data back to Alpine/Titan Ridge. */
static uint8_t            ridge_slave_read_buf[RIDGE_SLAVE_MAX_READ_SIZE];

/* Current read pointer. */
static volatile uint8_t  *ridge_slave_read_ptr = 0;

/* Limit upto which I2C data read is allowed. */
static volatile uint8_t  *ridge_slave_read_limit = 0;

/* Flag indicating ridge slave task pending status. */
static volatile bool      ridge_slave_task_pending = false;

#if RIDGE_I2C_HPD_ENABLE
/* Flag indicating that HPD dequeueing process should be run. */
static volatile bool      hpd_run[NO_OF_TYPEC_PORTS];
#endif /* RIDGE_I2C_HPD_ENABLE */

#if ((defined(CCG5)) && (DELAYED_TBT_LSXX_CONNECT))
/* Function to enable the TBT-LSXX connections after a delay. */
static void sbu_delayed_connect_cb (uint8_t port, uint8_t timer_id)
{
#if SBU_MUX_PASSTHROUGH
    sbu_switch_configure (port, SBU_CONNECT_AUX1, SBU_CONNECT_AUX2);
#else
    if (dpm_get_info(port)->polarity)
    {
        sbu_switch_configure (port, SBU_CONNECT_LSRX, SBU_CONNECT_LSTX);
    }
    else
    {
        sbu_switch_configure (port, SBU_CONNECT_LSTX, SBU_CONNECT_LSRX);
    }
#endif
}
#endif /* ((defined(CCG5)) && (DELAYED_TBT_LSXX_CONNECT)) */

void ridge_reg_reset(uint8_t port)
{
	ridge_slave_cmd_reg[port] = NO_DATA;
	ridge_slave_stat_reg[port] = NO_DATA;
}

/* Function called by Alt. Mode layer to update the command register. */
void ridge_slave_cmd_update(uint8_t  port, uint32_t cmd)
{
    if (port == 0)
    {
        /* Update the command register value, clear the Interrupt ACK bit and raise the interrupt. */
        if (ridge_slave_cmd_reg[0] != cmd)
        {
            ridge_slave_cmd_reg[0] |= cmd;
            AR_INT_P1_Write (0);
        }
    }

#if CCG_PD_DUALPORT_ENABLE
    if (port == 1)
    {
        /* Update the command register value, clear the Interrupt ACK bit and raise the interrupt. */
        if (ridge_slave_cmd_reg[1] != cmd)
        {
            ridge_slave_cmd_reg[1]  |= cmd;
            AR_INT_P2_Write (0);
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

/* Function called by Alt. Mode layer to update the status register. */
void ridge_slave_status_update(uint8_t  port, uint32_t status, bool rewrite)
{
    if (port == 0)
    {
        /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
        if ((ridge_slave_stat_reg[0] != status) || (rewrite))
        {
            ridge_slave_stat_reg[0]  = status;
            AR_INT_P1_Write (0);
        }
    }

#if CCG_PD_DUALPORT_ENABLE
    if (port == 1)
    {
        /* Update the status register value, clear the Interrupt ACK bit and raise the interrupt. */
        if ((ridge_slave_stat_reg[1] != status) || (rewrite))
        {
            ridge_slave_stat_reg[1]  = status;
            ridge_slave_cmd_reg[1]  &= ~RIDGE_CMD_INT_CLEAR;
            AR_INT_P2_Write (0);
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

void ridge_slave_update_ocp_status(uint8_t port, bool status)
{
    uint32_t mask = 0;

    if (port < NO_OF_TYPEC_PORTS)
    {
        /* Check if the OCP status has changed and flag interrupt if so. */
        if (status)
            mask = RIDGE_STATUS_OCP_MASK;
        if ((ridge_slave_stat_reg[port] & RIDGE_STATUS_OCP_MASK) != mask)
        {
            ridge_slave_status_update (port, (ridge_slave_stat_reg[port] ^ mask), false); 
        }
    }
}

void ridge_slave_task(void)
{
    uint32_t cmd;
    uint8_t  port = 0;
    uint8_t  regaddrloc = 1;
    uint8_t  intr_state;

    intr_state = CyEnterCriticalSection ();

    if (ridge_slave_task_pending)
    {
#if CCG_PD_DUALPORT_ENABLE
        /* Select the port being read/written. */
        switch (ridge_slave_write_buf[0] >> 1)
        {
            case RIDGE_SLAVE_ADDR_P0:
                port = 0;
                break;
            case RIDGE_SLAVE_ADDR_P1:
                port = 1;
                break;
            default:
                /* Re-enable slave ACK. */
                i2c_slave_ack_ctrl (ridge_slave_scb_idx, false);
                ridge_slave_task_pending = false;
                CyExitCriticalSection (intr_state);
                return;
        }
#else
        port = 0;
        regaddrloc = 0;
#endif

        /* Valid write request. The only write-able register is the command register. */
        if (ridge_slave_write_buf[regaddrloc] == RIDGE_REG_CCG_COMMAND)
        {
            /* Handle soft reset request. */
            if ((ridge_slave_write_buf[regaddrloc + 2] & RIDGE_CMD_CCG_RESET) != 0)
            {
                AR_INT_P1_Write (1);
#if CCG_PD_DUALPORT_ENABLE
                AR_INT_P2_Write (1);
#endif /* CCG_PD_DUALPORT_ENABLE */

                CyExitCriticalSection (intr_state);
                CySoftwareReset ();
            }

            /* Handle interrupt clear command. */
            if ((ridge_slave_write_buf[regaddrloc + 2] & RIDGE_CMD_INT_CLEAR) != 0)
            {
#if CCG_PD_DUALPORT_ENABLE
                if (port == 0)
#endif /* CCG_PD_DUALPORT_ENABLE */
                {
                    AR_INT_P1_Write (1);
                }
#if CCG_PD_DUALPORT_ENABLE
                else
                {
                    AR_INT_P2_Write (1);
                }
#endif /* CCG_PD_DUALPORT_ENABLE */

#if RIDGE_I2C_HPD_ENABLE
                /* Check if clear interrupt is the response to DP event */
                if ((tr_is_hpd_change(port) != false) && (dpm_get_info(port)->cur_port_type != PRT_TYPE_UFP))
                {
                    /*
                     * If IRQ_ACKfmTR bit is set then set IRQ_HPDStickyfmTR
                     * and alert Titan Ridge that CCG was notified about successful
                     * Titan Ridge DP IRQ retransmission
                     */
                    if (ridge_slave_write_buf[regaddrloc + 3] & (TR_IRQ_ACK_MASK >> 8))
                    {
                        /* Set IRQ_HPDStickyfmTR bit to zero and alert TR */
                        ridge_slave_status_update(port, (ridge_slave_stat_reg[port] & (~TR_IRQ_ACK_MASK)), false);
                    }
                    else
                    {
                        /* HPD event processed successfully - goto HPD queue */
                        hpd_run[port] = true;
                    }
                }
#endif /* RIDGE_I2C_HPD_ENABLE */
            } 

            {
                /* Check if TR command changed from the previous time */
                cmd = MAKE_DWORD (0, 0, ridge_slave_write_buf[regaddrloc + 3], ridge_slave_write_buf[regaddrloc + 2]);
                cmd &= ~RIDGE_CMD_INT_CLEAR;

                if (ridge_slave_cmd_reg[port] != cmd)
                {
                        /* Create variable to hold mask of changed command bits */
                        uint32_t tmp_mask = 0;

                        /* Create bit mask of changed command bits */
                        tmp_mask = (ridge_slave_cmd_reg[port] ^ cmd);
                        /* Save received Titan Ridge command */
                        ridge_slave_cmd_reg[port] = cmd;

                    /* Evaluate received command register update. */
                    ridge_eval_cmd(port, ridge_slave_cmd_reg[port], tmp_mask);
                    }

                if (port == 0)
                {
                    AR_INT_P1_Write (1);
                }
#if CCG_PD_DUALPORT_ENABLE
                if (port != 0)
                {
                    AR_INT_P2_Write (1);
                }
#endif /* CCG_PD_DUALPORT_ENABLE */
            }
        }

        /* Re-enable slave ACK. */
        i2c_slave_ack_ctrl (ridge_slave_scb_idx, false);
        ridge_slave_task_pending = false;
    }

    CyExitCriticalSection (intr_state);

#if RIDGE_I2C_HPD_ENABLE
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        if (hpd_run[port] != false)
        {
            tr_hpd_sendevt(port, HPD_COMMAND_DONE);        
            hpd_run[port] = false;
        }
    }
#endif /* RIDGE_I2C_HPD_ENABLE */
}

/*
 * @brief I2C command callback function that implements the actual Alpine/Titan Ridge
 * interface logic. This is called from SCB interrupt handler. Since the work
 * to be done is limited, it is completely handled from the callback.
 *
 * @param cmd I2C operation that caused this callback.
 * @param i2c_state Current state of the I2C slave state machine.
 * @param count Size of data written in the case of a write command.
 *
 * @return true if the command is valid, false if it is invalid.
 */
static bool ridge_slave_i2c_cmd_callback(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count)
{
    bool retval = false;
    uint16_t size;
    uint8_t temp, port = 0;
    uint8_t regaddrloc = 1;

    (void)i2c_state;

    if (cmd == I2C_CB_CMD_WRITE)
    {
        /* Hold off further I2C transactions while handling this write. */
        i2c_slave_ack_ctrl (ridge_slave_scb_idx, true);

#if CCG_PD_DUALPORT_ENABLE
        /* Select the port being read/written. */
        switch (ridge_slave_write_buf[0] >> 1)
        {
            case RIDGE_SLAVE_ADDR_P0:
                port = 0;
                break;
            case RIDGE_SLAVE_ADDR_P1:
                port = 1;
                break;
            default:
                /* Re-enable slave ACK. */
                i2c_slave_ack_ctrl (ridge_slave_scb_idx, false);
                return false;
        }
#else
        port = 0;
        regaddrloc = 0;
        count++;
#endif

        /* If we get a single byte only, allow the existing data in the read buf to be read. */
        switch (count)
        {
            case 0:
            case 1:
                /* This will never happen. */
                retval = true;
                break;

            case 2:
            case 3:
                /* Only address has been set. Set the read pointer and return. */
                {
                    /* Start with an invalid address. */
                    ridge_slave_read_ptr   = 0;
                    ridge_slave_read_limit = 0;

                    /* Clear the read data to start with. */
                    memset ((void *)ridge_slave_read_buf, 0, sizeof (ridge_slave_read_buf));
                    switch (ridge_slave_write_buf[regaddrloc])
                    {
                        case RIDGE_REG_CCG_COMMAND:
                            /* Copy in the relevant command register value. */
                            ridge_slave_read_buf[0] = 0x04;
                            ridge_slave_read_buf[1] = DWORD_GET_BYTE0 (ridge_slave_cmd_reg[port]);
                            ridge_slave_read_buf[2] = DWORD_GET_BYTE1 (ridge_slave_cmd_reg[port]);
                            ridge_slave_read_buf[3] = DWORD_GET_BYTE2 (ridge_slave_cmd_reg[port]);
                            ridge_slave_read_buf[4] = DWORD_GET_BYTE3 (ridge_slave_cmd_reg[port]);
                            break;

                        case RIDGE_REG_CCG_STATUS:
                            /* Copy in the relevant command register value. */
                            ridge_slave_read_buf[0] = 0x04;
                            ridge_slave_read_buf[1] = DWORD_GET_BYTE0 (ridge_slave_stat_reg[port]);
                            ridge_slave_read_buf[2] = DWORD_GET_BYTE1 (ridge_slave_stat_reg[port]);
                            ridge_slave_read_buf[3] = DWORD_GET_BYTE2 (ridge_slave_stat_reg[port]);
                            ridge_slave_read_buf[4] = DWORD_GET_BYTE3 (ridge_slave_stat_reg[port]);

#if ((defined(CCG5)) && (DELAYED_TBT_LSXX_CONNECT))
                            /*
                               Status has been provided to the Ridge. We can now start a timer to enable
                               the SBU connections where required.
                             */
                            if (ridge_slave_read_buf[3] != 0)
                            {
                                timer_start(port, APP_SBU_DELAYED_CONNECT_TIMER, APP_SBU_DELAYED_CONNECT_PERIOD,
                                        sbu_delayed_connect_cb);
                            }
                            else
                            {
                                timer_stop(port, APP_SBU_DELAYED_CONNECT_TIMER);
                            }
#endif /* ((defined(CCG5)) && (DELAYED_TBT_LSXX_CONNECT)) */
                            break;

                        default:
                            i2c_slave_ack_ctrl (ridge_slave_scb_idx, false);
                            return false;
                    }

                    /* Set the valid address range for the read operation. */
                    ridge_slave_read_ptr   = (uint8_t *)ridge_slave_read_buf;
                    ridge_slave_read_limit = (uint8_t *)(ridge_slave_read_buf + RIDGE_SLAVE_MAX_READ_SIZE);
                }

                retval = true;
                break;

            default:
                ridge_slave_task_pending = true;
                retval = true;
                break;
        }

        /* Re-enable slave ACK. */
        if (!ridge_slave_task_pending)
        {
            i2c_slave_ack_ctrl (ridge_slave_scb_idx, false);
        }
    }

    if (cmd == I2C_CB_CMD_READ)
    {
        /* Read handler: return data while it is available. */
        if ((ridge_slave_read_ptr != 0) && (ridge_slave_read_ptr < ridge_slave_read_limit))
        {
            size = GET_MIN (I2C_SCB_TX_FIFO_SIZE, (unsigned int)(ridge_slave_read_limit - ridge_slave_read_ptr));
            if (size != 0)
            {
                i2c_scb_write (ridge_slave_scb_idx, (uint8_t *)ridge_slave_read_ptr, (uint8_t)size, &temp);
                ridge_slave_read_ptr += temp;
                retval = true;
            }
        }
    }

    if (cmd == I2C_CB_CMD_XFER_END)
    {
        /* Just roll the read pointer back to the base of the read buffer. */
        ridge_slave_read_ptr   = (uint8_t *)ridge_slave_read_buf;
        ridge_slave_read_limit = (uint8_t *)(ridge_slave_read_buf + RIDGE_SLAVE_MAX_READ_SIZE);
        retval = true;
    }

    return retval;
}

/* Alpine/Titan Ridge interface initialization. */
void ridge_slave_init(uint8_t ridge_scbnum, uint8_t portsel)
{
    /* Clear all structures and status. */
    memset ((void *)ridge_slave_write_buf, 0, RIDGE_SLAVE_MAX_WRITE_SIZE);
    memset ((void *)ridge_slave_stat_reg, 0, sizeof (ridge_slave_stat_reg));
    memset ((void *)ridge_slave_cmd_reg, 0, sizeof (ridge_slave_cmd_reg));

    ridge_slave_scb_idx    = ridge_scbnum;
    ridge_slave_read_ptr   = 0;
    ridge_slave_read_limit = 0;

#if CCG_PD_DUALPORT_ENABLE
    i2c_scb_init (ridge_slave_scb_idx, I2C_SCB_MODE_ALP_RIDGE, RIDGE_SLAVE_SCB_CLOCK_FREQ,
            RIDGE_SLAVE_ADDR_P0, RIDGE_SLAVE_ADDR_MASK,
            ridge_slave_i2c_cmd_callback, ridge_slave_write_buf, RIDGE_SLAVE_MAX_WRITE_SIZE);
#else
    i2c_scb_init (ridge_slave_scb_idx, I2C_SCB_MODE_ALP_RIDGE, RIDGE_SLAVE_SCB_CLOCK_FREQ,
            ((portsel != 0) ? RIDGE_SLAVE_ADDR_P1 : RIDGE_SLAVE_ADDR_P0), I2C_SLAVE_ADDR_MASK_DEFAULT,
            ridge_slave_i2c_cmd_callback, ridge_slave_write_buf, RIDGE_SLAVE_MAX_WRITE_SIZE);
#endif

    /* De-assert the interrupt at start-up. */
    AR_INT_P1_Write (1);

#if RIDGE_I2C_HPD_ENABLE
    hpd_run[0] = false;
#endif /* RIDGE_I2C_HPD_ENABLE */

#if CCG_PD_DUALPORT_ENABLE
    AR_INT_P2_Write (1);

#if RIDGE_I2C_HPD_ENABLE
    hpd_run[1] = false;
#endif /* RIDGE_I2C_HPD_ENABLE */
#endif /* CCG_PD_DUALPORT_ENABLE */
}

/* Check if the Alpine/Titan Ridge interface is idle and prepare for deep sleep. */
bool ridge_slave_sleep(void)
{
    /* Assume that we can sleep by default. */
    bool stat = true;

    /* Sleep is not if the I2C interface is busy. */
    if ((ridge_slave_task_pending) || (!i2c_scb_is_idle (ridge_slave_scb_idx)))
    {
        stat = false;
    }
    else
    {
        /* Enable wakeup due to HPI activity. */
        i2c_scb_enable_wakeup (ridge_slave_scb_idx);
    }

    return stat;
}

bool ridge_slave_is_host_connected(uint8_t port)
{
    bool ret = false;

    if (port < NO_OF_TYPEC_PORTS)
    {
        /* Return whether the TBT_Host_Connected bit has been set. */
        if ((ridge_slave_cmd_reg[port] & TBT_HOST_CONN_MASK) != 0)
        {
            ret = true;
        }
    }

    return ret;
}

/* [] END OF FILE */

