/**
 * @file solution.c
 *
 * @brief @{Solution source file for one port CCG5 NON-TBT solution layer port.@}
 *
 *******************************************************************************
 *
 * Copyright (2016-2018), Cypress Semiconductor Corporation or a subsidiary of
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

#include <pd.h>
#include <timer.h>
#include <dpm.h>
#include <app.h>
#include <psource.h>
#include <solution.h>
#include <NCP81239_regs.h>
#include <pdss_hal.h>

#define MAKE_MUX_STATE(polarity,cfg)    ((uint8_t)(polarity << 7) | (cfg & 0x7F))

/* MUX access timeout indication. */
volatile bool mux_xfer_timeout = false;

/* I2C slave address assigned to PI3DPX1205A data mux for each PD port. */
static const uint8_t mux_addr[NO_OF_TYPEC_PORTS] =
{
    MUX_PORT_1_SLAVE_ADDR
#if CCG_PD_DUALPORT_ENABLE
        ,
    MUX_PORT_2_SLAVE_ADDR
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* Current mux state. */
static volatile uint8_t mux_state[NO_OF_TYPEC_PORTS] =
{
    MUX_CONFIG_DEINIT
#if CCG_PD_DUALPORT_ENABLE
        ,
    MUX_CONFIG_DEINIT
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* I2C slave address assigned to NCP81239 controller for each PD port. */
static const uint8_t pd_ctrl_addr[NO_OF_TYPEC_PORTS] =
{
    NCP_PORT_1_SLAVE_ADDR
#if CCG_PD_DUALPORT_ENABLE
        ,
    NCP_PORT_2_SLAVE_ADDR
#endif /* CCG_PD_DUALPORT_ENABLE */
};

uint8_t pd_ctrl_config[][2] =
{
#if CCG_FRS_RX_ENABLE
    {PD_CTRL_VPS_REG_ADDR, PD_CTRL_VPS_5V},
#else /* !CCG_FRS_RX_ENABLE */
    {PD_CTRL_VPS_REG_ADDR, PD_CTRL_VPS_0V},
#endif /* CCG_FRS_RX_ENABLE */

    {PD_CTRL_SKEW_RATE_REG_ADDR, PD_CTRL_SKEW_RATE_4_9_MV_US}
};

/* Timer callback used for I2C transactions to the MUX. */
void mux_xfer_timer_cb(uint8_t port, timer_id_t id)
{
    (void)port;
    (void)id;

    /*
     * I2C transmission to MUX continues longer than timeout. Slave doesn't
     * respond.
     */
    mux_xfer_timeout = true;
}

/* Configure the I2C interface. */
void i2cm_init(void)
{
    I2C_MSTR_CTRL_REG     = I2C_MSTR_I2C_DEFAULT_CTRL;
    I2C_MSTR_I2C_CTRL_REG = I2C_MSTR_I2C_DEFAULT_I2C_CTRL;

    /* Configure TX direction. */
    I2C_MSTR_TX_CTRL_REG      = I2C_MSTR_I2C_DEFAULT_TX_CTRL;
    I2C_MSTR_TX_FIFO_CTRL_REG = I2C_MSTR_I2C_DEFAULT_TX_FIFO_CTRL;

    I2C_MSTR_CTRL_REG |= I2C_MSTR_CTRL_ENABLED;
}

/* Sending Data to I2C device */
bool I2C_Write( uint8_t addr, uint8_t *buffer, uint32_t count)
{
    uint32_t i;
    uint8_t  status = false;

    /* Clear the timeout flag and start a timer. */
    mux_xfer_timeout = false;
    timer_start (0, MUX_I2C_TIMER, MUX_I2C_TIMER_PERIOD, mux_xfer_timer_cb);

    /* If the bus is free, generate a Start condition. */
    if ((I2C_MSTR_I2C_STATUS_REG & I2C_MSTR_I2C_STATUS_BUS_BUSY) == 0)
    {
        /* Assume operation passed for now. */
        status = true;

        /* TX and RX FIFO have to be EMPTY. */
        I2C_MSTR_TX_FIFO_WR_REG = (uint32_t)(addr << 1); /* Put address in TX FIFO. */
        I2C_MSTR_ClearMasterInterruptSource (I2C_MSTR_INTR_MASTER_ALL);
        I2C_MSTR_I2C_MASTER_GENERATE_START;

        /* Wait for an ACK from the MUX. */
        while (!I2C_MSTR_CHECK_INTR_MASTER(I2C_MSTR_INTR_MASTER_I2C_BUS_ERROR |
                    I2C_MSTR_INTR_MASTER_I2C_ACK |
                    I2C_MSTR_INTR_MASTER_I2C_NACK |
                    I2C_MSTR_INTR_MASTER_I2C_ARB_LOST) &&
                (!mux_xfer_timeout));

        /* Transfer the remaining data out to the MUX. */
        for (i = 0; ((!mux_xfer_timeout) && (i < count)); i++)
        {
            /* Clear ACK interrupt from earlier. */
            I2C_MSTR_CLEAR_INTR_MASTER (I2C_MSTR_INTR_MASTER_I2C_ACK);

            I2C_MSTR_TX_FIFO_WR_REG = buffer[i];
            while (!I2C_MSTR_CHECK_INTR_MASTER(I2C_MSTR_INTR_MASTER_I2C_BUS_ERROR|
                        I2C_MSTR_INTR_MASTER_I2C_ACK |
                        I2C_MSTR_INTR_MASTER_I2C_NACK |
                        I2C_MSTR_INTR_MASTER_I2C_ARB_LOST) &&
                    (!mux_xfer_timeout));
        }
    }

    /* Send a STOP to the slave. */
    I2C_MSTR_I2C_MASTER_GENERATE_STOP;
    while (!I2C_MSTR_CHECK_INTR_MASTER(I2C_MSTR_INTR_MASTER_I2C_STOP |
                I2C_MSTR_INTR_MASTER_I2C_ARB_LOST |
                I2C_MSTR_INTR_MASTER_I2C_BUS_ERROR) &&
            (!mux_xfer_timeout));

    /* Check the results of the address phase. */
    if (
            I2C_MSTR_CHECK_INTR_MASTER (I2C_MSTR_INTR_MASTER_I2C_NACK |
                I2C_MSTR_INTR_MASTER_I2C_ARB_LOST |
                I2C_MSTR_INTR_MASTER_I2C_BUS_ERROR) ||
            (mux_xfer_timeout) ||
            (status == false)
       )
    {
        /* Transaction failed. Reset the SCB block and return error. */
        I2C_MSTR_CTRL_REG &= ((uint32) ~I2C_MSTR_CTRL_ENABLED);
        I2C_MSTR_CTRL_REG |= ((uint32)  I2C_MSTR_CTRL_ENABLED);
        status = false;
    }

    timer_stop (0, MUX_I2C_TIMER);
    return (status);
}

/* Update the data mux settings as required. */
bool mux_ctrl_set_cfg(uint8_t port, mux_select_t cfg, uint8_t polarity)
{
    /* PI3DPX1205A write buffer. First 3 bytes are read-only and not influence to MUX operation */
    uint8_t    tmp_buf[MUX_BUFF_SIZE] = {0x00, 0x00, 0x00, 0x00};
    bool       status = false;
    dpdm_mux_cfg_t     dpdm_conf;
    sbu_switch_state_t sbu1_conf = SBU_NOT_CONNECTED;
    sbu_switch_state_t sbu2_conf = SBU_NOT_CONNECTED;

    /* Compare  new and recent mux states. Write to the MUX only if the config is changing */
    if ((port < NO_OF_TYPEC_PORTS) && (MAKE_MUX_STATE(polarity, cfg) != mux_state[port]))
    {
        /* Enable USB 2.0 connections through the appropriate signals. */
        if (polarity)
            dpdm_conf = DPDM_MUX_CONN_USB_BOT_UART;
        else
            dpdm_conf = DPDM_MUX_CONN_USB_TOP_UART;

        switch(cfg)
        {
            case MUX_CONFIG_SS_ONLY:
                /* Set mux to USB-only mode state. */
                tmp_buf[MUX_CONFIG_IDX] = MUX_USB_SS;
                break;

            case MUX_CONFIG_DP_4_LANE:
                /* Set mux to 4-lane DP mode state. */
                tmp_buf[MUX_CONFIG_IDX] = MUX_DP_4_LANE;
                if (polarity)
                {
                    /* Flipped connection: SBU1 <-> AUXN, SBU2 <-> AUXP. */
                    sbu1_conf = SBU_CONNECT_AUX2;
                    sbu2_conf = SBU_CONNECT_AUX1;
                }
                else
                {
                    /* Straight connection: SBU1 <-> AUXP, SBU2 <-> AUXN */
                    sbu1_conf = SBU_CONNECT_AUX1;
                    sbu2_conf = SBU_CONNECT_AUX2;
                }
                break;

            case MUX_CONFIG_DP_2_LANE:
                /* Set mux to to 2-lane DP plus USB-only mode state. */
                tmp_buf[MUX_CONFIG_IDX] = MUX_DP_2_LANE;
                if (polarity)
                {
                    /* Flipped connection: SBU1 <-> AUXN, SBU2 <-> AUXP. */
                    sbu1_conf = SBU_CONNECT_AUX2;
                    sbu2_conf = SBU_CONNECT_AUX1;
                }
                else
                {
                    /* Straight connection: SBU1 <-> AUXP, SBU2 <-> AUXN */
                    sbu1_conf = SBU_CONNECT_AUX1;
                    sbu2_conf = SBU_CONNECT_AUX2;
                }
                break;

            case MUX_CONFIG_RIDGE_CUSTOM:
                /* Set mux to powerdown state. */
                tmp_buf[MUX_CONFIG_IDX] = MUX_SAFE_STATE;
                if (polarity)
                {
                    /* Flipped connection: SBU1 <-> LSRX, SBU2 <-> LSTX. */
                    sbu1_conf = SBU_CONNECT_LSRX;
                    sbu2_conf = SBU_CONNECT_LSTX;
                }
                else
                {
                    /* Straight connection: SBU1 <-> LSTX, SBU2 <-> LSRX */
                    sbu1_conf = SBU_CONNECT_LSTX;
                    sbu2_conf = SBU_CONNECT_LSRX;
                }
                break;

            default:
                /* Set mux to powerdown state. */
                tmp_buf[MUX_CONFIG_IDX] = MUX_SAFE_STATE;

                /* No USB 2.0 connections required. */
                dpdm_conf = DPDM_MUX_CONN_NONE;
                break;
        }

        /* Configure D+/D- MUX as required. */
        ccg_config_dp_dm_mux (port, dpdm_conf);

        /* Enable SBU to LSXX connection for testing purposes. */
        sbu_switch_configure (port, sbu1_conf, sbu2_conf);

        /* Enable WDT */
        MUX_WDT_EN(tmp_buf[MUX_CONFIG_IDX]);
        /* Set polarity */
        MUX_SET_POLARITY(tmp_buf[MUX_CONFIG_IDX], polarity);

        /* Update current mux state if the operation succeeded. */
        status = I2C_Write (mux_addr[port], tmp_buf, MUX_BUFF_SIZE);

        /* Store the current MUX configuration. */
        mux_state[port] = MAKE_MUX_STATE (polarity, cfg);
    }

    return (status);
}

/* Initialize the MUX control SCB block. */
bool mux_ctrl_init(uint8_t port)
{
    bool status = false;

    /* Configure the I2C interface. */
    i2cm_init();
    /* Force the MUX into ISOLATE state. */
    status = mux_ctrl_set_cfg (port, MUX_CONFIG_INIT, 0);

    return status;
}

bool pd_ctrl_init(void)
{
    uint8_t idx1, idx2, count;
    uint8_t wr_buf[2];
    uint8_t status = true;

    count = sizeof (pd_ctrl_config) / (2 * sizeof (uint8_t));
    for (idx1 = TYPEC_PORT_0_IDX; idx1 < NO_OF_TYPEC_PORTS; idx1++)
    {
#if CCG_FRS_RX_ENABLE
        /* Leave the regulator enabled as FRS swap may be triggered at any time. */
        if (idx1 == TYPEC_PORT_0_IDX)
        {
            NCP81239_EN_P1();
        }
#if CCG_PD_DUALPORT_ENABLE
        else
        {
            NCP81239_EN_P2();
        }
#endif /* CCG_PD_DUALPORT_ENABLE */
#endif /* CCG_FRS_RX_ENABLE */

        for (idx2 = 0; idx2 < count; idx2++)
        {
            /* Write data in index 1 to address in index 0. */
            wr_buf[0] = pd_ctrl_config[idx2][0];
            wr_buf[1] = pd_ctrl_config[idx2][1];

            status = I2C_Write(pd_ctrl_addr[idx1], wr_buf, 2);
            if (!status)
                return status;
        }
    }
    return status;
}

/* Setting power voltage on NCP81239 controller */
bool set_pd_ctrl_voltage(uint8_t port, uint16_t volt)
{
    uint8_t wr_buf[2];
    uint8_t status;

#if CCG_FRS_RX_ENABLE
    /* If fast role swap is enabled, we need to have a 5V supply read at all times. */
    if (volt < VSAFE_5V)
    {
        volt = VSAFE_5V;
    }
#endif /* CCG_FRS_RX_ENABLE */

    /* Configure the regulator output voltage. */
    wr_buf[0] = PD_CTRL_VPS_REG_ADDR;
    wr_buf[1] = (volt / NCP_REG_VOLT_RESOLUTION) + NCP_REG_EXCESS_VOLTAGE;

    status = I2C_Write(pd_ctrl_addr[port], wr_buf, 2);
    return (status);
}

void sln_psrc_enable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
    /* Enable NCP81239 controller before enabling the FET. */
    if (port == TYPEC_PORT_0_IDX)
    {
        NCP81239_EN_P1();
    }
#if CCG_PD_DUALPORT_ENABLE		
    else
    {
        NCP81239_EN_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Invoke default psrc enable function used by PD stack. */
    psrc_enable (port, pwr_ready_handler);
}

void sln_psrc_disable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
    /* Invoke default psrc enable function used by PD stack. */
    psrc_disable (port, pwr_ready_handler);
}

/* End of file */

