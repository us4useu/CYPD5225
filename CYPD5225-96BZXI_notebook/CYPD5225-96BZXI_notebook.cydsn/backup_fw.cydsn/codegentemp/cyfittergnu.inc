/*******************************************************************************
* File Name: cyfittergnu.inc
* 
* PSoC Creator  4.2
*
* Description:
* 
* This file is automatically generated by PSoC Creator.
*
********************************************************************************
* Copyright (c) 2007-2018 Cypress Semiconductor.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

.ifndef INCLUDED_CYFITTERGNU_INC
.set INCLUDED_CYFITTERGNU_INC, 1
.include "cydevicegnu_trm.inc"

/* EC_INT */
.set EC_INT__0__DR, CYREG_GPIO_PRT2_DR
.set EC_INT__0__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set EC_INT__0__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set EC_INT__0__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set EC_INT__0__HSIOM, CYREG_HSIOM_PORT_SEL2
.set EC_INT__0__HSIOM_MASK, 0x00F00000
.set EC_INT__0__HSIOM_SHIFT, 20
.set EC_INT__0__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set EC_INT__0__INTR, CYREG_GPIO_PRT2_INTR
.set EC_INT__0__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set EC_INT__0__INTSTAT, CYREG_GPIO_PRT2_INTR
.set EC_INT__0__MASK, 0x20
.set EC_INT__0__PC, CYREG_GPIO_PRT2_PC
.set EC_INT__0__PC2, CYREG_GPIO_PRT2_PC2
.set EC_INT__0__PORT, 2
.set EC_INT__0__PS, CYREG_GPIO_PRT2_PS
.set EC_INT__0__SHIFT, 5
.set EC_INT__DR, CYREG_GPIO_PRT2_DR
.set EC_INT__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set EC_INT__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set EC_INT__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set EC_INT__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set EC_INT__INTR, CYREG_GPIO_PRT2_INTR
.set EC_INT__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set EC_INT__INTSTAT, CYREG_GPIO_PRT2_INTR
.set EC_INT__MASK, 0x20
.set EC_INT__PC, CYREG_GPIO_PRT2_PC
.set EC_INT__PC2, CYREG_GPIO_PRT2_PC2
.set EC_INT__PORT, 2
.set EC_INT__PS, CYREG_GPIO_PRT2_PS
.set EC_INT__SHIFT, 5

/* FW_LED */
.set FW_LED__0__DR, CYREG_GPIO_PRT1_DR
.set FW_LED__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set FW_LED__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set FW_LED__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set FW_LED__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set FW_LED__0__HSIOM_MASK, 0x0F000000
.set FW_LED__0__HSIOM_SHIFT, 24
.set FW_LED__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set FW_LED__0__INTR, CYREG_GPIO_PRT1_INTR
.set FW_LED__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set FW_LED__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set FW_LED__0__MASK, 0x40
.set FW_LED__0__PC, CYREG_GPIO_PRT1_PC
.set FW_LED__0__PC2, CYREG_GPIO_PRT1_PC2
.set FW_LED__0__PORT, 1
.set FW_LED__0__PS, CYREG_GPIO_PRT1_PS
.set FW_LED__0__SHIFT, 6
.set FW_LED__DR, CYREG_GPIO_PRT1_DR
.set FW_LED__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set FW_LED__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set FW_LED__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set FW_LED__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set FW_LED__INTR, CYREG_GPIO_PRT1_INTR
.set FW_LED__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set FW_LED__INTSTAT, CYREG_GPIO_PRT1_INTR
.set FW_LED__MASK, 0x40
.set FW_LED__PC, CYREG_GPIO_PRT1_PC
.set FW_LED__PC2, CYREG_GPIO_PRT1_PC2
.set FW_LED__PORT, 1
.set FW_LED__PS, CYREG_GPIO_PRT1_PS
.set FW_LED__SHIFT, 6

/* HPI_IF */
.set HPI_IF_SCB__CMD_RESP_CTRL, CYREG_SCB0_CMD_RESP_CTRL
.set HPI_IF_SCB__CMD_RESP_STATUS, CYREG_SCB0_CMD_RESP_STATUS
.set HPI_IF_SCB__CTRL, CYREG_SCB0_CTRL
.set HPI_IF_SCB__EZ_DATA0, CYREG_SCB0_EZ_DATA0
.set HPI_IF_SCB__EZ_DATA1, CYREG_SCB0_EZ_DATA1
.set HPI_IF_SCB__EZ_DATA10, CYREG_SCB0_EZ_DATA10
.set HPI_IF_SCB__EZ_DATA11, CYREG_SCB0_EZ_DATA11
.set HPI_IF_SCB__EZ_DATA12, CYREG_SCB0_EZ_DATA12
.set HPI_IF_SCB__EZ_DATA13, CYREG_SCB0_EZ_DATA13
.set HPI_IF_SCB__EZ_DATA14, CYREG_SCB0_EZ_DATA14
.set HPI_IF_SCB__EZ_DATA15, CYREG_SCB0_EZ_DATA15
.set HPI_IF_SCB__EZ_DATA16, CYREG_SCB0_EZ_DATA16
.set HPI_IF_SCB__EZ_DATA17, CYREG_SCB0_EZ_DATA17
.set HPI_IF_SCB__EZ_DATA18, CYREG_SCB0_EZ_DATA18
.set HPI_IF_SCB__EZ_DATA19, CYREG_SCB0_EZ_DATA19
.set HPI_IF_SCB__EZ_DATA2, CYREG_SCB0_EZ_DATA2
.set HPI_IF_SCB__EZ_DATA20, CYREG_SCB0_EZ_DATA20
.set HPI_IF_SCB__EZ_DATA21, CYREG_SCB0_EZ_DATA21
.set HPI_IF_SCB__EZ_DATA22, CYREG_SCB0_EZ_DATA22
.set HPI_IF_SCB__EZ_DATA23, CYREG_SCB0_EZ_DATA23
.set HPI_IF_SCB__EZ_DATA24, CYREG_SCB0_EZ_DATA24
.set HPI_IF_SCB__EZ_DATA25, CYREG_SCB0_EZ_DATA25
.set HPI_IF_SCB__EZ_DATA26, CYREG_SCB0_EZ_DATA26
.set HPI_IF_SCB__EZ_DATA27, CYREG_SCB0_EZ_DATA27
.set HPI_IF_SCB__EZ_DATA28, CYREG_SCB0_EZ_DATA28
.set HPI_IF_SCB__EZ_DATA29, CYREG_SCB0_EZ_DATA29
.set HPI_IF_SCB__EZ_DATA3, CYREG_SCB0_EZ_DATA3
.set HPI_IF_SCB__EZ_DATA30, CYREG_SCB0_EZ_DATA30
.set HPI_IF_SCB__EZ_DATA31, CYREG_SCB0_EZ_DATA31
.set HPI_IF_SCB__EZ_DATA4, CYREG_SCB0_EZ_DATA4
.set HPI_IF_SCB__EZ_DATA5, CYREG_SCB0_EZ_DATA5
.set HPI_IF_SCB__EZ_DATA6, CYREG_SCB0_EZ_DATA6
.set HPI_IF_SCB__EZ_DATA7, CYREG_SCB0_EZ_DATA7
.set HPI_IF_SCB__EZ_DATA8, CYREG_SCB0_EZ_DATA8
.set HPI_IF_SCB__EZ_DATA9, CYREG_SCB0_EZ_DATA9
.set HPI_IF_SCB__I2C_CFG, CYREG_SCB0_I2C_CFG
.set HPI_IF_SCB__I2C_CTRL, CYREG_SCB0_I2C_CTRL
.set HPI_IF_SCB__I2C_M_CMD, CYREG_SCB0_I2C_M_CMD
.set HPI_IF_SCB__I2C_S_CMD, CYREG_SCB0_I2C_S_CMD
.set HPI_IF_SCB__I2C_STATUS, CYREG_SCB0_I2C_STATUS
.set HPI_IF_SCB__INTR_CAUSE, CYREG_SCB0_INTR_CAUSE
.set HPI_IF_SCB__INTR_I2C_EC, CYREG_SCB0_INTR_I2C_EC
.set HPI_IF_SCB__INTR_I2C_EC_MASK, CYREG_SCB0_INTR_I2C_EC_MASK
.set HPI_IF_SCB__INTR_I2C_EC_MASKED, CYREG_SCB0_INTR_I2C_EC_MASKED
.set HPI_IF_SCB__INTR_M, CYREG_SCB0_INTR_M
.set HPI_IF_SCB__INTR_M_MASK, CYREG_SCB0_INTR_M_MASK
.set HPI_IF_SCB__INTR_M_MASKED, CYREG_SCB0_INTR_M_MASKED
.set HPI_IF_SCB__INTR_M_SET, CYREG_SCB0_INTR_M_SET
.set HPI_IF_SCB__INTR_RX, CYREG_SCB0_INTR_RX
.set HPI_IF_SCB__INTR_RX_MASK, CYREG_SCB0_INTR_RX_MASK
.set HPI_IF_SCB__INTR_RX_MASKED, CYREG_SCB0_INTR_RX_MASKED
.set HPI_IF_SCB__INTR_RX_SET, CYREG_SCB0_INTR_RX_SET
.set HPI_IF_SCB__INTR_S, CYREG_SCB0_INTR_S
.set HPI_IF_SCB__INTR_S_MASK, CYREG_SCB0_INTR_S_MASK
.set HPI_IF_SCB__INTR_S_MASKED, CYREG_SCB0_INTR_S_MASKED
.set HPI_IF_SCB__INTR_S_SET, CYREG_SCB0_INTR_S_SET
.set HPI_IF_SCB__INTR_SPI_EC, CYREG_SCB0_INTR_SPI_EC
.set HPI_IF_SCB__INTR_SPI_EC_MASK, CYREG_SCB0_INTR_SPI_EC_MASK
.set HPI_IF_SCB__INTR_SPI_EC_MASKED, CYREG_SCB0_INTR_SPI_EC_MASKED
.set HPI_IF_SCB__INTR_TX, CYREG_SCB0_INTR_TX
.set HPI_IF_SCB__INTR_TX_MASK, CYREG_SCB0_INTR_TX_MASK
.set HPI_IF_SCB__INTR_TX_MASKED, CYREG_SCB0_INTR_TX_MASKED
.set HPI_IF_SCB__INTR_TX_SET, CYREG_SCB0_INTR_TX_SET
.set HPI_IF_SCB__RX_CTRL, CYREG_SCB0_RX_CTRL
.set HPI_IF_SCB__RX_FIFO_CTRL, CYREG_SCB0_RX_FIFO_CTRL
.set HPI_IF_SCB__RX_FIFO_RD, CYREG_SCB0_RX_FIFO_RD
.set HPI_IF_SCB__RX_FIFO_RD_SILENT, CYREG_SCB0_RX_FIFO_RD_SILENT
.set HPI_IF_SCB__RX_FIFO_STATUS, CYREG_SCB0_RX_FIFO_STATUS
.set HPI_IF_SCB__RX_MATCH, CYREG_SCB0_RX_MATCH
.set HPI_IF_SCB__SPI_CTRL, CYREG_SCB0_SPI_CTRL
.set HPI_IF_SCB__SPI_STATUS, CYREG_SCB0_SPI_STATUS
.set HPI_IF_SCB__SS0_POSISTION, 0
.set HPI_IF_SCB__SS1_POSISTION, 1
.set HPI_IF_SCB__SS2_POSISTION, 2
.set HPI_IF_SCB__SS3_POSISTION, 3
.set HPI_IF_SCB__STATUS, CYREG_SCB0_STATUS
.set HPI_IF_SCB__TX_CTRL, CYREG_SCB0_TX_CTRL
.set HPI_IF_SCB__TX_FIFO_CTRL, CYREG_SCB0_TX_FIFO_CTRL
.set HPI_IF_SCB__TX_FIFO_STATUS, CYREG_SCB0_TX_FIFO_STATUS
.set HPI_IF_SCB__TX_FIFO_WR, CYREG_SCB0_TX_FIFO_WR
.set HPI_IF_SCB__UART_CTRL, CYREG_SCB0_UART_CTRL
.set HPI_IF_SCB__UART_FLOW_CTRL, CYREG_SCB0_UART_FLOW_CTRL
.set HPI_IF_SCB__UART_RX_CTRL, CYREG_SCB0_UART_RX_CTRL
.set HPI_IF_SCB__UART_RX_STATUS, CYREG_SCB0_UART_RX_STATUS
.set HPI_IF_SCB__UART_TX_CTRL, CYREG_SCB0_UART_TX_CTRL
.set HPI_IF_SCB_IRQ__INTC_CLR_EN_REG, CYREG_CM0_ICER
.set HPI_IF_SCB_IRQ__INTC_CLR_PD_REG, CYREG_CM0_ICPR
.set HPI_IF_SCB_IRQ__INTC_MASK, 0x400
.set HPI_IF_SCB_IRQ__INTC_NUMBER, 10
.set HPI_IF_SCB_IRQ__INTC_PRIOR_MASK, 0xC00000
.set HPI_IF_SCB_IRQ__INTC_PRIOR_NUM, 3
.set HPI_IF_SCB_IRQ__INTC_PRIOR_REG, CYREG_CM0_IPR2
.set HPI_IF_SCB_IRQ__INTC_SET_EN_REG, CYREG_CM0_ISER
.set HPI_IF_SCB_IRQ__INTC_SET_PD_REG, CYREG_CM0_ISPR
.set HPI_IF_SCBCLK__CTRL_REGISTER, CYREG_PERI_PCLK_CTL0
.set HPI_IF_SCBCLK__DIV_ID, 0x00000000
.set HPI_IF_SCBCLK__DIV_REGISTER, CYREG_PERI_DIV_8_CTL0
.set HPI_IF_SCBCLK__PA_DIV_ID, 0x000000FF
.set HPI_IF_scl__0__DR, CYREG_GPIO_PRT5_DR
.set HPI_IF_scl__0__DR_CLR, CYREG_GPIO_PRT5_DR_CLR
.set HPI_IF_scl__0__DR_INV, CYREG_GPIO_PRT5_DR_INV
.set HPI_IF_scl__0__DR_SET, CYREG_GPIO_PRT5_DR_SET
.set HPI_IF_scl__0__HSIOM, CYREG_HSIOM_PORT_SEL5
.set HPI_IF_scl__0__HSIOM_GPIO, 0
.set HPI_IF_scl__0__HSIOM_I2C, 15
.set HPI_IF_scl__0__HSIOM_I2C_SCL, 15
.set HPI_IF_scl__0__HSIOM_MASK, 0x000000F0
.set HPI_IF_scl__0__HSIOM_SHIFT, 4
.set HPI_IF_scl__0__INTCFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_scl__0__INTR, CYREG_GPIO_PRT5_INTR
.set HPI_IF_scl__0__INTR_CFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_scl__0__INTSTAT, CYREG_GPIO_PRT5_INTR
.set HPI_IF_scl__0__MASK, 0x02
.set HPI_IF_scl__0__PC, CYREG_GPIO_PRT5_PC
.set HPI_IF_scl__0__PC2, CYREG_GPIO_PRT5_PC2
.set HPI_IF_scl__0__PORT, 5
.set HPI_IF_scl__0__PS, CYREG_GPIO_PRT5_PS
.set HPI_IF_scl__0__SHIFT, 1
.set HPI_IF_scl__DR, CYREG_GPIO_PRT5_DR
.set HPI_IF_scl__DR_CLR, CYREG_GPIO_PRT5_DR_CLR
.set HPI_IF_scl__DR_INV, CYREG_GPIO_PRT5_DR_INV
.set HPI_IF_scl__DR_SET, CYREG_GPIO_PRT5_DR_SET
.set HPI_IF_scl__INTCFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_scl__INTR, CYREG_GPIO_PRT5_INTR
.set HPI_IF_scl__INTR_CFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_scl__INTSTAT, CYREG_GPIO_PRT5_INTR
.set HPI_IF_scl__MASK, 0x02
.set HPI_IF_scl__PC, CYREG_GPIO_PRT5_PC
.set HPI_IF_scl__PC2, CYREG_GPIO_PRT5_PC2
.set HPI_IF_scl__PORT, 5
.set HPI_IF_scl__PS, CYREG_GPIO_PRT5_PS
.set HPI_IF_scl__SHIFT, 1
.set HPI_IF_sda__0__DR, CYREG_GPIO_PRT5_DR
.set HPI_IF_sda__0__DR_CLR, CYREG_GPIO_PRT5_DR_CLR
.set HPI_IF_sda__0__DR_INV, CYREG_GPIO_PRT5_DR_INV
.set HPI_IF_sda__0__DR_SET, CYREG_GPIO_PRT5_DR_SET
.set HPI_IF_sda__0__HSIOM, CYREG_HSIOM_PORT_SEL5
.set HPI_IF_sda__0__HSIOM_GPIO, 0
.set HPI_IF_sda__0__HSIOM_I2C, 15
.set HPI_IF_sda__0__HSIOM_I2C_SDA, 15
.set HPI_IF_sda__0__HSIOM_MASK, 0x0000000F
.set HPI_IF_sda__0__HSIOM_SHIFT, 0
.set HPI_IF_sda__0__INTCFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_sda__0__INTR, CYREG_GPIO_PRT5_INTR
.set HPI_IF_sda__0__INTR_CFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_sda__0__INTSTAT, CYREG_GPIO_PRT5_INTR
.set HPI_IF_sda__0__MASK, 0x01
.set HPI_IF_sda__0__PC, CYREG_GPIO_PRT5_PC
.set HPI_IF_sda__0__PC2, CYREG_GPIO_PRT5_PC2
.set HPI_IF_sda__0__PORT, 5
.set HPI_IF_sda__0__PS, CYREG_GPIO_PRT5_PS
.set HPI_IF_sda__0__SHIFT, 0
.set HPI_IF_sda__DR, CYREG_GPIO_PRT5_DR
.set HPI_IF_sda__DR_CLR, CYREG_GPIO_PRT5_DR_CLR
.set HPI_IF_sda__DR_INV, CYREG_GPIO_PRT5_DR_INV
.set HPI_IF_sda__DR_SET, CYREG_GPIO_PRT5_DR_SET
.set HPI_IF_sda__INTCFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_sda__INTR, CYREG_GPIO_PRT5_INTR
.set HPI_IF_sda__INTR_CFG, CYREG_GPIO_PRT5_INTR_CFG
.set HPI_IF_sda__INTSTAT, CYREG_GPIO_PRT5_INTR
.set HPI_IF_sda__MASK, 0x01
.set HPI_IF_sda__PC, CYREG_GPIO_PRT5_PC
.set HPI_IF_sda__PC2, CYREG_GPIO_PRT5_PC2
.set HPI_IF_sda__PORT, 5
.set HPI_IF_sda__PS, CYREG_GPIO_PRT5_PS
.set HPI_IF_sda__SHIFT, 0

/* I2C_CFG */
.set I2C_CFG__0__DR, CYREG_GPIO_PRT1_DR
.set I2C_CFG__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_CFG__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_CFG__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_CFG__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set I2C_CFG__0__HSIOM_MASK, 0x0000000F
.set I2C_CFG__0__HSIOM_SHIFT, 0
.set I2C_CFG__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_CFG__0__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_CFG__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_CFG__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_CFG__0__MASK, 0x01
.set I2C_CFG__0__PC, CYREG_GPIO_PRT1_PC
.set I2C_CFG__0__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_CFG__0__PORT, 1
.set I2C_CFG__0__PS, CYREG_GPIO_PRT1_PS
.set I2C_CFG__0__SHIFT, 0
.set I2C_CFG__DR, CYREG_GPIO_PRT1_DR
.set I2C_CFG__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_CFG__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_CFG__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_CFG__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_CFG__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_CFG__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_CFG__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_CFG__MASK, 0x01
.set I2C_CFG__PC, CYREG_GPIO_PRT1_PC
.set I2C_CFG__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_CFG__PORT, 1
.set I2C_CFG__PS, CYREG_GPIO_PRT1_PS
.set I2C_CFG__SHIFT, 0

/* I2C_MSTR */
.set I2C_MSTR_SCB__CMD_RESP_CTRL, CYREG_SCB1_CMD_RESP_CTRL
.set I2C_MSTR_SCB__CMD_RESP_STATUS, CYREG_SCB1_CMD_RESP_STATUS
.set I2C_MSTR_SCB__CTRL, CYREG_SCB1_CTRL
.set I2C_MSTR_SCB__EZ_DATA0, CYREG_SCB1_EZ_DATA0
.set I2C_MSTR_SCB__EZ_DATA1, CYREG_SCB1_EZ_DATA1
.set I2C_MSTR_SCB__EZ_DATA10, CYREG_SCB1_EZ_DATA10
.set I2C_MSTR_SCB__EZ_DATA11, CYREG_SCB1_EZ_DATA11
.set I2C_MSTR_SCB__EZ_DATA12, CYREG_SCB1_EZ_DATA12
.set I2C_MSTR_SCB__EZ_DATA13, CYREG_SCB1_EZ_DATA13
.set I2C_MSTR_SCB__EZ_DATA14, CYREG_SCB1_EZ_DATA14
.set I2C_MSTR_SCB__EZ_DATA15, CYREG_SCB1_EZ_DATA15
.set I2C_MSTR_SCB__EZ_DATA16, CYREG_SCB1_EZ_DATA16
.set I2C_MSTR_SCB__EZ_DATA17, CYREG_SCB1_EZ_DATA17
.set I2C_MSTR_SCB__EZ_DATA18, CYREG_SCB1_EZ_DATA18
.set I2C_MSTR_SCB__EZ_DATA19, CYREG_SCB1_EZ_DATA19
.set I2C_MSTR_SCB__EZ_DATA2, CYREG_SCB1_EZ_DATA2
.set I2C_MSTR_SCB__EZ_DATA20, CYREG_SCB1_EZ_DATA20
.set I2C_MSTR_SCB__EZ_DATA21, CYREG_SCB1_EZ_DATA21
.set I2C_MSTR_SCB__EZ_DATA22, CYREG_SCB1_EZ_DATA22
.set I2C_MSTR_SCB__EZ_DATA23, CYREG_SCB1_EZ_DATA23
.set I2C_MSTR_SCB__EZ_DATA24, CYREG_SCB1_EZ_DATA24
.set I2C_MSTR_SCB__EZ_DATA25, CYREG_SCB1_EZ_DATA25
.set I2C_MSTR_SCB__EZ_DATA26, CYREG_SCB1_EZ_DATA26
.set I2C_MSTR_SCB__EZ_DATA27, CYREG_SCB1_EZ_DATA27
.set I2C_MSTR_SCB__EZ_DATA28, CYREG_SCB1_EZ_DATA28
.set I2C_MSTR_SCB__EZ_DATA29, CYREG_SCB1_EZ_DATA29
.set I2C_MSTR_SCB__EZ_DATA3, CYREG_SCB1_EZ_DATA3
.set I2C_MSTR_SCB__EZ_DATA30, CYREG_SCB1_EZ_DATA30
.set I2C_MSTR_SCB__EZ_DATA31, CYREG_SCB1_EZ_DATA31
.set I2C_MSTR_SCB__EZ_DATA4, CYREG_SCB1_EZ_DATA4
.set I2C_MSTR_SCB__EZ_DATA5, CYREG_SCB1_EZ_DATA5
.set I2C_MSTR_SCB__EZ_DATA6, CYREG_SCB1_EZ_DATA6
.set I2C_MSTR_SCB__EZ_DATA7, CYREG_SCB1_EZ_DATA7
.set I2C_MSTR_SCB__EZ_DATA8, CYREG_SCB1_EZ_DATA8
.set I2C_MSTR_SCB__EZ_DATA9, CYREG_SCB1_EZ_DATA9
.set I2C_MSTR_SCB__I2C_CFG, CYREG_SCB1_I2C_CFG
.set I2C_MSTR_SCB__I2C_CTRL, CYREG_SCB1_I2C_CTRL
.set I2C_MSTR_SCB__I2C_M_CMD, CYREG_SCB1_I2C_M_CMD
.set I2C_MSTR_SCB__I2C_S_CMD, CYREG_SCB1_I2C_S_CMD
.set I2C_MSTR_SCB__I2C_STATUS, CYREG_SCB1_I2C_STATUS
.set I2C_MSTR_SCB__INTR_CAUSE, CYREG_SCB1_INTR_CAUSE
.set I2C_MSTR_SCB__INTR_I2C_EC, CYREG_SCB1_INTR_I2C_EC
.set I2C_MSTR_SCB__INTR_I2C_EC_MASK, CYREG_SCB1_INTR_I2C_EC_MASK
.set I2C_MSTR_SCB__INTR_I2C_EC_MASKED, CYREG_SCB1_INTR_I2C_EC_MASKED
.set I2C_MSTR_SCB__INTR_M, CYREG_SCB1_INTR_M
.set I2C_MSTR_SCB__INTR_M_MASK, CYREG_SCB1_INTR_M_MASK
.set I2C_MSTR_SCB__INTR_M_MASKED, CYREG_SCB1_INTR_M_MASKED
.set I2C_MSTR_SCB__INTR_M_SET, CYREG_SCB1_INTR_M_SET
.set I2C_MSTR_SCB__INTR_RX, CYREG_SCB1_INTR_RX
.set I2C_MSTR_SCB__INTR_RX_MASK, CYREG_SCB1_INTR_RX_MASK
.set I2C_MSTR_SCB__INTR_RX_MASKED, CYREG_SCB1_INTR_RX_MASKED
.set I2C_MSTR_SCB__INTR_RX_SET, CYREG_SCB1_INTR_RX_SET
.set I2C_MSTR_SCB__INTR_S, CYREG_SCB1_INTR_S
.set I2C_MSTR_SCB__INTR_S_MASK, CYREG_SCB1_INTR_S_MASK
.set I2C_MSTR_SCB__INTR_S_MASKED, CYREG_SCB1_INTR_S_MASKED
.set I2C_MSTR_SCB__INTR_S_SET, CYREG_SCB1_INTR_S_SET
.set I2C_MSTR_SCB__INTR_SPI_EC, CYREG_SCB1_INTR_SPI_EC
.set I2C_MSTR_SCB__INTR_SPI_EC_MASK, CYREG_SCB1_INTR_SPI_EC_MASK
.set I2C_MSTR_SCB__INTR_SPI_EC_MASKED, CYREG_SCB1_INTR_SPI_EC_MASKED
.set I2C_MSTR_SCB__INTR_TX, CYREG_SCB1_INTR_TX
.set I2C_MSTR_SCB__INTR_TX_MASK, CYREG_SCB1_INTR_TX_MASK
.set I2C_MSTR_SCB__INTR_TX_MASKED, CYREG_SCB1_INTR_TX_MASKED
.set I2C_MSTR_SCB__INTR_TX_SET, CYREG_SCB1_INTR_TX_SET
.set I2C_MSTR_SCB__RX_CTRL, CYREG_SCB1_RX_CTRL
.set I2C_MSTR_SCB__RX_FIFO_CTRL, CYREG_SCB1_RX_FIFO_CTRL
.set I2C_MSTR_SCB__RX_FIFO_RD, CYREG_SCB1_RX_FIFO_RD
.set I2C_MSTR_SCB__RX_FIFO_RD_SILENT, CYREG_SCB1_RX_FIFO_RD_SILENT
.set I2C_MSTR_SCB__RX_FIFO_STATUS, CYREG_SCB1_RX_FIFO_STATUS
.set I2C_MSTR_SCB__RX_MATCH, CYREG_SCB1_RX_MATCH
.set I2C_MSTR_SCB__SPI_CTRL, CYREG_SCB1_SPI_CTRL
.set I2C_MSTR_SCB__SPI_STATUS, CYREG_SCB1_SPI_STATUS
.set I2C_MSTR_SCB__SS0_POSISTION, 0
.set I2C_MSTR_SCB__SS1_POSISTION, 1
.set I2C_MSTR_SCB__SS2_POSISTION, 2
.set I2C_MSTR_SCB__SS3_POSISTION, 3
.set I2C_MSTR_SCB__STATUS, CYREG_SCB1_STATUS
.set I2C_MSTR_SCB__TX_CTRL, CYREG_SCB1_TX_CTRL
.set I2C_MSTR_SCB__TX_FIFO_CTRL, CYREG_SCB1_TX_FIFO_CTRL
.set I2C_MSTR_SCB__TX_FIFO_STATUS, CYREG_SCB1_TX_FIFO_STATUS
.set I2C_MSTR_SCB__TX_FIFO_WR, CYREG_SCB1_TX_FIFO_WR
.set I2C_MSTR_SCB__UART_CTRL, CYREG_SCB1_UART_CTRL
.set I2C_MSTR_SCB__UART_FLOW_CTRL, CYREG_SCB1_UART_FLOW_CTRL
.set I2C_MSTR_SCB__UART_RX_CTRL, CYREG_SCB1_UART_RX_CTRL
.set I2C_MSTR_SCB__UART_RX_STATUS, CYREG_SCB1_UART_RX_STATUS
.set I2C_MSTR_SCB__UART_TX_CTRL, CYREG_SCB1_UART_TX_CTRL
.set I2C_MSTR_SCB_IRQ__INTC_CLR_EN_REG, CYREG_CM0_ICER
.set I2C_MSTR_SCB_IRQ__INTC_CLR_PD_REG, CYREG_CM0_ICPR
.set I2C_MSTR_SCB_IRQ__INTC_MASK, 0x800
.set I2C_MSTR_SCB_IRQ__INTC_NUMBER, 11
.set I2C_MSTR_SCB_IRQ__INTC_PRIOR_MASK, 0xC0000000
.set I2C_MSTR_SCB_IRQ__INTC_PRIOR_NUM, 3
.set I2C_MSTR_SCB_IRQ__INTC_PRIOR_REG, CYREG_CM0_IPR2
.set I2C_MSTR_SCB_IRQ__INTC_SET_EN_REG, CYREG_CM0_ISER
.set I2C_MSTR_SCB_IRQ__INTC_SET_PD_REG, CYREG_CM0_ISPR
.set I2C_MSTR_SCBCLK__CTRL_REGISTER, CYREG_PERI_PCLK_CTL1
.set I2C_MSTR_SCBCLK__DIV_ID, 0x00000001
.set I2C_MSTR_SCBCLK__DIV_REGISTER, CYREG_PERI_DIV_8_CTL1
.set I2C_MSTR_SCBCLK__PA_DIV_ID, 0x000000FF
.set I2C_MSTR_scl__0__DR, CYREG_GPIO_PRT1_DR
.set I2C_MSTR_scl__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_MSTR_scl__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_MSTR_scl__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_MSTR_scl__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set I2C_MSTR_scl__0__HSIOM_GPIO, 0
.set I2C_MSTR_scl__0__HSIOM_I2C, 15
.set I2C_MSTR_scl__0__HSIOM_I2C_SCL, 15
.set I2C_MSTR_scl__0__HSIOM_MASK, 0x00000F00
.set I2C_MSTR_scl__0__HSIOM_SHIFT, 8
.set I2C_MSTR_scl__0__HSIOM_SPI, 14
.set I2C_MSTR_scl__0__HSIOM_SPI_MISO, 14
.set I2C_MSTR_scl__0__HSIOM_UART, 10
.set I2C_MSTR_scl__0__HSIOM_UART_CTS, 10
.set I2C_MSTR_scl__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_scl__0__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_scl__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_scl__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_scl__0__MASK, 0x04
.set I2C_MSTR_scl__0__PC, CYREG_GPIO_PRT1_PC
.set I2C_MSTR_scl__0__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_MSTR_scl__0__PORT, 1
.set I2C_MSTR_scl__0__PS, CYREG_GPIO_PRT1_PS
.set I2C_MSTR_scl__0__SHIFT, 2
.set I2C_MSTR_scl__DR, CYREG_GPIO_PRT1_DR
.set I2C_MSTR_scl__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_MSTR_scl__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_MSTR_scl__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_MSTR_scl__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_scl__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_scl__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_scl__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_scl__MASK, 0x04
.set I2C_MSTR_scl__PC, CYREG_GPIO_PRT1_PC
.set I2C_MSTR_scl__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_MSTR_scl__PORT, 1
.set I2C_MSTR_scl__PS, CYREG_GPIO_PRT1_PS
.set I2C_MSTR_scl__SHIFT, 2
.set I2C_MSTR_sda__0__DR, CYREG_GPIO_PRT1_DR
.set I2C_MSTR_sda__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_MSTR_sda__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_MSTR_sda__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_MSTR_sda__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set I2C_MSTR_sda__0__HSIOM_GPIO, 0
.set I2C_MSTR_sda__0__HSIOM_I2C, 15
.set I2C_MSTR_sda__0__HSIOM_I2C_SDA, 15
.set I2C_MSTR_sda__0__HSIOM_MASK, 0x000000F0
.set I2C_MSTR_sda__0__HSIOM_SHIFT, 4
.set I2C_MSTR_sda__0__HSIOM_SPI, 14
.set I2C_MSTR_sda__0__HSIOM_SPI_MOSI, 14
.set I2C_MSTR_sda__0__HSIOM_UART, 10
.set I2C_MSTR_sda__0__HSIOM_UART_TX, 10
.set I2C_MSTR_sda__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_sda__0__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_sda__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_sda__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_sda__0__MASK, 0x02
.set I2C_MSTR_sda__0__PC, CYREG_GPIO_PRT1_PC
.set I2C_MSTR_sda__0__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_MSTR_sda__0__PORT, 1
.set I2C_MSTR_sda__0__PS, CYREG_GPIO_PRT1_PS
.set I2C_MSTR_sda__0__SHIFT, 1
.set I2C_MSTR_sda__DR, CYREG_GPIO_PRT1_DR
.set I2C_MSTR_sda__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set I2C_MSTR_sda__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set I2C_MSTR_sda__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set I2C_MSTR_sda__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_sda__INTR, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_sda__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set I2C_MSTR_sda__INTSTAT, CYREG_GPIO_PRT1_INTR
.set I2C_MSTR_sda__MASK, 0x02
.set I2C_MSTR_sda__PC, CYREG_GPIO_PRT1_PC
.set I2C_MSTR_sda__PC2, CYREG_GPIO_PRT1_PC2
.set I2C_MSTR_sda__PORT, 1
.set I2C_MSTR_sda__PS, CYREG_GPIO_PRT1_PS
.set I2C_MSTR_sda__SHIFT, 1

/* DP_HPD_P1 */
.set DP_HPD_P1__0__DR, CYREG_GPIO_PRT3_DR
.set DP_HPD_P1__0__DR_CLR, CYREG_GPIO_PRT3_DR_CLR
.set DP_HPD_P1__0__DR_INV, CYREG_GPIO_PRT3_DR_INV
.set DP_HPD_P1__0__DR_SET, CYREG_GPIO_PRT3_DR_SET
.set DP_HPD_P1__0__HSIOM, CYREG_HSIOM_PORT_SEL3
.set DP_HPD_P1__0__HSIOM_MASK, 0x0000000F
.set DP_HPD_P1__0__HSIOM_SHIFT, 0
.set DP_HPD_P1__0__INTCFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P1__0__INTR, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P1__0__INTR_CFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P1__0__INTSTAT, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P1__0__MASK, 0x01
.set DP_HPD_P1__0__PC, CYREG_GPIO_PRT3_PC
.set DP_HPD_P1__0__PC2, CYREG_GPIO_PRT3_PC2
.set DP_HPD_P1__0__PORT, 3
.set DP_HPD_P1__0__PS, CYREG_GPIO_PRT3_PS
.set DP_HPD_P1__0__SHIFT, 0
.set DP_HPD_P1__DR, CYREG_GPIO_PRT3_DR
.set DP_HPD_P1__DR_CLR, CYREG_GPIO_PRT3_DR_CLR
.set DP_HPD_P1__DR_INV, CYREG_GPIO_PRT3_DR_INV
.set DP_HPD_P1__DR_SET, CYREG_GPIO_PRT3_DR_SET
.set DP_HPD_P1__INTCFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P1__INTR, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P1__INTR_CFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P1__INTSTAT, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P1__MASK, 0x01
.set DP_HPD_P1__PC, CYREG_GPIO_PRT3_PC
.set DP_HPD_P1__PC2, CYREG_GPIO_PRT3_PC2
.set DP_HPD_P1__PORT, 3
.set DP_HPD_P1__PS, CYREG_GPIO_PRT3_PS
.set DP_HPD_P1__SHIFT, 0

/* DP_HPD_P2 */
.set DP_HPD_P2__0__DR, CYREG_GPIO_PRT3_DR
.set DP_HPD_P2__0__DR_CLR, CYREG_GPIO_PRT3_DR_CLR
.set DP_HPD_P2__0__DR_INV, CYREG_GPIO_PRT3_DR_INV
.set DP_HPD_P2__0__DR_SET, CYREG_GPIO_PRT3_DR_SET
.set DP_HPD_P2__0__HSIOM, CYREG_HSIOM_PORT_SEL3
.set DP_HPD_P2__0__HSIOM_MASK, 0x000F0000
.set DP_HPD_P2__0__HSIOM_SHIFT, 16
.set DP_HPD_P2__0__INTCFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P2__0__INTR, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P2__0__INTR_CFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P2__0__INTSTAT, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P2__0__MASK, 0x10
.set DP_HPD_P2__0__PC, CYREG_GPIO_PRT3_PC
.set DP_HPD_P2__0__PC2, CYREG_GPIO_PRT3_PC2
.set DP_HPD_P2__0__PORT, 3
.set DP_HPD_P2__0__PS, CYREG_GPIO_PRT3_PS
.set DP_HPD_P2__0__SHIFT, 4
.set DP_HPD_P2__DR, CYREG_GPIO_PRT3_DR
.set DP_HPD_P2__DR_CLR, CYREG_GPIO_PRT3_DR_CLR
.set DP_HPD_P2__DR_INV, CYREG_GPIO_PRT3_DR_INV
.set DP_HPD_P2__DR_SET, CYREG_GPIO_PRT3_DR_SET
.set DP_HPD_P2__INTCFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P2__INTR, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P2__INTR_CFG, CYREG_GPIO_PRT3_INTR_CFG
.set DP_HPD_P2__INTSTAT, CYREG_GPIO_PRT3_INTR
.set DP_HPD_P2__MASK, 0x10
.set DP_HPD_P2__PC, CYREG_GPIO_PRT3_PC
.set DP_HPD_P2__PC2, CYREG_GPIO_PRT3_PC2
.set DP_HPD_P2__PORT, 3
.set DP_HPD_P2__PS, CYREG_GPIO_PRT3_PS
.set DP_HPD_P2__SHIFT, 4

/* NCP81239_EN_P1 */
.set NCP81239_EN_P1__0__DR, CYREG_GPIO_PRT1_DR
.set NCP81239_EN_P1__0__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set NCP81239_EN_P1__0__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set NCP81239_EN_P1__0__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set NCP81239_EN_P1__0__HSIOM, CYREG_HSIOM_PORT_SEL1
.set NCP81239_EN_P1__0__HSIOM_MASK, 0x0000F000
.set NCP81239_EN_P1__0__HSIOM_SHIFT, 12
.set NCP81239_EN_P1__0__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set NCP81239_EN_P1__0__INTR, CYREG_GPIO_PRT1_INTR
.set NCP81239_EN_P1__0__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set NCP81239_EN_P1__0__INTSTAT, CYREG_GPIO_PRT1_INTR
.set NCP81239_EN_P1__0__MASK, 0x08
.set NCP81239_EN_P1__0__PC, CYREG_GPIO_PRT1_PC
.set NCP81239_EN_P1__0__PC2, CYREG_GPIO_PRT1_PC2
.set NCP81239_EN_P1__0__PORT, 1
.set NCP81239_EN_P1__0__PS, CYREG_GPIO_PRT1_PS
.set NCP81239_EN_P1__0__SHIFT, 3
.set NCP81239_EN_P1__DR, CYREG_GPIO_PRT1_DR
.set NCP81239_EN_P1__DR_CLR, CYREG_GPIO_PRT1_DR_CLR
.set NCP81239_EN_P1__DR_INV, CYREG_GPIO_PRT1_DR_INV
.set NCP81239_EN_P1__DR_SET, CYREG_GPIO_PRT1_DR_SET
.set NCP81239_EN_P1__INTCFG, CYREG_GPIO_PRT1_INTR_CFG
.set NCP81239_EN_P1__INTR, CYREG_GPIO_PRT1_INTR
.set NCP81239_EN_P1__INTR_CFG, CYREG_GPIO_PRT1_INTR_CFG
.set NCP81239_EN_P1__INTSTAT, CYREG_GPIO_PRT1_INTR
.set NCP81239_EN_P1__MASK, 0x08
.set NCP81239_EN_P1__PC, CYREG_GPIO_PRT1_PC
.set NCP81239_EN_P1__PC2, CYREG_GPIO_PRT1_PC2
.set NCP81239_EN_P1__PORT, 1
.set NCP81239_EN_P1__PS, CYREG_GPIO_PRT1_PS
.set NCP81239_EN_P1__SHIFT, 3

/* NCP81239_EN_P2 */
.set NCP81239_EN_P2__0__DR, CYREG_GPIO_PRT2_DR
.set NCP81239_EN_P2__0__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set NCP81239_EN_P2__0__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set NCP81239_EN_P2__0__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set NCP81239_EN_P2__0__HSIOM, CYREG_HSIOM_PORT_SEL2
.set NCP81239_EN_P2__0__HSIOM_MASK, 0x000000F0
.set NCP81239_EN_P2__0__HSIOM_SHIFT, 4
.set NCP81239_EN_P2__0__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set NCP81239_EN_P2__0__INTR, CYREG_GPIO_PRT2_INTR
.set NCP81239_EN_P2__0__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set NCP81239_EN_P2__0__INTSTAT, CYREG_GPIO_PRT2_INTR
.set NCP81239_EN_P2__0__MASK, 0x02
.set NCP81239_EN_P2__0__PC, CYREG_GPIO_PRT2_PC
.set NCP81239_EN_P2__0__PC2, CYREG_GPIO_PRT2_PC2
.set NCP81239_EN_P2__0__PORT, 2
.set NCP81239_EN_P2__0__PS, CYREG_GPIO_PRT2_PS
.set NCP81239_EN_P2__0__SHIFT, 1
.set NCP81239_EN_P2__DR, CYREG_GPIO_PRT2_DR
.set NCP81239_EN_P2__DR_CLR, CYREG_GPIO_PRT2_DR_CLR
.set NCP81239_EN_P2__DR_INV, CYREG_GPIO_PRT2_DR_INV
.set NCP81239_EN_P2__DR_SET, CYREG_GPIO_PRT2_DR_SET
.set NCP81239_EN_P2__INTCFG, CYREG_GPIO_PRT2_INTR_CFG
.set NCP81239_EN_P2__INTR, CYREG_GPIO_PRT2_INTR
.set NCP81239_EN_P2__INTR_CFG, CYREG_GPIO_PRT2_INTR_CFG
.set NCP81239_EN_P2__INTSTAT, CYREG_GPIO_PRT2_INTR
.set NCP81239_EN_P2__MASK, 0x02
.set NCP81239_EN_P2__PC, CYREG_GPIO_PRT2_PC
.set NCP81239_EN_P2__PC2, CYREG_GPIO_PRT2_PC2
.set NCP81239_EN_P2__PORT, 2
.set NCP81239_EN_P2__PS, CYREG_GPIO_PRT2_PS
.set NCP81239_EN_P2__SHIFT, 1

/* PDSS_PORT0_RX_CLK */
.set PDSS_PORT0_RX_CLK__DIV_ID, 0x00000002
.set PDSS_PORT0_RX_CLK__DIV_REGISTER, CYREG_PERI_DIV_8_CTL2
.set PDSS_PORT0_RX_CLK__PA_DIV_ID, 0x000000FF

/* PDSS_PORT0_TX_CLK */
.set PDSS_PORT0_TX_CLK__DIV_ID, 0x00000005
.set PDSS_PORT0_TX_CLK__DIV_REGISTER, CYREG_PERI_DIV_8_CTL5
.set PDSS_PORT0_TX_CLK__PA_DIV_ID, 0x000000FF

/* PDSS_PORT0_SAR_CLK */
.set PDSS_PORT0_SAR_CLK__DIV_ID, 0x00000004
.set PDSS_PORT0_SAR_CLK__DIV_REGISTER, CYREG_PERI_DIV_8_CTL4
.set PDSS_PORT0_SAR_CLK__PA_DIV_ID, 0x000000FF

/* PDSS_PORT0_FILT1_CLK */
.set PDSS_PORT0_FILT1_CLK__DIV_ID, 0x00000040
.set PDSS_PORT0_FILT1_CLK__DIV_REGISTER, CYREG_PERI_DIV_16_CTL0
.set PDSS_PORT0_FILT1_CLK__PA_DIV_ID, 0x000000FF

/* PDSS_PORTX_REFGEN_CLK */
.set PDSS_PORTX_REFGEN_CLK__DIV_ID, 0x00000003
.set PDSS_PORTX_REFGEN_CLK__DIV_REGISTER, CYREG_PERI_DIV_8_CTL3
.set PDSS_PORTX_REFGEN_CLK__PA_DIV_ID, 0x000000FF

/* Miscellaneous */
.set CYDEV_BCLK__HFCLK__HZ, 48000000
.set CYDEV_BCLK__HFCLK__KHZ, 48000
.set CYDEV_BCLK__HFCLK__MHZ, 48
.set CYDEV_BCLK__SYSCLK__HZ, 24000000
.set CYDEV_BCLK__SYSCLK__KHZ, 24000
.set CYDEV_BCLK__SYSCLK__MHZ, 24
.set CYDEV_CHIP_DIE_LEOPARD, 1
.set CYDEV_CHIP_DIE_PSOC4A, 18
.set CYDEV_CHIP_DIE_PSOC5LP, 2
.set CYDEV_CHIP_DIE_PSOC5TM, 3
.set CYDEV_CHIP_DIE_TMA4, 4
.set CYDEV_CHIP_DIE_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_FM0P, 5
.set CYDEV_CHIP_FAMILY_FM3, 6
.set CYDEV_CHIP_FAMILY_FM4, 7
.set CYDEV_CHIP_FAMILY_PSOC3, 1
.set CYDEV_CHIP_FAMILY_PSOC4, 2
.set CYDEV_CHIP_FAMILY_PSOC5, 3
.set CYDEV_CHIP_FAMILY_PSOC6, 4
.set CYDEV_CHIP_FAMILY_UNKNOWN, 0
.set CYDEV_CHIP_FAMILY_USED, CYDEV_CHIP_FAMILY_PSOC4
.set CYDEV_CHIP_JTAG_ID, 0x210011B1
.set CYDEV_CHIP_MEMBER_3A, 1
.set CYDEV_CHIP_MEMBER_4A, 18
.set CYDEV_CHIP_MEMBER_4D, 13
.set CYDEV_CHIP_MEMBER_4E, 6
.set CYDEV_CHIP_MEMBER_4F, 19
.set CYDEV_CHIP_MEMBER_4G, 4
.set CYDEV_CHIP_MEMBER_4H, 17
.set CYDEV_CHIP_MEMBER_4I, 23
.set CYDEV_CHIP_MEMBER_4J, 14
.set CYDEV_CHIP_MEMBER_4K, 15
.set CYDEV_CHIP_MEMBER_4L, 22
.set CYDEV_CHIP_MEMBER_4M, 21
.set CYDEV_CHIP_MEMBER_4N, 10
.set CYDEV_CHIP_MEMBER_4O, 7
.set CYDEV_CHIP_MEMBER_4P, 20
.set CYDEV_CHIP_MEMBER_4Q, 12
.set CYDEV_CHIP_MEMBER_4R, 8
.set CYDEV_CHIP_MEMBER_4S, 11
.set CYDEV_CHIP_MEMBER_4T, 9
.set CYDEV_CHIP_MEMBER_4U, 5
.set CYDEV_CHIP_MEMBER_4V, 16
.set CYDEV_CHIP_MEMBER_5A, 3
.set CYDEV_CHIP_MEMBER_5B, 2
.set CYDEV_CHIP_MEMBER_6A, 24
.set CYDEV_CHIP_MEMBER_FM3, 28
.set CYDEV_CHIP_MEMBER_FM4, 29
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1, 25
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2, 26
.set CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3, 27
.set CYDEV_CHIP_MEMBER_UNKNOWN, 0
.set CYDEV_CHIP_MEMBER_USED, CYDEV_CHIP_MEMBER_4S
.set CYDEV_CHIP_DIE_EXPECT, CYDEV_CHIP_MEMBER_USED
.set CYDEV_CHIP_DIE_ACTUAL, CYDEV_CHIP_DIE_EXPECT
.set CYDEV_CHIP_REV_LEOPARD_ES1, 0
.set CYDEV_CHIP_REV_LEOPARD_ES2, 1
.set CYDEV_CHIP_REV_LEOPARD_ES3, 3
.set CYDEV_CHIP_REV_LEOPARD_PRODUCTION, 3
.set CYDEV_CHIP_REV_PSOC4A_ES0, 17
.set CYDEV_CHIP_REV_PSOC4A_PRODUCTION, 17
.set CYDEV_CHIP_REV_PSOC5LP_ES0, 0
.set CYDEV_CHIP_REV_PSOC5LP_PRODUCTION, 0
.set CYDEV_CHIP_REV_PSOC5TM_ES0, 0
.set CYDEV_CHIP_REV_PSOC5TM_ES1, 1
.set CYDEV_CHIP_REV_PSOC5TM_PRODUCTION, 1
.set CYDEV_CHIP_REV_TMA4_ES, 17
.set CYDEV_CHIP_REV_TMA4_ES2, 33
.set CYDEV_CHIP_REV_TMA4_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_3A_ES1, 0
.set CYDEV_CHIP_REVISION_3A_ES2, 1
.set CYDEV_CHIP_REVISION_3A_ES3, 3
.set CYDEV_CHIP_REVISION_3A_PRODUCTION, 3
.set CYDEV_CHIP_REVISION_4A_ES0, 17
.set CYDEV_CHIP_REVISION_4A_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_4D_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD, 0
.set CYDEV_CHIP_REVISION_4E_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA, 0
.set CYDEV_CHIP_REVISION_4F_PRODUCTION_256K, 0
.set CYDEV_CHIP_REVISION_4G_ES, 17
.set CYDEV_CHIP_REVISION_4G_ES2, 33
.set CYDEV_CHIP_REVISION_4G_PRODUCTION, 17
.set CYDEV_CHIP_REVISION_4H_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4I_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4J_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4K_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4L_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4M_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4N_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4O_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4P_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4Q_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4R_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4S_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4T_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4U_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_4V_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_5A_ES0, 0
.set CYDEV_CHIP_REVISION_5A_ES1, 1
.set CYDEV_CHIP_REVISION_5A_PRODUCTION, 1
.set CYDEV_CHIP_REVISION_5B_ES0, 0
.set CYDEV_CHIP_REVISION_5B_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_6A_ES, 17
.set CYDEV_CHIP_REVISION_6A_NO_UDB, 33
.set CYDEV_CHIP_REVISION_6A_PRODUCTION, 33
.set CYDEV_CHIP_REVISION_FM3_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_FM4_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION, 0
.set CYDEV_CHIP_REVISION_USED, CYDEV_CHIP_REVISION_4S_PRODUCTION
.set CYDEV_CHIP_REV_EXPECT, CYDEV_CHIP_REVISION_USED
.set CYDEV_CONFIG_READ_ACCELERATOR, 1
.set CYDEV_CONFIG_UNUSED_IO_AllowButWarn, 0
.set CYDEV_CONFIG_UNUSED_IO_AllowWithInfo, 1
.set CYDEV_CONFIG_UNUSED_IO_Disallowed, 2
.set CYDEV_CONFIGURATION_COMPRESSED, 1
.set CYDEV_CONFIGURATION_MODE_COMPRESSED, 0
.set CYDEV_CONFIGURATION_MODE, CYDEV_CONFIGURATION_MODE_COMPRESSED
.set CYDEV_CONFIGURATION_MODE_DMA, 2
.set CYDEV_CONFIGURATION_MODE_UNCOMPRESSED, 1
.set CYDEV_DEBUG_PROTECT_KILL, 4
.set CYDEV_DEBUG_PROTECT_OPEN, 1
.set CYDEV_DEBUG_PROTECT, CYDEV_DEBUG_PROTECT_OPEN
.set CYDEV_DEBUG_PROTECT_PROTECTED, 2
.set CYDEV_DEBUGGING_DPS_Disable, 3
.set CYDEV_DEBUGGING_DPS, CYDEV_DEBUGGING_DPS_Disable
.set CYDEV_DEBUGGING_DPS_SWD, 2
.set CYDEV_DEBUGGING_ENABLE, 0
.set CYDEV_DFT_SELECT_CLK0, 8
.set CYDEV_DFT_SELECT_CLK1, 9
.set CYDEV_HEAP_SIZE, 0
.set CYDEV_IMO_TRIMMED_BY_USB, 0
.set CYDEV_IMO_TRIMMED_BY_WCO, 0
.set CYDEV_IS_EXPORTING_CODE, 0
.set CYDEV_IS_IMPORTING_CODE, 0
.set CYDEV_PROJ_TYPE, 2
.set CYDEV_PROJ_TYPE_BOOTLOADER, 1
.set CYDEV_PROJ_TYPE_LAUNCHER, 5
.set CYDEV_PROJ_TYPE_LOADABLE, 2
.set CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER, 4
.set CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER, 3
.set CYDEV_PROJ_TYPE_STANDARD, 0
.set CYDEV_STACK_SIZE, 0x0800
.set CYDEV_USE_BUNDLED_CMSIS, 1
.set CYDEV_VARIABLE_VDDA, 1
.set CYDEV_VDDD_MV, 3300
.set CYDEV_VDDIO_MV, 3300
.set CYDEV_WDT_GENERATE_ISR, 0
.set CYIPBLOCK_m0s8cpussv2_VERSION, 1
.set CYIPBLOCK_m0s8ioss_VERSION, 1
.set CYIPBLOCK_m0s8peri_VERSION, 1
.set CYIPBLOCK_m0s8scb_VERSION, 2
.set CYIPBLOCK_m0s8tcpwm_VERSION, 2
.set CYIPBLOCK_mxusbpd_VERSION, 1
.set CYIPBLOCK_s8srsslt_VERSION, 1
.set CYDEV_BOOTLOADER_ENABLE, 0
.endif
