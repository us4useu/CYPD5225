/***************************************************************************//**
* \file HPI_IF_PINS.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PINS_HPI_IF_H)
#define CY_SCB_PINS_HPI_IF_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define HPI_IF_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define HPI_IF_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define HPI_IF_REMOVE_TX_SDA_MISO_PIN      (1u)
#define HPI_IF_REMOVE_CTS_SCLK_PIN      (1u)
#define HPI_IF_REMOVE_RTS_SS0_PIN      (1u)
#define HPI_IF_REMOVE_SS1_PIN                 (1u)
#define HPI_IF_REMOVE_SS2_PIN                 (1u)
#define HPI_IF_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define HPI_IF_REMOVE_I2C_PINS                (0u)
#define HPI_IF_REMOVE_SPI_MASTER_PINS         (1u)
#define HPI_IF_REMOVE_SPI_MASTER_SCLK_PIN     (1u)
#define HPI_IF_REMOVE_SPI_MASTER_MOSI_PIN     (1u)
#define HPI_IF_REMOVE_SPI_MASTER_MISO_PIN     (1u)
#define HPI_IF_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define HPI_IF_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define HPI_IF_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define HPI_IF_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define HPI_IF_REMOVE_SPI_SLAVE_PINS          (1u)
#define HPI_IF_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define HPI_IF_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define HPI_IF_REMOVE_UART_TX_PIN             (1u)
#define HPI_IF_REMOVE_UART_RX_TX_PIN          (1u)
#define HPI_IF_REMOVE_UART_RX_PIN             (1u)
#define HPI_IF_REMOVE_UART_RX_WAKE_PIN        (1u)
#define HPI_IF_REMOVE_UART_RTS_PIN            (1u)
#define HPI_IF_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define HPI_IF_RX_WAKE_SCL_MOSI_PIN (0u == HPI_IF_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define HPI_IF_RX_SCL_MOSI_PIN     (0u == HPI_IF_REMOVE_RX_SCL_MOSI_PIN)
#define HPI_IF_TX_SDA_MISO_PIN     (0u == HPI_IF_REMOVE_TX_SDA_MISO_PIN)
#define HPI_IF_CTS_SCLK_PIN     (0u == HPI_IF_REMOVE_CTS_SCLK_PIN)
#define HPI_IF_RTS_SS0_PIN     (0u == HPI_IF_REMOVE_RTS_SS0_PIN)
#define HPI_IF_SS1_PIN                (0u == HPI_IF_REMOVE_SS1_PIN)
#define HPI_IF_SS2_PIN                (0u == HPI_IF_REMOVE_SS2_PIN)
#define HPI_IF_SS3_PIN                (0u == HPI_IF_REMOVE_SS3_PIN)

/* Mode defined pins */
#define HPI_IF_I2C_PINS               (0u == HPI_IF_REMOVE_I2C_PINS)
#define HPI_IF_SPI_MASTER_PINS        (0u == HPI_IF_REMOVE_SPI_MASTER_PINS)
#define HPI_IF_SPI_MASTER_SCLK_PIN    (0u == HPI_IF_REMOVE_SPI_MASTER_SCLK_PIN)
#define HPI_IF_SPI_MASTER_MOSI_PIN    (0u == HPI_IF_REMOVE_SPI_MASTER_MOSI_PIN)
#define HPI_IF_SPI_MASTER_MISO_PIN    (0u == HPI_IF_REMOVE_SPI_MASTER_MISO_PIN)
#define HPI_IF_SPI_MASTER_SS0_PIN     (0u == HPI_IF_REMOVE_SPI_MASTER_SS0_PIN)
#define HPI_IF_SPI_MASTER_SS1_PIN     (0u == HPI_IF_REMOVE_SPI_MASTER_SS1_PIN)
#define HPI_IF_SPI_MASTER_SS2_PIN     (0u == HPI_IF_REMOVE_SPI_MASTER_SS2_PIN)
#define HPI_IF_SPI_MASTER_SS3_PIN     (0u == HPI_IF_REMOVE_SPI_MASTER_SS3_PIN)
#define HPI_IF_SPI_SLAVE_PINS         (0u == HPI_IF_REMOVE_SPI_SLAVE_PINS)
#define HPI_IF_SPI_SLAVE_MOSI_PIN     (0u == HPI_IF_REMOVE_SPI_SLAVE_MOSI_PIN)
#define HPI_IF_SPI_SLAVE_MISO_PIN     (0u == HPI_IF_REMOVE_SPI_SLAVE_MISO_PIN)
#define HPI_IF_UART_TX_PIN            (0u == HPI_IF_REMOVE_UART_TX_PIN)
#define HPI_IF_UART_RX_TX_PIN         (0u == HPI_IF_REMOVE_UART_RX_TX_PIN)
#define HPI_IF_UART_RX_PIN            (0u == HPI_IF_REMOVE_UART_RX_PIN)
#define HPI_IF_UART_RX_WAKE_PIN       (0u == HPI_IF_REMOVE_UART_RX_WAKE_PIN)
#define HPI_IF_UART_RTS_PIN           (0u == HPI_IF_REMOVE_UART_RTS_PIN)
#define HPI_IF_UART_CTS_PIN           (0u == HPI_IF_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (HPI_IF_RX_WAKE_SCL_MOSI_PIN)
    #include "HPI_IF_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (HPI_IF_RX_SCL_MOSI) */

#if (HPI_IF_RX_SCL_MOSI_PIN)
    #include "HPI_IF_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (HPI_IF_RX_SCL_MOSI) */

#if (HPI_IF_TX_SDA_MISO_PIN)
    #include "HPI_IF_uart_tx_i2c_sda_spi_miso.h"
#endif /* (HPI_IF_TX_SDA_MISO) */

#if (HPI_IF_CTS_SCLK_PIN)
    #include "HPI_IF_uart_cts_spi_sclk.h"
#endif /* (HPI_IF_CTS_SCLK) */

#if (HPI_IF_RTS_SS0_PIN)
    #include "HPI_IF_uart_rts_spi_ss0.h"
#endif /* (HPI_IF_RTS_SS0_PIN) */

#if (HPI_IF_SS1_PIN)
    #include "HPI_IF_spi_ss1.h"
#endif /* (HPI_IF_SS1_PIN) */

#if (HPI_IF_SS2_PIN)
    #include "HPI_IF_spi_ss2.h"
#endif /* (HPI_IF_SS2_PIN) */

#if (HPI_IF_SS3_PIN)
    #include "HPI_IF_spi_ss3.h"
#endif /* (HPI_IF_SS3_PIN) */

#if (HPI_IF_I2C_PINS)
    #include "HPI_IF_scl.h"
    #include "HPI_IF_sda.h"
#endif /* (HPI_IF_I2C_PINS) */

#if (HPI_IF_SPI_MASTER_PINS)
#if (HPI_IF_SPI_MASTER_SCLK_PIN)
    #include "HPI_IF_sclk_m.h"
#endif /* (HPI_IF_SPI_MASTER_SCLK_PIN) */

#if (HPI_IF_SPI_MASTER_MOSI_PIN)
    #include "HPI_IF_mosi_m.h"
#endif /* (HPI_IF_SPI_MASTER_MOSI_PIN) */

#if (HPI_IF_SPI_MASTER_MISO_PIN)
    #include "HPI_IF_miso_m.h"
#endif /*(HPI_IF_SPI_MASTER_MISO_PIN) */
#endif /* (HPI_IF_SPI_MASTER_PINS) */

#if (HPI_IF_SPI_SLAVE_PINS)
    #include "HPI_IF_sclk_s.h"
    #include "HPI_IF_ss_s.h"

#if (HPI_IF_SPI_SLAVE_MOSI_PIN)
    #include "HPI_IF_mosi_s.h"
#endif /* (HPI_IF_SPI_SLAVE_MOSI_PIN) */

#if (HPI_IF_SPI_SLAVE_MISO_PIN)
    #include "HPI_IF_miso_s.h"
#endif /*(HPI_IF_SPI_SLAVE_MISO_PIN) */
#endif /* (HPI_IF_SPI_SLAVE_PINS) */

#if (HPI_IF_SPI_MASTER_SS0_PIN)
    #include "HPI_IF_ss0_m.h"
#endif /* (HPI_IF_SPI_MASTER_SS0_PIN) */

#if (HPI_IF_SPI_MASTER_SS1_PIN)
    #include "HPI_IF_ss1_m.h"
#endif /* (HPI_IF_SPI_MASTER_SS1_PIN) */

#if (HPI_IF_SPI_MASTER_SS2_PIN)
    #include "HPI_IF_ss2_m.h"
#endif /* (HPI_IF_SPI_MASTER_SS2_PIN) */

#if (HPI_IF_SPI_MASTER_SS3_PIN)
    #include "HPI_IF_ss3_m.h"
#endif /* (HPI_IF_SPI_MASTER_SS3_PIN) */

#if (HPI_IF_UART_TX_PIN)
    #include "HPI_IF_tx.h"
#endif /* (HPI_IF_UART_TX_PIN) */

#if (HPI_IF_UART_RX_TX_PIN)
    #include "HPI_IF_rx_tx.h"
#endif /* (HPI_IF_UART_RX_TX_PIN) */

#if (HPI_IF_UART_RX_PIN)
    #include "HPI_IF_rx.h"
#endif /* (HPI_IF_UART_RX_PIN) */

#if (HPI_IF_UART_RX_WAKE_PIN)
    #include "HPI_IF_rx_wake.h"
#endif /* (HPI_IF_UART_RX_WAKE_PIN) */

#if (HPI_IF_UART_RTS_PIN)
    #include "HPI_IF_rts.h"
#endif /* (HPI_IF_UART_RTS_PIN) */

#if (HPI_IF_UART_CTS_PIN)
    #include "HPI_IF_cts.h"
#endif /* (HPI_IF_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (HPI_IF_RX_SCL_MOSI_PIN)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    
    #define HPI_IF_RX_SCL_MOSI_HSIOM_MASK      (HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_POS       (HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_SEL_GPIO  (HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_SEL_I2C   (HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_SEL_SPI   (HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define HPI_IF_RX_SCL_MOSI_HSIOM_SEL_UART  (HPI_IF_uart_rx_i2c_scl_spi_mosi__0__HSIOM_UART)
    
#elif (HPI_IF_RX_WAKE_SCL_MOSI_PIN)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG   (*(reg32 *) HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_PTR   ( (reg32 *) HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_MASK      (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_POS       (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_SEL_GPIO  (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C   (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI   (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART  (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_UART)    
   
    #define HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_REG (*(reg32 *) HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_PTR ( (reg32 *) HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (HPI_IF_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK ((uint32) HPI_IF_INTCFG_TYPE_MASK << \
                                                                           HPI_IF_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#else
    /* None of pins HPI_IF_RX_SCL_MOSI_PIN or HPI_IF_RX_WAKE_SCL_MOSI_PIN present.*/
#endif /* (HPI_IF_RX_SCL_MOSI_PIN) */

#if (HPI_IF_TX_SDA_MISO_PIN)
    #define HPI_IF_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define HPI_IF_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    
    #define HPI_IF_TX_SDA_MISO_HSIOM_MASK      (HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define HPI_IF_TX_SDA_MISO_HSIOM_POS       (HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
    #define HPI_IF_TX_SDA_MISO_HSIOM_SEL_GPIO  (HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM_GPIO)
    #define HPI_IF_TX_SDA_MISO_HSIOM_SEL_I2C   (HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM_I2C)
    #define HPI_IF_TX_SDA_MISO_HSIOM_SEL_SPI   (HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM_SPI)
    #define HPI_IF_TX_SDA_MISO_HSIOM_SEL_UART  (HPI_IF_uart_tx_i2c_sda_spi_miso__0__HSIOM_UART)
#endif /* (HPI_IF_TX_SDA_MISO_PIN) */

#if (HPI_IF_CTS_SCLK_PIN)
    #define HPI_IF_CTS_SCLK_HSIOM_REG   (*(reg32 *) HPI_IF_uart_cts_spi_sclk__0__HSIOM)
    #define HPI_IF_CTS_SCLK_HSIOM_PTR   ( (reg32 *) HPI_IF_uart_cts_spi_sclk__0__HSIOM)
    
    #define HPI_IF_CTS_SCLK_HSIOM_MASK      (HPI_IF_uart_cts_spi_sclk__0__HSIOM_MASK)
    #define HPI_IF_CTS_SCLK_HSIOM_POS       (HPI_IF_uart_cts_spi_sclk__0__HSIOM_SHIFT)
    #define HPI_IF_CTS_SCLK_HSIOM_SEL_GPIO  (HPI_IF_uart_cts_spi_sclk__0__HSIOM_GPIO)
    #define HPI_IF_CTS_SCLK_HSIOM_SEL_I2C   (HPI_IF_uart_cts_spi_sclk__0__HSIOM_I2C)
    #define HPI_IF_CTS_SCLK_HSIOM_SEL_SPI   (HPI_IF_uart_cts_spi_sclk__0__HSIOM_SPI)
    #define HPI_IF_CTS_SCLK_HSIOM_SEL_UART  (HPI_IF_uart_cts_spi_sclk__0__HSIOM_UART)
#endif /* (HPI_IF_CTS_SCLK_PIN) */

#if (HPI_IF_RTS_SS0_PIN)
    #define HPI_IF_RTS_SS0_HSIOM_REG   (*(reg32 *) HPI_IF_uart_rts_spi_ss0__0__HSIOM)
    #define HPI_IF_RTS_SS0_HSIOM_PTR   ( (reg32 *) HPI_IF_uart_rts_spi_ss0__0__HSIOM)
    
    #define HPI_IF_RTS_SS0_HSIOM_MASK      (HPI_IF_uart_rts_spi_ss0__0__HSIOM_MASK)
    #define HPI_IF_RTS_SS0_HSIOM_POS       (HPI_IF_uart_rts_spi_ss0__0__HSIOM_SHIFT)
    #define HPI_IF_RTS_SS0_HSIOM_SEL_GPIO  (HPI_IF_uart_rts_spi_ss0__0__HSIOM_GPIO)
    #define HPI_IF_RTS_SS0_HSIOM_SEL_I2C   (HPI_IF_uart_rts_spi_ss0__0__HSIOM_I2C)
    #define HPI_IF_RTS_SS0_HSIOM_SEL_SPI   (HPI_IF_uart_rts_spi_ss0__0__HSIOM_SPI)
#if !(HPI_IF_CY_SCBIP_V0 || HPI_IF_CY_SCBIP_V1)
    #define HPI_IF_RTS_SS0_HSIOM_SEL_UART  (HPI_IF_uart_rts_spi_ss0__0__HSIOM_UART)
#endif /* !(HPI_IF_CY_SCBIP_V0 || HPI_IF_CY_SCBIP_V1) */
#endif /* (HPI_IF_RTS_SS0_PIN) */

#if (HPI_IF_SS1_PIN)
    #define HPI_IF_SS1_HSIOM_REG  (*(reg32 *) HPI_IF_spi_ss1__0__HSIOM)
    #define HPI_IF_SS1_HSIOM_PTR  ( (reg32 *) HPI_IF_spi_ss1__0__HSIOM)
    
    #define HPI_IF_SS1_HSIOM_MASK     (HPI_IF_spi_ss1__0__HSIOM_MASK)
    #define HPI_IF_SS1_HSIOM_POS      (HPI_IF_spi_ss1__0__HSIOM_SHIFT)
    #define HPI_IF_SS1_HSIOM_SEL_GPIO (HPI_IF_spi_ss1__0__HSIOM_GPIO)
    #define HPI_IF_SS1_HSIOM_SEL_I2C  (HPI_IF_spi_ss1__0__HSIOM_I2C)
    #define HPI_IF_SS1_HSIOM_SEL_SPI  (HPI_IF_spi_ss1__0__HSIOM_SPI)
#endif /* (HPI_IF_SS1_PIN) */

#if (HPI_IF_SS2_PIN)
    #define HPI_IF_SS2_HSIOM_REG     (*(reg32 *) HPI_IF_spi_ss2__0__HSIOM)
    #define HPI_IF_SS2_HSIOM_PTR     ( (reg32 *) HPI_IF_spi_ss2__0__HSIOM)
    
    #define HPI_IF_SS2_HSIOM_MASK     (HPI_IF_spi_ss2__0__HSIOM_MASK)
    #define HPI_IF_SS2_HSIOM_POS      (HPI_IF_spi_ss2__0__HSIOM_SHIFT)
    #define HPI_IF_SS2_HSIOM_SEL_GPIO (HPI_IF_spi_ss2__0__HSIOM_GPIO)
    #define HPI_IF_SS2_HSIOM_SEL_I2C  (HPI_IF_spi_ss2__0__HSIOM_I2C)
    #define HPI_IF_SS2_HSIOM_SEL_SPI  (HPI_IF_spi_ss2__0__HSIOM_SPI)
#endif /* (HPI_IF_SS2_PIN) */

#if (HPI_IF_SS3_PIN)
    #define HPI_IF_SS3_HSIOM_REG     (*(reg32 *) HPI_IF_spi_ss3__0__HSIOM)
    #define HPI_IF_SS3_HSIOM_PTR     ( (reg32 *) HPI_IF_spi_ss3__0__HSIOM)
    
    #define HPI_IF_SS3_HSIOM_MASK     (HPI_IF_spi_ss3__0__HSIOM_MASK)
    #define HPI_IF_SS3_HSIOM_POS      (HPI_IF_spi_ss3__0__HSIOM_SHIFT)
    #define HPI_IF_SS3_HSIOM_SEL_GPIO (HPI_IF_spi_ss3__0__HSIOM_GPIO)
    #define HPI_IF_SS3_HSIOM_SEL_I2C  (HPI_IF_spi_ss3__0__HSIOM_I2C)
    #define HPI_IF_SS3_HSIOM_SEL_SPI  (HPI_IF_spi_ss3__0__HSIOM_SPI)
#endif /* (HPI_IF_SS3_PIN) */

#if (HPI_IF_I2C_PINS)
    #define HPI_IF_SCL_HSIOM_REG  (*(reg32 *) HPI_IF_scl__0__HSIOM)
    #define HPI_IF_SCL_HSIOM_PTR  ( (reg32 *) HPI_IF_scl__0__HSIOM)
    
    #define HPI_IF_SCL_HSIOM_MASK     (HPI_IF_scl__0__HSIOM_MASK)
    #define HPI_IF_SCL_HSIOM_POS      (HPI_IF_scl__0__HSIOM_SHIFT)
    #define HPI_IF_SCL_HSIOM_SEL_GPIO (HPI_IF_sda__0__HSIOM_GPIO)
    #define HPI_IF_SCL_HSIOM_SEL_I2C  (HPI_IF_sda__0__HSIOM_I2C)
    
    #define HPI_IF_SDA_HSIOM_REG  (*(reg32 *) HPI_IF_sda__0__HSIOM)
    #define HPI_IF_SDA_HSIOM_PTR  ( (reg32 *) HPI_IF_sda__0__HSIOM)
    
    #define HPI_IF_SDA_HSIOM_MASK     (HPI_IF_sda__0__HSIOM_MASK)
    #define HPI_IF_SDA_HSIOM_POS      (HPI_IF_sda__0__HSIOM_SHIFT)
    #define HPI_IF_SDA_HSIOM_SEL_GPIO (HPI_IF_sda__0__HSIOM_GPIO)
    #define HPI_IF_SDA_HSIOM_SEL_I2C  (HPI_IF_sda__0__HSIOM_I2C)
#endif /* (HPI_IF_I2C_PINS) */

#if (HPI_IF_SPI_SLAVE_PINS)
    #define HPI_IF_SCLK_S_HSIOM_REG   (*(reg32 *) HPI_IF_sclk_s__0__HSIOM)
    #define HPI_IF_SCLK_S_HSIOM_PTR   ( (reg32 *) HPI_IF_sclk_s__0__HSIOM)
    
    #define HPI_IF_SCLK_S_HSIOM_MASK      (HPI_IF_sclk_s__0__HSIOM_MASK)
    #define HPI_IF_SCLK_S_HSIOM_POS       (HPI_IF_sclk_s__0__HSIOM_SHIFT)
    #define HPI_IF_SCLK_S_HSIOM_SEL_GPIO  (HPI_IF_sclk_s__0__HSIOM_GPIO)
    #define HPI_IF_SCLK_S_HSIOM_SEL_SPI   (HPI_IF_sclk_s__0__HSIOM_SPI)
    
    #define HPI_IF_SS0_S_HSIOM_REG    (*(reg32 *) HPI_IF_ss0_s__0__HSIOM)
    #define HPI_IF_SS0_S_HSIOM_PTR    ( (reg32 *) HPI_IF_ss0_s__0__HSIOM)
    
    #define HPI_IF_SS0_S_HSIOM_MASK       (HPI_IF_ss0_s__0__HSIOM_MASK)
    #define HPI_IF_SS0_S_HSIOM_POS        (HPI_IF_ss0_s__0__HSIOM_SHIFT)
    #define HPI_IF_SS0_S_HSIOM_SEL_GPIO   (HPI_IF_ss0_s__0__HSIOM_GPIO)  
    #define HPI_IF_SS0_S_HSIOM_SEL_SPI    (HPI_IF_ss0_s__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_SLAVE_PINS) */

#if (HPI_IF_SPI_SLAVE_MOSI_PIN)
    #define HPI_IF_MOSI_S_HSIOM_REG   (*(reg32 *) HPI_IF_mosi_s__0__HSIOM)
    #define HPI_IF_MOSI_S_HSIOM_PTR   ( (reg32 *) HPI_IF_mosi_s__0__HSIOM)
    
    #define HPI_IF_MOSI_S_HSIOM_MASK      (HPI_IF_mosi_s__0__HSIOM_MASK)
    #define HPI_IF_MOSI_S_HSIOM_POS       (HPI_IF_mosi_s__0__HSIOM_SHIFT)
    #define HPI_IF_MOSI_S_HSIOM_SEL_GPIO  (HPI_IF_mosi_s__0__HSIOM_GPIO)
    #define HPI_IF_MOSI_S_HSIOM_SEL_SPI   (HPI_IF_mosi_s__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_SLAVE_MOSI_PIN) */

#if (HPI_IF_SPI_SLAVE_MISO_PIN)
    #define HPI_IF_MISO_S_HSIOM_REG   (*(reg32 *) HPI_IF_miso_s__0__HSIOM)
    #define HPI_IF_MISO_S_HSIOM_PTR   ( (reg32 *) HPI_IF_miso_s__0__HSIOM)
    
    #define HPI_IF_MISO_S_HSIOM_MASK      (HPI_IF_miso_s__0__HSIOM_MASK)
    #define HPI_IF_MISO_S_HSIOM_POS       (HPI_IF_miso_s__0__HSIOM_SHIFT)
    #define HPI_IF_MISO_S_HSIOM_SEL_GPIO  (HPI_IF_miso_s__0__HSIOM_GPIO)
    #define HPI_IF_MISO_S_HSIOM_SEL_SPI   (HPI_IF_miso_s__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_SLAVE_MISO_PIN) */

#if (HPI_IF_SPI_MASTER_MISO_PIN)
    #define HPI_IF_MISO_M_HSIOM_REG   (*(reg32 *) HPI_IF_miso_m__0__HSIOM)
    #define HPI_IF_MISO_M_HSIOM_PTR   ( (reg32 *) HPI_IF_miso_m__0__HSIOM)
    
    #define HPI_IF_MISO_M_HSIOM_MASK      (HPI_IF_miso_m__0__HSIOM_MASK)
    #define HPI_IF_MISO_M_HSIOM_POS       (HPI_IF_miso_m__0__HSIOM_SHIFT)
    #define HPI_IF_MISO_M_HSIOM_SEL_GPIO  (HPI_IF_miso_m__0__HSIOM_GPIO)
    #define HPI_IF_MISO_M_HSIOM_SEL_SPI   (HPI_IF_miso_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_MISO_PIN) */

#if (HPI_IF_SPI_MASTER_MOSI_PIN)
    #define HPI_IF_MOSI_M_HSIOM_REG   (*(reg32 *) HPI_IF_mosi_m__0__HSIOM)
    #define HPI_IF_MOSI_M_HSIOM_PTR   ( (reg32 *) HPI_IF_mosi_m__0__HSIOM)
    
    #define HPI_IF_MOSI_M_HSIOM_MASK      (HPI_IF_mosi_m__0__HSIOM_MASK)
    #define HPI_IF_MOSI_M_HSIOM_POS       (HPI_IF_mosi_m__0__HSIOM_SHIFT)
    #define HPI_IF_MOSI_M_HSIOM_SEL_GPIO  (HPI_IF_mosi_m__0__HSIOM_GPIO)
    #define HPI_IF_MOSI_M_HSIOM_SEL_SPI   (HPI_IF_mosi_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_MOSI_PIN) */

#if (HPI_IF_SPI_MASTER_SCLK_PIN)
    #define HPI_IF_SCLK_M_HSIOM_REG   (*(reg32 *) HPI_IF_sclk_m__0__HSIOM)
    #define HPI_IF_SCLK_M_HSIOM_PTR   ( (reg32 *) HPI_IF_sclk_m__0__HSIOM)
    
    #define HPI_IF_SCLK_M_HSIOM_MASK      (HPI_IF_sclk_m__0__HSIOM_MASK)
    #define HPI_IF_SCLK_M_HSIOM_POS       (HPI_IF_sclk_m__0__HSIOM_SHIFT)
    #define HPI_IF_SCLK_M_HSIOM_SEL_GPIO  (HPI_IF_sclk_m__0__HSIOM_GPIO)
    #define HPI_IF_SCLK_M_HSIOM_SEL_SPI   (HPI_IF_sclk_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_SCLK_PIN) */

#if (HPI_IF_SPI_MASTER_SS0_PIN)
    #define HPI_IF_SS0_M_HSIOM_REG    (*(reg32 *) HPI_IF_ss0_m__0__HSIOM)
    #define HPI_IF_SS0_M_HSIOM_PTR    ( (reg32 *) HPI_IF_ss0_m__0__HSIOM)
    
    #define HPI_IF_SS0_M_HSIOM_MASK       (HPI_IF_ss0_m__0__HSIOM_MASK)
    #define HPI_IF_SS0_M_HSIOM_POS        (HPI_IF_ss0_m__0__HSIOM_SHIFT)
    #define HPI_IF_SS0_M_HSIOM_SEL_GPIO   (HPI_IF_ss0_m__0__HSIOM_GPIO)
    #define HPI_IF_SS0_M_HSIOM_SEL_SPI    (HPI_IF_ss0_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_SS0_PIN) */

#if (HPI_IF_SPI_MASTER_SS1_PIN)
    #define HPI_IF_SS1_M_HSIOM_REG    (*(reg32 *) HPI_IF_ss1_m__0__HSIOM)
    #define HPI_IF_SS1_M_HSIOM_PTR    ( (reg32 *) HPI_IF_ss1_m__0__HSIOM)
    
    #define HPI_IF_SS1_M_HSIOM_MASK       (HPI_IF_ss1_m__0__HSIOM_MASK)
    #define HPI_IF_SS1_M_HSIOM_POS        (HPI_IF_ss1_m__0__HSIOM_SHIFT)
    #define HPI_IF_SS1_M_HSIOM_SEL_GPIO   (HPI_IF_ss1_m__0__HSIOM_GPIO)
    #define HPI_IF_SS1_M_HSIOM_SEL_SPI    (HPI_IF_ss1_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_SS1_PIN) */

#if (HPI_IF_SPI_MASTER_SS2_PIN)
    #define HPI_IF_SS2_M_HSIOM_REG    (*(reg32 *) HPI_IF_ss2_m__0__HSIOM)
    #define HPI_IF_SS2_M_HSIOM_PTR    ( (reg32 *) HPI_IF_ss2_m__0__HSIOM)
    
    #define HPI_IF_SS2_M_HSIOM_MASK       (HPI_IF_ss2_m__0__HSIOM_MASK)
    #define HPI_IF_SS2_M_HSIOM_POS        (HPI_IF_ss2_m__0__HSIOM_SHIFT)
    #define HPI_IF_SS2_M_HSIOM_SEL_GPIO   (HPI_IF_ss2_m__0__HSIOM_GPIO)
    #define HPI_IF_SS2_M_HSIOM_SEL_SPI    (HPI_IF_ss2_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_SS2_PIN) */

#if (HPI_IF_SPI_MASTER_SS3_PIN)
    #define HPI_IF_SS3_M_HSIOM_REG    (*(reg32 *) HPI_IF_ss3_m__0__HSIOM)
    #define HPI_IF_SS3_M_HSIOM_PTR    ( (reg32 *) HPI_IF_ss3_m__0__HSIOM)
    
    #define HPI_IF_SS3_M_HSIOM_MASK      (HPI_IF_ss3_m__0__HSIOM_MASK)
    #define HPI_IF_SS3_M_HSIOM_POS       (HPI_IF_ss3_m__0__HSIOM_SHIFT)
    #define HPI_IF_SS3_M_HSIOM_SEL_GPIO  (HPI_IF_ss3_m__0__HSIOM_GPIO)
    #define HPI_IF_SS3_M_HSIOM_SEL_SPI   (HPI_IF_ss3_m__0__HSIOM_SPI)
#endif /* (HPI_IF_SPI_MASTER_SS3_PIN) */

#if (HPI_IF_UART_RX_PIN)
    #define HPI_IF_RX_HSIOM_REG   (*(reg32 *) HPI_IF_rx__0__HSIOM)
    #define HPI_IF_RX_HSIOM_PTR   ( (reg32 *) HPI_IF_rx__0__HSIOM)
    
    #define HPI_IF_RX_HSIOM_MASK      (HPI_IF_rx__0__HSIOM_MASK)
    #define HPI_IF_RX_HSIOM_POS       (HPI_IF_rx__0__HSIOM_SHIFT)
    #define HPI_IF_RX_HSIOM_SEL_GPIO  (HPI_IF_rx__0__HSIOM_GPIO)
    #define HPI_IF_RX_HSIOM_SEL_UART  (HPI_IF_rx__0__HSIOM_UART)
#endif /* (HPI_IF_UART_RX_PIN) */

#if (HPI_IF_UART_RX_WAKE_PIN)
    #define HPI_IF_RX_WAKE_HSIOM_REG   (*(reg32 *) HPI_IF_rx_wake__0__HSIOM)
    #define HPI_IF_RX_WAKE_HSIOM_PTR   ( (reg32 *) HPI_IF_rx_wake__0__HSIOM)
    
    #define HPI_IF_RX_WAKE_HSIOM_MASK      (HPI_IF_rx_wake__0__HSIOM_MASK)
    #define HPI_IF_RX_WAKE_HSIOM_POS       (HPI_IF_rx_wake__0__HSIOM_SHIFT)
    #define HPI_IF_RX_WAKE_HSIOM_SEL_GPIO  (HPI_IF_rx_wake__0__HSIOM_GPIO)
    #define HPI_IF_RX_WAKE_HSIOM_SEL_UART  (HPI_IF_rx_wake__0__HSIOM_UART)
#endif /* (HPI_IF_UART_WAKE_RX_PIN) */

#if (HPI_IF_UART_CTS_PIN)
    #define HPI_IF_CTS_HSIOM_REG   (*(reg32 *) HPI_IF_cts__0__HSIOM)
    #define HPI_IF_CTS_HSIOM_PTR   ( (reg32 *) HPI_IF_cts__0__HSIOM)
    
    #define HPI_IF_CTS_HSIOM_MASK      (HPI_IF_cts__0__HSIOM_MASK)
    #define HPI_IF_CTS_HSIOM_POS       (HPI_IF_cts__0__HSIOM_SHIFT)
    #define HPI_IF_CTS_HSIOM_SEL_GPIO  (HPI_IF_cts__0__HSIOM_GPIO)
    #define HPI_IF_CTS_HSIOM_SEL_UART  (HPI_IF_cts__0__HSIOM_UART)
#endif /* (HPI_IF_UART_CTS_PIN) */

#if (HPI_IF_UART_TX_PIN)
    #define HPI_IF_TX_HSIOM_REG   (*(reg32 *) HPI_IF_tx__0__HSIOM)
    #define HPI_IF_TX_HSIOM_PTR   ( (reg32 *) HPI_IF_tx__0__HSIOM)
    
    #define HPI_IF_TX_HSIOM_MASK      (HPI_IF_tx__0__HSIOM_MASK)
    #define HPI_IF_TX_HSIOM_POS       (HPI_IF_tx__0__HSIOM_SHIFT)
    #define HPI_IF_TX_HSIOM_SEL_GPIO  (HPI_IF_tx__0__HSIOM_GPIO)
    #define HPI_IF_TX_HSIOM_SEL_UART  (HPI_IF_tx__0__HSIOM_UART)
#endif /* (HPI_IF_UART_TX_PIN) */

#if (HPI_IF_UART_RX_TX_PIN)
    #define HPI_IF_RX_TX_HSIOM_REG   (*(reg32 *) HPI_IF_rx_tx__0__HSIOM)
    #define HPI_IF_RX_TX_HSIOM_PTR   ( (reg32 *) HPI_IF_rx_tx__0__HSIOM)
    
    #define HPI_IF_RX_TX_HSIOM_MASK      (HPI_IF_rx_tx__0__HSIOM_MASK)
    #define HPI_IF_RX_TX_HSIOM_POS       (HPI_IF_rx_tx__0__HSIOM_SHIFT)
    #define HPI_IF_RX_TX_HSIOM_SEL_GPIO  (HPI_IF_rx_tx__0__HSIOM_GPIO)
    #define HPI_IF_RX_TX_HSIOM_SEL_UART  (HPI_IF_rx_tx__0__HSIOM_UART)
#endif /* (HPI_IF_UART_RX_TX_PIN) */

#if (HPI_IF_UART_RTS_PIN)
    #define HPI_IF_RTS_HSIOM_REG      (*(reg32 *) HPI_IF_rts__0__HSIOM)
    #define HPI_IF_RTS_HSIOM_PTR      ( (reg32 *) HPI_IF_rts__0__HSIOM)
    
    #define HPI_IF_RTS_HSIOM_MASK     (HPI_IF_rts__0__HSIOM_MASK)
    #define HPI_IF_RTS_HSIOM_POS      (HPI_IF_rts__0__HSIOM_SHIFT)    
    #define HPI_IF_RTS_HSIOM_SEL_GPIO (HPI_IF_rts__0__HSIOM_GPIO)
    #define HPI_IF_RTS_HSIOM_SEL_UART (HPI_IF_rts__0__HSIOM_UART)    
#endif /* (HPI_IF_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* HSIOM switch values. */ 
#define HPI_IF_HSIOM_DEF_SEL      (0x00u)
#define HPI_IF_HSIOM_GPIO_SEL     (0x00u)
/* The HSIOM values provided below are valid only for HPI_IF_CY_SCBIP_V0 
* and HPI_IF_CY_SCBIP_V1. It is not recommended to use them for 
* HPI_IF_CY_SCBIP_V2. Use pin name specific HSIOM constants provided 
* above instead for any SCB IP block version.
*/
#define HPI_IF_HSIOM_UART_SEL     (0x09u)
#define HPI_IF_HSIOM_I2C_SEL      (0x0Eu)
#define HPI_IF_HSIOM_SPI_SEL      (0x0Fu)

/* Pins settings index. */
#define HPI_IF_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define HPI_IF_RX_SCL_MOSI_PIN_INDEX       (0u)
#define HPI_IF_TX_SDA_MISO_PIN_INDEX       (1u)
#define HPI_IF_CTS_SCLK_PIN_INDEX       (2u)
#define HPI_IF_RTS_SS0_PIN_INDEX       (3u)
#define HPI_IF_SS1_PIN_INDEX                  (4u)
#define HPI_IF_SS2_PIN_INDEX                  (5u)
#define HPI_IF_SS3_PIN_INDEX                  (6u)

/* Pins settings mask. */
#define HPI_IF_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << HPI_IF_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define HPI_IF_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << HPI_IF_RX_SCL_MOSI_PIN_INDEX)
#define HPI_IF_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << HPI_IF_TX_SDA_MISO_PIN_INDEX)
#define HPI_IF_CTS_SCLK_PIN_MASK     ((uint32) 0x01u << HPI_IF_CTS_SCLK_PIN_INDEX)
#define HPI_IF_RTS_SS0_PIN_MASK     ((uint32) 0x01u << HPI_IF_RTS_SS0_PIN_INDEX)
#define HPI_IF_SS1_PIN_MASK                ((uint32) 0x01u << HPI_IF_SS1_PIN_INDEX)
#define HPI_IF_SS2_PIN_MASK                ((uint32) 0x01u << HPI_IF_SS2_PIN_INDEX)
#define HPI_IF_SS3_PIN_MASK                ((uint32) 0x01u << HPI_IF_SS3_PIN_INDEX)

/* Pin interrupt constants. */
#define HPI_IF_INTCFG_TYPE_MASK           (0x03u)
#define HPI_IF_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants. */
#define HPI_IF_PIN_DM_ALG_HIZ  (0u)
#define HPI_IF_PIN_DM_DIG_HIZ  (1u)
#define HPI_IF_PIN_DM_OD_LO    (4u)
#define HPI_IF_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define HPI_IF_DM_MASK    (0x7u)
#define HPI_IF_DM_SIZE    (3u)
#define HPI_IF_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) HPI_IF_DM_MASK << (HPI_IF_DM_SIZE * (pos)))) >> \
                                                              (HPI_IF_DM_SIZE * (pos)) )

#if (HPI_IF_TX_SDA_MISO_PIN)
    #define HPI_IF_CHECK_TX_SDA_MISO_PIN_USED \
                (HPI_IF_PIN_DM_ALG_HIZ != \
                    HPI_IF_GET_P4_PIN_DM(HPI_IF_uart_tx_i2c_sda_spi_miso_PC, \
                                                   HPI_IF_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (HPI_IF_TX_SDA_MISO_PIN) */

#if (HPI_IF_RTS_SS0_PIN)
    #define HPI_IF_CHECK_RTS_SS0_PIN_USED \
                (HPI_IF_PIN_DM_ALG_HIZ != \
                    HPI_IF_GET_P4_PIN_DM(HPI_IF_uart_rts_spi_ss0_PC, \
                                                   HPI_IF_uart_rts_spi_ss0_SHIFT))
#endif /* (HPI_IF_RTS_SS0_PIN) */

/* Set bits-mask in register */
#define HPI_IF_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define HPI_IF_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define HPI_IF_SET_HSIOM_SEL(reg, mask, pos, sel) HPI_IF_SET_REGISTER_BITS(reg, mask, pos, sel)
#define HPI_IF_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        HPI_IF_SET_REGISTER_BITS(reg, mask, pos, intType)
#define HPI_IF_SET_INP_DIS(reg, mask, val) HPI_IF_SET_REGISTER_BIT(reg, mask, val)

/* HPI_IF_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  HPI_IF_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (HPI_IF_CY_SCBIP_V0)
#if (HPI_IF_I2C_PINS)
    #define HPI_IF_SET_I2C_SCL_DR(val) HPI_IF_scl_Write(val)

    #define HPI_IF_SET_I2C_SCL_HSIOM_SEL(sel) \
                          HPI_IF_SET_HSIOM_SEL(HPI_IF_SCL_HSIOM_REG,  \
                                                         HPI_IF_SCL_HSIOM_MASK, \
                                                         HPI_IF_SCL_HSIOM_POS,  \
                                                         (sel))
    #define HPI_IF_WAIT_SCL_SET_HIGH  (0u == HPI_IF_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (HPI_IF_RX_WAKE_SCL_MOSI_PIN)
    #define HPI_IF_SET_I2C_SCL_DR(val) \
                            HPI_IF_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define HPI_IF_SET_I2C_SCL_HSIOM_SEL(sel) \
                    HPI_IF_SET_HSIOM_SEL(HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define HPI_IF_WAIT_SCL_SET_HIGH  (0u == HPI_IF_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (HPI_IF_RX_SCL_MOSI_PIN)
    #define HPI_IF_SET_I2C_SCL_DR(val) \
                            HPI_IF_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define HPI_IF_SET_I2C_SCL_HSIOM_SEL(sel) \
                            HPI_IF_SET_HSIOM_SEL(HPI_IF_RX_SCL_MOSI_HSIOM_REG,  \
                                                           HPI_IF_RX_SCL_MOSI_HSIOM_MASK, \
                                                           HPI_IF_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define HPI_IF_WAIT_SCL_SET_HIGH  (0u == HPI_IF_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define HPI_IF_SET_I2C_SCL_DR(val)        do{ /* Does nothing */ }while(0)
    #define HPI_IF_SET_I2C_SCL_HSIOM_SEL(sel) do{ /* Does nothing */ }while(0)

    #define HPI_IF_WAIT_SCL_SET_HIGH  (0u)
#endif /* (HPI_IF_I2C_PINS) */

/* SCB I2C: sda signal */
#if (HPI_IF_I2C_PINS)
    #define HPI_IF_WAIT_SDA_SET_HIGH  (0u == HPI_IF_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (HPI_IF_TX_SDA_MISO_PIN)
    #define HPI_IF_WAIT_SDA_SET_HIGH  (0u == HPI_IF_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define HPI_IF_WAIT_SDA_SET_HIGH  (0u)
#endif /* (HPI_IF_MOSI_SCL_RX_PIN) */
#endif /* (HPI_IF_CY_SCBIP_V0) */

/* Clear UART wakeup source */
#if (HPI_IF_RX_SCL_MOSI_PIN)
    #define HPI_IF_CLEAR_UART_RX_WAKE_INTR        do{ /* Does nothing */ }while(0)
    
#elif (HPI_IF_RX_WAKE_SCL_MOSI_PIN)
    #define HPI_IF_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) HPI_IF_uart_rx_wake_i2c_scl_spi_mosi_ClearInterrupt(); \
            }while(0)

#elif(HPI_IF_UART_RX_WAKE_PIN)
    #define HPI_IF_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) HPI_IF_rx_wake_ClearInterrupt(); \
            }while(0)
#else
#endif /* (HPI_IF_RX_SCL_MOSI_PIN) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define HPI_IF_REMOVE_MOSI_SCL_RX_WAKE_PIN    HPI_IF_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define HPI_IF_REMOVE_MOSI_SCL_RX_PIN         HPI_IF_REMOVE_RX_SCL_MOSI_PIN
#define HPI_IF_REMOVE_MISO_SDA_TX_PIN         HPI_IF_REMOVE_TX_SDA_MISO_PIN
#ifndef HPI_IF_REMOVE_SCLK_PIN
#define HPI_IF_REMOVE_SCLK_PIN                HPI_IF_REMOVE_CTS_SCLK_PIN
#endif /* HPI_IF_REMOVE_SCLK_PIN */
#ifndef HPI_IF_REMOVE_SS0_PIN
#define HPI_IF_REMOVE_SS0_PIN                 HPI_IF_REMOVE_RTS_SS0_PIN
#endif /* HPI_IF_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define HPI_IF_MOSI_SCL_RX_WAKE_PIN   HPI_IF_RX_WAKE_SCL_MOSI_PIN
#define HPI_IF_MOSI_SCL_RX_PIN        HPI_IF_RX_SCL_MOSI_PIN
#define HPI_IF_MISO_SDA_TX_PIN        HPI_IF_TX_SDA_MISO_PIN
#ifndef HPI_IF_SCLK_PIN
#define HPI_IF_SCLK_PIN               HPI_IF_CTS_SCLK_PIN
#endif /* HPI_IF_SCLK_PIN */
#ifndef HPI_IF_SS0_PIN
#define HPI_IF_SS0_PIN                HPI_IF_RTS_SS0_PIN
#endif /* HPI_IF_SS0_PIN */

#if (HPI_IF_MOSI_SCL_RX_WAKE_PIN)
    #define HPI_IF_MOSI_SCL_RX_WAKE_HSIOM_REG     HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define HPI_IF_MOSI_SCL_RX_WAKE_HSIOM_PTR     HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define HPI_IF_MOSI_SCL_RX_WAKE_HSIOM_MASK    HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define HPI_IF_MOSI_SCL_RX_WAKE_HSIOM_POS     HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define HPI_IF_MOSI_SCL_RX_WAKE_INTCFG_REG    HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define HPI_IF_MOSI_SCL_RX_WAKE_INTCFG_PTR    HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define HPI_IF_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define HPI_IF_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  HPI_IF_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (HPI_IF_RX_WAKE_SCL_MOSI_PIN) */

#if (HPI_IF_MOSI_SCL_RX_PIN)
    #define HPI_IF_MOSI_SCL_RX_HSIOM_REG      HPI_IF_RX_SCL_MOSI_HSIOM_REG
    #define HPI_IF_MOSI_SCL_RX_HSIOM_PTR      HPI_IF_RX_SCL_MOSI_HSIOM_PTR
    #define HPI_IF_MOSI_SCL_RX_HSIOM_MASK     HPI_IF_RX_SCL_MOSI_HSIOM_MASK
    #define HPI_IF_MOSI_SCL_RX_HSIOM_POS      HPI_IF_RX_SCL_MOSI_HSIOM_POS
#endif /* (HPI_IF_MOSI_SCL_RX_PIN) */

#if (HPI_IF_MISO_SDA_TX_PIN)
    #define HPI_IF_MISO_SDA_TX_HSIOM_REG      HPI_IF_TX_SDA_MISO_HSIOM_REG
    #define HPI_IF_MISO_SDA_TX_HSIOM_PTR      HPI_IF_TX_SDA_MISO_HSIOM_REG
    #define HPI_IF_MISO_SDA_TX_HSIOM_MASK     HPI_IF_TX_SDA_MISO_HSIOM_REG
    #define HPI_IF_MISO_SDA_TX_HSIOM_POS      HPI_IF_TX_SDA_MISO_HSIOM_REG
#endif /* (HPI_IF_MISO_SDA_TX_PIN_PIN) */

#if (HPI_IF_SCLK_PIN)
    #ifndef HPI_IF_SCLK_HSIOM_REG
    #define HPI_IF_SCLK_HSIOM_REG     HPI_IF_CTS_SCLK_HSIOM_REG
    #define HPI_IF_SCLK_HSIOM_PTR     HPI_IF_CTS_SCLK_HSIOM_PTR
    #define HPI_IF_SCLK_HSIOM_MASK    HPI_IF_CTS_SCLK_HSIOM_MASK
    #define HPI_IF_SCLK_HSIOM_POS     HPI_IF_CTS_SCLK_HSIOM_POS
    #endif /* HPI_IF_SCLK_HSIOM_REG */
#endif /* (HPI_IF_SCLK_PIN) */

#if (HPI_IF_SS0_PIN)
    #ifndef HPI_IF_SS0_HSIOM_REG
    #define HPI_IF_SS0_HSIOM_REG      HPI_IF_RTS_SS0_HSIOM_REG
    #define HPI_IF_SS0_HSIOM_PTR      HPI_IF_RTS_SS0_HSIOM_PTR
    #define HPI_IF_SS0_HSIOM_MASK     HPI_IF_RTS_SS0_HSIOM_MASK
    #define HPI_IF_SS0_HSIOM_POS      HPI_IF_RTS_SS0_HSIOM_POS
    #endif /* HPI_IF_SS0_HSIOM_REG */
#endif /* (HPI_IF_SS0_PIN) */

#define HPI_IF_MOSI_SCL_RX_WAKE_PIN_INDEX HPI_IF_RX_WAKE_SCL_MOSI_PIN_INDEX
#define HPI_IF_MOSI_SCL_RX_PIN_INDEX      HPI_IF_RX_SCL_MOSI_PIN_INDEX
#define HPI_IF_MISO_SDA_TX_PIN_INDEX      HPI_IF_TX_SDA_MISO_PIN_INDEX
#ifndef HPI_IF_SCLK_PIN_INDEX
#define HPI_IF_SCLK_PIN_INDEX             HPI_IF_CTS_SCLK_PIN_INDEX
#endif /* HPI_IF_SCLK_PIN_INDEX */
#ifndef HPI_IF_SS0_PIN_INDEX
#define HPI_IF_SS0_PIN_INDEX              HPI_IF_RTS_SS0_PIN_INDEX
#endif /* HPI_IF_SS0_PIN_INDEX */

#define HPI_IF_MOSI_SCL_RX_WAKE_PIN_MASK HPI_IF_RX_WAKE_SCL_MOSI_PIN_MASK
#define HPI_IF_MOSI_SCL_RX_PIN_MASK      HPI_IF_RX_SCL_MOSI_PIN_MASK
#define HPI_IF_MISO_SDA_TX_PIN_MASK      HPI_IF_TX_SDA_MISO_PIN_MASK
#ifndef HPI_IF_SCLK_PIN_MASK
#define HPI_IF_SCLK_PIN_MASK             HPI_IF_CTS_SCLK_PIN_MASK
#endif /* HPI_IF_SCLK_PIN_MASK */
#ifndef HPI_IF_SS0_PIN_MASK
#define HPI_IF_SS0_PIN_MASK              HPI_IF_RTS_SS0_PIN_MASK
#endif /* HPI_IF_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_HPI_IF_H) */


/* [] END OF FILE */
