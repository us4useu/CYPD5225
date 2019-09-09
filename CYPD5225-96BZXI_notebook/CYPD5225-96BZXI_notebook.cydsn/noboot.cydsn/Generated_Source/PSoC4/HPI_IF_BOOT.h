/***************************************************************************//**
* \file HPI_IF_BOOT.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values of the bootloader
*  communication APIs for the SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2014-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_BOOT_HPI_IF_H)
#define CY_SCB_BOOT_HPI_IF_H

#include "HPI_IF_PVT.h"

#if (HPI_IF_SCB_MODE_I2C_INC)
    #include "HPI_IF_I2C.h"
#endif /* (HPI_IF_SCB_MODE_I2C_INC) */

#if (HPI_IF_SCB_MODE_EZI2C_INC)
    #include "HPI_IF_EZI2C.h"
#endif /* (HPI_IF_SCB_MODE_EZI2C_INC) */

#if (HPI_IF_SCB_MODE_SPI_INC || HPI_IF_SCB_MODE_UART_INC)
    #include "HPI_IF_SPI_UART.h"
#endif /* (HPI_IF_SCB_MODE_SPI_INC || HPI_IF_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define HPI_IF_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_HPI_IF) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (HPI_IF_SCB_MODE_I2C_INC)
    #define HPI_IF_I2C_BTLDR_COMM_ENABLED     (HPI_IF_BTLDR_COMM_ENABLED && \
                                                            (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             HPI_IF_I2C_SLAVE_CONST))
#else
     #define HPI_IF_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (HPI_IF_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (HPI_IF_SCB_MODE_EZI2C_INC)
    #define HPI_IF_EZI2C_BTLDR_COMM_ENABLED   (HPI_IF_BTLDR_COMM_ENABLED && \
                                                         HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define HPI_IF_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (HPI_IF_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (HPI_IF_SCB_MODE_SPI_INC)
    #define HPI_IF_SPI_BTLDR_COMM_ENABLED     (HPI_IF_BTLDR_COMM_ENABLED && \
                                                            (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             HPI_IF_SPI_SLAVE_CONST))
#else
        #define HPI_IF_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (HPI_IF_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (HPI_IF_SCB_MODE_UART_INC)
       #define HPI_IF_UART_BTLDR_COMM_ENABLED    (HPI_IF_BTLDR_COMM_ENABLED && \
                                                            (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (HPI_IF_UART_RX_DIRECTION && \
                                                              HPI_IF_UART_TX_DIRECTION)))
#else
     #define HPI_IF_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (HPI_IF_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define HPI_IF_BTLDR_COMM_MODE_ENABLED    (HPI_IF_I2C_BTLDR_COMM_ENABLED   || \
                                                     HPI_IF_SPI_BTLDR_COMM_ENABLED   || \
                                                     HPI_IF_EZI2C_BTLDR_COMM_ENABLED || \
                                                     HPI_IF_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void HPI_IF_I2CCyBtldrCommStart(void);
    void HPI_IF_I2CCyBtldrCommStop (void);
    void HPI_IF_I2CCyBtldrCommReset(void);
    cystatus HPI_IF_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus HPI_IF_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (HPI_IF_SCB_MODE_I2C_CONST_CFG)
        #define HPI_IF_CyBtldrCommStart   HPI_IF_I2CCyBtldrCommStart
        #define HPI_IF_CyBtldrCommStop    HPI_IF_I2CCyBtldrCommStop
        #define HPI_IF_CyBtldrCommReset   HPI_IF_I2CCyBtldrCommReset
        #define HPI_IF_CyBtldrCommRead    HPI_IF_I2CCyBtldrCommRead
        #define HPI_IF_CyBtldrCommWrite   HPI_IF_I2CCyBtldrCommWrite
    #endif /* (HPI_IF_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void HPI_IF_EzI2CCyBtldrCommStart(void);
    void HPI_IF_EzI2CCyBtldrCommStop (void);
    void HPI_IF_EzI2CCyBtldrCommReset(void);
    cystatus HPI_IF_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus HPI_IF_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (HPI_IF_SCB_MODE_EZI2C_CONST_CFG)
        #define HPI_IF_CyBtldrCommStart   HPI_IF_EzI2CCyBtldrCommStart
        #define HPI_IF_CyBtldrCommStop    HPI_IF_EzI2CCyBtldrCommStop
        #define HPI_IF_CyBtldrCommReset   HPI_IF_EzI2CCyBtldrCommReset
        #define HPI_IF_CyBtldrCommRead    HPI_IF_EzI2CCyBtldrCommRead
        #define HPI_IF_CyBtldrCommWrite   HPI_IF_EzI2CCyBtldrCommWrite
    #endif /* (HPI_IF_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void HPI_IF_SpiCyBtldrCommStart(void);
    void HPI_IF_SpiCyBtldrCommStop (void);
    void HPI_IF_SpiCyBtldrCommReset(void);
    cystatus HPI_IF_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus HPI_IF_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (HPI_IF_SCB_MODE_SPI_CONST_CFG)
        #define HPI_IF_CyBtldrCommStart   HPI_IF_SpiCyBtldrCommStart
        #define HPI_IF_CyBtldrCommStop    HPI_IF_SpiCyBtldrCommStop
        #define HPI_IF_CyBtldrCommReset   HPI_IF_SpiCyBtldrCommReset
        #define HPI_IF_CyBtldrCommRead    HPI_IF_SpiCyBtldrCommRead
        #define HPI_IF_CyBtldrCommWrite   HPI_IF_SpiCyBtldrCommWrite
    #endif /* (HPI_IF_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void HPI_IF_UartCyBtldrCommStart(void);
    void HPI_IF_UartCyBtldrCommStop (void);
    void HPI_IF_UartCyBtldrCommReset(void);
    cystatus HPI_IF_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus HPI_IF_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (HPI_IF_SCB_MODE_UART_CONST_CFG)
        #define HPI_IF_CyBtldrCommStart   HPI_IF_UartCyBtldrCommStart
        #define HPI_IF_CyBtldrCommStop    HPI_IF_UartCyBtldrCommStop
        #define HPI_IF_CyBtldrCommReset   HPI_IF_UartCyBtldrCommReset
        #define HPI_IF_CyBtldrCommRead    HPI_IF_UartCyBtldrCommRead
        #define HPI_IF_CyBtldrCommWrite   HPI_IF_UartCyBtldrCommWrite
    #endif /* (HPI_IF_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_BTLDR_COMM_ENABLED)
    #if (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void HPI_IF_CyBtldrCommStart(void);
        void HPI_IF_CyBtldrCommStop (void);
        void HPI_IF_CyBtldrCommReset(void);
        cystatus HPI_IF_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus HPI_IF_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_HPI_IF)
        #define CyBtldrCommStart    HPI_IF_CyBtldrCommStart
        #define CyBtldrCommStop     HPI_IF_CyBtldrCommStop
        #define CyBtldrCommReset    HPI_IF_CyBtldrCommReset
        #define CyBtldrCommWrite    HPI_IF_CyBtldrCommWrite
        #define CyBtldrCommRead     HPI_IF_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_HPI_IF) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define HPI_IF_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define HPI_IF_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define HPI_IF_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define HPI_IF_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef HPI_IF_SPI_BYTE_TO_BYTE
    #define HPI_IF_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef HPI_IF_UART_BYTE_TO_BYTE
    #define HPI_IF_UART_BYTE_TO_BYTE  (2500u)
#endif /* HPI_IF_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_HPI_IF_H) */


/* [] END OF FILE */
