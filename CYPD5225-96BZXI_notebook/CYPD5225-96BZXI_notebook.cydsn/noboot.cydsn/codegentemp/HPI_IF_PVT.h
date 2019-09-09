/***************************************************************************//**
* \file .h
* \version 4.0
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_HPI_IF_H)
#define CY_SCB_PVT_HPI_IF_H

#include "HPI_IF.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define HPI_IF_SetI2CExtClkInterruptMode(interruptMask) HPI_IF_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define HPI_IF_ClearI2CExtClkInterruptSource(interruptMask) HPI_IF_CLEAR_INTR_I2C_EC(interruptMask)
#define HPI_IF_GetI2CExtClkInterruptSource()                (HPI_IF_INTR_I2C_EC_REG)
#define HPI_IF_GetI2CExtClkInterruptMode()                  (HPI_IF_INTR_I2C_EC_MASK_REG)
#define HPI_IF_GetI2CExtClkInterruptSourceMasked()          (HPI_IF_INTR_I2C_EC_MASKED_REG)

#if (!HPI_IF_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define HPI_IF_SetSpiExtClkInterruptMode(interruptMask) \
                                                                HPI_IF_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define HPI_IF_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                HPI_IF_CLEAR_INTR_SPI_EC(interruptMask)
    #define HPI_IF_GetExtSpiClkInterruptSource()                 (HPI_IF_INTR_SPI_EC_REG)
    #define HPI_IF_GetExtSpiClkInterruptMode()                   (HPI_IF_INTR_SPI_EC_MASK_REG)
    #define HPI_IF_GetExtSpiClkInterruptSourceMasked()           (HPI_IF_INTR_SPI_EC_MASKED_REG)
#endif /* (!HPI_IF_CY_SCBIP_V1) */

#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void HPI_IF_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (HPI_IF_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_HPI_IF_CUSTOM_INTR_HANDLER)
    extern cyisraddress HPI_IF_customIntrHandler;
#endif /* !defined (CY_REMOVE_HPI_IF_CUSTOM_INTR_HANDLER) */
#endif /* (HPI_IF_SCB_IRQ_INTERNAL) */

extern HPI_IF_BACKUP_STRUCT HPI_IF_backup;

#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 HPI_IF_scbMode;
    extern uint8 HPI_IF_scbEnableWake;
    extern uint8 HPI_IF_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 HPI_IF_mode;
    extern uint8 HPI_IF_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * HPI_IF_rxBuffer;
    extern uint8   HPI_IF_rxDataBits;
    extern uint32  HPI_IF_rxBufferSize;

    extern volatile uint8 * HPI_IF_txBuffer;
    extern uint8   HPI_IF_txDataBits;
    extern uint32  HPI_IF_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 HPI_IF_numberOfAddr;
    extern uint8 HPI_IF_subAddrSize;
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (HPI_IF_SCB_MODE_I2C_CONST_CFG || \
        HPI_IF_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 HPI_IF_IntrTxMask;
#endif /* (! (HPI_IF_SCB_MODE_I2C_CONST_CFG || \
              HPI_IF_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define HPI_IF_SCB_MODE_I2C_RUNTM_CFG     (HPI_IF_SCB_MODE_I2C      == HPI_IF_scbMode)
    #define HPI_IF_SCB_MODE_SPI_RUNTM_CFG     (HPI_IF_SCB_MODE_SPI      == HPI_IF_scbMode)
    #define HPI_IF_SCB_MODE_UART_RUNTM_CFG    (HPI_IF_SCB_MODE_UART     == HPI_IF_scbMode)
    #define HPI_IF_SCB_MODE_EZI2C_RUNTM_CFG   (HPI_IF_SCB_MODE_EZI2C    == HPI_IF_scbMode)
    #define HPI_IF_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (HPI_IF_SCB_MODE_UNCONFIG == HPI_IF_scbMode)

    /* Defines wakeup enable */
    #define HPI_IF_SCB_WAKE_ENABLE_CHECK       (0u != HPI_IF_scbEnableWake)
#endif /* (HPI_IF_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!HPI_IF_CY_SCBIP_V1)
    #define HPI_IF_SCB_PINS_NUMBER    (7u)
#else
    #define HPI_IF_SCB_PINS_NUMBER    (2u)
#endif /* (!HPI_IF_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_HPI_IF_H) */


/* [] END OF FILE */
