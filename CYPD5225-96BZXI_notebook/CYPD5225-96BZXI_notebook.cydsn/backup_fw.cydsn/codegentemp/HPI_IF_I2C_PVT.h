/***************************************************************************//**
* \file .h
* \version 4.0
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component in I2C mode.
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

#if !defined(CY_SCB_I2C_PVT_HPI_IF_H)
#define CY_SCB_I2C_PVT_HPI_IF_H

#include "HPI_IF_I2C.h"


/***************************************
*     Private Global Vars
***************************************/

extern volatile uint8 HPI_IF_state; /* Current state of I2C FSM */

#if(HPI_IF_I2C_SLAVE_CONST)
    extern volatile uint8 HPI_IF_slStatus;          /* Slave Status */

    /* Receive buffer variables */
    extern volatile uint8 * HPI_IF_slWrBufPtr;      /* Pointer to Receive buffer  */
    extern volatile uint32  HPI_IF_slWrBufSize;     /* Slave Receive buffer size  */
    extern volatile uint32  HPI_IF_slWrBufIndex;    /* Slave Receive buffer Index */

    /* Transmit buffer variables */
    extern volatile uint8 * HPI_IF_slRdBufPtr;      /* Pointer to Transmit buffer  */
    extern volatile uint32  HPI_IF_slRdBufSize;     /* Slave Transmit buffer size  */
    extern volatile uint32  HPI_IF_slRdBufIndex;    /* Slave Transmit buffer Index */
    extern volatile uint32  HPI_IF_slRdBufIndexTmp; /* Slave Transmit buffer Index Tmp */
    extern volatile uint8   HPI_IF_slOverFlowCount; /* Slave Transmit Overflow counter */
#endif /* (HPI_IF_I2C_SLAVE_CONST) */

#if(HPI_IF_I2C_MASTER_CONST)
    extern volatile uint16 HPI_IF_mstrStatus;      /* Master Status byte  */
    extern volatile uint8  HPI_IF_mstrControl;     /* Master Control byte */

    /* Receive buffer variables */
    extern volatile uint8 * HPI_IF_mstrRdBufPtr;   /* Pointer to Master Read buffer */
    extern volatile uint32  HPI_IF_mstrRdBufSize;  /* Master Read buffer size       */
    extern volatile uint32  HPI_IF_mstrRdBufIndex; /* Master Read buffer Index      */

    /* Transmit buffer variables */
    extern volatile uint8 * HPI_IF_mstrWrBufPtr;   /* Pointer to Master Write buffer */
    extern volatile uint32  HPI_IF_mstrWrBufSize;  /* Master Write buffer size       */
    extern volatile uint32  HPI_IF_mstrWrBufIndex; /* Master Write buffer Index      */
    extern volatile uint32  HPI_IF_mstrWrBufIndexTmp; /* Master Write buffer Index Tmp */
#endif /* (HPI_IF_I2C_MASTER_CONST) */

#if (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
    extern uint32 (*HPI_IF_customAddressHandler) (void);
#endif /* (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */

/***************************************
*     Private Function Prototypes
***************************************/

#if(HPI_IF_SCB_MODE_I2C_CONST_CFG)
    void HPI_IF_I2CInit(void);
#endif /* (HPI_IF_SCB_MODE_I2C_CONST_CFG) */

void HPI_IF_I2CStop(void);
void HPI_IF_I2CFwBlockReset(void);

void HPI_IF_I2CSaveConfig(void);
void HPI_IF_I2CRestoreConfig(void);

#if(HPI_IF_I2C_MASTER_CONST)
    void HPI_IF_I2CReStartGeneration(void);
#endif /* (HPI_IF_I2C_MASTER_CONST) */

#endif /* (CY_SCB_I2C_PVT_HPI_IF_H) */


/* [] END OF FILE */
