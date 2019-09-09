/*******************************************************************************
* File Name: HPI_IF_SCBCLK.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_HPI_IF_SCBCLK_H)
#define CY_CLOCK_HPI_IF_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void HPI_IF_SCBCLK_StartEx(uint32 alignClkDiv);
#define HPI_IF_SCBCLK_Start() \
    HPI_IF_SCBCLK_StartEx(HPI_IF_SCBCLK__PA_DIV_ID)

#else

void HPI_IF_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void HPI_IF_SCBCLK_Stop(void);

void HPI_IF_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 HPI_IF_SCBCLK_GetDividerRegister(void);
uint8  HPI_IF_SCBCLK_GetFractionalDividerRegister(void);

#define HPI_IF_SCBCLK_Enable()                         HPI_IF_SCBCLK_Start()
#define HPI_IF_SCBCLK_Disable()                        HPI_IF_SCBCLK_Stop()
#define HPI_IF_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    HPI_IF_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define HPI_IF_SCBCLK_SetDivider(clkDivider)           HPI_IF_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define HPI_IF_SCBCLK_SetDividerValue(clkDivider)      HPI_IF_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define HPI_IF_SCBCLK_DIV_ID     HPI_IF_SCBCLK__DIV_ID

#define HPI_IF_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define HPI_IF_SCBCLK_CTRL_REG   (*(reg32 *)HPI_IF_SCBCLK__CTRL_REGISTER)
#define HPI_IF_SCBCLK_DIV_REG    (*(reg32 *)HPI_IF_SCBCLK__DIV_REGISTER)

#define HPI_IF_SCBCLK_CMD_DIV_SHIFT          (0u)
#define HPI_IF_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define HPI_IF_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define HPI_IF_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define HPI_IF_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << HPI_IF_SCBCLK_CMD_DISABLE_SHIFT))
#define HPI_IF_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << HPI_IF_SCBCLK_CMD_ENABLE_SHIFT))

#define HPI_IF_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define HPI_IF_SCBCLK_DIV_FRAC_SHIFT (3u)
#define HPI_IF_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define HPI_IF_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define HPI_IF_SCBCLK_DIV_REG        (*(reg32 *)HPI_IF_SCBCLK__REGISTER)
#define HPI_IF_SCBCLK_ENABLE_REG     HPI_IF_SCBCLK_DIV_REG
#define HPI_IF_SCBCLK_DIV_FRAC_MASK  HPI_IF_SCBCLK__FRAC_MASK
#define HPI_IF_SCBCLK_DIV_FRAC_SHIFT (16u)
#define HPI_IF_SCBCLK_DIV_INT_MASK   HPI_IF_SCBCLK__DIVIDER_MASK
#define HPI_IF_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_HPI_IF_SCBCLK_H) */

/* [] END OF FILE */
