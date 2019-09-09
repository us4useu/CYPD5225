/*******************************************************************************
* File Name: PDSS_PORT0_SAR_CLK.h
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

#if !defined(CY_CLOCK_PDSS_PORT0_SAR_CLK_H)
#define CY_CLOCK_PDSS_PORT0_SAR_CLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void PDSS_PORT0_SAR_CLK_StartEx(uint32 alignClkDiv);
#define PDSS_PORT0_SAR_CLK_Start() \
    PDSS_PORT0_SAR_CLK_StartEx(PDSS_PORT0_SAR_CLK__PA_DIV_ID)

#else

void PDSS_PORT0_SAR_CLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void PDSS_PORT0_SAR_CLK_Stop(void);

void PDSS_PORT0_SAR_CLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 PDSS_PORT0_SAR_CLK_GetDividerRegister(void);
uint8  PDSS_PORT0_SAR_CLK_GetFractionalDividerRegister(void);

#define PDSS_PORT0_SAR_CLK_Enable()                         PDSS_PORT0_SAR_CLK_Start()
#define PDSS_PORT0_SAR_CLK_Disable()                        PDSS_PORT0_SAR_CLK_Stop()
#define PDSS_PORT0_SAR_CLK_SetDividerRegister(clkDivider, reset)  \
    PDSS_PORT0_SAR_CLK_SetFractionalDividerRegister((clkDivider), 0u)
#define PDSS_PORT0_SAR_CLK_SetDivider(clkDivider)           PDSS_PORT0_SAR_CLK_SetDividerRegister((clkDivider), 1u)
#define PDSS_PORT0_SAR_CLK_SetDividerValue(clkDivider)      PDSS_PORT0_SAR_CLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define PDSS_PORT0_SAR_CLK_DIV_ID     PDSS_PORT0_SAR_CLK__DIV_ID

#define PDSS_PORT0_SAR_CLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define PDSS_PORT0_SAR_CLK_CTRL_REG   (*(reg32 *)PDSS_PORT0_SAR_CLK__CTRL_REGISTER)
#define PDSS_PORT0_SAR_CLK_DIV_REG    (*(reg32 *)PDSS_PORT0_SAR_CLK__DIV_REGISTER)

#define PDSS_PORT0_SAR_CLK_CMD_DIV_SHIFT          (0u)
#define PDSS_PORT0_SAR_CLK_CMD_PA_DIV_SHIFT       (8u)
#define PDSS_PORT0_SAR_CLK_CMD_DISABLE_SHIFT      (30u)
#define PDSS_PORT0_SAR_CLK_CMD_ENABLE_SHIFT       (31u)

#define PDSS_PORT0_SAR_CLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << PDSS_PORT0_SAR_CLK_CMD_DISABLE_SHIFT))
#define PDSS_PORT0_SAR_CLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << PDSS_PORT0_SAR_CLK_CMD_ENABLE_SHIFT))

#define PDSS_PORT0_SAR_CLK_DIV_FRAC_MASK  (0x000000F8u)
#define PDSS_PORT0_SAR_CLK_DIV_FRAC_SHIFT (3u)
#define PDSS_PORT0_SAR_CLK_DIV_INT_MASK   (0xFFFFFF00u)
#define PDSS_PORT0_SAR_CLK_DIV_INT_SHIFT  (8u)

#else 

#define PDSS_PORT0_SAR_CLK_DIV_REG        (*(reg32 *)PDSS_PORT0_SAR_CLK__REGISTER)
#define PDSS_PORT0_SAR_CLK_ENABLE_REG     PDSS_PORT0_SAR_CLK_DIV_REG
#define PDSS_PORT0_SAR_CLK_DIV_FRAC_MASK  PDSS_PORT0_SAR_CLK__FRAC_MASK
#define PDSS_PORT0_SAR_CLK_DIV_FRAC_SHIFT (16u)
#define PDSS_PORT0_SAR_CLK_DIV_INT_MASK   PDSS_PORT0_SAR_CLK__DIVIDER_MASK
#define PDSS_PORT0_SAR_CLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_PDSS_PORT0_SAR_CLK_H) */

/* [] END OF FILE */
