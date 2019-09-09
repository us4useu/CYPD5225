/*******************************************************************************
* File Name: FW_LED.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_FW_LED_ALIASES_H) /* Pins FW_LED_ALIASES_H */
#define CY_PINS_FW_LED_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define FW_LED_0			(FW_LED__0__PC)
#define FW_LED_0_PS		(FW_LED__0__PS)
#define FW_LED_0_PC		(FW_LED__0__PC)
#define FW_LED_0_DR		(FW_LED__0__DR)
#define FW_LED_0_SHIFT	(FW_LED__0__SHIFT)
#define FW_LED_0_INTR	((uint16)((uint16)0x0003u << (FW_LED__0__SHIFT*2u)))

#define FW_LED_INTR_ALL	 ((uint16)(FW_LED_0_INTR))


#endif /* End Pins FW_LED_ALIASES_H */


/* [] END OF FILE */
