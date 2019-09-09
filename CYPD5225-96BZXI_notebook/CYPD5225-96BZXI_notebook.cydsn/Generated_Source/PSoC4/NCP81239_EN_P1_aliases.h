/*******************************************************************************
* File Name: NCP81239_EN_P1.h  
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

#if !defined(CY_PINS_NCP81239_EN_P1_ALIASES_H) /* Pins NCP81239_EN_P1_ALIASES_H */
#define CY_PINS_NCP81239_EN_P1_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define NCP81239_EN_P1_0			(NCP81239_EN_P1__0__PC)
#define NCP81239_EN_P1_0_PS		(NCP81239_EN_P1__0__PS)
#define NCP81239_EN_P1_0_PC		(NCP81239_EN_P1__0__PC)
#define NCP81239_EN_P1_0_DR		(NCP81239_EN_P1__0__DR)
#define NCP81239_EN_P1_0_SHIFT	(NCP81239_EN_P1__0__SHIFT)
#define NCP81239_EN_P1_0_INTR	((uint16)((uint16)0x0003u << (NCP81239_EN_P1__0__SHIFT*2u)))

#define NCP81239_EN_P1_INTR_ALL	 ((uint16)(NCP81239_EN_P1_0_INTR))


#endif /* End Pins NCP81239_EN_P1_ALIASES_H */


/* [] END OF FILE */
