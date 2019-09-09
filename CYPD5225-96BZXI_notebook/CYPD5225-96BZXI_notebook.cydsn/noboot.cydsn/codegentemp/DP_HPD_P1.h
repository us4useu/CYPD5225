/*******************************************************************************
* File Name: DP_HPD_P1.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_DP_HPD_P1_H) /* Pins DP_HPD_P1_H */
#define CY_PINS_DP_HPD_P1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "DP_HPD_P1_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} DP_HPD_P1_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   DP_HPD_P1_Read(void);
void    DP_HPD_P1_Write(uint8 value);
uint8   DP_HPD_P1_ReadDataReg(void);
#if defined(DP_HPD_P1__PC) || (CY_PSOC4_4200L) 
    void    DP_HPD_P1_SetDriveMode(uint8 mode);
#endif
void    DP_HPD_P1_SetInterruptMode(uint16 position, uint16 mode);
uint8   DP_HPD_P1_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void DP_HPD_P1_Sleep(void); 
void DP_HPD_P1_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(DP_HPD_P1__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define DP_HPD_P1_DRIVE_MODE_BITS        (3)
    #define DP_HPD_P1_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - DP_HPD_P1_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the DP_HPD_P1_SetDriveMode() function.
         *  @{
         */
        #define DP_HPD_P1_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define DP_HPD_P1_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define DP_HPD_P1_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define DP_HPD_P1_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define DP_HPD_P1_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define DP_HPD_P1_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define DP_HPD_P1_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define DP_HPD_P1_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define DP_HPD_P1_MASK               DP_HPD_P1__MASK
#define DP_HPD_P1_SHIFT              DP_HPD_P1__SHIFT
#define DP_HPD_P1_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DP_HPD_P1_SetInterruptMode() function.
     *  @{
     */
        #define DP_HPD_P1_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define DP_HPD_P1_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define DP_HPD_P1_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define DP_HPD_P1_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(DP_HPD_P1__SIO)
    #define DP_HPD_P1_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(DP_HPD_P1__PC) && (CY_PSOC4_4200L)
    #define DP_HPD_P1_USBIO_ENABLE               ((uint32)0x80000000u)
    #define DP_HPD_P1_USBIO_DISABLE              ((uint32)(~DP_HPD_P1_USBIO_ENABLE))
    #define DP_HPD_P1_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define DP_HPD_P1_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define DP_HPD_P1_USBIO_ENTER_SLEEP          ((uint32)((1u << DP_HPD_P1_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << DP_HPD_P1_USBIO_SUSPEND_DEL_SHIFT)))
    #define DP_HPD_P1_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << DP_HPD_P1_USBIO_SUSPEND_SHIFT)))
    #define DP_HPD_P1_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << DP_HPD_P1_USBIO_SUSPEND_DEL_SHIFT)))
    #define DP_HPD_P1_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(DP_HPD_P1__PC)
    /* Port Configuration */
    #define DP_HPD_P1_PC                 (* (reg32 *) DP_HPD_P1__PC)
#endif
/* Pin State */
#define DP_HPD_P1_PS                     (* (reg32 *) DP_HPD_P1__PS)
/* Data Register */
#define DP_HPD_P1_DR                     (* (reg32 *) DP_HPD_P1__DR)
/* Input Buffer Disable Override */
#define DP_HPD_P1_INP_DIS                (* (reg32 *) DP_HPD_P1__PC2)

/* Interrupt configuration Registers */
#define DP_HPD_P1_INTCFG                 (* (reg32 *) DP_HPD_P1__INTCFG)
#define DP_HPD_P1_INTSTAT                (* (reg32 *) DP_HPD_P1__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define DP_HPD_P1_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(DP_HPD_P1__SIO)
    #define DP_HPD_P1_SIO_REG            (* (reg32 *) DP_HPD_P1__SIO)
#endif /* (DP_HPD_P1__SIO_CFG) */

/* USBIO registers */
#if !defined(DP_HPD_P1__PC) && (CY_PSOC4_4200L)
    #define DP_HPD_P1_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define DP_HPD_P1_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define DP_HPD_P1_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define DP_HPD_P1_DRIVE_MODE_SHIFT       (0x00u)
#define DP_HPD_P1_DRIVE_MODE_MASK        (0x07u << DP_HPD_P1_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins DP_HPD_P1_H */


/* [] END OF FILE */
