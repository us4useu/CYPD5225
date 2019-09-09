/*******************************************************************************
* File Name: FW_LED.h  
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

#if !defined(CY_PINS_FW_LED_H) /* Pins FW_LED_H */
#define CY_PINS_FW_LED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "FW_LED_aliases.h"


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
} FW_LED_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   FW_LED_Read(void);
void    FW_LED_Write(uint8 value);
uint8   FW_LED_ReadDataReg(void);
#if defined(FW_LED__PC) || (CY_PSOC4_4200L) 
    void    FW_LED_SetDriveMode(uint8 mode);
#endif
void    FW_LED_SetInterruptMode(uint16 position, uint16 mode);
uint8   FW_LED_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void FW_LED_Sleep(void); 
void FW_LED_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(FW_LED__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define FW_LED_DRIVE_MODE_BITS        (3)
    #define FW_LED_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - FW_LED_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the FW_LED_SetDriveMode() function.
         *  @{
         */
        #define FW_LED_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define FW_LED_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define FW_LED_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define FW_LED_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define FW_LED_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define FW_LED_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define FW_LED_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define FW_LED_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define FW_LED_MASK               FW_LED__MASK
#define FW_LED_SHIFT              FW_LED__SHIFT
#define FW_LED_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in FW_LED_SetInterruptMode() function.
     *  @{
     */
        #define FW_LED_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define FW_LED_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define FW_LED_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define FW_LED_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(FW_LED__SIO)
    #define FW_LED_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(FW_LED__PC) && (CY_PSOC4_4200L)
    #define FW_LED_USBIO_ENABLE               ((uint32)0x80000000u)
    #define FW_LED_USBIO_DISABLE              ((uint32)(~FW_LED_USBIO_ENABLE))
    #define FW_LED_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define FW_LED_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define FW_LED_USBIO_ENTER_SLEEP          ((uint32)((1u << FW_LED_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << FW_LED_USBIO_SUSPEND_DEL_SHIFT)))
    #define FW_LED_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << FW_LED_USBIO_SUSPEND_SHIFT)))
    #define FW_LED_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << FW_LED_USBIO_SUSPEND_DEL_SHIFT)))
    #define FW_LED_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(FW_LED__PC)
    /* Port Configuration */
    #define FW_LED_PC                 (* (reg32 *) FW_LED__PC)
#endif
/* Pin State */
#define FW_LED_PS                     (* (reg32 *) FW_LED__PS)
/* Data Register */
#define FW_LED_DR                     (* (reg32 *) FW_LED__DR)
/* Input Buffer Disable Override */
#define FW_LED_INP_DIS                (* (reg32 *) FW_LED__PC2)

/* Interrupt configuration Registers */
#define FW_LED_INTCFG                 (* (reg32 *) FW_LED__INTCFG)
#define FW_LED_INTSTAT                (* (reg32 *) FW_LED__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define FW_LED_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(FW_LED__SIO)
    #define FW_LED_SIO_REG            (* (reg32 *) FW_LED__SIO)
#endif /* (FW_LED__SIO_CFG) */

/* USBIO registers */
#if !defined(FW_LED__PC) && (CY_PSOC4_4200L)
    #define FW_LED_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define FW_LED_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define FW_LED_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define FW_LED_DRIVE_MODE_SHIFT       (0x00u)
#define FW_LED_DRIVE_MODE_MASK        (0x07u << FW_LED_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins FW_LED_H */


/* [] END OF FILE */
