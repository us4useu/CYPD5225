/*******************************************************************************
* File Name: NCP81239_EN_P2.h  
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

#if !defined(CY_PINS_NCP81239_EN_P2_H) /* Pins NCP81239_EN_P2_H */
#define CY_PINS_NCP81239_EN_P2_H

#include "cytypes.h"
#include "cyfitter.h"
#include "NCP81239_EN_P2_aliases.h"


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
} NCP81239_EN_P2_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   NCP81239_EN_P2_Read(void);
void    NCP81239_EN_P2_Write(uint8 value);
uint8   NCP81239_EN_P2_ReadDataReg(void);
#if defined(NCP81239_EN_P2__PC) || (CY_PSOC4_4200L) 
    void    NCP81239_EN_P2_SetDriveMode(uint8 mode);
#endif
void    NCP81239_EN_P2_SetInterruptMode(uint16 position, uint16 mode);
uint8   NCP81239_EN_P2_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void NCP81239_EN_P2_Sleep(void); 
void NCP81239_EN_P2_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(NCP81239_EN_P2__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define NCP81239_EN_P2_DRIVE_MODE_BITS        (3)
    #define NCP81239_EN_P2_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - NCP81239_EN_P2_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the NCP81239_EN_P2_SetDriveMode() function.
         *  @{
         */
        #define NCP81239_EN_P2_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define NCP81239_EN_P2_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define NCP81239_EN_P2_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define NCP81239_EN_P2_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define NCP81239_EN_P2_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define NCP81239_EN_P2_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define NCP81239_EN_P2_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define NCP81239_EN_P2_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define NCP81239_EN_P2_MASK               NCP81239_EN_P2__MASK
#define NCP81239_EN_P2_SHIFT              NCP81239_EN_P2__SHIFT
#define NCP81239_EN_P2_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in NCP81239_EN_P2_SetInterruptMode() function.
     *  @{
     */
        #define NCP81239_EN_P2_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define NCP81239_EN_P2_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define NCP81239_EN_P2_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define NCP81239_EN_P2_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(NCP81239_EN_P2__SIO)
    #define NCP81239_EN_P2_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(NCP81239_EN_P2__PC) && (CY_PSOC4_4200L)
    #define NCP81239_EN_P2_USBIO_ENABLE               ((uint32)0x80000000u)
    #define NCP81239_EN_P2_USBIO_DISABLE              ((uint32)(~NCP81239_EN_P2_USBIO_ENABLE))
    #define NCP81239_EN_P2_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define NCP81239_EN_P2_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define NCP81239_EN_P2_USBIO_ENTER_SLEEP          ((uint32)((1u << NCP81239_EN_P2_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << NCP81239_EN_P2_USBIO_SUSPEND_DEL_SHIFT)))
    #define NCP81239_EN_P2_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << NCP81239_EN_P2_USBIO_SUSPEND_SHIFT)))
    #define NCP81239_EN_P2_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << NCP81239_EN_P2_USBIO_SUSPEND_DEL_SHIFT)))
    #define NCP81239_EN_P2_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(NCP81239_EN_P2__PC)
    /* Port Configuration */
    #define NCP81239_EN_P2_PC                 (* (reg32 *) NCP81239_EN_P2__PC)
#endif
/* Pin State */
#define NCP81239_EN_P2_PS                     (* (reg32 *) NCP81239_EN_P2__PS)
/* Data Register */
#define NCP81239_EN_P2_DR                     (* (reg32 *) NCP81239_EN_P2__DR)
/* Input Buffer Disable Override */
#define NCP81239_EN_P2_INP_DIS                (* (reg32 *) NCP81239_EN_P2__PC2)

/* Interrupt configuration Registers */
#define NCP81239_EN_P2_INTCFG                 (* (reg32 *) NCP81239_EN_P2__INTCFG)
#define NCP81239_EN_P2_INTSTAT                (* (reg32 *) NCP81239_EN_P2__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define NCP81239_EN_P2_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(NCP81239_EN_P2__SIO)
    #define NCP81239_EN_P2_SIO_REG            (* (reg32 *) NCP81239_EN_P2__SIO)
#endif /* (NCP81239_EN_P2__SIO_CFG) */

/* USBIO registers */
#if !defined(NCP81239_EN_P2__PC) && (CY_PSOC4_4200L)
    #define NCP81239_EN_P2_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define NCP81239_EN_P2_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define NCP81239_EN_P2_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define NCP81239_EN_P2_DRIVE_MODE_SHIFT       (0x00u)
#define NCP81239_EN_P2_DRIVE_MODE_MASK        (0x07u << NCP81239_EN_P2_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins NCP81239_EN_P2_H */


/* [] END OF FILE */
