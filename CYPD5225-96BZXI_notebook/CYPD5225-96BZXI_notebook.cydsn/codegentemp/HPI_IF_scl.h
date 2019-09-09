/*******************************************************************************
* File Name: HPI_IF_scl.h  
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

#if !defined(CY_PINS_HPI_IF_scl_H) /* Pins HPI_IF_scl_H */
#define CY_PINS_HPI_IF_scl_H

#include "cytypes.h"
#include "cyfitter.h"
#include "HPI_IF_scl_aliases.h"


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
} HPI_IF_scl_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   HPI_IF_scl_Read(void);
void    HPI_IF_scl_Write(uint8 value);
uint8   HPI_IF_scl_ReadDataReg(void);
#if defined(HPI_IF_scl__PC) || (CY_PSOC4_4200L) 
    void    HPI_IF_scl_SetDriveMode(uint8 mode);
#endif
void    HPI_IF_scl_SetInterruptMode(uint16 position, uint16 mode);
uint8   HPI_IF_scl_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void HPI_IF_scl_Sleep(void); 
void HPI_IF_scl_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(HPI_IF_scl__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define HPI_IF_scl_DRIVE_MODE_BITS        (3)
    #define HPI_IF_scl_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - HPI_IF_scl_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the HPI_IF_scl_SetDriveMode() function.
         *  @{
         */
        #define HPI_IF_scl_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define HPI_IF_scl_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define HPI_IF_scl_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define HPI_IF_scl_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define HPI_IF_scl_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define HPI_IF_scl_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define HPI_IF_scl_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define HPI_IF_scl_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define HPI_IF_scl_MASK               HPI_IF_scl__MASK
#define HPI_IF_scl_SHIFT              HPI_IF_scl__SHIFT
#define HPI_IF_scl_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in HPI_IF_scl_SetInterruptMode() function.
     *  @{
     */
        #define HPI_IF_scl_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define HPI_IF_scl_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define HPI_IF_scl_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define HPI_IF_scl_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(HPI_IF_scl__SIO)
    #define HPI_IF_scl_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(HPI_IF_scl__PC) && (CY_PSOC4_4200L)
    #define HPI_IF_scl_USBIO_ENABLE               ((uint32)0x80000000u)
    #define HPI_IF_scl_USBIO_DISABLE              ((uint32)(~HPI_IF_scl_USBIO_ENABLE))
    #define HPI_IF_scl_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define HPI_IF_scl_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define HPI_IF_scl_USBIO_ENTER_SLEEP          ((uint32)((1u << HPI_IF_scl_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << HPI_IF_scl_USBIO_SUSPEND_DEL_SHIFT)))
    #define HPI_IF_scl_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << HPI_IF_scl_USBIO_SUSPEND_SHIFT)))
    #define HPI_IF_scl_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << HPI_IF_scl_USBIO_SUSPEND_DEL_SHIFT)))
    #define HPI_IF_scl_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(HPI_IF_scl__PC)
    /* Port Configuration */
    #define HPI_IF_scl_PC                 (* (reg32 *) HPI_IF_scl__PC)
#endif
/* Pin State */
#define HPI_IF_scl_PS                     (* (reg32 *) HPI_IF_scl__PS)
/* Data Register */
#define HPI_IF_scl_DR                     (* (reg32 *) HPI_IF_scl__DR)
/* Input Buffer Disable Override */
#define HPI_IF_scl_INP_DIS                (* (reg32 *) HPI_IF_scl__PC2)

/* Interrupt configuration Registers */
#define HPI_IF_scl_INTCFG                 (* (reg32 *) HPI_IF_scl__INTCFG)
#define HPI_IF_scl_INTSTAT                (* (reg32 *) HPI_IF_scl__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define HPI_IF_scl_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(HPI_IF_scl__SIO)
    #define HPI_IF_scl_SIO_REG            (* (reg32 *) HPI_IF_scl__SIO)
#endif /* (HPI_IF_scl__SIO_CFG) */

/* USBIO registers */
#if !defined(HPI_IF_scl__PC) && (CY_PSOC4_4200L)
    #define HPI_IF_scl_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define HPI_IF_scl_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define HPI_IF_scl_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define HPI_IF_scl_DRIVE_MODE_SHIFT       (0x00u)
#define HPI_IF_scl_DRIVE_MODE_MASK        (0x07u << HPI_IF_scl_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins HPI_IF_scl_H */


/* [] END OF FILE */
