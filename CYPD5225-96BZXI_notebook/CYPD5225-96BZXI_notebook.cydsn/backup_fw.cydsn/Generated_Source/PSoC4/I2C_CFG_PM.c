/*******************************************************************************
* File Name: I2C_CFG.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "I2C_CFG.h"

static I2C_CFG_BACKUP_STRUCT  I2C_CFG_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: I2C_CFG_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet I2C_CFG_SUT.c usage_I2C_CFG_Sleep_Wakeup
*******************************************************************************/
void I2C_CFG_Sleep(void)
{
    #if defined(I2C_CFG__PC)
        I2C_CFG_backup.pcState = I2C_CFG_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            I2C_CFG_backup.usbState = I2C_CFG_CR1_REG;
            I2C_CFG_USB_POWER_REG |= I2C_CFG_USBIO_ENTER_SLEEP;
            I2C_CFG_CR1_REG &= I2C_CFG_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(I2C_CFG__SIO)
        I2C_CFG_backup.sioState = I2C_CFG_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        I2C_CFG_SIO_REG &= (uint32)(~I2C_CFG_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: I2C_CFG_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to I2C_CFG_Sleep() for an example usage.
*******************************************************************************/
void I2C_CFG_Wakeup(void)
{
    #if defined(I2C_CFG__PC)
        I2C_CFG_PC = I2C_CFG_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            I2C_CFG_USB_POWER_REG &= I2C_CFG_USBIO_EXIT_SLEEP_PH1;
            I2C_CFG_CR1_REG = I2C_CFG_backup.usbState;
            I2C_CFG_USB_POWER_REG &= I2C_CFG_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(I2C_CFG__SIO)
        I2C_CFG_SIO_REG = I2C_CFG_backup.sioState;
    #endif
}


/* [] END OF FILE */
