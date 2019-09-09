/**
 * @file psink.h
 *
 * @brief @{Power Sink (Consumer) manager header file.@}
 */

/*
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#ifndef _PSINK_H_
#define _PSINK_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>    

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief This function configures the ADC block as a comparator
 *
 * @param port Port index the function is performed for.
 * @param volt_mV Comparator voltage level in mV
 *
 * @return None
 */
void psnk_set_voltage (uint8_t port, uint16_t volt_mV);
/**
 * @brief This is empty function
 *
 * @param port Port index the function is performed for.
 * @param cur_10mA Current value in 10mA
 *
 * @return None
 */
void psnk_set_current (uint8_t port, uint16_t cur_10mA);
/**
 * @brief Set VBus FET On if device policy manager is enabled
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void psnk_enable (uint8_t port);

/**
 * @brief Set VBus FET Off 
 *
 * @param port Port index the function is performed for.
 * @param snk_discharge_off_handler Sink Discharge fet off callback pointer
 * @return None
 */
void psnk_disable (uint8_t port, sink_discharge_off_cbk_t snk_discharge_off_handler);

#endif /* _PSINK_H_ */
/* End of File */

