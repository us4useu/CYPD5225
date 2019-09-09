/**
 * @file pdo.h
 *
 * @brief @{PDO evaluation and handler definitions.@}
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

#ifndef _PDO_H_
#define _PDO_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <pd.h>    

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
/**
 * @brief This function can be used to ask EC to evaluate a src cap message
 * For now evaluating here and executing the callback in this function itself
 * This function can be changed as per customer design.
 *
 * @param port Port index the function is performed for.
 * @param src_cap Pointer to pd packet wich contais source capability.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_src_cap(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler) ;

/**
 * @brief This function can be used to ask EC to evaluate a request message
 * For now evaluating here and executing the callback in this function itself
 * This function can be changed as per customer design
 *
 * @param port Port index the function is performed for.
 * @param rdo Pointer to pd packet wich contais request data object.
 * @param app_resp_handler Application handler callback function.
 *
 * @return None
 */
void eval_rdo(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler) ;

#endif /* _PDO_H_ */

/* End of File */

