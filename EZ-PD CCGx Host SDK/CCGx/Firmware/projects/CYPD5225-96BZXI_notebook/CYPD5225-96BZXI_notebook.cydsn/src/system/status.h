/**
 * @file status.h
 *
 * @brief @{API return status definitions.@}
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

#ifndef _STATUS_H_
#define _STATUS_H_

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/**
 * @brief Value to be added to the status code to get the HPI/UVDM response code.
 *
 * The HPI/UVDM interface response codes use a different convention than the core
 * function return codes. This value represents the offset that should be added to
 * the function return code to get the HPI/UVDM response code.
 */
#define CCG_STATUS_CODE_OFFSET          (2)

/**
 * @brief Convert CCG status code to HPI/UVDM response code.
 */
#define CCG_STATUS_TO_HPI_RESPONSE(c)   ((c) + CCG_STATUS_CODE_OFFSET)

/*****************************************************************************
* Enumerated Data Definition
*****************************************************************************/
/**
 * @brief Interface status codes
 *
 * Enumeration to hold status codes for all CCG interfaces. These values
 * are pre-defined for each interface and should not be modified. To make
 * interface usage easier, the enumeration starts at -2. This allows the
 * success status to have a value of zero. The response code should be 
 * incremented by two before sending out on the individual interfaces.
 */
typedef enum ccg_status
{
    CCG_STAT_NO_RESPONSE = -2,          /**< Special status code indicating
                                             no response. */
    CCG_STAT_SUCCESS = 0,               /**< Success status. */
    CCG_STAT_FLASH_DATA_AVAILABLE,      /**< Special status code indicating flash data
                                             availability. */
    CCG_STAT_BAD_PARAM,                 /**< Bad input parameter. */
    CCG_STAT_INVALID_COMMAND = 3,       /**< Operation failed due to invalid command. */
    CCG_STAT_FLASH_UPDATE_FAILED = 5,   /**< Flash write operation failed. */
    CCG_STAT_INVALID_FW,                /**< Special status code indcating invalid firmware */
    CCG_STAT_INVALID_ARGUMENT,          /**< Operation failed due to invalid arguments. */
    CCG_STAT_NOT_SUPPORTED,             /**< Feature not supported. */
    CCG_STAT_INVALID_SIGNATURE,         /**< Invalid signature parameter identified. */
    CCG_STAT_TRANS_FAILURE,             /**< Transaction failure status. */
    CCG_STAT_CMD_FAILURE,               /**< Command failure status */
    CCG_STAT_FAILURE,                   /**< Generic failure status. */
    CCG_STAT_READ_DATA,                 /**< Special status code indicating read data
                                             availability. */
    CCG_STAT_NOT_READY,                 /**< Operation failed due to device/stack not ready. */
    CCG_STAT_BUSY,                      /**< Operation failed due to device/stack busy status. */
    CCG_STAT_TIMEOUT                    /**< Operation timed out. */
} ccg_status_t;

#endif /* _STATUS_H_ */

/* [] END OF FILE */
