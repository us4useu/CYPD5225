/**
 * @file app_version.h
 *
 * @brief @{Version definition for the CCG firmware application. This version
 * information follows a Cypress defined format that identifies the type
 * of application along with the version information.@}
 *
 *******************************************************************************
 *
 * Copyright (2016-2018), Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef _APP_VERSION_H_
#define _APP_VERSION_H_
    
#include <project.h>

/******************************************************************************
 * Constant definitions.
 *****************************************************************************/

#define APP_TYPE_NOTEBOOK               (0x6e62)        /* "nb" */

#define APP_TYPE_STRING                 (APP_TYPE_NOTEBOOK)
#define APP_EXT_CIR_NUM                 (0x02)
#define APP_MAJOR_VERSION               (0x00)
#define APP_MINOR_VERSION               (0x00)

/*
   The version format is as follows:
   Bytes 1:0 identify the type of CCG application. 0x6e62 specifies that this is a notebook port controller.
   Byte    2 identifies the hardware design version.
   Byte    3 identifies the major and minor version of the firmware.
 */
#define APP_VERSION                                             \
    ((APP_TYPE_STRING) | (APP_EXT_CIR_NUM << 16) |              \
     (APP_MINOR_VERSION << 24) | (APP_MAJOR_VERSION << 28))

#endif /* _APP_VERSION_H_ */

/* [] END OF FILE */

