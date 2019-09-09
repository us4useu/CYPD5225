/**
 * @file flash_config.h
 *
 * @brief @{This file defines all of the CCG device flash parameters for use
 * by the firmware stack. The contents of this file are expected to
 * remain unchanged for a given CCG part.@}
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

#ifndef _FLASH_CONFIG_H_
#define _FLASH_CONFIG_H_

#include <project.h>

/******************************************************************************
 * Constant definitions.
 *****************************************************************************/

/** CCGx FLASH OPTIONS **/
/* Only the boot-loader last row and the configuration table size is expected
 * to change to match the project requirement. With a fixed boot-loader, none
 * of this fields should be modified.
 */

/* Total size of flash. */
#define CCG_FLASH_SIZE                          (CY_FLASH_SIZE)

/* Last row number of flash. */
#define CCG_LAST_FLASH_ROW_NUM                  (CY_FLASH_NUMBER_ROWS - 1)

/* Flash row size. This depends on the device type. Using PSOC Creator
 * provided MACRO. */
#define CCG_FLASH_ROW_SIZE                      (CY_FLASH_SIZEOF_ROW)

/* Shift value used for multiplying row number with flash row size to
 * get the actual flash address. This depends on the flash row size. Update this
 * field accordingly. */
#if (CY_FLASH_SIZEOF_ROW == 128)
#define CCG_FLASH_ROW_SHIFT_NUM                 (7u)
#elif (CY_FLASH_SIZEOF_ROW == 256)
#define CCG_FLASH_ROW_SHIFT_NUM                 (8u)
#else
#error "Selected device has unsupported flash row size."
#endif /* CY_FLASH_SIZEOF_ROW */ 

/*FW Metadata Table size in bytes. */
#define CCG_METADATA_TABLE_SIZE                 (0x40)

/* FW Image-1 metadata row number and address. */
#define CCG_IMG1_METADATA_ROW_NUM               (CCG_LAST_FLASH_ROW_NUM)
#define CCG_IMG1_FW_METADATA_ADDR               (((CCG_IMG1_METADATA_ROW_NUM + 1) <<\
                                                 CCG_FLASH_ROW_SHIFT_NUM) - CCG_METADATA_TABLE_SIZE)

/* FW Image-2 metadata row number and address. */
#define CCG_IMG2_METADATA_ROW_NUM               (CCG_LAST_FLASH_ROW_NUM - 1)
#define CCG_IMG2_FW_METADATA_ADDR               (((CCG_IMG2_METADATA_ROW_NUM + 1) <<\
                                                 CCG_FLASH_ROW_SHIFT_NUM) - CCG_METADATA_TABLE_SIZE)

/* FW Image-2 pseudo-metadata row number and address. */
#define CCG_IMG2_PSEUDO_METADATA_ROW_NUM        (CCG_LAST_FLASH_ROW_NUM - 2)
#define CCG_IMG2_FW_PSEUDO_METADATA_ADDR        (((CCG_IMG2_PSEUDO_METADATA_ROW_NUM + 1) <<\
                                                 CCG_FLASH_ROW_SHIFT_NUM) - CCG_METADATA_TABLE_SIZE)

/* Last Boot loader row. This field should be changed only when there is a
 * project level change. A change to this field should be synchronous with
 * the following fields:
 *   -> boot-loader cm0gcc.ld file configSection location:
 *      = ((CY_PD_BOOT_LOADER_LAST_ROW + 1) << CY_PD_FLASH_ROW_SHIFT_NUM)
 *   -> firmware boot-loadable component firmware start location:
 *      = (CY_PD_CONFIG_TABLE_ADDRESS + CY_PD_CONFIG_TABLE_SIZE) */
#define CCG_BOOT_LOADER_LAST_ROW                (0x13)

/* This MACRO defines the last row occupied by FW Image 1. FW Image 1 uses
 * this information to determine the section of flash assigned for FW Image 2. */
#define CCG_IMG1_LAST_FLASH_ROW_NUM             (0xDF)

/* This MACRO defines the last row occupied by FW Image 2. */
#define CCG_IMG2_LAST_FLASH_ROW_NUM             (CCG_IMG2_METADATA_ROW_NUM - 1)

/* Configuration table is located at a fixed offset of 0x100 from start of firmware location. */
#define CCG_FW_CONFTABLE_OFFSET                 (0x100)

/* Config table address boundary used to distinguish between FW1 and FW2. */
#define CCG_FW1_CONFTABLE_MAX_ADDR              (0x8000)

#endif /* _FLASH_CONFIG_H_ */

/* [] END OF FILE */
