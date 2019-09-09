/**
 * @file boot.h
 *
 * @brief @{Bootloader support header file.@}
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

#ifndef _BOOT_H_
#define _BOOT_H_

#include "config.h"
#include "status.h"
#include "system.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/**
 *  @brief Mask for Image-1 FW status bit in Boot mode reason byte.
 */
#define CY_PD_IMG1_FW_STATUS_BIT_MASK                   (0x08)

/**
 *  @brief Signature used for firmware to indicate boot mode request.
 */
#define CCG_BOOT_MODE_RQT_SIG                           (0x424C)

/**
 *  @brief Signature used to indicate boot FW1 request.
 */
#define CCG_FW1_BOOT_RQT_SIG                            (0x4231)

/**
 *  @brief Signature used to indicate boot FW2 request.
 */
#define CCG_FW2_BOOT_RQT_SIG                            (0x4232)

/**
 *  @brief Firmware boot sequence number offset.
 */
#define CCG_FW_METADATA_BOOTSEQ_OFFSET                  (0x1C)

/**
 *  @brief Default boot-wait window for CCGx boot-loader: 50 ms
 */
#define CCG_BL_WAIT_DEFAULT                             (50)

/**
 *  @brief Minimum boot-wait window duration supported: 20 ms
 */
#define CCG_BL_WAIT_MINUMUM                             (20)

/**
 *  @brief Maximum boot-wait window duration supported: 1000 ms
 */
#define CCG_BL_WAIT_MAXIMUM                             (1000)

/**
 *  @brief FW metadata application ID value requesting default boot-wait window.
 */
#define CCG_FWMETA_APPID_WAIT_DEF                       (0xFFFFu)

/**
 *  @brief FW metadata application ID value requesting a zero boot-wait window.
 */
#define CCG_FWMETA_APPID_WAIT_0                         (0x4359)

/**
 * @brief Configuration table parameters used for validation.
 */
#define CONFIGTABLE_SIGNATURE                           (0x4359)
#define CONFIGTABLE_SIZE_OFFSET                         (6)
#define CONFIGTABLE_CHECKSUM_OFFSET                     (8)
#define CONFIGTABLE_CHECKSUM_START                      (10)

/*****************************************************************************
* Data Struct Definition
*****************************************************************************/

/**
 * @brief Boot mode reason structure.
 *
 * This structure holds status of FW images and boot mode request.
 * If the CCGx device is running in Boot-loader mode, this register can be used
 * to identify the reason for this. The register will report the validity of
 * FW1 and FW2 binaries even in the case where the device is already running
 * in FW1 or FW2 mode.
 */
typedef union
{
    uint8_t val;                            /**< Integer field used for direct manipulation of reason code. */

    struct fw_mode_reason_t
    {
        uint8_t boot_mode_request : 1;      /**< Boot mode request made by FW. */
        uint8_t reserved          : 1;      /**< Reserved field: Will be zero. */
        uint8_t fw1_invalid       : 1;      /**< FW1 image invalid: 0=Valid, 1=Invalid. */
        uint8_t fw2_invalid       : 1;      /**< FW2 image invalid: 0=Valid, 1=Invalid. */
        uint8_t reserved1         : 4;      /**< Reserved for later use. */
    } status;                               /**< Union containing the status fields in the boot mode reason value. */

} fw_img_status_t;

/**
 * @brief CCGx Firmware metadata structure
 *
 * This structure defines the format of the firmware metadata that is stored
 * on device flash. The boot-loader uses the metadata to identify the firmware validity.
 * location, size, start address etc. The metadata for the two runtime firmware images
 * (FW1 and FW2) are located at fixed addresses (for each CCGx part), allowing the boot-loader
 * to precisely locate and validate the flash content during boot-up.
 */
typedef struct __attribute__((__packed__))
{
    uint8_t fw_checksum;                  /**< 00: Single Byte FW Checksum. */
    uint32_t fw_entry;                    /**< 01: FW Entry Address */
    uint16_t boot_last_row;               /**< 05: Last Flash row of Bootloader or previous firmware. */
    uint8_t reserved1[2];                 /**< 07: Reserved. */
    uint32_t fw_size;                     /**< 09: Size of Firmware. */
    uint8_t reserved2[3];                 /**< 0D: Reserved. */
    uint8_t active_boot_app;              /**< 10: Creator specific field. Not used in this implementation. */
    uint8_t boot_app_ver_status;          /**< 11: Creator specific field. Not used in this implementation. */
    uint16_t boot_app_version;            /**< 12: Creator specific field. Not used in this implementation. */
    uint16_t boot_app_id;                 /**< 14: Creator specific field. Not used in this implementation. */
    uint16_t metadata_valid;              /**< 16: Metadata Valid field. Valid if contains "CY". */
    uint32_t fw_version;                  /**< 18: Creator specific field. Not used in this implementation. */
    uint32_t boot_seq;                    /**< 1C: Boot sequence number field. Boot-loader will load the valid FW copy
                                                   that has the higher sequence number associated with it. */

} sys_fw_metadata_t;

/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/

/**
 *  @brief Pointer to metadata associated with the Image-1 FW binary.
 */
extern sys_fw_metadata_t *gl_img1_fw_metadata;

#if (!CCG_DUALAPP_DISABLE)
/**
 *  @brief Pointer to metadata associated with the Image-2 FW binary.
 */
extern sys_fw_metadata_t *gl_img2_fw_metadata;
#endif /* (!CCG_DUALAPP_DISABLE) */

#if (CCG_PSEUDO_METADATA_DISABLE == 0)

/**
 *  @brief Pointer to pseudo metadata associated with the Image-1 FW binary.
 */
extern sys_fw_metadata_t *gl_img1_fw_pseudo_metadata;

/**
 *  @brief Pointer to pseudo metadata associated with the Image-2 FW binary.
 */
extern sys_fw_metadata_t *gl_img2_fw_pseudo_metadata;

#endif /* CCG_PSEUDO_METADATA_DISABLE */

/*****************************************************************************
* Global Function Declaration
*****************************************************************************/

/**
 * @brief Validate the configuration table specified.
 *
 * Each copy of CCGx firmware on the device flash contains an embedded
 * configuration table that defines the runtime behaviour of the CCGx device. This
 * function checks whether the configuration table located at the specified location
 * is valid (has valid signature and checksum).
 *
 * @param table_p Pointer to the configuration table to be validated.
 *
 * @return CCG_STAT_SUCCESS if the table is valid, CCG_STAT_FAILURE otherwise.
 */
ccg_status_t boot_validate_configtable(uint8_t *table_p);

/**
 * @brief Validate the firmware image associated with the given metadata.
 *
 * This function validates the firmware binary associated with the
 * metadata specified in the fw_metadata parameter. The validity check includes
 * checks for signature, location, size and checksum. This function internally
 * performs validation of the embedded configuration table using the
 * boot_validate_configtable function.
 *
 * @param fw_metadata Pointer to metadata table of the FW which has to be validated.

 * @return CCG_STAT_SUCCESS if the firmware is valid, CCG_STAT_FAILURE otherwise.
 */
ccg_status_t boot_validate_fw(sys_fw_metadata_t *fw_metadata);

/**
 * @brief Handles the VALIDATE_FW command from HPI or UVDM.
 *
 * This API handles the VALIDATE_FW command received through
 * the HPI or UVDM interfaces.
 *
 * @param fw_mode Firmware binary id: 1 for FW1 and 2 for FW2.

 * @return Status code indicating the validity of the firmware.
 */
ccg_status_t boot_handle_validate_fw_cmd(sys_fw_mode_t fw_mode);

/**
 * @brief Returns the boot-wait delay configured for the application.
 *
 * This function identifies the boot-wait delay required by checking
 * the firmware metadata.
 *
 * @return Boot-wait delay in milliseconds.
 */
uint16_t boot_get_wait_time(void);

/**
 * @brief Identify the firmware binary to be loaded.
 *
 * This function is only used in the CCGx boot-loader, and
 * implements the main start-up logic of the boot-loader. The function
 * validates the two firmware binaries in device flash, and identifies
 * the binary to be loaded. If neither binary is valid, the function returns
 * false notifying the caller to continue in boot-loader mode.
 *
 * @return true if firmware load is allowed, false otherwise.
 */
bool boot_start(void);

#if (CCG_PSEUDO_METADATA_DISABLE == 0)

/**
 * @brief Check for the presence of alternate firmware waiting to be validated.
 *
 * This function checks whether the CCGx device flash contains an alternate firmware
 * binary which is waiting to be validated. This can happen if a firmware update happened
 * during the last power up of the device, and the binary is yet to be validated and made
 * active. The active firmware will make use of the pseudo metadata in flash to identify
 * the alternate firmware, validate it and activate it by updating the actual firmware
 * metadata.
 *
 * Please refer to the CCGx boot sequence description in the firmware guide for a more
 * detailed description of the boot procedure.
 *
 * @return NONE
 */
void boot_check_for_valid_fw(void);

#endif /* CCG_PSEUDO_METADATA_DISABLE */

/**
 * @brief Returns a bitmap containing the reason for boot mode.
 *
 * This function returns the bitmap value that is to be stored in the
 * BOOT_MODE_REASON HPI register, which identifies the validity of the
 * two firmware binaries. The validation of the firmware is expected to
 * have been completed earlier through the boot_start function. This
 * function only retrieves the status stored during the validation procedure.
 *
 * @see fw_img_status_t
 *
 * @return Boot mode reason bitmap.
 */
fw_img_status_t get_boot_mode_reason(void);

/**
 * @brief Transfer control to the firmware binary identified by boot_start.
 *
 * This function is only used by the CCGx boot-loader. This transfers control to
 * the firmware binary selected as the boot target by the boot_start function.
 * This is expected to be called after the boot-wait window has elapsed.
 *
 * @return None
 */
void boot_jump_to_fw(void);

/**
 * @brief Get the boot sequence number value for the specified firmware image.
 *
 * A boot sequence number field stored in the firmware metadata is used by the
 * CCGx boot-loader to identify the firmware binary to be loaded. This function
 * retrieves the sequence number associated with the specified firmware binary.
 *
 * @param fwid Firmware id whose sequence number is to be retrieved. 1 for FW1
 * and 2 for FW2.
 *
 * @return Boot sequence number value if the firmware is valid, 0 otherwise.
 */
uint32_t boot_get_boot_seq(uint8_t fwid);

/**
 * @brief Function to validate firmware images and update image status.
 *
 * This function is used to validate the firmware images in the CCG device flash
 * and update their status in the image status / boot-mode reason field.
 *
 * @return None
 */
void boot_update_fw_status(void);

/* Variable representing firmware image status. */
extern fw_img_status_t gl_img_status;
#endif /* _BOOT_H_ */

/* [] END OF FILE */
