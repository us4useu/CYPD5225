/**
 * @file boot.c
 *
 * @brief @{Bootloader support functions.@}
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

#include <stdbool.h>
#include <config.h>
#include <flash_config.h>
#include <app_version.h>
#include <utils.h>
#include <status.h>
#include <system.h>
#include <flash.h>
#include <boot.h>

#if (SECURE_FW_UPDATE == 1)
#include <secure_boot.h>
#include <crypto_hal.h>
#endif /* SECURE_FW_UPDATE */

/* Structure to hold reason for boot mode. */
fw_img_status_t gl_img_status;

#if CCG_DUALAPP_DISABLE
/* If Dual-App bootloading is disabled, provide stub variable to keep the compiler happy. */
volatile uint8_t Bootloader_1_activeApp = 0;
#endif

#if (CCG_PSEUDO_METADATA_DISABLE == 0)

/* Pointer to Image-1 FW pseudo-metadata table. */
sys_fw_metadata_t *gl_img1_fw_pseudo_metadata;

/* Pointer to Image-2 FW pseudo-metadata table. */
sys_fw_metadata_t *gl_img2_fw_pseudo_metadata =
    (sys_fw_metadata_t *)(CCG_IMG2_FW_PSEUDO_METADATA_ADDR);

#endif /* CCG_PSEUDO_METADATA_DISABLE */

/* Boot-wait duration specified by firmware metadata. */
static volatile uint16_t gl_boot_wait_delay = CCG_BL_WAIT_DEFAULT;

/* Pointer to Image-1 FW metadata table. */
sys_fw_metadata_t *gl_img1_fw_metadata = 
        (sys_fw_metadata_t *)(CCG_IMG1_FW_METADATA_ADDR);
        
#if (!CCG_DUALAPP_DISABLE)
/* Pointer to Image-2 FW metadata table. */
sys_fw_metadata_t *gl_img2_fw_metadata = 
        (sys_fw_metadata_t *)(CCG_IMG2_FW_METADATA_ADDR);
#endif /* (!CCG_DUALAPP_DISABLE) */

fw_img_status_t get_boot_mode_reason(void)
{
    /* Return the reason for boot mode. */
    return gl_img_status;
}

/* Check whether configuration table checksum is good. */
ccg_status_t boot_validate_configtable(uint8_t *table_p)
{
    uint16_t size = MAKE_WORD (table_p[CONFIGTABLE_SIZE_OFFSET + 1], table_p[CONFIGTABLE_SIZE_OFFSET]);

    if (((uint32_t)table_p >= CCG_FLASH_SIZE) ||
            (MAKE_WORD (table_p[1], table_p[0]) != CONFIGTABLE_SIGNATURE))
    {
        return CCG_STAT_INVALID_FW;
    }

    if (table_p[CONFIGTABLE_CHECKSUM_OFFSET] != mem_calculate_byte_checksum (
                table_p + CONFIGTABLE_CHECKSUM_START, size - CONFIGTABLE_CHECKSUM_START))
    {
        return CCG_STAT_INVALID_FW;
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t boot_validate_fw(sys_fw_metadata_t *fw_metadata)
{
    ccg_status_t status = CCG_STAT_SUCCESS;
    /* Pointer to FW image start address. */
    uint32_t fw_start = 0;
    /* Size of FW image. */
    uint32_t fw_size = 0;
   
    fw_start = ((fw_metadata->boot_last_row << CCG_FLASH_ROW_SHIFT_NUM)
            + CCG_FLASH_ROW_SIZE);
    fw_size = fw_metadata->fw_size;
    
    /*
     * Validate:
     * 1) FW size.
     * 2) FW checksum.
     * 3) FW entry.
     */
    if (
            (fw_size == 0) ||
            ((fw_start + fw_size) >= CCG_FLASH_SIZE) ||
            (fw_metadata->fw_entry < fw_start) ||
            (fw_metadata->fw_entry >= (fw_start + fw_size)) ||
            (fw_metadata->fw_checksum != mem_calculate_byte_checksum ((uint8_t *)fw_start, fw_size))
       )
    {
        status = CCG_STAT_INVALID_FW;
    }
    else
    {
        status = boot_validate_configtable ((uint8_t *)(fw_start + CCG_FW_CONFTABLE_OFFSET));
    }

    return status;
}

ccg_status_t boot_handle_validate_fw_cmd(sys_fw_mode_t fw_mode)
{
    sys_fw_metadata_t *md_p = NULL;
    ccg_status_t code = CCG_STAT_NO_RESPONSE;

#if (CCG_BOOT != 0)
    switch (fw_mode)
    {
        case SYS_FW_MODE_FWIMAGE_1:
            md_p = gl_img1_fw_metadata;
            break;
#if (!CCG_DUALAPP_DISABLE)
        case SYS_FW_MODE_FWIMAGE_2:
            md_p = gl_img2_fw_metadata;
            break;
#endif /* (!CCG_DUALAPP_DISABLE) */
        default:
            code = CCG_STAT_INVALID_ARGUMENT;
            break;
    }
#else /* (CCG_BOOT != 0) */
    if (fw_mode == sys_get_device_mode ())
    {
        /* There is no need to validate the currently running image. */
        code = CCG_STAT_SUCCESS;
    }
    else
    {
        switch (fw_mode)
        {
            case SYS_FW_MODE_FWIMAGE_1:
#if (CCG_PSEUDO_METADATA_DISABLE == 0)
                /*
                 * CCG is in FW Image 2 and request is to validate FW image 1. 
                 * There can be following cases:
                 * 1) If Image-1 PMD signature is "CP", use Image-1 PMD for validation.
                 * 2) Otherwise, use Image-1 MD for validation.
                 */
                if(gl_img1_fw_pseudo_metadata->metadata_valid == 
                    SYS_PSEUDO_METADATA_VALID_SIG)
                {
                    md_p = gl_img1_fw_pseudo_metadata;
                }
                else
#endif /* (CCG_PSEUDO_METADATA_DISABLE == 0) */
                {
                    md_p = gl_img1_fw_metadata;
                }
                break;
#if (!CCG_DUALAPP_DISABLE)
            case SYS_FW_MODE_FWIMAGE_2:
#if (CCG_PSEUDO_METADATA_DISABLE == 0)
                /*
                 * CCG is in FW Image 1 and request is to validate FW image 2. 
                 * There can be following cases:
                 * 1) If Image-2 PMD signature is "CP", use Image-2 PMD for validation.
                 * 2) Otherwise, use Image-1 MD for validation.
                 */
                if(gl_img2_fw_pseudo_metadata->metadata_valid == 
                    SYS_PSEUDO_METADATA_VALID_SIG)
                {
                    md_p = gl_img2_fw_pseudo_metadata;
                }
                else
#endif /* (CCG_PSEUDO_METADATA_DISABLE == 0) */
                {
                    md_p = gl_img2_fw_metadata;
                }
                break;
#endif /* (!CCG_DUALAPP_DISABLE) */
            default:
                code = CCG_STAT_INVALID_ARGUMENT;
                break;
        }
    }
#endif /* (CCG_BOOT != 0) */

    if (md_p != NULL)
    {
        if (boot_validate_fw (md_p) == CCG_STAT_SUCCESS)
        {
            code = CCG_STAT_SUCCESS;
            if(fw_mode == SYS_FW_MODE_FWIMAGE_1)
            {
                gl_img_status.status.fw1_invalid = 0;
            }
#if (!CCG_DUALAPP_DISABLE)
            else if(fw_mode == SYS_FW_MODE_FWIMAGE_2)
            {
                gl_img_status.status.fw2_invalid = 0;
            }
#endif /* !CCG_DUALAPP_DISABLE */
        }
        else
        {
            code = CCG_STAT_INVALID_FW;
            if(fw_mode == SYS_FW_MODE_FWIMAGE_1)
            {
                gl_img_status.status.fw1_invalid = 1;
            }
#if (!CCG_DUALAPP_DISABLE)
            else if(fw_mode == SYS_FW_MODE_FWIMAGE_2)
            {
                gl_img_status.status.fw2_invalid = 1;
            }
#endif /* !CCG_DUALAPP_DISABLE */
        }
    }
    return code;
}

#if (CCG_BOOT != 0)
#if (BOOT_WAIT_WINDOW_DISABLE == 0)
/* Return the boot-wait setting to the user code. */
uint16_t boot_get_wait_time(void)
{
    return (gl_boot_wait_delay);
}

static void boot_set_wait_timeout(sys_fw_metadata_t *md_p)
{
    /* Check for boot-wait option. */
    if (md_p->boot_app_id == CCG_FWMETA_APPID_WAIT_0)
    {
        gl_boot_wait_delay = 0;
    }
    else
    {
        if (md_p->boot_app_id != CCG_FWMETA_APPID_WAIT_DEF)
        {
            /* Get the boot-wait delay from metadata, applying the MIN and MAX limits. */
            gl_boot_wait_delay = GET_MAX (CCG_BL_WAIT_MAXIMUM, GET_MIN (CCG_BL_WAIT_MINUMUM,
                        md_p->boot_app_id));
        }
    }
}
#endif /* BOOT_WAIT_WINDOW_DISABLE */

/*This function schedules recent and valid FW. It sets boot mode reason */
bool boot_start(void)
{
    sys_fw_metadata_t *md_p;
    uint8_t img;

    bool boot_fw1 = false;
    bool boot_fw2 = false;

#if (CCG_DUALAPP_DISABLE)
    (void)boot_fw1;
    (void)boot_fw2;
#endif /* CCG_DUALAPP_DISABLE */

    md_p = NULL;
    gl_img_status.val = 0;
   
    /* Check the two firmware binaries for validity. */
    if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
    {
        gl_img_status.status.fw1_invalid  = 1;
    }

#if (!CCG_DUALAPP_DISABLE)
    if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
#endif /* CCG_DUALAPP_DISABLE */
    {
        gl_img_status.status.fw2_invalid = 1;
    }
    
    /* Check for the boot mode request. */
    /*
     * NOTE: cyBtldrRunType is Bootloader component provided variable.
     * It is used to store the jump signature. Check the lower two bytes
     * for signature.
     */
    if ((cyBtldrRunType & 0xFFFF) == SYS_BOOT_MODE_RQT_SIG)
    {
        /*
         * FW has made a request to stay in boot mode. Return
         * from here after clearing the variable.
         */
        cyBtldrRunType = 0;
        /* Set the reason for boot mode. */
        gl_img_status.status.boot_mode_request = true;
        return false;
    }
    
    /* Check if we have been asked to boot FW1 or FW2 specifically. */
    if ((cyBtldrRunType & 0xFFFF) == CCG_FW1_BOOT_RQT_SIG)
        boot_fw1 = true;

#if (!CCG_DUALAPP_DISABLE)
    if ((cyBtldrRunType & 0xFFFF) == CCG_FW2_BOOT_RQT_SIG)
        boot_fw2 = true;
#endif /* CCG_DUALAPP_DISABLE */

#if (!CCG_DUALAPP_DISABLE)
    /*
     * If we have been specifically asked to boot FW2, do that.
     * Otherwise, if we have not been specifically asked to boot FW1; choose the binary with
     * greater sequence number.
     */
    if (!gl_img_status.status.fw2_invalid)
    {
        /* 
         * FW2 is valid.
         * We can boot this if:
         * 1. We have been asked to boot FW2.
         * 2. FW1 is not valid.
         * 3. FW2 is newer than FW1, and we have not been asked to boot FW1.
         */
        if ((boot_fw2) || (gl_img_status.status.fw1_invalid) ||
                ((!boot_fw1) && (sys_get_recent_fw_image() == SYS_FW_MODE_FWIMAGE_2)))
        {
            md_p = gl_img2_fw_metadata;
            img  = Bootloader_1_MD_BTLDB_ACTIVE_1;
        }
        else
        {
            md_p = gl_img1_fw_metadata;
            img  = Bootloader_1_MD_BTLDB_ACTIVE_0;
        }
    }
    else
#endif /* CCG_DUALAPP_DISABLE */
    {
        /* FW2 is invalid. */
        /* Load FW1 if it is valid. */
        if (!gl_img_status.status.fw1_invalid)
        {
            md_p = gl_img1_fw_metadata;
            img  = Bootloader_1_MD_BTLDB_ACTIVE_0;
        }
    }
    
    if (md_p != NULL)
    {
#if (BOOT_WAIT_WINDOW_DISABLE == 0)
        /*
         * If we are in the middle of a jump-to-alt-fw command, do not provide
         * the boot wait window.
         */
        if ((boot_fw1) || (boot_fw2))
            gl_boot_wait_delay = 0;
        else
            boot_set_wait_timeout (md_p);
#endif /* BOOT_WAIT_WINDOW_DISABLE */

        Bootloader_1_activeApp = img;
        return true;
    }

    /* Stay in bootlaoder. */
    return false;
}

void boot_jump_to_fw(void)
{
    /* Schedule the FW and undergo a reset. */
    Bootloader_1_SET_RUN_TYPE (Bootloader_1_START_APP);
    CySoftwareReset ();
}
#else /* !CCG_BOOT */

void boot_update_fw_status(void)
{
#if (!CCG_BOOT)
    gl_img_status.val = 0;

    /* Check the two firmware binaries for validity. */
    if (boot_handle_validate_fw_cmd (SYS_FW_MODE_FWIMAGE_1) != CCG_STAT_SUCCESS)
    {
        gl_img_status.status.fw1_invalid = 1;
    }

#if (!CCG_DUALAPP_DISABLE)
    if (boot_handle_validate_fw_cmd (SYS_FW_MODE_FWIMAGE_2) != CCG_STAT_SUCCESS)
#endif /* !CCG_DUALAPP_DISABLE */
    {
        gl_img_status.status.fw2_invalid = 1;
    }

#if APP_PRIORITY_FEATURE_ENABLE
    /* Update the app-priority field if the feature is enabled. */
    gl_img_status.status.reserved1 = ((*(uint8_t *)(CCG_APP_PRIORITY_ROW_NUM << CCG_FLASH_ROW_SHIFT_NUM)) << 4);
#endif /* APP_PRIORITY_FEATURE_ENABLE */

#endif /* CCG_BOOT */
}
    
#if (CCG_PSEUDO_METADATA_DISABLE == 0)

void boot_check_for_valid_fw(void)
{
    /* Temporary data buffer (of size one Flash row) to store contents of pseudo metadata. */
    uint8_t temp_pseudo_metadata_buf[CCG_FLASH_ROW_SIZE] = {0};
    sys_fw_metadata_t *p_md, *alt_p_md, *md, *alt_md;
    uint16_t alt_p_md_row_num, alt_md_row_num, p_md_row_num;
    /* To hold the bit position of alternate image in Image status structure. */
    uint8_t alt_img_bit_pos;
    bool invalidate_md = false;
#if (SECURE_FW_UPDATE == 1)
    /* Buffer to hold FW HASH. */
    uint32_t fw_hash[CRYPTO_SHA_2_HASH_SIZE_WORDS] = {0};
#endif /* SECURE_FW_UPDATE */

    /*Pointer to the metadata table's start address  in pseudo metadata row buffer. */
    sys_fw_metadata_t *temp_fw_pmetadata = (sys_fw_metadata_t *)
            (temp_pseudo_metadata_buf + (CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE));

    /* Determine the pseudo and actual metadata row details based on FW mode. */
    if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
    {
        p_md = gl_img1_fw_pseudo_metadata;
        alt_p_md = gl_img2_fw_pseudo_metadata;
        md = gl_img1_fw_metadata;
        alt_md = gl_img2_fw_metadata;
        p_md_row_num = CCG_IMG1_LAST_FLASH_ROW_NUM;
        alt_p_md_row_num = CCG_IMG2_PSEUDO_METADATA_ROW_NUM;
        alt_md_row_num = CCG_IMG2_METADATA_ROW_NUM;
        alt_img_bit_pos = 3;
    }
    else
    {
        p_md = gl_img2_fw_pseudo_metadata;
        alt_p_md = gl_img1_fw_pseudo_metadata;
        md = gl_img2_fw_metadata;
        alt_md = gl_img1_fw_metadata;
        p_md_row_num = CCG_IMG2_PSEUDO_METADATA_ROW_NUM;
        alt_p_md_row_num = gl_img2_fw_metadata->boot_last_row;
        alt_md_row_num = CCG_IMG1_METADATA_ROW_NUM;
        alt_img_bit_pos = 2;
    }
    
    /* Current FW shall ensure that it's PMD row is cleared if it contains
     * valid signature. This ensures there is no stale PMD information in the flash. */
    if (p_md->metadata_valid == SYS_PSEUDO_METADATA_VALID_SIG)
    {
        flash_row_clear (p_md_row_num);
    }
        
    /*
     * Read pseudo-metadata row of other fw image. If pseudo-metadata has
     * signature "CP", which indicates that other FW Image got updated before
     * re-boot, and FW image is valid then copy this pseudo-metadta in to
     * its actual metadata row after changing its signature and
     * increamenting image pointer.
     */
    if (alt_p_md->metadata_valid == SYS_PSEUDO_METADATA_VALID_SIG)
    {
        if(boot_validate_fw (alt_p_md) == CCG_STAT_SUCCESS)
        {
#if (SECURE_FW_UPDATE == 1)
           if(sboot_authenticate_fw(alt_p_md, alt_p_md_row_num, fw_hash) == CCG_STAT_SUCCESS)
#endif /* SECURE_FW_UPDATE */
           {
                /* Read pesudo metadata in a temp buffer. */
                MEM_COPY ((uint8_t *)temp_fw_pmetadata, (uint8_t *)alt_p_md,
                    CCG_METADATA_TABLE_SIZE);
                temp_fw_pmetadata->metadata_valid = SYS_METADATA_VALID_SIG;
                temp_fw_pmetadata->boot_seq = (md->boot_seq) + 1;
#if (SECURE_FW_UPDATE == 1)
                /* Copy FW HASH in first 32 bytes of pseudo metadata row. */
                MEM_COPY (temp_pseudo_metadata_buf, (uint8_t *)fw_hash,
                    CRYPTO_SHA_2_HASH_SIZE_BYTES);
#endif /* SECURE_FW_UPDATE */
                /* This flash write can always be blocking. */
                if (CYRET_SUCCESS == CySysFlashWriteRow (alt_md_row_num,
                            temp_pseudo_metadata_buf))
                {
                    /*Invalidate Pseudo-metadata of FW Image 2.*/
                    flash_row_clear (alt_p_md_row_num);
                    /* Undergo Software Reset. */
                    CySoftwareReset();
                }
            }
#if (SECURE_FW_UPDATE == 1)
            else
            {
                invalidate_md = true;
            }
#endif /* SECURE_FW_UPDATE */
        }
        else
        {
            invalidate_md = true;
        }
    }
    else
    {
        /* Check if other image is valid. */
        if (boot_validate_fw (alt_md) != CCG_STAT_SUCCESS)
        {
            /* Mark the other image as invalid. */
            gl_img_status.val |= (1 << alt_img_bit_pos);
        }
    }

    if (invalidate_md)
    {
        /*Invalidate Pseudo-metadata and Metadata of other FW Image.*/
        flash_row_clear (alt_md_row_num);
        flash_row_clear (alt_p_md_row_num);
        /* Mark the other image as invalid. */
        gl_img_status.val |= (1 << alt_img_bit_pos);
    }
}
#endif /* CCG_PSEUDO_METADATA_DISABLE */

#endif /* CCG_BOOT */

uint32_t boot_get_boot_seq(uint8_t fwid)
{
    sys_fw_metadata_t *md_p;

    if (fwid == SYS_FW_MODE_FWIMAGE_1)
        md_p = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
#if (!CCG_DUALAPP_DISABLE)
    else
        md_p = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
#endif /* (!CCG_DUALAPP_DISABLE) */

#if (CCG_BOOT != 0)
    if (boot_validate_fw (md_p) == CCG_STAT_SUCCESS)
    {
        return (md_p->boot_seq);
    }
#else /* (CCG_BOOT != 0) */

    /* We only need to validate if the target is not the active firmware. */
    if ((sys_get_device_mode () == fwid) || (boot_validate_fw (md_p) == CCG_STAT_SUCCESS))
    {
        return (md_p->boot_seq);
    }
#endif /* (CCG_BOOT != 0) */
    return 0;
}
/* [] END OF FILE */
