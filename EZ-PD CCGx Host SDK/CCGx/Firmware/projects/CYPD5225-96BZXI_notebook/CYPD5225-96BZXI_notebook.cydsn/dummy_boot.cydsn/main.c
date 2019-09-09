/**
 * @file main.c
 *
 * @brief @{Main source file for the CCG5 Notebook bootloader project.@}
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

#include <project.h>
#include <stdbool.h>
#include "ccgx_version.h"
#include "app_version.h"
#include "config.h"
#include "flash_config.h"
#include "system.h"
#include "flash.h"
#include "i2c.h"
#include "hpi.h"
#include "boot.h"
#include "timer.h"
#include "utils.h"

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
__attribute__ ((section(".base_version"), used))
const uint32_t base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
const uint32_t app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
const uint32_t ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".boot_type"), used))
const uint32_t boot_loader_type = ((APP_PRIORITY_FEATURE_ENABLE << SYS_BOOT_TYPE_APP_PRIORITY_POS)
    | ((~FLASHING_MODE_HPI_ENABLE & 0x00000001) << SYS_BOOT_TYPE_FW_UPDATE_INTERFACE_POS)
    | SECURE_FW_UPDATE);
__attribute__ ((section(".fw_reserved"), used))
const uint32_t reserved_buf[4] = {0};

#if BOOTWAIT_ENABLE
/* Boot-wait window related defines and variables. */
static volatile bool gl_bootwait_elapsed = false;

/* Timer callback used to identify that boot-wait window has elapsed. */
static void
bl_timer_cb (
        uint8_t    port,
        timer_id_t id)
{
    /* BL only uses the one timer. We can ignore the parameters. */
    (void)port;
    (void)id;

    /* Ignore the boot-wait timer if flash access has been enabled. */
    if (flash_access_get_status (1 << FLASH_IF_HPI) == 0)
    {
        gl_bootwait_elapsed = true;
    }
}
#endif

static void
ccg_device_sleep (
        void)
{
    /* TODO: We can go into deep sleep here. */
}

#if (DISABLE_I2C_ADDR_CONFIG == 0)
/*
 * This function checks the I2C_CFG pin strap status to identify the I2C
 * slave address to be used for the HPI interface.
 */
static void get_hpi_slave_addr(void)
{
    uint8_t addr = HPI_ADDR_I2C_CFG_FLOAT;

    /* Check if IO is driven low. */
    I2C_CFG_SetDriveMode(I2C_CFG_DM_RES_UP);
    I2C_CFG_Write (1);
    CyDelayUs (5);
    if (I2C_CFG_Read () == 0)
    {
        addr = HPI_ADDR_I2C_CFG_LOW;
    }
    else
    {
        /* Check if IO is driven high. */
        I2C_CFG_SetDriveMode(I2C_CFG_DM_RES_DWN);
        I2C_CFG_Write(0);
        CyDelayUs(5);
        if (I2C_CFG_Read() != 0)
        {
            addr = HPI_ADDR_I2C_CFG_HIGH;
        }
    }

    /* Disable the pull up/pull down on IO. */
    I2C_CFG_SetDriveMode(I2C_CFG_DM_ALG_HIZ);

    hpi_set_fixed_slave_address(addr);
}
#endif /* (DISABLE_I2C_ADDR_CONFIG != 0) */

static void
update_hpi_regs (
        void)
{
    uint8_t mode, reason;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    sys_fw_metadata_t *fw1_md;

#if (!CCG_DUALAPP_DISABLE)
    sys_fw_metadata_t *fw2_md;
#endif

    uint8_t invalid_ver[8] = {0};

    /* Dual FW mode, 256 byte flash row, selected no. of ports, boot-loader running. */
    mode   = 0x90 | ((NO_OF_TYPEC_PORTS - 1) << 2);
    reason =  get_boot_mode_reason().val;
    hpi_set_mode_regs (mode, reason);

    /* Calculate the version address from the firmware metadata. */
    if ((reason & 0x04) != 0)
    {
        fw1_ver = (uint32_t)invalid_ver;
        fw1_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }
    else
    {
        fw1_md  = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
        fw1_ver = (((uint32_t)fw1_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM)
                + SYS_FW_VERSION_OFFSET;
        fw1_loc = fw1_md->boot_last_row + 1;
    }

#if (!CCG_DUALAPP_DISABLE)
    if ((reason & 0x08) != 0)
#endif
    {
        fw2_ver = (uint32_t)invalid_ver;
        fw2_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }
#if (!CCG_DUALAPP_DISABLE)
    else
    {
        fw2_md  = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
        fw2_ver = (((uint32_t)fw2_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM)
                + SYS_FW_VERSION_OFFSET;
        fw2_loc = fw2_md->boot_last_row + 1;
    }
#endif

    /* Update version information in the HPI registers. */
    hpi_update_versions (
            (uint8_t *)SYS_BOOT_VERSION_ADDRESS,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );

    /* Update firmware location registers. */
    hpi_update_fw_locations (fw1_loc, fw2_loc);
}

int main()
{
#if BOOTWAIT_ENABLE
    uint16_t wait = 0;
#endif

#if (DISABLE_I2C_ADDR_CONFIG == 0)
    /* Set the HPI slave address based on the state of the I2C_CFG pin. */
    get_hpi_slave_addr();
#endif /* (DISABLE_I2C_ADDR_CONFIG == 0) */

    /* Set the flash sizes and bootloader size limits. */
    hpi_set_flash_params (CCG_FLASH_SIZE, CCG_FLASH_ROW_SIZE, CCG_LAST_FLASH_ROW_NUM + 1, CCG_BOOT_LOADER_LAST_ROW);

    /* Initialize the HPI interface. */
    hpi_init (HPI_SCB_INDEX);

    CyGlobalIntEnable; /* Enable global interrupts. */

    /* If we have a valid firmware binary, load it. */
    if (boot_start () == true)
    {
#if BOOTWAIT_ENABLE
        wait = boot_get_wait_time ();
        if (wait == 0)
#endif
        {
            boot_jump_to_fw ();
        }
#if BOOTWAIT_ENABLE
        else
        {
            /* Make sure boot-wait elapsed flag is cleared. */
            gl_bootwait_elapsed = false;

            /* We need a timer to wait for the boot-wait timeout period. */
            timer_init ();
            timer_start (0, BL_BOOT_WAIT_TIMER_ID, wait, bl_timer_cb);
        }
#endif
    }

    /* Send a reset complete event to the EC. */
    hpi_send_fw_ready_event ();

    /* Update the HPI registers. */
    update_hpi_regs ();

    /* Set the flash access boundaries so that the boot-loader itself cannot be overwritten. */
    flash_set_access_limits (CCG_BOOT_LOADER_LAST_ROW + 1, CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM,
            CCG_BOOT_LOADER_LAST_ROW);

    for(;;)
    {
        /* Handle any pending HPI commands. */
        hpi_task ();

#if BOOTWAIT_ENABLE
        /* Jump to the selected firmware once the boot-wait window has elapsed. */
        if (gl_bootwait_elapsed)
        {
            boot_jump_to_fw ();
        }
#endif

        ccg_device_sleep ();
    }
}

/* [] END OF FILE */
