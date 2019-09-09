/**
 * @file main.c
 *
 * @brief @{Main source file for CCG firmware implementation.@}
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
#include <flash_config.h>
#include <system.h>
#include <timer.h>
#include <hpi.h>
#include <boot.h>
#include <flash.h>
#include <status.h>
#include <ccgx_version.h>
#include <app_version.h>
#include <utils.h>
#include <gpio.h>
#include <pd.h>
#include <dpm.h>
#include <app.h>
#include <psource.h>
#include <psink.h>
#include <pdo.h>
#include <swap.h>
#include <vdm.h>
#include <hal_ccgx.h>
#include <solution.h>
#include <instrumentation.h>

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#endif /* RIDGE_SLAVE_ENABLE */

/*
 * Reserve 32 bytes of space for Customer related info.
 * Fill this with customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
__attribute__ ((section(".customer_region"), used))
const uint32_t customer_info[8] = {0x00};

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
__attribute__ ((section(".base_version"), used))
const uint32_t base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
const uint32_t app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
const uint32_t ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".fw_reserved"), used))
const uint32_t reserved_buf[5] = {0};

/* Solution PD event handler */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data)
{
#if (CCG_HPI_PD_ENABLE == 1)
    /* Pass the callback to HPI */
    hpi_pd_event_handler(port, evt, data);
#endif /* (CCG_HPI_PD_ENABLE) */
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const app_cbk_t app_callback =
{
    app_event_handler,
    psrc_set_voltage,
    psrc_set_current,
    sln_psrc_enable,
    sln_psrc_disable,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_rdo,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
#if CCG_PD_REV3_ENABLE
    eval_fr_swap,
#else
    0,
#endif /* CCG_PD_REV3_ENABLE */
    vbus_get_value,
};

app_cbk_t* app_get_callback_ptr(uint8_t port)
{
    (void)port;
    /* Solution callback pointer is same for all ports */
    return ((app_cbk_t *)(&app_callback));
}

#if CCG_FIRMWARE_APP_ONLY
/*
   Provide a variable required by the HPI library. The real variable only
   exists if the boot-loadable component is included.
 */
volatile uint32_t cyBtldrRunType = 0;
#endif /* CCG_FIRMWARE_APP_ONLY */

#if CCG_HPI_ENABLE

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
#if CCG_FIRMWARE_APP_ONLY
    uint8_t mode, reason;
    uint8_t ver_invalid[8] = {0};

    /* Flash access is not allowed. */
    flash_set_access_limits (CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM,
            CCG_BOOT_LOADER_LAST_ROW);

    /* Update HPI registers with default values. */
    mode   = 0x95;              /* Dual boot, 256 byte flash, 2 ports, FW1 running. */
    reason = 0x08;              /* FW2 is not valid. */
    hpi_set_mode_regs (mode, reason);
    hpi_update_versions (ver_invalid, (uint8_t *)&base_version, ver_invalid);
    hpi_update_fw_locations (0, CCG_LAST_FLASH_ROW_NUM);

#else /* !CCG_FIRMWARE_APP_ONLY */

    uint8_t mode, reason = 0x00;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    sys_fw_metadata_t *fw1_md, *fw2_md;
    uint8_t ver_invalid[8] = {0};

    /* Set mode variables and flash access limits based on the active firmware. */
    if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
    {
        mode = 0x81 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW2 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x08;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_IMG1_LAST_FLASH_ROW_NUM + 1, CCG_IMG2_LAST_FLASH_ROW_NUM,
                CCG_IMG2_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }
    else
    {
        mode = 0x82 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW1 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x04;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_BOOT_LOADER_LAST_ROW + 1, gl_img2_fw_metadata->boot_last_row,
                CCG_IMG1_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }

    hpi_set_mode_regs (mode, reason);

    /* Calculate the version address from the firmware metadata. */
    if ((reason & 0x04) == 0)
    {
        fw1_md  = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
        fw1_ver = (((uint32_t)fw1_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw1_loc = fw1_md->boot_last_row + 1;
    }
    else
    {
        fw1_ver = (uint32_t)ver_invalid;
        fw1_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    if ((reason & 0x08) == 0)
    {
        fw2_md  = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
        fw2_ver = (((uint32_t)fw2_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw2_loc = fw2_md->boot_last_row + 1;
    }
    else
    {
        fw2_ver = (uint32_t)ver_invalid;
        fw2_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    /* Update version information in the HPI registers. */
    hpi_update_versions (
            (uint8_t *)SYS_BOOT_VERSION_ADDRESS,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );

    /* Update firmware location registers. */
    hpi_update_fw_locations (fw1_loc, fw2_loc);

    /*
     * Provide firmware boot priority information through HPI register interface. Default value is good for
     * the case where the last flashed binary is prioritized.
     */
#if CCG_PRIORITIZE_FW1
    hpi_set_boot_priority_conf (HPI_BOOT_PRIO_FW1);
#elif CCG_PRIORITIZE_FW2
    hpi_set_boot_priority_conf (HPI_BOOT_PRIO_FW2);
#endif /* CCG_PRIORITIZE_FWX */

#endif /* CCG_FIRMWARE_APP_ONLY */
}
#endif /* CCG_HPI_ENABLE */

#if APP_FW_LED_ENABLE

/* Blink the LED every LED_TIMER_PERIOD ms. This serves as an indication of the firmware running. */
void led_timer_cb (
    uint8_t port,
    timer_id_t id)
{
    (void)port;
    (void)id;

    gpio_set_value (FW_LED_GPIO_PORT_PIN, !(gpio_read_value (FW_LED_GPIO_PORT_PIN)));
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);
}

#endif /* APP_FW_LED_ENABLE */

int main()
{
    uint32_t conf_addr;
    uint8_t i;

    /* Enable this to delay the firmware execution under SWD connect. */
#ifdef BREAK_AT_MAIN
    uint8_t volatile x = 0;
    while(x==0);
#endif /* BREAK_AT_MAIN */

#if (CYDEV_BCLK__SYSCLK__MHZ == 48)
    /* Use the maximum number of flash access wait states when the CPU clock frequency is 48 MHz. */
    CPUSS_FLASH_CTL = 0x13;
#endif /* (CYDEV_BCLK__SYSCLK__MHZ == 48) */

#if CCG_FIRMWARE_APP_ONLY
    sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
#else /* !CCG_FIRMWARE_APP_ONLY */
    if ((uint32_t)&base_version < CCG_FW1_CONFTABLE_MAX_ADDR)
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
    }
    else
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_2);
    }
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Validate the signature and checksum of the configuration table. */
    conf_addr = (uint32_t)get_pd_config ();
    if ((boot_validate_configtable ((uint8_t *)conf_addr) != CCG_STAT_SUCCESS) ||
            (!app_validate_configtable_offsets()))
    {
#if CCG_FIRMWARE_APP_ONLY
        /* Can't do anything if config table is not valid. */
        while (1);
#else /* !CCG_FIRMWARE_APP_ONLY */
        /* Erase the firmware metadata so that this binary stops loading. */
        if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
        {
            flash_row_clear (CCG_IMG1_METADATA_ROW_NUM);
        }
        else
        {
            flash_row_clear (CCG_IMG2_METADATA_ROW_NUM);
        }

        /* Now reset the device. */
        CySoftwareReset ();
#endif /* CCG_FIRMWARE_APP_ONLY */
    }

    /* Initialize PD block clocks. */
    system_init();

    /* Timer INIT has to be done first. */
    timer_init();

    /* Enable global interrupts */
    CyGlobalIntEnable;

#if RIDGE_SLAVE_ENABLE
    ridge_slave_init (RIDGE_SLAVE_SCB, 0);
#endif /* RIDGE_SLAVE_ENABLE */

#if CCG_HPI_ENABLE

#if (DISABLE_I2C_ADDR_CONFIG == 0)
    /* Set the HPI slave address based on the state of the I2C_CFG pin. */
    get_hpi_slave_addr();
#endif /* (DISABLE_I2C_ADDR_CONFIG == 0) */

#if CCG_FIRMWARE_APP_ONLY
    /* Configure HPI for no-boot operation. */
    hpi_set_no_boot_mode (true);
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Set the flash sizes and bootloader size limits. */
    hpi_set_flash_params (CCG_FLASH_SIZE, CCG_FLASH_ROW_SIZE, CCG_LAST_FLASH_ROW_NUM + 1, CCG_BOOT_LOADER_LAST_ROW);

    /* Initialize the HPI interface. */
    hpi_init (HPI_SCB_INDEX);

    /* Update HPI registers with mode and version information. */
    update_hpi_regs ();

#endif /* CCG_HPI_ENABLE */

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Configure the VBus detach detection parameters (same as VBus poll params). */
    pd_hal_set_vbus_detach_params (APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);

#if (MUX_INIT_DELAY_MS != 0)
    /* Specify the MUX enable delay value to be used. */
    dpm_update_mux_enable_wait_period(MUX_INIT_DELAY_MS);
#endif /* (MUX_INIT_DELAY_MS != 0) */

    /* Perform application level initialization. */
    app_init();

    /* Initialize the Device Policy Manager for each PD port on the device. */
    for (i = TYPEC_PORT_0_IDX; i < NO_OF_TYPEC_PORTS; i++)
    {
        dpm_init (i, app_get_callback_ptr(i));
    }

#if CCG_HPI_ENABLE

    /* Send a reset complete event to the EC. */
    hpi_send_fw_ready_event ();

    /* Wait until EC ready event has been received or 100 ms has elapsed. */
    for (i = 0; i < 100; i ++)
    {
        hpi_task ();
        if (hpi_is_ec_ready ())
            break;

#if (CYDEV_BCLK__SYSCLK__MHZ == 48)
        /*
         * Using CyDelayCycles as CyDelay is leading to a 25% error.
         * The parameter used here assumes that CPU clock is running at 48 MHz.
         */
        CyDelayCycles (38400);
#else
        /*
         * Using CyDelayCycles as CyDelay is leading to a 25% error.
         * The parameter used here assumes that CPU clock is running at 24 MHz.
         */
        CyDelayCycles (19200);
#endif /* (CYDEV_BCLK__SYSCLK__MHZ == 48) */
    }

#endif /* CCG_HPI_ENABLE */

#if APP_FW_LED_ENABLE

    /* Configure the LED control GPIO as an output. */
    gpio_hsiom_set_config (FW_LED_GPIO_PORT_PIN, HSIOM_MODE_GPIO, GPIO_DM_STRONG, true);
    /* Start a timer that will blink the FW ACTIVE LED, if required. */
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);

#endif /* APP_FW_LED_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    for (i = TYPEC_PORT_0_IDX ; i < NO_OF_TYPEC_PORTS; i++)
    {
#if CCG_HPI_ENABLE
        /* Start the DPM for the port only if it is enabled at the HPI level. It is possible that the port
           got disabled before we got here.
         */
        if ((hpi_get_port_enable () & (1 << i)) != 0)
#endif /* CCG_HPI_ENABLE */
        {
            dpm_start (i);
        }
    }

    /* Enable NCP81239 controller */
    pd_ctrl_init();

    while(1)
    {
        /* Perform DPM, APP and HPI tasks. */
        for (i = TYPEC_PORT_0_IDX; i < NO_OF_TYPEC_PORTS; i++)
        {
            dpm_task (i);
            app_task (i);

#if CCG_HPI_ENABLE
            /* Handle any pending HPI commands. */
            hpi_task ();
#endif /* CCG_HPI_ENABLE */

            /* Perform tasks associated with instrumentation. */
            instrumentation_task();
        }

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter sleep mode for power saving. */
        system_sleep();
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* End of file */
