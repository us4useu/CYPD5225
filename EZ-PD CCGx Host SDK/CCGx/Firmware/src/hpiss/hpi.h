/**
 * @file hpi.h
 *
 * @brief @{Host Processor Interface (HPI) header file.@}
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

#ifndef _HPI_H_
#define _HPI_H_

#include <stdbool.h>
#include <stdint.h>
#include <config.h>
#include "status.h"
#include "i2c.h"

#if CCG_HPI_PD_ENABLE
#include "pd.h"
#include "dpm.h"
#include "app.h"
#endif /* CCG_HPI_PD_ENABLE */

/**************************************************************************************************
 ******************************************** MACROS **********************************************
 *************************************************************************************************/
#if CCG_UCSI_ENABLE
#define UCSI_STARTED                    0x00
#define UCSI_CMD_IN_PROGRESS            0x01
#define UCSI_EVENT_PENDING              0x02
#define UCSI_START                      0x01
#define UCSI_STOP                       0x02
#define UCSI_SILENCE                    0x03
#define UCSI_SIGNAL_CONNECT_EVENT_TO_OS 0x04 
#endif /* CCG_UCSI_ENABLE */
/**
 * @brief Timer ID used for command retry checks.
 */
#define HPI_PD_CMD_TIMER                (0x80)

/**
 * @brief I2C slave address to be used for HPI interface, when the I2C_CFG pin is sensed as LOW.
 */
#define HPI_ADDR_I2C_CFG_LOW            (0x40)

/**
 * @brief I2C slave address to be used for HPI interface, when the I2C_CFG pin is sensed as HIGH.
 */
#define HPI_ADDR_I2C_CFG_HIGH           (0x42)

/**
 * @brief I2C slave address to be used for HPI interface, when the I2C_CFG pin is sensed as FLOATING.
 */
#define HPI_ADDR_I2C_CFG_FLOAT          (0x08)

/**************************************************************************************************
 ****************************************** DATA TYPES ********************************************
 *************************************************************************************************/

/**
 * @typedef hpi_reg_section_t
 * @brief HPI register section definitions.
 *
 * HPI registers are grouped into sections corresponding to the functions that are supported.
 */
typedef enum
{
    HPI_REG_SECTION_DEV = 0,            /**< Device information registers. */
    HPI_REG_SECTION_PORT_0,             /**< USB-PD Port 0 related registers. */
    HPI_REG_SECTION_PORT_1,             /**< USB-PD Port 1 related registers. */
#if CCG_UCSI_ENABLE    
    HPI_REG_SECTION_UCSI = 0x0F,        /**< UCSI related registers. */    
#endif /*CCG_UCSI_ENABLE*/    
    HPI_REG_SECTION_ALL                 /**< Special definition to select all register spaces. */
} hpi_reg_section_t;

/**
 * @typedef hpi_reg_part_t
 * @brief Types of HPI register/memory regions.
 */
typedef enum
{
    HPI_REG_PART_REG = 0,               /**< Register region. */
    HPI_REG_PART_DATA = 1,              /**< Data memory for device section. */
    HPI_REG_PART_FLASH = 2,             /**< Flash memory. */
    HPI_REG_PART_PDDATA_READ = 4,       /**< Read Data memory for port section. */
    HPI_REG_PART_PDDATA_READ_H = 5,     /**< Upper fraction of read data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE = 8,      /**< Write Data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE_H = 9     /**< Upper fraction of write data memory for port section. */
} hpi_reg_part_t;

/**
 * @typedef hpi_write_cb_t
 * @brief Handler for HPI register writes.
 * @return Type of response to be sent to the EC. Only a single byte response
 * can be sent from here. Use hpi_reg_enqueue_event to send longer responses.
 */
typedef uint8_t (*hpi_write_cb_t)(
        uint16_t  reg_addr,             /**< Address of register that got written. */
        uint8_t   wr_size,              /**< Size of write operation. */
        uint8_t  *wr_data               /**< Buffer containing data written. */
        );

/**
 * @typedef hpi_boot_prio_conf_t
 * @brief Enumeration showing possible boot priority configurations for the firmware application.
 */
typedef enum
{
    HPI_BOOT_PRIO_LAST_FLASHED = 0,     /**< Last flashed firmware is prioritized. */
    HPI_BOOT_PRIO_APP_PRIO_ROW,         /**< Priority defined used App Priority flash row. */
    HPI_BOOT_PRIO_FW1,                  /**< FW1 is always prioritized. */
    HPI_BOOT_PRIO_FW2                   /**< FW2 is always prioritized. */
} hpi_boot_prio_conf_t;

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/

/**
 * @brief Initialize the HPI interface.
 *
 * This function initializes the I2C interface and EC_INT GPIO used for HPI hardware interface, and
 * initializes all HPI registers to their default values.
 *
 * @param scb_idx Index of SCB block to be used for HPI. Please note that this parameter is
 * not validated, and it is the caller's responsibility to pass the correct value.
 * @return None
 */
void hpi_init(uint8_t scb_idx);

/**
 * @brief De-initialize the HPI interface.
 *
 * This function can be used to de-initialize the HPI interface. This can be used in applications
 * where the HPI master (Billboard controller) in the system may be powered off under control of
 * the CCG device.
 *
 * @return None
 */
void hpi_deinit(void);

/**
 * @brief HPI task handler.
 *
 * This function handles the commands from the EC through the HPI registers. HPI writes from the EC
 * are handled in interrupt context, and any associated work is queued to be handled by this function.
 * The hpi_task is expected to be called periodically from the main task loop of the firmware
 * application.
 *
 * @return None
 */
void hpi_task(void);

/**
 * @brief Enqueue an event to the EC through the HPI interface.
 *
 * This function is used by the PD stack and application layers to send event
 * notifications to the EC through the HPI registers.
 *
 * Please note that only responses without associated data can be sent through
 * the response register for the HPI_REG_SECTION_DEV section. If any additional
 * response data is required, please use the user defined registers or the response
 * register associated with HPI_REG_SECTION_PORT_0 or HPI_REG_SECTION_PORT_1. 
 *
 * @param section Register section through which event is to be reported.
 * @param status The event code to be stored into the response register.
 * @param length Length of the data associated with the event.
 * @param data Pointer to buffer containing data associated with the event.
 *
 * @return true if the event queue has space for the event, false if there
 * is an overflow.
 */
bool hpi_reg_enqueue_event(hpi_reg_section_t section, uint8_t status, uint16_t length,
    uint8_t *data);

#if CCG_HPI_PD_ENABLE

/**
 * @brief Handler for PD events reported from the stack.
 *
 * Internal function used to receive PD events from the stack and to update the HPI registers.
 *
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that needs to be de-referenced
 * based on event type.
 *
 * @return None
 */
void hpi_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

#endif /* CCG_HPI_PD_ENABLE */

/**
 * @brief Check whether EC init complete event has been received.
 *
 * This function is used by the application to check whether the EC has sent
 * the EC initialization complete event notification.
 *
 * @return true if EC init has been received, false otherwise.
 */
bool hpi_is_ec_ready (void);

/**
 * @brief Check whether any HPI accesses have happened since start-up.
 * @return True if any HPI read/write access has happened.
 */
bool hpi_is_accessed (void);

/**
 * @brief Update firmware version information in HPI registers.
 *
 * This is an internal function used to update the firmware version information
 * in the HPI registers.
 *
 * @param bl_version Buffer containing Bootloader version information.
 * @param fw1_version Buffer containing firmware-1 version information.
 * @param fw2_version Buffer containing firmware-2 version information.
 *
 * @return void
 */
void hpi_update_versions (uint8_t *bl_version, uint8_t *fw1_version,
        uint8_t *fw2_version);

/**
 * @brief Set device mode and reason register values.
 *
 * This is an internal function used to update the device mode and boot mode
 * reason HPI registers.
 *
 * @param dev_mode Value to be set into the device mode register.
 * @param mode_reason Value to be set into the boot mode reason register.
 *
 * @return void
 */
void hpi_set_mode_regs (uint8_t dev_mode, uint8_t mode_reason);

/**
 * @brief Update the firmware location HPI registers.
 *
 * This is an internal function used to update the firmware binary location
 * HPI registers.
 *
 * @param fw1_location Flash row where FW1 is located.
 * @param fw2_location Flash row where FW2 is located.
 *
 * @return void
 */
void hpi_update_fw_locations (uint16_t fw1_location, uint16_t fw2_location);

/**
 * @brief Check whether EC control of VDMs is enabled.
 *
 * @param port USB-PD port to check the configuration for.
 *
 * @return true if EC control is enabled, false otherwise.
 */
bool hpi_is_vdm_ec_ctrl_enabled (uint8_t port);

/**
 * @brief Check whether handling of extended messages by EC is enabled. If not enabled,
 * CCG firmware will automatically respond with NOT_SUPPORTED messages.
 *
 * @param port USB-PD port to check the configuration for.
 *
 * @return true if EC handling of extended messages is enabled, false otherwise.
 */
bool hpi_is_extd_msg_ec_ctrl_enabled (uint8_t port);

/**
 * @brief Get the active EC alternate modes value.
 *
 * @param port USB-PD to check the configuration for.
 *
 * @return The Active EC modes setting programmed by EC.
 */
uint8_t hpid_get_ec_active_modes (uint8_t port);

/**
 * @brief Check if the CCG device can be put into deep-sleep.
 *
 * @return true if deep sleep is possible, false otherwise.
 */
bool hpi_sleep_allowed (void);

/**
 * @brief Prepare the HPI interface for device deep sleep.
 *
 * This function checks whether the I2C interface is idle so that the CCG device can enter
 * deep sleep mode. It also enables an I2C address match as a wake-up trigger from deep sleep.
 * hpi_sleep_allowed should have been called prior to calling this function.
 *
 * @return true if the HPI interface is ready for sleep, false otherwise.
 */
bool hpi_sleep (void);

/**
 * @brief Get the Port Enable register value.
 *
 * @return The Port Enable HPI register value.
 */
uint8_t hpi_get_port_enable (void);

/**
 * @brief Configure HPI to operate in No-boot support mode.
 * @param enable Whether to enable no-boot mode.
 * @return None
 */
void hpi_set_no_boot_mode (bool enable);

/**
 * @brief Set the I2C slave address to be used for the HPI interface.
 * @param slave_addr Slave address to be used.
 * @return None
 */
void hpi_set_fixed_slave_address (uint8_t slave_addr);

/**
 * @brief Send a FW ready notification through HPI to the EC. This event is sent
 * to the EC to indicate that the device is out of reset and has loaded firmware.
 *
 * @return None
 */
void hpi_send_fw_ready_event (void);

/**
 * @brief Set the CCG device flash parameters. These values are used for the
 * device status reporting and firmware update implementation.
 *
 * @param flash_size Total device flash size in bytes.
 * @param row_size Size of each flash row in bytes.
 * @param row_cnt Number of flash rows on the device.
 * @param bl_last_row Last flash row assigned to boot-loader.
 * @return None
 */
void hpi_set_flash_params (uint32_t flash_size, uint16_t row_size, uint16_t row_cnt, uint16_t bl_last_row);

#if (CCG_BOOT == 0)

/**
 * @brief This function initializes the user-defined HPI registers.
 *
 * This function is used to initialize the contents of the user-defined
 * registers that are part of the HPI register space.
 *
 * @param reg_addr The base address of registers to be updated. Should be in
 * the user defined address region.
 * @param size Number of registers to be updated. The upper limit should not
 * exceed the user defined address region.
 * @param data Buffer containing data to be copied into HPI registers.
 *
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t hpi_init_userdef_regs (uint16_t reg_addr, uint8_t size,
        uint8_t *data);

/**
 * @brief Enable handling of user-defined register writes in the Application.
 *
 * This function is used to enable handling of EC writes to the user-defined HPI
 * register region in the application code.
 *
 * @param wr_handler Pointer to function that handles the received HPI writes.
 */
void hpi_set_userdef_write_handler (hpi_write_cb_t wr_handler);

#endif

/**
 * @brief Store the reset counter value into the appropriate HPI register.
 *
 * @param count The reset count to be stored.
 * @return None
 */
void hpi_set_reset_count(uint8_t count);

/**
 * @brief Debug API to update registers at address 0x35 - 0x34
 * @param value 16-bit value to be updated into HPI registers 0x35 and 0x34.
 * @return None
 */
void hpi_set_reserved_reg_35(uint16_t value);

/**
 * @brief Debug API to update registers at address 0x37 - 0x36
 * @param value 16-bit value to be updated into HPI registers 0x37 and 0x36.
 * @return None
 */
void hpi_set_reserved_reg_37(uint16_t value);

/**
 * @brief Update the event mask value for the specified PD port.
 * @param port PD port index.
 * @param mask Event mask value to be set.
 * @return None
 */
void hpi_set_port_event_mask(uint8_t port, uint32_t mask);

/**
 * @brief Notify EC about a system hardware access error.
 * @param port PD port index.
 * @param err_type Type of error detected.
 * @return None
 */
void hpi_send_hw_error_event(uint8_t port, uint8_t err_type);

/**
 * @brief Get the content of the HPI system power state register.
 * @return The 8-bit unsigned content of the HPI syspwr_state register.
 */
uint8_t hpi_get_sys_pwr_state(void);

/**
 * @brief Update the firmware boot priority configuration reported through HPI.
 * @param conf Firmware boot priority supported by the firmware.
 * @return None
 */
void hpi_set_boot_priority_conf(uint8_t conf);

/**
 * @brief Enable/disable PDO update through HPI.
 *
 * @param disable Whether PDO update is to be disabled.
 * @return None
 */
void hpi_update_pdo_change(bool disable);

#if (CCG_HPI_BB_ENABLE != 0)

/**
 * @brief Set BB related register data.
 *
 * @param bb_reg_addr BB related register address.
 * @param data Pointer to data which writes to BB related register.
 * @return None.
 */
void hpi_bb_reg_update (uint8_t bb_reg_addr, void *data);

/**
 * @brief Get BB related register data.
 * @param bb_reg_addr BB related register address.
 * @return Data of selected BB related register.
 */
uint32_t hpi_bb_get_reg(uint8_t bb_reg_addr);

/**
 * @brief Enable the billboard event reporting for the specified PD port.
 * @param port PD port index.
 * @return None
 */
void hpi_bb_enable_events(uint8_t port);

/**
 * @brief Get the billboard firmware version.
 * @return Pointer to firmware version buffer.
 */
uint8_t *hpi_bb_get_version();

#endif /* (CCG_HPI_BB_ENABLE != 0) */

/**
 * @brief Get the HPI soft reset delay value
 * @return Delay value
 */
uint16_t get_hpi_soft_reset_delay(void);

/**
 * @brief Update the HPI soft reset delay value
 * @param value Delay value in ms
 * @return None
 */
void update_hpi_soft_reset_delay(uint16_t value);

/**
 * @brief Get the Timer ID related to HPI soft reset delay
 * @return Delay Timer ID
 */
uint8_t get_hpi_sof_reset_timer_id(void);

/**
 * @brief Update the Timer ID related to HPI soft reset delay
 * @param value Timer ID
 * @return None
 */
void update_hpi_sof_reset_timer_id(uint8_t value);

#if CCG_UCSI_ENABLE
typedef enum {
    I2C_OWNER_UCSI = 0,
    I2C_OWNER_HPI
} i2c_owner_t;

/*
 * @brief Update the Interrupt Status register and assert the EC_INT pin.
 * @param evt_code event code.
*/
void hpi_set_event (uint8_t evt_code);
/*
 * @brief Update the Interrupt Status register and assert the EC_INT pin.
*/
void hpi_clear_event (uint8_t evt_code);

/**
  @brief Clear status & control register.
 */
void ucsi_reg_reset();

/**
  @brief Clear appropriate bit_idx to 0 in the status register.
 */
void ucsi_clear_status_bit(uint8_t bit_idx);

/**
  @brief Set appropriate bit_idx to 0 in the status register.
 */
void ucsi_set_status_bit(uint8_t bit_idx);

/**
 * @brief Get the UCSI Control register value.
 *
 * @return The UCSI Control HPI register value.
 */
uint8_t hpi_get_ucsi_control();

/**
 * @brief Get the UCSI Status bit .
 *
 * @return The UCSI Status bit value.
 */
uint8_t ucsi_get_status_bit(uint8_t bit_idx);

#endif /* CCG_UCSI_ENABLE */

#endif /* _HPI_H_ */

/* [] END OF FILE */
