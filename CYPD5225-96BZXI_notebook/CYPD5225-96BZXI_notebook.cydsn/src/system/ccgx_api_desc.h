/**
 * @file ccgx_api_desc.h
 *
 * @brief @{CCGx Firmware API Reference Guide Introduction.@}
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

#ifndef _CCGX_API_DESC_H_
#define _CCGX_API_DESC_H_

/**
 * @mainpage CCGx Firmware Stack: API Reference Guide
 * @tableofcontents
 *
 * @section TypeCIntro Introduction
 *
 * USB Type-C is the new USB-IF standard that solves several challenges faced
 * when using today's Type-A and Type-B cables and connectors. USB Type-C uses
 * a slimmer connector (measuring only 2.4-mm in height) to allow for
 * increasing miniaturization of consumer and industrial products. The USB
 * Type-C standard is gaining rapid support by enabling small form-factor,
 * easy-to-use connectors and cables with the ability to transmit multiple
 * protocols and offer power delivery up to 100 W – a significant improvement
 * over the 7.5 W possible using previous standards.
 *
 * @subsection TypeCFeatures USB Type-C Highlights
 * - Brand new reversible connector, measuring only 2.4-mm in height.
 * - Compliant with USB Power Delivery 2.0, providing up to 100 W.
 * - Double the bandwidth of USB 3.0, increasing to 10 Gbps with SuperSpeedPlus
 *   USB3.1.
 * - Combines multiple protocols in a single cable, including DisplayPort™,
 *   PCIe® or Thunderbolt™.
 *
 * @section CCGIntro Cypress EZ-PD™ Type-C Controllers
 *
 * Cypress provides the EZ-PD™ family of USB Type-C controllers which can help
 * implement various Type-C applications. Cypress’s Type-C controllers are
 * based on Cypress’s PSoC® 4 programmable system-on-chip architecture, which
 * includes programmable analog and digital blocks, an ARM® Cortex®-M0 core and
 * 32 to 128 KB of flash memory.
 *
 * This product family is driving the industry’s first Type-C products with
 * top-tier PC makers, enabling them to bring these USB Type-C benefits to
 * market. Cypress has reference designs readily available for EMCA and dongle
 * applications. These are available online and could be used to speed-up our
 * customers design cycle.
 *
 * @subsection CCGFamily CCGx Product Families
 *
 * The EZ-PD CCGx product line consists of the following product families:
 * - EZ-PD™ CCG1: Industry’s First Programmable Type-C Port Controller
 * - EZ-PD™ CCG2: Industry’s Smallest Programmable Type-C Port Controller
 * - EZ-PD™ CCG3: Industry’s Most Integrated Type-C Port Controller
 * - EZ-PD™ CCG4: Industry’s First Dual-Port Type-C Port Controller
 * - EZ-PD™ CCG5: Dual-Port Type-C and PD Port Controller for PCs and Docks
 * - EZ-PD™ CCG3PA: Power Delivery Solution for Adapters and Chargers
 *
 * @section CCGFwStack CCGx Firmware Stack
 *
 * The CCGx product families are supported by a robust firmware stack which
 * includes:
 * -# USB Type-C and USB-PD specification compliant PD stack.
 * -# Drivers for the various hardware blocks on the CCGx controllers.
 * -# Implementation of a Host Processor Interface (HPI) that allows an
 *    external embedded controller to monitor and control the CCGx device
 *    operation.
 * -# Implementation of the DisplayPort Alternate Mode that allows transfer of
 *    video signals over the Type-C data lanes.
 *
 * The CCGx firmware stack is compliant to the:
 * - USB-PD Specification Revision 3.0
 * - Type-C Specification Revision 1.2
 *
 * This version of the CCGx Firmware Stack supports the CCG3, CCG4 and CCG5
 * families and is targeted at implementing USB-PD port controllers for desktops
 * and notebooks.
 *
 * Please refer to the CCGx Power SDK for stack and code examples for implementing
 * power adapters and chargers using the CCG3PA device family.  Please use the CCGx
 * SDK 3.0.2 version for stack and code examples for implementing Type-C video
 * adapters using the CCG3 device family.
 *
 * @subsection FwDirStruct Firmware Stack Organization
 *
 * The CCGx firmware stack is provided in source form with the SDK. The firmware
 * files are organized into the following directory structure:
 *
 * - system: The system folder contains header and source files relating to the
 *   CCGx device hardware and registers, boot-loader and flash access functions,
 *   low level drivers for the GPIO and USB-PD blocks on the CCGx device, and a
 *   soft timer implementation that is used by the firmware stack.
 *
 * - pd_hal: The pd_hal folder contains the Hardware Adaptation Layer (HAL) or
 *   low level hardware driver for the USB-PD hardware in the CCGx device.
 *   The driver functionality include PD block initialization, USB-PD message
 *   handling code, interrupt handling and ADC/Comparator configuration.
 *
 * - pd_common: The pd_common folder contains the core Type-C and USB-PD stack
 *   for the CCGx device. This includes the the Type-C port manager, the USB-PD
 *   protocol layer, the USB-PD policy engine and the Device Policy Manager.
 *   On devices that dual USB-PD ports, the stack allows both of the ports
 *   to function and be managed in a completely independent manner.
 *   The PD stack is provided in library form, and hence this folder will
 *   only contain the header files that define the stack interfaces.
 *
 * - scb: The scb folder contains the driver code for I2C slave mode operation
 *   using the Serial Controller Blocks (SCB) on the CCGx device. Since I2C
 *   slave mode is the most commonly used interface for CCGx, a specially
 *   optimized driver is provided for the same. The SCB driver is provided in
 *   library form, and the folder only contains the header files that define
 *   the SCB driver interface.
 *
 * - hpiss: The hpiss folder contains the implementation for the Host Processor
 *   Interface (HPI) which is an I2C based software protocol that allows an
 *   external Embedded Controller (EC) to monitor and control the CCGx device
 *   operation. HPI provides a register based interface through which the EC
 *   can manage the power contracts, send and receive VDMs and perform flash
 *   read/write operations. The HPI implementation is provided in library form
 *   and the folder only contains header files that define the interfaces.
 *   Please contact Cypress for more details about the HPI interface and
 *   its capabilities.
 *
 * - app: The app folder contains the top-level application layer functionality
 *   that implements the USB-PD controller functions required. This includes
 *   functionality such as PDO evaluation and contract negotiation, VDM handling
 *   for both DFP and UFP roles, handling of control messages like role swap;
 *   and alternate mode discovery and negotiation. The alternate mode specific
 *   implementation can be found in the app/alt_mode directory.
 *
 * @page FwStackDesc CCGx Firmware Architecture
 *
 * The CCGx firmware solution allows users to implement a variety of USB-PD
 * applications using the CCG devices and a fully tested firmware stack.
 *
 * The CCGx firmware solution contains the following components:
 *
 * - Hardware Abstraction Layer (HAL): This includes the low level drivers for the
 *   various hardware blocks on the CCG device. This includes drivers for the
 *   Type-C and USB-PD block, Serial Communication Blocks (SCBs), GPIOs, flash
 *   module and timer module.
 *
 * - USB Type-C and USB-PD Protocol Stack: This is the complete USB-PD protocol
 *   stack that includes the Type-C and USB-PD port managers, USB-PD protocol layer,
 *   the USB-PD policy engine and the device policy manager. The device policy
 *   manager is designed to allow all policy decisions to be made at the application
 *   level, either on an external Embedded Controller (EC) or in the CCG firmware
 *   itself.
 *
 * - Firmware update module: This is a firmware module that allows the device
 *   firmware maintained in internal flash to be updated. In Notebook PD port
 *   controller applications, the firmware update will be done from the EC side
 *   through an I2C interface.
 *
 * - Host Processor Interface (HPI): The Host Processor Interface (HPI) is an
 *   I2C based control interface that allows an Embedded Controller (EC) to
 *   monitor and control the USB-PD port on the CCG device. The HPI is the
 *   means to allow the PC platform to control the PD policy management.
 *
 * - Port Management: This module handles all of the PD port management functions
 *   including the algorithm for optimal contract negotiations, source and sink
 *   power control, source voltage selection, port role assignment and swap
 *   request handling.
 *
 * - Alternate Modes: This module implements the alternate mode handling for
 *   CCG as a DFP and UFP. A fully tested implementation of DisplayPort
 *   alternate mode with CCG as DFP is provided. The module also allows users
 *   to implement their own alternate mode support in both DFP and UFP modes.
 *
 * - Low Power: This module attempts to keep the CCG device in the low power
 *   standby mode as often as possible to minimize power consumption.
 *
 * - External Hardware Control: This is an hardware design dependent module which
 *   controls the external hardware blocks such as FETs, regulators and Type-C
 *   switches.
 *
 * - Solution specific tasks: This is an application layer module where any custom
 *   tasks required by the user solution can be implemented.
 *
 * A block diagram of the CCGx firmware architecture is shown below.
 * \image html ccgx_stack_blkdiag.jpg
 * \image latex ccgx_stack_blkdiag.eps "CCGx Firmware Stack Block Diagram"
 *
 * @section FwSolnStructure CCGx Firmware Solution Structure
 *
 * The CCGx firmware solution is broken down into multiple layers:
 *
 * - Hardware Adaptation Layer: The HAL is the lowest driver layer for
 *   the CCG device. The HAL code is provided in source form and can be
 *   located under the system folder in the CCGx firmware project.
 *
 * - USB-PD Stack: The USB-PD stack includes the Type-C and PD managers,
 *   the PD protocol, policy engine and device policy manager blocks. The
 *   USB-PD stack is provided in the form of a pre-compiled library.
 *
 * - Application Layer: The application layer includes the PD port
 *   management, HPI implementation, alternate modes and low power mode
 *   support. The application layer is provided in source form; and can
 *   be located under the app and alt_mode folders in the CCGx firmware
 *   project.
 *
 * - Solution Layer: The solution layer includes external hardware management,
 *   the main task loop and any user application specific functionality.
 *   A reference implementation is provided under the solution folder in the
 *   CCGx firmware project; and is expected to be customized by users.
 *
 * The CCGx firmware solution structure is shown below.
 * \image html ccgx_soln_structure.jpg
 * \image latex ccgx_soln_structure.eps "CCGx Firmware Solution Structure"
 *
 * @section PublicApi Public API Summary
 *
 * This document describes all of the data structures and functions that
 * are part of the CCGx firmware stack. Many of these functions are
 * intended to be used internally by the stack layers. The main public
 * interfaces that are expected to be used by user code are summarized
 * in this section.
 *
 * @subsection DpmApi Device Policy Manager (DPM) API
 *
 * The Device Policy Manager (DPM) APIs are the public interfaces to the
 * PD stack. These APIs include:
 * - dpm_init()
 * - dpm_start()
 * - dpm_stop()
 * - dpm_disable()
 * - dpm_deepsleep()
 * - dpm_wakeup()
 * - dpm_task()
 * - dpm_get_info()
 * - dpm_update_def_cable_cap()
 * - dpm_get_def_cable_cap()
 * - dpm_update_snk_wait_cap_period()
 * - dpm_get_snk_wait_cap_period()
 * - dpm_update_mux_enable_wait_period()
 * - dpm_get_mux_enable_wait_period()
 * - dpm_pd_command()
 * - dpm_typec_command()
 * - dpm_update_swap_response()
 * - dpm_update_src_cap()
 * - dpm_update_src_cap_mask()
 * - dpm_update_snk_cap()
 * - dpm_update_snk_cap_mask()
 * - dpm_update_snk_max_min()
 * - dpm_update_port_config()
 * - dpm_get_polarity()
 * - dpm_typec_deassert_rp_rd()
 * - dpm_update_port_status()
 * - dpm_get_pd_port_status()
 * - dpm_update_frs_enable()
 * - dpm_update_ext_src_cap()
 * - dpm_prot_reset()
 * - dpm_send_hard_reset()
 * - dpm_get_sink_detach_margin()
 * - dpm_get_sink_detach_voltage()
 *
 * @subsection HpiApi Host Processor Interface (HPI) API
 *
 * The Host Processor Interface (HPI) API are functions that initialize
 * the HPI register values and help perform message exchanges between
 * CCG and the Embedded Controller (EC).
 * - hpi_init()
 * - hpi_task()
 * - hpi_reg_enqueue_event()
 * - hpi_pd_event_handler()
 * - hpi_update_versions()
 * - hpi_set_mode_regs()
 * - hpi_update_fw_locations()
 * - hpi_sleep_allowed()
 * - hpi_sleep()
 * - hpi_get_port_enable()
 *
 * @subsection AppApi Application Layer API
 *
 * The application layer is where the policy management for the CCGx
 * application are implemented. The default implementations of these
 * functions are based on the configuration table and the parameters
 * provided by EC through the HPI registers.
 * - app_init()
 * - app_task()
 * - app_event_handler()
 * - app_get_status()
 * - app_sleep()
 * - app_wakeup()
 * - system_sleep()
 * - eval_src_cap()
 * - eval_rdo()
 * - psnk_set_voltage()
 * - psnk_set_current()
 * - psnk_enable()
 * - psnk_disable()
 * - psrc_set_voltage()
 * - psrc_set_current()
 * - psrc_enable()
 * - psrc_disable()
 * - vconn_enable()
 * - vconn_disable()
 * - vconn_is_present()
 * - vbus_is_present()
 * - vbus_discharge_on()
 * - vbus_discharge_off()
 * - app_ovp_enable()
 * - app_ovp_disable()
 * - eval_dr_swap()
 * - eval_vconn_swap()
 * - eval_pr_swap()
 * - vdm_data_init()
 * - vdm_update_data()
 * - eval_vdm()
 *
 * @subsubsection SolnCbs Solution Layer Functions
 *
 * The PD stack requires the user to provide a set of functions that
 * provide the stack configuration and also handle operations that are
 * hardware dependent. The following functions are expected to be
 * implemented at the solution layer in user code.
 * - mux_ctrl_init()
 * - mux_ctrl_set_cfg()
 * - sln_pd_event_handler()
 * - app_get_callback_ptr()
 *
 * @subsection AltModeApi Alternate Mode Manager API
 *
 * The following APIs are provided by the CCG Alternate Mode Manager
 * module. Some of these APIs are for use when CCG is a DFP, and the
 * others are for use when CCG is an UFP.
 * - enable_vdm_task_mngr()
 * - vdm_task_mngr_deinit()
 * - vdm_task_mngr()
 * - is_vdm_task_idle()
 * - eval_rec_vdm()
 *
 * @subsection HalApi Hardware Abstraction Layer (HAL) API
 *
 * This section documents the API provided as part of the Hardware
 * Adaptation Layer (HAL) which provides drivers for various hardware
 * blocks on the CCG device.
 *
 * @subsubsection GpioApi GPIO API
 *
 * - hsiom_set_config()
 * - gpio_set_drv_mode()
 * - gpio_hsiom_set_config()
 * - gpio_int_set_config()
 * - gpio_set_value()
 * - gpio_read_value()
 * - gpio_get_intr()
 * - gpio_clear_intr()
 *
 * @subsubsection I2cApi I2C Driver API
 *
 * - i2c_scb_init()
 * - i2c_scb_write()
 * - i2c_reset()
 * - i2c_slave_ack_ctrl()
 * - i2c_scb_is_idle()
 * - i2c_scb_enable_wakeup()
 *
 * @subsubsection FlashApi Flash Module API
 *
 * - flash_enter_mode()
 * - flash_access_get_status()
 * - flash_set_access_limits()
 * - flash_row_clear()
 * - flash_row_write()
 * - flash_row_read()
 *
 * @subsubsection TimerApi Soft Timer API
 *
 * - timer_init()
 * - timer_start()
 * - timer_stop()
 * - timer_is_running()
 * - timer_stop_all()
 * - timer_stop_range()
 * - timer_num_active()
 * - timer_enter_sleep()
 *
 * @subsection FwUpdApi Firmware Update API
 *
 * These APIs provide the basic boot and firmware upgrade related functions
 * that are used by various firmware update protocol implementations like
 * HPI and CC unstructured command handlers.
 *
 * - boot_validate_fw()
 * - boot_validate_configtable()
 * - get_boot_mode_reason()
 * - boot_get_boot_seq()
 * - sys_set_device_mode()
 * - sys_get_device_mode()
 *
 * Please see the detailed description of the APIs in the following
 * chapters. You can also refer to the CCGx SDK User Guide document
 * that provides some usage examples and guidelines.
 */

#endif /* _CCGX_API_DESC_H_ */

/* End of file */

