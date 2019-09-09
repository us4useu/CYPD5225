/**
 * @file dpm.h
 *
 * @brief @{Device Policy Manager (DPM) header file.@}
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

#ifndef _DPM_H_
#define _DPM_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include "pd.h"
#include "status.h"

/*******************************************************************************
 * Global Variable Declaration
 ******************************************************************************/
bool dpm_refresh_src_cap(uint8_t port);

bool dpm_refresh_snk_cap(uint8_t port);

/**
 * @brief Variable that holds the current port type (DFP/UFP)
 * for each CCG device PD port.
 */
extern port_type_t gl_dpm_port_type[NO_OF_TYPEC_PORTS];

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
/**
 * @brief This function initializes the device policy manager with callback
 * pointers and loads the system info from config table. This function also
 * initializes policy engine and type c manager.
 * This function must be called once on system init.
 * @param port Port index.
 * @param app_cbk Application callback function pointer.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_init(uint8_t port, app_cbk_t* app_cbk);

/**
 * @brief This function makes the specified port operational.
 * @param port Port index.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * if port index is not correct or dpm_init is not done, CCG_STAT_FAILURE if
 * port is disabled.
 */
ccg_status_t dpm_start(uint8_t port);

/**
 * @brief This function stops the port operation. The port will be put into the
 * lowest power state and and will not be operational.
 * @param port Port index.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_stop(uint8_t port);

/**
 * @brief This function disables PD port operation and limits it to receiving
 * hard reset signaling.
 * @param port Port index.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_disable(uint8_t port);

/**
 * @brief This function configures the device policy manager, for all ports, so
 * that the system can go to deepsleep.
 * This function does not put the system into deepsleep but only performs the
 * necessary tasks to enter deepsleep, if entry is possible.
 * @return Returns true if deepsleep is possible and configured, false otherwise.
 */
bool dpm_deepsleep(void);

/**
 * @brief This function configures the device policy manager, for all ports,
 * after the system comes out of deepsleep.
 * @return Returns true if successful, 0 otherwise.
 */
bool dpm_wakeup(void);

/**
 * @brief This function checks if the device policy manager can go into sleep
 * mode.
 * @return Returns true if possible to go into sleep mode, false otherwise.
 */
bool dpm_sleep(void);

/**
 * @brief This function runs the device policy manager task for the specified
 * port.
 * @param port Port index.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_task(uint8_t port);

/**
 * @brief This function returns device policy manager status information for
 * the specified port.
 * @param port Port index.
 * @return Pointer to a structure containing the DPM status information.
 * @warning The information provided by this API must not be altered by the
 * application.
 */
const dpm_status_t* dpm_get_info(uint8_t port);

/**
 * @brief This function updates the default cable current characteristics. The
 * parameter is used by the stack to determine the maximum current setting 
 * allowed when no e-marked cable is present. The default spec limit is 3A. It
 * should be overridden only for specific captive cable solution with a different
 * current capability. The function should be called only once before the DPM
 * is initialized and the parameter is used for all ports on the device.
 * If the function is not called, the default value of 3A is used.
 * @param def_cur Default current setting in 10mA unit (3A is represented as 300).
 * @warning The function should be called only when for a non-standard solution
 * and has the current capability.
 */
void dpm_update_def_cable_cap(uint16_t def_cur);

/**
 * @brief This function returns the default cable current characteristics for
 * the stack. The parameter is used by the stack to determine the maximum current
 * setting allowed when no e-marked cable is present.
 * @return Default current setting in 10mA unit (3A is represented as 300).
 */
uint16_t dpm_get_def_cable_cap(void);

/**
 * @brief Function to update the tTypeCSnkWaitCap period used by the CCG PD stack.
 * The default tTypeCSnkWaitCap value used by CCG PD stack is 400 ms which is the
 * mid-value of the spec defined range. However, this timer only starts when the
 * firmware has started running; which could be about 250 ms from device power up
 * in some cases. This function allows the user to update the tTypeCSnkWaitCap
 * period based on the worst case start-up delays for the application.
 *
 * @param period The tTypeCSnkWaitCap period to be used, in ms.
 * @return None
 */
void dpm_update_snk_wait_cap_period(uint16_t period);

/**
 * @brief Returns the current tTypeCSnkWaitCap value used by the CCG PD stack.
 * This function is used by the stack to get the default or user selected timer
 * period.
 * @return Current tTypeCSnkWaitCap timer period.
 */
uint16_t dpm_get_snk_wait_cap_period(void);

/**
 * @brief Function to specify the delay to be used between the APP_EVT_TYPEC_ATTACH
 * which is used to enable the Data Mux and the turning ON of the power source.
 * By default, the stack does not use any delay. In cases where the MUX used requires
 * time to properly turn ON, this function can be used to delay the VBus turn ON.
 *
 * @param period Delay to be provided (in ms) between MUX enable and VBus turning ON.
 *
 * @return None
 */
void dpm_update_mux_enable_wait_period(uint16_t period);

/**
 * @brief Returns the current delay between the MUX enable and VBus turn ON.
 *
 * @return Current delay in ms.
 */
uint16_t dpm_get_mux_enable_wait_period(void);

/**
 * @brief This function provides an interface for the application module to
 * send PD commands.
 * @param port Port index.
 * @param cmd Command name.
 * @param buf_ptr Pointer to the command buffer.
 * @param cmd_cbk Pointer to the callback function.
 * @return Returns CCG_STAT_SUCCESS if the command is registered,
 * CCG_STAT_CMD_FAILURE if the PD port is not ready for a command and
 * CCG_STAT_BUSY if there is another pending command.
 * @warning Data received via the callback should be copied out by the
 * application, because the buffer will be reused by the stack to store data on
 * new message reception.
 */
ccg_status_t dpm_pd_command(uint8_t port, dpm_pd_cmd_t cmd, dpm_pd_cmd_buf_t* buf_ptr,
                            dpm_pd_cmd_cbk_t cmd_cbk);

/**
 * @brief This function provides an interface for the HPI module to
 * send PD commands. This is only meant for HPI module wherein responses come from EC.
 * @param port Port index.
 * @param cmd Command name.
 * @param buf_ptr Pointer to the command buffer.
 * @param cmd_cbk Pointer to the callback function.
 * @return Returns CCG_STAT_SUCCESS if the command is registered,
 * CCG_STAT_CMD_FAILURE if the PD port is not ready for a command and
 * CCG_STAT_BUSY if there is another pending command.
 * @warning Data received via the callback should be copied out by the
 * application, because the buffer will be reused by the stack to store data on
 * new message reception.
 */
ccg_status_t dpm_pd_command_ec( uint8_t port, dpm_pd_cmd_t cmd,
                             dpm_pd_cmd_buf_t *buf_ptr, dpm_pd_cmd_cbk_t cmd_cbk);
/**
 * @brief This function provides an interface for the application module to control the Type C
 * interface.
 * @param port Port index.
 * @param cmd Command name.
 * @param cmd_cbk Pointer to the callback function.
 * @return Returns CCG_STAT_SUCCESS if the command is registered, CCG_STAT_BUSY
 * if a previous command is still active and CCG_STAT_CMD_FAILURE if the port
 * is in a disabled state.
 */
ccg_status_t dpm_typec_command(uint8_t port, dpm_typec_cmd_t cmd,
                               dpm_typec_cmd_cbk_t cmd_cbk);

/**
 * @brief Update the SWAP_RESPONSE setting for the device policy manager.
 * @param port Port index.
 * @param value New value for swap response.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_update_swap_response(uint8_t port, uint8_t value);

/**
 * @brief This function updates the source PDOs at runtime thereby overriding
 * the source PDOs in the config table.
 * @param port Port index.
 * @param count Count of PDOs.
 * @param pdo Pointer to the PDO array.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_update_src_cap(uint8_t port, uint8_t count, pd_do_t* pdo);

/**
 * @brief This function updates the source PDO mask at runtime thereby
 * overriding the source PDO mask in the config table.
 * @param port Port index.
 * @param mask PDO mask.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_update_src_cap_mask(uint8_t port, uint8_t mask);

/**
 * @brief This function updates the sink PDOs at runtime thereby overriding the
 * sink PDOs in the config table.
 * @param port Port index.
 * @param count Count of PDOs.
 * @param pdo Pointer to the PDO array.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_update_snk_cap(uint8_t port, uint8_t count, pd_do_t* pdo);

/**
 * @brief This function updates the sink PDO mask at runtime thereby
 * overriding the sink PDO mask specified in the config table.
 * @param port Port index.
 * @param mask PDO mask.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_update_snk_cap_mask(uint8_t port, uint8_t mask);

/**
 * @brief This function updates the sink max/min current/power at runtime
 * thereby overriding the sink max/min current/power specified in the
 * config table.
 * @param port Port index.
 * @param count Count of PDOs.
 * @param max_min Pointer to max/min cur/power array.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_update_snk_max_min(uint8_t port, uint8_t count, uint16_t* max_min);

/**
 * @brief Change the PD port configuration at runtime.
 *
 * This function allows changing the PD port configuration parameters like port
 * role, default port role, DRP toggle enable and Try.Src enable at runtime.
 * These changes are only allowed while the corresponding PD port is disabled.
 *
 * @param port USB-PD port to be configured.
 * @param role New port role selection (0 = Sink, 1 = Source, 2 = Dual Role).
 * @param dflt_role New default port role selection (0 = Sink, 1 = Source).
 * @param toggle_en New value for DRP toggle enable flag.
 * @param try_src_snk_en New value for Try.SRC/ TRY.SNK enable flag( 0 =
 * Both Try.SRC and TRY.SNK are disabled, 1 = Try.SRC is enabled,
 * 2 = TRY.SNK is enabled).
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * if port index is not correct or other parameters are not correct,
 * CCG_STAT_FAILURE if port is not disabled.
 */
ccg_status_t dpm_update_port_config(uint8_t port, uint8_t role, uint8_t dflt_role,
                               uint8_t toggle_en, uint8_t try_src_snk_en);

/**
 * @brief This generic function is provided by the device policy manager to
 * evaluate any RDO with respect to current source cap of the specified port.
 * @param port Port index.
 * @param rdo Request data object.
 * @return CCG_STAT_SUCCESS if rdo is valid, CCG_STAT_BAD_PARAM
 * if port index is not correct, CCG_STAT_FAILURE if rdo is not valid.
 */
ccg_status_t dpm_is_rdo_valid(uint8_t port, pd_do_t rdo);

/**
 * @brief Get the CC polarity of the Type-C connection.
 * @param port Port index.
 * @return Returns 0 if CC1 is used, and 1 if CC2 is used.
 */
uint8_t dpm_get_polarity(uint8_t port);

/**
 * @brief This function removes Rp and Rd from the specified CC channel.
 * @param port Port index.
 * @param channel CC channel.
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
ccg_status_t dpm_typec_deassert_rp_rd(uint8_t port, uint8_t channel);

/**
 * @brief Updates the PD port status.
 * @param port PD port to be updated.
 * @param input Present input status.
 * @param battery Present battery status.
 * @return void
 */
void dpm_update_port_status(uint8_t port, uint8_t input, uint8_t battery);

/**
 * @brief Get the PD port status.
 * @param port PD port to be queried.
 * @return 32-bit PD port status value to be reported through HPI.
 */
uint32_t dpm_get_pd_port_status(uint8_t port);

/**
 * @brief Enable/disable the PD 3.0 FRS functionality.
 * @param port PD port to be updated.
 * @param frs_rx_en Whether FRS receive is to be enabled.
 * @param frs_tx_en Whether FRS transmit is to be enabled.
 * @return None
 */
void dpm_update_frs_enable(uint8_t port, bool frs_rx_en, bool frs_tx_en);

/**
 * @brief Update the extended source capabilities for the PD port.
 * @param port PD port to be updated.
 * @param buf_p Pointer to buffer containing extended source capabilities data.
 * @return None
 */
void dpm_update_ext_src_cap(uint8_t port, uint8_t *buf_p);

/**
 * @brief This function resets protocol layer(RX and TX) counter for a specific sop type.
 * @param port port index
 * @param sop sop type
 * @return ccg_status_t
 */
ccg_status_t dpm_prot_reset(uint8_t port, sop_t sop);

/**
 * @brief This function resets protocol layer RX only counter for a specific sop type.
 * @param port port index
 * @param sop sop type
 * @return ccg_status_t
 */
ccg_status_t dpm_prot_reset_rx(uint8_t port, sop_t sop);

/**
 * @brief This function retrieve the event number from event bitmask and also
 * clears the original event variable. PE and Type C state machines use bitmask
 * event variable.
 * @max_events Maximum no of events possible.
 * @evt_ptr Pointer to the event variable
 * @return Returns event number
 * @warning This function should not be called if event is equal to zero.
 * This is an internal function. Not required for normal usecase.
 */
uint8_t dpm_get_evtno_n_clear(uint8_t max_events, volatile uint32_t* evt_ptr);

/**
 * @brief This function stops the policy engine. Used in fault scenario wherein
 * PD protocol need to be stopped but type c manager still runs.
 * @param port port index
 * @return ccg_status_t
 */
ccg_status_t dpm_pe_stop(uint8_t port);

/**
 * @brief This function sets alert ADO on ocp/ovp fault. Stack will automatically
 * send alert after explicit contract when alert ADO is non zero. Once alert is sent
 * or detach happens alert ADO will be cleared automatically by stack.
 * @param port port index
 * @param alert_ado Alert Augmented Data Object.
 * @return ccg_status_t
 */
ccg_status_t dpm_set_alert(uint8_t port, pd_do_t alert_ado);

/**
 * @brief This function sets/clears current foldback status. When in current foldback
 * mode this function can be used to convey the status to stack
 * @param port Port index
 * @param status CF status
 * @return ccg_status_t
 */
ccg_status_t dpm_set_cf(uint8_t port, bool status);
/**
 * @brief This function clears hard reset count. This is specifically provided
 * for VBUS fault handlers.
 * @param port Port index
 * @return ccg_status_t
 */
ccg_status_t dpm_clear_hard_reset_count(uint8_t port);

/**
 * @brief This function set internal flag if any fault is active ocp/ovp/otp etc
 * @param port Port index
 * @return ccg_status_t
 */
ccg_status_t dpm_set_fault_active(uint8_t port);

/**
 * @brief This function clears internal fault flag
 * @param port Port index
 * @return ccg_status_t
 */
ccg_status_t dpm_clear_fault_active(uint8_t port);

/**
 * @brief This function sets chunk transfer type. This should be called
 * after response of a chunk is received to inform stack that chunked transfer 
 * is not over yet.
 * @param port Port index
 * @return ccg_status_t
 */
ccg_status_t dpm_set_chunk_transfer_running(uint8_t port, pd_ams_type ams_type);

/**
 * @brief Try to send a HardReset to the port partner.
 * @param port PD port index.
 * @param reason Reason for the hard reset.
 * @return ccg_status_t
 */
ccg_status_t dpm_send_hard_reset(uint8_t port, uint8_t reason);

/**
 * @brief This function returns the margin on the VBUS below which sink can
 * assume detach has happened.
 * @param port Port index.
 * @return % margin
 */
int8_t dpm_get_sink_detach_margin(uint8_t port);

/**
 * @brief This function returns the nominal VBUS voltage threshold for which sink
 * will detect a disconnect. Note: Sink margin will also applied on this nominal
 * value to determine actual threshold.
 * @param port Port index.
 * @return VBUS voltage in mV units
 */
uint16_t dpm_get_sink_detach_voltage(uint8_t port);

/**
 * @brief This function monitors the PPS activity. This function needs to be called
 * periodically when in PPS mode of operation. Since the VBUS voltage is expected
 * to have stabilized when invoking this function, this is not handled internally
 * and is expected to be triggered from the application layer. The recommended
 * periodicity for this is 100ms.
 */
void dpm_pps_task(uint8_t port);

#endif /* _DPM_H_ */

/* End of file */

