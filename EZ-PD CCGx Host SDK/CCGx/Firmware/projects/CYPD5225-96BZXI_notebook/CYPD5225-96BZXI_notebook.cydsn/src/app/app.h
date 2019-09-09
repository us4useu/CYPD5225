/**
 * @file app.h
 *
 * @brief @{PD application handler header file.@}
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

#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include <pd.h>
#include <pdss_hal.h>
#include <alt_mode_hw.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/* Note: Application Timers Must have timer id  >=30 and < 128 */

/* Note: Psource timers IDs must not be changed and must be consecutive */

/**
   @brief Power source enable timer ID.
 */
#define APP_PSOURCE_EN_TIMER                            (APP_TIMERS_START_ID)
/**
   @brief Power source monitor enable timer ID.
 */
#define APP_PSOURCE_EN_MONITOR_TIMER                    (31u)
/**
   @brief Power source enable hysteresis timer ID.
 */
#define APP_PSOURCE_EN_HYS_TIMER                        (32u)
/**
   @brief Power source disable timer ID.
 */
#define APP_PSOURCE_DIS_TIMER                           (33u)
/**
   @brief Power source disable monitor timer ID.
 */
#define APP_PSOURCE_DIS_MONITOR_TIMER                   (34u)
/**
   @brief Power source Current foldback restart timer ID.
 */
#define APP_PSOURCE_CF_TIMER                            (35u)
#define APP_PSOURCE_CF_TIMER_PERIOD                     (100u)
/**
   @brief Power source disable extra discharge timer ID.
 */
#define APP_PSOURCE_DIS_EXT_DIS_TIMER                   (36u)

/**
   @brief Duration of extra VBus discharge after voltage drops below desired level.
 */
#define APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD            (10u)

/**
   @brief Dead battery Sink Fet disable delay timer
 */
#define APP_DB_SNK_FET_DIS_DELAY_TIMER                  (37u)

/**
   @brief Power sink discharge disable timer ID.
 */
#define APP_PSINK_DIS_TIMER                             (38u)
#define APP_PSINK_DIS_TIMER_PERIOD                      (250u)
#define APP_PSINK_DIS_VBUS_IN_DIS_PERIOD                (20u)

/**
   @brief Power sink disable monitor timer ID.
 */
#define APP_PSINK_DIS_MONITOR_TIMER                     (39u)
#define APP_PSINK_DIS_MONITOR_TIMER_PERIOD              (1u)

/**
   @brief VDM busy timer ID.
 */
#define APP_VDM_BUSY_TIMER                              (40u)
/**
   @brief tAME timer ID.
 */
#define APP_AME_TIMEOUT_TIMER                           (41u)
/**
    @brief VBUS OCP OFF Timer ID.
 */
#define APP_VBUS_OCP_OFF_TIMER                          (42u)

/**
    @brief VBUS OCP OFF Timer ID.
 */
#define APP_VBUS_OVP_OFF_TIMER                          (43u)

/**
    @brief VBUS OCP OFF Timer ID.
 */
#define APP_VBUS_UVP_OFF_TIMER                          (44u)

/**
    @brief VBUS SCP OFF Timer ID.
 */
#define APP_VBUS_SCP_OFF_TIMER                          (45u)

/**
   @brief App timer used to delay port enable after fault.
 */
#define APP_FAULT_RECOVERY_TIMER                        (46u)
#define APP_FAULT_RECOVERY_TIMER_PERIOD                 (100u)
#define APP_FAULT_RECOVERY_MAX_WAIT                     (500u)

/**
   @brief Timer used to do delayed SBU connection.
 */
#define APP_SBU_DELAYED_CONNECT_TIMER                   (47u)
#define APP_SBU_DELAYED_CONNECT_PERIOD                  (25u)
#define APP_SBU_TBT_CONNECT_DELAY                       (50u)

/**
   @brief Timer used to delay VDM response.
 */
#define APP_MUX_DELAY_TIMER                             (48u)
/**
   @brief Timer used to MUX status.
 */
#define APP_MUX_POLL_TIMER                              (49u)
/**
   @brief VDM busy timer period (in ms).
 */
#define APP_VDM_BUSY_TIMER_PERIOD                       (50u)
/**
   @brief VDM retry (on failure) timer period in ms.
 */
#define APP_VDM_FAIL_RETRY_PERIOD                       (100u)
/**
   @brief Time allowed for cable power up to be complete.
 */
#define APP_CABLE_POWER_UP_DELAY                        (55u)
/**
   @brief Cable query delay period in ms.
 */
#define APP_CABLE_VDM_START_DELAY                       (5u)
/**
   @brief tAME timer period (in ms).
 */
#define APP_AME_TIMEOUT_TIMER_PERIOD                    (800u)
/**
   @brief Dead battery Sink Fet disable delay timer period.
 */
#define APP_DB_SNK_FET_DIS_DELAY_TIMER_PERIOD           (50u)

/**
 * @brief Billboard enumeration ON delay timer. This timer is used to delay
 * back to back turn on. This allows USB host to detect disconnection correctly.
 * This timer is used only for devices with internal USB block (CCG3).
 */
#define APP_BB_ON_TIMER                                 (60u)
/**
 * @brief Billboard ON delay timer period. If billboard needs to re-started,
 * delay start by specified time to ensure that the host can detect the device
 * going away correctly.
 */
#define APP_BB_ON_TIMER_PERIOD                          (250u)
/**
 * @brief Billboard OFF delay timer. This timer is used to remove the billboard
 * enumeration to save power. The USB block shall be disabled after the a
 * specified timeout. The timeout value is expected to be specified via the
 * configuration table.
 */
#define APP_BB_OFF_TIMER                                (61u)
    

/**
   @brief Timer for initiating IRQ CLEAR ACK.
 */
#define APP_INITIATE_SEND_IRQ_CLEAR_ACK                 (64u)
/**
   @brief Timer period for initiating IRQ CLEAR ACK.
 */
#define APP_INITIATE_SEND_IRQ_CLEAR_ACK_PERIOD          (1u)
    
#if CCG_UCSI_ENABLE    
/* UCSI Connect Event timer. This timer is used to Signal Connect event to OS */
#define UCSI_CONNECT_EVENT_TIMER                        (65u)
#define UCSI_CONNECT_EVENT_PERIOD                       (500u)  
#endif /*CCG_UCSI_ENABLE*/

/* Battery Charging timers Start from 70, End at 80 */
#define APP_BC_GENERIC_TIMER1                           (70u)
#define APP_BC_GENERIC_TIMER2                           (71u)

/* Debounce timer */
#define APP_BC_DP_DM_DEBOUNCE_TIMER                     (72u)

/* VDM Not supported response timer. */
#define APP_VDM_NOT_SUPPORT_RESP_TIMER_ID               (73u)
#define APP_VDM_NOT_SUPPORT_RESP_TIMER_PERIOD           (5)

/* CCG activity timer ID. */
#define CCG_ACTIVITY_TIMER_ID                           (81u)

/* CCG activity timer period in ms. */
#define CCG_ACTIVITY_TIMER_PERIOD                       (500)

/* OTP debounce timer ID. */
#define OTP_DEBOUNCE_TIMER_ID                           (82u)

/* OTP debounce timer period in ms. */
#define OTP_DEBOUNCE_PERIOD                             (1)

/* TYPE-A current sense timer ID. */
#define TYPE_A_CUR_SENSE_TIMER_ID                       (83u)

/* TYPE-A current sense timer period. */
#define TYPE_A_CUR_SENSE_TIMER_PERIOD                   (30000)

/* TYPE-A regulator switch timer ID. */
#define TYPE_A_REG_SWITCH_TIMER_ID                      (84u)

/* TYPE-A regulator switch timer period in ms. */
#define TYPE_A_REG_SWITCH_TIMER_PERIOD                  (10)
    
/* TYPE-A port PWM step change timer. */
#define TYPE_A_PWM_STEP_TIMER_ID                        (85u)

/* TYPE-A port PWM step change time in ms. */
#define TYPE_A_PWM_STEP_TIMER_PERIOD                    (1)
    
/* PB debounce timer ID. */
#define PB_DEBOUNCE_TIMER_ID                            (86u)

/* PB debounce timer period in ms. */
#define PB_DEBOUNCE_PERIOD                              (1)

/* PB debounce period in ms. */
#define APP_PB_VBATT_DEBOUNCE_IN_MS                     (10)
    
/* Power source VBUS set timer ID */
#define APP_PSOURCE_VBUS_SET_TIMER                      (87u)
    
/* Power source VBUS set timer period in ms. */
#define APP_PSOURCE_VBUS_SET_TIMER_PERIOD               (1)

/* VBUS OFF time to do a VBUS power cycle. */
#define APP_BC_VBUS_CYCLE_TIMER_PERIOD                  (200u)
/* Sink DCD stable time period. */
#define APP_BC_SINK_CONTACT_STABLE_TIMER_PERIOD         (50u)
/* 
 * Tglitch_done time waiting for the portable device to complete detection.
 * This is used by QC/AFC devices to proceed with subsequent detection.
 */
#define APP_BC_DCP_DETECT_TIMER_PERIOD                  (1100u)
/*
 * Debounce time to verify if the attached device is Apple or not. Apple device
 * creates a glitch on DP line whereas BC device continues to drive DP lower.
 * This period is used when Apple and BC 1.2 source protocols are supported
 * together.
 */
#define APP_BC_APPLE_DETECT_TIMER_PERIOD                (5u)
/* Debounce time for identifying state change for DP and DM lines. */
#define APP_BC_DP_DM_DEBOUNCE_TIMER_PERIOD              (40u)
/* AFC detection time. */
#define APP_BC_AFC_DETECT_TIMER_PERIOD                  (100u)

/*
 *@brief TGLITCH_BC_DONE timer period. This timer is used in sink mode to detect
 * QC charger.
 */
#define APP_BC_GLITCH_BC_DONE_TIMER_PERIOD              (1500)

/*
 * @brief T_GLITCH_DM_HIGH timer period. After DCP opens D+/- short, Sink shall
 * wait for this time before requesting VBUS.
 */
#define APP_BC_GLITCH_DM_HIGH_TIMER_PERIOD              (40)

/*
 * @brief T_V_NEW_REQUEST timer period. After entering QC mode, sink must wait
 * this much time before requesting next voltage.
 */
#define APP_BC_V_NEW_REQUEST_TIMER_PERIOD               (200)

/*
 * @brief Minimum OVP detection voltage when ADC is used to implement OVP.
 */
#define ADC_VBUS_MIN_OVP_LEVEL                          (6500u)

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/

/**
 * @brief This type of the function is used by app layer to call MUX polling
 * function.
 *
 * @param port Port index the function is performed for.
 *
 * @return MUX polling status
 */
typedef mux_poll_status_t
(*mux_poll_fnc_cbk_t) (
        uint8_t         port);

/*
 * @brief Fault status bit that indicates VConn fault (OCP) active.
 */
enum {
    APP_PORT_FAULT_NONE                 = 0x00,
    APP_PORT_VCONN_FAULT_ACTIVE         = 0x01, /**< Status bit that indicates VConn fault is active. */
    APP_PORT_SINK_FAULT_ACTIVE          = 0x02, /**< Status bit that indicates sink fault handling is pending. */
    APP_PORT_SRC_FAULT_ACTIVE           = 0x04, /**< Status bit that indicates source fault handling is pending. */
    APP_PORT_VBUS_DROP_WAIT_ACTIVE      = 0x08, /**< Status bit that indicates wait for VBus drop is pending. */
    APP_PORT_DISABLE_IN_PROGRESS        = 0x80  /**< Port disable operation is in progress. */
};

/**
   @struct app_status_t
   @brief This structure hold all variables related to application layer functionality.
 */
typedef struct
{
    pwr_ready_cbk_t pwr_ready_cbk;        /**< Registered Power source callback. */
    sink_discharge_off_cbk_t snk_dis_cbk; /**< Registered Power sink callback. */
    app_resp_t app_resp;                  /**< Buffer for APP responses. */
    vdm_resp_t vdm_resp;                  /**< Buffer for VDM responses. */
    uint16_t psrc_volt;                   /**< Current Psource voltage in mV */
    uint16_t psrc_volt_old;               /**< Old Psource voltage in mV */
    uint16_t psnk_volt;                   /**< Current PSink voltage in mV units. */
    uint16_t psnk_cur;                    /**< Current PSink current in 10mA units. */
    uint8_t vdm_task_en;                  /**< Flag to indicate is vdm task manager enabled. */
    uint8_t cbl_disc_id_finished;         /**< Flag to indicate that cable disc id finished. */
    uint8_t vdm_version;                  /**< Live VDM version. */
    uint8_t alt_mode_trig_mask;           /**< Mask to indicate which alt mode should be enabled by EC. */
    volatile uint8_t fault_status;        /**< Fault status bits for this port. */
    bool alt_mode_entered;                /**< Flag to indicate is alternate modes currently entered. */
    bool vdm_prcs_failed;                 /**< Flag to indicate is vdm process failed. */
    bool is_vbus_on;                      /**< Is supplying VBUS flag. */
    bool is_vconn_on;                     /**< Is supplying VCONN flag. */
    bool vdm_retry_pending;               /**< Whether VDM retry on timeout is pending. */
    bool psrc_rising;                     /**< Voltage ramp up/down. */
    bool cur_fb_enabled;                  /**< Indicates that current foldback is enabled */
    bool ld_sw_ctrl;                      /**< Indicates whether the VBUS load switch control is active or not. */
    bool bc_12_src_disabled;              /**< BC 1.2 source disabled flag. */

    bool is_mux_busy;                     /**< Flag to indicate that mux is switching. */
    vdm_resp_cbk_t vdm_resp_cbk;          /**< VDM response handler callback. */
    bool is_vdm_pending;                  /**< VDM handling flag for MUX callback. */

    mux_poll_fnc_cbk_t mux_poll_cbk;      /**< Holds pointer to MUX polling function. */
}app_status_t;

/**
 * @typedef sys_hw_error_type_t
 * @brief List of possible hardware errors defined for the system.
 */
typedef enum {
    SYS_HW_ERROR_NONE        = 0x00,            /**< No error. */
    SYS_HW_ERROR_MUX_ACCESS  = 0x01,            /**< Error while accessing data MUX. */
    SYS_HW_ERROR_REG_ACCESS  = 0x02,            /**< Error while accessing regulator. */
    SYS_HW_ERROR_BAD_VOLTAGE = 0x04             /**< Unexpected voltage generated by source regulator. */
} sys_hw_error_type_t;

/*
 * @brief List of possible Thermistor type that can be configured.
 */
enum {
    APP_THERMISTOR_TYPE_NTC       = 0x00,            /**< NTC Thermistor type configured. */
    APP_THERMISTOR_TYPE_PTC       = 0x01,            /**< PTC Thermistor type configured. */
    APP_THERMISTOR_TYPE_ERROR     = 0x02             /**< Invalid Thermistor type configured. */
};

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/

/**
 * @brief Application level init function.
 *
 * This function performs any Application level initialization required
 * for the CCG solution. This should be called before calling the
 * dpm_init function.
 *
 * @return None.
 *
 */
void app_init(void);

/**
 * @brief Solution handler for PD events reported from the stack.
 *
 * The function provides all PD events to the solution. For a solution
 * supporting HPI, the solution function should re-direct the calls to
 * hpi_pd_event_handler. If no HPI is supported, the function can be a simple
 * dummy function.
 *
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that
 * needs to be de-referenced based on event type.
 *
 * @return None
 */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

/**
 * @brief Handler for application level asynchronous tasks.
 * @param port USB-PD port for which tasks are to be handled.
 * @return 1 in case of success, 0 in case of task handling error.
 */
uint8_t app_task(uint8_t port);

/**
 * @brief This function return the App callback structure pointer
 * @param port port index
 * @return  Application callback structure pointer
 */
app_cbk_t* app_get_callback_ptr(uint8_t port);

/**
 * @brief Handler for event notifications from the PD stack.
 * @param port Port on which events are to be handled.
 * @param evt Type of event to be handled.
 * @param dat Data associated with the event.
 * @return None
 */
void app_event_handler (uint8_t port, app_evt_t evt, const void* dat);

/**
 * @brief Get a handle to the application provide PD command response buffer.
 * @param port PD port corresponding to the command and response.
 * @return Pointer to the response buffer.
 */
app_resp_t* app_get_resp_buf(uint8_t port);

/**
 * @brief Get handle to structure containing information about the system status for a PD port.
 * @param port PD port to be queried.
 * @return Pointer to the system information structure.
 */
app_status_t* app_get_status(uint8_t port);

/**
 * @brief Check whether the APP handlers are ready to allow device deep sleep.
 * @return true if APP handler is idle, false otherwise.
 */
bool app_sleep (void);

/**
 * @brief Restore the APP handler state after CCG device wakes from deep-sleep.
 * @return None
 */
void app_wakeup (void);

/**
 * @brief Function to place CCG device in power saving mode if possible.
 *
 * This function places the CCG device in power saving deep sleep mode
 * if possible. The function checks for each interface (PD, HPI etc.)
 * being idle and then enters sleep mode with the appropriate wake-up
 * triggers. If the device enters sleep mode, the function will only
 * return after the device has woken up.
 *
 * @return true if the device went into sleep, false otherwise.
 */
bool system_sleep(void);

/*****************************************************************************
  Functions related to power
 *****************************************************************************/

/**
 * @brief This function enables VCONN power
 *
 * @param port Port index the function is performed for.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_enable(uint8_t port, uint8_t channel);

/**
 * @brief This function disables VCONN power
 *
 * @param port Port index the function is performed for.
 * @param channel Selected CC line.
 *
 * @return None
 */
void vconn_disable(uint8_t port, uint8_t channel);

/**
 * @brief This function checks if power is present on VConn
 *
 * @param port Port index the function is performed for.
 *
 * @return true if power is present on VConn, else returns false
 */
bool vconn_is_present(uint8_t port);

/**
 * @brief This function checks if power is present on VBus
 *
 * @param port Port index the function is performed for.
 * @param volt Voltage in mV units.
 * @param per  Threshold margin.
 *
 * @return true if power is present on VBus, else returns false
 */
bool vbus_is_present(uint8_t port, uint16_t volt, int8_t per);

/**
 * @brief This function return current VBUS voltage in mV
 *
 * @param port Port index the function is performed for.
 *
 * @return VBUS voltage in mV
 */
uint16_t vbus_get_value(uint8_t port);
/**
 * @brief This function turns on dischange FET on selected port
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void vbus_discharge_on(uint8_t port);

/**
 * @brief This function turns off dischange FET on selected port
 *
 * @param port Port index the function is performed for.
 *
 * @return None
 */
void vbus_discharge_off(uint8_t port);

/**
 * @brief This function enable vconn ocp
 * @param port Port index
 * @param cbk OCP callback
 * @return Returns true on success, false if parameters are invalid.
 */
bool system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk);

/**
 * @brief This function disable vconn ocp
 * @param port Port index
 * @return None
 */
void system_vconn_ocp_dis(uint8_t port);

/**
 * @brief Enable and configure the Over-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param ovp_cb Callback function to be triggered when there is an OV event.
 * @return None
 */
void app_ovp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T ovp_cb);

/**
 * @brief Disable the Over-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_ovp_disable(uint8_t port, bool pfet);

/**
 * @brief Enable and configure the Under-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param volt_mV Expected VBus voltage.
 * @param pfet Whether PFET is used for the power supply control.
 * @param uvp_cb Callback function to be triggered when there is an UV event.
 * @return None
 */
void app_uvp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T uvp_cb);

/**
 * @brief Disable the Under-Voltage protection circuitry.
 * @param port PD port to be configured.
 * @param pfet Whether PFET is used for the power supply control.
 * @return None
 */
void app_uvp_disable(uint8_t port, bool pfet);

/**
 * @brief Configures TYPE-C port for fault protection state.
 *
 * @param port PD port to be configured.

 * @return None
 */
void app_conf_for_faulty_dev_removal(uint8_t port);

/**
 * @brief Function to update the BC 1.2 source support.
 * @param port PD port to be configured.
 * @return None
 */
void app_update_bc_src_support(uint8_t port, uint8_t enable);

/**
 * @brief Function to update the system power state.
 * @param state Current system power state: 0=S0, None-zero=other states.
 */
void app_update_sys_pwr_state(uint8_t state);

/**
 * @brief Wrapper function for PD port disable. This function is used to ensure that
 * any application level state associated with a faulty connection are cleared when the
 * user wants to disable a PD port.
 * @param port Index of port to be disabled.
 * @param cbk Callback to be called after operation is complete.
 * @return CCG_STAT_SUCCESS on success, appropriate error code otherwise.
 */
ccg_status_t app_disable_pd_port(uint8_t port, dpm_typec_cmd_cbk_t cbk);

/**
 * @brief Validate the configuration table specified.
 *
 * Each copy of CCGx firmware on the device flash contains an embedded
 * configuration table that defines the runtime behaviour of the CCGx device. This
 * function checks whether the configuration table located at the specified location
 * is valid (has valid offsets).
 *
 * @param null.
 *
 * @return true if the table is valid, false otherwise.
 */
bool app_validate_configtable_offsets(void);

/*****************************************************************************
  Functions to be provided at the solution level.
 *****************************************************************************/

/**
 * @brief Initialize the Type-C Data Mux for a specific PD port.
 *
 * @param port USB-PD port for which the MUX is to be initialized.
 * @return Returns true if the MUX is initialized successfully, false otherwise.
 */
bool mux_ctrl_init(uint8_t port);

/**
 * @brief Set the Type-C MUX to the desired configuration.
 * @param port PD port on which MUX is to be configured.
 * @param cfg Desired MUX configuration.
 * @param polarity Polarity of the Type-C connection.
 * @return Returns true if the operation is successful, false otherwise.
 */
bool mux_ctrl_set_cfg(uint8_t port, mux_select_t cfg, uint8_t polarity);

/*****************************************************************************
  Functions to be used by CCGx periodic tasks.
 *****************************************************************************/
/**
 * @brief Initialize CCGx periodic app level tasks.
 *
 * @param Null
 * @return Null
 */
void ccg_app_task_init(void);

#endif /* _APP_H_ */

/* End of File */
