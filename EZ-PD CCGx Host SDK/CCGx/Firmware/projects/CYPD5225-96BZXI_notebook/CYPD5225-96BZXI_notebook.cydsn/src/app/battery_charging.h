/**
 * @file battery_charging.h
 *
 * @brief @{Battery Charging header file.@}
 *
 *******************************************************************************
 * @copyright
 *
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
#ifndef _BATTERY_CHARGING_H_
#define _BATTERY_CHARGING_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <config.h>
#include <pd.h>
#include <status.h>
#include <chgb_hal.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/* Combined mask for all legacy battery charging protocols in source mode. */
#define BATT_CHARGING_SRC_MASK  (BC_SRC_APPLE_MODE_ENABLE_MASK | \
        BC_SRC_1_2_MODE_ENABLE_MASK | BC_SRC_QC_MODE_ENABLE_MASK | \
        BC_SRC_AFC_MODE_ENABLE_MASK)

/* Combined mask for all legacy battery charging protocols in sink mode. */
#define BATT_CHARGING_SINK_MASK (BC_SINK_1_2_MODE_ENABLE_MASK | \
        BC_SINK_APPLE_MODE_ENABLE_MASK)

/* QC Dp/Dm macros*/
#define QC_MODE_12V                                 (((BC_D_0_6V) << 8)|BC_D_0_6V)
#define QC_MODE_9V                                  (((BC_D_0_6V) << 8)|BC_D_3_3V)
#define QC_MODE_CONT                                (((BC_D_3_3V) << 8)|BC_D_0_6V)
#define QC_MODE_20V                                 (((BC_D_3_3V) << 8)|BC_D_3_3V)
#define QC_MODE_5V                                  (((BC_D_GND) << 8)|BC_D_0_6V)
#define QC_MODE_RSVD                                (((BC_D_ERR) << 8)|BC_D_ERR)


/* BC events macros. */
#define BC_EVT_NONE                                 (0x00000000u)
#define BC_EVT_ENTRY                                (0x00000001u)
#define BC_EVT_CMP1_FIRE                            (0x00000002u)
#define BC_EVT_CMP2_FIRE                            (0x00000004u)
#define BC_EVT_QC_CHANGE                            (0x00000008u)
#define BC_EVT_QC_CONT                              (0x00000010u)
#define BC_EVT_AFC_RESET_RCVD                       (0x00000020u)
#define BC_EVT_AFC_MSG_RCVD                         (0x00000040u)
#define BC_EVT_AFC_MSG_SENT                         (0x00000080u)
#define BC_EVT_AFC_MSG_SEND_FAIL                    (0x00000100u)
#define BC_EVT_TIMEOUT1                             (0x00000200u)
#define BC_EVT_TIMEOUT2                             (0x00000400u)
#define BC_EVT_DISCONNECT                           (0x00000800u)
#define BC_EVT_ALL_MASK                             (0xFFFFFFFFu)

#define BC_CMP_0_IDX                                (0u)
#define BC_CMP_1_IDX                                (1u)

/* Current limits in 10mA units */
#define BC_AMP_LIMIT                                (300)
#define APPLE_AMP_1A                                (100)
#define APPLE_AMP_2_1A                              (210)
#define APPLE_AMP_2_4A                              (240)
#define QC_AMP_5V                                   (300)
#define QC_AMP_9V                                   (300)
#define QC_AMP_12V                                  (300)
#define QC_AMP_20V                                  (300)
#define QC_AMP_CONT                                 (300)

#define QC_CONT_VOLT_CHANGE_PER_PULSE               (200u) /* in mV */
#define QC3_MIN_VOLT                                (3400u) /* In mV */
/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/
/**
 * @enum: bc_port_type_t
 * @brief: Battery charging port type.
 */
typedef enum{
    BC_PORT_TYPE_A = 0,
    BC_PORT_TYPE_C
}bc_port_type_t;

/**
 * @enum: bc_port_role_t
 * @brief: Battery charging port role.
 */
typedef enum{
    BC_PORT_SINK = 0,
    BC_PORT_SOURCE
}bc_port_role_t;

/**
 * @enum: bc_qc_class_t
 * @brief: Qualcomm charger class.
 */
typedef enum{
    BC_QC_CLASS_A = 0,
    BC_QC_CLASS_B
}bc_qc_class_t;

/**
 * @enum: bc_qc_ver_t
 * @brief: Qualcomm charger version.
 */
typedef enum{
    BC_QC_VER_2 = 0,
    BC_QC_VER_3
}bc_qc_ver_t;

/**
 * @enum: bc_apple_id_t
 * @brief: Apple charger brick id.
 */
typedef enum{
    BC_APPLE_ID_1A = 0,
    BC_APPLE_ID_2_1A,
    BC_APPLE_ID_2_4A
}bc_apple_id_t;

/**
 * @enum: bc_charge_mode_t
 * @brief: Charge scheme options.
 */
typedef enum{
    BC_CHARGE_NONE = 0,
    BC_CHARGE_DCP,
    BC_CHARGE_QC2,
    BC_CHARGE_QC3,
    BC_CHARGE_AFC,
    BC_CHARGE_APPLE
}bc_charge_mode_t;

/**
 * @typedef bc_d_status_t
 * @brief Enum of the various Dp/Dm states
 */
typedef enum
{
    BC_D_GND = 0,  /* < 0.325 */
    BC_D_0_6V,     /* > 0.325 < 2V  */
    BC_D_3_3V,    /* > 2V */
    BC_D_ERR,

} bc_d_status_t;

/**
 * @typedef bc_apple_term
 * @brief Enum for Apple terminations codes.
 */
typedef enum
{
    APPLE_TERM1 = 1,                /**< Termination 1 code: 1 - 2.22 V */
    APPLE_TERM2 = 2,                /**< Termination 2 code: 2.22 - 2.89 V */
    APPLE_TERM3 = 3                 /**< Termination 3 code: 2.89+ V */
} bc_apple_term;

/**
 * @typedef bc_apple_brick_id
 * @brief Enum for possible Apple Brick IDs.
 */
typedef enum
{
    APPLE_BRICK_ID_0 = 0x11,        /**< DM_TERM1 : DP_TERM1 */
    APPLE_BRICK_ID_1 = 0x12,        /**< DM_TERM1 : DP_TERM2 */
    APPLE_BRICK_ID_2 = 0x13,        /**< DM_TERM1 : DP_TERM3 */
    APPLE_BRICK_ID_3 = 0x21,        /**< DM_TERM2 : DP_TERM1 */
    APPLE_BRICK_ID_4 = 0x22,        /**< DM_TERM2 : DP_TERM2 */
    APPLE_BRICK_ID_5 = 0x23,        /**< DM_TERM2 : DP_TERM3 */
    APPLE_BRICK_ID_6 = 0x31,        /**< DM_TERM3 : DP_TERM1 */
    APPLE_BRICK_ID_7 = 0x32,        /**< DM_TERM3 : DP_TERM2 */
    APPLE_BRICK_ID_8 = 0x33,        /**< DM_TERM3 : DP_TERM3 */
} bc_apple_brick_id;

/**
 * @typedef bc_dp_dm_state_t
 * @brief Union to hold Dp/Dm status.
 */
typedef union bc_d_state_t
{
    uint16_t state; /* Combined status of Dp and Dm. */
    uint8_t  d[2]; /* Individual status of Dp(d[0]) and Dm(d[1]). */
} bc_dp_dm_state_t;

/**
 * @enum: bc_state_t
 * @brief: Battery Charging states
 */
typedef enum{
    BC_FSM_OFF = 0,
    BC_FSM_SRC_LOOK_FOR_CONNECT,
    BC_FSM_SRC_INITIAL_CONNECT,
    BC_FSM_SRC_APPLE_CONNECTED,
    BC_FSM_SRC_OTHERS_CONNECTED,
    BC_FSM_SRC_QC_OR_AFC,
    BC_FSM_SRC_QC_CONNECTED,
    BC_FSM_SRC_AFC_CONNECTED,
#if (!(CCG_SOURCE_ONLY))
    BC_FSM_SINK_START,
    BC_FSM_SINK_APPLE_CHARGER_DETECT,
    BC_FSM_SINK_APPLE_BRICK_ID_DETECT,
    BC_FSM_SINK_PRIMARY_CHARGER_DETECT,
    BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED,
    BC_FSM_SINK_SECONDARY_CHARGER_DETECT,
    BC_FSM_SINK_DCP_CONNECTED,
    BC_FSM_SINK_SDP_CONNECTED,
    BC_FSM_SINK_CDP_CONNECTED,
#endif /* (!(CCG_SOURCE_ONLY)) */
    BC_FSM_MAX_STATES,
}bc_state_t;

/*
 * @enum: bc_sink_timer_t;
 * @brief: List of possible timers in sink mode.
 */
typedef enum
{
    BC_SINK_TIMER_NONE = 0,
    BC_SINK_DCD_DEBOUNCE_TIMER = 1,                     /**< DCD Debounce timer. */
} bc_sink_timer_t;

/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/

/**
 * @typedef bc_status_t
 * @brief Struct to define battery charger status.
 */
typedef struct{

    /** Current charging scheme */
    bc_charge_mode_t cur_mode;

    /* Current state of battery charger */
    bc_state_t bc_fsm_state;

    /** Stores BC events. */
    uint32_t volatile bc_evt;

    /* Current vbus in mV units */
    uint16_t cur_volt;

    /* Current Ampere in 10mA units */
    uint16_t cur_amp;

    /* Connected or not*/
    bool volatile connected;

    /* Comparator set for rising or falling edge */
    bool volatile comp_rising;

    /** Current debounced DP/DM status. */
    bc_dp_dm_state_t volatile dp_dm_status;

    /** Old DP/DM status. */
    bc_dp_dm_state_t volatile old_dp_dm_status;

    /* Sink mode: Current debounce timer active. */
    bc_sink_timer_t cur_timer;

    /* Holds successful transfer count. A transfer has to happen 3
     * times before making any VI changes
     */
    uint8_t afc_src_msg_count;

    /* Holds status of VI match of last received byte
     */
    bool volatile afc_src_is_matched;

    /* Holds current matched VI
     */
    uint8_t afc_src_cur_match_byte;

    /* Holds last matched VI
     */
    uint8_t afc_src_last_match_byte;

    /* Holds 2 out of 3 matched VI
     */
    uint8_t afc_src_matched_byte;

    /* Holds current matched VI
     */
    uint8_t afc_src_match_count;

    /* AFC transmission active. */
    uint8_t afc_tx_active;

    /* Attach detected. */
    bool attach;
}bc_status_t;

/*******************************************************************************
 * Global Variable Declaration
 ******************************************************************************/

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
/**
 * @brief This function initializes the Battery Charging block. This should be called
 * one time only at system startup.
 * @param cport Battery Charging port index.
 * @return ccg_status_t
 */
ccg_status_t bc_init(uint8_t cport);

/**
 * @brief This function starts the Battery Charging block with desired configuration.
 * @param cport Battery Charging port index.
 * @param port_role Battery Charging port power role.
 * @return ccg_status_t
 */
ccg_status_t bc_start(uint8_t cport, bc_port_role_t port_role);

/**
 * @brief This function stops the Battery Charging block.
 * @param cport Battery Charging port index.
 * @return ccg_status_t
 */
ccg_status_t bc_stop(uint8_t cport);

/**
 * @brief This function handles the Battery Charging block state machine.
 * @param cport Battery Charging port index.
 * @return ccg_status_t
 */
ccg_status_t bc_fsm(uint8_t cport);

/**
 * @brief This function puts the Battery Charging block to sleep.
 * @return Returns true if the sleep is successful, false otherwise.
 */
bool bc_sleep(void);

/**
 * @brief This function wakes up the Battery Charging block.
 * @return Returns true if wakeup successful, false otherwise
 */
bool bc_wakeup(void);

/**
 * @brief This function initializes the Battery Charging block.
 * @param cport Battery Charging port index.
 * @return bc_status_t*
 */
const bc_status_t* bc_get_status(uint8_t cport);

/**
 * @brief This function handles events from dpm.
 * @param port PD port index.
 * @param evt PD event.
 * @return None
 */
void bc_pd_event_handler(uint8_t port, app_evt_t evt);

/**
 * @brief This function sets a particular bc event.
 * @param cport Port index.
 * @param evt_mask Event Mask
 * @return None
 */
void bc_set_bc_evt(uint8_t cport, uint32_t evt_mask);

/**
 * @brief This function clears a particular bc event.
 * @param cport Port index.
 * @param evt_mask Event Mask
 * @return None
 */
void bc_clear_bc_evt(uint8_t cport, uint32_t evt_mask);

#if (LEGACY_APPLE_SRC_EXT_TERM_ENABLE)
/**
 * @brief This function enables solution specific Apple terminations on DP, DM.
 *
 * The legacy Apple + BC 1.2 source operation requires external resistive 
 * terminations. This function chooses to apply internal / external terminations
 * based on solution hardware and requirements. This can be used to selectively
 * apply parallel operation on port basis. The user is expected to implement
 * this function. The function should provide Apple termination for 2.4A when
 * requiring parallel operation. When using Apple only mode, internal termination
 * can be used.
 *
 * To limit the external components in the system, the following configuration
 * is recommended.
 *
 * Since the detection algorithm is applied only on DP line, the resistor divider
 * logic is only required on the DP line. The internal termination can be applied
 * for DM line. 
 *
 * For DP line, connect Rext from a dedicated GPIO to the DP line. The value of
 * this resistor should be such that the DP line voltage should be in range of
 * 2.5V - 2.7V. When the GPIO is strong driven high, it shall provide VDDIO voltage.
 * We can generate this voltage via resistor divider network. To avoid a second
 * resistance, internal Rdp_dwn can be used for this purpose. The resistance
 * range for this is 18.4KOhm - 21.91KOhm range (based on char data). Depending on
 * the VDDIO of the system, select Rext suitably to generate the optimum voltage.
 *
 * Over and above this, it is seen that some BC devices requires the DP-DM short
 * to exist before it applies the VDP_SRC correctly. For this purpose, it is 
 * recommended to use internal short.
 *
 * The below equations can be used to determine the optimal Rext.
 *
 * Rext_min = (((VDDIO - VDPmax) * Rint_max) / VDPmax)
 * Rext_max = (((VDDIO - VDPmin) * Rint_min) / VDPmin)
 *
 * Recommended Rext for 5V VDDIO is 18Kohm.
 * Recommended Rext for 3.3V VDDIO is 5.1KOhm.
 *
 * @param cport Port index.
 * @param charger_term Termination ID to be applied.
 * @return None
 */
void sln_apply_apple_src_term(uint8_t cport, chgb_src_term_t charger_term);

/**
 * @brief This function disables solution specific Apple terminations on DP, DM.
 *
 * The legacy Apple + BC 1.2 source operation requires external resistive 
 * terminiations. This function removes previously applied teriminations.
 *
 * @param cport Port index.
 * @return None
 */
void sln_remove_apple_src_term(uint8_t cport);

#endif /* (LEGACY_APPLE_SRC_EXT_TERM_ENABLE) */

#endif /* __BATTERY_CHARGING_H__ */

/* End of File */
