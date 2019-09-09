/**
 * @file chgb_hal.h
 *
 * @brief @{HAL layer for charger detect block on CCG3PA product family.@}
 */

/*
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
#ifndef _CHGB_HAL_H_
#define _CHGB_HAL_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include <ccgx_regs.h>
#include <status.h>

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

#define BCH_PORT_0_CMP1_2_INTR_MASK             (0x3u)
#define BCH_PORT_0_CMP1_INTR_MASK               (0x1u)
#define BCH_PORT_0_CMP2_INTR_MASK               (0x2u)
#define BCH_PORT_1_CMP1_2_INTR_MASK             (0xcu)
#define BCH_PORT_1_CMP1_INTR_MASK               (0x4u)
#define BCH_PORT_1_CMP2_INTR_MASK               (0x8u)

#define QC3_PORT_0_DP_DM_PULSE_MASK             (0x11u)
#define QC3_PORT_0_DP_PULSE_MASK                (0x01u)
#define QC3_PORT_0_DM_PULSE_MASK                (0x10u)
#define QC3_PORT_1_DP_DM_PULSE_MASK             (0x22u)
#define QC3_PORT_1_DP_PULSE_MASK                (0x02u)
#define QC3_PORT_1_DM_PULSE_MASK                (0x20u)
/* Translates to 160us for 1MHZ clock. */
#define QC3_DP_DM_PULSE_FILTER_CLOCK_SEL        (160u)

/* AFC UI (160us) in terms of number 1MHZ clock cycles. */
#define AFC_UI_CLK_CYCLE_COUNT                  (160u)

#define AFC_IDLE_OPCODE                         (0)
#define AFC_TX_PING_OPCODE                      (1)
#define AFC_RX_PING_OPCODE                      (2)
#define AFC_TX_DATA_M_OPCODE                    (3)
#define AFC_TX_DATA_S_OPCODE                    (4)
#define AFC_RX_DATA_OPCODE                      (5)

#if 1
#define AFC_SOURCE_OPCODE                       ((AFC_RX_PING_OPCODE << 0) | (AFC_TX_PING_OPCODE << 3) | \
                                                (AFC_RX_DATA_OPCODE << 6) | (AFC_TX_PING_OPCODE << 9) | \
                                                (AFC_TX_DATA_S_OPCODE << 12) | (AFC_RX_PING_OPCODE << 15) | \
                                                (AFC_TX_PING_OPCODE << 18) | (AFC_IDLE_OPCODE << 21))
#else
#define AFC_SOURCE_OPCODE                       ((AFC_RX_PING_OPCODE << 0) | (AFC_TX_PING_OPCODE << 3) | \
                                                (AFC_RX_DATA_OPCODE << 6) | (AFC_RX_PING_OPCODE << 9) | (AFC_TX_PING_OPCODE << 12) | \
                                                (AFC_TX_DATA_S_OPCODE << 15) | (AFC_RX_PING_OPCODE << 18) | \
                                                (AFC_TX_PING_OPCODE << 21) | (AFC_IDLE_OPCODE << 24))
#endif

#define AFC_SOURCE_CTRL_CFG                     (0x5434Au)
#define AFC_MAX_BYTES                           (16u)
#define AFC_BASE_VOLT                           (5000u) /* In mV steps */
#define AFC_VOLT_STEP                           (1000u) /* In mV steps */
#define AFC_BASE_AMP                            (75) /* In 10mA steps */
#define AFC_AMP_STEP                            (15) /* In 10mA steps */

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/

/**
 * @typedef bc_phy_evt_t
 * @brief battery charger event enumeration.
 */
typedef enum
{
    BC_PHY_EVT_COMP1_TRIG = 0,          /**< Comp1 triggered */
    BC_PHY_EVT_COMP2_TRIG,              /**< Comp2 triggered */

} bc_phy_evt_t;

/**
 * @typedef chgb_src_term_t
 * @brief Charger block source termination options
 */
typedef enum
{
    CHGB_SRC_TERM_APPLE_1A = 0,         /**< Apple 1A termination */
    CHGB_SRC_TERM_APPLE_2_1A,           /**< Apple 2.1A termination */
    CHGB_SRC_TERM_APPLE_2_4A,           /**< Apple 2.4A termination */
    CHGB_SRC_TERM_QC,                   /**< Quick charge termination */
    CHGB_SRC_TERM_AFC,                  /**< AFC termination */
    CHGB_SRC_TERM_DCP,                  /**< DCP termination */
    CHGB_SRC_TERM_CDP,                  /**< CDP termination */
    CHGB_SRC_TERM_SDP                   /**< SDP termination */

} chgb_src_term_t;

/**
 * @typedef chgb_snk_term_t
 * @brief Charger block sink termination options
 */
typedef enum
{
    CHGB_SINK_TERM_SPD = 0,             /**< Standard port detect */
    CHGB_SINK_TERM_PCD,                 /**< Primary charger detect. */
    CHGB_SINK_TERM_SCD,                 /**< Secondary charger detect. */
    CHGB_SINK_TERM_AFC,                 /**< AFC detect */
    CHGB_SINK_TERM_APPLE                /**< Apple detect */

} chgb_snk_term_t;

/**
 * @typedef chgb_comp_pinput_t
 * @brief Charger block comparator positive terminal input signal selection options
 */
typedef enum
{
    /* TODO: Define it as per new hardware */
    CHGB_COMP_P_DM = 0,                 /**< DM input */
    CHGB_COMP_P_VREF,                   /**< VREF input */
    CHGB_COMP_P_DP,                     /**< DP input */
    CHGB_COMP_P_GND                     /**< GND input */

} chgb_comp_pinput_t;

/**
 * @typedef chgb_comp_ninput_t
 * @brief Charger block comparator negative terminal input signal selection options
 */
typedef enum
{
    /* TODO: Define it as per new hardware */
    CHGB_COMP_N_DM = 0,                 /**< DM input */
    CHGB_COMP_N_VREF,                   /**< VREF input */
    CHGB_COMP_N_DP,                     /**< VREF input */
    CHGB_COMP_N_GND                     /**< GND input */

} chgb_comp_ninput_t;

/**
 * @typedef chgb_vref_t
 * @brief Charger block vref options
 */
typedef enum
{
#if (defined(CCG3PA) || defined(CCG3PA2))
    CHGB_VREF_0_425V = 0,               /**< 0.425V (0.325V with offset of 100mV) */
    CHGB_VREF_0_7V,                     /**< 0.6V */
    CHGB_VREF_0_85V,                    /**< 0.85V */
    CHGB_VREF_1_4V,                     /**< 1.4V */
    CHGB_VREF_1_7V,                     /**< 1.7V */
    CHGB_VREF_2V,                       /**< 2V */
    CHGB_VREF_2_2V,                     /**< 2.2V */
    CHGB_VREF_2_9V                      /**< 2.9V */
#else /* CCG3, CCG5 */
    CHGB_VREF_0_325V = 0,               /**< 0.325V */
    CHGB_VREF_0_6V,                     /**< 0.6V */
    CHGB_VREF_0_85V,                    /**< 0.85V */
    CHGB_VREF_1_4V,                     /**< 1.4V */
    CHGB_VREF_GND,                      /**< GND */
    CHGB_VREF_2V,                       /**< 2V */
    CHGB_VREF_2_2V,                     /**< 2.2V */
    CHGB_VREF_2_9V                      /**< 2.9V */
#endif /* CCGx */
} chgb_vref_t;

/**
 * @typedef chgb_comp_edge_t
 * @brief Charger block comparator neagtive terminal input signal selection options
 */
typedef enum
{
    CHGB_COMP_NO_INTR = 0,              /**< Interrupt disabled */
    CHGB_COMP_EDGE_FALLING,             /**< Falling edge interrupt only */
    CHGB_COMP_EDGE_RISING,              /**< Rising edge interrupt only */
    CHGB_COMP_EDGE_BOTH                 /**< Both edge interrupt */

} chgb_comp_edge_t;


/*******************************************************************************
 * Data Struct Definition
 ******************************************************************************/

/**
 * @typedef pd_phy_cbk_t
 * @brief PD PHY callback prototype. This function will be used to notify
 * the stack about PHY events.
 *
 * @param cport Charging port on which the PHY event occured.
 * @param event Type of BC PHY event.
 */
typedef void(*bc_phy_cbk_t)(uint8_t cport, uint32_t event);

/*******************************************************************************
 * Global Function Declaration
 ******************************************************************************/

/**
 * @brief This function applies dcp(Source) terminations on D+/D-
 *
 * @param cport Charging port index.
 * @param cbk Charger block callback handler
 * @return ccg_status_t
 */
ccg_status_t chgb_init(uint8_t cport, bc_phy_cbk_t cbk);

/**
 * @brief This function enables the charging block hardware. The function expects
 * that the charger block is already initialized.
 *
 * @param cpor Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_enable(uint8_t cport);

/**
 * @brief This function disables the charging block hardware after a previous
 * enable call.
 *
 * @param cpor Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_disable(uint8_t cport);
/**
 * @brief This function applies Source terminations on D+/D- for various modes
 * e.g. Apple, QC, AFC, DCP etc
 * @param cport Charging port index.
 * @param charger_term Source termination options
 *
 * @return ccg_status_t
 */
ccg_status_t chgb_apply_src_term(uint8_t cport, chgb_src_term_t charger_term);

/**
 * @brief This function applies Sink terminations on D+/D- for various modes
 * e.g. Apple, AFC, standard PD etc
 * @param cport Charging port index.
 * @param charger_term Sink termination options
 *
 * @return ccg_status_t
 */
ccg_status_t chgb_apply_sink_term(uint8_t cport, chgb_snk_term_t charger_term);

/**
 * @brief This function removes all terminations/Voltage sources from D+/D-
 *
 * @param cpor Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_remove_term(uint8_t cport);

/**
 * @brief This function removes all terminations from D+/D-
 *
 * @param cport Charging port index.
 * @param comp_idx Comparator index
 * @param p_input Positive input signal selection
 * @param n_input Negative input signal selection
 * @param vref Vref value in case input selection is vref. It is expected only
 * one terminal of comparator is connected to vref signal.
 * @param edge Edge selection. If edge is selected then callback registerd in chgb_init
 * will be called when event fires. If edge is no interrupt then this function
 * will revert comp to old settings after comparision is done.
 * @return comp output
 */
bool chgb_set_comp(uint8_t cport, uint8_t comp_idx, chgb_comp_pinput_t p_input,
    chgb_comp_ninput_t n_input, chgb_vref_t vref, chgb_comp_edge_t edge);

/**
 * @brief This function power down the comparator.
 *
 * @param cport Charging port index.
 * @param comp_idx Comparator index
 * @return Comp output
 */
ccg_status_t chgb_stop_comp(uint8_t cport, uint8_t comp_idx);

/**
 * @brief This function returns the comparator output.
 *
 * @param cport Charging port index.
 * @param comp_idx Comparator index
 * @return Comp output
 */
bool chgb_get_comp_result(uint8_t cport, uint8_t comp_idx);

/**
 * @brief This function initializes the charging block for QC Source operation.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_qc_src_init(uint8_t cport);

/**
 * @brief This function stops the charging block for QC Source operation.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_qc_src_stop(uint8_t cport);

/**
 * @brief This function starts the charging block for QC Source continuous mode operation.
 * This function expects that the charger block is already initialized for QC operation.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_qc_src_cont_mode_start(uint8_t cport);

/**
 * @brief This function stops the charging block for QC Source continuous mode operation.
 * This function expects that the charging block is already functioning in QC mode.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_qc_src_cont_mode_stop(uint8_t cport);

/**
 * @brief This function gets the current pulse count for QC continuous mode.
 * The function expects that the charger block is already setup for QC continuous
 * mode operation.
 *
 * Pulse count > 0 is +ve change. Pulse count < 0 is -ve change.
 * @param cport Charging port index.
 * @return pulse count
 */
int chgb_get_qc_pulse_count(uint8_t cport);

/**
 * @brief This function updates the current pulse count by subtracting new_count.
  * The function expects that the charger block is already setup for QC continuous
 * mode operation.
 *
 * @param cport Charging port index.
 * @param new_count New count subtarcted from the current count.
 * @return ccg_status_t
 */
void chgb_update_qc_pulse_count(uint8_t cport, int new_count);

/**
 * @brief This function initializes the charging block for AFC Source operation.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_afc_src_init(uint8_t cport);

/**
 * @brief This function starts the charging block for AFC Source operation. The
 * function expects that the charger block is already initialized for AFC source
 * operation.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_afc_src_start(uint8_t cport);

/**
 * @brief This function stops the charging block for AFC Source operation.
 * The function expects that the charging block is aready functioning as AFC source.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_afc_src_stop(uint8_t cport);

/**
 * @brief This function returns pointer to received data via AFC protocol.
 *
 * @param cport Charging port index.
 * @return pointer to rx data array
 */
uint8_t* chgb_afc_get_rx_data_ptr(uint8_t cport);

/**
 * @brief This function returns no of bytes received via AFC protocol.
 * @param cport Charging port index.
 * @return count
 */
uint8_t chgb_afc_get_rx_data_count(uint8_t cport);

/**
 * @brief This function sets the TX buffer for AFC transmission.
 * @param cport Charging port index.
 * @param data_ptr Pointer to data array.
 * @param count No of bytes to be transmitted.
 * @Warning Count shall never be > 16
 * @return None
 */
void chgb_afc_set_tx_data(uint8_t cport, uint8_t* data_ptr, uint8_t count);

/**
 * @brief This function applies pulldown on D+.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_apply_dp_pd(uint8_t cport);

/**
 * @brief This function removes pulldown on D+.
 *
 * @param cport Charging port index.
 * @return ccg_status_t
 */
ccg_status_t chgb_remove_dp_pd(uint8_t cport);

/**
 * @brief This function applies RDAT_LKG resistor on DP.
 *
 * @param port Charging port index.
 * @return NULL
 */
void chgb_apply_rdat_lkg_dp(uint8_t port);

/**
 * @brief This function applies RDAT_LKG resistor on DM.
 *
 * @param port Charging port index.
 * @return NULL
 */
void chgb_apply_rdat_lkg_dm(uint8_t port);

/**
 * @brief This function applies Apple source terminations DP.
 *
 * @param port Charging port index
 * @param apple_term_id Apple termination ID
 * @return NULL
 */
void chgb_apply_apple_src_dp(uint8_t port, chgb_src_term_t apple_term_id);

/**
 * @brief This function applies Apple source terminations DM.
 *
 * @param port Charging port index
 * @param apple_term_id Apple termination ID
 * @return NULL
 */
void chgb_apply_apple_src_dm(uint8_t port, chgb_src_term_t apple_term_id);

/**
 * @brief This function removes Apple source terminations DP.
 *
 * @param port Charging port index
 * @return NULL
 */
void chgb_remove_apple_src_dp(uint8_t port);

/**
 * @brief This function removes Apple source terminations DP.
 *
 * @param port Charging port index
 * @return NULL
 */
void chgb_remove_apple_src_dm(uint8_t port);

/**
 * @brief This function enables Apple Brick ID detection.
 *
 * @param port Charging port index.
 * @return NULL
 */
void chgb_enable_apple_det(uint8_t port);

/**
 * @brief This function disables Apple Brick ID detection.
 *
 * @param port Charging port index.
 * @return NULL
 */
void chgb_disable_apple_det(uint8_t port);

/**
 * @brief This function configures the charging block for deepsleep entry.
 * @param cport Port index. Caller should ensure to provide only valid values.
 */
void chgb_deepsleep(uint8_t cport);

/**
 * @brief This function configures the Charging block on deepsleep exit.
 * @param cport Port index. Caller should ensure to provide only valid values.
 */
void chgb_wakeup(uint8_t cport);

#endif /* __CHGB_HAL_H__ */

/* End of File */

