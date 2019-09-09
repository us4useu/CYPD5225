/**
 * @file usb.c
 *
 * @brief @{USB driver source file for CCG3 & Dock Management Controller (DMC).@}
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

#include "stdint.h"
#include "stdbool.h"
#include "config.h"
#include "ccgx_regs.h"
#include "status.h"
#include "usb.h"
#include "utils.h"

#if (USB_SUSPEND_CHECK_DISABLE == 0)
#include "timer.h"
#include "gpio.h"
#endif /* USB_SUSPEND_CHECK_DISABLE */

/*
 * The number of clock cycles required to identify a bus reset.
 * This value is hardware dependant and should not be changed.
 */
#define USB_BUS_RST_CNT_VALUE     (0x0000000F)
#define USB_REG_LOCK_TIMEOUT      (0xFFFFFFFF)

/*
 * Reference voltage enable bit for USB in SRSS->pwr_bg_config.
 * register. Do not change the value as the field is hardware
 * defined.
 */
#define SRSS_VREF_EN_USB          (1u << 18)

/* USB interrupt priority levels and vector location details. */
#if (defined(CCG3)) || (defined (DMC))
#define USB_INTR_HIGH_VECTOR_LOCATION   (18)
#define USB_INTR_MED_VECTOR_LOCATION    (19)
#define USB_INTR_LOW_VECTOR_LOCATION    (20)

#if (USB_SUSPEND_CHECK_DISABLE == 0)
/* Internal pad for USB DP line. */
#define USB_DP_GPIO                     (GPIO_PORT_4_PIN_0)
#define USB_GPIO_INTR_VECTOR            (GPIO_PORT4_INTR_NO)
#endif /* USB_SUSPEND_CHECK_DISABLE */

#define USB_INTR_HIGH_VECTOR_MASK       (1 << USB_INTR_HIGH_VECTOR_LOCATION)
#define USB_INTR_MED_VECTOR_MASK        (1 << USB_INTR_MED_VECTOR_LOCATION)
#define USB_INTR_LOW_VECTOR_MASK        (1 << USB_INTR_LOW_VECTOR_LOCATION)
#if (USB_SUSPEND_CHECK_DISABLE == 0)
#define USB_GPIO_INTR_VECTOR_MASK       (1 << GPIO_PORT4_INTR_NO)
#endif /* USB_SUSPEND_CHECK_DISABLE */

#if (USB_SUSPEND_CHECK_DISABLE == 0)
#define USB_INTR_VECTOR_MASK            \
        (                               \
        USB_INTR_HIGH_VECTOR_MASK       \
        | USB_INTR_MED_VECTOR_MASK      \
        | USB_INTR_LOW_VECTOR_MASK      \
        | USB_GPIO_INTR_VECTOR_MASK     \
        )
#else /* USB_SUSPEND_CHECK_DISABLE */
#define USB_INTR_VECTOR_MASK            \
        (                               \
        USB_INTR_HIGH_VECTOR_MASK       \
        | USB_INTR_MED_VECTOR_MASK      \
        | USB_INTR_LOW_VECTOR_MASK      \
        )
#endif /* USB_SUSPEND_CHECK_DISABLE */

#define USB_SIE_INTR_ALL_MASK           (USBDEV_BCD_USB_INTR_SIE_SOF_INTR | \
        USBDEV_BCD_USB_INTR_SIE_BUS_RESET_INTR | \
        USBDEV_BCD_USB_INTR_SIE_EP0_INTR | \
        USBDEV_BCD_USB_INTR_SIE_LPM_INTR | \
        USBDEV_BCD_USB_INTR_SIE_RESUME_INTR)

#define USB_EP_INTR_OFFSET              (8)
#define USB_EP_INTR_ALL_MASK            (0xFF0)

/* The trim value needs to be loaded differently from firmware as per CDT #267973. */
#define USBIO_TRIM_REG_EN_VALUE         (0x00000028)
#define USBIO_TRIM_REG_DIS_VALUE        (0x00000000)

#else /* Not supported. */
#error "Device not supported."
#endif

/* USB interrupt priority options */
typedef enum
{
    USB_INTR_PRIORITY_HIGH, /* Configured to use the USB_INTR_HIGH_VECTOR_LOCATION */
    USB_INTR_PRIORITY_MED,  /* Configured to use the USB_INTR_MED_VECTOR_LOCATION */
    USB_INTR_PRIORITY_LOW   /* Configured to use the USB_INTR_LOW_VECTOR_LOCATION */

} usb_intr_priority_t;

/*
 * Since the register addresses are scattered, the access to the register
 * is made easy by the following structure pointer arrays.
 */
const PUSBSIE_REGS_T USBSIE[] =
{
    (PUSBSIE_REGS_T)USBDEV_SIE_EP1_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP2_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP3_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP4_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP5_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP6_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP7_CNT0_ADDRESS,
    (PUSBSIE_REGS_T)USBDEV_SIE_EP8_CNT0_ADDRESS
};

const PUSBARB_REGS_T USBARB[] =
{
    (PUSBARB_REGS_T)USBDEV_ARB_EP1_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP2_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP3_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP4_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP5_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP6_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP7_CFG_ADDRESS,
    (PUSBARB_REGS_T)USBDEV_ARB_EP8_CFG_ADDRESS
};

const PUSBARB16_REGS_T USBARB16[] =
{
    (PUSBARB16_REGS_T)USBDEV_ARB_RW1_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW2_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW3_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW4_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW5_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW6_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW7_WA16_ADDRESS,
    (PUSBARB16_REGS_T)USBDEV_ARB_RW8_WA16_ADDRESS
};

/* USB module internal handle structure */
static volatile usb_handle_t gl_usb;

uint32_t usb_intr_lock(void)
{
    uint8_t state;
    uint32_t lock;

    /*
     * Read the interrupt vector status and disable the interrupts.
     * Status need to be read and disabled inside critical section
     * to avoid race conditions.
     */
    state = CyEnterCriticalSection();
    lock = CM0->iser & USB_INTR_VECTOR_MASK;
    CM0->icer = USB_INTR_VECTOR_MASK;
    CyExitCriticalSection(state);

    return lock;
}

void usb_intr_unlock(uint32_t lock)
{
    uint8_t state;

    /* Read the interrupt vector status and disable the interrupts.
     * Status need to be read and disabled inside critical section
     * to avoid race conditions. */
    state = CyEnterCriticalSection();
    CM0->iser = (lock & USB_INTR_VECTOR_MASK);
    CyExitCriticalSection(state);
}

/* Forward declaration of static functions. */
static void usb_intr_high_handler(void);
static void usb_intr_low_handler(void);
#if (USB_SUSPEND_CHECK_DISABLE == 0)
static void usb_gpio_intr_handler(void);
#endif /* USB_SUSPEND_CHECK_DISABLE */

ccg_status_t usb_init(usb_config_t *cfg)
{
    /* Check for parameter validity. */
    if (gl_usb.state != USB_STATE_DEINITED)
    {
        return CCG_STAT_BUSY;
    }

    if ((cfg == NULL) || (cfg->get_dscr_cb == NULL))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    /* Initialize the USB software state machine. */
    gl_usb.state = USB_STATE_DISABLED;
    gl_usb.cfg = *cfg;

    /*
     * Setup the interrupt vectors. EP0 and USB interrupts are
     * to be configured for high priority and EPx interrupts
     * are to be configured as low priority. The interrupts
     * are not enabled at this time.
     */
    USBDEV_BCD->intr_lvl_sel = ((USB_INTR_PRIORITY_HIGH << USBDEV_BCD_USB_INTR_LVL_SEL_SOF_LVL_SEL_POS) |
            (USB_INTR_PRIORITY_HIGH << USBDEV_BCD_USB_INTR_LVL_SEL_BUS_RESET_LVL_SEL_POS) |
            (USB_INTR_PRIORITY_HIGH << USBDEV_BCD_USB_INTR_LVL_SEL_EP0_LVL_SEL_POS) |
            (USB_INTR_PRIORITY_HIGH << USBDEV_BCD_USB_INTR_LVL_SEL_LPM_LVL_SEL_POS) |
            (USB_INTR_PRIORITY_HIGH << USBDEV_BCD_USB_INTR_LVL_SEL_RESUME_LVL_SEL_POS));

    CyIntSetVector(USB_INTR_HIGH_VECTOR_LOCATION, usb_intr_high_handler);
    CyIntSetVector(USB_INTR_LOW_VECTOR_LOCATION, usb_intr_low_handler);

#if (USB_SUSPEND_CHECK_DISABLE == 0)
    /*
     * The resume functionality works by configuring the DP GPIO as a interrupt
     * source. The DP line is driven low when a resume / reset is triggered.
     * Since this is the only interrupt usage model for this GPIO interrupt
     * handler, the vector itself is controlled for this purpose.
     */
    gpio_set_drv_mode(USB_DP_GPIO, GPIO_DM_HIZ_DIGITAL);
    CyIntSetVector(USB_GPIO_INTR_VECTOR, usb_gpio_intr_handler);
#endif /* USB_SUSPEND_CHECK_DISABLE */

    return CCG_STAT_SUCCESS;
}

/* Internal function which initalizes the EP0. */
static void usb_ep0_init(void)
{
    /* Update state-machine */
    gl_usb.ep0_state = USB_EP0_STATE_SETUP;

    /* Configure the endpoint. */
    USBDEV->ep0_cnt = 0;
    USBDEV->ep0_cr  = USBDEV_EP_MODE_NAK_INOUT;
}

/* Internal function which disables EP0. */
static void usb_ep0_disable(void)
{
    /* Disable the endpoint. */
    USBDEV->ep0_cnt = 0;
    USBDEV->ep0_cr = USBDEV_EP_MODE_DISABLE;

    /* Update the state machine. */
    gl_usb.ep0_state = USB_EP0_STATE_DISABLED;
    gl_usb.ep0_buffer = NULL;
    gl_usb.ep0_zlp_rqd = false;
}

/* Internal function which disables and resets all endpoints except EP0 */
static void usb_epx_disable(void)
{
    uint8_t index;

    for (index = 0; index < USB_NUM_EP; index++)
    {
        USBARB[index]->arb_epx_cfg = USBDEV_ARB_EPx_CFG_RESET_PTR |
            USBDEV_ARB_EPx_CFG_CRC_BYPASS;
        USBARB[index]->arb_epx_sr = ~0;
        USBARB[index]->arb_epx_int_en = 0;
        USBSIE[index]->sie_epx_cr0 = USBDEV_EP_MODE_DISABLE;

        gl_usb.ep_handle[index].toggle = false;
        gl_usb.ep_handle[index].enabled = false;
    }
}

/*
 * Internal function which handles the USB connection. This function is
 * invoked from the VBUS change interrupt handler. The function enables
 * the USB block for normal functionality. It is expected that the
 * function is invoked from either an interrupt handler or usb_interrupt
 * locked condition. In the current implementation, the VBUS is always
 * expected to be present when usb_enable() is called and so, there is
 * no VBUS interrupt handler.
 */
static void usb_connect(void)
{
    uint32_t value;

    USBDEV->cr0 = USBDEV_CR0_USB_ENABLE;

    value = (USBDEV_BCD_USB_POWER_CTRL_ENABLE | USBDEV_BCD_USB_POWER_CTRL_ENABLE_DMO |
            USBDEV_BCD_USB_POWER_CTRL_ENABLE_DPO | USBDEV_BCD_USB_POWER_CTRL_ENABLE_RCVR);
    value |= (USBDEV_BCD_USB_POWER_CTRL_SUSPEND | USBDEV_BCD_USB_POWER_CTRL_SUSPEND_DEL);
    value |= (3 << USBDEV_BCD_USB_POWER_CTRL_VBUS_VALID_OVR_POS);
    USBDEV_BCD->power_ctrl = value;
    CyDelayUs(2);
    value &= ~(USBDEV_BCD_USB_POWER_CTRL_SUSPEND);
    USBDEV_BCD->power_ctrl = value;
    CyDelayUs(2);
    value &= ~(USBDEV_BCD_USB_POWER_CTRL_SUSPEND_DEL);
    USBDEV_BCD->power_ctrl = value;

    /* Set the dedicated endpoint buffer sizes as 64 bytes. */
    USBDEV->buf_size = USB_BUF_SIZE_64BYTE;

    /* Disable all endpoints and configure endpoint 0. */
    usb_epx_disable();
    usb_ep0_init();

    USBDEV->usbio_cr1 = USBDEV_USBIO_CR1_USBPUEN;

    /* Enable the USB interrupts - EP0, BUS_RESET and SOF. */
    USBDEV_BCD->intr_sie = ~0;
    USBDEV_BCD->intr_sie_mask = (
            USBDEV_BCD_USB_INTR_SIE_EP0_INTR
            | USBDEV_BCD_USB_INTR_SIE_BUS_RESET_INTR
#if (USB_SUSPEND_CHECK_DISABLE == 0)
            | USBDEV_BCD_USB_INTR_SIE_SOF_INTR
#endif /* USB_SUSPEND_CHECK_DISABLE */
            );

    gl_usb.state = USB_STATE_CONNECTED;

#if (USB_SUSPEND_CHECK_DISABLE == 0)
    /*
     * Since you can have a situation where there is no USB activity on connect,
     * we may not get any SOFs. This will put us in a situation where we do not
     * go to low power mode. To avoid this, start looking for USB suspend from
     * the connect state directly. Fake an SOF interrupt.
     */
    USBDEV_BCD->intr_sie_set = USBDEV_BCD_USB_INTR_SIE_SET_SOF_INTR_SET;
#endif /* USB_SUSPEND_CHECK_DISABLE */
}

/*
 * Internal function which handles the USB disconnection. This function is
 * invoked from the VBUS change interrupt handler. The function disables
 * the USB block to save power. The D+ pull-up is still retained. It is
 * expected that the function is invoked from either an interrupt handler
 * or usb_interrupt locked condition. Since there is no seperate VBUS
 * interrupt handler, the call is only made from the usb_disable() function.
 */
static void usb_disconnect(void)
{
    /* Disable the USB interrupts. */
    USBDEV_BCD->intr_sie_mask = 0;
    USBDEV_BCD->intr_sie = ~0;

    usb_ep0_disable();

    USBDEV_BCD->intr_sie_mask = 0;
    USBDEV->usbio_cr1 = 0;
    USBDEV_BCD->power_ctrl = USBDEV_BCD_USB_POWER_CTRL_DEFAULT;
    gl_usb.state = USB_STATE_WAIT_FOR_VBUS;

    usb_epx_disable();

    USBDEV->cr1 = USBDEV_CR1_DEFAULT;
    USBDEV->cr0 = USBDEV_CR0_DEFAULT;
    USBDEV->usb_clk_en &= ~USBDEV_USB_CLK_EN_CSR_CLK_EN;
    CyDelayUs(1);

#if (USB_SUSPEND_CHECK_DISABLE == 0)
    gl_usb.suspend_check = false;
    timer_stop(USB_TIMER_INSTANCE, USB_SUSPEND_TIMER);
#endif /* USB_SUSPEND_CHECK_DISABLE */
}

void usb_vbus_int_handler(bool vbus_state)
{
    uint32_t lock;

    if (gl_usb.state <= USB_STATE_DISABLED)
    {
        return;
    }

    lock = usb_intr_lock();
    if ((vbus_state) && (gl_usb.state == USB_STATE_WAIT_FOR_VBUS))
    {
        usb_connect();
    }
    else if ((!vbus_state) && (gl_usb.state != USB_STATE_WAIT_FOR_VBUS))
    {
        usb_disconnect();
    }
    else
    {
        /* Spurious interrupt. Do nothing. */
    }

    if (gl_usb.cfg.event_cb != NULL)
    {
        gl_usb.cfg.event_cb(gl_usb.state, 0);
    }

    usb_intr_unlock(lock);
}

ccg_status_t usb_enable(bool reg_enable)
{
    if (gl_usb.state != USB_STATE_DISABLED)
    {
        return CCG_STAT_BUSY;
    }

    /* Update relevant software state machine. */
    gl_usb.dev_stat = 0;
    if (!gl_usb.cfg.bus_power)
    {
        gl_usb.dev_stat = 0x01;
    }

    gl_usb.int_event = 0;
    /*
     * Since usb_connect() is being called directly from here, there is no 
     * reason to load this settting twice. We can avoid initializing these
     * here. But code is left here, for indicating that these fields are
     * un-initialized at this point.
     * gl_usb.state = USB_STATE_WAIT_FOR_VBUS;
     * gl_usb.prev_state = USB_STATE_DISABLED;
     * gl_usb.ep0_state = USB_EP0_STATE_DISABLED;
     * gl_usb.active_cfg = 0;
     */
    memset((uint8_t *)&gl_usb.active_alt_inf, 0, USB_NUM_INTERFACE);

    /* Enable USB clock. */
    USBDEV->usb_clk_en = USBDEV_USB_CLK_EN_CSR_CLK_EN;
    CyDelayUs(1);

    /* Update the IMO trim settings to USB mode. */
    SRSSLT->clk_imo_trim1 = SFLASH->imo_trim_usbmode_48;
    /* Enable locking of clock as well as enable the regulator. */
    if (reg_enable)
    {
        USBDEV->cr1 = (USBDEV_CR1_ENABLE_LOCK | USBDEV_CR1_REG_ENABLE);
        USBDEV_BCD->usbio_trim = USBIO_TRIM_REG_EN_VALUE;
    }
    else
    {
        USBDEV->cr1 = (USBDEV_CR1_ENABLE_LOCK);
        USBDEV_BCD->usbio_trim = USBIO_TRIM_REG_DIS_VALUE;
    }

    /* Disable the PS2 mode. */
    USBDEV->usbio_cr1 = 0;
    /* Update the reset counter value. */
    USBDEV->bus_rst_cnt = USB_BUS_RST_CNT_VALUE;

    /*
     * Enable the interrupt vectors. The actual interrupts shall get
     * enabled as per the state machine.
     */
    CyIntEnable(USB_INTR_HIGH_VECTOR_LOCATION);
    CyIntEnable(USB_INTR_LOW_VECTOR_LOCATION);

    /* 
     * Since there is no seperate VBUS state indicator, start the USB
     * connection directly. The VBUS is expected to be already present when
     * this is invoked.
     */
    usb_connect();
    if (gl_usb.cfg.event_cb != NULL)
    {
        gl_usb.cfg.event_cb(gl_usb.state, 0);
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_disable(void)
{
    uint32_t value;

    if (gl_usb.state <= USB_STATE_DISABLED)
    {
        return CCG_STAT_NOT_READY;
    }

    /* Disable the interrupt vectors. */
    CyIntDisable(USB_INTR_HIGH_VECTOR_LOCATION);
    CyIntDisable(USB_INTR_LOW_VECTOR_LOCATION);

    /* Ensure that the USB device module is disconnected. */
    usb_disconnect();

    /* Disconnect physically from the bus. */
    USBDEV->usbio_cr1 = USBDEV_USBIO_CR1_DEFAULT;

    /* Update the IMO clock offset to the non-USB enabled value. */
    value = SRSSLT->clk_imo_trim1 & ~CLK_IMO_TRIM1_OFFSET_MASK;
    /* Assuming only 48MHz operation. */
    SRSSLT->clk_imo_trim1 = value | SFLASH->imo_trim_lt[24];

    /* Zero out fine trim */
    SRSSLT->clk_imo_trim2 &= CLK_IMO_TRIM2_FSOFFSET_MASK;

    /* Apply TC trim */
    value = SRSSLT->clk_imo_trim3 &~CLK_IMO_TRIM3_TCTRIM_MASK;
    /* Assuming only 48MHz operation. */
    SRSSLT->clk_imo_trim3 = (value | (SFLASH->imo_trim_lt[24] << CLK_IMO_TRIM3_TCTRIM_POS));

    gl_usb.state = USB_STATE_DISABLED;

    if (gl_usb.cfg.event_cb != NULL)
    {
        gl_usb.cfg.event_cb(gl_usb.state, 0);
    }

    return CCG_STAT_SUCCESS;
}

#if (USB_SUSPEND_CHECK_DISABLE == 0)
static void usb_remote_wakeup_cb(uint8_t instance, timer_id_t id)
{
    (void)instance;
    (void)id;

    /* Restore the manual K signal drive. */
    USBDEV->usbio_cr0 = 0;
}

ccg_status_t usb_remote_wakeup(void)
{
    uint8_t state;

    if ((gl_usb.state != USB_STATE_SUSPENDED) ||
            ((gl_usb.dev_stat & USB_REMOTE_WAKE) == 0))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Perform the following operation in critical section. */
    state = CyEnterCriticalSection();

    /* Clear PHY suspend. */
    USBDEV_BCD->power_ctrl &= ~(USBDEV_BCD_USB_POWER_CTRL_SUSPEND);
    CyDelayUs(2);
    USBDEV_BCD->power_ctrl &= ~(USBDEV_BCD_USB_POWER_CTRL_SUSPEND_DEL);

    /* Manually transmit K-signals for about 15 ms. */
    USBDEV->usbio_cr0 = USBDEV_USBIO_CR0_TEN;

    timer_start(USB_TIMER_INSTANCE, USB_REMOTE_WAKEUP_TIMER, 
            USB_REMOTE_WAKEUP_TIMER_PERIOD, usb_remote_wakeup_cb);
    CyExitCriticalSection(state);
    
    return CCG_STAT_SUCCESS;
}
#endif /* USB_SUSPEND_CHECK_DISABLE */

usb_state_t usb_get_state(void)
{
    return gl_usb.state;
}

bool usb_is_idle(void)
{
    if ((gl_usb.state <= USB_STATE_DISABLED) ||
            (gl_usb.int_event == 0))
    {
        return true;
    }

    return false;
}

/* The function queues a single packet read on EP0 */
static ccg_status_t usb_ep0_queue_read(void)
{
    if ((gl_usb.ep0_state != USB_EP0_STATE_DATA_OUT) ||
            (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD))
    {
        return CCG_STAT_NOT_READY;
    }

    USBDEV->ep0_cnt = 0;
    USBDEV->ep0_cr = USBDEV_EP_MODE_ACK_OUT_STATUS_IN;

    return CCG_STAT_SUCCESS;
}

/* The function reads one packet of data from EP0. It is expected that
 * the caller ensures the parameter validity and calls only when there
 * is valid data available in the endpoint buffer. */
static ccg_status_t usb_ep0_read_packet(uint8_t *data, uint8_t *count)
{
    uint8_t index;
    uint32_t value = USBDEV->ep0_cnt;
    uint8_t fifo_cnt = (value & USBDEV_EP0_CNT_BYTE_COUNT_MASK);
    bool toggle = (value & USBDEV_EP0_CNT_DATA_TOGGLE) ?
        true : false;
    ccg_status_t status = CCG_STAT_SUCCESS;

    fifo_cnt -= USB_CRC_SIZE;

    if ((gl_usb.ep0_state != USB_EP0_STATE_DATA_OUT) ||
            (!(value & USBDEV_EP0_CNT_DATA_VALID)) ||
            (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Check for the correct data toggle. */
    if (!(gl_usb.ep0_toggle ^ toggle))
    {
        /* Read data from the FIFO. */
        for (index = 0; index < fifo_cnt; index++)
        {
            data[index] = (uint8_t)USBDEV->ep0_dr[index];
        }

        gl_usb.ep0_toggle = (gl_usb.ep0_toggle) ?
            false : true;
    }
    else
    {
        status = CCG_STAT_TRANS_FAILURE;
    }

    /* Make the data invalid. */
    USBDEV->ep0_cnt = 0;

    *count = fifo_cnt;

    return status;
}

/* The function queues a single packet of data on EP0. The caller is
 * expected to ensure parameter validity and ensure that EP0 is ready
 * for transmission. */
static ccg_status_t usb_ep0_send_packet(uint8_t *data, uint8_t count)
{
    uint8_t index;

    if ((gl_usb.ep0_state != USB_EP0_STATE_DATA_IN) ||
            (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Copy data into the FIFO. */
    for (index = 0; index < count; index++)
    {
        USBDEV->ep0_dr[index] = data[index];
    }

    /* Commit the data. */
    if (gl_usb.ep0_toggle)
    {
        USBDEV->ep0_cnt = (count | USBDEV_EP0_CNT_DATA_TOGGLE);
        gl_usb.ep0_toggle = false;
    }
    else
    {
        USBDEV->ep0_cnt = count;
        gl_usb.ep0_toggle = true;
    }
    USBDEV->ep0_cr = USBDEV_EP_MODE_ACK_IN_STATUS_OUT;

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep0_send_recv_status(void)
{
    if (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD)
    {
        return CCG_STAT_NOT_READY;
    }

    if (gl_usb.setup_pkt.attrib & USB_SETUP_PKT_DIRECTION)
    {
        gl_usb.ep0_state = USB_EP0_STATE_STATUS_OUT;
        USBDEV->ep0_cr = USBDEV_EP_MODE_STATUS_OUT_ONLY;
    }
    else
    {
        gl_usb.ep0_state = USB_EP0_STATE_STATUS_IN;
        USBDEV->ep0_cr = USBDEV_EP_MODE_STATUS_IN_ONLY;
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep0_set_stall(void)
{
    if (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD)
    {
        return CCG_STAT_NOT_READY;
    }

    gl_usb.ep0_state = USB_EP0_STATE_STALL;
    USBDEV->ep0_cr = USBDEV_EP_MODE_STALL_INOUT;

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep0_setup_read(uint8_t *data, uint16_t length, bool last, usb_setup_cb_t cb)
{
    ccg_status_t status = CCG_STAT_SUCCESS;

    /* All read should be multiple of EP0 size. */
    if ((data == NULL) || ((length & (USB_EP0_SIZE - 1)) != 0))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    if ((gl_usb.ep0_state != USB_EP0_STATE_SETUP) &&
            (gl_usb.ep0_state != USB_EP0_STATE_DATA_OUT))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Update state-machine. */
    gl_usb.ep0_last = last;
    gl_usb.ep0_state = USB_EP0_STATE_DATA_OUT;
    gl_usb.ep0_buffer = data;
    gl_usb.ep0_length = length;
    gl_usb.ep0_xfer_cb = cb;

    /* Queue a read on the endpoint */
    status = usb_ep0_queue_read ();

    return status;
}

ccg_status_t usb_ep0_setup_write(uint8_t *data, uint16_t length, bool last, usb_setup_cb_t cb)
{
    uint8_t count;
    ccg_status_t status = CCG_STAT_SUCCESS;

    if (data == NULL)
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    if ((gl_usb.ep0_state != USB_EP0_STATE_SETUP) &&
            (gl_usb.ep0_state != USB_EP0_STATE_DATA_IN))
    {
        return CCG_STAT_NOT_READY;
    }

    /* Update state-machine. */
    gl_usb.ep0_last = last;
    gl_usb.ep0_state = USB_EP0_STATE_DATA_IN;
    count = GET_MIN(length, USB_EP0_SIZE);
    gl_usb.ep0_buffer = data + count;
    gl_usb.ep0_length = length - count;
    /* Check if the transfer requires a ZLP for
     * termination. This is required only when
     * the transfer length is less than the requested
     * length and the transfer length is a multiple of
     * EP0 max packet size. */
    if ((last) && (((length & (USB_EP0_SIZE - 1)) == 0) &&
                (length < gl_usb.setup_pkt.length)))
    {
        gl_usb.ep0_zlp_rqd = true;
    }
    else
    {
        gl_usb.ep0_zlp_rqd = false;
    }
    gl_usb.ep0_xfer_cb = cb;

    /* Send out the first packet of data. */
    status = usb_ep0_send_packet (data, count);

    return status;
}

/* The function waits for status phase on EP0. */
ccg_status_t usb_ep0_wait_for_ack(void)
{
    uint32_t timeout = 100000;

    /* Wait for the status to be sent. */
    while (--timeout)
    {
        if ((USBDEV->ep0_cr & (USBDEV_EP0_CR_IN_RCVD |
                        USBDEV_EP0_CR_ACKED_TXN)) ==
                (USBDEV_EP0_CR_IN_RCVD | USBDEV_EP0_CR_ACKED_TXN))
        {
            do
            {
                USBDEV->ep0_cr |= USBDEV_EP0_CR_IN_RCVD;
                CyDelayUs(1);
                timeout--;

                if (timeout == 0)
                {
                    return CCG_STAT_TIMEOUT;
                }

            } while (USBDEV->ep0_cr & USBDEV_EP0_CR_IN_RCVD);

            break;
        }

        /* We need to break out of the loop if there was
         * a USB reset. In all other cases, we can afford
         * to wait for the 100ms timeout. */
        if (USBDEV_BCD->intr_sie & USBDEV_BCD_USB_INTR_SIE_BUS_RESET_INTR)
        {
            return CCG_STAT_TRANS_FAILURE;
        }

        CyDelayUs(1);
    }

    if (timeout == 0)
    {
        return CCG_STAT_TIMEOUT;
    }

    return CCG_STAT_SUCCESS;
}

/* Copy data from EP0 buffer to internal buffer. The caller is expected to
 * ensure parameter validity and also ensure that there is a valid setup
 * packet available in the endpoint buffer. */
static void usb_copy_setup_pkt(uint8_t *ptr)
{
    uint8_t index;

    for (index = 0; index < 8; index++)
    {
        ptr[index] = USBDEV->ep0_dr[index] &
            USBDEV_EP0_DR_DATA_BYTE_MASK;
    }
}

/* Module defined USB EP0 standard request handler. The caller is
 * expected to ensure parameter validity. */
static ccg_status_t usb_ep0_std_rqt_handler(usb_setup_pkt_t *pkt)
{
    uint8_t target;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    target = (pkt->attrib & USB_TARGET_MASK);

    switch (pkt->cmd)
    {
        case USB_SC_GET_STATUS:

            if ((gl_usb.state != USB_STATE_CONFIGURED) &&
                    (target != USB_TARGET_DEVICE))
            {
                break;
            }

            if (target == USB_TARGET_ENDPT)
            {
                uint8_t ep = (pkt->index & 0x7F);
                uint16_t data = 0;
                if (ep <= USB_NUM_EP)
                {
                    if (ep != 0)
                    {
                        data = (USBSIE[(ep - 1)]->sie_epx_cr0 &
                                USBDEV_SIE_EPx_CR0_STALL) ? true : false;
                    }

                    status = usb_ep0_setup_write((uint8_t *)&data,
                            pkt->length, true, NULL);
                }
            }
            else /* (target == USB_TARGET_DEVICE) ||
                    (target == USB_TARGET_INTF) */
            {
                status = usb_ep0_setup_write ((uint8_t *)&gl_usb.dev_stat,
                        pkt->length, true, NULL);
            }
            break;

        case USB_SC_CLEAR_FEATURE:
            if ((gl_usb.state != USB_STATE_CONFIGURED) &&
                    (target != USB_TARGET_DEVICE))
            {
                break;
            }
            if (target == USB_TARGET_DEVICE)
            {
                uint8_t feature = (pkt->value & 0xFF);
                if ((feature == USB_REMOTE_WAKE) ||
                        (feature == USB_TEST_MODE))
                {
                    if (feature == USB_REMOTE_WAKE)
                    {
                        gl_usb.dev_stat &= 0xFD;
                    }

                    status = CCG_STAT_SUCCESS;
                }
            }
            else if ((target == USB_TARGET_ENDPT) &&
                    (pkt->value == USB_EP_HALT))
            {
                status = usb_ep_clear_stall ((pkt->index & 0x7F) - 1);
            }
            else /* Do nothing. */
            {
            }
            if (status == CCG_STAT_SUCCESS)
            {
                status = usb_ep0_send_recv_status ();
            }
            break;

        case USB_SC_SET_FEATURE:
            if ((gl_usb.state != USB_STATE_CONFIGURED) &&
                    (target != USB_TARGET_DEVICE))
            {
                break;
            }
            if (target == USB_TARGET_DEVICE)
            {
                uint8_t feature = (pkt->value & 0xFF);
                if ((feature == USB_REMOTE_WAKE) ||
                        (feature == USB_TEST_MODE))
                {
                    if (feature == USB_REMOTE_WAKE)
                    {
                        gl_usb.dev_stat |= 0x02;
                    }

                    status = CCG_STAT_SUCCESS;
                }
            }
            else if ((target == USB_TARGET_ENDPT) &&
                    (pkt->value == USB_EP_HALT))
            {
                status = usb_ep_set_stall ((pkt->index & 0x7F) - 1);
            }
            else /* Do nothing. */
            {
            }
            if (status == CCG_STAT_SUCCESS)
            {
                status = usb_ep0_send_recv_status ();
            }
            break;

        case USB_SC_GET_DESCRIPTOR:
            status = gl_usb.cfg.get_dscr_cb (pkt);
            break;

        case USB_SC_SET_ADDRESS:
            if ((pkt->value != 0) && (pkt->value <= USB_MAX_DEVICE_ADDR))
            {
                uint32_t value;

                /* Set the address only after responding
                 * to the setup request. */
                status = usb_ep0_send_recv_status();

                if (status == CCG_STAT_SUCCESS)
                {
                    status = usb_ep0_wait_for_ack();
                }
                if (status == CCG_STAT_SUCCESS)
                {
                    value = USBDEV->cr0 &
                        ~USBDEV_CR0_DEVICE_ADDRESS_MASK;
                    USBDEV->cr0 = value | pkt->value;
                    gl_usb.state = USB_STATE_ADDRESSED;

                    if(gl_usb.cfg.event_cb != NULL)
                    {
                        gl_usb.cfg.event_cb(gl_usb.state, 0);
                    }
                }
            }
            if (status != CCG_STAT_SUCCESS)
            {
                usb_ep0_set_stall();
                status = CCG_STAT_SUCCESS;
            }
            break;

        case USB_SC_SET_CONFIGURATION:
            if (pkt->value <= USB_NUM_CONFIGURATION)
            {
                usb_state_t state = USB_STATE_CONFIGURED;

                gl_usb.active_cfg = pkt->value;
                if (gl_usb.active_cfg != 0)
                {
                    gl_usb.state = USB_STATE_CONFIGURED;
                }
                else
                {
                    gl_usb.state = USB_STATE_ADDRESSED;
                    state = USB_STATE_UNCONFIGURED;
                }

                if(gl_usb.cfg.event_cb != NULL)
                {
                    gl_usb.cfg.event_cb(state, 0);
                }
                status = usb_ep0_send_recv_status();
            }
            break;

        case USB_SC_GET_CONFIGURATION:
            if (pkt->length == 1)
            {
                status = usb_ep0_setup_write((uint8_t *)&gl_usb.active_cfg,
                        1, true, NULL);
            }
            break;

        case USB_SC_SET_INTERFACE:
            if ((pkt->value < USB_NUM_ALT_INTERFACE)
                    && (pkt->index < USB_NUM_INTERFACE))
            {
                gl_usb.active_alt_inf[pkt->index] = pkt->value;
                status = usb_ep0_send_recv_status();
            }
            break;

        case USB_SC_GET_INTERFACE:
            if ((pkt->index < USB_NUM_INTERFACE) && (pkt->length == 1))
            {
                status = usb_ep0_setup_write((uint8_t *)&gl_usb.active_alt_inf[pkt->index],
                        1, true, NULL);
            }
            break;

        default:
            /* Do nothing. */
            break;
    }

    if (status == CCG_STAT_NOT_SUPPORTED)
    {
        if (gl_usb.cfg.fallback_rqt_cb != NULL)
        {
            status = gl_usb.cfg.fallback_rqt_cb(pkt);
        }
    }

    return status;
}

/* Internal setup packet handler. The caller is expected to ensure
 * parameter validity. The function is expected to be invoked from
 * the EP0 interrupt handler. */
static ccg_status_t usb_ep0_setup_pkt_handler(usb_setup_pkt_t *pkt)
{
    uint8_t type;
    ccg_status_t status = CCG_STAT_NOT_SUPPORTED;

    /* Setup packet is already copied into the
     * SRAM buffer. */
    type = pkt->attrib & USB_TYPE_MASK;

    if (type == USB_STANDARD_RQT)
    {
        status = usb_ep0_std_rqt_handler (pkt);
    }
    else if (type == USB_CLASS_RQT)
    {
        if (gl_usb.cfg.class_rqt_cb != NULL)
        {
            status = gl_usb.cfg.class_rqt_cb(pkt);
        }
    }
    else if (type == USB_VENDOR_RQT)
    {
        if (gl_usb.cfg.vendor_rqt_cb != NULL)
        {
            status = gl_usb.cfg.vendor_rqt_cb(pkt);
        }
    }
    else
    {
        /* Reserved type. Fail the request. */
    }

    return status;
}

ccg_status_t usb_ep_enable(usb_ep_index_t ep_index, bool is_out)
{
    uint16_t buf_offset;
    PUSBARB_REGS_T arb_p;
    PUSBSIE_REGS_T sie_p;

    if (gl_usb.ep_handle[ep_index].enabled)
    {
        return CCG_STAT_BUSY;
    }

    arb_p = USBARB[ep_index];
    sie_p = USBSIE[ep_index];
    buf_offset = (USB_EP_MAX_PKT_SIZE * ep_index);

    gl_usb.ep_handle[ep_index].toggle = false;

    /* Configure the endpoint. */
    arb_p->arb_epx_cfg = USBDEV_ARB_EPx_CFG_RESET_PTR |
        USBDEV_ARB_EPx_CFG_CRC_BYPASS;
    arb_p->arb_epx_sr = ~0;
    arb_p->arb_epx_int_en = 0;

    /* 8 Bit access */
    arb_p->arb_rwx_wa = arb_p->arb_rwx_ra = (buf_offset & 0xFF);
    arb_p->arb_rwx_wa_msb = arb_p->arb_rwx_ra_msb = (buf_offset >> 8);

    /*
     * Bad hardware. Set the toggle so that the following
     * read / write call shall toggle it again.
     */
    sie_p->sie_epx_cnt0 = USBDEV_SIE_EPx_CNT0_DATA_TOGGLE;
    sie_p->sie_epx_cnt1 = USB_EP_MAX_PKT_SIZE;
    if (is_out)
    {
        USBDEV->ep_type |= (uint8_t)(1 << ep_index);
        sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_NAK_OUT);
    }
    else /* IN EP */
    {
        USBDEV->ep_type &= ~(uint8_t)(1 << ep_index);
        sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_NAK_IN);
    }

    /* Store the configuration information. */
    gl_usb.ep_handle[ep_index].is_out = is_out;

    /* Store the configuration information. */
    gl_usb.ep_handle[ep_index].enabled = true;

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_disable(usb_ep_index_t ep_index)
{
    PUSBARB_REGS_T arb_p;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return CCG_STAT_NOT_READY;
    }

    arb_p = USBARB[ep_index];

    /* Disable the endpoint. */
    arb_p->arb_epx_cfg = USBDEV_ARB_EPx_CFG_RESET_PTR |
        USBDEV_ARB_EPx_CFG_CRC_BYPASS;
    arb_p->arb_epx_sr = ~0;
    arb_p->arb_epx_int_en = 0;
    USBSIE[ep_index]->sie_epx_cr0 = USBDEV_EP_MODE_DISABLE;

    /* Store the configuration information. */
    gl_usb.ep_handle[ep_index].enabled = false;

    return CCG_STAT_SUCCESS;
}

bool usb_ep_is_ready(usb_ep_index_t ep_index)
{
    uint32_t value;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    value = USBSIE[ep_index]->sie_epx_cr0;
    if (value & USBDEV_SIE_EPx_CR0_STALL)
    {
        return false;
    }

    if (gl_usb.ep_handle[ep_index].is_out)
    {
        if (((value & USBDEV_SIE_EPx_CR0_MODE_MASK) == USBDEV_EP_MODE_NAK_OUT)
                && (value & USBDEV_SIE_EPx_CR0_ACKED_TXN))
        {
            return true;
        }
    }
    else
    {
        if ((value & USBDEV_SIE_EPx_CR0_MODE_MASK) == USBDEV_EP_MODE_NAK_IN)
        {
            return true;
        }
    }

    return false;
}

ccg_status_t usb_ep_flush(usb_ep_index_t ep_index)
{
    bool is_ready;
    uint16_t buf_offset;
    PUSBARB_REGS_T arb_p;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return CCG_STAT_NOT_READY;
    }

    buf_offset = (USB_EP_MAX_PKT_SIZE * ep_index);
    arb_p = USBARB[ep_index];

    is_ready = usb_ep_is_ready (ep_index);

    if (!(gl_usb.ep_handle[ep_index].is_out | is_ready))
    {
        if (gl_usb.ep_handle[ep_index].toggle)
        {
            gl_usb.ep_handle[ep_index].toggle = false;
        }
        else
        {
            gl_usb.ep_handle[ep_index].toggle = true;
        }

        USBSIE[ep_index]->sie_epx_cr0 = (USBDEV_EP_MODE_NAK_IN);
    }

    arb_p->arb_rwx_wa = arb_p->arb_rwx_ra = (buf_offset & 0xFF);
    arb_p->arb_rwx_wa_msb = arb_p->arb_rwx_ra_msb = (buf_offset >> 8);

    if (gl_usb.ep_handle[ep_index].is_out & is_ready)
    {
        /* Update the next data toggle. */
        bool toggle = (USBSIE[ep_index]->sie_epx_cnt0 & USBDEV_SIE_EPx_CNT0_DATA_TOGGLE) ?
            true : false;
        /* Check if there is a PID mismatch. */
        if (toggle == gl_usb.ep_handle[ep_index].toggle)
        {
            gl_usb.ep_handle[ep_index].toggle = (toggle) ? false : true;
        }

        usb_ep_queue_read_single (ep_index);
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_set_stall(usb_ep_index_t ep_index)
{
    uint32_t value;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return CCG_STAT_NOT_READY;
    }

    /* Stall the endpoint. */
    value = (gl_usb.ep_handle[ep_index].is_out) ?
        USBDEV_EP_MODE_ACK_OUT : USBDEV_EP_MODE_ACK_IN;
    value |= USBDEV_SIE_EPx_CR0_STALL;
    USBSIE[ep_index]->sie_epx_cr0 = value;

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_clear_stall(usb_ep_index_t ep_index)
{
    uint16_t buf_offset;
    PUSBARB_REGS_T arb_p;
    PUSBSIE_REGS_T sie_p;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return CCG_STAT_NOT_READY;
    }

    buf_offset = (USB_EP_MAX_PKT_SIZE * ep_index);
    arb_p = USBARB[ep_index];
    sie_p = USBSIE[ep_index];
    gl_usb.ep_handle[ep_index].toggle = false;

    /* Reset the endpoint. */
    arb_p->arb_epx_cfg = USBDEV_ARB_EPx_CFG_RESET_PTR |
        USBDEV_ARB_EPx_CFG_CRC_BYPASS;
    arb_p->arb_rwx_wa = arb_p->arb_rwx_ra = (buf_offset & 0xFF);
    arb_p->arb_rwx_wa_msb = arb_p->arb_rwx_ra_msb = (buf_offset >> 8);

    /* Bad hardware. Set the toggle so that the following
     * read / write call shall toggle it again. */
    sie_p->sie_epx_cnt0 = USBDEV_SIE_EPx_CNT0_DATA_TOGGLE;
    sie_p->sie_epx_cr0 = (gl_usb.ep_handle[ep_index].is_out) ?
        USBDEV_EP_MODE_NAK_OUT : USBDEV_EP_MODE_NAK_IN;

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_queue_read_single(usb_ep_index_t ep_index)
{
    PUSBSIE_REGS_T sie_p;

    sie_p = USBSIE[ep_index];

    if ((sie_p->sie_epx_cr0 & USBDEV_SIE_EPx_CR0_MODE_MASK)
            != USBDEV_EP_MODE_NAK_OUT)
    {
        return CCG_STAT_NOT_READY;
    }

    /* Qeueue a read to the endpoint. */
    sie_p->sie_epx_cnt1 = USB_EP_MAX_PKT_SIZE;
    sie_p->sie_epx_cnt0 = 0;

    sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_ACK_OUT);

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_read_single(usb_ep_index_t ep_index, uint8_t *data, uint8_t *count)
{
    uint8_t index, ep_cnt;
    bool toggle;
    PUSBSIE_REGS_T sie_p;

    if ((data == NULL) || (count == NULL))
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    sie_p = USBSIE[ep_index];

    toggle = (sie_p->sie_epx_cnt0 & USBDEV_SIE_EPx_CNT0_DATA_TOGGLE) ?
        true : false;
    /* Check if there is a PID mismatch. */
    if (toggle ^ gl_usb.ep_handle[ep_index].toggle)
    {
        return CCG_STAT_TRANS_FAILURE;
    }

    ep_cnt = (sie_p->sie_epx_cnt1 - USB_CRC_SIZE);

    /* Read the data from the USB memory. */
    for (index = 0; index < ep_cnt; index++)
    {
        data[index] = USBDEV_ARB_RWx_DR(ep_index);
    }

    *count = ep_cnt;

    /* Update the next data toggle. */
    gl_usb.ep_handle[ep_index].toggle = (toggle) ? false : true;

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_send_zlp(usb_ep_index_t ep_index)
{
    PUSBSIE_REGS_T sie_p;

    sie_p = USBSIE[ep_index];

    /* Commit the data to USB. */
    sie_p->sie_epx_cnt1 = 0;
    if (gl_usb.ep_handle[ep_index].toggle)
    {
        sie_p->sie_epx_cnt0 = USBDEV_SIE_EPx_CNT0_DATA_TOGGLE;
        gl_usb.ep_handle[ep_index].toggle = false;

    }
    else
    {
        sie_p->sie_epx_cnt0 = 0;
        gl_usb.ep_handle[ep_index].toggle = true;
    }

    USBARB[ep_index]->arb_epx_cfg |= USBDEV_ARB_EPx_CFG_IN_DATA_RDY;
    sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_ACK_IN);

    return CCG_STAT_SUCCESS;
}

ccg_status_t usb_ep_write_single(usb_ep_index_t ep_index, uint8_t *data, uint8_t count)
{
    uint8_t index;
    volatile uint32_t *reg_addr;
    PUSBSIE_REGS_T sie_p;

    if (data == NULL)
    {
        return CCG_STAT_INVALID_ARGUMENT;
    }

    sie_p = USBSIE[ep_index];

    if ((sie_p->sie_epx_cr0 & USBDEV_SIE_EPx_CR0_MODE_MASK)
            != USBDEV_EP_MODE_NAK_IN)
    {
        return CCG_STAT_NOT_READY;
    }

    reg_addr = &USBARB[ep_index]->arb_rwx_dr;

    /* Write the data to the USB memory. */
    for (index = 0; index < count; index++)
    {
        *reg_addr = data[index];
    }

    /* Commit the data to USB. */
    sie_p->sie_epx_cnt1 = count;
    if (gl_usb.ep_handle[ep_index].toggle)
    {
        sie_p->sie_epx_cnt0 = USBDEV_SIE_EPx_CNT0_DATA_TOGGLE;
        gl_usb.ep_handle[ep_index].toggle = false;
    }
    else
    {
        sie_p->sie_epx_cnt0 = 0;
        gl_usb.ep_handle[ep_index].toggle = true;
    }

    USBARB[ep_index]->arb_epx_cfg |= USBDEV_ARB_EPx_CFG_IN_DATA_RDY;
    sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_ACK_IN);

    return CCG_STAT_SUCCESS;
}

/* Internal EP0 interrupt handler */
static void usb_ep0_int_handler(void)
{
    uint32_t value;
    uint32_t timeout;
    uint8_t count = 0;
    ccg_status_t status = CCG_STAT_SUCCESS;

    /* Clear the interrupt. */
    USBDEV_BCD->intr_sie = USBDEV_BCD_USB_INTR_SIE_EP0_INTR;

    /* Check if the interrupt was triggered
     * due to a new setup packet. */
    value = USBDEV->ep0_cr;
    if (value & USBDEV_EP0_CR_SETUP_RCVD)
    {
        /* Wait until the setup packet is fully received. */
        timeout = USB_REG_LOCK_TIMEOUT;
        do
        {
            /* Any writes to the register will clear the bit. */
            USBDEV->ep0_cr = (USBDEV_EP_MODE_NAK_INOUT |
                    USBDEV_EP0_CR_SETUP_RCVD);
            timeout--;
            if (timeout == 0)
            {
                /* 
                 * This is never expected. If this happens, hardware is broken.
                 * Continue handling the packet so that we exit the state machine
                 */
                break;
            }

        } while (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD);

        /* Copy the setup packet. */
        usb_copy_setup_pkt ((uint8_t *)&gl_usb.setup_pkt);

        /* Now update the state machine. */
        gl_usb.ep0_state = USB_EP0_STATE_SETUP;
        gl_usb.ep0_buffer = NULL;
        gl_usb.ep0_zlp_rqd = false;
        gl_usb.ep0_toggle = true;

        status = usb_ep0_setup_pkt_handler((usb_setup_pkt_t *)&gl_usb.setup_pkt);

        /* Update the register value. */
        /* Stall EP0 in case of error. */
        if (status != CCG_STAT_SUCCESS)
        {
            usb_ep0_set_stall();
        }
    }
    else if (value & USBDEV_EP0_CR_OUT_RCVD)
    {
        /* Make sure that the interrupt is cleared. */
        timeout = USB_REG_LOCK_TIMEOUT;
        do
        {
            /* Any writes to the register will clear the bit. */
            if (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD)
            {
                return;
            }
            USBDEV->ep0_cr = (USBDEV_EP_MODE_NAK_INOUT |
                    USBDEV_EP0_CR_OUT_RCVD);
            timeout--;
            if (timeout == 0)
            {
                /* 
                 * This is never expected. If this happens, hardware is broken.
                 * Continue handling the packet so that we exit the state machine
                 */
                break;
            }

        } while (USBDEV->ep0_cr & USBDEV_EP0_CR_OUT_RCVD);

        if ((gl_usb.ep0_state == USB_EP0_STATE_DATA_OUT) &&
                (gl_usb.ep0_buffer != NULL))
        {
            /* Variable created to satisfy compiler. */
            uint8_t *ptr = &count;
            /* A data packet has been received. Check
             * if additional data packets need to be
             * received. */
            status = usb_ep0_read_packet (gl_usb.ep0_buffer, ptr);

            if (status == CCG_STAT_SUCCESS)
            {
                gl_usb.ep0_buffer += count;
                gl_usb.ep0_length -= count;

                /* If this was a SLP / ZLP, this is the last transfer.
                 * Otherwise queue next read. */
                if ((count == USB_EP0_SIZE) &&
                        (gl_usb.ep0_length != 0))
                {
                    status = usb_ep0_queue_read();
                }
                else /* Last transfer. */
                {
                    if (gl_usb.ep0_xfer_cb != NULL)
                    {
                        status = gl_usb.ep0_xfer_cb ((usb_setup_pkt_t *)&gl_usb.setup_pkt);
                    }
                    /* Send the status phase if this is the last packet.
                     * Do not send the status phase if this was a partial
                     * request. */
                    if ((gl_usb.ep0_last) && (status == CCG_STAT_SUCCESS))
                    {
                        status = usb_ep0_send_recv_status();
                    }
                }
            }
            else if (status == CCG_STAT_TRANS_FAILURE)
            {
                /* If the data toggle was wrong, retry. Leave it to
                 * host to decide the number of retries. */
                status = usb_ep0_queue_read();
            }
            else
            {
                /* Do nothing. */
            }

            if (status != CCG_STAT_SUCCESS)
            {
                usb_ep0_set_stall();
            }
        }
    }

    else if (value & USBDEV_EP0_CR_IN_RCVD)
    {
        /* Make sure that the interrupt is cleared. */
        timeout = USB_REG_LOCK_TIMEOUT;
        do
        {
            /* Any writes to the register will clear the bit. */
            if (USBDEV->ep0_cr & USBDEV_EP0_CR_SETUP_RCVD)
            {
                return;
            }
            USBDEV->ep0_cr = (USBDEV_EP_MODE_NAK_INOUT |
                    USBDEV_EP0_CR_IN_RCVD);
            timeout--;
            if (timeout == 0)
            {
                /* 
                 * This is never expected. If this happens, hardware is broken.
                 * Continue handling the packet so that we exit the state machine
                 */
                break;
            }

        } while (USBDEV->ep0_cr & USBDEV_EP0_CR_IN_RCVD);

        if ((gl_usb.ep0_state == USB_EP0_STATE_DATA_IN) &&
                (gl_usb.ep0_buffer != NULL))
        {
            /* A data packet has been sent out.
             * If the last packet has been sent out,
             * change to status mode. Otherwise queue
             * the next packet. */
            if (gl_usb.ep0_length != 0)
            {
                count = GET_MIN(gl_usb.ep0_length, USB_EP0_SIZE);
                status = usb_ep0_send_packet(gl_usb.ep0_buffer, count);

                gl_usb.ep0_buffer += count;
                gl_usb.ep0_length -= count;
            }
            else if (gl_usb.ep0_zlp_rqd) /* Terminate with ZLP. */
            {
                status = usb_ep0_send_packet(gl_usb.ep0_buffer, 0);
                /* Clear the flag. */
                gl_usb.ep0_zlp_rqd = false;
            }
            else /* Last transfer. */
            {
                if (gl_usb.ep0_xfer_cb != NULL)
                {
                    gl_usb.ep0_xfer_cb ((usb_setup_pkt_t *)&gl_usb.setup_pkt);
                }
                /* Send the status phase if this is the last packet.
                 * Do not send the status phase if this was a partial
                 * request. */
                if (gl_usb.ep0_last)
                {
                    status = usb_ep0_send_recv_status();
                }
            }

            if (status != CCG_STAT_SUCCESS)
            {
                usb_ep0_set_stall();
            }
        }
    }
}

/*
 * Function to enable the SOF detection logic. It is used in
 * conjuntion with resume activity so that false suspend is
 * not detected.
 */
#if (USB_SUSPEND_CHECK_DISABLE == 0)
static void usb_sof_intr_enable(void)
{
    gl_usb.suspend_check = false;
    timer_stop(USB_TIMER_INSTANCE, USB_SUSPEND_TIMER);
    USBDEV_BCD->intr_sie = USBDEV_BCD_USB_INTR_SIE_SOF_INTR;
    USBDEV_BCD->intr_sie_mask |= USBDEV_BCD_USB_INTR_SIE_SOF_INTR;
}
#endif /* USB_SUSPEND_CHECK_DISABLE */

/* Internal USB reset interrupt handler */
static void usb_reset_int_handler(void)
{
    USBDEV->cr0 = USBDEV_CR0_USB_ENABLE;
    USBDEV->ep0_cr  = USBDEV_EP_MODE_NAK_INOUT;
    gl_usb.state = USB_STATE_RESET;
    gl_usb.dev_stat &= ~(0x02);

#if (USB_SUSPEND_CHECK_DISABLE == 0)
    /* Ensure to stop the suspend check and to start the SOF interrupt. */
    if (gl_usb.suspend_check != false)
    {
        usb_sof_intr_enable();
    }
#endif /* USB_SUSPEND_CHECK_DISABLE */

    if (gl_usb.cfg.event_cb != NULL)
    {
        gl_usb.cfg.event_cb(gl_usb.state, 0);
    }

    /* Clear the interrupt. */
    USBDEV_BCD->intr_sie = USBDEV_BCD_USB_INTR_SIE_BUS_RESET_INTR;
}

#if (USB_SUSPEND_CHECK_DISABLE == 0)
/*
 * The structure is used to save and restore non-retention registers across
 * deep sleep. The structure is created locally to ensure that it is not used
 * elsewhere. The member names are same as the register name to avoid confusion.
 * The number of registers is limited to save RAM. Known data is to be populated
 * manually.
 */
typedef struct usb_reg_save_t
{
    uint8_t ep_type;
    struct 
    {
        uint8_t sie_epx_cnt0;
        uint8_t sie_epx_cr0;
    } sie_epx_reg[8];

} usb_reg_save_t;

static volatile usb_reg_save_t gl_reg_save;

/*
 * USB registers are non retention and have to be saved and restored across
 * deep sleep. The caller is expected to ensure that USB is in actual suspended
 * state before invoking this function. If not, it will result in the registers
 * getting corrupted. Interrupt locks are expected to be done by the caller.
 */
static void usb_save_state(void)
{
    uint8_t index;

    gl_reg_save.ep_type = (uint8_t)USBDEV->ep_type;

    for (index = 0; index < USB_NUM_EP; index++)
    {
        gl_reg_save.sie_epx_reg[index].sie_epx_cnt0 =
            (uint8_t)USBSIE[index]->sie_epx_cnt0;
        gl_reg_save.sie_epx_reg[index].sie_epx_cr0 =
            (uint8_t)USBSIE[index]->sie_epx_cr0;
    }
}

/*
 * USB registers are non retention and have to be saved and restored across
 * deep sleep. The caller is expected to ensure that USB was in actual suspended
 * state and this is the first task on the resume operation. If not, it will
 * result in the registers getting corrupted. Interrupt locks are expected to
 * be done by the caller.
 */
static void usb_restore_state(void)
{
    uint8_t index;
    uint16_t buf_offset;

    /*
     * USB clock and PHY suspend is expected to be restored before
     * this call as this needs to be done as soon as possible.
     */

    /* Update the reset counter value. */
    USBDEV->bus_rst_cnt = USB_BUS_RST_CNT_VALUE;
    /* Set the dedicated endpoint buffer sizes as 64 bytes. */
    USBDEV->buf_size = USB_BUF_SIZE_64BYTE;

    /* Re-initialize EP0. */
    usb_ep0_init();

    /* Restore EPx registers. */
    USBDEV->ep_type = gl_reg_save.ep_type;
    for (index = 0; index < USB_NUM_EP; index++)
    {
        USBARB[index]->arb_epx_cfg = USBDEV_ARB_EPx_CFG_RESET_PTR |
        USBDEV_ARB_EPx_CFG_CRC_BYPASS;
        buf_offset = (USB_EP_MAX_PKT_SIZE * index);
        USBARB[index]->arb_rwx_wa = USBARB[index]->arb_rwx_ra =
            (buf_offset & 0xFF);
        USBARB[index]->arb_rwx_wa_msb = USBARB[index]->arb_rwx_ra_msb =
            (buf_offset >> 8);

        USBSIE[index]->sie_epx_cnt0 =
            gl_reg_save.sie_epx_reg[index].sie_epx_cnt0;
        USBSIE[index]->sie_epx_cnt1 = USB_EP_MAX_PKT_SIZE;
        USBSIE[index]->sie_epx_cr0 = 
            gl_reg_save.sie_epx_reg[index].sie_epx_cr0;

        /*
         * If EP interrupts need to be used, we need to save EPx_INT
         * configurations as well.
         */
    }
}

static void usb_suspend_timer_cb(uint8_t instance, timer_id_t id)
{
    (void)instance;
    (void)id;

    if (((gl_usb.state == USB_STATE_ADDRESSED) ||
                (gl_usb.state == USB_STATE_CONNECTED) ||
                (gl_usb.state == USB_STATE_CONFIGURED)) &&
            (USBDEV->usbio_cr1 & USBDEV_USBIO_CR1_DPO) &&
            (gl_usb.suspend_check != false))
    {
        if (USBDEV->cr1 & USBDEV_CR1_BUS_ACTIVITY)
        {
            /* There has been some bus activity. Re-do the scan. */
            USBDEV->cr1 &= ~USBDEV_CR1_BUS_ACTIVITY;
        }
        else
        {
            /*
             * If the SOF count has not changed within the timeout, then this is
             * a USB bus suspend indication. Suspend the PHY and indicate to the
             * application.
             */
            if (gl_usb.sof_data == USBDEV->sof16)
            {
                /*
                 * Enable the resume interrupt - Use the GPIO interrupt vector
                 * directly to control the enable / disable as this is the only
                 * GPIO for the port used for interrupt.
                 */
                gpio_clear_intr(USB_DP_GPIO);
                gpio_int_set_config(USB_DP_GPIO, GPIO_INTR_FALLING);
                CyIntEnable(USB_GPIO_INTR_VECTOR);

                /*
                 * There is case where the DP went low after the check. To avoid
                 * race, check if the DP is low. Since edge trigger cannot be 
                 * indicated, we need to avoid detecting a suspend.
                 */
                if (gpio_read_value(USB_DP_GPIO))
                {
                    gl_usb.prev_state = gl_usb.state;
                    gl_usb.state = USB_STATE_SUSPENDED;
                    USBDEV_BCD->power_ctrl |= (USBDEV_BCD_USB_POWER_CTRL_SUSPEND |
                            USBDEV_BCD_USB_POWER_CTRL_SUSPEND_DEL);

                    /*
                     * Save non-retention registers so that it can be restored
                     * on resume. This call should be done only from here.
                     */
                    usb_save_state();

                    gl_usb.suspend_check = false;

                    /* Indicate the USB suspend state to the application. */
                    if (gl_usb.cfg.event_cb != NULL)
                    {
                        gl_usb.cfg.event_cb(gl_usb.state, 0);
                    }
                }
                else
                {
                    /*
                     * Resume has already started or this was not a proper suspend.
                     * To avoid another race, ensure that suspend check is started
                     * only after SOFs have come back.
                     */
                    usb_sof_intr_enable();
                }
            }
        }
    }

    /* Re-start timer if required. */
    if (gl_usb.suspend_check != false)
    {
        gl_usb.sof_data = USBDEV->sof16;
        timer_start(USB_TIMER_INSTANCE, USB_SUSPEND_TIMER,
                USB_SUSPEND_TIMER_PERIOD, usb_suspend_timer_cb);
    }
}

static void usb_sof_int_handler(void)
{
    /*
     * The first SOF interrupt is used to start the USB suspend detection.
     * There are usually some delay before some hosts can resume and start
     * sending SOFs. If this is not done, this will result in us detecting
     * a false USB suspend and locking out on the bus. Start a timer for 
     * checking for SOF signals.
     */
    gl_usb.suspend_check = true;
    gl_usb.sof_data = USBDEV->sof16;
    timer_start(USB_TIMER_INSTANCE, USB_SUSPEND_TIMER,
            USB_SUSPEND_TIMER_PERIOD, usb_suspend_timer_cb);

    /* Clear the interrupt and disable it. */
    USBDEV_BCD->intr_sie = USBDEV_BCD_USB_INTR_SIE_SOF_INTR;
    USBDEV_BCD->intr_sie_mask &= ~USBDEV_BCD_USB_INTR_SIE_SOF_INTR;
}

static void usb_resume_int_handler(void)
{
    /*
     * On receiving a resume, the firmware should re-enable the PHY
     * and make it ready for functionality.
     */
    if (gl_usb.state == USB_STATE_SUSPENDED)
    {
        gl_usb.state = gl_usb.prev_state;

        /* First restore the USB clock. */
        USBDEV->usb_clk_en = USBDEV_USB_CLK_EN_CSR_CLK_EN;
        CyDelayUs(1);

        /* Clear PHY suspend. */
        USBDEV_BCD->power_ctrl &= ~(USBDEV_BCD_USB_POWER_CTRL_SUSPEND);
        CyDelayUs(2);
        USBDEV_BCD->power_ctrl &= ~(USBDEV_BCD_USB_POWER_CTRL_SUSPEND_DEL);

        /* 
         * Now restore the non-retention registers. This call should be done
         * only from here.
         */
        usb_restore_state();

        /* Indicate the event to application. */
        if (gl_usb.cfg.event_cb != NULL)
        {
            gl_usb.cfg.event_cb(USB_STATE_RESUMED, 0);
        }
    }

    /* Re-enable the SOF interrupts if not already enabled. */
    if (gl_usb.suspend_check == false)
    {
        usb_sof_intr_enable();
    }
}

static void usb_gpio_intr_handler(void)
{
    /* Disable the GPIO interrupt. */
    gpio_int_set_config(USB_DP_GPIO, GPIO_INTR_DISABLE);
    gpio_clear_intr(USB_DP_GPIO);
    CyIntClearPending(USB_DP_GPIO);
    CyIntDisable(USB_DP_GPIO);

    usb_resume_int_handler();
}
#endif /* USB_SUSPEND_CHECK_DISABLE */

/* Internal USB_INTR_HIGH interrupt handler */
static void usb_intr_high_handler(void)
{
    gl_usb.int_event |= USBDEV_BCD->intr_sie_masked;
    /*
     * Handle the reset interrupt in the interrupt context to
     * avoid race with GPIO and timer interrupts.
     */
    if (gl_usb.int_event & USBDEV_BCD_USB_INTR_SIE_BUS_RESET_INTR)
    {
        usb_reset_int_handler();
    }

    CyIntDisable(USB_INTR_HIGH_VECTOR_LOCATION);
}

/* Internal USB_INTR_LOW interrupt handler */
static void usb_intr_low_handler(void)
{
    /* Flag EP interrupts. */
    gl_usb.int_event |= USBDEV_BCD->intr_cause_lo;
    CyIntDisable(USB_INTR_LOW_VECTOR_LOCATION);
}

void usb_task(void)
{
    uint32_t lock_state;

    if(gl_usb.state <= USB_STATE_DISABLED)
    {
        return;
    }

    /* Lock out USB interrupts before handling this section. */
    lock_state = usb_intr_lock();

    /* Check for any pending interrupts to be handled. */
    if ((gl_usb.int_event & USB_SIE_INTR_ALL_MASK) != 0)
    {
        /* If the event is set, then the interrupt is disabled. So,
         * the handler does not need to lock interrupts. */
        if (gl_usb.int_event & USBDEV_BCD_USB_INTR_SIE_EP0_INTR)
        {
            usb_ep0_int_handler();
        }
#if (USB_SUSPEND_CHECK_DISABLE == 0)
        if (gl_usb.int_event & USBDEV_BCD_USB_INTR_SIE_SOF_INTR)
        {
            usb_sof_int_handler();
        }
#endif /* USB_SUSPEND_CHECK_DISABLE */

        /* Clear pending events as it has been handled. */
        gl_usb.int_event &= ~(USB_SIE_INTR_ALL_MASK);
        /* Re-enable the USB interrupts. */
        CyIntClearPending(USB_INTR_HIGH_VECTOR_LOCATION);
        CyIntEnable(USB_INTR_HIGH_VECTOR_LOCATION);
    }
    if ((gl_usb.int_event & USB_EP_INTR_ALL_MASK) != 0)
    {
        /* Implement USB EP interrupt handlers. */
    }

    /* Release lock on USB interrupts. */
    usb_intr_unlock(lock_state);
}

/* [] */
