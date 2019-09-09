/*******************************************************************************
 * File Name: ccg4pd_regs.h
 *
 * Version: M0S8-Product-CCG4PD3.xls
 *
 * Description:
 * Cypress product header file.
 *
 * This file is auto generated from the register map spreadsheet.
 * DO NOT MODIFY THIS FILE.
 *
 *******************************************************************************
 * © (2014-2018), Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
 * protected by and subject to worldwide patent protection (United States and
 * foreign), United States copyright laws and international treaty provisions.
 * Cypress hereby grants to licensee a personal, non-exclusive,
 * non-transferable license to copy, use, modify, create derivative works of,
 * and compile the Cypress source code and derivative works for the sole
 * purpose of creating custom software in support of licensee product, such
 * licensee product to be used only in conjunction with Cypress's integrated
 * circuit as specified in the applicable agreement. Any reproduction,
 * modification, translation, compilation, or representation of this Software
 * except as specified above is prohibited without the express written
 * permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Cypress reserves the right to make changes to the Software without notice.
 * Cypress does not assume any liability arising out of the application or use
 * of Software or any product or circuit described in the Software.
 * Cypress does not authorize its products for use as critical components in
 * any products where a malfunction or failure may reasonably be expected to
 * result in significant injury or death ("High Risk Product"). By including
 * Cypress's product in a High Risk Product, the manufacturer of such system or
 * application assumes all risk of such use and in doing so indemnifies Cypress
 * against all liability.
 *
 * Use of this Software may be limited by and subject to the applicable Cypress
 * software license agreement.
 ******************************************************************************/
#ifndef _CCG4PD_REGS_H_
#define _CCG4PD_REGS_H_

#include <stdint.h>
#include <stdbool.h>


#define SFLASH_BASE_ADDR                                 (0x0ffff000)

typedef struct __attribute__((__packed__)) {
    volatile uint8_t  prot_row[64];                       /* 0x0ffff000 */
    volatile uint8_t  rsrvd0[191];
    volatile uint8_t  prot_protection;                    /* 0x0ffff0ff */
    volatile uint8_t  av_pairs_8b[128];                   /* 0x0ffff100 */
    volatile uint32_t rsrvd1[32];
    volatile uint32_t av_pairs_32b[16];                   /* 0x0ffff200 */
    volatile uint32_t cpuss_wounding;                     /* 0x0ffff240 */
    volatile uint32_t silicon_id;                         /* 0x0ffff244 */
    volatile uint16_t cpuss_priv_ram;                     /* 0x0ffff248 */
    volatile uint16_t cpuss_priv_rom_brom;                /* 0x0ffff24a */
    volatile uint16_t cpuss_priv_flash;                   /* 0x0ffff24c */
    volatile uint16_t cpuss_priv_rom_srom;                /* 0x0ffff24e */
    volatile uint16_t hib_key_delay;                      /* 0x0ffff250 */
    volatile uint16_t dpslp_key_delay;                    /* 0x0ffff252 */
    volatile uint8_t  swd_config;                         /* 0x0ffff254 */
    volatile uint8_t  rsrvd2[3];
    volatile uint32_t swd_listen;                         /* 0x0ffff258 */
    volatile uint32_t flash_start;                        /* 0x0ffff25c */
    volatile uint8_t  rsrvd3[9];
    volatile uint8_t  skip_checksum;                      /* 0x0ffff269 */
    volatile uint8_t  initial_pwr_bg_trim1;               /* 0x0ffff26a */
    volatile uint8_t  initial_pwr_bg_trim1_inv;           /* 0x0ffff26b */
    volatile uint8_t  initial_pwr_bg_trim2;               /* 0x0ffff26c */
    volatile uint8_t  initial_pwr_bg_trim2_inv;           /* 0x0ffff26d */
    volatile uint8_t  initial_spcif_trim_m0_dac0;         /* 0x0ffff26e */
    volatile uint8_t  initial_spcif_trim_m0_dac0_inv;     /* 0x0ffff26f */
    volatile uint8_t  prot_virginkey[8];                  /* 0x0ffff270 */
    volatile uint8_t  die_lot[3];                         /* 0x0ffff278 */
    volatile uint8_t  die_wafer;                          /* 0x0ffff27b */
    volatile uint8_t  die_x;                              /* 0x0ffff27c */
    volatile uint8_t  die_y;                              /* 0x0ffff27d */
    volatile uint8_t  die_sort;                           /* 0x0ffff27e */
    volatile uint8_t  die_minor;                          /* 0x0ffff27f */
    volatile uint32_t rsrvd4[32];
    volatile uint8_t  pe_te_data[32];                     /* 0x0ffff300 */
    volatile uint32_t pp;                                 /* 0x0ffff320 */
    volatile uint32_t e;                                  /* 0x0ffff324 */
    volatile uint32_t p;                                  /* 0x0ffff328 */
    volatile uint32_t ea_e;                               /* 0x0ffff32c */
    volatile uint32_t ea_p;                               /* 0x0ffff330 */
    volatile uint32_t es_e;                               /* 0x0ffff334 */
    volatile uint32_t es_p_eo;                            /* 0x0ffff338 */
    volatile uint8_t  rsrvd5[2];
    volatile uint8_t  imo_trim_usbmode_24;                /* 0x0ffff33e */
    volatile uint8_t  imo_trim_usbmode_48;                /* 0x0ffff33f */
    volatile uint32_t rsrvd6[3];
    volatile uint8_t  imo_tctrim_lt[25];                  /* 0x0ffff34c */
    volatile uint8_t  imo_trim_lt[25];                    /* 0x0ffff365 */
    volatile uint8_t  rsrvd7[128];
    volatile uint16_t checksum;                           /* 0x0ffff3fe */
} SFLASH_REGS_T, *PSFLASH_REGS_T;

#define SFLASH        ((PSFLASH_REGS_T) SFLASH_BASE_ADDR)


#define PERI_BASE_ADDR                                   (0x40010000)

typedef struct {
    volatile uint32_t div_cmd;                            /* 0x40010000 */
    volatile uint32_t rsrvd0[63];
    volatile uint32_t pclk_ctl[14];                       /* 0x40010100 */
    volatile uint32_t rsrvd1[50];
    volatile uint32_t div_8_ctl[6];                       /* 0x40010200 */
    volatile uint32_t rsrvd2[58];
    volatile uint32_t div_16_ctl[4];                      /* 0x40010300 */
    volatile uint32_t rsrvd3[60];
    volatile uint32_t div_16_5_ctl[4];                    /* 0x40010400 */
} PERI_REGS_T, *PPERI_REGS_T;

#define PERI        ((PPERI_REGS_T) PERI_BASE_ADDR)


#define HSIOM_BASE_ADDR                                  (0x40020000)
#define HSIOM0_BASE_ADDR                                 (0x40020000)
#define HSIOM1_BASE_ADDR                                 (0x40020100)
#define HSIOM2_BASE_ADDR                                 (0x40020200)
#define HSIOM3_BASE_ADDR                                 (0x40020300)
#define HSIOM4_BASE_ADDR                                 (0x40020400)

typedef struct {
    volatile uint32_t port_sel;                           /* 0x40020000 */
    volatile uint32_t rsrvd0[2111];
    volatile uint32_t amux_split_ctl[2];                  /* 0x40022100 */
} HSIOM_REGS_T, *PHSIOM_REGS_T;

#define HSIOM0       ((PHSIOM_REGS_T) HSIOM0_BASE_ADDR)
#define HSIOM1       ((PHSIOM_REGS_T) HSIOM1_BASE_ADDR)
#define HSIOM2       ((PHSIOM_REGS_T) HSIOM2_BASE_ADDR)
#define HSIOM3       ((PHSIOM_REGS_T) HSIOM3_BASE_ADDR)
#define HSIOM4       ((PHSIOM_REGS_T) HSIOM4_BASE_ADDR)

/*
 * NOTE:
 * There are multiple instances of the above register section.
 * The following structure pointer array needs to be defined in
 * a source file before using it.
 * const PHSIOM_REGS_T HSIOM[5] = {HSIOM0, HSIOM1, HSIOM2, HSIOM3, HSIOM4};
 */
extern const PHSIOM_REGS_T HSIOM[];


#define SRSSLT_BASE_ADDR                                 (0x40030000)

typedef struct {
    volatile uint32_t pwr_control;                        /* 0x40030000 */
    volatile uint32_t pwr_key_delay;                      /* 0x40030004 */
    volatile uint32_t pwr_adft_select;                    /* 0x40030008 */
    volatile uint32_t pwr_ddft_select;                    /* 0x4003000c */
    volatile uint32_t pwr_ddft_xres;                      /* 0x40030010 */
    volatile uint32_t tst_mode;                           /* 0x40030014 */
    volatile uint32_t tst_ddft_ctrl;                      /* 0x40030018 */
    volatile uint32_t tst_trim_cntr1;                     /* 0x4003001c */
    volatile uint32_t tst_trim_cntr2;                     /* 0x40030020 */
    volatile uint32_t tst_adft_ctrl;                      /* 0x40030024 */
    volatile uint32_t clk_select;                         /* 0x40030028 */
    volatile uint32_t clk_ilo_config;                     /* 0x4003002c */
    volatile uint32_t clk_imo_config;                     /* 0x40030030 */
    volatile uint32_t clk_dft_select;                     /* 0x40030034 */
    volatile uint32_t wdt_disable_key;                    /* 0x40030038 */
    volatile uint32_t wdt_counter;                        /* 0x4003003c */
    volatile uint32_t wdt_match;                          /* 0x40030040 */
    volatile uint32_t srss_intr;                          /* 0x40030044 */
    volatile uint32_t srss_intr_set;                      /* 0x40030048 */
    volatile uint32_t srss_intr_mask;                     /* 0x4003004c */
    volatile uint32_t srss_adft_control;                  /* 0x40030050 */
    volatile uint32_t res_cause;                          /* 0x40030054 */
    volatile uint32_t res_dft;                            /* 0x40030058 */
    volatile uint32_t rsrvd0[937];
    volatile uint32_t pwr_bg_trim1;                       /* 0x40030f00 */
    volatile uint32_t pwr_bg_trim2;                       /* 0x40030f04 */
    volatile uint32_t clk_imo_select;                     /* 0x40030f08 */
    volatile uint32_t clk_imo_trim1;                      /* 0x40030f0c */
    volatile uint32_t clk_imo_trim2;                      /* 0x40030f10 */
    volatile uint32_t pwr_pwrsys_trim1;                   /* 0x40030f14 */
    volatile uint32_t clk_imo_trim3;                      /* 0x40030f18 */
} SRSSLT_REGS_T, *PSRSSLT_REGS_T;

#define SRSSLT        ((PSRSSLT_REGS_T) SRSSLT_BASE_ADDR)


#define GPIO_BASE_ADDR                                   (0x40040000)
#define GPIO0_BASE_ADDR                                  (0x40040000)
#define GPIO1_BASE_ADDR                                  (0x40040100)
#define GPIO2_BASE_ADDR                                  (0x40040200)
#define GPIO3_BASE_ADDR                                  (0x40040300)
#define GPIO4_BASE_ADDR                                  (0x40040400)

typedef struct {
    volatile uint32_t dr;                                 /* 0x40040000 */
    volatile uint32_t ps;                                 /* 0x40040004 */
    volatile uint32_t pc;                                 /* 0x40040008 */
    volatile uint32_t intr_cfg;                           /* 0x4004000c */
    volatile uint32_t intr;                               /* 0x40040010 */
    volatile uint32_t rsrvd0;
    volatile uint32_t pc2;                                /* 0x40040018 */
    volatile uint32_t rsrvd1[9];
    volatile uint32_t dr_set;                             /* 0x40040040 */
    volatile uint32_t dr_clr;                             /* 0x40040044 */
    volatile uint32_t dr_inv;                             /* 0x40040048 */
    volatile uint32_t rsrvd2[1005];
    volatile uint32_t intr_cause;                         /* 0x40041000 */
    volatile uint32_t rsrvd3[3];
    volatile uint32_t dft_io_test;                        /* 0x40041010 */
} GPIO_REGS_T, *PGPIO_REGS_T;

#define GPIO0       ((PGPIO_REGS_T) GPIO0_BASE_ADDR)
#define GPIO1       ((PGPIO_REGS_T) GPIO1_BASE_ADDR)
#define GPIO2       ((PGPIO_REGS_T) GPIO2_BASE_ADDR)
#define GPIO3       ((PGPIO_REGS_T) GPIO3_BASE_ADDR)
#define GPIO4       ((PGPIO_REGS_T) GPIO4_BASE_ADDR)

/*
 * NOTE:
 * There are multiple instances of the above register section.
 * The following structure pointer array needs to be defined in
 * a source file before using it.
 * const PGPIO_REGS_T GPIO[5] = {GPIO0, GPIO1, GPIO2, GPIO3, GPIO4};
 */
extern const PGPIO_REGS_T GPIO[];


#define SCB_PRT_BASE_ADDR                                (0x40050000)
#define SCB_PRT0_BASE_ADDR                               (0x40050000)
#define SCB_PRT1_BASE_ADDR                               (0x40060000)
#define SCB_PRT2_BASE_ADDR                               (0x40070000)
#define SCB_PRT3_BASE_ADDR                               (0x40080000)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x40050000 */
    volatile uint32_t status;                             /* 0x40050004 */
    volatile uint32_t cmd_resp_ctrl;                      /* 0x40050008 */
    volatile uint32_t cmd_resp_status;                    /* 0x4005000c */
    volatile uint32_t rsrvd0[4];
    volatile uint32_t spi_ctrl;                           /* 0x40050020 */
    volatile uint32_t spi_status;                         /* 0x40050024 */
    volatile uint32_t rsrvd1[6];
    volatile uint32_t uart_ctrl;                          /* 0x40050040 */
    volatile uint32_t uart_tx_ctrl;                       /* 0x40050044 */
    volatile uint32_t uart_rx_ctrl;                       /* 0x40050048 */
    volatile uint32_t uart_rx_status;                     /* 0x4005004c */
    volatile uint32_t uart_flow_ctrl;                     /* 0x40050050 */
    volatile uint32_t rsrvd2[3];
    volatile uint32_t i2c_ctrl;                           /* 0x40050060 */
    volatile uint32_t i2c_status;                         /* 0x40050064 */
    volatile uint32_t i2c_m_cmd;                          /* 0x40050068 */
    volatile uint32_t i2c_s_cmd;                          /* 0x4005006c */
    volatile uint32_t i2c_cfg;                            /* 0x40050070 */
    volatile uint32_t rsrvd3[99];
    volatile uint32_t tx_ctrl;                            /* 0x40050200 */
    volatile uint32_t tx_fifo_ctrl;                       /* 0x40050204 */
    volatile uint32_t tx_fifo_status;                     /* 0x40050208 */
    volatile uint32_t rsrvd4[13];
    volatile uint32_t tx_fifo_wr;                         /* 0x40050240 */
    volatile uint32_t rsrvd5[47];
    volatile uint32_t rx_ctrl;                            /* 0x40050300 */
    volatile uint32_t rx_fifo_ctrl;                       /* 0x40050304 */
    volatile uint32_t rx_fifo_status;                     /* 0x40050308 */
    volatile uint32_t rsrvd6;
    volatile uint32_t rx_match;                           /* 0x40050310 */
    volatile uint32_t rsrvd7[11];
    volatile uint32_t rx_fifo_rd;                         /* 0x40050340 */
    volatile uint32_t rx_fifo_rd_silent;                  /* 0x40050344 */
    volatile uint32_t rsrvd8[46];
    volatile uint32_t ez_data[32];                        /* 0x40050400 */
    volatile uint32_t rsrvd9[608];
    volatile uint32_t intr_cause;                         /* 0x40050e00 */
    volatile uint32_t rsrvd10[31];
    volatile uint32_t intr_i2c_ec;                        /* 0x40050e80 */
    volatile uint32_t rsrvd11;
    volatile uint32_t intr_i2c_ec_mask;                   /* 0x40050e88 */
    volatile uint32_t intr_i2c_ec_masked;                 /* 0x40050e8c */
    volatile uint32_t rsrvd12[12];
    volatile uint32_t intr_spi_ec;                        /* 0x40050ec0 */
    volatile uint32_t rsrvd13;
    volatile uint32_t intr_spi_ec_mask;                   /* 0x40050ec8 */
    volatile uint32_t intr_spi_ec_masked;                 /* 0x40050ecc */
    volatile uint32_t rsrvd14[12];
    volatile uint32_t intr_m;                             /* 0x40050f00 */
    volatile uint32_t intr_m_set;                         /* 0x40050f04 */
    volatile uint32_t intr_m_mask;                        /* 0x40050f08 */
    volatile uint32_t intr_m_masked;                      /* 0x40050f0c */
    volatile uint32_t rsrvd15[12];
    volatile uint32_t intr_s;                             /* 0x40050f40 */
    volatile uint32_t intr_s_set;                         /* 0x40050f44 */
    volatile uint32_t intr_s_mask;                        /* 0x40050f48 */
    volatile uint32_t intr_s_masked;                      /* 0x40050f4c */
    volatile uint32_t rsrvd16[12];
    volatile uint32_t intr_tx;                            /* 0x40050f80 */
    volatile uint32_t intr_tx_set;                        /* 0x40050f84 */
    volatile uint32_t intr_tx_mask;                       /* 0x40050f88 */
    volatile uint32_t intr_tx_masked;                     /* 0x40050f8c */
    volatile uint32_t rsrvd17[12];
    volatile uint32_t intr_rx;                            /* 0x40050fc0 */
    volatile uint32_t intr_rx_set;                        /* 0x40050fc4 */
    volatile uint32_t intr_rx_mask;                       /* 0x40050fc8 */
    volatile uint32_t intr_rx_masked;                     /* 0x40050fcc */
} SCB_PRT_REGS_T, *PSCB_PRT_REGS_T;

#define SCB_PRT0       ((PSCB_PRT_REGS_T) SCB_PRT0_BASE_ADDR)
#define SCB_PRT1       ((PSCB_PRT_REGS_T) SCB_PRT1_BASE_ADDR)
#define SCB_PRT2       ((PSCB_PRT_REGS_T) SCB_PRT2_BASE_ADDR)
#define SCB_PRT3       ((PSCB_PRT_REGS_T) SCB_PRT3_BASE_ADDR)

/*
 * NOTE:
 * There are multiple instances of the above register section.
 * The following structure pointer array needs to be defined in
 * a source file before using it.
 * const PSCB_PRT_REGS_T SCB_PRT[4] = {SCB_PRT0, SCB_PRT1, SCB_PRT2, SCB_PRT3};
 */
extern const PSCB_PRT_REGS_T SCB_PRT[];


#define TCPWM_BASE_ADDR                                  (0x40090000)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x40090000 */
    volatile uint32_t rsrvd0;
    volatile uint32_t cmd;                                /* 0x40090008 */
    volatile uint32_t intr_cause;                         /* 0x4009000c */
} TCPWM_REGS_T, *PTCPWM_REGS_T;

#define TCPWM        ((PTCPWM_REGS_T) TCPWM_BASE_ADDR)


#define CNT_BASE_ADDR                                    (0x40090100)
#define CNT0_BASE_ADDR                                   (0x40090100)
#define CNT1_BASE_ADDR                                   (0x40090140)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x40090100 */
    volatile uint32_t status;                             /* 0x40090104 */
    volatile uint32_t counter;                            /* 0x40090108 */
    volatile uint32_t cc;                                 /* 0x4009010c */
    volatile uint32_t cc_buff;                            /* 0x40090110 */
    volatile uint32_t period;                             /* 0x40090114 */
    volatile uint32_t period_buff;                        /* 0x40090118 */
    volatile uint32_t rsrvd0;
    volatile uint32_t tr_ctrl0;                           /* 0x40090120 */
    volatile uint32_t tr_ctrl1;                           /* 0x40090124 */
    volatile uint32_t tr_ctrl2;                           /* 0x40090128 */
    volatile uint32_t rsrvd1;
    volatile uint32_t intr;                               /* 0x40090130 */
    volatile uint32_t intr_set;                           /* 0x40090134 */
    volatile uint32_t intr_mask;                          /* 0x40090138 */
    volatile uint32_t intr_masked;                        /* 0x4009013c */
} CNT_REGS_T, *PCNT_REGS_T;

#define CNT0       ((PCNT_REGS_T) CNT0_BASE_ADDR)
#define CNT1       ((PCNT_REGS_T) CNT1_BASE_ADDR)

/*
 * NOTE:
 * There are multiple instances of the above register section.
 * The following structure pointer array needs to be defined in
 * a source file before using it.
 * const PCNT_REGS_T CNT[2] = {CNT0, CNT1};
 */
extern const PCNT_REGS_T CNT[];


#define PDSS_BASE_ADDR                                   (0x400a0000)
#define PDSS0_BASE_ADDR                                  (0x400a0000)
#define PDSS1_BASE_ADDR                                  (0x400b0000)

typedef struct {
    volatile uint32_t ctrl;                               /* 0x400a0000 */
    volatile uint32_t header_info;                        /* 0x400a0004 */
    volatile uint32_t rsrvd0;
    volatile uint32_t tx_header;                          /* 0x400a000c */
    volatile uint32_t tx_mem_data[16];                    /* 0x400a0010 */
    volatile uint32_t rsrvd1[3];
    volatile uint32_t rx_header;                          /* 0x400a005c */
    volatile uint32_t rx_mem_data[16];                    /* 0x400a0060 */
    volatile uint32_t sram_ptr;                           /* 0x400a00a0 */
    volatile uint32_t status;                             /* 0x400a00a4 */
    volatile uint32_t rx_sop_good_crc_en_ctrl;            /* 0x400a00a8 */
    volatile uint32_t rx_default_sop_goodcrc_ctrl_0;      /* 0x400a00ac */
    volatile uint32_t rx_default_sop_goodcrc_ctrl_1;      /* 0x400a00b0 */
    volatile uint32_t rx_default_sop_goodcrc_ctrl_2;      /* 0x400a00b4 */
    volatile uint32_t rx_prime_sop_goodcrc_ctrl_0;        /* 0x400a00b8 */
    volatile uint32_t rx_prime_sop_goodcrc_ctrl_1;        /* 0x400a00bc */
    volatile uint32_t rx_prime_sop_goodcrc_ctrl_2;        /* 0x400a00c0 */
    volatile uint32_t rx_dbl_prime_sop_goodcrc_ctrl_0;    /* 0x400a00c4 */
    volatile uint32_t rx_dbl_prime_sop_goodcrc_ctrl_1;    /* 0x400a00c8 */
    volatile uint32_t rx_dbl_prime_sop_goodcrc_ctrl_2;    /* 0x400a00cc */
    volatile uint32_t rx_expect_goodcrc_msg;              /* 0x400a00d0 */
    volatile uint32_t rx_goodcrc_msg;                     /* 0x400a00d4 */
    volatile uint32_t rx_cc;                              /* 0x400a00d8 */
    volatile uint32_t rsrvd2;
    volatile uint32_t rx_cc2;                             /* 0x400a00e0 */
    volatile uint32_t rx_order_set_ctrl;                  /* 0x400a00e4 */
    volatile uint32_t rx_reserved1_order_set;             /* 0x400a00e8 */
    volatile uint32_t rx_reserved2_order_set;             /* 0x400a00ec */
    volatile uint32_t tx_goodcrc_msg_order_set;           /* 0x400a00f0 */
    volatile uint32_t tx_ctrl;                            /* 0x400a00f4 */
    volatile uint32_t tx_sop_order_set;                   /* 0x400a00f8 */
    volatile uint32_t tx_hard_cable_order_set;            /* 0x400a00fc */
    volatile uint32_t crc_counter;                        /* 0x400a0100 */
    volatile uint32_t inter_packet_counter;               /* 0x400a0104 */
    volatile uint32_t timer_trigger;                      /* 0x400a0108 */
    volatile uint32_t intr_ddft_mux;                      /* 0x400a010c */
    volatile uint32_t ddft_mux;                           /* 0x400a0110 */
    volatile uint32_t intr_1_cfg;                         /* 0x400a0114 */
    volatile uint32_t intr0;                              /* 0x400a0118 */
    volatile uint32_t intr1;                              /* 0x400a011c */
    volatile uint32_t intr0_set;                          /* 0x400a0120 */
    volatile uint32_t intr1_set;                          /* 0x400a0124 */
    volatile uint32_t intr0_mask;                         /* 0x400a0128 */
    volatile uint32_t intr1_mask;                         /* 0x400a012c */
    volatile uint32_t intr0_masked;                       /* 0x400a0130 */
    volatile uint32_t intr1_masked;                       /* 0x400a0134 */
    volatile uint32_t debug_ctrl;                         /* 0x400a0138 */
    volatile uint32_t debug_cc_0;                         /* 0x400a013c */
    volatile uint32_t debug_cc_1;                         /* 0x400a0140 */
    volatile uint32_t debug_cc_2;                         /* 0x400a0144 */
    volatile uint32_t debug_spare_intr_cfg;               /* 0x400a0148 */
    volatile uint32_t intr2;                              /* 0x400a014c */
    volatile uint32_t intr2_set;                          /* 0x400a0150 */
    volatile uint32_t intr2_mask;                         /* 0x400a0154 */
    volatile uint32_t intr2_masked;                       /* 0x400a0158 */
    volatile uint32_t hpd_ctrl1;                          /* 0x400a015c */
    volatile uint32_t hpd_ctrl2;                          /* 0x400a0160 */
    volatile uint32_t hpd_ctrl3;                          /* 0x400a0164 */
    volatile uint32_t hpd_ctrl4;                          /* 0x400a0168 */
    volatile uint32_t hpd_ctrl5;                          /* 0x400a016c */
    volatile uint32_t hpd_queue;                          /* 0x400a0170 */
    volatile uint32_t hpdt_ctrl1;                         /* 0x400a0174 */
    volatile uint32_t hpdt_ctrl2;                         /* 0x400a0178 */
    volatile uint32_t adc1_sar_ctrl;                      /* 0x400a017c */
    volatile uint32_t adc1_ctrl;                          /* 0x400a0180 */
    volatile uint32_t adc2_sar_ctrl;                      /* 0x400a0184 */
    volatile uint32_t adc2_ctrl;                          /* 0x400a0188 */
    volatile uint32_t cc_ctrl_0;                          /* 0x400a018c */
    volatile uint32_t cc_ctrl_1;                          /* 0x400a0190 */
    volatile uint32_t dpslp_ref_ctrl;                     /* 0x400a0194 */
    volatile uint32_t rsrvd3;
    volatile uint32_t pump_ctrl;                          /* 0x400a019c */
    volatile uint32_t pfet300_ctrl;                       /* 0x400a01a0 */
    volatile uint32_t swap_ctrl0;                         /* 0x400a01a4 */
    volatile uint32_t swap_ctrl1;                         /* 0x400a01a8 */
    volatile uint32_t swap_ctrl2;                         /* 0x400a01ac */
    volatile uint32_t swap_ctrl3;                         /* 0x400a01b0 */
    volatile uint32_t swap_ctrl4;                         /* 0x400a01b4 */
    volatile uint32_t swap_ctrl5;                         /* 0x400a01b8 */
    volatile uint32_t swap_queue;                         /* 0x400a01bc */
    volatile uint32_t swapt_ctrl1;                        /* 0x400a01c0 */
    volatile uint32_t swapt_ctrl2;                        /* 0x400a01c4 */
    volatile uint32_t ngdo_ctrl_0;                        /* 0x400a01c8 */
    volatile uint32_t ngdo_ctrl_c;                        /* 0x400a01cc */
    volatile uint32_t ngdo_ctrl_p;                        /* 0x400a01d0 */
    volatile uint32_t ncell_ddft_mux;                     /* 0x400a01d4 */
    volatile uint32_t swap_cntrl_0;                       /* 0x400a01d8 */
    volatile uint32_t swap_cntrl_1;                       /* 0x400a01dc */
    volatile uint32_t bit_en_cntr_ctrl;                   /* 0x400a01e0 */
    volatile uint32_t rsrvd4[16199];
    volatile uint32_t s8usbpd_trim_0;                     /* 0x400aff00 */
    volatile uint32_t s8usbpd_trim_1;                     /* 0x400aff04 */
    volatile uint32_t s8usbpd_trim_2;                     /* 0x400aff08 */
    volatile uint32_t s8usbpd_trim_3;                     /* 0x400aff0c */
    volatile uint32_t s8usbpd_trim_4;                     /* 0x400aff10 */
    volatile uint32_t s8usbpd_trim_5;                     /* 0x400aff14 */
    volatile uint32_t s8usbpd_trim_6;                     /* 0x400aff18 */
    volatile uint32_t s8usbpd_trim_7;                     /* 0x400aff1c */
    volatile uint32_t s8usbpd_trim_spare1;                /* 0x400aff20 */
    volatile uint32_t s8usbpd_trim_spare2;                /* 0x400aff24 */
} PDSS_REGS_T, *PPDSS_REGS_T;

#define PDSS0       ((PPDSS_REGS_T) PDSS0_BASE_ADDR)
#define PDSS1       ((PPDSS_REGS_T) PDSS1_BASE_ADDR)

/*
 * NOTE:
 * There are multiple instances of the above register section.
 * The following structure pointer array needs to be defined in
 * a source file before using it.
 * const PPDSS_REGS_T PDSS[2] = {PDSS0, PDSS1};
 */
extern const PPDSS_REGS_T PDSS[];


#define CPUSS_BASE_ADDR                                  (0x40100000)

typedef struct {
    volatile uint32_t config;                             /* 0x40100000 */
    volatile uint32_t sysreq;                             /* 0x40100004 */
    volatile uint32_t sysarg;                             /* 0x40100008 */
    volatile uint32_t protection;                         /* 0x4010000c */
    volatile uint32_t priv_rom;                           /* 0x40100010 */
    volatile uint32_t priv_ram;                           /* 0x40100014 */
    volatile uint32_t priv_flash;                         /* 0x40100018 */
    volatile uint32_t wounding;                           /* 0x4010001c */
    volatile uint32_t rsrvd0[3];
    volatile uint32_t dft;                                /* 0x4010002c */
    volatile uint32_t flash_ctl;                          /* 0x40100030 */
    volatile uint32_t rom_ctl;                            /* 0x40100034 */
    volatile uint32_t rsrvd1[2];
    volatile uint32_t bist_cmd;                           /* 0x40100040 */
    volatile uint32_t bist_data;                          /* 0x40100044 */
    volatile uint32_t rsrvd2;
    volatile uint32_t bist_ctl;                           /* 0x4010004c */
    volatile uint32_t bist_step0_ctl;                     /* 0x40100050 */
    volatile uint32_t rsrvd3[11];
    volatile uint32_t bist_status;                        /* 0x40100080 */
    volatile uint32_t bist_data_act;                      /* 0x40100084 */
    volatile uint32_t bist_data_exp;                      /* 0x40100088 */
    volatile uint32_t bist_addr;                          /* 0x4010008c */
    volatile uint32_t bist_misr;                          /* 0x40100090 */
    volatile uint32_t rsrvd4[11];
    volatile uint32_t ptm_ctl;                            /* 0x401000c0 */
} CPUSS_REGS_T, *PCPUSS_REGS_T;

#define CPUSS        ((PCPUSS_REGS_T) CPUSS_BASE_ADDR)


#define SPCIF_BASE_ADDR                                  (0x40110000)

typedef struct {
    volatile uint32_t geometry;                           /* 0x40110000 */
    volatile uint32_t address;                            /* 0x40110004 */
    volatile uint32_t timer;                              /* 0x40110008 */
    volatile uint32_t flash_control;                      /* 0x4011000c */
    volatile uint32_t flash_wr_data;                      /* 0x40110010 */
    volatile uint32_t rsrvd0[5];
    volatile uint32_t bookmark;                           /* 0x40110028 */
    volatile uint32_t rsrvd1;
    volatile uint32_t fmlt_dft;                           /* 0x40110030 */
    volatile uint32_t rsrvd2[495];
    volatile uint32_t intr;                               /* 0x401107f0 */
    volatile uint32_t intr_set;                           /* 0x401107f4 */
    volatile uint32_t intr_mask;                          /* 0x401107f8 */
    volatile uint32_t intr_masked;                        /* 0x401107fc */
    volatile uint32_t rsrvd3[15808];
    volatile uint32_t trim_m0_dac0;                       /* 0x4011ff00 */
    volatile uint32_t trim_m0_dac1;                       /* 0x4011ff04 */
    volatile uint32_t trim_m0_dac2;                       /* 0x4011ff08 */
    volatile uint32_t trim_m0_dac3;                       /* 0x4011ff0c */
} SPCIF_REGS_T, *PSPCIF_REGS_T;

#define SPCIF        ((PSPCIF_REGS_T) SPCIF_BASE_ADDR)


#define CM0_BASE_ADDR                                    (0xe0001000)

typedef struct {
    volatile uint32_t dwt_ctrl;                           /* 0xe0001000 */
    volatile uint32_t rsrvd0[6];
    volatile uint32_t dwt_pcsr;                           /* 0xe000101c */
    volatile uint32_t dwt_comp0;                          /* 0xe0001020 */
    volatile uint32_t dwt_mask0;                          /* 0xe0001024 */
    volatile uint32_t dwt_function0;                      /* 0xe0001028 */
    volatile uint32_t rsrvd1;
    volatile uint32_t dwt_comp1;                          /* 0xe0001030 */
    volatile uint32_t dwt_mask1;                          /* 0xe0001034 */
    volatile uint32_t dwt_function1;                      /* 0xe0001038 */
    volatile uint32_t rsrvd2[997];
    volatile uint32_t dwt_pid4;                           /* 0xe0001fd0 */
    volatile uint32_t rsrvd3[3];
    volatile uint32_t dwt_pid0;                           /* 0xe0001fe0 */
    volatile uint32_t dwt_pid1;                           /* 0xe0001fe4 */
    volatile uint32_t dwt_pid2;                           /* 0xe0001fe8 */
    volatile uint32_t dwt_pid3;                           /* 0xe0001fec */
    volatile uint32_t dwt_cid0;                           /* 0xe0001ff0 */
    volatile uint32_t dwt_cid1;                           /* 0xe0001ff4 */
    volatile uint32_t dwt_cid2;                           /* 0xe0001ff8 */
    volatile uint32_t dwt_cid3;                           /* 0xe0001ffc */
    volatile uint32_t bp_ctrl;                            /* 0xe0002000 */
    volatile uint32_t rsrvd4;
    volatile uint32_t bp_comp0;                           /* 0xe0002008 */
    volatile uint32_t bp_comp1;                           /* 0xe000200c */
    volatile uint32_t bp_comp2;                           /* 0xe0002010 */
    volatile uint32_t bp_comp3;                           /* 0xe0002014 */
    volatile uint32_t rsrvd5[1006];
    volatile uint32_t bp_pid4;                            /* 0xe0002fd0 */
    volatile uint32_t rsrvd6[3];
    volatile uint32_t bp_pid0;                            /* 0xe0002fe0 */
    volatile uint32_t bp_pid1;                            /* 0xe0002fe4 */
    volatile uint32_t bp_pid2;                            /* 0xe0002fe8 */
    volatile uint32_t bp_pid3;                            /* 0xe0002fec */
    volatile uint32_t bp_cid0;                            /* 0xe0002ff0 */
    volatile uint32_t bp_cid1;                            /* 0xe0002ff4 */
    volatile uint32_t bp_cid2;                            /* 0xe0002ff8 */
    volatile uint32_t bp_cid3;                            /* 0xe0002ffc */
    volatile uint32_t rsrvd7[11266];
    volatile uint32_t actlr;                              /* 0xe000e008 */
    volatile uint32_t rsrvd8;
    volatile uint32_t syst_csr;                           /* 0xe000e010 */
    volatile uint32_t syst_rvr;                           /* 0xe000e014 */
    volatile uint32_t syst_cvr;                           /* 0xe000e018 */
    volatile uint32_t syst_calib;                         /* 0xe000e01c */
    volatile uint32_t rsrvd9[56];
    volatile uint32_t iser;                               /* 0xe000e100 */
    volatile uint32_t rsrvd10[31];
    volatile uint32_t icer;                               /* 0xe000e180 */
    volatile uint32_t rsrvd11[31];
    volatile uint32_t ispr;                               /* 0xe000e200 */
    volatile uint32_t rsrvd12[31];
    volatile uint32_t icpr;                               /* 0xe000e280 */
    volatile uint32_t rsrvd13[95];
    volatile uint32_t ipr[8];                             /* 0xe000e400 */
    volatile uint32_t rsrvd14[568];
    volatile uint32_t cpuid;                              /* 0xe000ed00 */
    volatile uint32_t icsr;                               /* 0xe000ed04 */
    volatile uint32_t rsrvd15;
    volatile uint32_t aircr;                              /* 0xe000ed0c */
    volatile uint32_t scr;                                /* 0xe000ed10 */
    volatile uint32_t ccr;                                /* 0xe000ed14 */
    volatile uint32_t rsrvd16;
    volatile uint32_t shpr2;                              /* 0xe000ed1c */
    volatile uint32_t shpr3;                              /* 0xe000ed20 */
    volatile uint32_t shcsr;                              /* 0xe000ed24 */
    volatile uint32_t rsrvd17[2];
    volatile uint32_t dfsr;                               /* 0xe000ed30 */
    volatile uint32_t rsrvd18[47];
    volatile uint32_t dhcsr;                              /* 0xe000edf0 */
    volatile uint32_t dcrsr;                              /* 0xe000edf4 */
    volatile uint32_t dcrdr;                              /* 0xe000edf8 */
    volatile uint32_t demcr;                              /* 0xe000edfc */
    volatile uint32_t rsrvd19[116];
    volatile uint32_t scs_pid4;                           /* 0xe000efd0 */
    volatile uint32_t rsrvd20[3];
    volatile uint32_t scs_pid0;                           /* 0xe000efe0 */
    volatile uint32_t scs_pid1;                           /* 0xe000efe4 */
    volatile uint32_t scs_pid2;                           /* 0xe000efe8 */
    volatile uint32_t scs_pid3;                           /* 0xe000efec */
    volatile uint32_t scs_cid0;                           /* 0xe000eff0 */
    volatile uint32_t scs_cid1;                           /* 0xe000eff4 */
    volatile uint32_t scs_cid2;                           /* 0xe000eff8 */
    volatile uint32_t scs_cid3;                           /* 0xe000effc */
    volatile uint32_t rsrvd21[245760];
    volatile uint32_t rom_scs;                            /* 0xe00ff000 */
    volatile uint32_t rom_dwt;                            /* 0xe00ff004 */
    volatile uint32_t rom_bpu;                            /* 0xe00ff008 */
    volatile uint32_t rom_end;                            /* 0xe00ff00c */
    volatile uint32_t rsrvd22[1007];
    volatile uint32_t rom_csmt;                           /* 0xe00fffcc */
    volatile uint32_t rom_pid4;                           /* 0xe00fffd0 */
    volatile uint32_t rsrvd23[3];
    volatile uint32_t rom_pid0;                           /* 0xe00fffe0 */
    volatile uint32_t rom_pid1;                           /* 0xe00fffe4 */
    volatile uint32_t rom_pid2;                           /* 0xe00fffe8 */
    volatile uint32_t rom_pid3;                           /* 0xe00fffec */
    volatile uint32_t rom_cid0;                           /* 0xe00ffff0 */
    volatile uint32_t rom_cid1;                           /* 0xe00ffff4 */
    volatile uint32_t rom_cid2;                           /* 0xe00ffff8 */
    volatile uint32_t rom_cid3;                           /* 0xe00ffffc */
} CM0_REGS_T, *PCM0_REGS_T;

#define CM0        ((PCM0_REGS_T) CM0_BASE_ADDR)


#define ROMTABLE_BASE_ADDR                               (0xf0000000)

typedef struct {
    volatile uint32_t addr;                               /* 0xf0000000 */
    volatile uint32_t rsrvd0[1010];
    volatile uint32_t did;                                /* 0xf0000fcc */
    volatile uint32_t pid4;                               /* 0xf0000fd0 */
    volatile uint32_t pid5;                               /* 0xf0000fd4 */
    volatile uint32_t pid6;                               /* 0xf0000fd8 */
    volatile uint32_t pid7;                               /* 0xf0000fdc */
    volatile uint32_t pid0;                               /* 0xf0000fe0 */
    volatile uint32_t pid1;                               /* 0xf0000fe4 */
    volatile uint32_t pid2;                               /* 0xf0000fe8 */
    volatile uint32_t pid3;                               /* 0xf0000fec */
    volatile uint32_t cid0;                               /* 0xf0000ff0 */
    volatile uint32_t cid1;                               /* 0xf0000ff4 */
    volatile uint32_t cid2;                               /* 0xf0000ff8 */
    volatile uint32_t cid3;                               /* 0xf0000ffc */
} ROMTABLE_REGS_T, *PROMTABLE_REGS_T;

#define ROMTABLE        ((PROMTABLE_REGS_T) ROMTABLE_BASE_ADDR)


/*
 * Per Page Write Protection
 * Flash row protection data for Macro #0 (1 bit for each page in main data
 * area).  Byte N, bit X applies to (Page 8*N+X).  When set indicates row
 * cannot be erased or programmed except during total erase and back to PROTECTION=OPEN.
 */
#define SFLASH_PROT_ROW_ADDRESS(n)                          (0x0ffff000 + ((n) * (0x0001)))
#define SFLASH_PROT_ROW(n)                                  (*(volatile uint8_t *)(0x0ffff000 + ((n) * 0x0001)))
#define SFLASH_PROT_ROW_DEFAULT                             (0x00000000)

/*
 * Protection Data (1b per page)
 */
#define SFLASH_PROT_ROW_DATA8_MASK                          (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_PROT_ROW_DATA8_POS                           (0)


/*
 * Protection Level
 * Copied during Boot to CPUSS.PROTECTION
 */
#define SFLASH_PROT_PROTECTION_ADDRESS                      (0x0ffff0ff)
#define SFLASH_PROT_PROTECTION                              (*(volatile uint8_t *)(0x0ffff0ff))
#define SFLASH_PROT_PROTECTION_DEFAULT                      (0x00000000)

/*
 * Current Protection Mode - note that encoding is different from CPUSS_PROTECTION
 * !!
 */
#define SFLASH_PROT_PROTECTION_PROT_LEVEL_MASK              (0x00000003) /* <0:1> :RW:X: */
#define SFLASH_PROT_PROTECTION_PROT_LEVEL_POS               (0)


/*
 * 8b Addr/Value pair Section
 * Up to 32 Strings of (8b Compressed Address (CA), N*8b Data).  For each
 * string the data value is copied into the trim region for the indicated
 * peripheral on boot.  This section is used only for trim registers.  Compressed
 * address is encoded as follows:
 * BridgeNo= CA[3]
 * PeripheralNo= CA[7:4]
 * NumberOfBytes= CA[2:0]+1
 * Address= 0x4000FF00 + BridgeNo*0x100000 + PerpheralNo*0x10000 + RegisterIndex*4
 */
#define SFLASH_AV_PAIRS_8B_ADDRESS(n)                       (0x0ffff100 + ((n) * (0x0001)))
#define SFLASH_AV_PAIRS_8B(n)                               (*(volatile uint8_t *)(0x0ffff100 + ((n) * 0x0001)))
#define SFLASH_AV_PAIRS_8B_DEFAULT                          (0x00000000)

/*
 * Address or Value Byte
 */
#define SFLASH_AV_PAIRS_8B_DATA8_MASK                       (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_AV_PAIRS_8B_DATA8_POS                        (0)


/*
 * 32b Addr/Value pair Section
 * 8 Pairs of full 32-bit Address and Value. For each pair the value is copied
 * into the indicated address on boot.
 */
#define SFLASH_AV_PAIRS_32B_ADDRESS(n)                      (0x0ffff200 + ((n) * (0x0004)))
#define SFLASH_AV_PAIRS_32B(n)                              (*(volatile uint32_t *)(0x0ffff200 + ((n) * 0x0004)))
#define SFLASH_AV_PAIRS_32B_DEFAULT                         (0x00000000)

/*
 * Address or Value Word
 */
#define SFLASH_AV_PAIRS_32B_DATA32_MASK                     (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_AV_PAIRS_32B_DATA32_POS                      (0)


/*
 * CPUSS Wounding Register
 * Copied during Boot to CPUSS.WOUNDING
 */
#define SFLASH_CPUSS_WOUNDING_ADDRESS                       (0x0ffff240)
#define SFLASH_CPUSS_WOUNDING                               (*(volatile uint32_t *)(0x0ffff240))
#define SFLASH_CPUSS_WOUNDING_DEFAULT                       (0x00000000)

/*
 * Data to use for register
 */
#define SFLASH_CPUSS_WOUNDING_DATA32_MASK                   (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_CPUSS_WOUNDING_DATA32_POS                    (0)


/*
 * Silicon ID
 * Copied during Boot to CPUSS.SILICON_ID
 */
#define SFLASH_SILICON_ID_ADDRESS                           (0x0ffff244)
#define SFLASH_SILICON_ID                                   (*(volatile uint32_t *)(0x0ffff244))
#define SFLASH_SILICON_ID_DEFAULT                           (0x00000000)

/*
 * Silicon ID
 */
#define SFLASH_SILICON_ID_ID_MASK                           (0x0000ffff) /* <0:15> :RW:X: */
#define SFLASH_SILICON_ID_ID_POS                            (0)


/*
 * RAM Privileged Limit
 * Copied during Boot to CPUSS.PRIV_RAM
 */
#define SFLASH_CPUSS_PRIV_RAM_ADDRESS                       (0x0ffff248)
#define SFLASH_CPUSS_PRIV_RAM                               (*(volatile uint16_t *)(0x0ffff248))
#define SFLASH_CPUSS_PRIV_RAM_DEFAULT                       (0x00000000)

/*
 * Indicates the limit where the privileged area of SRAM starts in increments
 * of 256 Bytes.
 * "0":  Entire SRAM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the SRAM indicates that the entire
 * SRAM is user mode accessible.
 */
#define SFLASH_CPUSS_PRIV_RAM_RAM_PROT_LIMIT_MASK           (0x000001ff) /* <0:8> R:RW:0: */
#define SFLASH_CPUSS_PRIV_RAM_RAM_PROT_LIMIT_POS            (0)


/*
 * Boot ROM Privileged Limit
 * Copied during Boot to CPUSS.PRIV_ROM.BROM_PROT_LIMIT
 */
#define SFLASH_CPUSS_PRIV_ROM_BROM_ADDRESS                  (0x0ffff24a)
#define SFLASH_CPUSS_PRIV_ROM_BROM                          (*(volatile uint16_t *)(0x0ffff24a))
#define SFLASH_CPUSS_PRIV_ROM_BROM_DEFAULT                  (0x00000000)

/*
 * Indicates the limit where the privileged area of the Boot ROM partition
 * starts in increments of 256 Bytes.
 * "0":  Entire Boot ROM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 * ...
 * BROM_PROT_LIMIT >= "Boot ROM partition capacity": Entire Boot ROM partition
 * is user mode accessible.
 */
#define SFLASH_CPUSS_PRIV_ROM_BROM_BROM_PROT_LIMIT_MASK     (0x000000ff) /* <0:7> R:RW:0: */
#define SFLASH_CPUSS_PRIV_ROM_BROM_BROM_PROT_LIMIT_POS      (0)


/*
 * Flash Privileged Limit
 * Copied during Boot CPUSS.PRIV_FLASH
 */
#define SFLASH_CPUSS_PRIV_FLASH_ADDRESS                     (0x0ffff24c)
#define SFLASH_CPUSS_PRIV_FLASH                             (*(volatile uint16_t *)(0x0ffff24c))
#define SFLASH_CPUSS_PRIV_FLASH_DEFAULT                     (0x00000000)

/*
 * Indicates the limit where the privileged area of flash starts in increments
 * of 256 Bytes.
 * "0":  Entire flash is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the flash indicates that the entire
 * flash is user mode accessible. Note that SuperVisory rows are always User
 * accessable.
 *
 * If FLASH_PROT_LIMIT defines a non-empty privileged area, the boot ROM
 * will assume that a system call table exists at the beginning of the Flash
 * privileged area and use it for all SystemCalls made using SYSREQ.
 */
#define SFLASH_CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_MASK       (0x000007ff) /* <0:10> R:RW:0: */
#define SFLASH_CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_POS        (0)


/*
 * System ROM Privileged Limit
 * Copied during Boot to CPUSS.PRIV_ROM.SROM_PROT_LIMIT
 */
#define SFLASH_CPUSS_PRIV_ROM_SROM_ADDRESS                  (0x0ffff24e)
#define SFLASH_CPUSS_PRIV_ROM_SROM                          (*(volatile uint16_t *)(0x0ffff24e))
#define SFLASH_CPUSS_PRIV_ROM_SROM_DEFAULT                  (0x00000000)

/*
 * Indicates the limit where the privileged area of System ROM partition
 * starts in increments of 256 Bytes. The limit is wrt. the start of the
 * ROM memory (start of the Boot ROM partition).
 * SROM_PROT_LIMIT * 256 Byte <= "Boot ROM partition capacity":  Entire System
 * ROM is Privileged.
 * SROM_PROT_LIMIT * 256 Byte > "Boot ROM partition capacity":  First SROM_PROT_LIMIT
 * * 256 - "Boot ROM partition capacity" Bytes are User accessable.
 * ...
 * SROM_PROT_LIMIT >= "ROM capacity": Entire System ROM is user mode accessible.
 */
#define SFLASH_CPUSS_PRIV_ROM_SROM_SROM_PROT_LIMIT_MASK     (0x000003ff) /* <0:9> R:RW:0: */
#define SFLASH_CPUSS_PRIV_ROM_SROM_SROM_PROT_LIMIT_POS      (0)


/*
 * Hibernate wakeup value for PWR_KEY_DELAY
 * Used by firmware during entry into hibernate
 */
#define SFLASH_HIB_KEY_DELAY_ADDRESS                        (0x0ffff250)
#define SFLASH_HIB_KEY_DELAY                                (*(volatile uint16_t *)(0x0ffff250))
#define SFLASH_HIB_KEY_DELAY_DEFAULT                        (0x00000000)

/*
 * Delay (in 12MHz IMO clock cycles) to wait for references to settle on
 * wakeup from hibernate/deepsleep.  PBOD is ignored and system does not
 * resume until this delay expires. Note that the same delay on POR is hard-coded.
 */
#define SFLASH_HIB_KEY_DELAY_WAKEUP_HOLDOFF_MASK            (0x000003ff) /* <0:9> R:RW:X: */
#define SFLASH_HIB_KEY_DELAY_WAKEUP_HOLDOFF_POS             (0)


/*
 * DeepSleep wakeup value for PWR_KEY_DELAY
 * Used by firmware during entry into hibernate
 */
#define SFLASH_DPSLP_KEY_DELAY_ADDRESS                      (0x0ffff252)
#define SFLASH_DPSLP_KEY_DELAY                              (*(volatile uint16_t *)(0x0ffff252))
#define SFLASH_DPSLP_KEY_DELAY_DEFAULT                      (0x00000000)

/*
 * Delay (in 12MHz IMO clock cycles) to wait for references to settle on
 * wakeup from hibernate/deepsleep.  PBOD is ignored and system does not
 * resume until this delay expires. Note that the same delay on POR is hard-coded.
 */
#define SFLASH_DPSLP_KEY_DELAY_WAKEUP_HOLDOFF_MASK          (0x000003ff) /* <0:9> R:RW:X: */
#define SFLASH_DPSLP_KEY_DELAY_WAKEUP_HOLDOFF_POS           (0)


/*
 * SWD pinout selector (not present in TSG4/TSG5-M)
 * Used in BootROM to determine primary or alternate SWD location.
 */
#define SFLASH_SWD_CONFIG_ADDRESS                           (0x0ffff254)
#define SFLASH_SWD_CONFIG                                   (*(volatile uint8_t *)(0x0ffff254))
#define SFLASH_SWD_CONFIG_DEFAULT                           (0x00000000)

/*
 * 0: Use Primary SWD location
 * 1: Use Alternate SWD location
 */
#define SFLASH_SWD_CONFIG_SWD_SELECT                        (1u << 0) /* <0:0> :RW:X: */


/*
 * Listen Window Length
 * Number of clock cycles to wait for SWD acquire to occur during boot.
 */
#define SFLASH_SWD_LISTEN_ADDRESS                           (0x0ffff258)
#define SFLASH_SWD_LISTEN                                   (*(volatile uint32_t *)(0x0ffff258))
#define SFLASH_SWD_LISTEN_DEFAULT                           (0x00000000)

/*
 * Number of clock cycles
 */
#define SFLASH_SWD_LISTEN_CYCLES_MASK                       (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_SWD_LISTEN_CYCLES_POS                        (0)


/*
 * Flash Image Start Address
 * Flash Image start address.  If this field is FFFF_FFFF (recommended) the
 * firmware start address is loaded from vector table locations 0,1 (0000_0000
 * - 0000_0007).  If this field is not FFFF_FFFF it is assumed to be a valid
 * firmware start address.
 */
#define SFLASH_FLASH_START_ADDRESS                          (0x0ffff25c)
#define SFLASH_FLASH_START                                  (*(volatile uint32_t *)(0x0ffff25c))
#define SFLASH_FLASH_START_DEFAULT                          (0x00000000)

/*
 * Start Address
 */
#define SFLASH_FLASH_START_ADDRESS_MASK                     (0xffffffff) /* <0:31> :RW:X: */
#define SFLASH_FLASH_START_ADDRESS_POS                      (0)


/*
 * Checksum Skip Option Register
 * Directs BootROM to perform CHECKSUM check on SR layout during boot or
 * not.
 */
#define SFLASH_SKIP_CHECKSUM_ADDRESS                        (0x0ffff269)
#define SFLASH_SKIP_CHECKSUM                                (*(volatile uint8_t *)(0x0ffff269))
#define SFLASH_SKIP_CHECKSUM_DEFAULT                        (0x00000000)

/*
 * 0: Perform checksum check (see CHECKSUM fueld below)
 * 1: Skip checksum check
 * >1: Undefined - do not use
 */
#define SFLASH_SKIP_CHECKSUM_SKIP_MASK                      (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_SKIP_CHECKSUM_SKIP_POS                       (0)


/*
 * SRSSLT BG Vref trim used during boot
 * This field contains SRSS-Lite Vref trim that is required for flash reads
 * to work.  It is retrieved using trial and error as documented in SAS Part-V,
 * Sec 3.4.  Note: this field exists only in products that use SRSS-Lite
 * (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_ADDRESS                 (0x0ffff26a)
#define SFLASH_INITIAL_PWR_BG_TRIM1                         (*(volatile uint8_t *)(0x0ffff26a))
#define SFLASH_INITIAL_PWR_BG_TRIM1_DEFAULT                 (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_REF_ITRIM_MASK          (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM1_REF_ITRIM_POS           (0)


/*
 * SRSSLT BG Vref trim used during boot
 * This field contains the complement of the PWR_BG_TRIM1 value above.  It
 * is used to verify correct readout in the trial and error process as documented
 * in SAS Part-V, Sec 3.4.  Note: this field exists only in products that
 * use SRSS-Lite (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_ADDRESS             (0x0ffff26b)
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV                     (*(volatile uint8_t *)(0x0ffff26b))
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_DEFAULT             (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_REF_ITRIM_MASK      (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM1_INV_REF_ITRIM_POS       (0)


/*
 * SRSSLT BG Iref trim used during boot
 * This field contains SRSS-Lite Iref trim that is required for flash reads
 * to work.  It is retrieved using trial and error as documented in SAS Part-V,
 * Sec 3.4.  Note: this field exists only in products that use SRSS-Lite
 * (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_ADDRESS                 (0x0ffff26c)
#define SFLASH_INITIAL_PWR_BG_TRIM2                         (*(volatile uint8_t *)(0x0ffff26c))
#define SFLASH_INITIAL_PWR_BG_TRIM2_DEFAULT                 (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_REF_ITRIM_MASK          (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM2_REF_ITRIM_POS           (0)


/*
 * SRSSLT BG Iref trim used during boot
 * This field contains the complement of the PWR_BG_TRIM2 value above.  It
 * is used to verify correct readout in the trial and error process as documented
 * in SAS Part-V, Sec 3.4.  Note: this field exists only in products that
 * use SRSS-Lite (s8srsslt).
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_ADDRESS             (0x0ffff26d)
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV                     (*(volatile uint8_t *)(0x0ffff26d))
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_DEFAULT             (0x00000000)

/*
 * See PWR_BG_TRIM2 in SRSSLT
 */
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_REF_ITRIM_MASK      (0x0000003f) /* <0:5> R:RW:0: */
#define SFLASH_INITIAL_PWR_BG_TRIM2_INV_REF_ITRIM_POS       (0)


/*
 * FLASH IDAC trim used during boot
 * This field contains FLASH IDAC trim that is required for flash reads to
 * work.  It is retrieved using trial and error as documented in SAS Part-V,
 * Sec 3.4.  Note: this field exists only in products that use SRSS-Lite
 * (s8srsslt).
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_ADDRESS           (0x0ffff26e)
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0                   (*(volatile uint8_t *)(0x0ffff26e))
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_DEFAULT           (0x00000000)

/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_IDAC_MASK         (0x0000001f) /* <0:4> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_IDAC_POS          (0)


/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_SLOPE_MASK        (0x000000e0) /* <5:7> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_SLOPE_POS         (5)


/*
 * FLASH IDAC trim used during boot
 * This field contains the complement of the SPCIF_TRIM_M0_DAC0 value above.
 *  It is used to verify correct readout in the trial and error process as
 * documented in SAS Part-V, Sec 3.4.  Note: this field exists only in products
 * that use SRSS-Lite (s8srsslt).
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_ADDRESS       (0x0ffff26f)
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV               (*(volatile uint8_t *)(0x0ffff26f))
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_DEFAULT       (0x00000000)

/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_IDAC_MASK     (0x0000001f) /* <0:4> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_IDAC_POS      (0)


/*
 * See SPCIF_TRIM1
 */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_SLOPE_MASK    (0x000000e0) /* <5:7> R:RW:0: */
#define SFLASH_INITIAL_SPCIF_TRIM_M0_DAC0_INV_SLOPE_POS     (5)


/*
 * Virgin Protection Mode Key
 * Magic 64b Key that indicates part is no longer in Virgin State.  Key is
 * programmed to fixed value of {0xDE, 0xAD, 0xBE, 0xEF, 0x19, 0x67, 0xFA,
 * 0xDE} during manufacturing to indicate PROT_PROTECTION is valid.
 */
#define SFLASH_PROT_VIRGINKEY_ADDRESS(n)                    (0x0ffff270 + ((n) * (0x0001)))
#define SFLASH_PROT_VIRGINKEY(n)                            (*(volatile uint8_t *)(0x0ffff270 + ((n) * 0x0001)))
#define SFLASH_PROT_VIRGINKEY_DEFAULT                       (0x00000000)

/*
 * Key Byte
 */
#define SFLASH_PROT_VIRGINKEY_KEY8_MASK                     (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_PROT_VIRGINKEY_KEY8_POS                      (0)


/*
 * Lot Number (3 bytes)
 * Manufacturing Data, can be used to form Unique DieID.
 */
#define SFLASH_DIE_LOT_ADDRESS(n)                           (0x0ffff278 + ((n) * (0x0001)))
#define SFLASH_DIE_LOT(n)                                   (*(volatile uint8_t *)(0x0ffff278 + ((n) * 0x0001)))
#define SFLASH_DIE_LOT_DEFAULT                              (0x00000000)

/*
 * Lot Number Byte
 */
#define SFLASH_DIE_LOT_LOT_MASK                             (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_LOT_LOT_POS                              (0)


/*
 * Wafer Number
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_WAFER_ADDRESS                            (0x0ffff27b)
#define SFLASH_DIE_WAFER                                    (*(volatile uint8_t *)(0x0ffff27b))
#define SFLASH_DIE_WAFER_DEFAULT                            (0x00000000)

/*
 * Wafer Number
 */
#define SFLASH_DIE_WAFER_WAFER_MASK                         (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_WAFER_WAFER_POS                          (0)


/*
 * X Position on Wafer, CRI Pass/Fail Bin
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_X_ADDRESS                                (0x0ffff27c)
#define SFLASH_DIE_X                                        (*(volatile uint8_t *)(0x0ffff27c))
#define SFLASH_DIE_X_DEFAULT                                (0x00000000)

/*
 * X Position
 */
#define SFLASH_DIE_X_X_MASK                                 (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_X_X_POS                                  (0)


/*
 * Y Position on Wafer, CHI Pass/Fail Bin
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_Y_ADDRESS                                (0x0ffff27d)
#define SFLASH_DIE_Y                                        (*(volatile uint8_t *)(0x0ffff27d))
#define SFLASH_DIE_Y_DEFAULT                                (0x00000000)

/*
 * Y Position
 */
#define SFLASH_DIE_Y_Y_MASK                                 (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_Y_Y_POS                                  (0)


/*
 * Sort1/2/3 Pass/Fail Bin
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_SORT_ADDRESS                             (0x0ffff27e)
#define SFLASH_DIE_SORT                                     (*(volatile uint8_t *)(0x0ffff27e))
#define SFLASH_DIE_SORT_DEFAULT                             (0x00000000)

/*
 * SORT1 Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_S1_PASS                             (1u << 0) /* <0:0> :RW:X: */


/*
 * SORT2 Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_S2_PASS                             (1u << 1) /* <1:1> :RW:X: */


/*
 * SORT3 Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_S3_PASS                             (1u << 2) /* <2:2> :RW:X: */


/*
 * CRI Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_CRI_PASS                            (1u << 3) /* <3:3> :RW:X: */


/*
 * CHI Pass Bin (1) or 0 (Fail Bin)
 */
#define SFLASH_DIE_SORT_CHI_PASS                            (1u << 4) /* <4:4> :RW:X: */


/*
 * ENG Pass Bin
 */
#define SFLASH_DIE_SORT_ENG_PASS                            (1u << 5) /* <5:5> :RW:X: */


/*
 * Minor Revision Number
 * Manufacturing Data, can be used to form Unique DieID
 */
#define SFLASH_DIE_MINOR_ADDRESS                            (0x0ffff27f)
#define SFLASH_DIE_MINOR                                    (*(volatile uint8_t *)(0x0ffff27f))
#define SFLASH_DIE_MINOR_DEFAULT                            (0x00000000)

/*
 * Minor revision number
 */
#define SFLASH_DIE_MINOR_MINOR_MASK                         (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_DIE_MINOR_MINOR_POS                          (0)


/*
 * PE/TE Data
 * PE/TE Specific Data
 */
#define SFLASH_PE_TE_DATA_ADDRESS(n)                        (0x0ffff300 + ((n) * (0x0001)))
#define SFLASH_PE_TE_DATA(n)                                (*(volatile uint8_t *)(0x0ffff300 + ((n) * 0x0001)))
#define SFLASH_PE_TE_DATA_DEFAULT                           (0x00000000)

/*
 * PE/TE Data
 */
#define SFLASH_PE_TE_DATA_DATA8_MASK                        (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_PE_TE_DATA_DATA8_POS                         (0)


/*
 * Preprogram Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_PP_ADDRESS                                   (0x0ffff320)
#define SFLASH_PP                                           (*(volatile uint32_t *)(0x0ffff320))
#define SFLASH_PP_DEFAULT                                   (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_PP_PERIOD_MASK                               (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_PP_PERIOD_POS                                (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_PP_PDAC_MASK                                 (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_PP_PDAC_POS                                  (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_PP_NDAC_MASK                                 (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_PP_NDAC_POS                                  (28)


/*
 * Erase Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_E_ADDRESS                                    (0x0ffff324)
#define SFLASH_E                                            (*(volatile uint32_t *)(0x0ffff324))
#define SFLASH_E_DEFAULT                                    (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_E_PERIOD_MASK                                (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_E_PERIOD_POS                                 (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_E_PDAC_MASK                                  (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_E_PDAC_POS                                   (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_E_NDAC_MASK                                  (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_E_NDAC_POS                                   (28)


/*
 * Program Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_P_ADDRESS                                    (0x0ffff328)
#define SFLASH_P                                            (*(volatile uint32_t *)(0x0ffff328))
#define SFLASH_P_DEFAULT                                    (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_P_PERIOD_MASK                                (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_P_PERIOD_POS                                 (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_P_PDAC_MASK                                  (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_P_PDAC_POS                                   (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_P_NDAC_MASK                                  (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_P_NDAC_POS                                   (28)


/*
 * Erase All - Erase Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_EA_E_ADDRESS                                 (0x0ffff32c)
#define SFLASH_EA_E                                         (*(volatile uint32_t *)(0x0ffff32c))
#define SFLASH_EA_E_DEFAULT                                 (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_EA_E_PERIOD_MASK                             (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_EA_E_PERIOD_POS                              (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_EA_E_PDAC_MASK                               (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_EA_E_PDAC_POS                                (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_EA_E_NDAC_MASK                               (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_EA_E_NDAC_POS                                (28)


/*
 * Erase All - Program Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_EA_P_ADDRESS                                 (0x0ffff330)
#define SFLASH_EA_P                                         (*(volatile uint32_t *)(0x0ffff330))
#define SFLASH_EA_P_DEFAULT                                 (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_EA_P_PERIOD_MASK                             (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_EA_P_PERIOD_POS                              (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_EA_P_PDAC_MASK                               (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_EA_P_PDAC_POS                                (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_EA_P_NDAC_MASK                               (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_EA_P_NDAC_POS                                (28)


/*
 * Erase Sector - Erase Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_ES_E_ADDRESS                                 (0x0ffff334)
#define SFLASH_ES_E                                         (*(volatile uint32_t *)(0x0ffff334))
#define SFLASH_ES_E_DEFAULT                                 (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_ES_E_PERIOD_MASK                             (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_ES_E_PERIOD_POS                              (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_ES_E_PDAC_MASK                               (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_ES_E_PDAC_POS                                (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_ES_E_NDAC_MASK                               (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_ES_E_NDAC_POS                                (28)


/*
 * Erase Sector - Program EO Settings
 * Contains pulse width (SPCIF_TIMER) and DAC (SPCIF_PNDAC) settings.
 */
#define SFLASH_ES_P_EO_ADDRESS                              (0x0ffff338)
#define SFLASH_ES_P_EO                                      (*(volatile uint32_t *)(0x0ffff338))
#define SFLASH_ES_P_EO_DEFAULT                              (0x00000000)

/*
 * Period of timer in clk_spcif_timer ticks.  For regular FLASH, clock is
 * 36MHz from dedicated oscillator. For FLASH-Lite, clock is same as clk_hf,
 * which must be set to 48MHz for 48MHz devices and 12MHz for max 16MHz devices
 */
#define SFLASH_ES_P_EO_PERIOD_MASK                          (0x00ffffff) /* <0:23> :RW:X: */
#define SFLASH_ES_P_EO_PERIOD_POS                           (0)


/*
 * PDAC input. Each increment in PDAC causes an increase of ~0.10V in VPOS
 */
#define SFLASH_ES_P_EO_PDAC_MASK                            (0x0f000000) /* <24:27> :RW:X: */
#define SFLASH_ES_P_EO_PDAC_POS                             (24)


/*
 * NDAC input. Each increment in NDAC causes an increase of ~0.10V in VNEG
 */
#define SFLASH_ES_P_EO_NDAC_MASK                            (0xf0000000) /* <28:31> :RW:X: */
#define SFLASH_ES_P_EO_NDAC_POS                             (28)


/*
 * USB IMO TRIM 24MHz
 * VCTAT Slope for Programming Operations
 */
#define SFLASH_IMO_TRIM_USBMODE_24_ADDRESS                  (0x0ffff33e)
#define SFLASH_IMO_TRIM_USBMODE_24                          (*(volatile uint8_t *)(0x0ffff33e))
#define SFLASH_IMO_TRIM_USBMODE_24_DEFAULT                  (0x00000000)

/*
 * TRIM value for IMO with USB at 24MHz
 */
#define SFLASH_IMO_TRIM_USBMODE_24_TRIM_24_MASK             (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_IMO_TRIM_USBMODE_24_TRIM_24_POS              (0)


/*
 * USB IMO TRIM 48MHz
 * VCTAT Slope for Programming Operations
 */
#define SFLASH_IMO_TRIM_USBMODE_48_ADDRESS                  (0x0ffff33f)
#define SFLASH_IMO_TRIM_USBMODE_48                          (*(volatile uint8_t *)(0x0ffff33f))
#define SFLASH_IMO_TRIM_USBMODE_48_DEFAULT                  (0x00000000)

/*
 * TRIM value for IMO with USB at 24MHz
 */
#define SFLASH_IMO_TRIM_USBMODE_48_TRIM_24_MASK             (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_IMO_TRIM_USBMODE_48_TRIM_24_POS              (0)


/*
 * IMO TempCo Trim Register (SRSS-Lite)
 * CLK_IMO_TRIM3 for each integer frequency 24..48MHz
 * IMO_TCTRIM_LT[0]: 24 MHz
 * IMO_TCTRIM_LT[1]: 25 MHz
 * etc.
 */
#define SFLASH_IMO_TCTRIM_LT_ADDRESS(n)                     (0x0ffff34c + ((n) * (0x0001)))
#define SFLASH_IMO_TCTRIM_LT(n)                             (*(volatile uint8_t *)(0x0ffff34c + ((n) * 0x0001)))
#define SFLASH_IMO_TCTRIM_LT_DEFAULT                        (0x00000050)

/*
 * IMO trim stepsize bits.  These bits are determined at manufacturing time
 * to adjust for process variation.  They are used to tune the stepsize of
 * the FSOFFSET and OFFSET trims.
 */
#define SFLASH_IMO_TCTRIM_LT_STEPSIZE_MASK                  (0x0000001f) /* <0:4> R:RW:16: */
#define SFLASH_IMO_TCTRIM_LT_STEPSIZE_POS                   (0)


/*
 * IMO temperature compesation trim.  These bits are determined at manufacturing
 * time to adjust for temperature dependence. This bits are dependent on
 * frequency and need to be changed using the Cypress provided frequency
 * change algorithm.
 */
#define SFLASH_IMO_TCTRIM_LT_TCTRIM_MASK                    (0x00000060) /* <5:6> R:RW:2: */
#define SFLASH_IMO_TCTRIM_LT_TCTRIM_POS                     (5)


/*
 * IMO Frequency Trim Register (SRSS-Lite)
 * CLK_IMO_TRIM1 for each integer frequency 24..48MHz
 * IMO_TRIM_LT[0]: 24 MHz
 * IMO_TRIM_LT[1]: 25 MHz
 * etc.
 */
#define SFLASH_IMO_TRIM_LT_ADDRESS(n)                       (0x0ffff365 + ((n) * (0x0001)))
#define SFLASH_IMO_TRIM_LT(n)                               (*(volatile uint8_t *)(0x0ffff365 + ((n) * 0x0001)))
#define SFLASH_IMO_TRIM_LT_DEFAULT                          (0x00000000)

/*
 * Frequency trim bits.  These bits are determined at manufacturing time
 * for each FREQ setting (IMO_TRIM2) and stored in SFLASH.
 */
#define SFLASH_IMO_TRIM_LT_OFFSET_MASK                      (0x000000ff) /* <0:7> :RW:X: */
#define SFLASH_IMO_TRIM_LT_OFFSET_POS                       (0)


/*
 * Boot Checksum
 */
#define SFLASH_CHECKSUM_ADDRESS                             (0x0ffff3fe)
#define SFLASH_CHECKSUM                                     (*(volatile uint16_t *)(0x0ffff3fe))
#define SFLASH_CHECKSUM_DEFAULT                             (0x00000000)

/*
 * Checksum of fixed data checked during boot.  This checksum covers all
 * of rows 1,2,3 of macro 0 + row 3 of macro 1 (except this checksum, and
 * row 3 of macro 1 only if it exists).
 */
#define SFLASH_CHECKSUM_CHECKSUM_MASK                       (0x0000ffff) /* <0:15> :RW:X: */
#define SFLASH_CHECKSUM_CHECKSUM_POS                        (0)


/*
 * Divider command register
 * The (PA_SEL_TYPE, PA_SEL_DIV) field pair allows a divider to be phase
 * aligned with another divider. E.g., consider a 48 MHz "clk_hf", and a
 * need for a 12 MHz divided clock A and a 8 MHz divided clock B. Clock 
 * A uses 8.0 integer divider 0 and is created by aligning it to "clk_hf"
 * ((PA_SEL_TYPE, PA_SEL_DIV) is (3, 63)) and DIV_8_CTL0.INT8_DIV is "4-1".
 * Clock  B uses 8.0 integer divider 1 and is created by aligning it to clock
 * A ((PA_SEL_TYPE, PA_SEL_DIV) is (0, 0)) and DIV_8_CTL1.INT8_DIV is "6-1".
 * This guarantees that clock B is phase aligned with clock B: as the smallest
 * common multiple of the two clock periods is 12 "clk_hf" cycles, the clocks
 * A and B will be aligned every 12 "clk_hf" cycles. Note: clock B is phase
 * aligned to clock A, but still uses "clk_hf" as a reference clock for its
 * divider value.
 */
#define PERI_DIV_CMD_ADDRESS                                (0x40010000)
#define PERI_DIV_CMD                                        (*(volatile uint32_t *)(0x40010000))
#define PERI_DIV_CMD_DEFAULT                                (0x0000ffff)

/*
 * (SEL_TYPE, SEL_DIV) specifies the divider on which the command (DISABLE/ENABLE)
 * is performed.
 *
 * If SEL_DIV is "63" and "SEL_TYPE" is "3" (default/reset value), no divider
 * is specified and no clock signal(s) are generated.
 */
#define PERI_DIV_CMD_SEL_DIV_MASK                           (0x0000003f) /* <0:5> R:RW:63: */
#define PERI_DIV_CMD_SEL_DIV_POS                            (0)


/*
 * Specifies the divider type of the divider on which the command is performed:
 * 0: 8.0 (integer) clock dividers.
 * 1: 16.0 (integer) clock dividers.
 * 2: 16.5 (fractional) clock dividers.
 * 3: 24.5 (fractional) clock dividers.
 */
#define PERI_DIV_CMD_SEL_TYPE_MASK                          (0x000000c0) /* <6:7> R:RW:3: */
#define PERI_DIV_CMD_SEL_TYPE_POS                           (6)


/*
 * (PA_SEL_TYPE, PA_SEL_DIV) pecifies the divider to which phase alignment
 * is performed for the clock enable command. Any enabled divider can be
 * used as reference. This allows all dividers to be aligned with each other,
 * even when they are enabled at different times.
 *
 * If PA_SEL_DIV is "63" and "PA_SEL_TYPE" is "3", "clk_hf" is used as reference.
 */
#define PERI_DIV_CMD_PA_SEL_DIV_MASK                        (0x00003f00) /* <8:13> R:RW:63: */
#define PERI_DIV_CMD_PA_SEL_DIV_POS                         (8)


/*
 * Specifies the divider type of the divider to which phase alignment is
 * performed for the clock enable command:
 * 0: 8.0 (integer) clock dividers.
 * 1: 16.0 (integer) clock dividers.
 * 2: 16.5 (fractional) clock dividers.
 * 3: 24.5 (fractional) clock dividers.
 */
#define PERI_DIV_CMD_PA_SEL_TYPE_MASK                       (0x0000c000) /* <14:15> R:RW:3: */
#define PERI_DIV_CMD_PA_SEL_TYPE_POS                        (14)


/*
 * Clock divider disable command (mutually exlusive with ENABLE). SW sets
 * this field to '1' and HW sets this field to '0'.
 *
 * The SEL_DIV and SEL_TYPE fields specify which divider is to be disabled.
 *
 * The HW sets the DISABLE field to '0' immediately and the HW sets the DIV_XXX_CTL.EN
 * field of the divider to '0' immediately.
 */
#define PERI_DIV_CMD_DISABLE                                (1u << 30) /* <30:30> RW1C:RW:0: */


/*
 * Clock divider enable command (mutually exclusive with DISABLE). Typically,
 * SW sets this field to '1' to enable a divider and HW sets this field to
 * '0' to indicate that divider enabling has completed. When a divider is
 * enabled, its integer and fractional (if present) counters are initialized
 * to "0". If a divider is to be re-enabled using different integer and fractional
 * divider values, the SW should follow these steps:
 * 0: Disable the divider using the DIV_CMD.DISABLE field.
 * 1: Configure the divider's DIV_XXX_CTL register.
 * 2: Enable the divider using the DIV_CMD_ENABLE field.
 *
 * The SEL_DIV and SEL_TYPE fields specify which divider is to be enabled.
 * The enabled divider may be phase aligned to either "clk_hf" (typical usage)
 * or to ANY enabled divider.
 *
 * The PA_SEL_DIV and P_SEL_TYPE fields specify the reference divider.
 *
 * The HW sets the ENABLE field to '0' when the enabling is performed and
 * the HW set the DIV_XXX_CTL.EN field of the divider to '1' when the enabling
 * is performed. Note that enabling with phase alignment to a low frequency
 * divider takes time. E.g. To align to a divider that generates a clock
 * of "clk_hf"/n (with n being the integer divider value INT_DIV+1), up to
 * n cycles may be required to perform alignment. Phase alignment to "clk_hf"
 * takes affect immediately. SW can set this field to '0' during phase alignment
 * to abort the enabling process.
 */
#define PERI_DIV_CMD_ENABLE                                 (1u << 31) /* <31:31> RW1C:RW:0: */


/*
 * Programmable clock control register
 */
#define PERI_PCLK_CTL_ADDRESS(n)                            (0x40010100 + ((n) * (0x0004)))
#define PERI_PCLK_CTL(n)                                    (*(volatile uint32_t *)(0x40010100 + ((n) * 0x0004)))
#define PERI_PCLK_CTL_DEFAULT                               (0x000000ff)

/*
 * Specifies one of the dividers of the divider type specified by SEL_TYPE.
 *
 * If SEL_DIV is "63" and "SEL_TYPE" is "3" (default/reset value), no divider
 * is specified and no clock control signal(s) are generated.
 *
 * When transitioning a clock between two out of phase dividers, spurious
 * clock control signals may be generated for one “clk_hf” cycle during this
 * transition. These clock control signals may cause a single clock period
 * that is smaller than any of the two divider periods. To prevent these
 * spurious clock signals, the clock multiplexer can be disconnected (SEL_DIV
 * is "63" and "SEL_TYPE" is "3") for a transition time that is larger than
 * the smaller of the two divider periods.
 */
#define PERI_PCLK_CTL_SEL_DIV_MASK                          (0x00000007) /* <0:2> R:RW:7:PCLK_DIV_ADDR_WIDTH */
#define PERI_PCLK_CTL_SEL_DIV_POS                           (0)


/*
 * Specifies divider type:
 * 0: 8.0 (integer) clock dividers.
 * 1: 16.0 (integer) clock dividers.
 * 2: 16.5 (fractional) clock dividers.
 * 3: 24.5 (fractional) clock dividers.
 */
#define PERI_PCLK_CTL_SEL_TYPE_MASK                         (0x000000c0) /* <6:7> R:RW:3: */
#define PERI_PCLK_CTL_SEL_TYPE_POS                          (6)


/*
 * Divider control register (for 8.0 divider)
 * Smallest of the divider types.
 */
#define PERI_DIV_8_CTL_ADDRESS(n)                           (0x40010200 + ((n) * (0x0004)))
#define PERI_DIV_8_CTL(n)                                   (*(volatile uint32_t *)(0x40010200 + ((n) * 0x0004)))
#define PERI_DIV_8_CTL_DEFAULT                              (0x00000000)

/*
 * Divider enabled. HW sets this field to '1' as a result of an ENABLE command.
 * HW sets this field to '0' as a result on a DISABLE command.
 *
 * Note that this field is retained. As a result, the divider does NOT have
 * to be re-enabled after transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_8_CTL_EN                                   (1u << 0) /* <0:0> RW:R:0: */


/*
 * Integer division by (1+INT8_DIV). Allows for integer divisions in the
 * range [1, 256]. Note: this type of divider does NOT allow for a fractional
 * division.
 *
 * For the generation of a divided clock, the integer division range is restricted
 * to [2, 256].
 *
 * For the generation of a 50/50% duty cycle digital divided clock, the integer
 * division range is resticited to even numbers in the range [2, 256]. The
 * generation of a 50/50 % duty cycle analog divided clock has no restrictions.
 *
 * Note that this field is retained. However, the counter that is used to
 * implement the division is not and will be initialized by HW to "0" when
 * transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_8_CTL_INT8_DIV_MASK                        (0x0000ff00) /* <8:15> R:RW:0: */
#define PERI_DIV_8_CTL_INT8_DIV_POS                         (8)


/*
 * Divider control register (for 16.0 divider)
 */
#define PERI_DIV_16_CTL_ADDRESS(n)                          (0x40010300 + ((n) * (0x0004)))
#define PERI_DIV_16_CTL(n)                                  (*(volatile uint32_t *)(0x40010300 + ((n) * 0x0004)))
#define PERI_DIV_16_CTL_DEFAULT                             (0x00000000)

/*
 * Divider enabled. HW sets this field to '1' as a result of an ENABLE command.
 * HW sets this field to '0' as a result on a DISABLE command.
 *
 * Note that this field is retained. As a result, the divider does NOT have
 * to be re-enabled after transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_CTL_EN                                  (1u << 0) /* <0:0> RW:R:0: */


/*
 * Integer division by (1+INT16_DIV). Allows for integer divisions in the
 * range [1, 65,536]. Note: this type of divider does NOT allow for a fractional
 * division.
 *
 * For the generation of a divided clock, the integer division range is restricted
 * to [2, 65,536].
 *
 * For the generation of a 50/50% duty cycle digital divided clock, the integer
 * division range is restricted to even numbers in the range [2, 65,536].
 * The generation of a 50/50 % duty cycle analog divided clock has no restrictions.
 *
 * Note that this field is retained. However, the counter that is used to
 * implement the division is not and will be initialized by HW to "0" when
 * transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_CTL_INT16_DIV_MASK                      (0x00ffff00) /* <8:23> R:RW:0: */
#define PERI_DIV_16_CTL_INT16_DIV_POS                       (8)


/*
 * Divider control register (for 16.5 divider)
 */
#define PERI_DIV_16_5_CTL_ADDRESS(n)                        (0x40010400 + ((n) * (0x0004)))
#define PERI_DIV_16_5_CTL(n)                                (*(volatile uint32_t *)(0x40010400 + ((n) * 0x0004)))
#define PERI_DIV_16_5_CTL_DEFAULT                           (0x00000000)

/*
 * Divider enabled. HW sets this field to '1' as a result of an ENABLE command.
 * HW sets this field to '0' as a result on a DISABLE command.
 *
 * Note that this field is retained. As a result, the divider does NOT have
 * to be re-enabled after transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_5_CTL_EN                                (1u << 0) /* <0:0> RW:R:0: */


/*
 * Fractional division by (FRAC5_DIV/32). Allows for fractional divisions
 * in the range [0, 31/32]. Note that fractional division results in clock
 * jitter as some clock periods may be 1 "clk_hf" cycle longer than other
 * clock periods.
 *
 * Note that this field is retained. However, the counter that is used to
 * implement the division is not and will be initialized by HW to "0" when
 * transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_5_CTL_FRAC5_DIV_MASK                    (0x000000f8) /* <3:7> R:RW:0: */
#define PERI_DIV_16_5_CTL_FRAC5_DIV_POS                     (3)


/*
 * Integer division by (1+INT16_DIV). Allows for integer divisions in the
 * range [1, 65,536]. Note: combined with fractional division, this divider
 * type allows for a division in the range [1, 65,536 31/32] in 1/32 increments.
 *
 * For the generation of a divided clock, the division range is restricted
 * to [2, 65,536 31/32].
 *
 * For the generation of a 50/50% duty cycle divided clock, the  division
 * range is restricted to [2, 65,536].
 *
 * Note that this field is retained. However, the counter that is used to
 * implement the division is not and will be initialized by HW to "0" when
 * transitioning from DeepSleep to Active power mode.
 */
#define PERI_DIV_16_5_CTL_INT16_DIV_MASK                    (0x00ffff00) /* <8:23> R:RW:0: */
#define PERI_DIV_16_5_CTL_INT16_DIV_POS                     (8)


/*
 * Port selection register
 * Note: these fields are HW:W purely for DfT observe functionality, not
 * for functional reasons.
 */
#define HSIOM_PORT_SEL_ADDRESS(m)                           (0x40020000 + ((m) * (0x0100)))
#define HSIOM_PORT_SEL(m)                                   (*(volatile uint32_t *)(0x40020000 + ((m) * 0x0100)))
#define HSIOM_PORT_SEL_DEFAULT                              (0x00000000)

/*
 * Selects connection for IO pad 0 route.
 */
#define HSIOM_PORT_SEL_IO0_SEL_MASK                         (0x0000000f) /* <0:3> RW:RW:0:IO0 */
#define HSIOM_PORT_SEL_IO0_SEL_POS                          (0)


/*
 * Selects connection for IO pad 1 route.
 */
#define HSIOM_PORT_SEL_IO1_SEL_MASK                         (0x000000f0) /* <4:7> RW:RW:0:IO1 */
#define HSIOM_PORT_SEL_IO1_SEL_POS                          (4)


/*
 * AMUX splitter cell control
 * This register controls the breaking of AMUX buses A and B into multiple
 * segments. It allows for grounding, disconnecting and feeding through.
 * Feeding through is be done straight (from Left segment AMUXBUSA/AMUXBUSB
 * to Right segment AMUXBUSA/AMUXBUSB).
 */
#define HSIOM_AMUX_SPLIT_CTL_ADDRESS(m,n)                   (0x40022100 + ((m) * (0x0100)) + ((n) * (0x0004)))
#define HSIOM_AMUX_SPLIT_CTL(m)                             (*(volatile uint32_t *)(0x40022100 + ((m) * (0x0100)) + ((n) * 0x0004)))
#define HSIOM_AMUX_SPLIT_CTL_DEFAULT                        (0x00000000)

/*
 * T-switch control for Left AMUXBUSA switch:
 * '0': switch open.
 * '1': switch closed.
 */
#define HSIOM_AMUX_SPLIT_CTL_SWITCH_AA_SL                   (1u << 0) /* <0:0> R:RW:0: */


/*
 * T-switch control for Right AMUXBUSA switch:
 * '0': switch open.
 * '1': switch closed.
 */
#define HSIOM_AMUX_SPLIT_CTL_SWITCH_AA_SR                   (1u << 1) /* <1:1> R:RW:0: */


/*
 * T-switch control for AMUXBUSA vssa/ground switch:
 * '0': switch open.
 * '1': switch closed.
 */
#define HSIOM_AMUX_SPLIT_CTL_SWITCH_AA_S0                   (1u << 2) /* <2:2> R:RW:0: */


/*
 * T-switch control for Left AMUXBUSB switch.
 */
#define HSIOM_AMUX_SPLIT_CTL_SWITCH_BB_SL                   (1u << 4) /* <4:4> R:RW:0: */


/*
 * T-switch control for Right AMUXBUSB switch.
 */
#define HSIOM_AMUX_SPLIT_CTL_SWITCH_BB_SR                   (1u << 5) /* <5:5> R:RW:0: */


/*
 * T-switch control for AMUXBUSB vssa/ground switch.
 */
#define HSIOM_AMUX_SPLIT_CTL_SWITCH_BB_S0                   (1u << 6) /* <6:6> R:RW:0: */


/*
 * Power Mode Control
 * Controls the device power mode options and allows observation of current
 * state.
 */
#define PWR_CONTROL_ADDRESS                                 (0x40030000)
#define PWR_CONTROL                                         (*(volatile uint32_t *)(0x40030000))
#define PWR_CONTROL_DEFAULT                                 (0x00000000)

/*
 * Current power mode of the device.  Note that this field cannot be read
 * in all power modes on actual silicon.
 */
#define PWR_CONTROL_POWER_MODE_MASK                         (0x0000000f) /* <0:3> RW:R:0: */
#define PWR_CONTROL_POWER_MODE_POS                          (0)


/*
 * Indicates whether a debug session is active (CDBGPWRUPREQ signal is 1)
 */
#define PWR_CONTROL_DEBUG_SESSION                           (1u << 4) /* <4:4> RW:R:0: */


/*
 * Indicates whether the low power mode regulator is ready to enter DEEPSLEEP
 * mode.
 * 0: If DEEPSLEEP mode is requested, device will enter SLEEP mode.  When
 * low power regulators are ready, device will automatically enter the originally
 * requested mode.
 * 1: Normal operation.  DEEPSLEEP works as described.
 */
#define PWR_CONTROL_LPM_READY                               (1u << 5) /* <5:5> RW:R:0: */


/*
 * Enables the die over temperature sensor.  Must be enabled when using the
 * TEMP_HIGH interrupt.
 */
#define PWR_CONTROL_OVER_TEMP_EN                            (1u << 16) /* <16:16> R:RW:0: */


/*
 * Over-temperature threshold.
 * 0: TEMP_HIGH condition occurs between 120C and 125C.
 * 1: TEMP_HIGH condition occurs between 60C and 75C (used for testing).
 */
#define PWR_CONTROL_OVER_TEMP_THRESH                        (1u << 17) /* <17:17> R:RW:0: */


/*
 * Spare AHB readback bits that are hooked to PWR_PWRSYS_TRIM1.SPARE_TRIM[1:0]
 * through spare logic equivalent to bitwise inversion.  Engineering only.
 */
#define PWR_CONTROL_SPARE_MASK                              (0x000c0000) /* <18:19> RW:R:0: */
#define PWR_CONTROL_SPARE_POS                               (18)


/*
 * Always write 0 except as noted below.
 *
 * PSoC4-S0 and Streetfighter CapSense products may set this bit if Vccd
 * is provided externally (on Vccd pin).  Setting this bit turns off the
 * active regulator and will lead to system reset (BOD) unless both Vddd
 * and Vccd pins are supplied externally.  This register bit only resets
 * for XRES, POR, or a detected BOD.
 */
#define PWR_CONTROL_EXT_VCCD                                (1u << 23) /* <23:23> A:RW:0: */


/*
 * Power System Key&Delay Register
 */
#define PWR_KEY_DELAY_ADDRESS                               (0x40030004)
#define PWR_KEY_DELAY                                       (*(volatile uint32_t *)(0x40030004))
#define PWR_KEY_DELAY_DEFAULT                               (0x000000f8)

/*
 * Delay to wait for references to settle on wakeup from deepsleep.  BOD
 * is ignored and system does not resume until this delay expires. Note that
 * the same delay on POR is hard-coded.  The default assumes the output of
 * the predivider is 48MHz + 3%.  Firmware may scale this setting according
 * to the fastest actual clock frequency that can occur when waking from
 * DEEPSLEEP.
 */
#define PWR_KEY_DELAY_WAKEUP_HOLDOFF_MASK                   (0x000003ff) /* <0:9> R:RW:248: */
#define PWR_KEY_DELAY_WAKEUP_HOLDOFF_POS                    (0)


/*
 * Power ADFT Mode Selection Register
 * Controls System Resources ADFT mode settings and observability.   Writes
 * to this register are ignored and settings in this register have no effect
 * unless the part is in a XRES key selected DfT mode.  Entire register is
 * engineering only.  Note that PWR_DDFT_XRES can be used to enter an XRES
 * key if XRES key sequence is not desired.
 */
#define PWR_ADFT_SELECT_ADDRESS                             (0x40030008)
#define PWR_ADFT_SELECT                                     (*(volatile uint32_t *)(0x40030008))
#define PWR_ADFT_SELECT_DEFAULT                             (0x00000000)

/*
 * Selects the current or voltage source from SRSS-Lite that needs to be
 * observed.  Currents are only available through amuxbusa.  Voltages are
 * available though both amuxbusa and amuxbusb (Kelvin connections).
 */
#define PWR_ADFT_SELECT_SRSS_SEL_MASK                       (0x0000001f) /* <0:4> R:RW:0: */
#define PWR_ADFT_SELECT_SRSS_SEL_POS                        (0)


/*
 * Connect amuxbusa to selected ADFT signal.
 * 0: do not connect
 * 1: connect
 */
#define PWR_ADFT_SELECT_SRSS_AMUXA                          (1u << 8) /* <8:8> R:RW:0: */


/*
 * Connect amuxbusb to selected ADFT signal.
 * 0: do not connect
 * 1: connect
 */
#define PWR_ADFT_SELECT_SRSS_AMUXB                          (1u << 9) /* <9:9> R:RW:0: */


/*
 * When set enables bleeder cells on various switched power nets to accelerate
 * discharge during low power mode testing.
 */
#define PWR_ADFT_SELECT_BLEED_EN                            (1u << 29) /* <29:29> R:RW:0: */


/*
 * Setting this bit will trigger a rst_por_hv_n reset from the s8srsslta.
 *  The system will go through POR reset.
 */
#define PWR_ADFT_SELECT_POR_TRIGGER                         (1u << 30) /* <30:30> R:RW:0: */


/*
 * Enables/disables the system resources ADFT observability.  Disable this
 * bit before changing the ADFT selection.  This prevents glitches and transients
 * from affecting the references.  There is no internal hardware break-before-make.
 */
#define PWR_ADFT_SELECT_SRSS_EN                             (1u << 31) /* <31:31> R:RW:0: */


/*
 * Power DDFT Mode Selection Register
 * Selects the signal sources output to the DDFT outputs of the power subsystem.
 * Entire register is engineering only.
 */
#define PWR_DDFT_SELECT_ADDRESS                             (0x4003000c)
#define PWR_DDFT_SELECT                                     (*(volatile uint32_t *)(0x4003000c))
#define PWR_DDFT_SELECT_DEFAULT                             (0x00000000)

/*
 * Select signal for power DDFT output #0
 */
#define PWR_DDFT_SELECT_DDFT0_SEL_MASK                      (0x0000000f) /* <0:3> R:RW:0: */
#define PWR_DDFT_SELECT_DDFT0_SEL_POS                       (0)


/*
 * Select signal for power DDFT output #1
 */
#define PWR_DDFT_SELECT_DDFT1_SEL_MASK                      (0x000000f0) /* <4:7> R:RW:0: */
#define PWR_DDFT_SELECT_DDFT1_SEL_POS                       (4)


/*
 * XRES DfT Key observer logic test register
 * This register is used to test the XRES TestMode key logic.  It allows
 * a test routine (firmware or ATE driven) to stimulate the key listener
 * and observe its functionality.  Extreme case must be taken in these tests,
 * since they will result in actual test mode entry.  For example, shifting
 * in a scan mode key, will transition the system into scan mode immediately.
 *  Note that test_scan_mode is not observable in this register for that
 * reason.
 */
#define PWR_DDFT_XRES_ADDRESS                               (0x40030010)
#define PWR_DDFT_XRES                                       (*(volatile uint32_t *)(0x40030010))
#define PWR_DDFT_XRES_DEFAULT                               (0x00000000)

/*
 * Tied to the XRES DfT key observation shift register input.
 */
#define PWR_DDFT_XRES_KEY_IN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Tied to the XRES DfT key observation shift register clock.
 */
#define PWR_DDFT_XRES_KEY_CLK                               (1u << 1) /* <1:1> R:RW:0: */


/*
 * Output of the 32-bit hard key shift register
 */
#define PWR_DDFT_XRES_HARD_KEY_OUT                          (1u << 2) /* <2:2> RW:R:0: */


/*
 * Set this to 1 to block all test_key_* signals (blocks both logic side
 * effects and  bits in this register)
 */
#define PWR_DDFT_XRES_BLOCK                                 (1u << 3) /* <3:3> R:RW:0: */


/*
 * Hooked up to test_key_dft_en
 */
#define PWR_DDFT_XRES_KEY_DFT_EN                            (1u << 8) /* <8:8> RW:R:0: */


/*
 * Hooked up to test_key_reg_disable
 */
#define PWR_DDFT_XRES_KEY_REG_DISABLE                       (1u << 9) /* <9:9> RW:R:0: */


/*
 * Hooked up to test_key_safe_mode
 */
#define PWR_DDFT_XRES_KEY_SAFE_MODE                         (1u << 10) /* <10:10> RW:R:0: */


/*
 * Hooked up to test_key_por_circuit
 */
#define PWR_DDFT_XRES_KEY_POR_CIRCUIT                       (1u << 11) /* <11:11> RW:R:0: */


/*
 * Hooked up to test_key_clk_ext
 */
#define PWR_DDFT_XRES_KEY_CLK_EXT                           (1u << 12) /* <12:12> RW:R:0: */


/*
 * Indicates that the 32-bit shift register has observed the correct key
 */
#define PWR_DDFT_XRES_HARD_KEY_OK                           (1u << 30) /* <30:30> RW:R:0: */


/*
 * Indicates that the 125-bit key observer has observed the correct key
 */
#define PWR_DDFT_XRES_SOFT_KEY_OK                           (1u << 31) /* <31:31> RW:R:0: */


/*
 * Test Mode Control Register
 * Controls primary test mode.  This is a single bit that can be written
 * directly from the ATE/Programmer in any protection mode.  It's main function
 * is to signal to the Boot ROM that normal firmware execution is not to
 * commence after boot is complete.  Instead the Boot ROM will enter a wait
 * loop for system commands.
 */
#define TST_MODE_ADDRESS                                    (0x40030014)
#define TST_MODE                                            (*(volatile uint32_t *)(0x40030014))
#define TST_MODE_DEFAULT                                    (0x00000000)

/*
 * 0: SWD not active
 * 1: SWD activated (Line Reset & Connect sequence passed)
 * (Note: this bit replaces TST_CTRL.SWD_CONNECTED and is present in all
 * M0S8 products except TSG4)
 */
#define TST_MODE_SWD_CONNECTED                              (1u << 2) /* <2:2> RW:R:0: */


/*
 * Relevant only for parts that have the alternate XRES mechanism of overloading
 * a GPIO pin temporarily as alternate XRES during test.  When set, this
 * bit blocks the alternate XRES function, such that the pin can be used
 * for normal I/O or for ddft/adft observation.  See SAS Part-V and Part-IX
 * for details. This register bit only resets for XRES, POR, or a detected
 * BOD.
 */
#define TST_MODE_BLOCK_ALT_XRES                             (1u << 28) /* <28:28> A:RW:0: */


/*
 * This bit is set when a XRES test mode key is shifted in.  It is the value
 * of the test_key_dft_en signal.  When this bit is set, the BootROM will
 * not yield execution to the FLASH image (same function as setting TEST_MODE
 * bit below).
 */
#define TST_MODE_TEST_KEY_DFT_EN                            (1u << 30) /* <30:30> RW:R:0: */


/*
 * 0: Normal operation mode
 * 1: Test mode (any test mode)
 * Setting this bit will prevent BootROM from yielding execution to Flash
 * image.
 */
#define TST_MODE_TEST_MODE                                  (1u << 31) /* <31:31> R:RW:0: */


/*
 * Digital DFT Control Register
 * Controls system level DDFT observability muxes and comparators.
 */
#define TST_DDFT_CTRL_ADDRESS                               (0x40030018)
#define TST_DDFT_CTRL                                       (*(volatile uint32_t *)(0x40030018))
#define TST_DDFT_CTRL_DEFAULT                               (0x00000f0f)

/*
 * Select signal for DDFT output #0
 */
#define TST_DDFT_CTRL_DFT_SEL0_MASK                         (0x0000000f) /* <0:3> R:RW:15: */
#define TST_DDFT_CTRL_DFT_SEL0_POS                          (0)


/*
 * Select signal for DDFT output #1
 */
#define TST_DDFT_CTRL_DFT_SEL1_MASK                         (0x00000f00) /* <8:11> R:RW:15: */
#define TST_DDFT_CTRL_DFT_SEL1_POS                          (8)


/*
 * 1: Enables DDFT functionality.  Connects output of DDFT mux to designated
 * DDFT pin.
 */
#define TST_DDFT_CTRL_ENABLE                                (1u << 31) /* <31:31> R:RW:0: */


/*
 * IMO trim down-counter and status (clk_sys)
 */
#define TST_TRIM_CNTR1_ADDRESS                              (0x4003001c)
#define TST_TRIM_CNTR1                                      (*(volatile uint32_t *)(0x4003001c))
#define TST_TRIM_CNTR1_DEFAULT                              (0x80000000)

/*
 * Down-counter clocked on clk_sys. By writing non-zero value to this counter
 * TRIM_CNTR2.COUNTER clears and counts up. TRIM_CNTR1.COUNTER counts down
 * until TRIM_CNTR1.COUNTER==0
 */
#define TST_TRIM_CNTR1_COUNTER_MASK                         (0x0000ffff) /* <0:15> RW:RW:0: */
#define TST_TRIM_CNTR1_COUNTER_POS                          (0)


/*
 * Status bit indicating that TRIM_CNTR1.COUNTER==0 and TRIM_CNT2.COUNTER
 * stopped counting up
 */
#define TST_TRIM_CNTR1_COUNTER_DONE                         (1u << 31) /* <31:31> W:R:1: */


/*
 * IMO trim up-counter  (ddft)
 */
#define TST_TRIM_CNTR2_ADDRESS                              (0x40030020)
#define TST_TRIM_CNTR2                                      (*(volatile uint32_t *)(0x40030020))
#define TST_TRIM_CNTR2_DEFAULT                              (0x00000000)

/*
 * Up-counter clocked on Clock DDFT output #1. When TRIM_CNTR1.COUNT_DONE==1
 * counter stopped and can be read by SW.  The expected final value is related
 * to the ratio of clock frequencies used for the two counters and the value
 * loaded into counter 1: TST_TRIM_CNTR2.COUNTER=(F_cntr2/F_cntr1)*(TST_TRIM_CNTR1.COUNTER-1)
 */
#define TST_TRIM_CNTR2_COUNTER_MASK                         (0x0000ffff) /* <0:15> RW:R:0: */
#define TST_TRIM_CNTR2_COUNTER_POS                          (0)


/*
 * ADFT buffer/comparator control register
 * Controls System Resources ADFT mode settings and observability.  Writes
 * to this register are ignored and settings in this register have no effect
 * unless the part is in a XRES key selected DfT mode.  Entire register is
 * engineering only. Note that PWR_DDFT_XRES can be used to enter an XRES
 * key if XRES key sequence is not desired.
 */
#define TST_ADFT_CTRL_ADDRESS                               (0x40030024)
#define TST_ADFT_CTRL                                       (*(volatile uint32_t *)(0x40030024))
#define TST_ADFT_CTRL_DEFAULT                               (0x00000000)

/*
 * The ADFT buffer/comparator has a common mode dependent offset that can
 * be greatly reduced by using this register bit.  After settling the input
 * signal(s), toggle this bit high briefly to sample and componsate for the
 * offset.  The buffer/comparator output will be unreliable when this bit
 * is set.
 */
#define TST_ADFT_CTRL_BUF_AUTO_ZERO                         (1u << 0) /* <0:0> R:RW:0: */


/*
 * Selects the operating mode for the ADFT buffer/comparator:
 * 0: Voltage buffer, input is amuxbusa, output is amuxbusb
 * 1: Voltage buffer, input is amuxbusb, output is amuxbusa
 * 2: Comparator, input+ is amuxbusa, input- is amuxbusb
 * 3: Comparator, input+ is amuxbusb, input- is amuxbusa
 */
#define TST_ADFT_CTRL_BUF_MODE_MASK                         (0x00000300) /* <8:9> R:RW:0: */
#define TST_ADFT_CTRL_BUF_MODE_POS                          (8)


/*
 * Output of the ADFT comparator, 0 if in analog voltage buffer mode.  This
 * bit is also observable as a DDFT signal (see PWR_DDFT_SELECT).
 */
#define TST_ADFT_CTRL_BUF_COMP_OUT                          (1u << 16) /* <16:16> RW:R:0: */


/*
 * Enables the functionality of the ADFT buffer/comparator
 */
#define TST_ADFT_CTRL_BUF_EN                                (1u << 31) /* <31:31> R:RW:0: */


/*
 * Clock Select Register
 * Configures direction of all clock multiplexers and selectors.  See Section
 * 20.3 in SAS for details on clock network topology.  See PAS for DSI signal
 * connectivity list.
 */
#define CLK_SELECT_ADDRESS                                  (0x40030028)
#define CLK_SELECT                                          (*(volatile uint32_t *)(0x40030028))
#define CLK_SELECT_DEFAULT                                  (0x00000008)

/*
 * Selects a source for clk_hf and dsi_in[0].  Note that not all products
 * support all clock sources.  Selecting a clock source that is not supported
 * will result in undefined behavior.
 */
#define CLK_SELECT_HFCLK_SEL_MASK                           (0x00000003) /* <0:1> R:RW:0: */
#define CLK_SELECT_HFCLK_SEL_POS                            (0)


/*
 * Selects clk_hf predivider value.
 */
#define CLK_SELECT_HFCLK_DIV_MASK                           (0x0000000c) /* <2:3> R:RW:2: */
#define CLK_SELECT_HFCLK_DIV_POS                            (2)


/*
 * Selects clock source for charge pump clock.  This clock is not guaranteed
 * to be glitch free when changing any of its sources or settings.
 */
#define CLK_SELECT_PUMP_SEL_MASK                            (0x00000030) /* <4:5> R:RW:0: */
#define CLK_SELECT_PUMP_SEL_POS                             (4)


/*
 * Select clk_sys prescaler value.
 */
#define CLK_SELECT_SYSCLK_DIV_MASK                          (0x000000c0) /* <6:7> R:RW:0: */
#define CLK_SELECT_SYSCLK_DIV_POS                           (6)


/*
 * ILO Configuration
 * Internal slow speed R/C oscillator (32kHz) configuration register. Note:
 * writes to this register are ignored when WDT_DISABLE_KEY is not set to
 * the magic value defined for it.
 */
#define CLK_ILO_CONFIG_ADDRESS                              (0x4003002c)
#define CLK_ILO_CONFIG                                      (*(volatile uint32_t *)(0x4003002c))
#define CLK_ILO_CONFIG_DEFAULT                              (0x80000000)

/*
 * Master enable for ILO oscillator.  This bit is hardware set whenever the
 * WD_DISABLE_KEY is not set to the magic value.
 */
#define CLK_ILO_CONFIG_ENABLE                               (1u << 31) /* <31:31> RW:RW:1: */


/*
 * IMO Configuration
 * Internal high speed R/C oscillator configuration register. Note that this
 * oscillator comes up active on power up.  The oscillator provides the primary
 * system clock (HFCLK) on power up until firmware configures differently.
 *  This oscillator is also used before system start to count out power up
 * delays.  This is done in fast IMO (FIMO) mode that does not require any
 * external references and runs at a fixed 12MHz.
 */
#define CLK_IMO_CONFIG_ADDRESS                              (0x40030030)
#define CLK_IMO_CONFIG                                      (*(volatile uint32_t *)(0x40030030))
#define CLK_IMO_CONFIG_DEFAULT                              (0x80000000)

/*
 * Master enable for IMO oscillator.  Clearing this bit will disable the
 * IMO.  Don't do this if the system is running off it.
 */
#define CLK_IMO_CONFIG_ENABLE                               (1u << 31) /* <31:31> R:RW:1: */


/*
 * Clock DFT Mode Selection Register
 * Selects which clock signals to bring out to to DFT pins.  Two signals
 * can be selected to enable comparison of clocks.  Clocks can be divided
 * down to deal with slower equipment and I/Os.  See TST_DFT_SELECT for details
 * on bringing these pins out.  Entire register is engineering only.
 */
#define CLK_DFT_SELECT_ADDRESS                              (0x40030034)
#define CLK_DFT_SELECT                                      (*(volatile uint32_t *)(0x40030034))
#define CLK_DFT_SELECT_DEFAULT                              (0x00000000)

/*
 * Select signal for DFT output #0
 */
#define CLK_DFT_SELECT_DFT_SEL0_MASK                        (0x0000000f) /* <0:3> R:RW:0: */
#define CLK_DFT_SELECT_DFT_SEL0_POS                         (0)


/*
 * DFT Output Divide Down.
 */
#define CLK_DFT_SELECT_DFT_DIV0_MASK                        (0x00000030) /* <4:5> R:RW:0: */
#define CLK_DFT_SELECT_DFT_DIV0_POS                         (4)


/*
 * Edge sensitivity for in-line divider on output #0 (only relevant when
 * DIV0>0).
 */
#define CLK_DFT_SELECT_DFT_EDGE0                            (1u << 6) /* <6:6> R:RW:0: */


/*
 * Select signal for DFT output #1
 */
#define CLK_DFT_SELECT_DFT_SEL1_MASK                        (0x00000f00) /* <8:11> R:RW:0: */
#define CLK_DFT_SELECT_DFT_SEL1_POS                         (8)


/*
 * DFT Output Divide Down.
 */
#define CLK_DFT_SELECT_DFT_DIV1_MASK                        (0x00003000) /* <12:13> R:RW:0: */
#define CLK_DFT_SELECT_DFT_DIV1_POS                         (12)


/*
 * Edge sensitivity for in-line divider on output #1 (only relevant when
 * DIV1>0).
 */
#define CLK_DFT_SELECT_DFT_EDGE1                            (1u << 14) /* <14:14> R:RW:0: */


/*
 * Watchdog Disable Key Register
 * This key can be used to disable the watchdog timer reset generation in
 * applications that do not require absolute brown-out safety and do not
 * want to deal with the hassle of feeding the watchdog regularly.  Setting
 * the key will also enable the CLK_ILO_CONFIG.ENABLE bit to be effective.
 *  It will not have any other effect, i.e. the WDT timer/interrupt functionality
 * can still be used.
 */
#define WDT_DISABLE_KEY_ADDRESS                             (0x40030038)
#define WDT_DISABLE_KEY                                     (*(volatile uint32_t *)(0x40030038))
#define WDT_DISABLE_KEY_DEFAULT                             (0x00000000)

/*
 * Disables WDT reset when equal to 0xACED8865.  The WDT reset functions
 * normally for any other setting.
 */
#define WDT_DISABLE_KEY_KEY_MASK                            (0xffffffff) /* <0:31> R:RW:0: */
#define WDT_DISABLE_KEY_KEY_POS                             (0)


/*
 * Watchdog Counter Register
 * Provides actual counter value for watchdog counter.  Watchdog counter
 * always counts up, is free-running and is clocked using clk_lf.
 */
#define WDT_COUNTER_ADDRESS                                 (0x4003003c)
#define WDT_COUNTER                                         (*(volatile uint32_t *)(0x4003003c))
#define WDT_COUNTER_DEFAULT                                 (0x00000000)

/*
 * Current value of WDT Counter
 */
#define WDT_COUNTER_COUNTER_MASK                            (0x0000ffff) /* <0:15> RW:R:0: */
#define WDT_COUNTER_COUNTER_POS                             (0)


/*
 * Watchdog Match Register
 * Firmware provided match value that is compared against WDT_COUNTER.  The
 * expectation is that firmware modifies this register after each match as
 * part of the WDT interrupt service routine.
 */
#define WDT_MATCH_ADDRESS                                   (0x40030040)
#define WDT_MATCH                                           (*(volatile uint32_t *)(0x40030040))
#define WDT_MATCH_DEFAULT                                   (0x00001000)

/*
 * Match value for Watchdog counter.  Every time WDT_COUNTER reaches MATCH
 * an interrupt is generated.  Two unserviced interrupts will lead to a system
 * reset (i.e. at the third match).
 */
#define WDT_MATCH_MATCH_MASK                                (0x0000ffff) /* <0:15> R:RW:4096: */
#define WDT_MATCH_MATCH_POS                                 (0)


/*
 * The number of MSB bits of the watchdog timer that are NOT checked against
 * MATCH.  This value provides control over the time-to-reset of the watchdog
 * (which happens after 3 successive matches).  Note that certain products
 * may enforce a minimum value for this register through design time configuration.
 */
#define WDT_MATCH_IGNORE_BITS_MASK                          (0x000f0000) /* <16:19> R:RW:0: */
#define WDT_MATCH_IGNORE_BITS_POS                           (16)


/*
 * SRSS Interrupt Register
 * The intent is that this register is cleared for every WDT interrupt under
 * all circumstances, including when the system is in DeepSleep.
 */
#define SRSS_INTR_ADDRESS                                   (0x40030044)
#define SRSS_INTR                                           (*(volatile uint32_t *)(0x40030044))
#define SRSS_INTR_DEFAULT                                   (0x00000000)

/*
 * WDT Interrupt Request.  This bit is set each time WDT_COUNTR==WDT_MATCH.
 *  Clearing this bit also feeds the watch dog.  Missing 2 interrupts in
 * a row will generate brown-out reset.  Due to internal synchronization,
 * it takes 2 SYSCLK cycles to update after a W1C.
 */
#define SRSS_INTR_WDT_MATCH                                 (1u << 0) /* <0:0> A:RW1C:0: */


/*
 * Regulator over-temp interrupt.  This interrupt can occur when a short
 * circuit exists on the vccd pin or when extreme loads are applied on IO-cells
 * causing the die to overheat.  Firmware is encourage to shutdown all IO
 * cells and then go to DeepSleep mode when this interrupt occurs if protection
 * against such conditions is desired.
 */
#define SRSS_INTR_TEMP_HIGH                                 (1u << 1) /* <1:1> A:RW1C:0: */


/*
 * SRSS Interrupt Set Register
 * Can be used to set interrupts for firmware testing. Note that SET functionality
 * is not available for WDT.
 */
#define SRSS_INTR_SET_ADDRESS                               (0x40030048)
#define SRSS_INTR_SET                                       (*(volatile uint32_t *)(0x40030048))
#define SRSS_INTR_SET_DEFAULT                               (0x00000000)

/*
 * Writing 1 to this bit internally sets the overtemp interrupt.  This can
 * be observed by reading SRSS_INTR.TEMP_HIGH.  This bit always reads back
 * as zero.
 */
#define SRSS_INTR_SET_TEMP_HIGH                             (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * SRSS Interrupt Mask Register
 * Controls whether interrupt is forwarded to CPU.
 */
#define SRSS_INTR_MASK_ADDRESS                              (0x4003004c)
#define SRSS_INTR_MASK                                      (*(volatile uint32_t *)(0x4003004c))
#define SRSS_INTR_MASK_DEFAULT                              (0x00000000)

/*
 * Clearing this bit will not forward the interrupt to the CPU.  It will
 * not, however, disable the WDT reset generation on 2 missed interrupts.
 */
#define SRSS_INTR_MASK_WDT_MATCH                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * Masks REG_OVERTEMP interrupt
 */
#define SRSS_INTR_MASK_TEMP_HIGH                            (1u << 1) /* <1:1> R:RW:0: */


/*
 * SRSS ADFT control register
 * This register can be used only when in a test mode entered through an
 * XRES:DFT:* key.  It provides direct control over and visibility of the
 * SRSS power and reference circuits. Note that act_reg_en is controlled
 * through PWR_CONTROL.EXT_VCCD.  It is possible to cause behavior that is
 * normally considered illegal, such as disabling a circuit without regard
 * for dependencies.  Engineering only. Note that PWR_DDFT_XRES can be used
 * to enter an XRES key if XRES key sequence is not desired.
 */
#define SRSS_ADFT_CONTROL_ADDRESS                           (0x40030050)
#define SRSS_ADFT_CONTROL                                   (*(volatile uint32_t *)(0x40030050))
#define SRSS_ADFT_CONTROL_DEFAULT                           (0x0000003f)

/*
 * Enables/disables the Active reference.  Set CLK_SELECT.HFCLK_SEL=EXTCLK
 * before disabling the reference.
 */
#define SRSS_ADFT_CONTROL_ACT_REF_EN                        (1u << 0) /* <0:0> R:RW:1: */


/*
 * Enables/disables the Active power comparator.  Set CLK_SELECT.HFCLK_SEL=EXTCLK
 * before disabling the comparator.
 */
#define SRSS_ADFT_CONTROL_ACT_COMP_EN                       (1u << 1) /* <1:1> R:RW:1: */


/*
 * Enables/disables the DeepSleep reference
 */
#define SRSS_ADFT_CONTROL_DPSLP_REF_EN                      (1u << 2) /* <2:2> R:RW:1: */


/*
 * Enables/disables the DeepSleep regulator
 */
#define SRSS_ADFT_CONTROL_DPSLP_REG_EN                      (1u << 3) /* <3:3> R:RW:1: */


/*
 * Enables/disables the DeepSleep power comparator
 */
#define SRSS_ADFT_CONTROL_DPSLP_COMP_EN                     (1u << 4) /* <4:4> R:RW:1: */


/*
 * Mode override for the DeepSleep reference.  0=DEEPSLEEP, 1=ACTIVE.  Clearing
 * this bit allows characterization of the DeepSleep reference during ACTIVE
 * mode.
 */
#define SRSS_ADFT_CONTROL_DPSLP_REF_MODE                    (1u << 5) /* <5:5> R:RW:1: */


/*
 * Indicates Active reference is valid
 */
#define SRSS_ADFT_CONTROL_ACT_REF_VALID                     (1u << 8) /* <8:8> RW:R:0: */


/*
 * Indicates Active regulator is regulating
 */
#define SRSS_ADFT_CONTROL_ACT_REG_VALID                     (1u << 9) /* <9:9> RW:R:0: */


/*
 * Indicates Active comparator is valid
 */
#define SRSS_ADFT_CONTROL_ACT_COMP_OUT                      (1u << 10) /* <10:10> RW:R:0: */


/*
 * Indicates DeepSleep comparator is valid
 */
#define SRSS_ADFT_CONTROL_DPSLP_COMP_OUT                    (1u << 11) /* <11:11> RW:R:0: */


/*
 * Reset Cause Observation Register
 * Indicates the cause for the latest reset(s) that occurred in the system.
 *  Note that resets due to power up and brown-outs below state retention
 * voltages in regulated and unregulated domains cannot be distinguished
 * from eachother.  All bits in this register assert when the corresponding
 * reset cause occurs and must be cleared by firmware.  These bits are cleared
 * by hardware only during XRES, POR or after a detected brown-out.
 */
#define RES_CAUSE_ADDRESS                                   (0x40030054)
#define RES_CAUSE                                           (*(volatile uint32_t *)(0x40030054))
#define RES_CAUSE_DEFAULT                                   (0x00000000)

/*
 * A WatchDog Timer reset has occurred since last power cycle.
 */
#define RES_CAUSE_RESET_WDT                                 (1u << 0) /* <0:0> A:RW1C:0: */


/*
 * A protection violation occurred that requires a RESET.  This includes,
 * but is not limited to, hitting a debug breakpoint while in Privileged
 * Mode.
 */
#define RES_CAUSE_RESET_PROT_FAULT                          (1u << 3) /* <3:3> A:RW1C:0: */


/*
 * Cortex-M0 requested a system reset through it's SYSRESETREQ.  This can
 * be done via a debugger probe or in firmware.
 */
#define RES_CAUSE_RESET_SOFT                                (1u << 4) /* <4:4> A:RW1C:0: */


/*
 * Reset DFT Register
 * Controls the DFT options for the reset system.   Writes to this register
 * are ignored and settings in this register have no effect unless DFT is
 * enabled through a XRES:DFT:* key (see SAS for details).  Note that PWR_DDFT_XRES
 * can be used to enter an XRES key if XRES key sequence is not desired.
 */
#define RES_DFT_ADDRESS                                     (0x40030058)
#define RES_DFT                                             (*(volatile uint32_t *)(0x40030058))
#define RES_DFT_DEFAULT                                     (0x00000000)

/*
 * Setting this bit will disconnect the output of the reset delay line from
 * the reset system blocking soft resets from reaching the logic.  Note:
 * XRES/POR/BOD resets still have other effects, but this bit MUST be cleared
 * before XRES or any other reset can be applied to the system after doing
 * the delay line testing.
 * When DELAY_BLOCK=1, reset DDFT output #0 is connected the input of the
 * delay line and output #1 is connected to the output of the delay line.
 */
#define RES_DFT_DELAY_BLOCK                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * When DELAY_BLOCK=1, this bit is connected to the input of the reset delay
 * line.  Both the input and output can be observed through the DDFT network
 * to measure their timing relationship.
 */
#define RES_DFT_DELAY_IN                                    (1u << 1) /* <1:1> R:RW:0: */


/*
 * Bandgap Trim Register
 * Trim bits for Reference System. Entire register is engineering only.
 */
#define PWR_BG_TRIM1_ADDRESS                                (0x40030f00)
#define PWR_BG_TRIM1                                        (*(volatile uint32_t *)(0x40030f00))
#define PWR_BG_TRIM1_DEFAULT                                (0x00000010)

/*
 * Trims the bandgap reference voltage output.  Used to trim the VBG to the
 * voltage where its temperature curvature is minimal.  Bit [5] is unused
 * within the bandgap block.
 */
#define PWR_BG_TRIM1_REF_VTRIM_MASK                         (0x0000003f) /* <0:5> R:RW:16: */
#define PWR_BG_TRIM1_REF_VTRIM_POS                          (0)


/*
 * Bandgap Trim Register
 * Trim bits for Reference System. Entire register is engineering only.
 */
#define PWR_BG_TRIM2_ADDRESS                                (0x40030f04)
#define PWR_BG_TRIM2                                        (*(volatile uint32_t *)(0x40030f04))
#define PWR_BG_TRIM2_DEFAULT                                (0x0000001c)

/*
 * Trims the bandgap reference current output.  Used to trim the IBG to the
 * voltage where its temperature curvature is minimal.
 */
#define PWR_BG_TRIM2_REF_ITRIM_MASK                         (0x0000003f) /* <0:5> R:RW:28: */
#define PWR_BG_TRIM2_REF_ITRIM_POS                          (0)


/*
 * IMO Frequency Select Register
 * Selects the operating frequency of the IMO
 */
#define CLK_IMO_SELECT_ADDRESS                              (0x40030f08)
#define CLK_IMO_SELECT                                      (*(volatile uint32_t *)(0x40030f08))
#define CLK_IMO_SELECT_DEFAULT                              (0x00000000)

/*
 * Select operating frequency
 */
#define CLK_IMO_SELECT_FREQ_MASK                            (0x00000007) /* <0:2> R:RW:0: */
#define CLK_IMO_SELECT_FREQ_POS                             (0)


/*
 * IMO Trim Register
 * Trims IMO frequency to within datasheet accuracy.  Must be applied
 */
#define CLK_IMO_TRIM1_ADDRESS                               (0x40030f0c)
#define CLK_IMO_TRIM1                                       (*(volatile uint32_t *)(0x40030f0c))
#define CLK_IMO_TRIM1_DEFAULT                               (0x00000080)

/*
 * Frequency trim bits.  These bits are determined at manufacturing time
 * for each FREQ setting (IMO_TRIM2) and stored in SFLASH.  This field is
 * hardware updated during USB osclock mode. This field is mapped to the
 * most significant bits of the IMO trim imo_clk_trim[10:3].  The step size
 * of 1 LSB on this field is approximately 120 kHz.
 */
#define CLK_IMO_TRIM1_OFFSET_MASK                           (0x000000ff) /* <0:7> RW:RW:128: */
#define CLK_IMO_TRIM1_OFFSET_POS                            (0)


/*
 * IMO Trim Register
 * IMO Trim Bits.   Entire register is engineering only.
 */
#define CLK_IMO_TRIM2_ADDRESS                               (0x40030f10)
#define CLK_IMO_TRIM2                                       (*(volatile uint32_t *)(0x40030f10))
#define CLK_IMO_TRIM2_DEFAULT                               (0x00000000)

/*
 * Frequency trim bits.  These bits are not trimmed during manufacturing
 * and kept at 0 under normal operation.  This field is hardware updated
 * during USB osclock mode. This field is mapped to the least significant
 * bits of the IMO trim imo_clk_trim[2:0].  The step size of 1 LSB on this
 * field is approximately 15 kHz.
 */
#define CLK_IMO_TRIM2_FSOFFSET_MASK                         (0x00000007) /* <0:2> RW:RW:0: */
#define CLK_IMO_TRIM2_FSOFFSET_POS                          (0)


/*
 * Power System Trim Register
 * Power System Trim Bits.   Entire register is engineering only.
 */
#define PWR_PWRSYS_TRIM1_ADDRESS                            (0x40030f14)
#define PWR_PWRSYS_TRIM1                                    (*(volatile uint32_t *)(0x40030f14))
#define PWR_PWRSYS_TRIM1_DEFAULT                            (0x00000000)

/*
 * Trims the DeepSleep reference that is used by the DeepSleep regulator
 * and DeepSleep power comparator.
 */
#define PWR_PWRSYS_TRIM1_DPSLP_REF_TRIM_MASK                (0x0000000f) /* <0:3> R:RW:0: */
#define PWR_PWRSYS_TRIM1_DPSLP_REF_TRIM_POS                 (0)


/*
 * Active-Reference temperature compensation trim (repurposed from spare
 * bits).
 * Bits [7:6] - trim the Active-Reference IREF temperature coefficient (TC).
 *   00: TC = 0 (unchanged)
 *   01: TC = +80ppm/C
 *   10: TC = -80ppm/C
 *   11: TC = -150ppm/C
 *
 * Bits [5:4] - trim the Active-Reference VREF temperature coefficient (TC).
 *   00: TC = 0 (unchanged)
 *   01: TC = -50ppm/C
 *   10: TC = -80ppm/C
 *   11: TC = +150ppm/C
 */
#define PWR_PWRSYS_TRIM1_SPARE_TRIM_MASK                    (0x000000f0) /* <4:7> R:RW:0: */
#define PWR_PWRSYS_TRIM1_SPARE_TRIM_POS                     (4)


/*
 * IMO Trim Register
 * IMO Trim Bits.   Entire register is engineering only.
 */
#define CLK_IMO_TRIM3_ADDRESS                               (0x40030f18)
#define CLK_IMO_TRIM3                                       (*(volatile uint32_t *)(0x40030f18))
#define CLK_IMO_TRIM3_DEFAULT                               (0x00000050)

/*
 * IMO trim stepsize bits.  These bits are determined at manufacturing time
 * to adjust for process variation.  They are used to tune the stepsize of
 * the FSOFFSET and OFFSET trims.
 */
#define CLK_IMO_TRIM3_STEPSIZE_MASK                         (0x0000001f) /* <0:4> R:RW:16: */
#define CLK_IMO_TRIM3_STEPSIZE_POS                          (0)


/*
 * IMO temperature compesation trim.  These bits are determined at manufacturing
 * time to adjust for temperature dependence. This bits are dependent on
 * frequency and need to be changed using the Cypress provided frequency
 * change algorithm.
 */
#define CLK_IMO_TRIM3_TCTRIM_MASK                           (0x00000060) /* <5:6> R:RW:2: */
#define CLK_IMO_TRIM3_TCTRIM_POS                            (5)


/*
 * Port output data register
 * Used to read and write the output data for the IO pads in the port. A
 * DR register write changes the output data to the written value. A DR register
 * read reflects the output data (and not the current state of the input
 * data for the IO pads). Using this DR register, Read-Modify-Write sequences
 * are safely performed on a port with some IO pads configured as inputs.
 */
#define GPIO_PRT_DR_ADDRESS(m)                              (0x40040000 + ((m) * (0x0100)))
#define GPIO_PRT_DR(m)                                      (*(volatile uint32_t *)(0x40040000 + ((m) * 0x0100)))
#define GPIO_PRT_DR_DEFAULT                                 (0x00000000)

/*
 * IO pad 0 output data.
 */
#define GPIO_PRT_DR_DATA0                                   (1u << 0) /* <0:0> RW:RW:0:IO0 */


/*
 * IO pad 1 output data.
 */
#define GPIO_PRT_DR_DATA1                                   (1u << 1) /* <1:1> RW:RW:0:IO1 */


/*
 * Port IO pad state register
 * Used to read. Writes to this register have no effect. If the drive mode
 * for the pin is set to high Z Analog, the state will read 0 independent
 * of the voltage on the pin.
 */
#define GPIO_PRT_PS_ADDRESS(m)                              (0x40040004 + ((m) * (0x0100)))
#define GPIO_PRT_PS(m)                                      (*(volatile uint32_t *)(0x40040004 + ((m) * 0x0100)))
#define GPIO_PRT_PS_DEFAULT                                 (0x00000000)

/*
 * IO pad 0 state:
 * 1: Logic high, if the pin voltage is above the input buffer threshold,
 * logic high.
 * 0: Logic low, if the pin voltage is below that threshold, logic low.
 * If the drive mode for the pin is set to high Z Analog, the pin state will
 * read 0 independent of the voltage on the pin.
 */
#define GPIO_PRT_PS_DATA0                                   (1u << 0) /* <0:0> W:R:0:IO0 */


/*
 * IO pad 1 state.
 */
#define GPIO_PRT_PS_DATA1                                   (1u << 1) /* <1:1> W:R:0:IO1 */


/*
 * Reads of this register return the logical state of the filtered pin.
 */
#define GPIO_PRT_PS_FLT_DATA                                (1u << 8) /* <8:8> W:R:0: */


/*
 * Port configuration register
 * Configures the output drive and input buffer state for each pin, and the
 * slew rate and input threshold selection for the whole port. One register
 * is provided per port.
 */
#define GPIO_PRT_PC_ADDRESS(m)                              (0x40040008 + ((m) * (0x0100)))
#define GPIO_PRT_PC(m)                                      (*(volatile uint32_t *)(0x40040008 + ((m) * 0x0100)))
#define GPIO_PRT_PC_DEFAULT                                 (0x00000000)

/*
 * The GPIO drive mode for IO pad 0.
 * Note: when initializing IO's that are connected to a live bus (such as
 * I2C), make sure the HSIOM is properly configured (HSIOM_PRT_SELx) before
 * turning the IO on here to avoid producing glitches on the bus.
 */
#define GPIO_PRT_PC_DM0_MASK                                (0x00000007) /* <0:2> R:RW:0:IO0 */
#define GPIO_PRT_PC_DM0_POS                                 (0)

/*
 * Mode 0 (analog mode): Output buffer off (high Z). Input buffer off.
 */
#define GPIO_PRT_PC_DM0_DM0_OFF                             (0)
/*
 * Mode 1: Output buffer off (high Z). Input buffer on.
 */
#define GPIO_PRT_PC_DM0_DM0_INPUT                           (1)
/*
 * Mode 2: Strong pull down ('0'), weak/resistive pull up (PU). Input buffer
 * on.
 */
#define GPIO_PRT_PC_DM0_DM0_0_PU                            (2)
/*
 * Mode 3: Weak/resistive pull down (PD), strong pull up ('1'). Input buffer
 * on.
 */
#define GPIO_PRT_PC_DM0_DM0_PD_1                            (3)
/*
 * Mode 4: Strong pull down ('0'), open drain (pull up off). Input buffer
 * on.
 */
#define GPIO_PRT_PC_DM0_DM0_0_Z                             (4)
/*
 * Mode 5: Open drain (pull down off), strong pull up ('1'). Input buffer
 * on.
 */
#define GPIO_PRT_PC_DM0_DM0_Z_1                             (5)
/*
 * Mode 6: Strong pull down ('0'), strong pull up ('1'). Input buffer on.
 */
#define GPIO_PRT_PC_DM0_DM0_0_1                             (6)
/*
 * Mode 7: Weak/resistive pull down (PD), weak/resistive pull up (PU). Input
 * buffer on.
 */
#define GPIO_PRT_PC_DM0_DM0_PD_PU                           (7)

/*
 * The GPIO drive mode for IO pad 1.
 */
#define GPIO_PRT_PC_DM1_MASK                                (0x00000038) /* <3:5> R:RW:0:IO1 */
#define GPIO_PRT_PC_DM1_POS                                 (3)


/*
 * The GPIO cells include a VTRIP_SEL signal to alter the input buffer voltage.
 * Note: this bit is ignored for SIO ports, the VTRIP_SEL settings in the
 * SIO register are used instead (a separate VTRIP_SEL is provided for each
 * pin pair).
 * 0: input buffer functions as a CMOS input buffer.
 * 1: input buffer functions as a LVTTL input buffer.
 */
#define GPIO_PRT_PC_PORT_VTRIP_SEL                          (1u << 24) /* <24:24> R:RW:0: */


/*
 * This field controls the output edge rate of all pins on the port:
 * '0': fast.
 * '1': slow.
 */
#define GPIO_PRT_PC_PORT_SLOW                               (1u << 25) /* <25:25> R:RW:0: */


/*
 * This field is used to improve the hysteresis (to 10% of vddio) of the
 * selectable trip point input buffer. The voltage reference comes from the
 * VREFGEN block and is only available when using the VREFGEN block:
 * '0': <= 2.2 V input signaling Voltage.
 * '1': > 2.2 V input signaling Voltage.
 */
#define GPIO_PRT_PC_PORT_HYST_TRIM                          (1u << 27) /* <27:27> R:RW:0:GPIO_OVT */


/*
 * Slew control. Only used in the O_Z drive mode (mode 4: strong pull down,
 * open drain): This field is intended for I2C functionality. See BROS 001-70428
 * for more details.
 */
#define GPIO_PRT_PC_PORT_SLEW_CTL_MASK                      (0x30000000) /* <28:29> R:RW:0:GPIO_OVT */
#define GPIO_PRT_PC_PORT_SLEW_CTL_POS                       (28)


/*
 * This field selects the input buffer reference. The size (1 or 2 bits)
 * and functionality is dependent on the IO cell.
 * For GPIOv2 IO cells, bit PORT_IB_MODE_SEL[1] is not used (GPIOv2 IO cell
 * replaces GPIO IO cell):
 * "0"/"2": CMOS input buffer (PORT_VTRIP_SEL is '0'), LVTTL input buffer
 * (PORT_VTRIP_SEL is '1')
 * "1"/"3": vcchib.
 * For GPIO_OVTv2 and SIOv2 IO cells:
 * "0": CMOS input buffer (PORT_VTRIP_SEL is '0'), LVTTL input buffer (PORT_VTRIP_SEL
 * is '1')
 * "1": vcchib.
 * "2": OVT.
 * "3": Reference (possibly from reference generator cell).
 * For SIO IO cell, this field is present but not used as the SIO IO cell
 * does not provide input buffer mode select functionality (SIOv2 IO cell
 * will replace SIO IO cell, as soon as it is available).
 */
#define GPIO_PRT_PC_PORT_IB_MODE_SEL_MASK                   (0xc0000000) /* <30:31> R:RW:0: */
#define GPIO_PRT_PC_PORT_IB_MODE_SEL_POS                    (30)


/*
 * Port interrupt configuration register
 * This register configures the IRQ configuration for all pins in a port,
 * with the IRQ type being individually pin-configurable.
 */
#define GPIO_PRT_INTR_CFG_ADDRESS(m)                        (0x4004000c + ((m) * (0x0100)))
#define GPIO_PRT_INTR_CFG(m)                                (*(volatile uint32_t *)(0x4004000c + ((m) * 0x0100)))
#define GPIO_PRT_INTR_CFG_DEFAULT                           (0x00000000)

/*
 * Sets which edge will trigger an IRQ for IO pad 0.
 */
#define GPIO_PRT_INTR_CFG_EDGE0_SEL_MASK                    (0x00000003) /* <0:1> R:RW:0:IO0 */
#define GPIO_PRT_INTR_CFG_EDGE0_SEL_POS                     (0)


/*
 * Sets which edge will trigger an IRQ for IO pad 1.
 */
#define GPIO_PRT_INTR_CFG_EDGE1_SEL_MASK                    (0x0000000c) /* <2:3> R:RW:0:IO1 */
#define GPIO_PRT_INTR_CFG_EDGE1_SEL_POS                     (2)


/*
 * Same for the glitch filtered pin (selected by FLT_SELECT).
 */
#define GPIO_PRT_INTR_CFG_FLT_EDGE_SEL_MASK                 (0x00030000) /* <16:17> R:RW:0: */
#define GPIO_PRT_INTR_CFG_FLT_EDGE_SEL_POS                  (16)


/*
 * Selects which pin is routed through the 50ns glitch filter to provide
 * a glitch-safe interrupt.
 */
#define GPIO_PRT_INTR_CFG_FLT_SEL_MASK                      (0x001c0000) /* <18:20> R:RW:0: */
#define GPIO_PRT_INTR_CFG_FLT_SEL_POS                       (18)


/*
 * Port interrupt status register
 * An interrupt cause is cleared (set to '0') by writing a '1' to the corresponding
 * bit field. It is not recommended to write 0xff to clear all interrupt
 * causes, as a new interrupt cause may have occurred between reading the
 * register and clearing. Note that the interrupt cause fields and the associated
 * interrupt provide Hibernate functionality (interrupt causes can be set
 * to '1' and the interrupt can be activated in Hibernate power mode). The
 * PS_DATA fields reflect the logical IO pad states of the port (also found
 * in the PS register).
 */
#define GPIO_PRT_INTR_ADDRESS(m)                            (0x40040010 + ((m) * (0x0100)))
#define GPIO_PRT_INTR(m)                                    (*(volatile uint32_t *)(0x40040010 + ((m) * 0x0100)))
#define GPIO_PRT_INTR_DEFAULT                               (0x00000000)

/*
 * Interrupt pending on IO pad 0. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA0                                 (1u << 0) /* <0:0> A:RW1C:0:IO0 */


/*
 * Interrupt pending on IO pad 1. Firmware writes 1 to clear the interrupt.
 */
#define GPIO_PRT_INTR_DATA1                                 (1u << 1) /* <1:1> A:RW1C:0:IO1 */


/*
 * Deglitched interrupt pending (selected by FLT_SELECT).
 */
#define GPIO_PRT_INTR_FLT_DATA                              (1u << 8) /* <8:8> A:RW1C:0: */


/*
 * `
 */
#define GPIO_PRT_INTR_PS_DATA0                              (1u << 16) /* <16:16> W:R:0:IO0 */


#define GPIO_PRT_INTR_PS_DATA1                              (1u << 17) /* <17:17> W:R:0:IO1 */


/*
 * This is a duplicate of the contents of the PS register, provided here
 * to allow reading of both pin state and interrupt state of the port in
 * a single read operation.
 */
#define GPIO_PRT_INTR_PS_FLT_DATA                           (1u << 24) /* <24:24> W:R:0: */


/*
 * Port configuration register 2
 * Configures the input disable for each pin.
 */
#define GPIO_PRT_PC2_ADDRESS(m)                             (0x40040018 + ((m) * (0x0100)))
#define GPIO_PRT_PC2(m)                                     (*(volatile uint32_t *)(0x40040018 + ((m) * 0x0100)))
#define GPIO_PRT_PC2_DEFAULT                                (0x00000000)

/*
 * Disables the input buffer for IO pad 0 independent of the port control
 * drive mode (PC.DM). This bit should be set when analog signals are present
 * on the pin and PC.DM != 0 is required to use the output driver.
 */
#define GPIO_PRT_PC2_INP_DIS0                               (1u << 0) /* <0:0> R:RW:0:IO0 */


/*
 * Disables the input buffer for IO pad 1.
 */
#define GPIO_PRT_PC2_INP_DIS1                               (1u << 1) /* <1:1> R:RW:0:IO1 */


/*
 * Port output data set register
 * Used to set output data of specific IO pads in the corresponding port
 * to '1', without affecting the output data of the other IO pads in the
 * port. A DR_SET register read returns the same value as a DR register read.
 */
#define GPIO_PRT_DR_SET_ADDRESS(m)                          (0x40040040 + ((m) * (0x0100)))
#define GPIO_PRT_DR_SET(m)                                  (*(volatile uint32_t *)(0x40040040 + ((m) * 0x0100)))
#define GPIO_PRT_DR_SET_DEFAULT                             (0x00000000)

/*
 * IO pad i:
 * '0': Output state DR.DATA[i] not affected.
 * '1': Output state DR.DATA[i] set to '1'.
 */
#define GPIO_PRT_DR_SET_DATA_MASK                           (0x000000ff) /* <0:7> A:RW:0: */
#define GPIO_PRT_DR_SET_DATA_POS                            (0)


/*
 * Port output data clear register
 * Used to clear output data of specific IO pads in the corresponding port
 * to '0', without affecting the output data of the other IO pads in the
 * port. A DR_CLR register read returns the same value as a DR register read.
 */
#define GPIO_PRT_DR_CLR_ADDRESS(m)                          (0x40040044 + ((m) * (0x0100)))
#define GPIO_PRT_DR_CLR(m)                                  (*(volatile uint32_t *)(0x40040044 + ((m) * 0x0100)))
#define GPIO_PRT_DR_CLR_DEFAULT                             (0x00000000)

/*
 * IO pad i:
 * '0': Output state DR.DATA[i] not affected.
 * '1': Output state DR.DATA[i] set to '0'.
 */
#define GPIO_PRT_DR_CLR_DATA_MASK                           (0x000000ff) /* <0:7> A:RW:0: */
#define GPIO_PRT_DR_CLR_DATA_POS                            (0)


/*
 * Port output data invert register
 * Used to invert output data of specific IO pads in the corresponding port,
 * without affecting the output data of the other IO pads in the port. A
 * DR_INV register read returns the same value as a DR register read.
 */
#define GPIO_PRT_DR_INV_ADDRESS(m)                          (0x40040048 + ((m) * (0x0100)))
#define GPIO_PRT_DR_INV(m)                                  (*(volatile uint32_t *)(0x40040048 + ((m) * 0x0100)))
#define GPIO_PRT_DR_INV_DEFAULT                             (0x00000000)

/*
 * IO pad i:
 * '0': Output state DR.DATA[i] not affected.
 * '1': Output state DR.DATA[i] inverted ('0' => '1', '1' => '0').
 */
#define GPIO_PRT_DR_INV_DATA_MASK                           (0x000000ff) /* <0:7> A:RW:0: */
#define GPIO_PRT_DR_INV_DATA_POS                            (0)


/*
 * Interrupt port cause register
 */
#define GPIO_INTR_CAUSE_ADDRESS(m)                          (0x40041000 + ((m) * (0x0100)))
#define GPIO_INTR_CAUSE(m)                                  (*(volatile uint32_t *)(0x40041000 + ((m) * 0x0100)))
#define GPIO_INTR_CAUSE_DEFAULT                             (0x00000000)

/*
 * Each IO port has an associated bit field in this register. The bit field
 * reflects the IO port's interrupt line (bit field i reflects "gpio_interrupts[i]"
 * for IO port i). The register is used when the system uses a shared/combined
 * interrupt line "gpio_interrupt". The SW ISR reads the register to deternine
 * which IO port(s) is responsible for the shared/combined interrupt line
 * "gpio_interrupt". Once, the IO port(s) is determined, the IO port's INTR
 * register is read to determine the IO pad(s) in the IO port that caused
 * the interrupt.
 */
#define GPIO_INTR_CAUSE_PORT_INT_MASK                       (0x0000001f) /* <0:4> W:R:0:GPIO_PORT_NR */
#define GPIO_INTR_CAUSE_PORT_INT_POS                        (0)


/*
 * IO SELF TEST control register for DfT purposes only
 * This register is used to significantly reduce the test time for IO cells.
 * It also avoids the need to develop a large amount of chip specific functional
 * test vectors.
 * With ATPG it is not possible to get full stuck-at fault coverage for some
 * IO cell inputs (“hld_ovr, oe_n, analog_en, analog_sel, analog_pol”). This
 * register gives direct controlabilty on those  inputs of all IO cells by
 * bypassing the functional paths. That allows generic (not chip specific)
 * ROOS code (also SWD IO cells are included) to get DfT fault-coverage for
 * these signals. This register is used in conjunction with the GPIO.PC.dm[2:0],
 * GPIO.DR.out and SRSS.CORE.PWR_STOP.FREEZE for control and results are
 * observed through GPIO.PS.data.
 * Only one IO cell on the whole chip gets IO_TEST_0  and only one gets IO_TEST_1
 * and default values of IO_TEST_0 is '0' and IO_TEST_1 is '1'. Also only
 * one IO cell on the whole chip gets asssigned ADFT-0 and only one gets
 * asssigned ADFT-1. All four IO_TEST_0/1 and ADFT-0/1 pins are assigned
 * in the product pin spreadsheet.
 */
#define GPIO_DFT_IO_TEST_ADDRESS(m)                         (0x40041010 + ((m) * (0x0100)))
#define GPIO_DFT_IO_TEST(m)                                 (*(volatile uint32_t *)(0x40041010 + ((m) * 0x0100)))
#define GPIO_DFT_IO_TEST_DEFAULT                            (0x02020200)

/*
 * DfT IO SELF TEST mode:
 */
#define GPIO_DFT_IO_TEST_DFT_IO_TEST_MODE_MASK              (0x00000003) /* <0:1> R:RW:0: */
#define GPIO_DFT_IO_TEST_DFT_IO_TEST_MODE_POS               (0)


/*
 * “hld_ovr” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “hld_ovr” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “hld_ovr” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_HLD_OVR_0                      (1u << 8) /* <8:8> R:RW:0: */


/*
 * “oe_n” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “oe_n” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “oe_n” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_OE_N_0                         (1u << 9) /* <9:9> R:RW:1: */


/*
 * “analog_en” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_en” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “analog_en” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANALOG_EN_0                    (1u << 10) /* <10:10> R:RW:0: */


/*
 * “analog_sel” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_sel” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “analog_sel” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_SEL_0                      (1u << 11) /* <11:11> R:RW:0: */


/*
 * “analog_pol” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_pol” of the ADFT-0  assigned IO cell.
 * TEST_ANA: Connects to “analog_pol” of the IO_TEST_0  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_POL_0                      (1u << 12) /* <12:12> R:RW:0: */


/*
 * “hld_ovr” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “hld_ovr” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “hld_ovr” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_HLD_OVR_1                      (1u << 16) /* <16:16> R:RW:0: */


/*
 * “oe_n” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “oe_n” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “oe_n” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_OE_N_1                         (1u << 17) /* <17:17> R:RW:1: */


/*
 * “analog_en” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_en” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “analog_en” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANALOG_EN_1                    (1u << 18) /* <18:18> R:RW:0: */


/*
 * “analog_sel” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_sel” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “analog_sel” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_SEL_1                      (1u << 19) /* <19:19> R:RW:0: */


/*
 * “analog_pol” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_pol” of the ADFT-1  assigned IO cell.
 * TEST_ANA: Connects to “analog_pol” of the IO_TEST_1  assigned IO cell.
 * TEST_GEN: not used.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_POL_1                      (1u << 20) /* <20:20> R:RW:0: */


/*
 * “hld_ovr” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “hld_ovr” of all IO cells other than ADFT-0/1 .
 * TEST_ANA: Connects to “hld_ovr” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “hld_ovr” of all IO cells.
 */
#define GPIO_DFT_IO_TEST_DFT_HLD_OVR_2                      (1u << 24) /* <24:24> R:RW:0: */


/*
 * “oe_n” DfT control for IO cells depending on DFT_IO_TEST_MODE as given
 * below.
 * TEST_ADFT: Connects to “oe_n” of all IO cells other than ADFT-0/1.
 * TEST_ANA: Connects to “oe_n” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “oe_n” of all IO cells
 */
#define GPIO_DFT_IO_TEST_DFT_OE_N_2                         (1u << 25) /* <25:25> R:RW:1: */


/*
 * “analog_en” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_en” of all IO cells other than ADFT-0/1.
 * TEST_ANA: DFT_ANALOG_EN_2 && DM[0] connects to “analog_en” of all IO cells
 * other than IO_TEST_0/1.
 * TEST_GEN: Connects to “analog_en” of all IO cells
 */
#define GPIO_DFT_IO_TEST_DFT_ANALOG_EN_2                    (1u << 26) /* <26:26> R:RW:0: */


/*
 * “analog_sel” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_sel” of all IO cells other than ADFT-0/1.
 * TEST_ANA: Connects to “analog_sel” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “analog_sel” of all IO cells.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_SEL_2                      (1u << 27) /* <27:27> R:RW:0: */


/*
 * “analog_pol” DfT control for IO cells depending on DFT_IO_TEST_MODE as
 * given below.
 * TEST_ADFT: Connects to “analog_pol” of all IO cells other than ADFT-0/1.
 * TEST_ANA: Connects to “analog_pol” of all IO cells other than IO_TEST_0/1.
 * TEST_GEN: Connects to “analog_pol” of all IO cells.
 */
#define GPIO_DFT_IO_TEST_DFT_ANA_POL_2                      (1u << 28) /* <28:28> R:RW:0: */


/*
 * Generic control register.
 */
#define SCB_CTRL_ADDRESS(m)                                 (0x40050000 + ((m) * (0x10000)))
#define SCB_CTRL(m)                                         (*(volatile uint32_t *)(0x40050000 + ((m) * 0x10000)))
#define SCB_CTRL_DEFAULT                                    (0x0300000f)

/*
 * Serial interface bit period oversampling factor expressed in lP clock
 * cycles. Used for SPI and UART functionality. OVS + 1 IP clock cycles constitute
 * a single serial interface clock/bit cycle. The IP clock is provided by
 * the programmable clock IP. This field is NOT used in externally clocked
 * mode. If OVS is odd, the oversampling factor is even and the first and
 * second phase of the interface clock period are the same. If OVS is even,
 * the oversampling factor is odd and the first phase of the interface clock
 * period is 1 peripheral clock cycle longer than the second phase of the
 * interface clock period.
 *
 * In SPI master mode, the valid range is [3, 15]. At an IP frequency of
 * 48 MHz, the maximum IP bit rate is 12 Mbps. The effective system bit rate
 * is dependent on the external SPI slave that we communicate with. If the
 * SPI output clock "spi_clk_out" to SPI MISO input "spi_miso_in" round trip
 * delay is introducing significant delays (multiple "spi_clk_out" cycles),
 * it may be necessary to increase OVS and/or to set SPI_CTRL.LATE_MISO_SAMPLE
 * to '1' to achieve the maximum possible system bit rate. The maximum IP
 * bit rate of 12 Mbps provides an IP centric perspective, assuming ideal
 * (0 ns) IO cell and routing (chip and board) delay. The required IP clock/IF
 * clock ratio increases and the calculated maximum bit rate decreases, when
 * realistic chip and routing delays are taken into account.
 *
 * In SPI slave mode, the OVS field is NOT used. However, there is a frequency
 * requirement for the IP clock wrt. the interface (IF) clock to guarantee
 * functional correct behavior. This requirement is expressed as a ratio:
 * IP clock/IF clock. The ratio is dependent on the setting of RX_CTRL.MEDIAN
 * and the external SPI master's capability to support "late MISO sample"
 * functionality (similar to our SPI master functionality represented by
 * SPI_CTRL.LATE_MISO_SAMPLE):
 * - MEDIAN is '0' and external SPI master has NO "late MISO sample functionality":
 * IP clock/IF clock >= 6. At a IP frequency of 48 MHz,  the maximum bit
 * rate is 8 Mbps.
 * - MEDIAN is '0' and external SPI master has "late MISO sample functionality":
 * IP clock/IF clock >= 3. At a IP frequency of 48 MHz,  the maximum bit
 * rate is 16 Mbps.
 * - MEDIAN is '1' and external SPI master has NO "late MISO sample functionality":
 * IP clock/IF clock >= 8. At a IP frequency of 48 MHz,  the maximum bit
 * rate is 6 Mbps.
 * - MEDIAN is '1' and external SPI master has "late MISO sample functionality":
 * IP clock/IF clock >= 4. At a IP frequency of 48 MHz,  the maximum bit
 * rate is 12 Mbps.
 * The maximum bit rates provide an IP centric perspective, assuming ideal
 * (0 ns) IO cell and routing (chip and board) delay. The required IP clock/IF
 * clock ratio increases and the calculated maximum bit rate decreases, when
 * realistic chip and routing delays are taken into account.
 *
 * In UART standard submode (including LIN), the valid range is [7, 15].
 * In UART SmartCard submode, the valid range is [7, 15].
 *
 * In UART TX IrDA submode this field indirectly specifies the oversampling.
 * The oversampling determines the interface clock/bit cycle and the width
 * of the pulse.  Only normal transmission mode is supported, the pulse is
 * roughly 3/16 of the bit period (for all bit rates). There is only one
 * valid OVS value:
 * - 0: 16 times oversampling.
 *            IP clock frequency of 16*115.2 KHz for 115.2 Kbps.
 *            IP clock frequency of 16*57.6 KHz for 57.6 Kbps.
 *            IP clock frequency of 16*38.4 KHz for 38.4 Kbps.
 *            IP clock frequency of 16*19.2 KHz for 19.2 Kbps.
 *            IP clock frequency of 16*9.6 KHz for 9.6 Kbps.
 *            IP clock frequency of 16*2.4 KHz for 2.4 Kbps.
 *            IP clock frequency of 16*1.2 KHz for 1.2 Kbps.
 * - all other values are not used in normal mode.
 *
 * In UART RX IrDA submode (1.2, 2.4, 9.6, 19.2, 38.4, 57.6 and 115.2 Kbps)
 * this field indirectly specifies the oversampling. The oversampling determines
 * the interface clock/bit cycle and the width of the pulse. In normal transmission
 * mode, this pulse is roughly 3/16 of the bit period (for all bit rates).
 * In low power transmission mode, this pulse is potentially smaller (down
 * to 1.62 us typical and 1.41 us minimal) than 3/16 of the bit period (for
 * < 115.2 Kbps bitrates). Pulse widths greater or equal than two IP clock
 * cycles are guaranteed to be detected by the receiver. Pulse widths less
 * than two IP clock cycles and greater or equal than one IP clock cycle
 * may be detected by the receiver. Pulse widths less than one IP clock cycle
 * will not be detected by the receiver. RX_CTRL.MEDIAN should be set to
 * '1' for IrDA receiver functionality. The IP clock (as provided by the
 * programmable clock IP) and the oversampling together determine the IrDA
 * bitrate. Normal mode, OVS field values (with the required IP clock frequency):
 * - 0: 16 times oversampling.
 *            IP clock frequency of 16*115.2 KHz for 115.2 Kbps.
 *            IP clock frequency of 16*57.6 KHz for 57.6 Kbps.
 *            IP clock frequency of 16*38.4 KHz for 38.4 Kbps.
 *            IP clock frequency of 16*19.2 KHz for 19.2 Kbps.
 *            IP clock frequency of 16*9.6 KHz for 9.6 Kbps.
 *            IP clock frequency of 16*2.4 KHz for 2.4 Kbps.
 *            IP clock frequency of 16*1.2 KHz for 1.2 Kbps.
 * - all other values are not used in normal mode.
 * Low power mode, OVS field values (with the required IP clock frequency):
 * - 0: 16 times oversampling.
 *            IP clock frequency of 16*115.2 KHz for 115.2 Kbps.
 * - 1: 32 times oversampling.
 *            IP clock frequency of 32*57.6 KHz for 57.6 Kbps.
 * - 2: 48 times oversampling.
 *            IP clock frequency of 48*38.4 KHz for 38.4 Kbps.
 * - 3: 96 times oversampling.
 *            IP clock frequency of 96*19.2 KHz for 19.2 Kbps.
 * - 4: 192 times oversampling.
 *            IP clock frequency of 192*9.6 KHz for 9.6 Kbps.
 * - 5: 768 times oversampling.
 *            IP clock frequency of 768*2.4 KHz for 2.4 Kbps.
 * - 6: 1536 times oversampling.
 *            IP clock frequency of 1536*1.2 KHz for 1.2 Kbps.
 * - all other values are not used in low power mode.
 */
#define SCB_CTRL_OVS_MASK                                   (0x0000000f) /* <0:3> R:RW:15: */
#define SCB_CTRL_OVS_POS                                    (0)


/*
 * Internally clocked mode ('0') or externally clocked mode ('1') address
 * matching (I2C) or selection (SPI). In internally clocked mode, the serial
 * interface protocols run off the peripheral clock. In externally clocked
 * mode, the serial interface protocols run off the clock as provided by
 * the serial interface. Externally clocked mode is only used for synchronous
 * serial interface protocols (SPI and I2C) in slave mode. In SPI mode, only
 * Motorola submode (all Motorola modes: 0, 1, 2, 3) is supported.
 *
 * In UART mode this field should be '0'.
 */
#define SCB_CTRL_EC_AM_MODE                                 (1u << 8) /* <8:8> R:RW:0:EC */


/*
 * Internally clocked mode ('0') or externally clocked mode ('1') operation.
 * In internally clocked mode, the serial interface protocols run off the
 * peripheral clock. In externally clocked mode, the serial interface protocols
 * run off the clock as provided by the serial interface. Externally clocked
 * operation mode is only used for synchronous serial interface protocols
 * (SPI and I2C) in slave mode AND EZ mode. In SPI mode, only Motorola submode
 * (all Motorola modes: 0, 1, 2, 3) is supported. The maximum SPI slave,
 * EZ mode bitrate is 48 Mbps (transmission and IO delays outside the IP
 * will degrade the effective bitrate).
 *
 * In UART mode this field should be '0'.
 */
#define SCB_CTRL_EC_OP_MODE                                 (1u << 9) /* <9:9> R:RW:0:EC */


/*
 * Non EZ mode ('0') or EZ mode ('1'). In EZ mode, a meta protocol is applied
 * to the serial interface protocol. This meta protocol adds meaning to the
 * data frames transferred by the serial interface protocol: a data frame
 * can represent a memory address, a write memory data element or a read
 * memory data element. EZ mode is only used for synchronous serial interface
 * protocols: SPI and I2C. In SPI mode, only Motorola submode (all Motorola
 * modes: 0, 1, 2, 3) is supported and the transmitter should use continuous
 * data frames; i.e. data frames mot seperated by slave deselection. This
 * mode is only applicable to slave functionality. In EZ mode, the slave
 * can read from and write to an addressable memory structure of 32 bytes.
 * In EZ mode, data frames should 8-bit in size and should be transmitted
 * and received with the Most Significant Bit (MSB) first.
 *
 * In UART mode this field should be '0'.
 */
#define SCB_CTRL_EZ_MODE                                    (1u << 10) /* <10:10> R:RW:0:EZ */


/*
 * Determines the number of bits per FIFO data element:
 * '0': 16-bit FIFO data elements.
 * '1': 8-bit FIFO data elements. This mode doubles the amount of FIFO entries,
 * but  TX_CTRL.DATA_WIDTH and RX_CTRL.DATA_WIDTH are restricted to [0, 7].
 */
#define SCB_CTRL_BYTE_MODE                                  (1u << 11) /* <11:11> R:RW:0: */


/*
 * Determines CMD_RESP mode of operation:
 * '0': CMD_RESP mode disabled.
 * '1': CMD_RESP mode enabled (also requires EC_AM_MODE and EC_OP_MODE to
 * be set to '1').
 */
#define SCB_CTRL_CMD_RESP_MODE                              (1u << 12) /* <12:12> R:RW:0:CMD_RESP */


/*
 * Determines whether a received matching address is accepted in the RX FIFO
 * ('1') or not ('0').
 *
 * In I2C mode, this field is used to allow the slave to put the received
 * slave address or general call address in the RX FIFO. Note that a received
 * matching address is put in the RX FIFO when ADDR_ACCEPT is '1' for both
 * I2C read and write transfers.
 *
 * In multi-processor UART receiver mode, this field is used to allow the
 * receiver to put the received address in the RX FIFO. Note: non-matching
 * addresses are never put in the RX FIFO.
 */
#define SCB_CTRL_ADDR_ACCEPT                                (1u << 16) /* <16:16> R:RW:0: */


/*
 * Only used in externally clocked mode. If the externally clocked logic
 * and the MMIO SW accesses to EZ memory coincide/collide, this bit determines
 * whether a SW access should block and result in bus wait states ('BLOCK
 * is 1') or not (BLOCK is '0'). IF BLOCK is ‘0’ and the accesses collide,
 * MMIO read operations return 0xffff:ffff and MMIO write operations are
 * ignored. Colliding accesses are registered as interrupt causes: field
 * BLOCKED of MMIO registers INTR_TX and INTR_RX.
 */
#define SCB_CTRL_BLOCK                                      (1u << 17) /* <17:17> R:RW:0:EC */


/*
 * Mode of operation (3: Reserved)
 */
#define SCB_CTRL_MODE_MASK                                  (0x03000000) /* <24:25> R:RW:3: */
#define SCB_CTRL_MODE_POS                                   (24)

/*
 * Inter-Integrated Circuits (I2C) mode.
 */
#define SCB_CTRL_MODE_MODE_I2C                              (0)
/*
 * Serial Peripheral Interface (SPI) mode.
 */
#define SCB_CTRL_MODE_MODE_SPI                              (1)
/*
 * Universal Asynchronous Receiver/Transmitter (UART) mode.
 */
#define SCB_CTRL_MODE_MODE_UART                             (2)

/*
 * IP enabled ('1') or not ('0'). The proper order in which to initialize
 * the IP is as follows:
 * - Program protocol specific information using SPI_CTRL, UART_CTRL (and
 * UART_TX_CTRL and UART_RX_CTRL) or I2C_CTRL. This includes selection of
 * a submode, master/slave functionality and transmitter/receiver functionality
 * when applicable.
 * - Program generic transmitter (TX_CTRL) and receiver (RX_CTRL) information.
 * This includes enabling of the transmitter and receiver functionality.
 * - Program transmitter FIFO (TX_FIFO_CTRL) and receiver FIFO (RX_FIFO_CTRL)
 * information.
 * - Program CTRL to enable IP, select the specific operation mode and oversampling
 * factor.
 * When the IP is enabled, no control information should be changed. Changes
 * should be made AFTER disabling the IP, e.g. to modify the operation mode
 * (from I2C to SPI) or to go from externally to internally clocked. The
 * change takes effect after the IP is re-enabled. Note that disabling the
 * IP will cause re-initialization of the design and associated state is
 * lost (e.g. FIFO content).
 */
#define SCB_CTRL_ENABLED                                    (1u << 31) /* <31:31> R:RW:0: */


/*
 * Generic status register.
 */
#define SCB_STATUS_ADDRESS(m)                               (0x40050004 + ((m) * (0x10000)))
#define SCB_STATUS(m)                                       (*(volatile uint32_t *)(0x40050004 + ((m) * 0x10000)))
#define SCB_STATUS_DEFAULT                                  (0x00000000)

/*
 * Inidicates whether the externally clocked logic is potentially accessing
 * the EZ memory (this is only possible in EZ mode). This bit can be used
 * by SW to determine whether it is safe to issue a SW access to the EZ memory
 * (without bus wait states (a blocked SW access) or bus errors being generated).
 * Note that the INTR_TX.BLOCKED and INTR_RX.BLOCKED interrupt causes are
 * used to indicate whether a SW access was actually blocked by externally
 * clocked logic.
 */
#define SCB_STATUS_EC_BUSY                                  (1u << 0) /* <0:0> W:R:Undefined: */


/*
 * Command/response control register.
 */
#define SCB_CMD_RESP_CTRL_ADDRESS(m)                        (0x40050008 + ((m) * (0x10000)))
#define SCB_CMD_RESP_CTRL(m)                                (*(volatile uint32_t *)(0x40050008 + ((m) * 0x10000)))
#define SCB_CMD_RESP_CTRL_DEFAULT                           (0x00000000)

/*
 * I2C/SPI read base address for CMD_RESP mode. Address is used by a I2C
 * CMD_RESP mode read transfer (CTRL.MODE is I2C) or a SPI CMD_RESP mode
 * read transfer (CTRL.MODE is SPI): at the start of a read transfer BASE_RD_ADDR
 * is copied to CMD_RESP_STATUS.CURR_RD_ADDR. This field should not be modified
 * during ongoing bus transfers.
 */
#define SCB_CMD_RESP_CTRL_BASE_RD_ADDR_MASK                 (0x0000001f) /* <0:4> R:RW:0:EZ_DATA_NR_LOG2 */
#define SCB_CMD_RESP_CTRL_BASE_RD_ADDR_POS                  (0)


/*
 * I2C/SPI read base address for CMD_RESP mode. Address is used by a I2C
 * CMD_RESP mode write transfer (CTRL.MODE is I2C) or a SPI CMD_RESP mode
 * write transfer (CTRL.MODE is SPI): at the start of a write transfer BASE_WE_ADDR
 * is copied to CMD_RESP_STATUS.CURR_WR_ADDR. This field should not be modified
 * during ongoing bus transfers.
 */
#define SCB_CMD_RESP_CTRL_BASE_WR_ADDR_MASK                 (0x001f0000) /* <16:20> R:RW:0:EZ_DATA_NR_LOG2 */
#define SCB_CMD_RESP_CTRL_BASE_WR_ADDR_POS                  (16)


/*
 * Command/response status register.
 * The register fields reflect register states without a default/reset value
 * (CURR_RD_ADDR and CURR_WR_ADDR) or reflect an external bus state. Therefore,
 * these registers are undefined after the IP is enabled.
 */
#define SCB_CMD_RESP_STATUS_ADDRESS(m)                      (0x4005000c + ((m) * (0x10000)))
#define SCB_CMD_RESP_STATUS(m)                              (*(volatile uint32_t *)(0x4005000c + ((m) * 0x10000)))
#define SCB_CMD_RESP_STATUS_DEFAULT                         (0x00000000)

/*
 * I2C/SPI read current address for CMD_RESP mode. HW increments the field
 * after a read access to the memory buffer. However, when the last memory
 * buffer address is reached, the address is NOT incremented (but remains
 * at the maximim memory buffer address).
 *
 * The field is used to determine how many bytes have been read (# bytes
 * = CURR_RD_ADDR - CMD_RESP_CTRL.BASE_RD_ADDR).
 *
 * This field is reliable during when there is no bus transfer. This field
 * is potentially unreliable when there is a bus transfer bus transfer: when
 * CMD_RESP_EC_BUSY is '0', the field is reliable.
 */
#define SCB_CMD_RESP_STATUS_CURR_RD_ADDR_MASK               (0x0000001f) /* <0:4> W:R:Undefined:EZ_DATA_NR_LOG2 */
#define SCB_CMD_RESP_STATUS_CURR_RD_ADDR_POS                (0)


/*
 * I2C/SPI write current address for CMD_RESP mode. HW increments the field
 * after a read access to the memory buffer. However, when the last memory
 * buffer address is reached, the address is NOT incremented (but remains
 * at the maximim memory buffer address).
 *
 * The field is used to determine how many bytes have been written (# bytes
 * = CURR_WR_ADDR - CMD_RESP_CTRL.BASE_WR_ADDR).
 *
 * This field is reliable during when there is no bus transfer. This field
 * is potentially unreliable when there is a bus transfer bus transfer: when
 * CMD_RESP_EC_BUSY is '0', the field is reliable.
 */
#define SCB_CMD_RESP_STATUS_CURR_WR_ADDR_MASK               (0x001f0000) /* <16:20> W:R:Undefined:EZ_DATA_NR_LOG2 */
#define SCB_CMD_RESP_STATUS_CURR_WR_ADDR_POS                (16)


/*
 * Indicates whether there is an ongoing bus transfer to the IP.
 * '0': no ongoing bus transfer.
 * '1': ongoing bus transferr.
 *
 * For SPI, the field is '1' when the slave is selected.
 *
 * For I2C, the field is set to '1' at a I2C START/RESTART. In case of an
 * address match, the  field is set to '0' on a I2C STOP. In case of NO address
 * match, the field is set to '0' after the failing address match.
 */
#define SCB_CMD_RESP_STATUS_CMD_RESP_EC_BUS_BUSY            (1u << 30) /* <30:30> W:R:Undefined: */


/*
 * Indicates whether the CURR_RD_ADDR and CURR_WR_ADDR fields in this register
 * are reliable (when CMD_RESP_EC_BUSY is '0') or not reliable (when CMD_RESP_EC_BUSY
 * is '1'). Note:
 * - When there is no ongoing bus transfer, CMD_RESP_EC_BUSY is '0' (reliable).
 * - When there is a ongoing bus transfer, CMD_RESP_EC_BUSY is '0' (reliable),
 * when the CURR_RD_ADDR and CURR_WR_ADDR are not being updated by the HW.
 * - When there is a ongoing bus transfer, CMD_RESP_EC_BUSY is '1' (not reliable),
 * when the CURR_RD_ADDR or CURR_WR_ADDR are being updated by the HW.
 *    Note that this update lasts one I2C clock cycle, or two SPI clock cycles.
 */
#define SCB_CMD_RESP_STATUS_CMD_RESP_EC_BUSY                (1u << 31) /* <31:31> W:R:Undefined: */


/*
 * SPI control register.
 */
#define SCB_SPI_CTRL_ADDRESS(m)                             (0x40050020 + ((m) * (0x10000)))
#define SCB_SPI_CTRL(m)                                     (*(volatile uint32_t *)(0x40050020 + ((m) * 0x10000)))
#define SCB_SPI_CTRL_DEFAULT                                (0x03000000)

/*
 * Continuous SPI data transfers enabled ('1') or not ('0'). This field is
 * used in master mode. In slave mode, both continuous and non-continuous
 * SPI data transfers are supported independent of this field.
 *
 * When continuous transfers are enabled individual data frame transfers
 * are not necessarily seperated by slave deselection (as indicated by the
 * level or pulse on the SELECT line): if the TX FIFO has multiple data frames,
 * data frames are send out without slave deselection.
 *
 * When continuous transfers are not enabled individual data frame transfers
 * are always seperated by slave deselection: independent of the availability
 * of TX FIFO data frames, data+G65 frames are send out with slave deselection.
 */
#define SCB_SPI_CTRL_CONTINUOUS                             (1u << 0) /* <0:0> R:RW:0:SPI_M */


/*
 * Only used in SPI Texas Instruments' submode.
 *
 * When '1', the data frame start indication is a pulse on the SELECT line
 * that precedes the transfer of the first data frame bit.
 *
 * When '0', the data frame start indication is a pulse on the SELECT line
 * that coincides with the transfer of the first data frame bit.
 */
#define SCB_SPI_CTRL_SELECT_PRECEDE                         (1u << 1) /* <1:1> R:RW:0: */


/*
 * Only applicable in SPI Motorola submode. Indicates the clock phase. This
 * field, together with the CPOL field, indicates when MOSI data is driven
 * and MISO data is captured:
 * - Motorola mode 0. CPOL is ‘0’, CPHA is ‘0’: MOSI  is driven on a falling
 * edge of SCLK. MISO is captured on a rising edge of SCLK.
 * - Motorola mode 1. CPOL is ‘0’, CPHA is ‘1’: MOSI  is driven on a rising
 * edge of SCLK. MISO is captured on a falling edge of SCLK.
 * - Motorola mode 2. CPOL is ‘1’, CPHA is ‘0’: MOSI  is driven on a rising
 * edge of SCLK. MISO is captured on a falling edge of SCLK.
 * - Motorola mode 3. CPOL is ‘1’, CPHA is ‘1’: MOSI  is driven on a falling
 * edge of SCLK. MISO is captured on a rising edge of SCLK.
 */
#define SCB_SPI_CTRL_CPHA                                   (1u << 2) /* <2:2> R:RW:0: */


/*
 * Indicates the clock polarity. Only used in SPI Motorola submode. This
 * field, together with the CPHA field, indicates when MOSI data is driven
 * and MISO data is captured:
 * - CPOL is ‘0’: SCLK is ‘0’ when not transmitting data.
 * - CPOL is ‘1’: SCLK is ‘1’ when not transmitting data.
 */
#define SCB_SPI_CTRL_CPOL                                   (1u << 3) /* <3:3> R:RW:0: */


/*
 * Changes the SCLK edge on which MISO is captured. Only used in master mode.
 *
 * When '0', the default applies (for Motorola as determined by CPOL and
 * CPHA, for Texas Instruments on the falling edge of SCLK and for National
 * Semiconductors on the rising edge of SCLK).
 *
 * When '1', the alternate clock edge is used (which comes half a SPI SCLK
 * period later). Late sampling addresses the round trip delay associated
 * with transmitting SCLK from the master to the slave and transmitting MISO
 * from the slave to the master.
 */
#define SCB_SPI_CTRL_LATE_MISO_SAMPLE                       (1u << 4) /* <4:4> R:RW:0:SPI_M */


/*
 * Only applicable in master mode.
 * '0': SCLK is generated, when the SPI master is enabled and data is transmitted.
 * '1': SCLK is generated, when the SPI master is enabled. This mode is useful
 * for slave devices that use SCLK for functional operation other than just
 * SPI functionality.
 */
#define SCB_SPI_CTRL_SCLK_CONTINUOUS                        (1u << 5) /* <5:5> R:RW:0:SPI_M */


/*
 * Slave select polarity. SSEL_POLARITY0 applies to the outgoing SPI slave
 * select signal 0 (master mode) and to the incoming SPI slave select signal
 * (slave mode). For Motorola and National Semiconductors submodes:
 * '0': slave select is low/'0' active.
 * '1': slave select is high/'1' active.
 * For Texas Istruments submode:
 * '0': high/'1' active precede/coincide pulse.
 * '1': low/'0' active precede/coincide pulse.
 */
#define SCB_SPI_CTRL_SSEL_POLARITY0                         (1u << 8) /* <8:8> R:RW:0: */


/*
 * Slave select polarity. SSEL_POLARITY1 applies to the outgoing SPI slave
 * select signal 1 (master mode).
 */
#define SCB_SPI_CTRL_SSEL_POLARITY1                         (1u << 9) /* <9:9> R:RW:0:SPI_M */


/*
 * Slave select polarity. SSEL_POLARITY2 applies to the outgoing SPI slave
 * select signal 2 (master mode).
 */
#define SCB_SPI_CTRL_SSEL_POLARITY2                         (1u << 10) /* <10:10> R:RW:0:SPI_M */


/*
 * Slave select polarity. SSEL_POLARITY3 applies to the outgoing SPI slave
 * select signal 3 (master mode).
 */
#define SCB_SPI_CTRL_SSEL_POLARITY3                         (1u << 11) /* <11:11> R:RW:0:SPI_M */


/*
 * Local loopback control (does NOT affect the information on the pins).
 * Only used in master mode. Not used in National Semiconductors submode.
 * '0': the SPI master MISO line "spi_miso_in" is connected to the SPI MISO
 * pin.
 * '1': the SPI master MISO line "spi_miso_in" is connected to the SPI master
 * MOSI line "spi_mosi_out". In other words, in loopback mode the SPI master
 * receives on MISO what it transmits on MOSI.
 */
#define SCB_SPI_CTRL_LOOPBACK                               (1u << 16) /* <16:16> R:RW:0:SPI_M */


/*
 * Submode of SPI operation (3: Reserved).
 */
#define SCB_SPI_CTRL_MODE_MASK                              (0x03000000) /* <24:25> R:RW:3: */
#define SCB_SPI_CTRL_MODE_POS                               (24)

/*
 * SPI Motorola submode. In master mode, when not transmitting data (SELECT
 * is inactive), SCLK is stable at CPOL. In slave mode, when not selected,
 * SCLK is ignored; i.e. it can be either stable or clocking. In master mode,
 * when there is no data to transmit (TX FIFO is empty), SELECT is inactive.
 */
#define SCB_SPI_CTRL_MODE_MODE_SPI_MOTOROLA                 (0)

/*
 * Selects one of the four outgoing SPI slave select signals:
 * - 0: Slave 0, SPI_SELECT[0].
 * - 1: Slave 1, SPI_SELECT[1].
 * - 2: Slave 2, SPI_SELECT[2].
 * - 3: Slave 3, SPI_SELECT[3].
 * Only used in master mode. The IP should be disabled when changes are made
 * to this field.
 */
#define SCB_SPI_CTRL_SLAVE_SELECT_MASK                      (0x0c000000) /* <26:27> R:RW:0:SPI_M */
#define SCB_SPI_CTRL_SLAVE_SELECT_POS                       (26)


/*
 * Master ('1') or slave ('0') mode. In master mode, transmission will commence
 * on availability of data frames in the TX FIFO. In slave mode, when selected
 * and there is no data frame in the TX FIFO, the slave will transmit all
 * '1's. In both master and slave modes, received data frames will be lost
 * if the RX FIFO is full.
 */
#define SCB_SPI_CTRL_MASTER_MODE                            (1u << 31) /* <31:31> R:RW:0:SPI_M */


/*
 * SPI status register.
 */
#define SCB_SPI_STATUS_ADDRESS(m)                           (0x40050024 + ((m) * (0x10000)))
#define SCB_SPI_STATUS(m)                                   (*(volatile uint32_t *)(0x40050024 + ((m) * 0x10000)))
#define SCB_SPI_STATUS_DEFAULT                              (0x00000000)

/*
 * SPI bus is busy. The bus is considered busy ('1') during an ongoing transaction.
 * For Motorola and National submodes, the busy bit is '1', when the slave
 * selection (low active) is activated. For TI submode, the busy bit is '1'
 * from the time the preceding/coinciding slave select (high active) is activated
 * for the first transmitted data frame, till the last MOSI/MISO bit of the
 * last data frame is transmitted.
 */
#define SCB_SPI_STATUS_BUS_BUSY                             (1u << 0) /* <0:0> W:R:Undefined: */


/*
 * Inidicates whether the externally clocked logic is potentially accessing
 * the EZ memory and/or updating BASE_ADDR or CURR_ADDR (this is only possible
 * in EZ mode). This bit can be used by SW to determine whether BASE_ADDR
 * and CURR_ADDR are reliable.
 */
#define SCB_SPI_STATUS_SPI_EC_BUSY                          (1u << 1) /* <1:1> W:R:Undefined:SPI_S_EC */


/*
 * SPI current EZ address. Current address pointer. This field is only reliable
 * in internally clocked mode. In externally clocked mode the field may be
 * unreliable (during an ongoing transfer when SPI_EC_BUSY is '1'), as clock
 * domain synchronization is not performed in the design.
 */
#define SCB_SPI_STATUS_CURR_EZ_ADDR_MASK                    (0x0000ff00) /* <8:15> W:R:Undefined:SPI_S_EZ */
#define SCB_SPI_STATUS_CURR_EZ_ADDR_POS                     (8)


/*
 * SPI base EZ address. Address as provided by a SPI write transfer. This
 * field is only reliable in internally clocked mode. In externally clocked
 * mode the field may be unreliable, as clock domain synchronization is not
 * performed in the design.
 */
#define SCB_SPI_STATUS_BASE_EZ_ADDR_MASK                    (0x00ff0000) /* <16:23> W:R:Undefined:SPI_S_EZ */
#define SCB_SPI_STATUS_BASE_EZ_ADDR_POS                     (16)


/*
 * UART control register.
 */
#define SCB_UART_CTRL_ADDRESS(m)                            (0x40050040 + ((m) * (0x10000)))
#define SCB_UART_CTRL(m)                                    (*(volatile uint32_t *)(0x40050040 + ((m) * 0x10000)))
#define SCB_UART_CTRL_DEFAULT                               (0x03000000)

/*
 * Local loopback control (does NOT affect the information on the pins).
 * When '0', the transmitter TX line "uart_tx_out" is connected to the TX
 * pin and the receiver RX line "uart_rx_in" is connected to the RX pin.
 * When '1', the transmitter TX line "uart_tx_out" is connected to the receiver
 * RX line "uart_rx_in". A similar connections scheme is followed for "uart_rts_out"
 * and "uart_cts_in".
 *
 * This allows a SCB UART transmitter to communicate with its receiver counterpart.
 */
#define SCB_UART_CTRL_LOOPBACK                              (1u << 16) /* <16:16> R:RW:0: */


/*
 * Submode of UART operation (3: Reserved)
 */
#define SCB_UART_CTRL_MODE_MASK                             (0x03000000) /* <24:25> R:RW:3: */
#define SCB_UART_CTRL_MODE_POS                              (24)

/*
 * Standard UART submode.
 */
#define SCB_UART_CTRL_MODE_MODE_UART_STD                    (0)
/*
 * SmartCard (ISO7816) submode. Support for negative acknowledgement (NACK)
 * on the receiver side and retransmission on the transmitter side.
 */
#define SCB_UART_CTRL_MODE_MODE_UART_SMARTCARD              (1)
/*
 * Infrared Data Association (IrDA) submode. Return to Zero modulation scheme.
 * In this mode, the oversampling factor should be 16, that is OVS is 15.
 */
#define SCB_UART_CTRL_MODE_MODE_UART_IRDA                   (2)

/*
 * UART transmitter control register.
 */
#define SCB_UART_TX_CTRL_ADDRESS(m)                         (0x40050044 + ((m) * (0x10000)))
#define SCB_UART_TX_CTRL(m)                                 (*(volatile uint32_t *)(0x40050044 + ((m) * 0x10000)))
#define SCB_UART_TX_CTRL_DEFAULT                            (0x00000002)

/*
 * Stop bits. STOP_BITS + 1 is the duration of the stop period in terms of
 * halve bit periods. Valid range is [1, 7]; i.e. a stop period should last
 * at least one bit period.
 */
#define SCB_UART_TX_CTRL_STOP_BITS_MASK                     (0x00000007) /* <0:2> R:RW:2: */
#define SCB_UART_TX_CTRL_STOP_BITS_POS                      (0)


/*
 * Parity bit. When '0', the transmitter generates an even parity. When '1',
 * the transmitter generates an odd parity. Only applicable in standard UART
 * and SmartCard submodes.
 */
#define SCB_UART_TX_CTRL_PARITY                             (1u << 4) /* <4:4> R:RW:0: */


/*
 * Parity generation enabled ('1') or not ('0'). Only applicable in standard
 * UART submodes. In SmartCard submode, parity generation is always enabled
 * through hardware. In IrDA submode, parity generation is always disabled
 * through hardware
 */
#define SCB_UART_TX_CTRL_PARITY_ENABLED                     (1u << 5) /* <5:5> R:RW:0: */


/*
 * When '1', a data frame is retransmitted when a negative acknowledgement
 * is received. Only applicable to the SmartCard submode.
 */
#define SCB_UART_TX_CTRL_RETRY_ON_NACK                      (1u << 8) /* <8:8> R:RW:0: */


/*
 * UART receiver control register.
 */
#define SCB_UART_RX_CTRL_ADDRESS(m)                         (0x40050048 + ((m) * (0x10000)))
#define SCB_UART_RX_CTRL(m)                                 (*(volatile uint32_t *)(0x40050048 + ((m) * 0x10000)))
#define SCB_UART_RX_CTRL_DEFAULT                            (0x000a0002)

/*
 * Stop bits. STOP_BITS + 1 is the duration of the stop period in terms of
 * halve bit periods. Valid range is [1, 7]; i.e. a stop period should last
 * at least one bit period. If STOP_BITS is '1', stop bits error detection
 * is NOT performed. If STOP_BITS is in [2, 7], stop bits error detection
 * is performed and the associated interrupt cause INTR_RX.FRAME_ERROR is
 * set to '1' if an error is detected. In other words, the receiver supports
 * data frames with a 1 bit period stop bit sequence, but requires at least
 * 1.5 bit period stop bit sequences to detect errors. This limitation is
 * due to possible transmitter and receiver clock skew that prevents the
 * design from doing reliable stop bit detection for short (1 bit bit period)
 * stop bit sequences. Note that in case of a stop bits error, the successive
 * data frames may get lost as the receiver needs to resynchronize its start
 * bit detection. The amount of lost data frames depends on both the amount
 * of stop bits, the idle ('1') time between data frames and the data frame
 * value.
 */
#define SCB_UART_RX_CTRL_STOP_BITS_MASK                     (0x00000007) /* <0:2> R:RW:2: */
#define SCB_UART_RX_CTRL_STOP_BITS_POS                      (0)


/*
 * Parity bit. When '0', the receiver expects an even parity. When '1', the
 * receiver expects an odd parity. Only applicable in standard UART and SmartCard
 * submodes.
 */
#define SCB_UART_RX_CTRL_PARITY                             (1u << 4) /* <4:4> R:RW:0: */


/*
 * Parity checking enabled ('1') or not ('0'). Only applicable in standard
 * UART submode. In SmartCard submode, parity checking is always enabled
 * through hardware. In IrDA submode, parity checking is always disabled
 * through hardware.
 */
#define SCB_UART_RX_CTRL_PARITY_ENABLED                     (1u << 5) /* <5:5> R:RW:0: */


/*
 * Inverts incoming RX line signal "uart_rx_in". Inversion is after local
 * loopback. This functionality is intended for IrDA receiver functionality.
 */
#define SCB_UART_RX_CTRL_POLARITY                           (1u << 6) /* <6:6> R:RW:0: */


/*
 * Behaviour when a parity check fails. When '0', received data is send to
 * the RX FIFO. When '1', received data is dropped and lost. Only applicable
 * in standard UART and SmartCard submodes (negatively acknowledged SmartCard
 * data frames may be dropped with this field).
 */
#define SCB_UART_RX_CTRL_DROP_ON_PARITY_ERROR               (1u << 8) /* <8:8> R:RW:0: */


/*
 * Behaviour when an error is detected in a start or stop period. When '0',
 * received data is send to the RX FIFO. When '1', received data is dropped
 * and lost.
 */
#define SCB_UART_RX_CTRL_DROP_ON_FRAME_ERROR                (1u << 9) /* <9:9> R:RW:0: */


/*
 * Multi-processor mode. When '1', multi-processor mode is enabled. In this
 * mode, RX_CTRL.DATA_WIDTH should indicate a 9-bit data frame. In multi-processor
 * mode, the 9th received bit of a data frame seperates addresses (bit is
 * '1') from data (bit is '0'). A received address is matched with RX_MATCH.DATA
 * and RX_MATCH.MASK. In the case of a match, subsequent received data are
 * sent to the RX FIFO. In the case of NO match, subsequent received data
 * are dropped.
 */
#define SCB_UART_RX_CTRL_MP_MODE                            (1u << 10) /* <10:10> R:RW:0: */


/*
 * Only applicable in standard UART submode. When '1', the receiver performs
 * break detection and baud rate detection on the incoming data. First, break
 * detection counts the amount of bit periods that have a line value of '0'.
 * BREAK_WIDTH specifies the minum required amount of bit periods. Successful
 * break detection sets the INTR_RX.BREAK_DETECT interrupt cause to '1'.
 * Second, baud rate detection counts the amount of peripheral clock periods
 * that are use to receive the synchronization byte (0x55; least significant
 * bit first). The count is available through UART_RX_STATUS.BR_COUNTER.
 * Successful baud rate detection sets the INTR_RX.BAUD_DETECT interrupt
 * cause to '1' (BR_COUNTER is reliable). This functionality is used to synchronize/refine
 * the receiver clock to the transmitter clock. The receiver software can
 * use the BR_COUNTER value to set the right IP clock (from the programmable
 * clock IP) to guarantee successful receipt of the first LIN data frame
 * (Protected Identifier Field) after the synchronization byte.
 */
#define SCB_UART_RX_CTRL_LIN_MODE                           (1u << 12) /* <12:12> R:RW:0: */


/*
 * Only applicable in standard UART submode. When '1', the receiver skips
 * start bit detection for the first received data frame. Instead, it synchronizes
 * on the first received data frame bit, which should be a '1'. This functionality
 * is intended for wake up from DeepSleep when receiving a data frame. The
 * transition from idle ('1') to START ('0') on the RX line is used to wake
 * up the CPU. The transition detection (and the associated wake up functionality)
 * is performed by the GPIO2 IP. The woken up CPU will enable the SCB's UART
 * receiver functionality. Once enabled, it is assumed that the START bit
 * is ongoing (the CPU wakeup and SCB enable time should be less than the
 * START bit period). The SCB will synchronize to a '0' to '1' transition,
 * which indicates the first data frame bit is received (first data frame
 * bit should be '1'). After synchronization to the first data frame bit,
 * the SCB will resume normal UART functionality: subsequent data frames
 * will be synchronized on the receipt of a START bit.
 */
#define SCB_UART_RX_CTRL_SKIP_START                         (1u << 13) /* <13:13> R:RW:0: */


/*
 * Break width. BREAK_WIDTH + 1 is the minimum width in bit periods of a
 * break. During a break the transmitted/received line value is '0'. This
 * feature is useful for standard UART submode and LIN submode ("break field"
 * detection). Once, the break is detected, the INTR_RX.BREAK_DETECT bit
 * is set to '1'. Note that break detection precedes baud rate detection,
 * which is used to synchronize/refine the receiver clock to the transmitter
 * clock. As a result, break detection operates with an unsynchronized/unrefined
 * receiver clock. Therefore, the receiver's definition of a bit period is
 * imprecise and the setting of this field should take this imprecision into
 * account. The LIN standard also accounts for this imprecision: a LIN start
 * bit followed by 8 data bits allows for up to 9 consecutive '0' bit periods
 * during regular transmission, whereas the LIN break detection should be
 * at least 13 consecutive '0' bit periods. This provides for a margin of
 * 4 bit periods. Therefore, the default value of this field is set to 10,
 * representing a minimal break field with of 10+1 = 11 bit periods; a value
 * in between the 9 consecutive bit periods of a regular transmission and
 * the 13 consecutive bit periods of a break field. This provides for slight
 * imprecisions of the receiver clock wrt. the transmitter clock. There should
 * not be a need to program this field to any value other than its default
 * value.
 */
#define SCB_UART_RX_CTRL_BREAK_WIDTH_MASK                   (0x000f0000) /* <16:19> R:RW:10: */
#define SCB_UART_RX_CTRL_BREAK_WIDTH_POS                    (16)


/*
 * UART receiver status register.
 */
#define SCB_UART_RX_STATUS_ADDRESS(m)                       (0x4005004c + ((m) * (0x10000)))
#define SCB_UART_RX_STATUS(m)                               (*(volatile uint32_t *)(0x4005004c + ((m) * 0x10000)))
#define SCB_UART_RX_STATUS_DEFAULT                          (0x00000000)

/*
 * Amount of peripheral clock periods that constitute the transmission of
 * a 0x55 data frame (sent least signficant bit first) as determined by the
 * receiver. BR_COUNTER / 8 is the amount of peripheral clock periods that
 * constitute a bit period. This field has valid data when INTR_RX.BAUD_DETECT
 * is set to '1'.
 */
#define SCB_UART_RX_STATUS_BR_COUNTER_MASK                  (0x00000fff) /* <0:11> W:R:Undefined: */
#define SCB_UART_RX_STATUS_BR_COUNTER_POS                   (0)


/*
 * UART flow control register
 * UART flow control is a design time configuration parameter, which make
 * the presence of this MMIO register conditional to the configuration. The
 * "uart_rts_out" and "uart_cts_in" are always present on the IP interface.
 * If flow control is configured out, "uart_rts_out" is NOT connected, and
 * "uart_cts_in" should be connected to '0'.
 */
#define SCB_UART_FLOW_CTRL_ADDRESS(m)                       (0x40050050 + ((m) * (0x10000)))
#define SCB_UART_FLOW_CTRL(m)                               (*(volatile uint32_t *)(0x40050050 + ((m) * 0x10000)))
#define SCB_UART_FLOW_CTRL_DEFAULT                          (0x00000000)

/*
 * Trigger level. When the receiver FIFO has less entries than the amount
 * of this field, a Ready To Send (RTS) output signal "uart_rts_out" is activated.
 * By setting this field to "0", flow control is effectively SW disabled
 * (may be useful for debug purposes).
 */
#define SCB_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK               (0x0000000f) /* <0:3> R:RW:0:FF_DATA_NR_LOG2 */
#define SCB_UART_FLOW_CTRL_TRIGGER_LEVEL_POS                (0)


/*
 * Polarity of the RTS output signal "uart_rts_out":
 * '0': RTS is low/'0' active; "uart_rts_out" is '0' when active and "uart_rts_out"
 * is '1' when inactive.
 * '1': RTS is high/'1' active; "uart_rts_out" is '1' when active and "uart_rts_out"
 * is '0' when inactive.
 *
 * During IP reset (Hibernate system power mode), "uart_rts_out" is '1'.
 * This represents an inactive state assuming a low/'0' active polarity.
 */
#define SCB_UART_FLOW_CTRL_RTS_POLARITY                     (1u << 16) /* <16:16> R:RW:0: */


/*
 * Polarity of the CTS input signal "uart_cts_in":
 * '0': CTS is low/'0' active; "uart_cts_in" is '0' when active and "uart_cts_in"
 * is '1' when inactive.
 * '1': CTS is high/'1' active; "uart_cts_in" is '1' when active and "uart_cts_in"
 * is '0' when inactive.
 */
#define SCB_UART_FLOW_CTRL_CTS_POLARITY                     (1u << 24) /* <24:24> R:RW:0: */


/*
 * Enable use of CTS input signal "uart_cts_in" by the UART transmitter:
 * '0': Disabled. The UART transmitter ignores "uart_cts_in", and transmits
 * when a data frame is available for transmission in the TX FIFO or the
 * TX shift register.
 * '1': Enabled. The UART transmitter uses "uart_cts_in" to qualify the transmission
 * of data. It transmits when "uart_cts_in" is active and a data frame is
 * available for transmission in the TX FIFO or the TX shift register.
 *
 * If UART_CTRL.LOOPBACK is '1', "uart_cts_in" is connected to "uart_rts_out"
 * in the IP (both signals are subjected to signal polarity changes are indicated
 * by RTS_POLARITY and CTS_POLARITY).
 */
#define SCB_UART_FLOW_CTRL_CTS_ENABLED                      (1u << 25) /* <25:25> R:RW:0: */


/*
 * I2C control register.
 */
#define SCB_I2C_CTRL_ADDRESS(m)                             (0x40050060 + ((m) * (0x10000)))
#define SCB_I2C_CTRL(m)                                     (*(volatile uint32_t *)(0x40050060 + ((m) * 0x10000)))
#define SCB_I2C_CTRL_DEFAULT                                (0x0000fb88)

/*
 * Serial I2C interface high phase oversampling factor. HIGH_PHASE_OVS +
 * 1 peripheral clock periods constitute the high phase of a bit period.
 * The valid range is [5, 15] with input signal median filtering and [4,
 * 15] without input signal median filtering.
 *
 * The field is only used in master mode. In slave mode, the field is NOT
 * used. However, there is a frequency requirement for the IP clock wrt.
 * the regular interface (IF) high time to guarantee functional correct behavior.
 * With input signal median filtering, the IF high time should be >= 6 IP
 * clock cycles and <= 16 IP clock cycles. Without input signal median filtering,
 * the IF high time should be >= 5 IP clock cycles and <= 16 IP clock cycles.
 */
#define SCB_I2C_CTRL_HIGH_PHASE_OVS_MASK                    (0x0000000f) /* <0:3> R:RW:8:I2C_M */
#define SCB_I2C_CTRL_HIGH_PHASE_OVS_POS                     (0)


/*
 * Serial I2C interface low phase oversampling factor. LOW_PHASE_OVS + 1
 * peripheral clock periods constitute the low phase of a bit period. The
 * valid range is [7, 15] with input signal median filtering and [6, 15]
 * without input signal median filtering.
 *
 * The field is only used in master mode. In slave mode, the field is NOT
 * used. However, there is a frequency requirement for the IP clock wrt.
 * the regular (no stretching) interface (IF) low time to guarantee functional
 * correct behavior. With input signal median filtering, the IF low time
 * should be >= 8 IP clock cycles and <= 16 IP clock cycles. Without input
 * signal median filtering, the IF low time should be >= 7 IP clock cycles
 * and <= 16 IP clock cycles.
 */
#define SCB_I2C_CTRL_LOW_PHASE_OVS_MASK                     (0x000000f0) /* <4:7> R:RW:8:I2C_M */
#define SCB_I2C_CTRL_LOW_PHASE_OVS_POS                      (4)


/*
 * When '1', a received data element by the master is immediately ACK'd when
 * the receiver FIFO is not full.
 */
#define SCB_I2C_CTRL_M_READY_DATA_ACK                       (1u << 8) /* <8:8> R:RW:1:I2C_M */


/*
 * When '1', a received data element byte the master is immediately NACK'd
 * when the receiver FIFO is full. When '0', clock stretching is used instead
 * (till the receiver FIFO is no longer full).
 */
#define SCB_I2C_CTRL_M_NOT_READY_DATA_NACK                  (1u << 9) /* <9:9> R:RW:1:I2C_M */


/*
 * When '1', a received general call slave address is immediately NACK'd
 * (no ACK or clock stretching) and treated as a non matching slave address.
 * This is useful for slaves that do not need any data supplied within the
 * general call structure.
 */
#define SCB_I2C_CTRL_S_GENERAL_IGNORE                       (1u << 11) /* <11:11> R:RW:1:I2C_S */


/*
 * When '1', a received (matching) slave address is immediately ACK'd when
 * the receiver FIFO is not full. In EZ mode, this field should be set to
 * '1'.
 */
#define SCB_I2C_CTRL_S_READY_ADDR_ACK                       (1u << 12) /* <12:12> R:RW:1:I2C_S */


/*
 * When '1', a received data element by the slave is immediately ACK'd when
 * the receiver FIFO is not full. In EZ mode, this field should be set to
 * '1'.
 */
#define SCB_I2C_CTRL_S_READY_DATA_ACK                       (1u << 13) /* <13:13> R:RW:1:I2C_S */


/*
 * For internally clocked logic (EC_AM is '0' and EC_OP is '0') on an address
 * match or general call address (and S_GENERAL_IGNORE is '0'). Only used
 * when:
 * - EC_AM is '0', EC_OP is '0' and non EZ mode.
 * Functionality is as follows:
 * - 1: a received (matching) slave address is immediately NACK'd when the
 * receiver FIFO is full.
 * - 0: clock stretching is performed (till the receiver FIFO is no longer
 * full).
 *
 * For externally clocked logic (EC_AM is '1') on an address match or general
 * call address (and S_GENERAL_IGNORE is '0'). Only used when (NOT used when
 * EC_AM is '1' and EC_OP is '1' and address match and EZ mode):
 * - EC_AM is '1' and EC_OP is '0'.
 * - EC_AM is '1' and general call address match.
 * - EC_AM is '1' and non EZ mode.
 * Functionality is as follows:
 * - 1: a received (matching or general) slave address is always immediately
 * NACK'd. There are two possibilities: 1). the internally clocked logic
 * is enabled (we are in Active system power mode) and it handles the rest
 * of the current transfer. In this case the I2C master will not observe
 * the NACK. 2). the internally clocked logic is not enabled (we are in DeepSleep
 * system power mode). In this case the I2C master will observe the NACK
 * and may retry the transfer in the future (which gives the internally clocked
 * logic the time to wake up from DeepSleep system power mode).
 * - 0: clock stretching is performed (till the internally clocked logic
 * takes over). The internally clocked logic will handle the ongoing transfer
 * as soon as it is enabled.
 */
#define SCB_I2C_CTRL_S_NOT_READY_ADDR_NACK                  (1u << 14) /* <14:14> R:RW:1:I2C_S */


/*
 * For internally clocked logic only. Only used when:
 * - non EZ mode.
 * Functionality is as follows:
 * - 1: a received data element byte the slave is immediately NACK'd when
 * the receiver FIFO is full.
 * - 0: clock stretching is performed (till the receiver FIFO is no longer
 * full).
 */
#define SCB_I2C_CTRL_S_NOT_READY_DATA_NACK                  (1u << 15) /* <15:15> R:RW:1:I2C_S */


/*
 * Local loopback control (does NOT affect the information on the pins).
 * Only applicable in master/slave mode. When '0', the I2C SCL and SDA lines
 * are connected to the I2C SCL and SDA pins. When '1', I2C SCL and SDA lines
 * are routed internally in the peripheral, and as a result unaffected by
 * other I2C devices. This allows a SCB I2C peripheral to address itself.
 */
#define SCB_I2C_CTRL_LOOPBACK                               (1u << 16) /* <16:16> R:RW:0:I2C_M_S */


/*
 * Slave mode enabled ('1') or not ('0').
 */
#define SCB_I2C_CTRL_SLAVE_MODE                             (1u << 30) /* <30:30> R:RW:0:I2C_S */


/*
 * Master mode enabled ('1') or not ('0'). Note that both master and slave
 * modes can be enabled at the same time. This allows the IP to address itself.
 */
#define SCB_I2C_CTRL_MASTER_MODE                            (1u << 31) /* <31:31> R:RW:0:I2C_M */


/*
 * I2C status register.
 */
#define SCB_I2C_STATUS_ADDRESS(m)                           (0x40050064 + ((m) * (0x10000)))
#define SCB_I2C_STATUS(m)                                   (*(volatile uint32_t *)(0x40050064 + ((m) * 0x10000)))
#define SCB_I2C_STATUS_DEFAULT                              (0x00000000)

/*
 * I2C bus is busy. The bus is considered busy ('1'), from the time a START
 * is detected or from the time the SCL line is '0'. The bus is considered
 * idle ('0'), from the time a STOP is detected. If the IP is disabled, BUS_BUSY
 * is '0'. After enabling the IP, it takes time for the BUS_BUSY to detect
 * a busy bus. This time is the maximum high time of the SCL line. For a
 * 100 kHz interface frequency, this maximum high time may last roughly 5
 * us (half a bit period).
 *
 * For single master systems, BUS_BUSY does not have to be used to detect
 * an idle bus before a master starts a transfer using I2C_M_CMD.M_START
 * (no bus collisions).
 *
 * For multi-master systems, BUS_BUSY can be used to detect an idle bus before
 * a master starts a transfer using I2C_M_CMD.M_START_ON_IDLE (to prevent
 * bus collisions).
 */
#define SCB_I2C_STATUS_BUS_BUSY                             (1u << 0) /* <0:0> W:R:0: */


/*
 * Inidicates whether the externally clocked logic is potentially accessing
 * the EZ memory and/or updating BASE_ADDR or CURR_ADDR (this is only possible
 * in EZ mode). This bit can be used by SW to determine whether BASE_ADDR
 * and CURR_ADDR are reliable.
 */
#define SCB_I2C_STATUS_I2C_EC_BUSY                          (1u << 1) /* <1:1> W:R:Undefined:I2C_S_EC */


/*
 * I2C slave read transfer ('1') or I2C slave write transfer ('0'). When
 * the I2C slave is inactive/idle or receiving START, REPEATED START, STOP
 * or an address, this field is '0''.
 */
#define SCB_I2C_STATUS_S_READ                               (1u << 4) /* <4:4> W:R:0:I2C_S */


/*
 * I2C master read transfer ('1') or I2C master write transfer ('0'). When
 * the I2C master is inactive/idle or transmitting START, REPEATED START,
 * STOP or an address, this field is '0''.
 */
#define SCB_I2C_STATUS_M_READ                               (1u << 5) /* <5:5> W:R:0:I2C_M */


/*
 * I2C slave current EZ address. Current address pointer. This field is only
 * reliable in internally clocked mode. In externally clocked mode the field
 * may be unreliable (during an ongoing transfer when I2C_EC_BUSY is '1'),
 * as clock domain synchronization is not performed in the design.
 */
#define SCB_I2C_STATUS_CURR_EZ_ADDR_MASK                    (0x0000ff00) /* <8:15> W:R:Undefined:I2C_S_EZ */
#define SCB_I2C_STATUS_CURR_EZ_ADDR_POS                     (8)


/*
 * I2C slave base EZ address. Address as provided by an I2C write transfer.
 * This field is only reliable in internally clocked mode. In externally
 * clocked mode the field may be unreliable, as clock domain synchronization
 * is not performed in the design.
 */
#define SCB_I2C_STATUS_BASE_EZ_ADDR_MASK                    (0x00ff0000) /* <16:23> W:R:Undefined:I2C_S_EZ */
#define SCB_I2C_STATUS_BASE_EZ_ADDR_POS                     (16)


/*
 * I2C master command register.
 * The register fields are not retained. This is to ensure that they come
 * up as '0' after coming out of DeepSleep system power mode.
 */
#define SCB_I2C_M_CMD_ADDRESS(m)                            (0x40050068 + ((m) * (0x10000)))
#define SCB_I2C_M_CMD(m)                                    (*(volatile uint32_t *)(0x40050068 + ((m) * 0x10000)))
#define SCB_I2C_M_CMD_DEFAULT                               (0x00000000)

/*
 * When '1', transmit a START or REPEATED START. Whether a START or REPEATED
 * START is transmitted depends on the state of the master state machine.
 * A START is only transmitted when the master state machine is in the default
 * state. A REPEATED START is transmitted when the master state machine is
 * not in the default state, but is working on an ongoing transaction. The
 * REPEATED START can only be transmitted after a NACK or ACK has been received
 * for a transmitted data element or after a NACK has been transmitted for
 * a received data element. When this action is performed, the hardware sets
 * this field to '0'.
 */
#define SCB_I2C_M_CMD_M_START                               (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * When '1', transmit a START as soon as the bus is idle (I2C_STATUS.BUS_BUSY
 * is '0', note that BUSY has a default value of '0'). For bus idle detection
 * the hardware relies on STOP detection. As a result, bus idle detection
 * is only functional after at least one I2C bus transfer has been detected
 * on the bus (default/reset value of BUSY is '0') . A START is only transmitted
 * when the master state machine is in the default state. When this action
 * is performed, the hardware sets this field to '0'.
 */
#define SCB_I2C_M_CMD_M_START_ON_IDLE                       (1u << 1) /* <1:1> RW1C:RW:0: */


/*
 * When '1', attempt to transmit an acknowledgement (ACK). When this action
 * is performed, the hardware sets this field to '0'.
 */
#define SCB_I2C_M_CMD_M_ACK                                 (1u << 2) /* <2:2> RW1C:RW:0: */


/*
 * When '1', attempt to transmit a negative acknowledgement (NACK). When
 * this action is performed, the hardware sets this field to '0'.
 */
#define SCB_I2C_M_CMD_M_NACK                                (1u << 3) /* <3:3> RW1C:RW:0: */


/*
 * When '1', attempt to transmit a STOP. When this action is performed, the
 * hardware sets this field to '0'. This command has a higher priority than
 * I2C_M_CMD.M_START: in situations where both a STOP and a REPEATED START
 * could be transmitted, M_STOP takes precedence over M_START.
 */
#define SCB_I2C_M_CMD_M_STOP                                (1u << 4) /* <4:4> RW1C:RW:0: */


/*
 * I2C slave command register.
 * The register fields are not retained. This is to ensure that they come
 * up as '0' after coming out of DeepSleep system power mode.
 */
#define SCB_I2C_S_CMD_ADDRESS(m)                            (0x4005006c + ((m) * (0x10000)))
#define SCB_I2C_S_CMD(m)                                    (*(volatile uint32_t *)(0x4005006c + ((m) * 0x10000)))
#define SCB_I2C_S_CMD_DEFAULT                               (0x00000000)

/*
 * When '1', attempt to transmit an acknowledgement (ACK). When this action
 * is performed, the hardware sets this field to '0'. In EZ mode, this field
 * should be set to '0' (it is only to be used in non EZ mode).
 */
#define SCB_I2C_S_CMD_S_ACK                                 (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * When '1', attempt to transmit a negative acknowledgement (NACK). When
 * this action is performed, the hardware sets this field to '0'.  In EZ
 * mode, this field should be set to '0' (it is only to be used in non EZ
 * mode). This command has a higher priority than I2C_S_CMD.S_ACK, I2C_CTRL.S_READY_ADDR_ACK
 * or I2C_CTRL.S_READY_DATA_ACK.
 */
#define SCB_I2C_S_CMD_S_NACK                                (1u << 1) /* <1:1> RW1C:RW:0: */


/*
 * I2C configuration register.
 * The filters are used to remove glitches and to guarantee I2C compliant
 * SCL and SDA setup and hold times. The filters are trimmable.
 */
#define SCB_I2C_CFG_ADDRESS(m)                              (0x40050070 + ((m) * (0x10000)))
#define SCB_I2C_CFG(m)                                      (*(volatile uint32_t *)(0x40050070 + ((m) * 0x10000)))
#define SCB_I2C_CFG_DEFAULT                                 (0x002a1013)

/*
 * Trim bits for "i2c_sda_in" 50 ns filter. See s8i2cs BROS (001-59539) for
 * more details on the trim bit values.
 */
#define SCB_I2C_CFG_SDA_IN_FILT_TRIM_MASK                   (0x00000003) /* <0:1> R:RW:3: */
#define SCB_I2C_CFG_SDA_IN_FILT_TRIM_POS                    (0)


/*
 * Selection of "i2c_sda_in" filter delay:
 * '0': 0 ns.
 * '1: 50 ns (filter enabled).
 */
#define SCB_I2C_CFG_SDA_IN_FILT_SEL                         (1u << 4) /* <4:4> R:RW:1: */


/*
 * Trim bits for "i2c_scl_in" 50 ns filter. See s8i2cs BROS (001-59539) for
 * more details on the trim bit values.
 */
#define SCB_I2C_CFG_SCL_IN_FILT_TRIM_MASK                   (0x00000300) /* <8:9> R:RW:0: */
#define SCB_I2C_CFG_SCL_IN_FILT_TRIM_POS                    (8)


/*
 * Selection of "i2c_scl_in" filter delay:
 * '0': 0 ns.
 * '1: 50 ns (filter enabled).
 */
#define SCB_I2C_CFG_SCL_IN_FILT_SEL                         (1u << 12) /* <12:12> R:RW:1: */


/*
 * Trim bits for "i2c_sda_out" 50 ns filter 0. See s8i2cs BROS (001-59539)
 * for more details on the trim bit values.
 */
#define SCB_I2C_CFG_SDA_OUT_FILT0_TRIM_MASK                 (0x00030000) /* <16:17> R:RW:2: */
#define SCB_I2C_CFG_SDA_OUT_FILT0_TRIM_POS                  (16)


/*
 * Trim bits for "i2c_sda_out" 50 ns filter 1. See s8i2cs BROS (001-59539)
 * for more details on the trim bit values.
 */
#define SCB_I2C_CFG_SDA_OUT_FILT1_TRIM_MASK                 (0x000c0000) /* <18:19> R:RW:2: */
#define SCB_I2C_CFG_SDA_OUT_FILT1_TRIM_POS                  (18)


/*
 * Trim bits for "i2c_sda_out" 50 ns filter 2. See s8i2cs BROS (001-59539)
 * for more details on the trim bit values.
 */
#define SCB_I2C_CFG_SDA_OUT_FILT2_TRIM_MASK                 (0x00300000) /* <20:21> R:RW:2: */
#define SCB_I2C_CFG_SDA_OUT_FILT2_TRIM_POS                  (20)


/*
 * Selection of cumulative "i2c_sda_out" filter delay:
 * "0": 0 ns.
 * "1": 50 ns (filter 0 enabled).
 * "2": 100 ns (filters 0 and 1 enabled).
 * "3": 150 ns (filters 0, 1 and 2 enabled).
 */
#define SCB_I2C_CFG_SDA_OUT_FILT_SEL_MASK                   (0x30000000) /* <28:29> R:RW:0: */
#define SCB_I2C_CFG_SDA_OUT_FILT_SEL_POS                    (28)


/*
 * Transmitter control register.
 */
#define SCB_TX_CTRL_ADDRESS(m)                              (0x40050200 + ((m) * (0x10000)))
#define SCB_TX_CTRL(m)                                      (*(volatile uint32_t *)(0x40050200 + ((m) * 0x10000)))
#define SCB_TX_CTRL_DEFAULT                                 (0x00000107)

/*
 * Dataframe width. DATA_WIDTH + 1 is the amount of bits in a transmitted
 * data frame. This number does not include start, parity and stop bits.
 * For UART mode, the valid range is [3, 8]. For SPI, the valid range is
 * [3, 15]. For I2C the only valid value is 7.
 */
#define SCB_TX_CTRL_DATA_WIDTH_MASK                         (0x0000000f) /* <0:3> R:RW:7: */
#define SCB_TX_CTRL_DATA_WIDTH_POS                          (0)


/*
 * Least significant bit first ('0') or most significant bit first ('1').
 * For I2C, this field should be '1'.
 */
#define SCB_TX_CTRL_MSB_FIRST                               (1u << 8) /* <8:8> R:RW:1: */


/*
 * Transmitter FIFO control register.
 */
#define SCB_TX_FIFO_CTRL_ADDRESS(m)                         (0x40050204 + ((m) * (0x10000)))
#define SCB_TX_FIFO_CTRL(m)                                 (*(volatile uint32_t *)(0x40050204 + ((m) * 0x10000)))
#define SCB_TX_FIFO_CTRL_DEFAULT                            (0x00000000)

/*
 * Trigger level. When the transmitter FIFO has less entries than the number
 * of this field, a transmitter trigger event is generated.
 */
#define SCB_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK                 (0x0000000f) /* <0:3> R:RW:0:FF_DATA_NR_LOG2 */
#define SCB_TX_FIFO_CTRL_TRIGGER_LEVEL_POS                  (0)


/*
 * When '1', the transmitter FIFO and transmitter shift register are cleared/invalidated.
 * Invalidation will last for as long as this field is '1'. If a quick clear/invalidation
 * is required, the field should be set to '1' and be followed by a set to
 * '0'. If a clear/invalidation is required for an extended time period,
 * the field should be set to '1' during the complete time period.
 */
#define SCB_TX_FIFO_CTRL_CLEAR                              (1u << 16) /* <16:16> R:RW:0: */


/*
 * When '1', hardware reads from the transmitter FIFO do not remove FIFO
 * entries. Freeze will not advance the TX FIFO read pointer.
 */
#define SCB_TX_FIFO_CTRL_FREEZE                             (1u << 17) /* <17:17> R:RW:0: */


/*
 * Transmitter FIFO status register.
 * This register is not used in EZ and CMD_RESP modes.
 */
#define SCB_TX_FIFO_STATUS_ADDRESS(m)                       (0x40050208 + ((m) * (0x10000)))
#define SCB_TX_FIFO_STATUS(m)                               (*(volatile uint32_t *)(0x40050208 + ((m) * 0x10000)))
#define SCB_TX_FIFO_STATUS_DEFAULT                          (0x00000000)

/*
 * Amount of enties in the transmitter FIFO. The value of this field ranges
 * from 0 to FF_DATA_NR.
 */
#define SCB_TX_FIFO_STATUS_USED_MASK                        (0x0000001f) /* <0:4> W:R:0:FF_DATA_NR_LOG2_PLUS1 */
#define SCB_TX_FIFO_STATUS_USED_POS                         (0)


/*
 * Indicates whether the TX shift registers holds a valid data frame ('1')
 * or not ('0'). The shift register can be considered the top of the TX FIFO
 * (the data frame is not included in the USED field of the TX FIFO). The
 * shift register is a working register and holds the data frame that is
 * currently transmitted (when the protocol state machine is transmitting
 * a data frame) or the data frame that is tranmitted next (when the protocol
 * state machine is not transmitting a data frame).
 */
#define SCB_TX_FIFO_STATUS_SR_VALID                         (1u << 15) /* <15:15> W:R:0: */


/*
 * FIFO read pointer: FIFO location from which a data frame is read by the
 * hardware.
 */
#define SCB_TX_FIFO_STATUS_RD_PTR_MASK                      (0x000f0000) /* <16:19> W:R:0:FF_DATA_NR_LOG2 */
#define SCB_TX_FIFO_STATUS_RD_PTR_POS                       (16)


/*
 * FIFO write pointer: FIFO location at which a new data frame is written.
 */
#define SCB_TX_FIFO_STATUS_WR_PTR_MASK                      (0x0f000000) /* <24:27> W:R:0:FF_DATA_NR_LOG2 */
#define SCB_TX_FIFO_STATUS_WR_PTR_POS                       (24)


/*
 * Transmitter FIFO write register.
 * When the IP is disabled (CTRL.ENABLED is '0') or when the TX FIFO is full,
 * a write to this register is dropped. This register should only be used
 * in FIFO mode (and not in EZ or CMD_RESP modes). This register is "write
 * only"; a read from this register returns "0".
 */
#define SCB_TX_FIFO_WR_ADDRESS(m)                           (0x40050240 + ((m) * (0x10000)))
#define SCB_TX_FIFO_WR(m)                                   (*(volatile uint32_t *)(0x40050240 + ((m) * 0x10000)))
#define SCB_TX_FIFO_WR_DEFAULT                              (0x00000000)

/*
 * Data frame written into the transmitter FIFO. Behavior is similar to that
 * of a PUSH operation. Note that when CTRL.BYTE_MODE is '1', only DATA[7:0]
 * are used.
 *
 * A write to a full TX FIFO sets INTR_TX.OVERFLOW to '1'.
 */
#define SCB_TX_FIFO_WR_DATA_MASK                            (0x0000ffff) /* <0:15> R:W:0: */
#define SCB_TX_FIFO_WR_DATA_POS                             (0)


/*
 * Receiver control register.
 */
#define SCB_RX_CTRL_ADDRESS(m)                              (0x40050300 + ((m) * (0x10000)))
#define SCB_RX_CTRL(m)                                      (*(volatile uint32_t *)(0x40050300 + ((m) * 0x10000)))
#define SCB_RX_CTRL_DEFAULT                                 (0x00000107)

/*
 * Dataframe width. DATA_WIDTH + 1 is the expected amount of bits in received
 * data frame. This number does not include start, parity and stop bits.
 * For UART mode, the valid range is [3, 8]. For SPI, the valid range is
 * [3, 15]. For I2C the only valid value is 7. In EZ mode (for both SPI and
 * I2C), the only valid value is 7.
 */
#define SCB_RX_CTRL_DATA_WIDTH_MASK                         (0x0000000f) /* <0:3> R:RW:7: */
#define SCB_RX_CTRL_DATA_WIDTH_POS                          (0)


/*
 * Least significant bit first ('0') or most significant bit first ('1').
 * For I2C, this field should be '1'.
 */
#define SCB_RX_CTRL_MSB_FIRST                               (1u << 8) /* <8:8> R:RW:1: */


/*
 * Median filter. When '1', a digital 3 taps median filter is performed on
 * input interface lines. This filter should reduce the susceptability to
 * errors. However, its requires higher oversampling values. For UART IrDA
 * submode, this field should always be '1'.
 */
#define SCB_RX_CTRL_MEDIAN                                  (1u << 9) /* <9:9> R:RW:0: */


/*
 * Receiver FIFO control register.
 */
#define SCB_RX_FIFO_CTRL_ADDRESS(m)                         (0x40050304 + ((m) * (0x10000)))
#define SCB_RX_FIFO_CTRL(m)                                 (*(volatile uint32_t *)(0x40050304 + ((m) * 0x10000)))
#define SCB_RX_FIFO_CTRL_DEFAULT                            (0x00000000)

/*
 * Trigger level. When the receiver FIFO has more entries than the number
 * of this field, a receiver trigger event is generated.
 */
#define SCB_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK                 (0x0000000f) /* <0:3> R:RW:0:FF_DATA_NR_LOG2 */
#define SCB_RX_FIFO_CTRL_TRIGGER_LEVEL_POS                  (0)


/*
 * When '1', the receiver FIFO and receiver shift register are cleared/invalidated.
 * Invalidation will last for as long as this field is '1'. If a quick clear/invalidation
 * is required, the field should be set to '1' and be followed by a set to
 * '0'. If a clear/invalidation is required for an extended time period,
 * the field should be set to '1' during the complete time period.
 */
#define SCB_RX_FIFO_CTRL_CLEAR                              (1u << 16) /* <16:16> R:RW:0: */


/*
 * When '1', hardware writes to the receiver FIFO have no effect. Freeze
 * will not advance the RX FIFO write pointer.
 */
#define SCB_RX_FIFO_CTRL_FREEZE                             (1u << 17) /* <17:17> R:RW:0: */


/*
 * Receiver FIFO status register.
 * This register is not used in EZ and CMD_RESP modes.
 */
#define SCB_RX_FIFO_STATUS_ADDRESS(m)                       (0x40050308 + ((m) * (0x10000)))
#define SCB_RX_FIFO_STATUS(m)                               (*(volatile uint32_t *)(0x40050308 + ((m) * 0x10000)))
#define SCB_RX_FIFO_STATUS_DEFAULT                          (0x00000000)

/*
 * Amount of enties in the receiver FIFO. The value of this field ranges
 * from 0 to FF_DATA_NR.
 */
#define SCB_RX_FIFO_STATUS_USED_MASK                        (0x0000001f) /* <0:4> W:R:0:FF_DATA_NR_LOG2_PLUS1 */
#define SCB_RX_FIFO_STATUS_USED_POS                         (0)


/*
 * Indicates whether the RX shift registers holds a (partial) valid data
 * frame ('1') or not ('0'). The shift register can be considered the bottom
 * of the RX FIFO (the data frame is not included in the USED field of the
 * RX FIFO). The shift register is a working register and holds the data
 * frame that is currently being received (when the protocol state machine
 * is receiving a data frame).
 */
#define SCB_RX_FIFO_STATUS_SR_VALID                         (1u << 15) /* <15:15> W:R:0: */


/*
 * FIFO read pointer: FIFO location from which a data frame is read.
 */
#define SCB_RX_FIFO_STATUS_RD_PTR_MASK                      (0x000f0000) /* <16:19> W:R:0:FF_DATA_NR_LOG2 */
#define SCB_RX_FIFO_STATUS_RD_PTR_POS                       (16)


/*
 * FIFO write pointer: FIFO location at which a new data frame is written
 * by the hardware.
 */
#define SCB_RX_FIFO_STATUS_WR_PTR_MASK                      (0x0f000000) /* <24:27> W:R:0:FF_DATA_NR_LOG2 */
#define SCB_RX_FIFO_STATUS_WR_PTR_POS                       (24)


/*
 * Slave address and mask register.
 */
#define SCB_RX_MATCH_ADDRESS(m)                             (0x40050310 + ((m) * (0x10000)))
#define SCB_RX_MATCH(m)                                     (*(volatile uint32_t *)(0x40050310 + ((m) * 0x10000)))
#define SCB_RX_MATCH_DEFAULT                                (0x00000000)

/*
 * Slave device address.
 *
 * In UART multi-processor mode, all 8 bits are used.
 *
 * In I2C slave mode, only bits 7 down to 1 are used. This reflects the organization
 * of the first transmitted byte in a I2C transfer: the first 7 bits represent
 * the address of the addressed slave, and the last 1 bit is a read/write
 * indicator ('0': write, '1': read).
 */
#define SCB_RX_MATCH_ADDR_MASK                              (0x000000ff) /* <0:7> R:RW:0: */
#define SCB_RX_MATCH_ADDR_POS                               (0)


/*
 * Slave device address mask. This field is a mask that specifies which of
 * the ADDR field bits in the ADDR field take part in the matching of the
 * slave address: MATCH = ((ADDR & MASK) == ("slave address" & MASK)).
 */
#define SCB_RX_MATCH_MASK_MASK                              (0x00ff0000) /* <16:23> R:RW:0: */
#define SCB_RX_MATCH_MASK_POS                               (16)


/*
 * Receiver FIFO read register.
 * When the IP is disabled (CTRL.ENABLED is '0') or when the RX FIFO is empty,
 * a read from this register returns 0xffff:ffff. This register should only
 * be used in FIFO mode (and not in EZ or CMD_RESP modes). This register
 * is "read only"; a write to this register is ignored.
 */
#define SCB_RX_FIFO_RD_ADDRESS(m)                           (0x40050340 + ((m) * (0x10000)))
#define SCB_RX_FIFO_RD(m)                                   (*(volatile uint32_t *)(0x40050340 + ((m) * 0x10000)))
#define SCB_RX_FIFO_RD_DEFAULT                              (0x00000000)

/*
 * Data read from the receiver FIFO. Reading a data frame will remove the
 * data frame from the FIFO; i.e. behavior is similar to that of a POP operation.
 * Note that when CTRL.BYTE_MODE is '1', only DATA[7:0] are used.
 *
 * This register has a side effect when read by software: a data frame is
 * removed from the FIFO. This may be undesirable during debug; i.e. a read
 * during debug should NOT have a side effect. To this end, the IP uses the
 * AHB-Lite "hmaster[0]" input signal. When this signal is '1' in the address
 * cycle of a bus transfer, a read transfer will not have a side effect.
 * As a result, a read from this register will not remove a data frame from
 * the FIFO. As a result, a read from this register behaves as a read from
 * the SCB_RX_FIFO_RD_SILENT register.
 *
 * A read from an empty RX FIFO sets INTR_RX.UNDERFLOW to '1'.
 */
#define SCB_RX_FIFO_RD_DATA_MASK                            (0x0000ffff) /* <0:15> W:R:Undefined: */
#define SCB_RX_FIFO_RD_DATA_POS                             (0)


/*
 * Receiver FIFO read register.
 * When the IP is disabled (CTRL.ENABLED is '0') or when the RX FIFO is empty,
 * a read from this register returns 0xffff:ffff. This register should only
 * be used in FIFO mode (and not in EZ or CMD_RESP modes).
 */
#define SCB_RX_FIFO_RD_SILENT_ADDRESS(m)                    (0x40050344 + ((m) * (0x10000)))
#define SCB_RX_FIFO_RD_SILENT(m)                            (*(volatile uint32_t *)(0x40050344 + ((m) * 0x10000)))
#define SCB_RX_FIFO_RD_SILENT_DEFAULT                       (0x00000000)

/*
 * Data read from the receiver FIFO. Reading a data frame will NOT remove
 * the data frame from the FIFO; i.e. behavior is similar to that of a PEEK
 * operation. Note that when CTRL.BYTE_MODE is '1', only DATA[7:0] are used.
 *
 * A read from an empty RX FIFO sets INTR_RX.UNDERFLOW to '1'.
 */
#define SCB_RX_FIFO_RD_SILENT_DATA_MASK                     (0x0000ffff) /* <0:15> W:R:Undefined: */
#define SCB_RX_FIFO_RD_SILENT_DATA_POS                      (0)


/*
 * Memory buffer registers.
 * When the IP is disabled (CTRL.ENABLED is '0'), a read from these registers
 * return 0xffff:ffff. It is under MMIO register control whether accesses
 * to this register should introduce bus wait states or be discarded when
 * the externally clocked logic is accessing the memory structure. These
 * registers should only be used in EZ and CMD_RESP modes (and not in FIFO
 * mode).
 */
#define SCB_EZ_DATA_ADDRESS(m,n)                            (0x40050400 + ((m) * (0x10000)) + ((n) * (0x0004)))
#define SCB_EZ_DATA(m)                                      (*(volatile uint32_t *)(0x40050400 + ((m) * (0x10000)) + ((n) * 0x0004)))
#define SCB_EZ_DATA_DEFAULT                                 (0x00000000)

/*
 * Data in buffer memory location. In case of a blocked discarded access,
 * a read access returns 0xffff:ffff and a write access is dropped. Note
 * that the 0xffff:ffff value is unique (not a legal EZ_DATA byte value)
 * and can be detected by SW. Note that a discarded write access can be detected
 * by reading back the written value.
 */
#define SCB_EZ_DATA_EZ_DATA_MASK                            (0x000000ff) /* <0:7> RW:RW:Undefined: */
#define SCB_EZ_DATA_EZ_DATA_POS                             (0)


/*
 * Active clocked interrupt signal register
 * Enables software to determine the source of the combined interrupt output
 * signals "interrupt_ic", "interrupt_ec" and "interrupt".
 */
#define SCB_INTR_CAUSE_ADDRESS(m)                           (0x40050e00 + ((m) * (0x10000)))
#define SCB_INTR_CAUSE(m)                                   (*(volatile uint32_t *)(0x40050e00 + ((m) * 0x10000)))
#define SCB_INTR_CAUSE_DEFAULT                              (0x00000000)

/*
 * Master interrupt active ("interrupt_master"): INTR_M_MASKED != 0.
 */
#define SCB_INTR_CAUSE_M                                    (1u << 0) /* <0:0> W:R:0: */


/*
 * Slave interrupt active ("interrupt_slave"): INTR_S_MASKED != 0.
 */
#define SCB_INTR_CAUSE_S                                    (1u << 1) /* <1:1> W:R:0: */


/*
 * Transmitter interrupt active ("interrupt_tx"): INTR_TX_MASKED != 0.
 */
#define SCB_INTR_CAUSE_TX                                   (1u << 2) /* <2:2> W:R:0: */


/*
 * Receiver interrupt active ("interrupt_rx"): INTR_RX_MASKED != 0.
 */
#define SCB_INTR_CAUSE_RX                                   (1u << 3) /* <3:3> W:R:0: */


/*
 * Externally clock I2C interrupt active ("interrupt_i2c_ec"): INTR_I2C_EC_MASKED
 * != 0.
 */
#define SCB_INTR_CAUSE_I2C_EC                               (1u << 4) /* <4:4> W:R:0:I2C_S_EC */


/*
 * Externally clocked SPI interrupt active ("interrupt_spi_ec"): INTR_SPI_EC_MASKED
 * != 0.
 */
#define SCB_INTR_CAUSE_SPI_EC                               (1u << 5) /* <5:5> W:R:0:SPI_S_EC */


/*
 * Externally clocked I2C interrupt request register
 * The fields in this register are set by HW and are cleared by software
 * by writing a '1'.  These interrupt causes are generated by externally
 * clocked logic. HW clears the interrupt causes to '0', when the IP is disabled.
 */
#define SCB_INTR_I2C_EC_ADDRESS(m)                          (0x40050e80 + ((m) * (0x10000)))
#define SCB_INTR_I2C_EC(m)                                  (*(volatile uint32_t *)(0x40050e80 + ((m) * 0x10000)))
#define SCB_INTR_I2C_EC_DEFAULT                             (0x00000000)

/*
 * Wake up request. Active on incoming slave request (with address match).
 *
 * Only used when EC_AM is '1'.
 */
#define SCB_INTR_I2C_EC_WAKE_UP                             (1u << 0) /* <0:0> A:RW1C:0: */


/*
 * STOP detection. Activated on the end of a every transfer (I2C STOP).
 *
 * Only available for a slave request with an address match, in EZ and CMD_RESP
 * modes, when EC_OP is '1'.
 */
#define SCB_INTR_I2C_EC_EZ_STOP                             (1u << 1) /* <1:1> A:RW1C:0:EZ_CMD_RESP */


/*
 * STOP detection after a write transfer occurred. Activated on the end of
 * a write transfer (I2C STOP). This event is an indication that a buffer
 * memory location has been written to. For EZ mode: a  transfer that only
 * writes the base address does NOT activate this event.
 *
 * Only available for a slave request with an address match, in EZ and CMD_RESP
 * modes, when EC_OP is '1'.
 */
#define SCB_INTR_I2C_EC_EZ_WRITE_STOP                       (1u << 2) /* <2:2> A:RW1C:0:EZ_CMD_RESP */


/*
 * STOP detection after a read transfer occurred. Activated on the end of
 * a read transfer (I2C STOP). This event is an indication that a buffer
 * memory location has been read from.
 *
 * Only available for a slave request with an address match, in EZ and CMD_RESP
 * modes, when EC_OP is '1'.
 */
#define SCB_INTR_I2C_EC_EZ_READ_STOP                        (1u << 3) /* <3:3> A:RW1C:0:EZ_CMD_RESP */


/*
 * Externally clocked I2C interrupt mask register
 */
#define SCB_INTR_I2C_EC_MASK_ADDRESS(m)                     (0x40050e88 + ((m) * (0x10000)))
#define SCB_INTR_I2C_EC_MASK(m)                             (*(volatile uint32_t *)(0x40050e88 + ((m) * 0x10000)))
#define SCB_INTR_I2C_EC_MASK_DEFAULT                        (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_I2C_EC_MASK_WAKE_UP                        (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_I2C_EC_MASK_EZ_STOP                        (1u << 1) /* <1:1> R:RW:0:EZ_CMD_RESP */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_I2C_EC_MASK_EZ_WRITE_STOP                  (1u << 2) /* <2:2> R:RW:0:EZ_CMD_RESP */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_I2C_EC_MASK_EZ_READ_STOP                   (1u << 3) /* <3:3> R:RW:0:EZ_CMD_RESP */


/*
 * Externally clocked I2C interrupt masked register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers. This register allows SW to read the status
 * of all mask enabled interrupt causes with a single load operation, rather
 * than two load operations: one for the interrupt causes and one for the
 * masks. This simplifies Firmware development. The associated interrupt
 * is active ('1'), when INTR_I2C_EC_MASKED != 0.
 */
#define SCB_INTR_I2C_EC_MASKED_ADDRESS(m)                   (0x40050e8c + ((m) * (0x10000)))
#define SCB_INTR_I2C_EC_MASKED(m)                           (*(volatile uint32_t *)(0x40050e8c + ((m) * 0x10000)))
#define SCB_INTR_I2C_EC_MASKED_DEFAULT                      (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_I2C_EC_MASKED_WAKE_UP                      (1u << 0) /* <0:0> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_I2C_EC_MASKED_EZ_STOP                      (1u << 1) /* <1:1> W:R:0:EZ_CMD_RESP */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_I2C_EC_MASKED_EZ_WRITE_STOP                (1u << 2) /* <2:2> W:R:0:EZ_CMD_RESP */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_I2C_EC_MASKED_EZ_READ_STOP                 (1u << 3) /* <3:3> W:R:0:EZ_CMD_RESP */


/*
 * Externally clocked SPI interrupt request register
 * The fields in this register are set by HW and are cleared by software
 * by writing a '1'. These interrupt causes are generated by externally clocked
 * logic. HW clears the interrupt causes to '0', when the IP is disabled.
 */
#define SCB_INTR_SPI_EC_ADDRESS(m)                          (0x40050ec0 + ((m) * (0x10000)))
#define SCB_INTR_SPI_EC(m)                                  (*(volatile uint32_t *)(0x40050ec0 + ((m) * 0x10000)))
#define SCB_INTR_SPI_EC_DEFAULT                             (0x00000000)

/*
 * Wake up request. Active on incoming slave request when externally clocked
 * selection is '1'.
 *
 * Only used when EC_AM is '1'.
 */
#define SCB_INTR_SPI_EC_WAKE_UP                             (1u << 0) /* <0:0> A:RW1C:0: */


/*
 * STOP detection. Activated on the end of a every transfer (SPI deselection).
 *
 * Only available in EZ and CMD_RESP mode and when EC_OP is '1'.
 */
#define SCB_INTR_SPI_EC_EZ_STOP                             (1u << 1) /* <1:1> A:RW1C:0:EZ_CMD_RESP */


/*
 * STOP detection after a write transfer occurred. Activated on the end of
 * a write transfer (SPI deselection). This event is an indication that a
 * buffer memory location has been written to. For EZ mode: a  transfer that
 * only writes the base address does NOT activate this event.
 *
 * Only used in EZ and CMD_RESP modes and when EC_OP is '1'.
 */
#define SCB_INTR_SPI_EC_EZ_WRITE_STOP                       (1u << 2) /* <2:2> A:RW1C:0:EZ_CMD_RESP */


/*
 * STOP detection after a read transfer occurred. Activated on the end of
 * a read transfer (SPI deselection). This event is an indication that a
 * buffer memory location has been read from.
 *
 * Only used in EZ and CMD_RESP modes and when EC_OP is '1'.
 */
#define SCB_INTR_SPI_EC_EZ_READ_STOP                        (1u << 3) /* <3:3> A:RW1C:0:EZ_CMD_RESP */


/*
 * Externally clocked SPI interrupt mask register
 */
#define SCB_INTR_SPI_EC_MASK_ADDRESS(m)                     (0x40050ec8 + ((m) * (0x10000)))
#define SCB_INTR_SPI_EC_MASK(m)                             (*(volatile uint32_t *)(0x40050ec8 + ((m) * 0x10000)))
#define SCB_INTR_SPI_EC_MASK_DEFAULT                        (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_SPI_EC_MASK_WAKE_UP                        (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_SPI_EC_MASK_EZ_STOP                        (1u << 1) /* <1:1> R:RW:0:EZ_CMD_RESP */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_SPI_EC_MASK_EZ_WRITE_STOP                  (1u << 2) /* <2:2> R:RW:0:EZ_CMD_RESP */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_SPI_EC_MASK_EZ_READ_STOP                   (1u << 3) /* <3:3> R:RW:0:EZ_CMD_RESP */


/*
 * Externally clocked SPI interrupt masked register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers. This register allows SW to read the status
 * of all mask enabled interrupt causes with a single load operation, rather
 * than two load operations: one for the interrupt causes and one for the
 * masks. This simplifies Firmware development. The associated interrupt
 * is active ('1'), when INTR_SPI_EC_MASKED != 0.
 */
#define SCB_INTR_SPI_EC_MASKED_ADDRESS(m)                   (0x40050ecc + ((m) * (0x10000)))
#define SCB_INTR_SPI_EC_MASKED(m)                           (*(volatile uint32_t *)(0x40050ecc + ((m) * 0x10000)))
#define SCB_INTR_SPI_EC_MASKED_DEFAULT                      (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_SPI_EC_MASKED_WAKE_UP                      (1u << 0) /* <0:0> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_SPI_EC_MASKED_EZ_STOP                      (1u << 1) /* <1:1> W:R:0:EZ_CMD_RESP */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_SPI_EC_MASKED_EZ_WRITE_STOP                (1u << 2) /* <2:2> W:R:0:EZ_CMD_RESP */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_SPI_EC_MASKED_EZ_READ_STOP                 (1u << 3) /* <3:3> W:R:0:EZ_CMD_RESP */


/*
 * Master interrupt request register.
 * The register fields are not retained In DeepSleep power mode: HW clears
 * the interrupt causes to '0', when coming out of DeepSleep power mode.
 * In addition, HW clears the interrupt causes to '0', when the IP is disabled.
 * As a result, the interrupt causes are only available in Active/Sleep power
 * modes; they are generated by internally clocked logic (this logic operates
 * on a clock that is only available in Active/Sleep power modes).
 *
 * The interrupt causes should only be used for internally clocked operation;
 * i.e. EC_OP is '0'.
 */
#define SCB_INTR_M_ADDRESS(m)                               (0x40050f00 + ((m) * (0x10000)))
#define SCB_INTR_M(m)                                       (*(volatile uint32_t *)(0x40050f00 + ((m) * 0x10000)))
#define SCB_INTR_M_DEFAULT                                  (0x00000000)

/*
 * I2C master lost arbitration: the value driven by the master on the SDA
 * line is not the same as the value observed on the SDA line.
 */
#define SCB_INTR_M_I2C_ARB_LOST                             (1u << 0) /* <0:0> RW1S:RW1C:0:I2C_M */


/*
 * I2C master negative acknowledgement. Set to '1', when the master receives
 * a NACK (typically after the master transmitted the slave address or TX
 * data).
 */
#define SCB_INTR_M_I2C_NACK                                 (1u << 1) /* <1:1> RW1S:RW1C:0:I2C_M */


/*
 * I2C master acknowledgement. Set to '1', when the master receives a ACK
 * (typically after the master transmitted the slave address or TX data).
 */
#define SCB_INTR_M_I2C_ACK                                  (1u << 2) /* <2:2> RW1S:RW1C:0:I2C_M */


/*
 * I2C master STOP. Set to '1', when the master has transmitted a STOP.
 */
#define SCB_INTR_M_I2C_STOP                                 (1u << 4) /* <4:4> RW1S:RW1C:0:I2C_M */


/*
 * I2C master bus error (unexpected detection of START or STOP condition).
 */
#define SCB_INTR_M_I2C_BUS_ERROR                            (1u << 8) /* <8:8> RW1S:RW1C:0:I2C_M */


/*
 * SPI master transfer done event: all data frames in the transmit FIFO are
 * sent and the transmit FIFO is empty.
 */
#define SCB_INTR_M_SPI_DONE                                 (1u << 9) /* <9:9> RW1S:RW1C:0:SPI_M */


/*
 * Master interrupt set request register
 * When read, this register reflects the interrupt request register.
 */
#define SCB_INTR_M_SET_ADDRESS(m)                           (0x40050f04 + ((m) * (0x10000)))
#define SCB_INTR_M_SET(m)                                   (*(volatile uint32_t *)(0x40050f04 + ((m) * 0x10000)))
#define SCB_INTR_M_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_SET_I2C_ARB_LOST                         (1u << 0) /* <0:0> A:RW1S:0:I2C_M */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_SET_I2C_NACK                             (1u << 1) /* <1:1> A:RW1S:0:I2C_M */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_SET_I2C_ACK                              (1u << 2) /* <2:2> A:RW1S:0:I2C_M */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_SET_I2C_STOP                             (1u << 4) /* <4:4> A:RW1S:0:I2C_M */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_SET_I2C_BUS_ERROR                        (1u << 8) /* <8:8> A:RW1S:0:I2C_M */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_SET_SPI_DONE                             (1u << 9) /* <9:9> A:RW1S:0:SPI_M */


/*
 * Master interrupt mask register.
 */
#define SCB_INTR_M_MASK_ADDRESS(m)                          (0x40050f08 + ((m) * (0x10000)))
#define SCB_INTR_M_MASK(m)                                  (*(volatile uint32_t *)(0x40050f08 + ((m) * 0x10000)))
#define SCB_INTR_M_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_MASK_I2C_ARB_LOST                        (1u << 0) /* <0:0> R:RW:0:I2C_M */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_MASK_I2C_NACK                            (1u << 1) /* <1:1> R:RW:0:I2C_M */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_MASK_I2C_ACK                             (1u << 2) /* <2:2> R:RW:0:I2C_M */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_MASK_I2C_STOP                            (1u << 4) /* <4:4> R:RW:0:I2C_M */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_MASK_I2C_BUS_ERROR                       (1u << 8) /* <8:8> R:RW:0:I2C_M */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_M_MASK_SPI_DONE                            (1u << 9) /* <9:9> R:RW:0:SPI_M */


/*
 * Master interrupt masked request register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers. This register allows SW to read the status
 * of all mask enabled interrupt causes with a single load operation, rather
 * than two load operations: one for the interrupt causes and one for the
 * masks. This simplifies Firmware development. The associated interrupt
 * is active ('1'), when INTR_M_MASKED != 0.
 */
#define SCB_INTR_M_MASKED_ADDRESS(m)                        (0x40050f0c + ((m) * (0x10000)))
#define SCB_INTR_M_MASKED(m)                                (*(volatile uint32_t *)(0x40050f0c + ((m) * 0x10000)))
#define SCB_INTR_M_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_M_MASKED_I2C_ARB_LOST                      (1u << 0) /* <0:0> W:R:0:I2C_M */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_M_MASKED_I2C_NACK                          (1u << 1) /* <1:1> W:R:0:I2C_M */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_M_MASKED_I2C_ACK                           (1u << 2) /* <2:2> W:R:0:I2C_M */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_M_MASKED_I2C_STOP                          (1u << 4) /* <4:4> W:R:0:I2C_M */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_M_MASKED_I2C_BUS_ERROR                     (1u << 8) /* <8:8> W:R:0:I2C_M */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_M_MASKED_SPI_DONE                          (1u << 9) /* <9:9> W:R:0:SPI_M */


/*
 * Slave interrupt request register.
 * The register fields are not retained In DeepSleep power mode: HW clears
 * the interrupt causes to '0', when coming out of DeepSleep power mode.
 * In addition, HW clears the interrupt causes to '0', when the IP is disabled.
 * As a result, the interrupt causes are only available in Active/Sleep power
 * modes; they are generated by internally clocked logic (this logic operates
 * on a clock that is only available in Active/Sleep power modes).
 *
 * The interrupt causes should only be used for internally clocked operation;
 * i.e. EC_OP is '0'.
 */
#define SCB_INTR_S_ADDRESS(m)                               (0x40050f40 + ((m) * (0x10000)))
#define SCB_INTR_S(m)                                       (*(volatile uint32_t *)(0x40050f40 + ((m) * 0x10000)))
#define SCB_INTR_S_DEFAULT                                  (0x00000000)

/*
 * I2C slave lost arbitration: the value driven on the SDA line is not the
 * same as the value observed on the SDA line (while the SCL line is '1').
 * This should not occur, it represents erroneous I2C bus behavior. In case
 * of lost arbitration, the I2C slave state machine abort the ongoing transfer.
 * The Firmware may decide to clear the TX and RX FIFOs in case of this error.
 */
#define SCB_INTR_S_I2C_ARB_LOST                             (1u << 0) /* <0:0> RW1S:RW1C:0:I2C_S */


/*
 * I2C slave negative acknowledgement received. Set to '1', when the slave
 * receives a NACK (typically after the slave transmitted TX data).
 */
#define SCB_INTR_S_I2C_NACK                                 (1u << 1) /* <1:1> RW1S:RW1C:0:I2C_S */


/*
 * I2C slave acknowledgement received. Set to '1', when the slave receives
 * a ACK (typically after the slave transmitted TX data).
 */
#define SCB_INTR_S_I2C_ACK                                  (1u << 2) /* <2:2> RW1S:RW1C:0:I2C_S */


/*
 * I2C STOP event for I2C write transfer intended for this slave (address
 * matching is performed). Set to '1', when STOP or REPEATED START event
 * is detected. The REPEATED START event is included in this interrupt cause
 * such that the I2C transfers separated by a REPEATED START can be distinguished
 * and potentially treated separately by the Firmware. Note that the second
 * I2C transfer (after a REPEATED START) may be to a different slave address.
 *
 * In non EZ mode, the event is detected on any I2C write transfer intended
 * for this slave. Note that a I2C write address intended for the slave (address
 * is matching and a it is a write transfer) will result in a I2C_WRITE_STOP
 * event independent of whether the I2C address is ACK'd or NACK'd.
 *
 * In EZ mode, the event is detected only on I2C write transfers that have
 * EZ data written to the memory structure (an I2C write transfer that only
 * communicates an I2C address and EZ address, will not result in this event
 * being detected).
 */
#define SCB_INTR_S_I2C_WRITE_STOP                           (1u << 3) /* <3:3> RW1S:RW1C:0:I2C_S */


/*
 * I2C STOP event for I2C (read or write) transfer intended for this slave
 * (address matching is performed). Set to '1', when STOP or REPEATED START
 * event is detected. The REPEATED START event is included in this interrupt
 * cause such that the I2C transfers separated by a REPEATED START can be
 * distinguished and potentially treated separately by the Firmware. Note
 * that the second I2C transfer (after a REPEATED START) may be to a different
 * slave address.
 *
 * The event is detected on any I2C transfer intended for this slave. Note
 * that a I2C address intended for the slave (address is matching) will result
 * in a I2C_STOP event independent of whether the I2C address is ACK'd or
 * NACK'd.
 */
#define SCB_INTR_S_I2C_STOP                                 (1u << 4) /* <4:4> RW1S:RW1C:0:I2C_S */


/*
 * I2C slave START received. Set to '1', when START or REPEATED START event
 * is detected.
 *
 * In the case of externally clocked address matching (CTRL.EC_AM_MODE is
 * '1') AND clock stretching is performed (till the internally clocked logic
 * takes over) (I2C_CTRL.S_NOT_READY_ADDR_NACK is '0'), this field is NOT
 * set. The Firmware should use INTR_S_EC.WAKE_UP, INTR_S.I2C_ADDR_MATCH
 * and INTR_S.I2C_GENERAL.
 */
#define SCB_INTR_S_I2C_START                                (1u << 5) /* <5:5> RW1S:RW1C:0:I2C_S */


/*
 * I2C slave matching address received. If CTRL.ADDR_ACCEPT, the received
 * address (including the R/W bit) is available in the RX FIFO. In the case
 * of externally clocked address matching (CTRL.EC_AM_MODE is '1') and internally
 * clocked operation (CTRL.EC_OP_MODE is '0'), this field is set when the
 * event is detected.
 */
#define SCB_INTR_S_I2C_ADDR_MATCH                           (1u << 6) /* <6:6> RW1S:RW1C:0:I2C_S */


/*
 * I2C slave general call address received.  If CTRL.ADDR_ACCEPT, the received
 * address 0x00 (including the R/W bit) is available in the RX FIFO.   In
 * the case of externally clocked address matching (CTRL.EC_AM_MODE is '1')
 * and internally clocked operation (CTRL.EC_OP_MODE is '0'), this field
 * is set when the event is detected.
 */
#define SCB_INTR_S_I2C_GENERAL                              (1u << 7) /* <7:7> RW1S:RW1C:0:I2C_S */


/*
 * I2C slave bus error (unexpected detection of START or STOP condition).
 * This should not occur, it represents erroneous I2C bus behavior. In case
 * of a bus error, the I2C slave state machine abort the ongoing transfer.
 * The Firmware may decide to clear the TX and RX FIFOs in case of this error.
 */
#define SCB_INTR_S_I2C_BUS_ERROR                            (1u << 8) /* <8:8> RW1S:RW1C:0:I2C_S */


/*
 * SPI slave deselected after a write EZ SPI transfer occurred.
 */
#define SCB_INTR_S_SPI_EZ_WRITE_STOP                        (1u << 9) /* <9:9> RW1S:RW1C:0:SPI_S */


/*
 * SPI slave deselected after any EZ SPI transfer occurred.
 */
#define SCB_INTR_S_SPI_EZ_STOP                              (1u << 10) /* <10:10> RW1S:RW1C:0:SPI_S */


/*
 * SPI slave deselected at an unexpected time in the SPI transfer. The Firmware
 * may decide to clear the TX and RX FIFOs in case of this error.
 */
#define SCB_INTR_S_SPI_BUS_ERROR                            (1u << 11) /* <11:11> RW1S:RW1C:0:SPI_S */


/*
 * Slave interrupt set request register.
 * When read, this register reflects the interrupt request register.
 */
#define SCB_INTR_S_SET_ADDRESS(m)                           (0x40050f44 + ((m) * (0x10000)))
#define SCB_INTR_S_SET(m)                                   (*(volatile uint32_t *)(0x40050f44 + ((m) * 0x10000)))
#define SCB_INTR_S_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_ARB_LOST                         (1u << 0) /* <0:0> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_NACK                             (1u << 1) /* <1:1> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_ACK                              (1u << 2) /* <2:2> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_WRITE_STOP                       (1u << 3) /* <3:3> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_STOP                             (1u << 4) /* <4:4> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_START                            (1u << 5) /* <5:5> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_ADDR_MATCH                       (1u << 6) /* <6:6> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_GENERAL                          (1u << 7) /* <7:7> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_I2C_BUS_ERROR                        (1u << 8) /* <8:8> A:RW1S:0:I2C_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_SPI_EZ_WRITE_STOP                    (1u << 9) /* <9:9> A:RW1S:0:SPI_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_SPI_EZ_STOP                          (1u << 10) /* <10:10> A:RW1S:0:SPI_S */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_SET_SPI_BUS_ERROR                        (1u << 11) /* <11:11> A:RW1S:0:SPI_S */


/*
 * Slave interrupt mask register.
 */
#define SCB_INTR_S_MASK_ADDRESS(m)                          (0x40050f48 + ((m) * (0x10000)))
#define SCB_INTR_S_MASK(m)                                  (*(volatile uint32_t *)(0x40050f48 + ((m) * 0x10000)))
#define SCB_INTR_S_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_ARB_LOST                        (1u << 0) /* <0:0> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_NACK                            (1u << 1) /* <1:1> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_ACK                             (1u << 2) /* <2:2> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_WRITE_STOP                      (1u << 3) /* <3:3> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_STOP                            (1u << 4) /* <4:4> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_START                           (1u << 5) /* <5:5> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_ADDR_MATCH                      (1u << 6) /* <6:6> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_GENERAL                         (1u << 7) /* <7:7> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_I2C_BUS_ERROR                       (1u << 8) /* <8:8> R:RW:0:I2C_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_SPI_EZ_WRITE_STOP                   (1u << 9) /* <9:9> R:RW:0:SPI_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_SPI_EZ_STOP                         (1u << 10) /* <10:10> R:RW:0:SPI_S */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_S_MASK_SPI_BUS_ERROR                       (1u << 11) /* <11:11> R:RW:0:SPI_S */


/*
 * Slave interrupt masked request register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers. This register allows SW to read the status
 * of all mask enabled interrupt causes with a single load operation, rather
 * than two load operations: one for the interrupt causes and one for the
 * masks. This simplifies Firmware development. The associated interrupt
 * is active ('1'), when INTR_S_MASKED != 0.
 */
#define SCB_INTR_S_MASKED_ADDRESS(m)                        (0x40050f4c + ((m) * (0x10000)))
#define SCB_INTR_S_MASKED(m)                                (*(volatile uint32_t *)(0x40050f4c + ((m) * 0x10000)))
#define SCB_INTR_S_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_ARB_LOST                      (1u << 0) /* <0:0> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_NACK                          (1u << 1) /* <1:1> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_ACK                           (1u << 2) /* <2:2> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_WRITE_STOP                    (1u << 3) /* <3:3> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_STOP                          (1u << 4) /* <4:4> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_START                         (1u << 5) /* <5:5> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_ADDR_MATCH                    (1u << 6) /* <6:6> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_GENERAL                       (1u << 7) /* <7:7> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_I2C_BUS_ERROR                     (1u << 8) /* <8:8> W:R:0:I2C_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_SPI_EZ_WRITE_STOP                 (1u << 9) /* <9:9> W:R:0:SPI_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_SPI_EZ_STOP                       (1u << 10) /* <10:10> W:R:0:SPI_S */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_S_MASKED_SPI_BUS_ERROR                     (1u << 11) /* <11:11> W:R:0:SPI_S */


/*
 * Transmitter interrupt request register.
 * The register fields are not retained In DeepSleep power mode: HW clears
 * the interrupt causes to '0', when coming out of DeepSleep power mode.
 * In addition, HW clears the interrupt causes to '0', when the IP is disabled.
 * As a result, the interrupt causes are only available in Active/Sleep power
 * modes; they are generated by internally clocked logic (this logic operates
 * on a clock that is only available in Active/Sleep power modes).
 */
#define SCB_INTR_TX_ADDRESS(m)                              (0x40050f80 + ((m) * (0x10000)))
#define SCB_INTR_TX(m)                                      (*(volatile uint32_t *)(0x40050f80 + ((m) * 0x10000)))
#define SCB_INTR_TX_DEFAULT                                 (0x00000000)

/*
 * Less entries in the TX FIFO than the value specified by TX_FIFO_CTRL.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_TX_TRIGGER                                 (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * TX FIFO is not full. Dependent on CTRL.BYTE_MODE:
 * BYTE_MODE is '0': # entries != FF_DATA_NR/2.
 * BYTE_MODE is '1': # entries != FF_DATA_NR.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_TX_NOT_FULL                                (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * TX FIFO is empty; i.e. it has 0 entries.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_TX_EMPTY                                   (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * Attempt to write to a full TX FIFO.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_TX_OVERFLOW                                (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * Attempt to read from an empty TX FIFO. This happens when the IP is ready
 * to transfer data and EMPTY is '1'.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_TX_UNDERFLOW                               (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * AHB-Lite write transfer can not get access to the EZ memory (EZ data access),
 * due to an externally clocked EZ access. This may happen when STATUS.EC_BUSY
 * is '1'.
 */
#define SCB_INTR_TX_BLOCKED                                 (1u << 7) /* <7:7> RW1S:RW1C:0:EC */


/*
 * UART transmitter received a negative acknowledgement in SmartCard mode.
 * Set to '1', when event is detected. Write with '1' to clear bit.
 */
#define SCB_INTR_TX_UART_NACK                               (1u << 8) /* <8:8> RW1S:RW1C:0:UART */


/*
 * UART transmitter done event. This happens when the IP is done transferring
 * all data in the TX FIFO; i.e. EMPTY is '1'. Set to '1', when event is
 * detected. Write with '1' to clear bit.
 */
#define SCB_INTR_TX_UART_DONE                               (1u << 9) /* <9:9> RW1S:RW1C:0:UART */


/*
 * UART lost arbitration: the value driven on the TX line is not the same
 * as the value observed on the RX line. This condition event is usefull
 * when transmitter and receiver share a TX/RX line. This is the case in
 * LIN or SmartCard modes. Set to '1', when event is detected. Write with
 * '1' to clear bit.
 */
#define SCB_INTR_TX_UART_ARB_LOST                           (1u << 10) /* <10:10> RW1S:RW1C:0:UART */


/*
 * Transmitter interrupt set request register
 * When read, this register reflects the interrupt request register.
 */
#define SCB_INTR_TX_SET_ADDRESS(m)                          (0x40050f84 + ((m) * (0x10000)))
#define SCB_INTR_TX_SET(m)                                  (*(volatile uint32_t *)(0x40050f84 + ((m) * 0x10000)))
#define SCB_INTR_TX_SET_DEFAULT                             (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_TRIGGER                             (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_NOT_FULL                            (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_EMPTY                               (1u << 4) /* <4:4> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_OVERFLOW                            (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_UNDERFLOW                           (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_BLOCKED                             (1u << 7) /* <7:7> A:RW1S:0:EC */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_UART_NACK                           (1u << 8) /* <8:8> A:RW1S:0:UART */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_UART_DONE                           (1u << 9) /* <9:9> A:RW1S:0:UART */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_SET_UART_ARB_LOST                       (1u << 10) /* <10:10> A:RW1S:0:UART */


/*
 * Transmitter interrupt mask register.
 */
#define SCB_INTR_TX_MASK_ADDRESS(m)                         (0x40050f88 + ((m) * (0x10000)))
#define SCB_INTR_TX_MASK(m)                                 (*(volatile uint32_t *)(0x40050f88 + ((m) * 0x10000)))
#define SCB_INTR_TX_MASK_DEFAULT                            (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_TRIGGER                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_NOT_FULL                           (1u << 1) /* <1:1> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_EMPTY                              (1u << 4) /* <4:4> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_OVERFLOW                           (1u << 5) /* <5:5> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_UNDERFLOW                          (1u << 6) /* <6:6> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_BLOCKED                            (1u << 7) /* <7:7> R:RW:0:EC */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_UART_NACK                          (1u << 8) /* <8:8> R:RW:0:UART */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_UART_DONE                          (1u << 9) /* <9:9> R:RW:0:UART */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_TX_MASK_UART_ARB_LOST                      (1u << 10) /* <10:10> R:RW:0:UART */


/*
 * Transmitter interrupt masked request register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers. This register allows SW to read the status
 * of all mask enabled interrupt causes with a single load operation, rather
 * than two load operations: one for the interrupt causes and one for the
 * masks. This simplifies Firmware development. The associated interrupt
 * is active ('1'), when INTR_TX_MASKED != 0.
 */
#define SCB_INTR_TX_MASKED_ADDRESS(m)                       (0x40050f8c + ((m) * (0x10000)))
#define SCB_INTR_TX_MASKED(m)                               (*(volatile uint32_t *)(0x40050f8c + ((m) * 0x10000)))
#define SCB_INTR_TX_MASKED_DEFAULT                          (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_TRIGGER                          (1u << 0) /* <0:0> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_NOT_FULL                         (1u << 1) /* <1:1> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_EMPTY                            (1u << 4) /* <4:4> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_OVERFLOW                         (1u << 5) /* <5:5> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_UNDERFLOW                        (1u << 6) /* <6:6> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_BLOCKED                          (1u << 7) /* <7:7> W:R:0:EC */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_UART_NACK                        (1u << 8) /* <8:8> W:R:0:UART */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_UART_DONE                        (1u << 9) /* <9:9> W:R:0:UART */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_TX_MASKED_UART_ARB_LOST                    (1u << 10) /* <10:10> W:R:0:UART */


/*
 * Receiver interrupt request register.
 * The register fields are not retained In DeepSleep power mode: HW clears
 * the interrupt causes to '0', when coming out of DeepSleep power mode.
 * In addition, HW clears the interrupt causes to '0', when the IP is disabled.
 * As a result, the interrupt causes are only available in Active/Sleep power
 * modes; they are generated by internally clocked logic (this logic operates
 * on a clock that is only available in Active/Sleep power modes).
 */
#define SCB_INTR_RX_ADDRESS(m)                              (0x40050fc0 + ((m) * (0x10000)))
#define SCB_INTR_RX(m)                                      (*(volatile uint32_t *)(0x40050fc0 + ((m) * 0x10000)))
#define SCB_INTR_RX_DEFAULT                                 (0x00000000)

/*
 * More entries in the RX FIFO than the value specified by TRIGGER_LEVEL
 * in SCB_RX_FIFO_CTL.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_RX_TRIGGER                                 (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * RX FIFO is not empty.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_RX_NOT_EMPTY                               (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * RX FIFO is full. Note that received data frames are lost when the RX FIFO
 * is full. Dependent on CTRL.BYTE_MODET:
 * BYTE_MODE is '0': # entries == FF_DATA_NR/2.
 * BYTE_MODE is '1': # entries == FF_DATA_NR.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_RX_FULL                                    (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * Attempt to write to a full RX FIFO. Note: in I2C mode, the OVERFLOW is
 * set when a data frame is received and the RX FIFO is full, independent
 * of whether it is ACK'd or NACK'd.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_RX_OVERFLOW                                (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * Attempt to read from an empty RX FIFO.
 *
 * Only used in FIFO mode.
 */
#define SCB_INTR_RX_UNDERFLOW                               (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * AHB-Lite read transfer can not get access to the EZ memory (EZ_DATA accesses),
 * due to an externally clocked EZ access. This may happen when STATUS.EC_BUSY
 * is '1'.
 */
#define SCB_INTR_RX_BLOCKED                                 (1u << 7) /* <7:7> RW1S:RW1C:0:EC */


/*
 * Frame error in received data frame. Set to '1', when event is detected.
 * Write with '1' to clear bit. This can be either a start or stop bit(s)
 * error:
 * Start bit error: after the detection of the beginning of a start bit period
 * (RX line changes from '1' to '0'), the middle of the start bit period
 * is sampled erroneously (RX line is '1').  Note: a start bit error is detected
 * BEFORE a data frame is received.
 * Stop bit error: the RX line is sampled as '0', but a '1' was expected.
 * Note: a stop bit error may result in failure to receive successive data
 * frame(s). Note: a stop bit error is detected AFTER a data frame is received.
 *
 * A stop bit error is detected after a data frame is received, and the UART_RX_CTL.DROP_ON_FRAME_ERROR
 * field specifies whether the received frame is dropped or send to the RX
 * FIFO. If UART_RX_CTL.DROP_ON_FRAME_ERROR is '1', the received data frame
 * is dropped. If UART_RX_CTL.DROP_ON_FRAME_ERROR is '0', the received data
 * frame is send to the RX FIFO. Note that Firmware can only identify the
 * erroneous data frame in the RX FIFO if it is fast enough to read the data
 * frame before the hardware writes a next data frame into the RX FIFO; i.e.
 * the RX FIFO does not have error flags to tag erroneous data frames.
 */
#define SCB_INTR_RX_FRAME_ERROR                             (1u << 8) /* <8:8> RW1S:RW1C:0:UART */


/*
 * Parity error in received data frame. Set to '1', when event is detected.
 * Write with '1' to clear bit. If UART_RX_CTL.DROP_ON_PARITY_ERROR is '1',
 * the received frame is dropped. If UART_RX_CTL.DROP_ON_PARITY_ERROR is
 * '0', the received frame is send to the RX FIFO. In SmartCard submode,
 * negatively acknowledged data frames generate a parity error. Note that
 * Firmware can only identify the erroneous data frame in the RX FIFO if
 * it is fast enough to read the data frame before the hardware writes a
 * next data frame into the RX FIFO.
 */
#define SCB_INTR_RX_PARITY_ERROR                            (1u << 9) /* <9:9> RW1S:RW1C:0:UART */


/*
 * LIN baudrate detection is completed.  The receiver software uses the UART_RX_STATUS.BR_COUNTER
 * value to set the right IP clock (from the programmable clock IP) to guarantee
 * successful receipt of the first LIN data frame (Protected Identifier Field)
 * after the synchronization byte. Set to '1', when event is detected. Write
 * with '1' to clear bit.
 */
#define SCB_INTR_RX_BAUD_DETECT                             (1u << 10) /* <10:10> RW1S:RW1C:0:UART */


/*
 * Break detection is successful: the line is '0' for UART_RX_CTRL.BREAK_WIDTH
 * + 1 bit period. Can occur at any time to address unanticipated break fields;
 * i.e. "break-in-data" is supported. This feature is supported for the UART
 * standard and LIN submodes. For the UART standard submodes, ongoing receipt
 * of data frames is NOT affected; i.e. Firmware is ecpected to take the
 * proper action. For the LIN submode, possible ongoing receipt of a data
 * frame is stopped and the (partially) received data frame is dropped and
 * baud rate detection is started. Set to '1', when event is detected. Write
 * with '1' to clear bit.
 */
#define SCB_INTR_RX_BREAK_DETECT                            (1u << 11) /* <11:11> RW1S:RW1C:0:UART */


/*
 * Receiver interrupt set request register.
 * When read, this register reflects the interrupt request register.
 */
#define SCB_INTR_RX_SET_ADDRESS(m)                          (0x40050fc4 + ((m) * (0x10000)))
#define SCB_INTR_RX_SET(m)                                  (*(volatile uint32_t *)(0x40050fc4 + ((m) * 0x10000)))
#define SCB_INTR_RX_SET_DEFAULT                             (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_SET_TRIGGER                             (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_NOT_EMPTY                           (1u << 2) /* <2:2> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_FULL                                (1u << 3) /* <3:3> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_OVERFLOW                            (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_UNDERFLOW                           (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_BLOCKED                             (1u << 7) /* <7:7> A:RW1S:0:EC */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_FRAME_ERROR                         (1u << 8) /* <8:8> A:RW1S:0:UART */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_PARITY_ERROR                        (1u << 9) /* <9:9> A:RW1S:0:UART */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_BAUD_DETECT                         (1u << 10) /* <10:10> A:RW1S:0:UART */


/*
 * Write with '1' to set corresponding bit in interrupt status register.
 */
#define SCB_INTR_RX_SET_BREAK_DETECT                        (1u << 11) /* <11:11> A:RW1S:0:UART */


/*
 * Receiver interrupt mask register.
 */
#define SCB_INTR_RX_MASK_ADDRESS(m)                         (0x40050fc8 + ((m) * (0x10000)))
#define SCB_INTR_RX_MASK(m)                                 (*(volatile uint32_t *)(0x40050fc8 + ((m) * 0x10000)))
#define SCB_INTR_RX_MASK_DEFAULT                            (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_TRIGGER                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_NOT_EMPTY                          (1u << 2) /* <2:2> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_FULL                               (1u << 3) /* <3:3> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_OVERFLOW                           (1u << 5) /* <5:5> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_UNDERFLOW                          (1u << 6) /* <6:6> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_BLOCKED                            (1u << 7) /* <7:7> R:RW:0:EC */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_FRAME_ERROR                        (1u << 8) /* <8:8> R:RW:0:UART */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_PARITY_ERROR                       (1u << 9) /* <9:9> R:RW:0:UART */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_BAUD_DETECT                        (1u << 10) /* <10:10> R:RW:0:UART */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define SCB_INTR_RX_MASK_BREAK_DETECT                       (1u << 11) /* <11:11> R:RW:0:UART */


/*
 * Receiver interrupt masked request register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers. This register allows SW to read the status
 * of all mask enabled interrupt causes with a single load operation, rather
 * than two load operations: one for the interrupt causes and one for the
 * masks. This simplifies Firmware development. The associated interrupt
 * is active ('1'), when INTR_RX_MASKED != 0.
 */
#define SCB_INTR_RX_MASKED_ADDRESS(m)                       (0x40050fcc + ((m) * (0x10000)))
#define SCB_INTR_RX_MASKED(m)                               (*(volatile uint32_t *)(0x40050fcc + ((m) * 0x10000)))
#define SCB_INTR_RX_MASKED_DEFAULT                          (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_TRIGGER                          (1u << 0) /* <0:0> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_NOT_EMPTY                        (1u << 2) /* <2:2> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_FULL                             (1u << 3) /* <3:3> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_OVERFLOW                         (1u << 5) /* <5:5> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_UNDERFLOW                        (1u << 6) /* <6:6> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_BLOCKED                          (1u << 7) /* <7:7> W:R:0:EC */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_FRAME_ERROR                      (1u << 8) /* <8:8> W:R:0:UART */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_PARITY_ERROR                     (1u << 9) /* <9:9> W:R:0:UART */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_BAUD_DETECT                      (1u << 10) /* <10:10> W:R:0:UART */


/*
 * Logical and of corresponding request and mask bits.
 */
#define SCB_INTR_RX_MASKED_BREAK_DETECT                     (1u << 11) /* <11:11> W:R:0:UART */


/*
 * TCPWM control register 0.
 * Used to disbale/enable the counters.
 */
#define TCPWM_CTRL_ADDRESS                                  (0x40090000)
#define TCPWM_CTRL                                          (*(volatile uint32_t *)(0x40090000))
#define TCPWM_CTRL_DEFAULT                                  (0x00000000)

/*
 * Counter enables for counters 0 up to CNT_NR-1.
 * '0': counter disabled.
 * '1': counter enabled.
 * Counter static configuration information (e.g. CTRL.MODE, all TR_CTRL0,
 * TR_CTRL1, and TR_CTRL2 register fields) should only be modified when the
 * counter is disabled. When a counter is disabled, command and status information
 * associated to the counter is cleared by HW, this includes:
 * - the associated counter triggers in the CMD register are set to '0'.
 * - the counter's interrupt cause fields in counter's INTR register.
 * - the counter's status fields in counter's STATUS register..
 * - the counter's trigger outputs ("tr_overflow", "tr_underflow" and "tr_compare_match").
 * - the counter's line outputs ("line_out" and "line_compl_out").
 */
#define TCPWM_CTRL_COUNTER_ENABLED_MASK                     (0x00000003) /* <0:1> R:RW:0:CNT_NR */
#define TCPWM_CTRL_COUNTER_ENABLED_POS                      (0)


/*
 * TCPWM command register.
 * Enables software controlled counter operation.
 */
#define TCPWM_CMD_ADDRESS                                   (0x40090008)
#define TCPWM_CMD                                           (*(volatile uint32_t *)(0x40090008))
#define TCPWM_CMD_DEFAULT                                   (0x00000000)

/*
 * Counters SW capture trigger. When written with '1', a capture trigger
 * is generated and the HW sets the field to '0' when the SW trigger has
 * taken effect. It should be noted that the HW operates on the counter frequency.
 * If the counter is disabled through CTRL.COUNTER_ENABLED, the field is
 * immediately set to '0'.
 */
#define TCPWM_CMD_COUNTER_CAPTURE_MASK                      (0x00000003) /* <0:1> RW1C:RW1S:0:CNT_NR */
#define TCPWM_CMD_COUNTER_CAPTURE_POS                       (0)


/*
 * Counters SW reload trigger. For HW behavior, see COUNTER_CAPTURE field.
 */
#define TCPWM_CMD_COUNTER_RELOAD_MASK                       (0x00000300) /* <8:9> RW1C:RW1S:0:CNT_NR */
#define TCPWM_CMD_COUNTER_RELOAD_POS                        (8)


/*
 * Counters SW stop trigger. For HW behavior, see COUNTER_CAPTURE field.
 */
#define TCPWM_CMD_COUNTER_STOP_MASK                         (0x00030000) /* <16:17> RW1C:RW1S:0:CNT_NR */
#define TCPWM_CMD_COUNTER_STOP_POS                          (16)


/*
 * Counters SW start trigger. For HW behavior, see COUNTER_CAPTURE field.
 */
#define TCPWM_CMD_COUNTER_START_MASK                        (0x03000000) /* <24:25> RW1C:RW1S:0:CNT_NR */
#define TCPWM_CMD_COUNTER_START_POS                         (24)


/*
 * TCPWM Counter interrupt cause register.
 * Enables software to determine the source of the combined interrupt output
 * signal "interrupt". The register fields are not retained. This is to ensure
 * that they come up as '0' after coming out of DeepSleep system power mode.
 */
#define TCPWM_INTR_CAUSE_ADDRESS                            (0x4009000c)
#define TCPWM_INTR_CAUSE                                    (*(volatile uint32_t *)(0x4009000c))
#define TCPWM_INTR_CAUSE_DEFAULT                            (0x00000000)

/*
 * Counters interrupt signal active. If the counter is disabled through CTRL.COUNTER_ENABLED,
 * the associated interrupt field is immediately set to '0'.
 */
#define TCPWM_INTR_CAUSE_COUNTER_INT_MASK                   (0x00000003) /* <0:1> W:R:0:CNT_NR */
#define TCPWM_INTR_CAUSE_COUNTER_INT_POS                    (0)


/*
 * Counter control register
 */
#define TCPWM_CNT_CTRL_ADDRESS(m)                           (0x40090100 + ((m) * (0x0040)))
#define TCPWM_CNT_CTRL(m)                                   (*(volatile uint32_t *)(0x40090100 + ((m) * 0x0040)))
#define TCPWM_CNT_CTRL_DEFAULT                              (0x00000000)

/*
 * Specifies switching of the CC and buffered CC values. This field has a
 * function in TIMER, PWM, PWM_DT and PWM_PR modes.
 * Timer mode:
 * '0': never switch.
 * '1': switch on a compare match event.
 * PWM, PWM_DT, PWM_PR modes:
 * '0: never switch.
 * '1': switch on a terminal count event with an actively pending switch
 * event.
 */
#define TCPWM_CNT_CTRL_AUTO_RELOAD_CC                       (1u << 0) /* <0:0> R:RW:0: */


/*
 * Specifies switching of the PERIOD and buffered PERIOD values. This field
 * has a function in PWM, PWM_DT and PWM_PR modes.
 * '0': never switch.
 * '1': switch on a terminal count event with and actively pending siwtch
 * event.
 */
#define TCPWM_CNT_CTRL_AUTO_RELOAD_PERIOD                   (1u << 1) /* <1:1> R:RW:0: */


/*
 * Specifies asynchronous/synchronous kill behavior:
 * '1': synchronous kill mode: the kill event disables the "dt_line_out"
 * and "dt_line_compl_out" signals till the next terminal count event (synchronous
 * kill). In synchronous kill mode, STOP_EDGE should  be RISING_EDGE.
 * '0': asynchronous kill mode: the kill event only disables the "dt_line_out"
 * and "dt_line_compl_out" signals when present. In asynchronous kill mode,
 * STOP_EDGE should be NO_EDGE_DET.
 *
 * This field has a function in PWM and PWM_DT modes only. This field is
 * only used when PWM_STOP_ON_KILL is '0'.
 */
#define TCPWM_CNT_CTRL_PWM_SYNC_KILL                        (1u << 2) /* <2:2> R:RW:0: */


/*
 * Specifies whether the counter stops on a kill events:
 * '0': kill event does NOT stop counter.
 * '1': kill event stops counter.
 *
 * This field has a function in PWM, PWM_DT and PWM_PR modes only.
 */
#define TCPWM_CNT_CTRL_PWM_STOP_ON_KILL                     (1u << 3) /* <3:3> R:RW:0: */


/*
 * Generic 8-bit control field. In PWM_DT mode, this field is used to determine
 * the dead time: amount of dead time cycles in the counter clock domain.
 * In all other modes, the lower 3 bits of this field determine pre-scaling
 * of the selected counter clock.
 */
#define TCPWM_CNT_CTRL_GENERIC_MASK                         (0x0000ff00) /* <8:15> R:RW:0: */
#define TCPWM_CNT_CTRL_GENERIC_POS                          (8)


/*
 * Determines counter direction.
 */
#define TCPWM_CNT_CTRL_UP_DOWN_MODE_MASK                    (0x00030000) /* <16:17> R:RW:0: */
#define TCPWM_CNT_CTRL_UP_DOWN_MODE_POS                     (16)


/*
 * When '0', counter runs continuous. When '1', counter is turned off by
 * hardware when a terminal count event is generated.
 */
#define TCPWM_CNT_CTRL_ONE_SHOT                             (1u << 18) /* <18:18> R:RW:0: */


/*
 * In QUAD mode selects quadrature encoding mode (X1/X2/X4).
 * In PWM, PWM_DT and PWM_PR modes, these two bits can be used to invert
 * "dt_line_out" and "dt_line_compl_out".  Inversion is the last step in
 * generation of "dt_line_out" and "dt_line_compl_out"; i.e. a disabled output
 * line "dt_line_out" has the value QUADRATURE_MODE[0] and a disabled output
 * line "dt_line_compl_out" has the value QUADRATURE_MODE[1].
 */
#define TCPWM_CNT_CTRL_QUADRATURE_MODE_MASK                 (0x00300000) /* <20:21> R:RW:0: */
#define TCPWM_CNT_CTRL_QUADRATURE_MODE_POS                  (20)


/*
 * Counter mode.
 */
#define TCPWM_CNT_CTRL_MODE_MASK                            (0x07000000) /* <24:26> R:RW:0: */
#define TCPWM_CNT_CTRL_MODE_POS                             (24)


/*
 * Counter status register
 */
#define TCPWM_CNT_STATUS_ADDRESS(m)                         (0x40090104 + ((m) * (0x0040)))
#define TCPWM_CNT_STATUS(m)                                 (*(volatile uint32_t *)(0x40090104 + ((m) * 0x0040)))
#define TCPWM_CNT_STATUS_DEFAULT                            (0x00000000)

/*
 * When '0', counter is counting up. When '1', counter is counting down.
 * In QUAD mode, this field indicates the direction of the latest counter
 * change: '0' when last incremented and '1' when last decremented.
 */
#define TCPWM_CNT_STATUS_DOWN                               (1u << 0) /* <0:0> RW:R:0: */


/*
 * Generic 8-bit counter field. In PWM_DT mode, this counter is used for
 * dead time insertion. In all other modes, this counter is used for pre-scaling
 * the selected counter clock. PWM_DT mode can NOT use prescaled clock functionality.
 */
#define TCPWM_CNT_STATUS_GENERIC_MASK                       (0x0000ff00) /* <8:15> RW:R:0: */
#define TCPWM_CNT_STATUS_GENERIC_POS                        (8)


/*
 * When '0', the counter is NOT running. When '1', the counter is running.
 */
#define TCPWM_CNT_STATUS_RUNNING                            (1u << 31) /* <31:31> RW:R:0: */


/*
 * Counter count register
 */
#define TCPWM_CNT_COUNTER_ADDRESS(m)                        (0x40090108 + ((m) * (0x0040)))
#define TCPWM_CNT_COUNTER(m)                                (*(volatile uint32_t *)(0x40090108 + ((m) * 0x0040)))
#define TCPWM_CNT_COUNTER_DEFAULT                           (0x00000000)

/*
 * 16-bit counter value. It is advised to not write to this field when the
 * counter is running.
 */
#define TCPWM_CNT_COUNTER_COUNTER_MASK                      (0x0000ffff) /* <0:15> RW:RW:0: */
#define TCPWM_CNT_COUNTER_COUNTER_POS                       (0)


/*
 * Counter compare/capture register
 */
#define TCPWM_CNT_CC_ADDRESS(m)                             (0x4009010c + ((m) * (0x0040)))
#define TCPWM_CNT_CC(m)                                     (*(volatile uint32_t *)(0x4009010c + ((m) * 0x0040)))
#define TCPWM_CNT_CC_DEFAULT                                (0x0000ffff)

/*
 * In CAPTURE mode, captures the counter value. In other modes, compared
 * to counter value.
 */
#define TCPWM_CNT_CC_CC_MASK                                (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_CC_CC_POS                                 (0)


/*
 * Counter buffered compare/capture register
 */
#define TCPWM_CNT_CC_BUFF_ADDRESS(m)                        (0x40090110 + ((m) * (0x0040)))
#define TCPWM_CNT_CC_BUFF(m)                                (*(volatile uint32_t *)(0x40090110 + ((m) * 0x0040)))
#define TCPWM_CNT_CC_BUFF_DEFAULT                           (0x0000ffff)

/*
 * Additional buffer for counter CC register.
 */
#define TCPWM_CNT_CC_BUFF_CC_MASK                           (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_CC_BUFF_CC_POS                            (0)


/*
 * Counter period register
 */
#define TCPWM_CNT_PERIOD_ADDRESS(m)                         (0x40090114 + ((m) * (0x0040)))
#define TCPWM_CNT_PERIOD(m)                                 (*(volatile uint32_t *)(0x40090114 + ((m) * 0x0040)))
#define TCPWM_CNT_PERIOD_DEFAULT                            (0x0000ffff)

/*
 * Period value: upper value of the counter. When the counter should count
 * for n cycles, this field should be set to n-1.
 */
#define TCPWM_CNT_PERIOD_PERIOD_MASK                        (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_PERIOD_PERIOD_POS                         (0)


/*
 * Counter buffered period register
 */
#define TCPWM_CNT_PERIOD_BUFF_ADDRESS(m)                    (0x40090118 + ((m) * (0x0040)))
#define TCPWM_CNT_PERIOD_BUFF(m)                            (*(volatile uint32_t *)(0x40090118 + ((m) * 0x0040)))
#define TCPWM_CNT_PERIOD_BUFF_DEFAULT                       (0x0000ffff)

/*
 * Additional buffer for counter PERIOD register.
 */
#define TCPWM_CNT_PERIOD_BUFF_PERIOD_MASK                   (0x0000ffff) /* <0:15> RW:RW:65535: */
#define TCPWM_CNT_PERIOD_BUFF_PERIOD_POS                    (0)


/*
 * Counter trigger control register 0
 * Used to select triggers for specific counter events.
 */
#define TCPWM_CNT_TR_CTRL0_ADDRESS(m)                       (0x40090120 + ((m) * (0x0040)))
#define TCPWM_CNT_TR_CTRL0(m)                               (*(volatile uint32_t *)(0x40090120 + ((m) * 0x0040)))
#define TCPWM_CNT_TR_CTRL0_DEFAULT                          (0x00000010)

/*
 * Selects one of the 16 input triggers as a capture trigger. Input trigger
 * 0 is always '0' and input trigger is always '1'. In the PWM, PWM_DT and
 * PWM_PR modes this trigger is used to switch the values if the compare
 * and period registers with their buffer counterparts.
 */
#define TCPWM_CNT_TR_CTRL0_CAPTURE_SEL_MASK                 (0x0000000f) /* <0:3> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_CAPTURE_SEL_POS                  (0)


/*
 * Selects one of the 16 input triggers as a count trigger. In QUAD mode,
 * this is the first phase (phi A). Default setting selects input trigger
 * 1, which is always '1'.
 */
#define TCPWM_CNT_TR_CTRL0_COUNT_SEL_MASK                   (0x000000f0) /* <4:7> R:RW:1: */
#define TCPWM_CNT_TR_CTRL0_COUNT_SEL_POS                    (4)


/*
 * Selects one of the 16 input triggers as a reload trigger. In QUAD mode,
 * this is the index or revolution pulse. In this mode, it will update the
 * counter with the value in the TCPWM_CNTn_PERIOD register.
 */
#define TCPWM_CNT_TR_CTRL0_RELOAD_SEL_MASK                  (0x00000f00) /* <8:11> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_RELOAD_SEL_POS                   (8)


/*
 * Selects one of the 16 input triggers as a stop trigger. In PWM, PWM_DT
 * and PWM_PR modes, this is the kill trigger. In these modes, the kill trigger
 * is used to either temporarily block the PWM outputs (PWM_STOP_ON_KILL
 * is '0') or stop the functionality (PWM_STOP_ON_KILL is '1'). For the PWM
 * and PWM_DT modes, the blocking of the output signals can be  asynchronous
 * (STOP_EDGE should be NO_EDGE_DET) in which case the blocking is as long
 * as the trigger is '1' or synchronous (STOP_EDGE should be RISING_EDGE)
 * in which case it extends till the next terminal count event.
 */
#define TCPWM_CNT_TR_CTRL0_STOP_SEL_MASK                    (0x0000f000) /* <12:15> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_STOP_SEL_POS                     (12)


/*
 * Selects one of the 16 input triggers as a start trigger. In QUAD mode,
 * this is the second phase (phi B).
 */
#define TCPWM_CNT_TR_CTRL0_START_SEL_MASK                   (0x000f0000) /* <16:19> R:RW:0: */
#define TCPWM_CNT_TR_CTRL0_START_SEL_POS                    (16)


/*
 * Counter trigger control register 1
 * Used to determine edge detection for specific counter triggers. Events
 * will only take effect on enabled counters.
 */
#define TCPWM_CNT_TR_CTRL1_ADDRESS(m)                       (0x40090124 + ((m) * (0x0040)))
#define TCPWM_CNT_TR_CTRL1(m)                               (*(volatile uint32_t *)(0x40090124 + ((m) * 0x0040)))
#define TCPWM_CNT_TR_CTRL1_DEFAULT                          (0x000003ff)

/*
 * A capture event will copy the counter value into the CC register.
 */
#define TCPWM_CNT_TR_CTRL1_CAPTURE_EDGE_MASK                (0x00000003) /* <0:1> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_CAPTURE_EDGE_POS                 (0)


/*
 * A counter event will increase or decrease the counter by '1'.
 */
#define TCPWM_CNT_TR_CTRL1_COUNT_EDGE_MASK                  (0x0000000c) /* <2:3> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_COUNT_EDGE_POS                   (2)


/*
 * A reload event will initialize the counter. When counting up, the counter
 * is initialized to "0". When counting down, the counter is initialized
 * with PERIOD.
 */
#define TCPWM_CNT_TR_CTRL1_RELOAD_EDGE_MASK                 (0x00000030) /* <4:5> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_RELOAD_EDGE_POS                  (4)


/*
 * A stop event, will stop the counter; i.e. it will no longer be running.
 * Stopping will NOT disable the counter.
 */
#define TCPWM_CNT_TR_CTRL1_STOP_EDGE_MASK                   (0x000000c0) /* <6:7> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_STOP_EDGE_POS                    (6)


/*
 * A start event will start the counter; i.e. the counter will become running.
 * Starting does NOT enable the counter. A start event will not initialize
 * the counter whereas the reload event does.
 */
#define TCPWM_CNT_TR_CTRL1_START_EDGE_MASK                  (0x00000300) /* <8:9> R:RW:3: */
#define TCPWM_CNT_TR_CTRL1_START_EDGE_POS                   (8)


/*
 * Counter trigger control register 2
 * Used to control counter "line_out", "dt_line_out" and "dt_line_compl_out"
 * output signals.
 */
#define TCPWM_CNT_TR_CTRL2_ADDRESS(m)                       (0x40090128 + ((m) * (0x0040)))
#define TCPWM_CNT_TR_CTRL2(m)                               (*(volatile uint32_t *)(0x40090128 + ((m) * 0x0040)))
#define TCPWM_CNT_TR_CTRL2_DEFAULT                          (0x0000003f)

/*
 * Determines the effect of a compare match event (COUNTER equals CC register)
 * on the "line_out" output signals.  Note that INVERT is especially useful
 * for center aligned pulse width modulation.
 * To generate a duty cycle of 0%, the counter CC register should be set
 * to "0". For a 100% duty cycle, the counter CC register should be set to
 * larger than the counter PERIOD register.
 */
#define TCPWM_CNT_TR_CTRL2_CC_MATCH_MODE_MASK               (0x00000003) /* <0:1> R:RW:3: */
#define TCPWM_CNT_TR_CTRL2_CC_MATCH_MODE_POS                (0)


/*
 * Determines the effect of a counter overflow event (COUNTER reaches PERIOD)
 * on the "line_out" output signals.
 */
#define TCPWM_CNT_TR_CTRL2_OVERFLOW_MODE_MASK               (0x0000000c) /* <2:3> R:RW:3: */
#define TCPWM_CNT_TR_CTRL2_OVERFLOW_MODE_POS                (2)


/*
 * Determines the effect of a counter underflow event (COUNTER reaches "0")
 * on the "line_out" output signals.
 */
#define TCPWM_CNT_TR_CTRL2_UNDERFLOW_MODE_MASK              (0x00000030) /* <4:5> R:RW:3: */
#define TCPWM_CNT_TR_CTRL2_UNDERFLOW_MODE_POS               (4)


/*
 * Interrupt request register.
 * The register fields are not retained. This is to ensure that they come
 * up as '0' after coming out of DeepSleep system power mode. HW clears the
 * interrupt causes to '0', when the counter is disabled.
 */
#define TCPWM_CNT_INTR_ADDRESS(m)                           (0x40090130 + ((m) * (0x0040)))
#define TCPWM_CNT_INTR(m)                                   (*(volatile uint32_t *)(0x40090130 + ((m) * 0x0040)))
#define TCPWM_CNT_INTR_DEFAULT                              (0x00000000)

/*
 * Terminal count event. Set to '1', when event is detected. Write with '1'
 * to clear bit.
 */
#define TCPWM_CNT_INTR_TC                                   (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * Counter matches CC register event. Set to '1', when event is detected.
 * Write with '1' to clear bit.
 */
#define TCPWM_CNT_INTR_CC_MATCH                             (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * Interrupt set request register.
 * When read, this register reflects the interrupt request register.
 */
#define TCPWM_CNT_INTR_SET_ADDRESS(m)                       (0x40090134 + ((m) * (0x0040)))
#define TCPWM_CNT_INTR_SET(m)                               (*(volatile uint32_t *)(0x40090134 + ((m) * 0x0040)))
#define TCPWM_CNT_INTR_SET_DEFAULT                          (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_SET_TC                               (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_SET_CC_MATCH                         (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Interrupt mask register.
 */
#define TCPWM_CNT_INTR_MASK_ADDRESS(m)                      (0x40090138 + ((m) * (0x0040)))
#define TCPWM_CNT_INTR_MASK(m)                              (*(volatile uint32_t *)(0x40090138 + ((m) * 0x0040)))
#define TCPWM_CNT_INTR_MASK_DEFAULT                         (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_MASK_TC                              (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define TCPWM_CNT_INTR_MASK_CC_MATCH                        (1u << 1) /* <1:1> R:RW:0: */


/*
 * Interrupt masked request register
 * When read, this register reflects a bitwise AND between the interrupt
 * request and mask registers.
 */
#define TCPWM_CNT_INTR_MASKED_ADDRESS(m)                    (0x4009013c + ((m) * (0x0040)))
#define TCPWM_CNT_INTR_MASKED(m)                            (*(volatile uint32_t *)(0x4009013c + ((m) * 0x0040)))
#define TCPWM_CNT_INTR_MASKED_DEFAULT                       (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define TCPWM_CNT_INTR_MASKED_TC                            (1u << 0) /* <0:0> W:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define TCPWM_CNT_INTR_MASKED_CC_MATCH                      (1u << 1) /* <1:1> W:R:0: */


/*
 * Generic control register.
 */
#define PDSS_CTRL_ADDRESS(m)                                (0x400a0000 + ((m) * (0x10000)))
#define PDSS_CTRL(m)                                        (*(volatile uint32_t *)(0x400a0000 + ((m) * 0x10000)))
#define PDSS_CTRL_DEFAULT                                   (0x00000000)

/*
 * Setting this register will bypass 5b/4b, CRC.
 */
#define PDSS_CTRL_TX_BYPASS_EN                              (1u << 0) /* <0:0> R:RW:0: */


/*
 * Setting this register will bypass 5b/4b, CRC.
 */
#define PDSS_CTRL_RX_BYPASS_EN_MASK                         (0x00000006) /* <1:2> R:RW:0: */
#define PDSS_CTRL_RX_BYPASS_EN_POS                          (1)


/*
 * Defines the direction of GPIO used for HPD operation.
 * 0: GPIO is input. Receive operation.
 * 1: GPIO is output. Transmit operation
 * This bit must be set for correct mode of HPD operation (Receive / Transmit).
 */
#define PDSS_CTRL_HPD_DIRECTION                             (1u << 3) /* <3:3> R:RW:0: */


/*
 * 0: Clocks is turn off for HPD block
 * 1: Clock is runing in the HPD block
 * This bit must be set when HPD Receive functionality is needed. This will
 * turn clock on for the RX module. resetting to zero will turn clock off.
 * Clock used for HPD is same clock that is used for CC transmit module (600
 * Khz).
 */
#define PDSS_CTRL_HPD_ENABLED                               (1u << 29) /* <29:29> R:RW:0: */


/*
 * 0: Clocks is turn off for HPDT block
 * 1: Clock is runing in the HPDT block.
 * This bit must be set when HPDT transmit functionality is needed. This
 * will turn clock on for the TX module. resetting to zero will turn clock
 * off.
 * Clock used for HPD is same clock that is used for CC transmit module (600
 * Khz).
 */
#define PDSS_CTRL_HPDT_ENABLED                              (1u << 30) /* <30:30> R:RW:0: */


/*
 * IP enabled ('1') or not ('0').
 * "0" Resets the IP. The reset is an async reset.
 * Note that when the IP is disabled, all the interrupt sources are also
 * disabled.
 * All the clocks that their source is clk_hf will be turned off when IP
 * is disabled.
 */
#define PDSS_CTRL_IP_ENABLED                                (1u << 31) /* <31:31> R:RW:0: */


/*
 * Header INFO
 */
#define PDSS_HEADER_INFO_ADDRESS(m)                         (0x400a0004 + ((m) * (0x10000)))
#define PDSS_HEADER_INFO(m)                                 (*(volatile uint32_t *)(0x400a0004 + ((m) * 0x10000)))
#define PDSS_HEADER_INFO_DEFAULT                            (0x03f10f00)

/*
 * This bit will enable/disable extended data messaging.
 * 0: Disable RX/TX extended data messaging
 * 1: Enable  RX/TX extended data messaging
 */
#define PDSS_HEADER_INFO_EN_RX_EXTENDED_DATA                (1u << 0) /* <0:0> R:RW:0: */


/*
 * This bit will enable/disable extended data messaging.
 * 0: Disable RX/TX extended data messaging
 * 1: Enable  RX/TX extended data messaging
 */
#define PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA                (1u << 1) /* <1:1> R:RW:0: */


/*
 * The location of the extended data field in the Header[15:0].
 * 0: First Bit of the header
 * 1: Second Bit of the header
 * …
 * 15: 15th bit of theheader
 */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BIT_LOCATION_MASK    (0x00000f00) /* <8:11> R:RW:15: */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BIT_LOCATION_POS     (8)


/*
 * The first bit location of the extended data size field in the header.
 * 0: First Bit of the header
 * 1: Second Bit of the header
 * …
 * 31: Thirt first bit of the header
 */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_MASK    (0x0001f000) /* <12:16> R:RW:16: */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_FIRST_BIT_LOCATION_POS    (12)


/*
 * The first bit location of the extended data size field in the header.
 * 0: First Bit of the header
 * 1: Second Bit of the header
 * …
 * 31: Thirt first bit of the header
 */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_MASK    (0x003e0000) /* <17:21> R:RW:24: */
#define PDSS_HEADER_INFO_EXTENDED_DATA_BYTE_LAST_BIT_LOCATION_POS    (17)


/*
 * The location of the chunk field in the Header[31:16].
 * 16: 16th Bit of the header
 * 17: 17th Bit of the header
 * …
 * 31: 31th bit of theheader
 */
#define PDSS_HEADER_INFO_CHUNK_BIT_LOCATION_MASK            (0x03c00000) /* <22:25> R:RW:15: */
#define PDSS_HEADER_INFO_CHUNK_BIT_LOCATION_POS             (22)


/*
 * Transmit Header
 */
#define PDSS_TX_HEADER_ADDRESS(m)                           (0x400a000c + ((m) * (0x10000)))
#define PDSS_TX_HEADER(m)                                   (*(volatile uint32_t *)(0x400a000c + ((m) * 0x10000)))
#define PDSS_TX_HEADER_DEFAULT                              (0x00000000)

/*
 * The transmit Header. This register contains the 16-bit(Regular Packet)
 * or 32-bit(Extended) header. Hardware uses this register along with HEADER_INFO
 * register to send either 16-bit (Regular Packet) or 32-bit (Extended Packet)
 * header.
 */
#define PDSS_TX_HEADER_TX_HEADER_MASK                       (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_TX_HEADER_TX_HEADER_POS                        (0)


/*
 * TX SRAM Data
 * The memory for the TX USB power controller is a 64 byte SRAM. This SRAM
 * containts only Data part of a packet in non-bypass mode.
 * Any access to address space 0x0010 - 0x004C will map to SRAM address x0-x31
 */
#define PDSS_TX_MEM_DATA_ADDRESS(m,n)                       (0x400a0010 + ((m) * (0x10000)) + ((n) * (0x0004)))
#define PDSS_TX_MEM_DATA(m)                                 (*(volatile uint32_t *)(0x400a0010 + ((m) * (0x10000)) + ((n) * 0x0004)))
#define PDSS_TX_MEM_DATA_DEFAULT                            (0x00000000)

/*
 * Data information in the transmitter SRAM. SOP/CRC/EOP will be appened
 * by HW.
 * The TX Header needs to be programmed in the TX_HEADER register.
 */
#define PDSS_TX_MEM_DATA_DATA_MASK                          (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_TX_MEM_DATA_DATA_POS                           (0)


/*
 * Receive Header
 */
#define PDSS_RX_HEADER_ADDRESS(m)                           (0x400a005c + ((m) * (0x10000)))
#define PDSS_RX_HEADER(m)                                   (*(volatile uint32_t *)(0x400a005c + ((m) * 0x10000)))
#define PDSS_RX_HEADER_DEFAULT                              (0x00000000)

/*
 * The receive Header. This register contains the 16-bit(Regular Packet)
 * or 32-bit(Extended) header. The INTR2.EXTENDED_MSG_DET and INTR2.CHUNK_DET
 * interrupts indicates of the Packet type.  Hardware uses the HEADER_INFO
 * register for decoding the incoming packets.
 */
#define PDSS_RX_HEADER_RX_HEADER_MASK                       (0xffffffff) /* <0:31> RW:R:0: */
#define PDSS_RX_HEADER_RX_HEADER_POS                        (0)


/*
 * RX SRAM Data
 * The memory for the RX USB power controller is a 64 byte SRAM. This SRAM
 * containts only the Data part of a packet in non-bypass mode.
 * Any access to address space 0x0060 - 0x009C will map to SRAM address x0-x31
 */
#define PDSS_RX_MEM_DATA_ADDRESS(m,n)                       (0x400a0060 + ((m) * (0x10000)) + ((n) * (0x0004)))
#define PDSS_RX_MEM_DATA(m)                                 (*(volatile uint32_t *)(0x400a0060 + ((m) * (0x10000)) + ((n) * 0x0004)))
#define PDSS_RX_MEM_DATA_DEFAULT                            (0x00000000)

/*
 * The Data information in the receive SRAM. SOP type is stored in STATUS.SOP_TYPE_DETECTED
 * register.
 * STATUS.SOP_TYPE_DETECTED contains the SOP type for the packet in the RX
 * SRAM.
 * At the start of every packet, INTR.RCV_PACKET_COMPLETE and INTR.RCV_RST
 * status is evaluated, if its reset, then only a new packet will be written
 * else new packet will be dropped.
 */
#define PDSS_RX_MEM_DATA_DATA_MASK                          (0xffffffff) /* <0:31> RW:R:0: */
#define PDSS_RX_MEM_DATA_DATA_POS                           (0)


/*
 * TX/RX SRAM Read/Write pointer
 * FW can use these pointers to Read/Write more data after the RX_SRAM_HALF_END/TX_SRAM_HALF_END
 * interrupts.
 */
#define PDSS_SRAM_PTR_ADDRESS(m)                            (0x400a00a0 + ((m) * (0x10000)))
#define PDSS_SRAM_PTR(m)                                    (*(volatile uint32_t *)(0x400a00a0 + ((m) * 0x10000)))
#define PDSS_SRAM_PTR_DEFAULT                               (0x00000000)

/*
 * The transmit SRAM read pointer.
 */
#define PDSS_SRAM_PTR_TX_FUNC_RD_PTR_MASK                   (0x0000001f) /* <0:4> RW:R:0: */
#define PDSS_SRAM_PTR_TX_FUNC_RD_PTR_POS                    (0)


/*
 * The recevie SRAM write pointer.
 */
#define PDSS_SRAM_PTR_RX_FUNC_WR_PTR_MASK                   (0x00001f00) /* <8:12> RW:R:0: */
#define PDSS_SRAM_PTR_RX_FUNC_WR_PTR_POS                    (8)


/*
 * The packet size is odd. Only the first byte of the last two byte is valid.
 * (RX_FUNC_WR_PTR - 1)
 */
#define PDSS_SRAM_PTR_RX_ODD_LENGTH                         (1u << 13) /* <13:13> RW:R:0: */


/*
 * Generic status register.
 */
#define PDSS_STATUS_ADDRESS(m)                              (0x400a00a4 + ((m) * (0x10000)))
#define PDSS_STATUS(m)                                      (*(volatile uint32_t *)(0x400a00a4 + ((m) * 0x10000)))
#define PDSS_STATUS_DEFAULT                                 (0x00000000)

/*
 * Receiver is currently receiving a packet.
 * This signal gives information that Controlller has locked on to SOP and
 * is in process of receiving a packet. This will assert only after locking
 * on to Sop* symbols
 */
#define PDSS_STATUS_RX_BUSY                                 (1u << 0) /* <0:0> RW:R:0: */


/*
 * Transmitter is currently transmitting a packet or the crc timmer is running
 */
#define PDSS_STATUS_TX_BUSY                                 (1u << 1) /* <1:1> RW:R:0: */


/*
 * This status bit shows the CC_RX_VALID signal.
 * 0: No Valid data on the CC line'
 * 1: Valid Data detectd on the CC line
 */
#define PDSS_STATUS_CC_DATA_VALID                           (1u << 2) /* <2:2> RW:R:0: */


/*
 * Type of SOP detected for the packet stored in the RX SRAM:
 * At the start of every packet, INTR.RCV_PACKET_COMPLETE and INTR.RCV_RST
 * status is evaluated.
 * If both are "0", then this register will be updated with the new packet
 * SOP value.
 * There is no clearing option.
 */
#define PDSS_STATUS_SOP_TYPE_DETECTED_MASK                  (0x00000038) /* <3:5> RW:R:0: */
#define PDSS_STATUS_SOP_TYPE_DETECTED_POS                   (3)


/*
 * GoodCrc Message SOP type detected:
 * At the start of every packet, INTR.RCV_GOODCRC_MSG_COMPLETE status is
 * evaluated, if its reset, then this register will be updated with the new
 * packet SOP value.
 */
#define PDSS_STATUS_GOODCRC_MSG_SOP_TYPE_DETECTED_MASK      (0x000001c0) /* <6:8> RW:R:0: */
#define PDSS_STATUS_GOODCRC_MSG_SOP_TYPE_DETECTED_POS       (6)


/*
 * RST Type detected:
 */
#define PDSS_STATUS_RST_TYPE_DET_MASK                       (0x00000e00) /* <9:11> RW:R:0: */
#define PDSS_STATUS_RST_TYPE_DET_POS                        (9)


/*
 * CC1 status (wakeup interrupt from deepsleep)
 * 1: CC1 attached
 * 0: CC1 detached
 */
#define PDSS_STATUS_CC1_STATUS                              (1u << 14) /* <14:14> RW:R:0: */


/*
 * CC2 status (wakeup interrupt from deepsleep)
 * 1: CC2 attached
 * 0: CC2 detached
 */
#define PDSS_STATUS_CC2_STATUS                              (1u << 15) /* <15:15> RW:R:0: */


/*
 * This register provides the status CC_LINE_ACTIVITY (wakeup interrupt from
 * deepsleep).
 */
#define PDSS_STATUS_VCMP_LA_STATUS                          (1u << 16) /* <16:16> RW:R:0: */


/*
 * This register provides the status of VCMP_UP (wakeup interrupt from deepsleep).
 * Not enabled unless CMP_EN bit is set.
 * Edge: Ra/Rd value changed
 * {VCMP_UP, VCMP_DN}:
 * 00: Ra connected
 * 01: Rd connected
 * 11: Nothing connected (float)
 *                OR
 * Edge: Rp value changed
 * {VCMP_UP, VCMP_DN}:
 * 00: Default Rp broadcast
 * 01: 1.5A Rp broadcast
 * 11: 3.0A Rp broadcast
 */
#define PDSS_STATUS_VCMP_UP_STATUS                          (1u << 17) /* <17:17> RW:R:0: */


/*
 * This register provides the status of VCMP_DN (wakeup interrupt from deepsleep).
 * Not enabled unless CMP_EN bit is set.
 * Edge: Ra/Rd value changed
 *                OR
 * Edge: Rp value changed
 */
#define PDSS_STATUS_VCMP_DN_STATUS                          (1u << 18) /* <18:18> RW:R:0: */


/*
 * This status bit shows the VCM_FS comparator output from s8usbpd_cc_top.
 */
#define PDSS_STATUS_VCMP_FS                                 (1u << 19) /* <19:19> RW:R:0: */


/*
 * This status bit shows that the transmit is in the process of sending googcrc
 * msg
 */
#define PDSS_STATUS_SENDING_GOODCRC_MSG                     (1u << 20) /* <20:20> RW:R:0: */


/*
 * This status bit shows the v5v from s8usbpd_300ma_sw_top
 */
#define PDSS_STATUS_V5V_STATUS                              (1u << 21) /* <21:21> RW:R:0:FET_300_EN */


/*
 * The status of cmp_out from the first ADC
 */
#define PDSS_STATUS_ADC1_CMP_OUT_STATUS                     (1u << 22) /* <22:22> RW:R:0: */


/*
 * The status of cmp_out from the second ADC
 */
#define PDSS_STATUS_ADC2_CMP_OUT_STATUS                     (1u << 23) /* <23:23> RW:R:0: */


/*
 * This reads the level of HPD input from GPIO.
 */
#define PDSS_STATUS_HPD_STATUS                              (1u << 24) /* <24:24> RW:R:0: */


/*
 * Spare6 status. There is mainly kept as a spare
 */
#define PDSS_STATUS_SPARE6_STATUS                           (1u << 25) /* <25:25> RW:R:0: */


/*
 * Spare7 status. There is mainly kept as a spare
 */
#define PDSS_STATUS_SPARE7_STATUS                           (1u << 26) /* <26:26> RW:R:0: */


/*
 * RX SOP Control for sending GoodCRC Message
 * Hardware will wait for programmable IDLE_COUNTER and then send Good Crc
 * Message.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_ADDRESS(m)             (0x400a00a8 + ((m) * (0x10000)))
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL(m)                     (*(volatile uint32_t *)(0x400a00a8 + ((m) * 0x10000)))
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_DEFAULT                (0x000000c0)

/*
 * Setting this bit will enable the TX block to automatically issue a GoodCRC
 * Message when the received
 * packet with Dafault SOP which has no CRC issue and the packet has been
 * written into SRAM successfully.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Setting this bit will enable the TX block to automatically issue a GoodCRC
 * Message when the received
 * packet with prime SOP which has no CRC issue and the packet has been written
 * into SRAM successfully.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_PRIME_SOP_GOOD_CRC_EN    (1u << 1) /* <1:1> R:RW:0: */


/*
 * Setting this bit will enable the TX block to automatically issue a GoodCRC
 * Message when the received
 * packet with dbl prime SOP which has no CRC issue and the packet has been
 * written into SRAM successfully.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN    (1u << 2) /* <2:2> R:RW:0: */


/*
 * Setting this bit will enable the TX block to automatically issue a GoodCRC
 * Message when the extended received
 * packet with Dafault SOP which has no CRC issue and the packet has been
 * written into SRAM successfully.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_DEFAULT_SOP_GOOD_CRC_EN    (1u << 3) /* <3:3> R:RW:0: */


/*
 * Setting this bit will enable the TX block to automatically issue a GoodCRC
 * Message when the  extended received
 * packet with prime SOP which has no CRC issue and the packet has been written
 * into SRAM successfully.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_PRIME_SOP_GOOD_CRC_EN    (1u << 4) /* <4:4> R:RW:0: */


/*
 * Setting this bit will enable the TX block to automatically issue a GoodCRC
 * Message when the  extended received
 * packet with dbl prime SOP which has no CRC issue and the packet has been
 * written into SRAM successfully.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_EXT_TX_SEND_DBL_PRIME_SOP_GOOD_CRC_EN    (1u << 5) /* <5:5> R:RW:0: */


/*
 * Setting this bit will enable sending GoodCrcMsg for packet with Bad EOP.
 * This should be left to default for normal operation.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_SEND_GOOD_CRC_BAD_EOP    (1u << 6) /* <6:6> R:RW:1: */


/*
 * Setting this bit will enable sending GoodCrcMsg for packet with KCHAR
 * Error. This should be left to default for normal operation.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_SEND_GOOD_CRC_BAD_KCHAR    (1u << 7) /* <7:7> R:RW:1: */


/*
 * Setting this bit will enable sending GoodCrcMsg for packet even when there
 * is RX Sram Over Flow is detected. This should be left to default for normal
 * operation.
 */
#define PDSS_RX_SOP_GOOD_CRC_EN_CTRL_SEND_GOOD_CRC_SRAM_OVERFLOW    (1u << 8) /* <8:8> R:RW:0: */


/*
 * RX Default SOP GoodCRC Control 0
 * There are two purposes for this register:
 * 1. Transmit Path:
 *     When hardware is done transmitting a packet, it will start the RX_CRC_TIMER.
 *     The CRC timer should stop on following conditions:
 *     1. Good CRC Received: With Matching Message ID, Matching Header Sop
 * type.
 *     2. Hard Reset on Receive side
 *     3. Soft Reset on Receive Side:
 *     4. Any other message legal in the current firmware state:
 *     Condition 1, condition 2: These two conditions are automatically taken
 * care by hardware and CRC timer is stopped.
 *     Condition 3, condition 4: Firmware needs to take care of. Firmware
 * can program what legal messages it is expecting in a particular state
 * and hardware will stop its counter.
 *                                         E.G: If firmware wants the transmit
 * logic to stop its CRC counter and not retry the packet on reception of
 * these following messages:
 *                                         • Soft Reset Control Message:
 *  Message Type 1101
 *                                         • Get Source Cap Control Message:
 * Message Type 0111
 *                                         • Vendor Defined Data Message:
 *  Message Type 1111
 *                                         Then in that case firmware will
 * program RX_DEFAULT_SOP_GOODCRC_CTRL to: 8000_2080 (Bit 7th, bit 13th and
 * bit 31st)
 *                                         Hardware will stop the timers
 * on reception of these packets and will also automatically send GoodCRC
 * message to these messages
 *                                         if the proper auto bit is set
 * in RX_SOP_GOOD_CRC_EN_CTRL. Other messages received will be logged in
 * RX_Memory but Good CRC will not be returned or
 *                                         timer will not be stopped.
 * 2. Receive  Path:
 *     Based on one hot encoding of RX_DEFAULT_SOP_GOODCRC_CTRL mapped to
 * message type field in the incoming header,
 *     Good CRC will be returned automatically if the correct RX_SOP_GOOD_CRC_EN_CTRL
 * bit is set.
 */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_0_ADDRESS(m)       (0x400a00ac + ((m) * (0x10000)))
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_0(m)               (*(volatile uint32_t *)(0x400a00ac + ((m) * 0x10000)))
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_0_DEFAULT          (0x00000000)

/*
 * The Message type of a control message is a 5-bit field. Every bit of this
 * register mapps to the control message types. CPU can used this register
 * to select for which one of the the message types of a control message
 * should hardware send a GOODCRC message.
 */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_0_SEND_GOOD_CRC_PER_CTRL_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_0_SEND_GOOD_CRC_PER_CTRL_MSG_TYP_POS    (0)


/*
 * RX Default SOP GoodCRC Control 1
 * There are two purposes for this register:
 * 1. Transmit Path:
 *     When hardware is done transmitting a packet, it will start the RX_CRC_TIMER.
 *     The CRC timer should stop on following conditions:
 *     1. Good CRC Received: With Matching Message ID, Matching Header Sop
 * type.
 *     2. Hard Reset on Receive side
 *     3. Soft Reset on Receive Side:
 *     4. Any other message legal in the current firmware state:
 *     Condition 1, condition 2: These two conditions are automatically taken
 * care by hardware and CRC timer is stopped.
 *     Condition 3, condition 4: Firmware needs to take care of. Firmware
 * can program what legal messages it is expecting in a particular state
 * and hardware will stop its counter.
 *                                         E.G: If firmware wants the transmit
 * logic to stop its CRC counter and not retry the packet on reception of
 * these following messages:
 *                                         • Soft Reset Control Message:
 *  Message Type 1101
 *                                         • Get Source Cap Control Message:
 * Message Type 0111
 *                                         • Vendor Defined Data Message:
 *  Message Type 1111
 *                                         Then in that case firmware will
 * program RX_DEFAULT_SOP_GOODCRC_CTRL to: 8000_2080 (Bit 7th, bit 13th and
 * bit 31st)
 *                                         Hardware will stop the timers
 * on reception of these packets and will also automatically send GoodCRC
 * message to these messages
 *                                         if the proper auto bit is set
 * in RX_SOP_GOOD_CRC_EN_CTRL. Other messages received will be logged in
 * RX_Memory but Good CRC will not be returned or
 *                                         timer will not be stopped.
 * 2. Receive  Path:
 *     Based on one hot encoding of RX_DEFAULT_SOP_GOODCRC_CTRL mapped to
 * message type field in the incoming header,
 *     Good CRC will be returned automatically if the correct RX_SOP_GOOD_CRC_EN_CTRL
 * bit is set.
 */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_1_ADDRESS(m)       (0x400a00b0 + ((m) * (0x10000)))
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_1(m)               (*(volatile uint32_t *)(0x400a00b0 + ((m) * 0x10000)))
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_1_DEFAULT          (0x00000000)

/*
 * The Message type of a data message is a 5-bit field. Every bit of this
 * register mapps to the data message types. CPU can used this register to
 * select for which one of the the message types of a data message should
 * hardware send a GOODCRC message.
 */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_1_SEND_GOOD_CRC_PER_DATA_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_1_SEND_GOOD_CRC_PER_DATA_MSG_TYP_POS    (0)


/*
 * RX Default SOP GoodCRC Control 2
 * There are two purposes for this register:
 * 1. Transmit Path:
 *     When hardware is done transmitting a packet, it will start the RX_CRC_TIMER.
 *     The CRC timer should stop on following conditions:
 *     1. Good CRC Received: With Matching Message ID, Matching Header Sop
 * type.
 *     2. Hard Reset on Receive side
 *     3. Soft Reset on Receive Side:
 *     4. Any other message legal in the current firmware state:
 *     Condition 1, condition 2: These two conditions are automatically taken
 * care by hardware and CRC timer is stopped.
 *     Condition 3, condition 4: Firmware needs to take care of. Firmware
 * can program what legal messages it is expecting in a particular state
 * and hardware will stop its counter.
 *                                         E.G: If firmware wants the transmit
 * logic to stop its CRC counter and not retry the packet on reception of
 * these following messages:
 *                                         • Soft Reset Control Message:
 *  Message Type 1101
 *                                         • Get Source Cap Control Message:
 * Message Type 0111
 *                                         • Vendor Defined Data Message:
 *  Message Type 1111
 *                                         Then in that case firmware will
 * program RX_DEFAULT_SOP_GOODCRC_CTRL to: 8000_2080 (Bit 7th, bit 13th and
 * bit 31st)
 *                                         Hardware will stop the timers
 * on reception of these packets and will also automatically send GoodCRC
 * message to these messages
 *                                         if the proper auto bit is set
 * in RX_SOP_GOOD_CRC_EN_CTRL. Other messages received will be logged in
 * RX_Memory but Good CRC will not be returned or
 *                                         timer will not be stopped.
 * 2. Receive  Path:
 *     Based on one hot encoding of RX_DEFAULT_SOP_GOODCRC_CTRL mapped to
 * message type field in the incoming header,
 *     Good CRC will be returned automatically if the correct RX_SOP_GOOD_CRC_EN_CTRL
 * bit is set.
 */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_2_ADDRESS(m)       (0x400a00b4 + ((m) * (0x10000)))
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_2(m)               (*(volatile uint32_t *)(0x400a00b4 + ((m) * 0x10000)))
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_2_DEFAULT          (0x00000000)

/*
 * Only for extended data.
 * The Message type of a data message is a 5-bit field. Every bit of this
 * register mapps to the data message types. CPU can used this register to
 * select for which one of the the message types of a data message should
 * hardware send a GOODCRC message.
 */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_2_SEND_GOOD_CRC_PER_DATA_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_DEFAULT_SOP_GOODCRC_CTRL_2_SEND_GOOD_CRC_PER_DATA_MSG_TYP_POS    (0)


/*
 * RX Prime SOP GoodCRC Control 0
 */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_0_ADDRESS(m)         (0x400a00b8 + ((m) * (0x10000)))
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_0(m)                 (*(volatile uint32_t *)(0x400a00b8 + ((m) * 0x10000)))
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_0_DEFAULT            (0x00000000)

/*
 * The Message type of a control message is a 5-bit field. Every bit of this
 * register mapps to the control
 * message types. CPU can used this register to select for which one of the
 * the message types of a control message should hardware send a GOODCRC
 * message.
 */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_0_SEND_GOOD_CRC_PER_CTRL_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_0_SEND_GOOD_CRC_PER_CTRL_MSG_TYP_POS    (0)


/*
 * RX Prime SOP GoodCRC Control 1
 */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_1_ADDRESS(m)         (0x400a00bc + ((m) * (0x10000)))
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_1(m)                 (*(volatile uint32_t *)(0x400a00bc + ((m) * 0x10000)))
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_1_DEFAULT            (0x00000000)

/*
 * The Message type of a data message is a 5-bit field. Every bit of this
 * register mapps to the data
 * message types. CPU can used this register to select for which one of the
 * the message types of a data message should hardware send a GOODCRC message.
 */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_1_SEND_GOOD_CRC_PER_DATA_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_1_SEND_GOOD_CRC_PER_DATA_MSG_TYP_POS    (0)


/*
 * RX Prime SOP GoodCRC Control 2
 */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_2_ADDRESS(m)         (0x400a00c0 + ((m) * (0x10000)))
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_2(m)                 (*(volatile uint32_t *)(0x400a00c0 + ((m) * 0x10000)))
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_2_DEFAULT            (0x00000000)

/*
 * Only for extended data.
 * The Message type of a data message is a 5-bit field. Every bit of this
 * register mapps to the data
 * message types. CPU can used this register to select for which one of the
 * the message types of a data message should hardware send a GOODCRC message.
 */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_2_SEND_GOOD_CRC_PER_DATA_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_PRIME_SOP_GOODCRC_CTRL_2_SEND_GOOD_CRC_PER_DATA_MSG_TYP_POS    (0)


/*
 * RX DBL Prime SOP GoodCRC Control 0
 */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_0_ADDRESS(m)     (0x400a00c4 + ((m) * (0x10000)))
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_0(m)             (*(volatile uint32_t *)(0x400a00c4 + ((m) * 0x10000)))
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_0_DEFAULT        (0x00000000)

/*
 * The Message type of a control message is a 5-bit field. Every bit of this
 * register mapps to the control
 * message types. CPU can used this register to select for which one of the
 * the message types of a control message should hardware send a GOODCRC
 * message.
 */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_0_SEND_GOOD_CRC_PER_CTRL_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_0_SEND_GOOD_CRC_PER_CTRL_MSG_TYP_POS    (0)


/*
 * RX DBL Prime SOP GoodCRC Control 1
 */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_1_ADDRESS(m)     (0x400a00c8 + ((m) * (0x10000)))
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_1(m)             (*(volatile uint32_t *)(0x400a00c8 + ((m) * 0x10000)))
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_1_DEFAULT        (0x00000000)

/*
 * The Message type of a data message is a 5-bit field. Every bit of this
 * register mapps to the data
 * message types. CPU can used this register to select for which one of the
 * the message types of a data message should hardware send a GOODCRC message.
 */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_1_SEND_GOOD_CRC_PER_DATA_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_1_SEND_GOOD_CRC_PER_DATA_MSG_TYP_POS    (0)


/*
 * RX DBL Prime SOP GoodCRC Control 2
 */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_2_ADDRESS(m)     (0x400a00cc + ((m) * (0x10000)))
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_2(m)             (*(volatile uint32_t *)(0x400a00cc + ((m) * 0x10000)))
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_2_DEFAULT        (0x00000000)

/*
 * Only for extended data.
 * The Message type of a data message is a 5-bit field. Every bit of this
 * register mapps to the data
 * message types. CPU can used this register to select for which one of the
 * the message types of a data message should hardware send a GOODCRC message.
 */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_2_SEND_GOOD_CRC_PER_DATA_MSG_TYP_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_DBL_PRIME_SOP_GOODCRC_CTRL_2_SEND_GOOD_CRC_PER_DATA_MSG_TYP_POS    (0)


/*
 * RX Excepted good CRC message to stop the CRC timers
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_ADDRESS(m)               (0x400a00d0 + ((m) * (0x10000)))
#define PDSS_RX_EXPECT_GOODCRC_MSG(m)                       (*(volatile uint32_t *)(0x400a00d0 + ((m) * 0x10000)))
#define PDSS_RX_EXPECT_GOODCRC_MSG_DEFAULT                  (0x00000001)

/*
 * The expected GoodCRC Messgae Header on the RX side. The expected message
 * ID is handled by Firmware. The DEBUG_CC_2.EXPECTED_HEADER_MASK can be
 * used to mask comparing individual bits.
 * The CRC timer will stop on:
 * 1: On the reception of GoodCRC Messegae with good CRC32 where its header
 * matches with this register AND
 * 2: The SOP of the GoodCRC Messgae matches with the EXPECTED_SOP.
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_HEADER_MASK     (0x0000ffff) /* <0:15> R:RW:1: */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_HEADER_POS      (0)


/*
 * The expected SOP of GoodCRC Messgae on the RX side.
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_SOP_MASK        (0x00070000) /* <16:18> R:RW:0: */
#define PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_SOP_POS         (16)


/*
 * FW should toggle this bit before commiting a new packet to be transferred
 * from TX_MEM.
 * 0: Don’t disable the RX CRC count down
 * 1: Disable the RX CRC count down.
 *    FW can disable the RX CRC timer whenever it detects the required condition.
 */
#define PDSS_RX_EXPECT_GOODCRC_MSG_DISABLE_RX_CRC_TIMER     (1u << 19) /* <19:19> R:RW:0: */


/*
 * The 2-Byte Header of the received GoodCRC Message
 */
#define PDSS_RX_GOODCRC_MSG_ADDRESS(m)                      (0x400a00d4 + ((m) * (0x10000)))
#define PDSS_RX_GOODCRC_MSG(m)                              (*(volatile uint32_t *)(0x400a00d4 + ((m) * 0x10000)))
#define PDSS_RX_GOODCRC_MSG_DEFAULT                         (0x00000000)

/*
 * The INTR.RCV_GOODCRC_MSG_COMPLETE interrupt indicates the 2-Byte header
 * for GoodCRC message is received and stored in this registers.
 * GOODCRC_MSG_SOP_TYPE_DETECTED contains the SOP type for the GoodCRC MSG.
 * At the start of every packet, INTR.RCV_GOODCRC_MSG_COMPLETE status is
 * evaluated, if its reset, then only a new packet will be written else new
 * packet will be dropped.
 */
#define PDSS_RX_GOODCRC_MSG_HEADER_MASK                     (0x0000ffff) /* <0:15> RW:R:0: */
#define PDSS_RX_GOODCRC_MSG_HEADER_POS                      (0)


/*
 * The Receive C-Connect registers
 */
#define PDSS_RX_CC_ADDRESS(m)                               (0x400a00d8 + ((m) * (0x10000)))
#define PDSS_RX_CC(m)                                       (*(volatile uint32_t *)(0x400a00d8 + ((m) * 0x10000)))
#define PDSS_RX_CC_DEFAULT                                  (0x00000580)

/*
 * This value is internally multiplied by 16.
 * The 16X value when multiplied by the period of CLK_RX defines the maximum
 * clock period.
 * This value is used to cause the RX state machine to return to idle state
 * if no transitions are detected.
 * For 12 Mhz, the count should be 20 Decimal ( becomes approx. 26usec)
 * For 24 Mhz, the count should be 40 Decimal
 */
#define PDSS_RX_CC_RX_CNT_MAX_MASK                          (0x000000ff) /* <0:7> R:RW:128: */
#define PDSS_RX_CC_RX_CNT_MAX_POS                           (0)


/*
 * Hardware calculates the UI by averaging the pre-programmed number of Preamble
 * Bits in the DEBUG_CC_1 register.
 * Once UI is calculated, hardware uses a sampling point percentage to know
 * whether incoming BMC pattern is a 0 or 1.
 * The Hardware automatically calculates the 75% location with respect to
 * UI.
 * The value of this register determines the sampling point by subtracting
 * number of RX_CLK from 75% UI. The value of 0 will mean that hardware samples
 * at 75% of UI and 1 will mean hardware will sample at around 72.5% of UI
 * for 12Mhz clock and 73.75 for 24Mhz RX_CLK
 * (RX_CLK period/UI will determine the step size percentage)
 * For 12 Mhz CLK_RX operations, set the value to 0x2 which means that hardware
 * will sample incoming BMC pattern at around 70%.
 * For 24Mhz CLK_RX operations, set the value to 0x4 which means that hardware
 * will sample incoming BMC pattern at around 70%
 */
#define PDSS_RX_CC_RX_UI_BOUNDARY_DELTA_MASK                (0x00003f00) /* <8:13> R:RW:5: */
#define PDSS_RX_CC_RX_UI_BOUNDARY_DELTA_POS                 (8)


/*
 * Spare
 */
#define PDSS_RX_CC_DELAY_HALF                               (1u << 14) /* <14:14> R:RW:0: */


/*
 * Spare
 */
#define PDSS_RX_CC_DELAY_VALUE_MASK                         (0x00078000) /* <15:18> R:RW:0: */
#define PDSS_RX_CC_DELAY_VALUE_POS                          (15)


/*
 * Spare
 */
#define PDSS_RX_CC_DELAY_WIDEN_HIGH                         (1u << 19) /* <19:19> R:RW:0: */


/*
 * Spare
 */
#define PDSS_RX_CC_DELAY_AUTO                               (1u << 20) /* <20:20> R:RW:0: */


/*
 * The Receive C-Connect registers
 */
#define PDSS_RX_CC2_ADDRESS(m)                              (0x400a00e0 + ((m) * (0x10000)))
#define PDSS_RX_CC2(m)                                      (*(volatile uint32_t *)(0x400a00e0 + ((m) * 0x10000)))
#define PDSS_RX_CC2_DEFAULT                                 (0x00043008)

/*
 * This is a debug feature.
 * It defines minimum pulse value. If a pulse for current packet is calculated
 * to be smaller than this value, an interrupt will be issued.  MIN_PULSE_VIOLATED
 * will be set.  This check will be made after number of transitions specified
 * by NUM_TRANS_AVG are complete.
 */
#define PDSS_RX_CC2_MIN_PULSE_VALUE_MASK                    (0x000000ff) /* <0:7> R:RW:8: */
#define PDSS_RX_CC2_MIN_PULSE_VALUE_POS                     (0)


/*
 * This is a debug feature. It defines maximum pulse value. If a pulse for
 * current packet is calculated to be larger than this value, an interrupt
 * will be issued.  MAX_PULSE_VIOLATED will be set.  This check will be made
 * after number of transitions specified by NUM_TRANS_AVG are complete.
 */
#define PDSS_RX_CC2_MAX_PULSE_VALUE_MASK                    (0x0000ff00) /* <8:15> R:RW:48: */
#define PDSS_RX_CC2_MAX_PULSE_VALUE_POS                     (8)


/*
 * Number of RX_CC transitions before RX_VALID output is raised.
 */
#define PDSS_RX_CC2_DELAY_VALID_COUNT_MASK                  (0x000f0000) /* <16:19> R:RW:4: */
#define PDSS_RX_CC2_DELAY_VALID_COUNT_POS                   (16)


/*
 * Receive SOPs and RSTs order set control
 */
#define PDSS_RX_ORDER_SET_CTRL_ADDRESS(m)                   (0x400a00e4 + ((m) * (0x10000)))
#define PDSS_RX_ORDER_SET_CTRL(m)                           (*(volatile uint32_t *)(0x400a00e4 + ((m) * 0x10000)))
#define PDSS_RX_ORDER_SET_CTRL_DEFAULT                      (0x00006103)

/*
 * This register is used for SOP, SOP',SOP'", DEBUG SOP', DEBUG SOP" and
 * RX_RESERVED1/2_ORDER_SET(if configured for SOP) oder set detection. It
 * is recommended that CPU program this register to 1 ( 4 out of 4 option).
 * 0: Compare 3 out of 4 order sets
 * 1: Compare 4 out of 4 order sets
 */
#define PDSS_RX_ORDER_SET_CTRL_SOP_CMP_OPT                  (1u << 0) /* <0:0> R:RW:1: */


/*
 * This register is used for Cable RST, Hard RST and RX_RESERVED1/2_ORDER_SET(if
 * configure for RST) order set detection.
 * It is recommended that CPU program this register to 1 ( 4 out of 4 option).
 * 0: Compare 3 out of 4 order sets
 * 1: Compare 4 out of 4 order sets
 */
#define PDSS_RX_ORDER_SET_CTRL_RST_CMP_OPT                  (1u << 1) /* <1:1> R:RW:1: */


/*
 * This register is used to enable/disdable 4-bit preamble detection for
 * SOP detection.
 * 0: SOP Detection:                              SOP logic detection
 * 1: SOP detection: Preamble(4-bit)+ SOP logic detection
 */
#define PDSS_RX_ORDER_SET_CTRL_PREAMBLE_SOP_EN              (1u << 2) /* <2:2> R:RW:0: */


/*
 * This register is used to enable/disdable 4-bit preamble detection for
 * RST detection.
 * 0: RST Detection:                              RST logic detection
 * 1: RST detection: Preamble(4-bit)+ RST logic detection
 */
#define PDSS_RX_ORDER_SET_CTRL_PREAMBLE_RST_EN              (1u << 3) /* <3:3> R:RW:0: */


/*
 * Host Mode: F/W can enable SOP, SOP’, SOP” and Hard Reset Detection.
 * Device Mode: F/W should enable only SOP and Hard Reset Detection.
 * Cable Mode: Either SOP’ or SOP” based on VCONN, Hard Reset and Cable Reset
 * should be enabled.
 */
#define PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK              (0x0007ff00) /* <8:18> R:RW:97: */
#define PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_POS               (8)


/*
 * Receive Reserved1 order set
 */
#define PDSS_RX_RESERVED1_ORDER_SET_ADDRESS(m)              (0x400a00e8 + ((m) * (0x10000)))
#define PDSS_RX_RESERVED1_ORDER_SET(m)                      (*(volatile uint32_t *)(0x400a00e8 + ((m) * 0x10000)))
#define PDSS_RX_RESERVED1_ORDER_SET_DEFAULT                 (0x00000000)

/*
 * Receive Reserved1 order set
 */
#define PDSS_RX_RESERVED1_ORDER_SET_RX_RESERVED1_OS_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_RX_RESERVED1_ORDER_SET_RX_RESERVED1_OS_POS     (0)


/*
 * Receive Reserved2 order set
 */
#define PDSS_RX_RESERVED2_ORDER_SET_ADDRESS(m)              (0x400a00ec + ((m) * (0x10000)))
#define PDSS_RX_RESERVED2_ORDER_SET(m)                      (*(volatile uint32_t *)(0x400a00ec + ((m) * 0x10000)))
#define PDSS_RX_RESERVED2_ORDER_SET_DEFAULT                 (0x00000000)

/*
 * Receive Reserved2  order set
 */
#define PDSS_RX_RESERVED2_ORDER_SET_RX_RESERVED2_OS_MASK    (0x000fffff) /* <0:19> R:RW:0: */
#define PDSS_RX_RESERVED2_ORDER_SET_RX_RESERVED2_OS_POS     (0)


/*
 * Transmit GoodCrc Message order set
 */
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_ADDRESS(m)            (0x400a00f0 + ((m) * (0x10000)))
#define PDSS_TX_GOODCRC_MSG_ORDER_SET(m)                    (*(volatile uint32_t *)(0x400a00f0 + ((m) * 0x10000)))
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_DEFAULT               (0x00000000)

/*
 * [15:0]   Can also be used for Transmiting GoodCRC Message to SOP'
 * [31:16] Can also be used for Transmiting GoodCRC Message to SOP"
 * This register constains the Transmit GoodCRC Message Header except the
 * MessageID which Is handled by Hardware.
 * [11:9] Message ID (This is handled by HardWare) for SOP'
 * [27:25] Message ID (This is handled by HardWare) for SOP"
 */
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_TX_GOODCRC_OS_MASK    (0xffffffff) /* <0:31> R:RW:0: */
#define PDSS_TX_GOODCRC_MSG_ORDER_SET_TX_GOODCRC_OS_POS     (0)


/*
 * TX Control
 */
#define PDSS_TX_CTRL_ADDRESS(m)                             (0x400a00f4 + ((m) * (0x10000)))
#define PDSS_TX_CTRL(m)                                     (*(volatile uint32_t *)(0x400a00f4 + ((m) * 0x10000)))
#define PDSS_TX_CTRL_DEFAULT                                (0x8f200041)

/*
 * For SOP Only.
 * This register constains the Transmit GoodCRC Message Header except the
 * MessageID which Is handled by Hardware.
 * [11:9] Message ID (This is handled by HardWare)
 */
#define PDSS_TX_CTRL_GOODCRC_MSG_BITS_MASK                  (0x0000ffff) /* <0:15> R:RW:65: */
#define PDSS_TX_CTRL_GOODCRC_MSG_BITS_POS                   (0)


/*
 * Setting the EN_TX_BIST_CM2 to "1" will start the transmision of Bist Carrier
 * Mode 2 pattern.
 * FW must manually set TX_CTRL.TX_REG_EN to "1" before setting this register(EN_TX_BIST_CM2)
 * The TX_GO register is not required to be set for this mode.
 * FW should wait for TX_REG_TIMMER (50usec) before setting this bit.
 */
#define PDSS_TX_CTRL_EN_TX_BIST_CM2                         (1u << 16) /* <16:16> R:RW:0: */


/*
 * TX_GO causes a packet to be sent. FW can send GoodCrcMsg by storing it
 * in the TX SRAM and use TX_GO to send it.
 * Writing a 1 to this bit to cause the message stored in the SRAM Memory
 * to be sent.  Hardware clears this bit once the command is accepted and
 * processing has begun.
 * If TX_GO is set and there is a ongoing receive packet, the TX packet wont
 * be sent and the COLLISION_TYPE1
 * interrupt will be set. In this case, hardware clears the TX_GO.
 * Before setting this FW should check:
 * INTR0->RX_GOOD_PKT && STATUS->DATA_VALID == 0
 */
#define PDSS_TX_CTRL_TX_GO                                  (1u << 17) /* <17:17> RW1C:RW:0: */


/*
 * Send a Reset over the link. Write a 1 to this bit to cause the transmitter
 * to send a Hard Reset or Cable Reset(TX_HARD_CABLE_ORDER_SET register)
 * over the link. Hardware clears this bit once the command is accepted and
 * processing has begun.
 * If TX_SEND_RST is set and there is a ongoing receive packet, the Reset
 * Sequqnce wont be sent and the COLLISION_TYPE4 interrupt will be set. In
 * this case, hardware clears the TX_SEND_RST.
 * Before settting this FW should check:
 * INTR0->RX_GOOD_PKT && STATUS->DATA_VALID == 0
 */
#define PDSS_TX_CTRL_TX_SEND_RST                            (1u << 18) /* <18:18> RW1C:RW:0: */


/*
 * Enable transmit retry. Hardware clears this bit once the command is accepted
 * and processing has begun.
 * CPU should increment the retry counter in firmware once TX_PACKET_DONE
 * interrupt is detected by CPU.
 * The following operation is recommneded to FW:
 *    • FW maintains the retry counter
 *    • FW writes a packet in TX_Memory.
 *    • FW checks its retry counter and if its >0 , sets the retry enable
 * bit.
 *    • FW sets the TX_GO.
 *    • HW sends the packet and starts CRC_Timer if enabled.
 *    • On Expiry of CRC_timer, HW retries the packet if retry enable bit
 * was set and then HW auto clears that bit and generates the
 *      TX_RETRY_ENABLE_CLRD interrupt
 *    • HW will start the CRC_timer again.
 *    • FW in parallel would have received the CRC_TIMER expiry interrupt…
 * FW will decrement its retry counter and if retry
 *      counter is still >0, then it will set the retry enable again
 *
 * The usage model is as follows for setting the retry enable bit:
 * 1. Firmware gets the CRC receive timeout interrupt. After getting this
 * interrupt, firmware can either do step 2 or step 3.
 * 2. Firmware should wait for 75 usec after the timeout interrupt and sample
 * retry bit. Retry bit should be zero at this point indicating retry packet
 * has started transmitting or if retry bit is still one, then it could be
 * that collission has been detected on the bus. If Retry bit is 0, firmware
 * can set the retry bit firmware will have approximately Packet size  time
 * + crc timer (Minimum of 1.5 msec)
 * 3. Firmware doesnt want to put delay of 75 usec, it can instead use TX_RETRY_ENABLE_CLRD
 * interrupt.
 */
#define PDSS_TX_CTRL_TX_RETRY_ENABLE                        (1u << 19) /* <19:19> RW1C:RW:0: */


/*
 * Enable the transmitter regulator.
 * This should be set to "1" for BIST mode other wise hardware automatically
 * takes enabling the regulator when TX_REG_CFG=1
 */
#define PDSS_TX_CTRL_TX_REG_EN                              (1u << 20) /* <20:20> R:RW:0: */


/*
 * 0: Hardware controlling of TX regulator Enable is disabled. CPU can fully
 * control the TX regulator enable by using TX_REG_EN.
 * 1: Hardware controlling the TX regulator Enable is enabled. In this case,
 * CPU can only set the regulator enable to one by setting TX_REG_EN
 */
#define PDSS_TX_CTRL_TX_REG_CFG                             (1u << 21) /* <21:21> R:RW:1: */


/*
 * The time needed to enable the TX regulator before transmission.
 * The counter runs on CLK_TX_HALF(600Khz)
 */
#define PDSS_TX_CTRL_TX_REG_TIMER_MASK                      (0x3f000000) /* <24:29> R:RW:15: */
#define PDSS_TX_CTRL_TX_REG_TIMER_POS                       (24)


/*
 * Setting this bit will enable corrupting the TX CRC when there is TX Sram
 * Under Flow is detected. This should be left to default for normal operation.
 */
#define PDSS_TX_CTRL_TX_CORRUPT_CRC_ON_UNDER_FLOW           (1u << 31) /* <31:31> R:RW:1: */


/*
 * Transmit SOP order set
 */
#define PDSS_TX_SOP_ORDER_SET_ADDRESS(m)                    (0x400a00f8 + ((m) * (0x10000)))
#define PDSS_TX_SOP_ORDER_SET(m)                            (*(volatile uint32_t *)(0x400a00f8 + ((m) * 0x10000)))
#define PDSS_TX_SOP_ORDER_SET_DEFAULT                       (0x0008e318)

/*
 * Transmit SOP order Set use in transmit except GoodCrcMsg
 */
#define PDSS_TX_SOP_ORDER_SET_TX_SOP_OS_MASK                (0x000fffff) /* <0:19> R:RW:582424: */
#define PDSS_TX_SOP_ORDER_SET_TX_SOP_OS_POS                 (0)


/*
 * Transmit Hard/Cable reset order set
 */
#define PDSS_TX_HARD_CABLE_ORDER_SET_ADDRESS(m)             (0x400a00fc + ((m) * (0x10000)))
#define PDSS_TX_HARD_CABLE_ORDER_SET(m)                     (*(volatile uint32_t *)(0x400a00fc + ((m) * 0x10000)))
#define PDSS_TX_HARD_CABLE_ORDER_SET_DEFAULT                (0x000e7393)

/*
 * Transmit Hard/Cable Reset order Set.
 * Default: Hard Reset Value 0xE7393
 * Cable Reset Value: 0xE0F8C
 */
#define PDSS_TX_HARD_CABLE_ORDER_SET_TX_RESET_OS_MASK       (0x000fffff) /* <0:19> R:RW:947091: */
#define PDSS_TX_HARD_CABLE_ORDER_SET_TX_RESET_OS_POS        (0)


/*
 * The CRC timer counters configuration
 * Counters used for the timmers needed by this IP
 */
#define PDSS_CRC_COUNTER_ADDRESS(m)                         (0x400a0100 + ((m) * (0x10000)))
#define PDSS_CRC_COUNTER(m)                                 (*(volatile uint32_t *)(0x400a0100 + ((m) * 0x10000)))
#define PDSS_CRC_COUNTER_DEFAULT                            (0x00000000)

/*
 * This counter will run on TX_CLK/2 (PD bit period) clock.
 * This counter is used for RCReceiveTimer(tReceive)/BISTReceiveErrorTimertBistReceive
 * 0: Counter is disabled. Hardware will NOT wait for the CRC_COUNTER to
 * expire.
 * Other: Hardware will wait for the CRC_COUNTER to expire.
 * Once the CRC_COUNTER reaches zero and no Valid CoodCRC message is not
 * received, The CRC_RX_TIMER_EXP interrupt gets set.
 *
 * If TX_BYPASS_EN = 1:
 *         [15:0]: Total number of bits
 */
#define PDSS_CRC_COUNTER_CRC_COUNTER_MASK                   (0x0000ffff) /* <0:15> R:RW:0: */
#define PDSS_CRC_COUNTER_CRC_COUNTER_POS                    (0)


/*
 * The Inter Packet counters
 * Counters used for IDLE/IFG and  by this IP
 * All the timers/counters have a resolution of 1 UI (Unit Interval) of a
 * Bit. If transmit rate is 300Khz, then each count will tick for 3.33 usec.
 */
#define PDSS_INTER_PACKET_COUNTER_ADDRESS(m)                (0x400a0104 + ((m) * (0x10000)))
#define PDSS_INTER_PACKET_COUNTER(m)                        (*(volatile uint32_t *)(0x400a0104 + ((m) * 0x10000)))
#define PDSS_INTER_PACKET_COUNTER_DEFAULT                   (0x01004008)

/*
 * USED FOR TX->TX
 * This register is used by DUT to create gap between two back to back transmit
 * packets. For example: For a DFP application if DFP wants to send Hard
 * Reset after a valid packet, then this register could be used for complying
 * with Inter-Packet Gap of 25 usec specified by spec. In cable application,
 * after sending Good CRC handshake for request from DUT, this register could
 * be used to comply with 750usec requirement of cable response after sending
 * Good CRC pkt.
 */
#define PDSS_INTER_PACKET_COUNTER_BUS_IDLE_CNT_MASK         (0x000007ff) /* <0:10> R:RW:8: */
#define PDSS_INTER_PACKET_COUNTER_BUS_IDLE_CNT_POS          (0)


/*
 * RX -> AUTO_GOODCRC_RESPONSE
 * This counter specifies how long the HW should wait after the end of RX
 * packet to send GoodCRC message. This can be used to comply with interpacket
 * gap of 25usec.
 * 0: Counter is disabled. Hardware will issue goodcrc message if needed
 * after end of the RX Packet
 *
 * For CCG2 ** Silicon, this can stay at spec value of 25usec,
 * For CCG2 *A Silicon, this need to be programmed at minimum of 150usec
 * (Value 50). The new noise circuit gets settled at 75usec and then RX_VALID
 * logic will need another 75usec to declare IDLE on the bus. This value
 * is worst case number and can be used for CCG2 ** Silicon as well.
 * For CCG3/4: The value can be as low as 120usec.
 */
#define PDSS_INTER_PACKET_COUNTER_IDLE_COUNTER_MASK         (0x001ff800) /* <11:20> R:RW:8: */
#define PDSS_INTER_PACKET_COUNTER_IDLE_COUNTER_POS          (11)


/*
 * END OF ANY RX ON CC-LINE
 * On any RX Packet, this counter will start at end of RX Packet and will
 * reset on Start of RX Packet. This register can be again used to comply
 * with Interpacket Gap (25 usec + end of IDLE detection(12 usec)).
 * CPU after seeing no activity on the bus can immedeately set the TX_GO/TX_SEND_RST
 * bit and this register will make sure that we don't violate any interpacket
 * gap requirement.
 * 0: Counter is disabled.
 */
#define PDSS_INTER_PACKET_COUNTER_IFG_COUNTER_MASK          (0xffe00000) /* <21:31> R:RW:8: */
#define PDSS_INTER_PACKET_COUNTER_IFG_COUNTER_POS           (21)


/*
 * The trigger enable registers
 * The tr_out[4:0] pins of this IP will be connected to the tr_in pin of
 * m0s8tcpwm_ver2 IP at the full chip. The mapping of the these signals is
 * SoC depended and it is defined
 * in SAS.
 */
#define PDSS_TIMER_TRIGGER_ADDRESS(m)                       (0x400a0108 + ((m) * (0x10000)))
#define PDSS_TIMER_TRIGGER(m)                               (*(volatile uint32_t *)(0x400a0108 + ((m) * 0x10000)))
#define PDSS_TIMER_TRIGGER_DEFAULT                          (0x00000000)

/*
 * 1: The tr_out[0] pin of the IP will toggle on the transmission of the
 * last Bit Of EOP.
 * 0: The toggling of the tr_out[0] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER0                      (1u << 0) /* <0:0> R:RW:0: */


/*
 * 1: The tr_out[1] pin of the IP will toggle on the reception of EOP for
 * any message with Valid CRC (Includes Good Crc Message)
 * 0: The toggling of the tr_out[1] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER1                      (1u << 1) /* <1:1> R:RW:0: */


/*
 * 1: The tr_out[2] pin of the IP will toggle on the reception of EOP for
 * any message with Valid CRC (Excludes the message types specified in the
 * RX_ORDER_SET_CTRL register)
 * 0: The toggling of the tr_out[2] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER2                      (1u << 2) /* <2:2> R:RW:0: */


/*
 * 1: The tr_out[3] pin of the IP will toggle on the reception of EOP for
 * any message (Example: RX Hard Reset/ BIST)
 * 0: The toggling of the tr_out[3] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER3                      (1u << 3) /* <3:3> R:RW:0: */


/*
 * 1: The tr_out[4] pin of the IP will toggle on the transmission of the
 * first Bit of SOP (Additional trigger for starting some different timer).
 * 0: The toggling of the tr_out[4] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER4                      (1u << 4) /* <4:4> R:RW:0: */


/*
 * 1: The tr_out[5] pin of the IP will toggle on rx_valid
 * 0: The toggling of the tr_out[5] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER5                      (1u << 5) /* <5:5> R:RW:0: */


/*
 * 1: The tr_out[6] pin of the IP will toggle on the reception of valid SOP.
 * 0: The toggling of the tr_out[6] pin of the IP is disabled.
 */
#define PDSS_TIMER_TRIGGER_EN_TRIGGER6                      (1u << 6) /* <6:6> R:RW:0: */


/*
 * Interrupt DDFT Selections
 */
#define PDSS_INTR_DDFT_MUX_ADDRESS(m)                       (0x400a010c + ((m) * (0x10000)))
#define PDSS_INTR_DDFT_MUX(m)                               (*(volatile uint32_t *)(0x400a010c + ((m) * 0x10000)))
#define PDSS_INTR_DDFT_MUX_DEFAULT                          (0x00000000)

/*
 * 70      intr_cause_vswap_vbus_less_5_done
 * 69      intr_cause_swapt_command_done_done
 * 68      intr_cause_swap_queue_done
 * 67      intr_cause_swap_unstable_done
 * 66      intr_cause_swap_unpluged_done
 * 65      intr_cause_swap_pluged_done
 * 64      intr_cause_swap_irq_done
 * 63      intr_cause_ngdo_c_spacing_done_done
 * 62      intr_cause_ngdo_p_spacing_done_done
 * 61      1'b0
 * 60      1'b0
 * 59      intr_cause_chunk_det_done
 * 58      intr_cause_tx_sram_under_flow_done
 * 57      intr_cause_rx_sram_over_flow_done
 * 56      intr_cause_spare5_done
 * 55      intr_cause_spare4_done
 * 54      intr_cause_spare3_done
 * 53      intr_cause_extended_msg_det_done
 * 52      intr_cause_hpdt_command_done_done
 * 51      intr_cause_hpd_queue_done
 * 50      intr_cause_hpd_unstable_done
 * 49      intr_cause_hpd_unpluged_done
 * 48      intr_cause_hpd_pluged_done
 * 47      intr_cause_hpd_irq_done
 * 46      intr_cause_ui_cal_done_done
 * 45      intr_cause_pulse_min_violated_done
 * 44      intr_cause_pulse_max_violated_done
 * 43      intr1_cause_spare7_done
 * 42      intr1_cause_spare6_done
 * 41      intr1_cause_hpdin_changed_done
 * 40      intr1_cause_cmp_out_done[1]
 * 39      intr1_cause_cmp_out_done[0]
 * 38      intr1_cause_v5v_done
 * 37      intr1_cause_vcmp_dn_done
 * 36      intr1_cause_vcmp_up_done
 * 35      intr1_cause_vcmp_la_done
 * 34      intr1_cause_cc2_changed_done
 * 33      intr1_cause_cc1_changed_done
 * 32      intr1_cause_vconn2_changed_done
 * 31      intr1_cause_vconn1_changed_done
 * 30      intr_cause_cmp_out_det_done[1]
 * 29      intr_cause_sar_done[1]
 * 28      intr_cause_rx_state_idle_done
 * 27      intr_cause_tx_state_idle_done
 * 26      intr_cause_tx_regulator_enabled_done
 * 25      intr_cause_tx_cc_data_oen_deassert_done
 * 24      intr_cause_tx_cc_data_oen_assert_done
 * 23      intr_cause_kchar_error_done
 * 22      intr_cause_tx_retry_enable_clrd_done
 * 21      intr_cause_rx_sram_half_end_done
 * 20      intr_cause_tx_sram_half_end_done
 * 19      intr_cause_cmp_out_det_done[0]
 * 18      intr_cause_collision_type4_done
 * 17      intr_cause_collision_type3_done
 * 16      intr_cause_collision_type2_done
 * 15      intr_cause_collision_type1_done
 * 14      intr_cause_crc_rx_timer_exp_done
 * 13      intr_cause_cc_no_valid_data_detected_done
 * 12      intr_cause_cc_valid_data_detected_done
 * 11      intr_cause_tx_goodcrc_msg_done
 * 10      intr_cause_sar_done[0]
 * 9       intr_cause_rcv_rst_done
 * 8       intr_cause_tx_hard_rst_done
 * 7       intr_cause_tx_packet_done
 * 6       intr_cause_rx_over_run_done
 * 5       intr_cause_eop_error_done
 * 4       intr_cause_rcv_expt_goodcrc_msg_complete_done
 * 3       intr_cause_rcv_goodcrc_msg_complete_done
 * 2       intr_cause_rx_sop_done
 * 1       intr_cause_rcv_bad_packet_complete_done
 * 0       intr_cause_rcv_good_packet_complete_done
 */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT0_SEL_MASK              (0x0000007f) /* <0:6> R:RW:0: */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT0_SEL_POS               (0)


/*
 * 70      intr_cause_vswap_vbus_less_5_done
 * 69      intr_cause_swapt_command_done_done
 * 68      intr_cause_swap_queue_done
 * 67      intr_cause_swap_unstable_done
 * 66      intr_cause_swap_unpluged_done
 * 65      intr_cause_swap_pluged_done
 * 64      intr_cause_swap_irq_done
 * 63      intr_cause_ngdo_c_spacing_done_done
 * 62      intr_cause_ngdo_p_spacing_done_done
 * 61      1'b0
 * 60      1'b0
 * 59      intr_cause_chunk_det_done
 * 58      intr_cause_tx_sram_under_flow_done
 * 57      intr_cause_rx_sram_over_flow_done
 * 56      intr_cause_spare5_done
 * 55      intr_cause_spare4_done
 * 54      intr_cause_spare3_done
 * 53      intr_cause_extended_msg_det_done
 * 52      intr_cause_hpdt_command_done_done
 * 51      intr_cause_hpd_queue_done
 * 50      intr_cause_hpd_unstable_done
 * 49      intr_cause_hpd_unpluged_done
 * 48      intr_cause_hpd_pluged_done
 * 47      intr_cause_hpd_irq_done
 * 46      intr_cause_ui_cal_done_done
 * 45      intr_cause_pulse_min_violated_done
 * 44      intr_cause_pulse_max_violated_done
 * 43      intr1_cause_spare7_done
 * 42      intr1_cause_spare6_done
 * 41      intr1_cause_hpdin_changed_done
 * 40      intr1_cause_cmp_out_done[1]
 * 39      intr1_cause_cmp_out_done[0]
 * 38      intr1_cause_v5v_done
 * 37      intr1_cause_vcmp_dn_done
 * 36      intr1_cause_vcmp_up_done
 * 35      intr1_cause_vcmp_la_done
 * 34      intr1_cause_cc2_changed_done
 * 33      intr1_cause_cc1_changed_done
 * 32      intr1_cause_vconn2_changed_done
 * 31      intr1_cause_vconn1_changed_done
 * 30      intr_cause_cmp_out_det_done[1]
 * 29      intr_cause_sar_done[1]
 * 28      intr_cause_rx_state_idle_done
 * 27      intr_cause_tx_state_idle_done
 * 26      intr_cause_tx_regulator_enabled_done
 * 25      intr_cause_tx_cc_data_oen_deassert_done
 * 24      intr_cause_tx_cc_data_oen_assert_done
 * 23      intr_cause_kchar_error_done
 * 22      intr_cause_tx_retry_enable_clrd_done
 * 21      intr_cause_rx_sram_half_end_done
 * 20      intr_cause_tx_sram_half_end_done
 * 19      intr_cause_cmp_out_det_done[0]
 * 18      intr_cause_collision_type4_done
 * 17      intr_cause_collision_type3_done
 * 16      intr_cause_collision_type2_done
 * 15      intr_cause_collision_type1_done
 * 14      intr_cause_crc_rx_timer_exp_done
 * 13      intr_cause_cc_no_valid_data_detected_done
 * 12      intr_cause_cc_valid_data_detected_done
 * 11      intr_cause_tx_goodcrc_msg_done
 * 10      intr_cause_sar_done[0]
 * 9       intr_cause_rcv_rst_done
 * 8       intr_cause_tx_hard_rst_done
 * 7       intr_cause_tx_packet_done
 * 6       intr_cause_rx_over_run_done
 * 5       intr_cause_eop_error_done
 * 4       intr_cause_rcv_expt_goodcrc_msg_complete_done
 * 3       intr_cause_rcv_goodcrc_msg_complete_done
 * 2       intr_cause_rx_sop_done
 * 1       intr_cause_rcv_bad_packet_complete_done
 * 0       intr_cause_rcv_good_packet_complete_done
 */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT1_SEL_MASK              (0x00003f80) /* <7:13> R:RW:0: */
#define PDSS_INTR_DDFT_MUX_INTR_DDFT1_SEL_POS               (7)


/*
 * IP DDFT Selections
 */
#define PDSS_DDFT_MUX_ADDRESS(m)                            (0x400a0110 + ((m) * (0x10000)))
#define PDSS_DDFT_MUX(m)                                    (*(volatile uint32_t *)(0x400a0110 + ((m) * 0x10000)))
#define PDSS_DDFT_MUX_DEFAULT                               (0x00000000)

/*
 * 47      intr_swap_queue_set
 * 46      intr_swap_unstable_set
 * 45      intr_swap_unpluged_set
 * 44      intr_swap_pluged_set
 * 43      intr_swap_irq_set
 * 42      intr_swapt_command_done_set
 * 41      intr_ddft1
 * 40      intr_ddft0
 * 39      ncell_ddft1
 * 38      ncell_ddft0
 * 37      spare7_fx_scan
 * 36      spare6_fx_scan
 * 35      intr_hpd_queue_set
 * 34      intr_hpd_unstable_set
 * 33      intr_hpd_unpluged_set
 * 32      intr_hpd_pluged_set
 * 31      intr_hpd_irq_set
 * 30      intr_hpdt_command_done_set
 * 29      hpdin_fx_scan
 * 28      ddft_collision_src[4]
 * 27      ddft_collision_src[3]
 * 26      ddft_collision_src[2]
 * 25      ddft_collision_src[1]
 * 24      ddft_collision_src[0]
 * 23      cmp_out_fx_scan[1]
 * 22      cmp_out_fx_scan[0]
 * 21      v5v_fx_scan
 * 20      ddft_cc_core_rx_data
 * 19      vcmp_fs
 * 18      ddft_cc_ctrl_tx_en
 * 17      ddft_cc_tx_data_eop
 * 16      ddft_cc_tx_data_valid
 * 15      clk_tx
 * 14      vconn2_fx_scan
 * 13      vconn1_fx_scan
 * 12      vcmp_dn_fx_scan
 * 11      vcmp_la_fx_scan
 * 10      vcmp_up_fx_scan
 * 9       ddft_sop_valid
 * 8       ddft_rx_eop
 * 7       ddft_raw_cc_rx_valid
 * 6       ddft_cc_rx_bit_en
 * 5       ddft_cc_core_tx_data
 * 4       cc_tx_data
 * 3       clk_rx
 * 2       cc1_fx_scan
 * 1       cc2_fx_scan
 * 0: DDFT0 from previous m0s8usbpd instantiation if any
 */
#define PDSS_DDFT_MUX_DDFT0_SEL_MASK                        (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_DDFT_MUX_DDFT0_SEL_POS                         (0)


/*
 * 47      intr_swap_queue_set
 * 46      intr_swap_unstable_set
 * 45      intr_swap_unpluged_set
 * 44      intr_swap_pluged_set
 * 43      intr_swap_irq_set
 * 42      intr_swapt_command_done_set
 * 41      intr_ddft1
 * 40      intr_ddft0
 * 39      ncell_ddft1
 * 38      ncell_ddft0
 * 37      spare7_fx_scan
 * 36      spare6_fx_scan
 * 35      intr_hpd_queue_set
 * 34      intr_hpd_unstable_set
 * 33      intr_hpd_unpluged_set
 * 32      intr_hpd_pluged_set
 * 31      intr_hpd_irq_set
 * 30      intr_hpdt_command_done_set
 * 29      hpdin_fx_scan
 * 28      ddft_collision_src[4]
 * 27      ddft_collision_src[3]
 * 26      ddft_collision_src[2]
 * 25      ddft_collision_src[1]
 * 24      ddft_collision_src[0]
 * 23      cmp_out_fx_scan[1]
 * 22      cmp_out_fx_scan[0]
 * 21      v5v_fx_scan
 * 20      ddft_cc_core_rx_data
 * 19      vcmp_fs
 * 18      ddft_cc_ctrl_tx_en
 * 17      ddft_cc_tx_data_eop
 * 16      ddft_cc_tx_data_valid
 * 15      clk_tx
 * 14      vconn2_fx_scan
 * 13      vconn1_fx_scan
 * 12      vcmp_dn_fx_scan
 * 11      vcmp_la_fx_scan
 * 10      vcmp_up_fx_scan
 * 9       ddft_sop_valid
 * 8       ddft_rx_eop
 * 7       ddft_raw_cc_rx_valid
 * 6       ddft_cc_rx_bit_en
 * 5       ddft_cc_core_tx_data
 * 4       cc_tx_data
 * 3       clk_rx
 * 2       cc1_fx_scan
 * 1       cc2_fx_scan
 * 0: DDFT1 from previous m0s8usbpd instantiation if any
 */
#define PDSS_DDFT_MUX_DDFT1_SEL_MASK                        (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_DDFT_MUX_DDFT1_SEL_POS                         (6)


/*
 * Wakeup Interrupts edge and filter configuration
 */
#define PDSS_INTR_1_CFG_ADDRESS(m)                          (0x400a0114 + ((m) * (0x10000)))
#define PDSS_INTR_1_CFG(m)                                  (*(volatile uint32_t *)(0x400a0114 + ((m) * 0x10000)))
#define PDSS_INTR_1_CFG_DEFAULT                             (0xd6dfeaaa)

/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_CC1_CFG_MASK                        (0x00000030) /* <4:5> R:RW:2: */
#define PDSS_INTR_1_CFG_CC1_CFG_POS                         (4)


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_CC2_CFG_MASK                        (0x000000c0) /* <6:7> R:RW:2: */
#define PDSS_INTR_1_CFG_CC2_CFG_POS                         (6)


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_VCMP_UP_CFG_MASK                    (0x00000300) /* <8:9> R:RW:2: */
#define PDSS_INTR_1_CFG_VCMP_UP_CFG_POS                     (8)


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_VCMP_DN_CFG_MASK                    (0x00000c00) /* <10:11> R:RW:2: */
#define PDSS_INTR_1_CFG_VCMP_DN_CFG_POS                     (10)


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_VCMP_LA_CFG_MASK                    (0x00003000) /* <12:13> R:RW:2: */
#define PDSS_INTR_1_CFG_VCMP_LA_CFG_POS                     (12)


/*
 * Filtering the CC1_ATTACH from s8usbpd.:
 * 0: No Filtering
 * 1: Enable 2 cycles of clk_lf filtering
 */
#define PDSS_INTR_1_CFG_CC1_CFG_FILT_EN                     (1u << 16) /* <16:16> R:RW:1: */


/*
 * Filtering the CC2_ATTACH from s8usbpd.:
 * 0: No Filtering
 * 1: Enable 2 cycles of clk_lf filtering
 */
#define PDSS_INTR_1_CFG_CC2_FILT_EN                         (1u << 17) /* <17:17> R:RW:1: */


/*
 * Filtering the VCMP_UP from s8usbpd.:
 * 0: No Filtering
 * 1: Enable 2 cycles of clk_lf filtering
 */
#define PDSS_INTR_1_CFG_VCMP_UP_FILT_EN                     (1u << 18) /* <18:18> R:RW:1: */


/*
 * Filtering the VCMP_DNfrom s8usbpd.:
 * 0: No Filtering
 * 1: Enable 2 cycles of clk_lf filtering
 */
#define PDSS_INTR_1_CFG_VCMP_DN_FILT_EN                     (1u << 19) /* <19:19> R:RW:1: */


/*
 * Filtering the v5v_det from s8usbpd so FW can enable the VCONN FET to route
 * the V5V to appropriate VCONN line (CC1/CC2)
 * 0: No Filtering
 * 1: Enable 2 cycles of clk_lf filtering
 */
#define PDSS_INTR_1_CFG_V5V_FILT_EN                         (1u << 20) /* <20:20> R:RW:1:FET_300_EN */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_V5V_CFG_MASK                        (0x00600000) /* <21:22> R:RW:2:FET_300_EN */
#define PDSS_INTR_1_CFG_V5V_CFG_POS                         (21)


/*
 * Filtering the cmp_out from first s8usbpd_adc_top.:
 * 0: No Filtering
 * 1: Refer to ADC1_SAR_CTRL.CMP_OUT_TO_GPIO_FILT_SEL
 */
#define PDSS_INTR_1_CFG_CMP_OUT1_FILT_EN                    (1u << 23) /* <23:23> R:RW:1: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_ADC1_CFG_MASK                       (0x03000000) /* <24:25> R:RW:2: */
#define PDSS_INTR_1_CFG_ADC1_CFG_POS                        (24)


/*
 * Filtering the cmp_out from first s8usbpd_adc_top.:
 * 0: No Filtering
 * 1: Refer to ADC2_SAR_CTRL.CMP_OUT_TO_GPIO_FILT_SEL
 */
#define PDSS_INTR_1_CFG_CMP_OUT2_FILT_EN                    (1u << 26) /* <26:26> R:RW:1: */


/*
 * Edge detect positive/negative enable/disable
 */
#define PDSS_INTR_1_CFG_ADC2_CFG_MASK                       (0x18000000) /* <27:28> R:RW:2: */
#define PDSS_INTR_1_CFG_ADC2_CFG_POS                        (27)


/*
 * Edge detect positive/negative enable/disable. For HPD_IN input
 */
#define PDSS_INTR_1_CFG_HPDIN_CFG_MASK                      (0x60000000) /* <29:30> R:RW:2: */
#define PDSS_INTR_1_CFG_HPDIN_CFG_POS                       (29)


/*
 * Filtering the HPDIN_ATTACH .
 * 0: No Filtering
 * 1: Enable 2 cycles of clk_lf filtering
 */
#define PDSS_INTR_1_CFG_HPDIN_FILT_EN                       (1u << 31) /* <31:31> R:RW:1: */


/*
 * INTR0 Cause. These are the active interrupts get reflected on interrupt_usbpd
 * pin.
 */
#define PDSS_INTR0_ADDRESS(m)                               (0x400a0118 + ((m) * (0x10000)))
#define PDSS_INTR0(m)                                       (*(volatile uint32_t *)(0x400a0118 + ((m) * 0x10000)))
#define PDSS_INTR0_DEFAULT                                  (0x00000000)

/*
 * Receive a Good non-GoodCRC-message Data Packet Complete. Indicates that
 * the Receive Packet has been received in its entirety.  The received packet
 * had no CRC and no KCHAR error.
 * If this interrupt is not cleared, then RX_OVER_RUN will be set on the
 * next new data and the new data won't be written into RX SRAM.
 */
#define PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE                 (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * Receive a Bad non-GoodCRC-message Data Packet Complete. Indicates that
 * the Receive Packet has been received in its entirety.  The received packet
 * had CRC or KCHAR error.
 */
#define PDSS_INTR0_RCV_BAD_PACKET_COMPLETE                  (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * Receive a SOP. FW should read SOP_TYPE_DETECTED for the SOP type
 */
#define PDSS_INTR0_RX_SOP                                   (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * Receive GoodCRC-message Complete. Indicates that the GoodCRC-message is
 * stored in RX_GOODCRC_MSG register.
 * The received GoodCRC-message had no CRC error and no KCHAR error
 */
#define PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE                 (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * Receive the expted GoodCRC-message based on the RX_EXPECT_GOODCRC_MSG
 * register. Indicates that the expected GoodCRC-message is stored in RX_GOODCRC_MSG
 * register.
 * The received expected GoodCRC-message had no CRC error and no KCHAR error
 * This interrupt gets evaluated on end of every packet (except hard reset)
 * and needs to be cleared.
 * For Correct usage of the interrupt:
 * This interrupt should be cleared on every packet if set.
 * RCV_GOOGCRC_MSG_COMPLETE should be cleared before new good crc response
 * comes else RX_GOODCRC_MSG will not get updated.
 */
#define PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE            (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * Received Symbol wasn't a valid EOP K-Code.  It should be evaludated after
 * RCV_PACKET_COMPLETE.
 */
#define PDSS_INTR0_EOP_ERROR                                (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * New data was received when the RCV_PACKET_COMPLETE is not cleared by CPU.
 */
#define PDSS_INTR0_RX_OVER_RUN                              (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * Transmitter done sending data packet.
 */
#define PDSS_INTR0_TX_PACKET_DONE                           (1u << 7) /* <7:7> RW1S:RW1C:0: */


/*
 * Transmitter done sending Hard Reset
 */
#define PDSS_INTR0_TX_HARD_RST_DONE                         (1u << 8) /* <8:8> RW1S:RW1C:0: */


/*
 * Received a REST. FW should read RST_TYPE for the type of RST.
 * Firmware should process this interrupt according to the USB-PD spec.
 * Hardware does not process the Reset packets other than providing this
 * interrupt.
 * Hardware will stop processing any pending transmit packet until this interrupt
 * is cleared.
 */
#define PDSS_INTR0_RCV_RST                                  (1u << 9) /* <9:9> RW1S:RW1C:0: */


/*
 * Marks Completion of First ADC SAR conversion at the end of 8 cycles of
 * clk_sar when SAR_EN is "1"
 */
#define PDSS_INTR0_SAR1_DONE                                (1u << 10) /* <10:10> RW1S:RW1C:0: */


/*
 * A GoodCRC message was transmitted.
 */
#define PDSS_INTR0_TX_GOODCRC_MSG_DONE                      (1u << 11) /* <11:11> RW1S:RW1C:0: */


/*
 * Valid Data detected on the CC line
 */
#define PDSS_INTR0_CC_VALID_DATA_DETECTED                   (1u << 12) /* <12:12> RW1S:RW1C:0: */


/*
 * Valid Data got de-asserted on the CC line
 */
#define PDSS_INTR0_CC_NO_VALID_DATA_DETECTED                (1u << 13) /* <13:13> RW1S:RW1C:0: */


/*
 * CRCReceiveTimer has expired
 */
#define PDSS_INTR0_CRC_RX_TIMER_EXP                         (1u << 14) /* <14:14> RW1S:RW1C:0: */


/*
 * Transmit Collision Type1:
 * Collsion is detected Due to TX_GO(TX Data)/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE1                          (1u << 15) /* <15:15> RW1S:RW1C:0: */


/*
 * Transmit Collision Type2:
 * Collsion is detected  due to TX-RETRY/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE2                          (1u << 16) /* <16:16> RW1S:RW1C:0: */


/*
 * Transmit Collision Type3:
 * Collsion is detected due to TX-GoodCrc_MSG/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE3                          (1u << 17) /* <17:17> RW1S:RW1C:0: */


/*
 * Transmit Collision Type4:
 * Collsion is detected due to TX_SEND_RST/RX has occurred
 */
#define PDSS_INTR0_COLLISION_TYPE4                          (1u << 18) /* <18:18> RW1S:RW1C:0: */


/*
 * cmp_out pin of first ADC s8usbpd_adc_top changed based on the ADC_SAR_CTRL.INTR_CMP_SEL
 * INTR1.CMP_OUT1_CHANGED has the same function except INTR1 is for wakeup
 */
#define PDSS_INTR0_CMP_OUT1_DET                             (1u << 19) /* <19:19> RW1S:RW1C:0: */


/*
 * Hardware has passed reading the data from Half or End of the TX SRAM Memory
 * Location
 */
#define PDSS_INTR0_TX_SRAM_HALF_END                         (1u << 20) /* <20:20> RW1S:RW1C:0: */


/*
 * Hardware has passed writing the data to Half or End of the RX SRAM Memory
 * Location
 */
#define PDSS_INTR0_RX_SRAM_HALF_END                         (1u << 21) /* <21:21> RW1S:RW1C:0: */


/*
 * TX_RETRY_ENABLE is cleared
 */
#define PDSS_INTR0_TX_RETRY_ENABLE_CLRD                     (1u << 22) /* <22:22> RW1S:RW1C:0: */


/*
 * Received Symbol wasn't a valid K-Code.
 */
#define PDSS_INTR0_KCHAR_ERROR                              (1u << 23) /* <23:23> RW1S:RW1C:0: */


/*
 * TX Data Output Enable of TX-CC is asserted
 */
#define PDSS_INTR0_TX_CC_DATA_OEN_ASSERT                    (1u << 24) /* <24:24> RW1S:RW1C:0: */


/*
 * TX Data Output Enable of TX-CC is de-asserted
 */
#define PDSS_INTR0_TX_CC_DATA_OEN_DEASSERT                  (1u << 25) /* <25:25> RW1S:RW1C:0: */


/*
 * The TX CC regulator is enabled
 */
#define PDSS_INTR0_TX_REGULATOR_ENABLED                     (1u << 26) /* <26:26> RW1S:RW1C:0: */


/*
 * The TX State Machine entered Idle state
 */
#define PDSS_INTR0_TX_STATE_IDLE                            (1u << 27) /* <27:27> RW1S:RW1C:0: */


/*
 * The RX State Machine entered Idle state
 */
#define PDSS_INTR0_RX_STATE_IDLE                            (1u << 28) /* <28:28> RW1S:RW1C:0: */


/*
 * Marks Completion of Second ADC SAR conversion at the end of 8 cycles of
 * clk_sar when SAR_EN is "1"
 */
#define PDSS_INTR0_SAR2_DONE                                (1u << 29) /* <29:29> RW1S:RW1C:0: */


/*
 * cmp_out pin of second ADC s8usbpd_adc_top changed based on the ADC_SAR_CTRL.INTR_CMP_SEL
 * INTR1.CMP_OUT2_CHANGED has the same function except INTR1 is for wakeup
 */
#define PDSS_INTR0_CMP_OUT2_DET                             (1u << 30) /* <30:30> RW1S:RW1C:0: */


/*
 * INTR1 Cause.  These are the wakeup interrupts get reflected on interrupt_wakeup
 * pin.
 * The configurations for using the comparators:
 *
 * DFP waiting for attach:
 * vcmp_up connected to CC1: HI = no connect, LO = attach
 * vcmp_dn connected to CC2: HI = no connect, LO = attach
 *
 * DFP after attach:
 * vcmup_up at detach threshold: HI = detach, LO = attach
 * vcmp_dn at Rd/Ra threshold: HI = Rd connected, LO = Ra connected
 * vcmp_la at CC line activity threshold: HI = no activity, LO/Toggling =
 * activity
 *
 * UFP (with VBUS present):
 * vcmup_up at Default/1.5A threshold: HI = Default, LO = 1.5A
 * vcmp_dn at 1.5A/3.0A threshold: HI = 1.5A, LO = 3.0A
 * vcmp_la at CC line activity threshold: HI = no activity, LO/Toggling =
 * activity
 *
 * For detecting the difference between Rd/Ra, firmware will have to check
 * the “DFP after attach” state above to determine it.
 */
#define PDSS_INTR1_ADDRESS(m)                               (0x400a011c + ((m) * (0x10000)))
#define PDSS_INTR1(m)                                       (*(volatile uint32_t *)(0x400a011c + ((m) * 0x10000)))
#define PDSS_INTR1_DEFAULT                                  (0x00000000)

/*
 * CC1 changed (wakeup interrupt from deepsleep)
 * Check the STATUS.CC1_STATUS value
 */
#define PDSS_INTR1_CC1_CHANGED                              (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * CC2 changed (wakeup interrupt from deepsleep)
 * Check the STATUS.CC2_STATUS value
 */
#define PDSS_INTR1_CC2_CHANGED                              (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * VCMP_LA changed (wakeup interrupt from deepsleep)
 * Check the STATUS.VCMP_LA_STATUS value
 */
#define PDSS_INTR1_VCMP_LA_CHANGED                          (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * VCMP_UP changed (wakeup interrupt from deepsleep)
 * Check the STATUS.VCMP_UP_STATUS value
 */
#define PDSS_INTR1_VCMP_UP_CHANGED                          (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * VCMP_DN changed (wakeup interrupt from deepsleep)
 * Check the STATUS.VCMP_DN_STATUS value
 */
#define PDSS_INTR1_VCMP_DN_CHANGED                          (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * V5V changed (wakeup interrupt from deepsleep)
 * Check the STATUS.V5V_STATUS value
 */
#define PDSS_INTR1_V5V_CHANGED                              (1u << 7) /* <7:7> RW1S:RW1C:0:FET_300_EN */


/*
 * CMP_OUT1 changed (wakeup interrupt from deepsleep)
 * Check the STATUS.ADC1_COMP_OUT_STATUS value
 */
#define PDSS_INTR1_CMP_OUT1_CHANGED                         (1u << 8) /* <8:8> RW1S:RW1C:0: */


/*
 * CMP_OUT2 changed (wakeup interrupt from deepsleep)
 * Check the STATUS.ADC2_COMP_OUT_STATUS value
 */
#define PDSS_INTR1_CMP_OUT2_CHANGED                         (1u << 9) /* <9:9> RW1S:RW1C:0: */


/*
 * HPD_IN changed (wakeup interrupt from deepsleep). For HPD input
 * Check the STATUS.HPDIN_STATUS value. Any change in HPD input will trigger
 * a wakeup interrupt.
 */
#define PDSS_INTR1_HPDIN_CHANGED                            (1u << 10) /* <10:10> RW1S:RW1C:0: */


/*
 * SPARE6. Check the STATUS.SPARE6_STATUS value
 */
#define PDSS_INTR1_SPARE6                                   (1u << 11) /* <11:11> RW1S:RW1C:0: */


/*
 * SPARE7. Check the STATUS.SPARE7_STATUS value
 */
#define PDSS_INTR1_SPARE7                                   (1u << 12) /* <12:12> RW1S:RW1C:0: */


/*
 * INTR0 Set
 */
#define PDSS_INTR0_SET_ADDRESS(m)                           (0x400a0120 + ((m) * (0x10000)))
#define PDSS_INTR0_SET(m)                                   (*(volatile uint32_t *)(0x400a0120 + ((m) * 0x10000)))
#define PDSS_INTR0_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_GOOD_PACKET_COMPLETE             (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_BAD_PACKET_COMPLETE              (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_SOP                               (1u << 2) /* <2:2> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_GOODCRC_MSG_COMPLETE             (1u << 3) /* <3:3> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_EXPT_GOODCRC_MSG_COMPLETE        (1u << 4) /* <4:4> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_EOP_ERROR                            (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_OVER_RUN                          (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_PACKET_DONE                       (1u << 7) /* <7:7> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_HARD_RST_DONE                     (1u << 8) /* <8:8> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RCV_RST                              (1u << 9) /* <9:9> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_SAR1_DONE                            (1u << 10) /* <10:10> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_GOODCRC_MSG_DONE                  (1u << 11) /* <11:11> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CC_VALID_DATA_DETECTED               (1u << 12) /* <12:12> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CC_NO_VALID_DATA_DETECTED            (1u << 13) /* <13:13> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CRC_RX_TIMER_EXP                     (1u << 14) /* <14:14> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE1                      (1u << 15) /* <15:15> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE2                      (1u << 16) /* <16:16> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE3                      (1u << 17) /* <17:17> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_COLLISION_TYPE4                      (1u << 18) /* <18:18> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CMP_OUT1_DET                         (1u << 19) /* <19:19> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_SRAM_HALF_END                     (1u << 20) /* <20:20> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_SRAM_HALF_END                     (1u << 21) /* <21:21> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_RETRY_ENABLE_CLRD                 (1u << 22) /* <22:22> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_KCHAR_ERROR                          (1u << 23) /* <23:23> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_CC_DATA_OEN_ASSERT                (1u << 24) /* <24:24> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_CC_DATA_OEN_DEASSERT              (1u << 25) /* <25:25> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_REGULATOR_ENABLED                 (1u << 26) /* <26:26> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_TX_STATE_IDLE                        (1u << 27) /* <27:27> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_RX_STATE_IDLE                        (1u << 28) /* <28:28> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_SAR2_DONE                            (1u << 29) /* <29:29> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_SET_CMP_OUT2_DET                         (1u << 30) /* <30:30> A:RW1S:0: */


/*
 * INTR1 Set
 */
#define PDSS_INTR1_SET_ADDRESS(m)                           (0x400a0124 + ((m) * (0x10000)))
#define PDSS_INTR1_SET(m)                                   (*(volatile uint32_t *)(0x400a0124 + ((m) * 0x10000)))
#define PDSS_INTR1_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_CC1_CHANGED                          (1u << 2) /* <2:2> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_CC2_CHANGED                          (1u << 3) /* <3:3> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_VCMP_LA_CHANGED                      (1u << 4) /* <4:4> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_VCMP_UP_CHANGED                      (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_VCMP_DN_CHANGED                      (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_V5V_CHANGED                          (1u << 7) /* <7:7> A:RW1S:0:FET_300_EN */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_CMP_OUT1_CHANGED                     (1u << 8) /* <8:8> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_CMP_OUT2_CHANGED                     (1u << 9) /* <9:9> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_HPDIN_CHANGED                        (1u << 10) /* <10:10> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_SPARE6                               (1u << 11) /* <11:11> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_SET_SPARE7                               (1u << 12) /* <12:12> A:RW1S:0: */


/*
 * INTR0 Mask
 */
#define PDSS_INTR0_MASK_ADDRESS(m)                          (0x400a0128 + ((m) * (0x10000)))
#define PDSS_INTR0_MASK(m)                                  (*(volatile uint32_t *)(0x400a0128 + ((m) * 0x10000)))
#define PDSS_INTR0_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_GOOD_PACKET_COMPLETE_MASK       (1u << 0) /* <0:0>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_BAD_PACKET_COMPLETE_MASK        (1u << 1) /* <1:1>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_SOP_MASK                         (1u << 2) /* <2:2>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_GOODCRC_MSG_COMPLETE_MASK       (1u << 3) /* <3:3>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_EXPT_GOODCRC_MSG_COMPLETE_MASK    (1u << 4) /* <4:4>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_EOP_ERROR_MASK                      (1u << 5) /* <5:5> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_OVER_RUN_MASK                    (1u << 6) /* <6:6> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_PACKET_DONE_MASK                 (1u << 7) /* <7:7> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_HARD_RST_DONE_MASK               (1u << 8) /* <8:8> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RCV_RST_MASK                        (1u << 9) /* <9:9> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_SAR1_DONE_MASK                      (1u << 10) /* <10:10> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_GOODCRC_MSG_DONE_MASK            (1u << 11) /* <11:11> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CC_VALID_DATA_DETECTED_MASK         (1u << 12) /* <12:12> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CC_NO_VALID_DATA_DETECTED_MASK      (1u << 13) /* <13:13> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CRC_RX_TIMER_EXP_MASK               (1u << 14) /* <14:14> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE1_MASK                (1u << 15) /* <15:15> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE2_MASK                (1u << 16) /* <16:16> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE3_MASK                (1u << 17) /* <17:17> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_COLLISION_TYPE4_MASK                (1u << 18) /* <18:18> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CMP_OUT1_DET_MASK                   (1u << 19) /* <19:19> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_SRAM_HALF_END_MASK               (1u << 20) /* <20:20> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_SRAM_HALF_END_MASK               (1u << 21) /* <21:21> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_RETRY_ENABLE_CLRD_MASK           (1u << 22) /* <22:22> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_KCHAR_ERROR_MASK                    (1u << 23) /* <23:23> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_CC_DATA_OEN_ASSERT_MASK          (1u << 24) /* <24:24> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_CC_DATA_OEN_DEASSERT_MASK        (1u << 25) /* <25:25> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_REGULATOR_ENABLED_MASK           (1u << 26) /* <26:26> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_TX_STATE_IDLE_MASK                  (1u << 27) /* <27:27> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_RX_STATE_IDLE_MASK                  (1u << 28) /* <28:28> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_SAR2_DONE_MASK                      (1u << 29) /* <29:29> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR0_MASK_CMP_OUT2_DET_MASK                   (1u << 30) /* <30:30> R:RW:0: */


/*
 * INTR1 Mask
 */
#define PDSS_INTR1_MASK_ADDRESS(m)                          (0x400a012c + ((m) * (0x10000)))
#define PDSS_INTR1_MASK(m)                                  (*(volatile uint32_t *)(0x400a012c + ((m) * 0x10000)))
#define PDSS_INTR1_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_CC1_CHANGED_MASK                    (1u << 2) /* <2:2>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_CC2_CHANGED_MASK                    (1u << 3) /* <3:3>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_VCMP_LA_CHANGED_MASK                (1u << 4) /* <4:4>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_VCMP_UP_CHANGED_MASK                (1u << 5) /* <5:5>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_VCMP_DN_CHANGED_MASK                (1u << 6) /* <6:6>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_V5V_CHANGED_MASK                    (1u << 7) /* <7:7>  R:RW:0:FET_300_EN */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_CMP_OUT1_CHANGED_MASK               (1u << 8) /* <8:8>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_CMP_OUT2_CHANGED_MASK               (1u << 9) /* <9:9>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_HPDIN_CHANGED_MASK                  (1u << 10) /* <10:10>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_SPARE6_MASK                         (1u << 11) /* <11:11>  R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR1_MASK_SPARE7_MASK                         (1u << 12) /* <12:12>  R:RW:0: */


/*
 * INTR0 Masked
 */
#define PDSS_INTR0_MASKED_ADDRESS(m)                        (0x400a0130 + ((m) * (0x10000)))
#define PDSS_INTR0_MASKED(m)                                (*(volatile uint32_t *)(0x400a0130 + ((m) * 0x10000)))
#define PDSS_INTR0_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_GOOD_PACKET_COMPLETE_MASKED    (1u << 0) /* <0:0> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_BAD_PACKET_COMPLETE_MASKED    (1u << 1) /* <1:1> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_SOP_MASKED                     (1u << 2) /* <2:2> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_GOODCRC_MSG_COMPLETE_MASKED    (1u << 3) /* <3:3> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_EXPT_GOODCRC_MSG_COMPLETE_MASKED    (1u << 4) /* <4:4> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_EOP_ERROR_MASKED                  (1u << 5) /* <5:5> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_OVER_RUN_MASKED                (1u << 6) /* <6:6> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_PACKET_DONE_MASKED             (1u << 7) /* <7:7> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_HARD_RST_DONE_MASKED           (1u << 8) /* <8:8> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RCV_RST_MASKED                    (1u << 9) /* <9:9> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_SAR1_DONE_MASKED                  (1u << 10) /* <10:10> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_GOODCRC_MSG_DONE_MASKED        (1u << 11) /* <11:11> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CC_VALID_DATA_DETECTED_MASKED     (1u << 12) /* <12:12> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CC_NO_VALID_DATA_DETECTED_MASKED    (1u << 13) /* <13:13> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CRC_RX_TIMER_EXP_MASKED           (1u << 14) /* <14:14> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE1_MASKED            (1u << 15) /* <15:15> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE2_MASKED            (1u << 16) /* <16:16> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE3_MASKED            (1u << 17) /* <17:17> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_COLLISION_TYPE4_MASKED            (1u << 18) /* <18:18> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CMP_OUT1_DET_MASKED               (1u << 19) /* <19:19> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_SRAM_HALF_END_MASKED           (1u << 20) /* <20:20> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_SRAM_HALF_END_MASKED           (1u << 21) /* <21:21> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_RETRY_ENABLE_CLRD_MASKED       (1u << 22) /* <22:22> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_KCHAR_ERROR_MASKED                (1u << 23) /* <23:23> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_CC_DATA_OEN_ASSERT_MASKED      (1u << 24) /* <24:24> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_CC_DATA_OEN_DEASSERT_MASKED    (1u << 25) /* <25:25> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_REGULATOR_ENABLED_MASKED       (1u << 26) /* <26:26> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_TX_STATE_IDLE_MASKED              (1u << 27) /* <27:27> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_RX_STATE_IDLE_MASKED              (1u << 28) /* <28:28> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_SAR2_DONE_MASKED                  (1u << 29) /* <29:29> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR0_MASKED_CMP_OUT2_DET_MASKED               (1u << 30) /* <30:30> RW:R:0: */


/*
 * INTR1 Masked
 */
#define PDSS_INTR1_MASKED_ADDRESS(m)                        (0x400a0134 + ((m) * (0x10000)))
#define PDSS_INTR1_MASKED(m)                                (*(volatile uint32_t *)(0x400a0134 + ((m) * 0x10000)))
#define PDSS_INTR1_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_CC1_CHANGED_MASKED                (1u << 2) /* <2:2> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_CC2_CHANGED_MASKED                (1u << 3) /* <3:3> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_VCMP_LA_CHANGED_MASKED            (1u << 4) /* <4:4> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_VCMP_UP_CHANGED_MASKED            (1u << 5) /* <5:5> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_VCMP_DN_CHANGED_MASKED            (1u << 6) /* <6:6> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_V5V_CHANGED_MASKED                (1u << 7) /* <7:7> RW:R:0:FET_300_EN */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_CMP_OUT1_CHANGED_MASKED           (1u << 8) /* <8:8> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_CMP_OUT2_CHANGED_MASKED           (1u << 9) /* <9:9> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_HPDIN_CHANGED_MASKED              (1u << 10) /* <10:10> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_SPARE6_MASKED                     (1u << 11) /* <11:11> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR1_MASKED_SPARE7_MASKED                     (1u << 12) /* <12:12> RW:R:0: */


/*
 * Debug Control Register
 */
#define PDSS_DEBUG_CTRL_ADDRESS(m)                          (0x400a0138 + ((m) * (0x10000)))
#define PDSS_DEBUG_CTRL(m)                                  (*(volatile uint32_t *)(0x400a0138 + ((m) * 0x10000)))
#define PDSS_DEBUG_CTRL_DEFAULT                             (0x037f0000)

/*
 * This register are for debugging purposes.
 * 0: Receive path is not at reset.
 * 1: Reset the logic on the receive path except the Hard-IP.
 *     FW should check STATUS.RX_BUSY to make sure it is zero before setting
 * this bit.
 */
#define PDSS_DEBUG_CTRL_RESET_RX                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * Message Cal State
 */
#define PDSS_DEBUG_CTRL_RX_MSG_CAL_STATE_MASK               (0x0000000e) /* <1:3> RW:R:0: */
#define PDSS_DEBUG_CTRL_RX_MSG_CAL_STATE_POS                (1)


/*
 * This register are for debugging purposes
 * 0: Transmit path is not at reset.
 * 1: Reset the logic on the transmit path except the Hard-IP.
 *     FW should check STATUS.TX_BUSY to make sure it is zero before setting
 * this bit.
 */
#define PDSS_DEBUG_CTRL_RESET_TX                            (1u << 8) /* <8:8> R:RW:0: */


/*
 * Transmit state machine
 */
#define PDSS_DEBUG_CTRL_TX_MSG_STATE_MASK                   (0x00000e00) /* <9:11> RW:R:0: */
#define PDSS_DEBUG_CTRL_TX_MSG_STATE_POS                    (9)


/*
 * TX SRC Select state machine
 */
#define PDSS_DEBUG_CTRL_TX_SRC_SEL_STATE_MASK               (0x0000e000) /* <13:15> RW:R:0: */
#define PDSS_DEBUG_CTRL_TX_SRC_SEL_STATE_POS                (13)


/*
 * Number of TX preambles+1 (bit transitions)
 */
#define PDSS_DEBUG_CTRL_TX_PREAMBLE_CNT_MASK                (0x003f0000) /* <16:21> R:RW:63: */
#define PDSS_DEBUG_CTRL_TX_PREAMBLE_CNT_POS                 (16)


/*
 * EOP value for Both RX and TX
 */
#define PDSS_DEBUG_CTRL_EOP_VALUE_MASK                      (0x07c00000) /* <22:26> R:RW:13: */
#define PDSS_DEBUG_CTRL_EOP_VALUE_POS                       (22)


/*
 * This bit enables the Receive state machine to exit from RX DATA and CRC
 * states when an EOP is detected.
 * 0: Ignore EOP during the Data and CRC state
 * 1: Exit from Data and CRC state when an EOP is detected and move to STATUS
 * state.
 */
#define PDSS_DEBUG_CTRL_ENABLE_EXIT_ON_EOP                  (1u << 27) /* <27:27> R:RW:0: */


/*
 * C-Connector Debug control register 0
 */
#define PDSS_DEBUG_CC_0_ADDRESS(m)                          (0x400a013c + ((m) * (0x10000)))
#define PDSS_DEBUG_CC_0(m)                                  (*(volatile uint32_t *)(0x400a013c + ((m) * 0x10000)))
#define PDSS_DEBUG_CC_0_DEFAULT                             (0x00000000)

/*
 * FW can only use this bit when the DEBUG_CC_0.TX_CC_DRIVE_SRC is set to
 * "1".
 * 0: Disables the Transceiver to transmit data
 * 1: Enables the Transceiver to transmit data
 */
#define PDSS_DEBUG_CC_0_TX_FIRST_BIT_LEVEL                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * When set enables TX to RX loopback before CC encoding/Decoding data.
 */
#define PDSS_DEBUG_CC_0_LOOP_BACK_NO_BMC                    (1u << 1) /* <1:1> R:RW:0: */


/*
 * Loobback after data encdoing. When set, the BMC encoded tx output will
 * loop back into cc_rx module.
 */
#define PDSS_DEBUG_CC_0_LOOP_BACK_WITH_BMC                  (1u << 2) /* <2:2> R:RW:0: */


/*
 * When set, enables rx module to decode cc_rx line all the time. (Including
 * during transmission).
 */
#define PDSS_DEBUG_CC_0_EXT_LOOP_BACK                       (1u << 3) /* <3:3> R:RW:0: */


/*
 * When set to one, clears the BMC decoder RX state machines and counters.
 * It has to be set back to zero for normal operations.
 * RX_RESET is not required to be set
 */
#define PDSS_DEBUG_CC_0_RX_CLEAR                            (1u << 4) /* <4:4> R:RW:0: */


/*
 * When set to one, clears the TX state machines and counters. It has to
 * be set back to zero for normal operations.
 */
#define PDSS_DEBUG_CC_0_TX_CLEAR                            (1u << 5) /* <5:5> R:RW:0: */


/*
 * This will selects either the m0s8usbpd_cc_tx or FW to control the TX_EN/TX_DATA
 * ports of the s8usbpd_cc_top Hard IP.
 * 0: Hardware (m0s8usbpd_cc_tx) controls the TX_EN/TX_DATA ports of the
 * s8usbpd_cc_top Hard IP.
 * 1: This option is for Testing/Char. FW controls the TX_EN/TX_DATA ports
 * of the s8usbpd_cc_top Hard IP.
 */
#define PDSS_DEBUG_CC_0_TX_CC_DRIVE_SRC                     (1u << 6) /* <6:6> R:RW:0: */


/*
 * FW can use this bit to dirve the CC data line when the TX_CC_DRIVE_SRC=1
 * and CC_DPSLP_REF_CTRL.TX_EN=1
 * When TX_CC_DRIVE_SRC is set to one:
 * - TX_EN port of the s8usbpd_cc_top is controlled by CC_DPSLP_REF_CTRL.TX_EN
 * - TX_DATA port of s8usbpd_cc_top Hard IP is controlled by TX_CC_DATA
 */
#define PDSS_DEBUG_CC_0_TX_CC_DATA                          (1u << 7) /* <7:7> R:RW:0: */


/*
 * Selects the inputs to CC_DEBUG_OUT. Used for debug.
 * 0. RX clk_cnt_q[7:0]
 * 1. RX {clk_cnt_q[9:8], cdr_accum_q[11:8], cq_2, cq_3}
 * 2. {count_q, 2'h0}
 * 3. cdr_accum_q[7:0]
 * 4. {1'h0, rx_state_q, cc_rx_data_del_q}
 * 5. limit_q
 * 6. RX {one_detect_q, cc_rx_bit_early_q,  cc_rx_data, cc_rx_bit, cc_rx_valid,
 * cc_rx_data_2_sync, cc_tx_data_pin, cc_data_pin_oe}
 * 7. cdr_average_q_latched_q
 * 8. {diff_q[3:0], low_count_q[3:0]}
 * 9. cdr_average_q[7:0]
 * 16. Not defined yet.
 * 17 TX {2'h0, tx_state_q, new_data_q}
 * 18. TX {2'h0, cq_0, cq_3, cc_tx_data_lat_q, cc_tx_eof_lat_q, cc_tx_data_valid_lat_q,
 * cc_new_data_lat_q}
 * 19.  TX {3'h0, cc_tx_data_pin, cc_data_pin_oe, cc_tx_data, cc_tx_eof,
 * cc_tx_data_valid}
 * 20-15 not defined yet.
 */
#define PDSS_DEBUG_CC_0_DEBUG_SEL_MASK                      (0x00001f00) /* <8:12> R:RW:0: */
#define PDSS_DEBUG_CC_0_DEBUG_SEL_POS                       (8)


/*
 * 0: RX_CC_DATA_VALID signal is not disabled.
 * 1: RX_CC_DATA_VALID signal is disabled.
 */
#define PDSS_DEBUG_CC_0_RX_CC_DATA_VALID_DIS                (1u << 13) /* <13:13> R:RW:0: */


/*
 * Delays CC_TX output by number of clocks specified by TX_DELAY_VALUE. CLK_HF
 * is used for this purpose.
 * 0: delay falling edge of CC TX output
 * 1: delay rising edge of CC TX output
 */
#define PDSS_DEBUG_CC_0_TX_DELAY_FALLN_RISE                 (1u << 14) /* <14:14> R:RW:0: */


/*
 * Specifies number of HF clocks to delay CC_TX output.
 * 0 - Do not delay output.
 * 1 - Delay by 1 HF clock
 * 2 - Delay by 2 HF clock.
 *
 * 15 - Delay by 15 HF clock.
 */
#define PDSS_DEBUG_CC_0_TX_DELAY_VALUE_MASK                 (0x00078000) /* <15:18> R:RW:0: */
#define PDSS_DEBUG_CC_0_TX_DELAY_VALUE_POS                  (15)


/*
 * Debug output register. Its inputs are selected by CC_DEBUG_SEL
 */
#define PDSS_DEBUG_CC_0_DEBUG_OUT_MASK                      (0x0ff00000) /* <20:27> RW:R:0: */
#define PDSS_DEBUG_CC_0_DEBUG_OUT_POS                       (20)


/*
 * Selection bit for deepsleep vs. active current reference for Rp pull-up
 * termination
 * 0 - Select deepsleep current reference
 * 1 - Select active current reference
 */
#define PDSS_DEBUG_CC_0_IREF_SEL                            (1u << 28) /* <28:28> R:RW:0: */


/*
 * 0: Create EOP in the case of TX BIST
 * 1: Don’t create EOP in the case of TX BIST
 */
#define PDSS_DEBUG_CC_0_DISABLE_BIST_EOP                    (1u << 29) /* <29:29> R:RW:0: */


/*
 * 0: Source for turing off the VBUS_C Gate driver is SWAPR IRQ
 * 1: Source for turing off the VBUS_C Gate driver is SWAPR IRQ and VBUS_C
 * less than 5 volt
 */
#define PDSS_DEBUG_CC_0_VBUS_C_SWAP_SOURCE_SEL              (1u << 30) /* <30:30> R:RW:0: */


/*
 * Spare1 (Used for CSA BW[1])
 */
#define PDSS_DEBUG_CC_0_SPARE1                              (1u << 31) /* <31:31> R:RW:0: */


/*
 * C-Connector Debug control register 1
 */
#define PDSS_DEBUG_CC_1_ADDRESS(m)                          (0x400a0140 + ((m) * (0x10000)))
#define PDSS_DEBUG_CC_1(m)                                  (*(volatile uint32_t *)(0x400a0140 + ((m) * 0x10000)))
#define PDSS_DEBUG_CC_1_DEFAULT                             (0x000140cc)

/*
 * Number of preamble bits to be used in the RX for averaging CDR frequency.
 * Any time the value of these bits are changed, the values of NUM_TRANS_AVG
 * will need to be updated.
 */
#define PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_MASK               (0x00000007) /* <0:2> R:RW:4: */
#define PDSS_DEBUG_CC_1_NUM_PREAMBLE_AVG_POS                (0)


/*
 * Number of transitions required to complete averaging in the receiver.
 * This register will need to be updated any time values of NUM_PREAMBLE_AVG
 * is changed.
 * Total of this register and RX_IGNR_TRANS_NUM should not be more than 63
 * decimal.
 * The values programmed into this register comes from the following table:
 * NUM_PREAMBLE_AVG = 000 : Use 0x19
 * NUM_PREAMBLE_AVG = 001 : Use 0x19
 * NUM_PREAMBLE_AVG = 010 : Use 0x7
 * NUM_PREAMBLE_AVG = 011 : Use 0xd
 * NUM_PREAMBLE_AVG = 100 : Use 0x19
 * NUM_PREAMBLE_AVG = 101 : Use 0x31
 */
#define PDSS_DEBUG_CC_1_NUM_TRANS_AVG_MASK                  (0x000001f8) /* <3:8> R:RW:25: */
#define PDSS_DEBUG_CC_1_NUM_TRANS_AVG_POS                   (3)


/*
 * 0: Automatic bit rate calculation by HW
 * 1: Disables the RX-CC automatic bit rate detection and the RX_UI_PRERIOD
 * register is used for RX UI period.
 */
#define PDSS_DEBUG_CC_1_RX_DISABLE_AUTO_ADJ                 (1u << 9) /* <9:9> R:RW:0: */


/*
 * When RX_DISABLE_AUTO_ADJ is set, this register value will define RX UI
 * period.
 */
#define PDSS_DEBUG_CC_1_RX_UI_PERIOD_MASK                   (0x0003fc00) /* <10:17> R:RW:80: */
#define PDSS_DEBUG_CC_1_RX_UI_PERIOD_POS                    (10)


/*
 * 0: The TX statemachine does not reset to Idle on the assertion of "send_good_crc"
 * 1: The TX statemachine does       reset to Idle on the assertion of "send_good_crc"
 */
#define PDSS_DEBUG_CC_1_TX_STATE_RST                        (1u << 18) /* <18:18> R:RW:0: */


/*
 *  300ma switch CC1 Pull down value
 */
#define PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC1               (1u << 19) /* <19:19> R:RW:0: */


/*
 *  300ma switch CC2 Pull down value
 */
#define PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC2               (1u << 20) /* <20:20> R:RW:0: */


/*
 * 0: If PFET300_CTRL.EN_SWITCH_CC1 is "1" then
 *                        300ma_switch pulldn_en_cc1 and SWAPT_OUT_GPIO1
 * will be driven "0"
 *     else
 *                        300ma_switch pulldn_en_cc1 and SWAPT_OUT_GPIO1
 *  will be driven based on
 *                        SWAP_CTRL0.SWAPT_TO_GPIO1_EN register
 * 1: 300ma_switch pulldn_en_cc1 and SWAPT_OUT_GPIO1  will be driven based
 * on
 *                        SWAP_CTRL0.SWAPT_TO_GPIO1_EN register
 */
#define PDSS_DEBUG_CC_1_CC1_PULLDN_CFG                      (1u << 21) /* <21:21> R:RW:0: */


/*
 * 0: If PFET300_CTRL.EN_SWITCH_CC2 is "1" then
 *                        300ma_switch pulldn_en_cc2 and SWAPT_OUT_GPIO2
 * will be driven "0"
 *     else
 *                        300ma_switch pulldn_en_cc2 and SWAPT_OUT_GPIO2
 *  will be driven based on
 *                        SWAP_CTRL0.SWAPT_TO_GPIO2_EN register
 * 1: 300ma_switch pulldn_en_cc2 and SWAPT_OUT_GPIO2  will be driven based
 * on
 *                        SWAP_CTRL0.SWAPT_TO_GPIO2_EN register
 */
#define PDSS_DEBUG_CC_1_CC2_PULLDN_CFG                      (1u << 22) /* <22:22> R:RW:0: */


/*
 * 0: Connect PFET300_PULLDN_EN_CC1 to 300ma_switch (pulldn_en_cc1 pins)
 * 1: Connect SWAPT to 300ma_switch (pulldn_en_cc1 pins)
 */
#define PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN                     (1u << 23) /* <23:23> R:RW:0: */


/*
 * 0: Connect PFET300_PULLDN_EN_CC2 to 300ma_switch (pulldn_en_cc2 pins)
 * 1: Connect SWAPT to 300ma_switch (pulldn_en_cc2 pins)
 */
#define PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN                     (1u << 24) /* <24:24> R:RW:0: */


/*
 * This bit is used only for Receive Extended Messages with the Chunk bit
 * set.
 * 0: Include the 2-byte extended data message header count
 * 1: Don't include the 2-byte extended data message header count
 */
#define PDSS_DEBUG_CC_1_INC_EXT_CHUNK_HDR_COUNT             (1u << 25) /* <25:25> R:RW:0: */


/*
 * This bit is used only for Receive Extended Messages with the Chunk bit
 * set.
 * 0: No gating
 * 1: Turn off the transmiter (set the following pins of s8usbpd_cc_top to
 * zero: tx_reg_en, tx_en, tx_data)
 */
#define PDSS_DEBUG_CC_1_STOP_TX_ON_SWAP                     (1u << 26) /* <26:26> R:RW:0: */


/*
 * If CTRL.RX_BYPASS_EN OR DEBUG_CC_0.LOOP_BACK_NO_BMC are set then the RX
 * processes will be based on Header Length.
 * 1: The RX processes the packets based on the EOP
 * 0: The RX processes the packets based on the Header length
 */
#define PDSS_DEBUG_CC_1_RX_EOP_BASED_EN                     (1u << 27) /* <27:27> R:RW:0: */


/*
 * This bit will disable the RX CC to generate the last rx_cc_bit_en and
 * RX State machine will create rx_cc_bit_en when the internal counter (BIT_EN_CNTR_CTRL.BIT_EN_CNTR)
 * is equal to BIT_EN_CNTR_CTRL.GEN_BIT_EN_CNTR
 */
#define PDSS_DEBUG_CC_1_DIS_CC_LAST_BIT_EN                  (1u << 28) /* <28:28> R:RW:0: */


/*
 * TBD
 */
#define PDSS_DEBUG_CC_1_EN_CRC_COLL_GO_CLR                  (1u << 29) /* <29:29> R:RW:0: */


/*
 * C-Connector Debug control register 2
 */
#define PDSS_DEBUG_CC_2_ADDRESS(m)                          (0x400a0144 + ((m) * (0x10000)))
#define PDSS_DEBUG_CC_2(m)                                  (*(volatile uint32_t *)(0x400a0144 + ((m) * 0x10000)))
#define PDSS_DEBUG_CC_2_DEFAULT                             (0x00ff0100)

/*
 * 0: delta value is subtracted from calculated UI period.
 * 1: delta value is added to calculated UI.
 */
#define PDSS_DEBUG_CC_2_RX_DELTA_POLARITY                   (1u << 0) /* <0:0> R:RW:0: */


/*
 * 1: Disable CC line monitoring while waiting for Auto GoodCRC response
 * for detection collision.
 * When set, Collision will be only checked once IDLE_COUNTER reaches it
 * programmed value.
 * 0: Enable CC line monitoring even when IDLE_COUNTER
 * is running. Any activity on CC line will trigger a collision.
 */
#define PDSS_DEBUG_CC_2_DIS_CC_MON_AUTO_CRC                 (1u << 1) /* <1:1> R:RW:0: */


/*
 * 1: If any EOP pattern is received during packet, Packet will be aborted.
 * 0: Idle condition/Normal packe-end on CC line will cause abort.
 */
#define PDSS_DEBUG_CC_2_RESET_RX_ON_EOP                     (1u << 2) /* <2:2> R:RW:0: */


/*
 *  Number of initial transitions to be ignored before Preamble averaging
 * starts and cc_rx_valid is raised.
 * Total of this register and NUM_TRANS_AVG should not be more than 63 decimal.
 */
#define PDSS_DEBUG_CC_2_RX_IGNR_TRANS_NUM_MASK              (0x00000fc0) /* <6:11> R:RW:4: */
#define PDSS_DEBUG_CC_2_RX_IGNR_TRANS_NUM_POS               (6)


/*
 * The bit wise mask bit for RX_EXPEC_GOODCRC_MSG.
 * This is a bit wise mask and if set, that particular bit is enabled for
 * comparison.
 */
#define PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_MASK           (0xffff0000) /* <16:31> R:RW:255: */
#define PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS            (16)


/*
 * SPARE6,7 Interrupt config
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_ADDRESS(m)                (0x400a0148 + ((m) * (0x10000)))
#define PDSS_DEBUG_SPARE_INTR_CFG(m)                        (*(volatile uint32_t *)(0x400a0148 + ((m) * 0x10000)))
#define PDSS_DEBUG_SPARE_INTR_CFG_DEFAULT                   (0x0000038e)

/*
 * Edge detect positive/negative enable/disable.
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE6_CFG_MASK           (0x00000003) /* <0:1> R:RW:2: */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE6_CFG_POS            (0)


/*
 * Filtering the positive edge of ocp_det from s8usbpd.:
 * 0: No Filtering
 * 1: Refer to SPARE6_FILT_SEL
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE6_FILT_EN            (1u << 2) /* <2:2> R:RW:1: */


/*
 * The positive edge of spare6 detection signal goes through a digital filter
 * which runs at CLK_LF.
 * If filtering is required, then SPARE6_FILT_EN needs to beset to "1".
 * 1: Filter for one CLK_LF
 * 2: Filter for two CLK_LF
 * 3: Filter for three CLK_LF
 * 0, 4:7: Reserved
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE6_FILT_SEL_MASK      (0x00000038) /* <3:5> R:RW:1: */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE6_FILT_SEL_POS       (3)


/*
 * Edge detect positive/negative enable/disable.
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE7_CFG_MASK           (0x000000c0) /* <6:7> R:RW:2: */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE7_CFG_POS            (6)


/*
 * Filtering the positive edge of ocp_det from s8usbpd.:
 * 0: No Filtering
 * 1: Refer to SPARE6_FILT_SEL
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE7_FILT_EN            (1u << 8) /* <8:8> R:RW:1: */


/*
 * The positive edge of spare7 detection signal goes through a digital filter
 * which runs at CLK_LF.
 * If filtering is required, then SPARE7_FILT_EN needs to beset to "1".
 * 1: Filter for one CLK_LF
 * 2: Filter for two CLK_LF
 * 3: Filter for three CLK_LF
 * 0, 4:7: Reserved
 */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE7_FILT_SEL_MASK      (0x00000e00) /* <9:11> R:RW:1: */
#define PDSS_DEBUG_SPARE_INTR_CFG_SPARE7_FILT_SEL_POS       (9)


/*
 * INTR2 Cause.  These are the active interrupts get reflected on interrupt_usbpd
 * pin.
 */
#define PDSS_INTR2_ADDRESS(m)                               (0x400a014c + ((m) * (0x10000)))
#define PDSS_INTR2(m)                                       (*(volatile uint32_t *)(0x400a014c + ((m) * 0x10000)))
#define PDSS_INTR2_DEFAULT                                  (0x00000000)

/*
 * The interrupt is raised at the end of UI calculation during preamble.
 */
#define PDSS_INTR2_UI_CAL_DONE                              (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * The interrupt is raised when small pulse as defined by MIN_PULSE_VALUE
 * is detected
 */
#define PDSS_INTR2_PULSE_MIN_VIOLATED                       (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * The interrupt is raised when large pulse as defined by MAX_PULSE_VALUE
 * is detected
 */
#define PDSS_INTR2_PULSE_MAX_VIOLATED                       (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * The interrupt is raised when HPD detects IRQ signaling on HPD line
 */
#define PDSS_INTR2_HPD_IRQ                                  (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * The interrupt is raised when HPD detects pluged condition
 */
#define PDSS_INTR2_HPD_PLUGED                               (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * The interrupt is raised when HPD detects unpluged condition
 */
#define PDSS_INTR2_HPD_UNPLUGED                             (1u << 5) /* <5:5> RW1S:RW1C:0: */


/*
 * The interrupt is raised when HPD detects undefined activity
 */
#define PDSS_INTR2_HPD_UNSTABLE                             (1u << 6) /* <6:6> RW1S:RW1C:0: */


/*
 * The interrupt is raised when an entry to HPD queue is made.
 */
#define PDSS_INTR2_HPD_QUEUE                                (1u << 7) /* <7:7> RW1S:RW1C:0: */


/*
 * The interrupt is raised when HPDT completes the command
 */
#define PDSS_INTR2_HPDT_COMMAND_DONE                        (1u << 8) /* <8:8> RW1S:RW1C:0: */


/*
 * The extended receive message detected
 */
#define PDSS_INTR2_EXTENDED_MSG_DET                         (1u << 9) /* <9:9> RW1S:RW1C:0: */


/*
 * The chunked-extended receive message detected
 */
#define PDSS_INTR2_CHUNK_DET                                (1u << 10) /* <10:10> RW1S:RW1C:0: */


/*
 * Hardware has passed writing the data to Half or End of the RX SRAM Memory
 * Location but CPU has not read the Data (RX_SRAM_HALF_EN not cleared bt
 * FW). This interrupt should be cleared only after the end of the RX packet.
 */
#define PDSS_INTR2_RX_SRAM_OVER_FLOW                        (1u << 11) /* <11:11> RW1S:RW1C:0: */


/*
 * Hardware has passed reading the data to Half or End of the TX SRAM Memory
 * Location but CPU has not wrote the Data (TX_SRAM_HALF_EN not cleared bt
 * FW). This interrupt should be cleared only after the end of the TX packet.
 */
#define PDSS_INTR2_TX_SRAM_UNDER_FLOW                       (1u << 12) /* <12:12> RW1S:RW1C:0: */


/*
 * SPARE3
 */
#define PDSS_INTR2_SPARE3                                   (1u << 13) /* <13:13> RW1S:RW1C:0: */


/*
 * SPARE4
 */
#define PDSS_INTR2_SPARE4                                   (1u << 14) /* <14:14> RW1S:RW1C:0: */


/*
 * SPARE5 TX (SRAM_HALF)
 */
#define PDSS_INTR2_SPARE5                                   (1u << 15) /* <15:15> RW1S:RW1C:0: */


/*
 * NGDO C Spacing is complete
 */
#define PDSS_INTR2_NGDO_C_SPACING_DONE                      (1u << 16) /* <16:16> RW1S:RW1C:0: */


/*
 * NGDO P Spacing is complete
 */
#define PDSS_INTR2_NGDO_P_SPACING_DONE                      (1u << 17) /* <17:17> RW1S:RW1C:0: */


/*
 * VSWAP and VBUS_C less that 5V is detected
 */
#define PDSS_INTR2_VSWAP_VBUS_LESS_5_DONE                   (1u << 20) /* <20:20> RW1S:RW1C:0: */


/*
 * The interrupt is raised when SWAP detects an interrupt
 */
#define PDSS_INTR2_SWAP_IRQ                                 (1u << 21) /* <21:21> RW1S:RW1C:0: */


/*
 * The interrupt is raised when SWAP detects pluged condition
 */
#define PDSS_INTR2_SWAP_PLUGED                              (1u << 22) /* <22:22> RW1S:RW1C:0: */


/*
 * The interrupt is raised when SWAP detects unpluged condition
 */
#define PDSS_INTR2_SWAP_UNPLUGED                            (1u << 23) /* <23:23> RW1S:RW1C:0: */


/*
 * The interrupt is raised when SWAP detects undefined activity
 */
#define PDSS_INTR2_SWAP_UNSTABLE                            (1u << 24) /* <24:24> RW1S:RW1C:0: */


/*
 * The interrupt is raised when an entry to SWAP queue is made.
 */
#define PDSS_INTR2_SWAP_QUEUE                               (1u << 25) /* <25:25> RW1S:RW1C:0: */


/*
 * The interrupt is raised when SWAP completes the command
 */
#define PDSS_INTR2_SWAP_COMMAND_DONE                        (1u << 26) /* <26:26> RW1S:RW1C:0: */


/*
 * INTR2 Set
 */
#define PDSS_INTR2_SET_ADDRESS(m)                           (0x400a0150 + ((m) * (0x10000)))
#define PDSS_INTR2_SET(m)                                   (*(volatile uint32_t *)(0x400a0150 + ((m) * 0x10000)))
#define PDSS_INTR2_SET_DEFAULT                              (0x00000000)

/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_UI_CAL_DONE                          (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_PULSE_MIN_VIOLATED                   (1u << 1) /* <1:1> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_PULSE_MAX_VIOLATED                   (1u << 2) /* <2:2> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_HPD_IRQ                              (1u << 3) /* <3:3> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_HPD_PLUGED                           (1u << 4) /* <4:4> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_HPD_UNPLUGED                         (1u << 5) /* <5:5> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_HPD_UNSTABLE                         (1u << 6) /* <6:6> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_HPD_QUEUE                            (1u << 7) /* <7:7> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_HPDT_COMMAND_DONE                    (1u << 8) /* <8:8> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_EXTENDED_MSG_DET                     (1u << 9) /* <9:9> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_CHUNK_DET                            (1u << 10) /* <10:10> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_RX_SRAM_OVER_FLOW                    (1u << 11) /* <11:11> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_TX_SRAM_UNDER_FLOW                   (1u << 12) /* <12:12> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SPARE3                               (1u << 13) /* <13:13> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SPARE4                               (1u << 14) /* <14:14> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SPARE5                               (1u << 15) /* <15:15> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_NGDO_C_SPACING_DONE                  (1u << 16) /* <16:16> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_NGDO_P_SPACING_DONE                  (1u << 17) /* <17:17> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_VSWAP_VBUS_LESS_5_DONE               (1u << 20) /* <20:20> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SWAP_IRQ                             (1u << 21) /* <21:21> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SWAP_PLUGED                          (1u << 22) /* <22:22> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SWAP_UNPLUGED                        (1u << 23) /* <23:23> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SWAP_UNSTABLE                        (1u << 24) /* <24:24> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SWAP_QUEUE                           (1u << 25) /* <25:25> A:RW1S:0: */


/*
 * Write with '1' to set corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_SET_SWAP_COMMAND_DONE                    (1u << 26) /* <26:26> A:RW1S:0: */


/*
 * INTR2 Mask
 */
#define PDSS_INTR2_MASK_ADDRESS(m)                          (0x400a0154 + ((m) * (0x10000)))
#define PDSS_INTR2_MASK(m)                                  (*(volatile uint32_t *)(0x400a0154 + ((m) * 0x10000)))
#define PDSS_INTR2_MASK_DEFAULT                             (0x00000000)

/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_UI_CAL_DONE_MASK                    (1u << 0) /* <0:0> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_PULSE_MIN_VIOLATED_MASK             (1u << 1) /* <1:1> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_PULSE_MAX_VIOLATED_MASK             (1u << 2) /* <2:2> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_HPD_IRQ_MASK                        (1u << 3) /* <3:3> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_HPD_PLUGED_MASK                     (1u << 4) /* <4:4> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_HPD_UNPLUGED_MASK                   (1u << 5) /* <5:5> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_HPD_UNSTABLE_MASK                   (1u << 6) /* <6:6> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_HPD_QUEUE_MASK                      (1u << 7) /* <7:7> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK              (1u << 8) /* <8:8> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_EXTENDED_MSG_DET_MASK               (1u << 9) /* <9:9> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_CHUNK_DET_MASK                      (1u << 10) /* <10:10> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_RX_SRAM_OVER_FLOW_MASK              (1u << 11) /* <11:11> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_TX_SRAM_UNDER_FLOW_MASK             (1u << 12) /* <12:12> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SPARE3_MASK                         (1u << 13) /* <13:13> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SPARE4_MASK                         (1u << 14) /* <14:14> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SPARE5_MASK                         (1u << 15) /* <15:15> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_NGDO_C_SPACING_DONE_MASK            (1u << 16) /* <16:16> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_NGDO_P_SPACING_DONE_MASK            (1u << 17) /* <17:17> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_VSWAP_VBUS_LESS_5_DONE_MASK         (1u << 20) /* <20:20> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SWAP_IRQ_MASK                       (1u << 21) /* <21:21> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SWAP_PLUGED_MASK                    (1u << 22) /* <22:22> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SWAP_UNPLUGED_MASK                  (1u << 23) /* <23:23> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SWAP_UNSTABLE_MASK                  (1u << 24) /* <24:24> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SWAP_QUEUE_MASK                     (1u << 25) /* <25:25> R:RW:0: */


/*
 * Mask bit for corresponding bit in interrupt request register.
 */
#define PDSS_INTR2_MASK_SWAP_COMMAND_DONE_MASK              (1u << 26) /* <26:26> R:RW:0: */


/*
 * INTR2 Masked
 */
#define PDSS_INTR2_MASKED_ADDRESS(m)                        (0x400a0158 + ((m) * (0x10000)))
#define PDSS_INTR2_MASKED(m)                                (*(volatile uint32_t *)(0x400a0158 + ((m) * 0x10000)))
#define PDSS_INTR2_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_UI_CAL_DONE_MASKED                (1u << 0) /* <0:0> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_PULSE_MIN_VIOLATED_MASKED         (1u << 1) /* <1:1> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_PULSE_MAX_VIOLATED_MASKED         (1u << 2) /* <2:2> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_HPD_IRQ_MASKED                    (1u << 3) /* <3:3> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_HPD_PLUGED_MASKED                 (1u << 4) /* <4:4> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_HPD_UNPLUGED_MASKED               (1u << 5) /* <5:5> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_HPD_UNSTABLE_MASKED               (1u << 6) /* <6:6> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_HPD_QUEUE_MASKED                  (1u << 7) /* <7:7> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_HPDT_COMMAND_DONE_MASKED          (1u << 8) /* <8:8> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_EXTENDED_MSG_DET_MASKED           (1u << 9) /* <9:9> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_CHUNK_DET_MASKED                  (1u << 10) /* <10:10> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_RX_SRAM_OVER_FLOW_MASKED          (1u << 11) /* <11:11> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_TX_SRAM_UNDER_FLOW_MASKED         (1u << 12) /* <12:12> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SPARE3_MASKED                     (1u << 13) /* <13:13> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SPARE4_MASKED                     (1u << 14) /* <14:14> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SPARE5_MASKED                     (1u << 15) /* <15:15> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_NGDO_C_SPACING_DONE_MASKED        (1u << 16) /* <16:16> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_NGDO_P_SPACING_DONE_MASKED        (1u << 17) /* <17:17> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_VSWAP_VBUS_LESS_5_DONE_MASKED     (1u << 20) /* <20:20> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SWAP_IRQ_MASKED                   (1u << 21) /* <21:21> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SWAP_PLUGED_MASKED                (1u << 22) /* <22:22> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SWAP_UNPLUGED_MASKED              (1u << 23) /* <23:23> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SWAP_UNSTABLE_MASKED              (1u << 24) /* <24:24> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SWAP_QUEUE_MASKED                 (1u << 25) /* <25:25> RW:R:0: */


/*
 * Logical and of corresponding request and mask bits.
 */
#define PDSS_INTR2_MASKED_SWAP_COMMAND_DONE_MASKED          (1u << 26) /* <26:26> RW:R:0: */


/*
 * Configures HPD module, IRQ_MIN, IRQ_MAX and ENABLE
 */
#define PDSS_HPD_CTRL1_ADDRESS(m)                           (0x400a015c + ((m) * (0x10000)))
#define PDSS_HPD_CTRL1(m)                                   (*(volatile uint32_t *)(0x400a015c + ((m) * 0x10000)))
#define PDSS_HPD_CTRL1_DEFAULT                              (0x8025812c)

/*
 * Defines the min width of IRQ pulse. Default = 0.5 ms. Assumes 600 Khz
 * clock.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL1_IRQ_MIN_MASK                         (0x00000fff) /* <0:11> R:RW:300: */
#define PDSS_HPD_CTRL1_IRQ_MIN_POS                          (0)


/*
 * Defines the max width of IRQ pulse. Default = 1.0 ms. Assumes 600 Khz
 * clock.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL1_IRQ_MAX_MASK                         (0x00fff000) /* <12:23> R:RW:600: */
#define PDSS_HPD_CTRL1_IRQ_MAX_POS                          (12)


/*
 * When set it flushes the queue.
 */
#define PDSS_HPD_CTRL1_FLUSH_QUEUE                          (1u << 24) /* <24:24> R:RW:0: */


/*
 * When set, input to hpd module will come from output of hpdt module.
 */
#define PDSS_HPD_CTRL1_LOOPBACK_EN                          (1u << 30) /* <30:30> R:RW:0: */


/*
 * When set, HPD state machine is at Idle. This bit must be set to zero for
 * HPD module to operate.
 */
#define PDSS_HPD_CTRL1_RESET_HPD_STATE                      (1u << 31) /* <31:31> R:RW:1: */


/*
 * Configures HPD module, Glitch width.
 */
#define PDSS_HPD_CTRL2_ADDRESS(m)                           (0x400a0160 + ((m) * (0x10000)))
#define PDSS_HPD_CTRL2(m)                                   (*(volatile uint32_t *)(0x400a0160 + ((m) * 0x10000)))
#define PDSS_HPD_CTRL2_DEFAULT                              (0x00096096)

/*
 * Defines the width of glitch. Default = 0.25 ms. Assumes clk of 600 Khz
 * on the rising edge of hpd_in input
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL2_GLITCH_WIDTH_HIGH_MASK               (0x00000fff) /* <0:11> R:RW:150: */
#define PDSS_HPD_CTRL2_GLITCH_WIDTH_HIGH_POS                (0)


/*
 * Defines the width of glitch. Default = 0.25 ms. Assumes clk of 600 Khz
 * on the falling edge of hpd_in input
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL2_GLITCH_WIDTH_LOW_MASK                (0x00fff000) /* <12:23> R:RW:150: */
#define PDSS_HPD_CTRL2_GLITCH_WIDTH_LOW_POS                 (12)


/*
 * Configures HPD module, stable high and stable low values
 */
#define PDSS_HPD_CTRL3_ADDRESS(m)                           (0x400a0164 + ((m) * (0x10000)))
#define PDSS_HPD_CTRL3(m)                                   (*(volatile uint32_t *)(0x400a0164 + ((m) * 0x10000)))
#define PDSS_HPD_CTRL3_DEFAULT                              (0x0004b04b)

/*
 * Defines the minimum time before plugged condition is detected for subsequent
 * connect.  Default = 2 ms. Assumes 600 Khz clock. This value is internally
 * multiplied by 16.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL3_STABLE_HIGH_MASK                     (0x00000fff) /* <0:11> R:RW:75: */
#define PDSS_HPD_CTRL3_STABLE_HIGH_POS                      (0)


/*
 * Defines the minimum time before unplugged condition is detected for subsequent
 * connect.  Default = 2 ms. Assumes 600 Khz clock. This value is internally
 * multiplied by 16.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL3_STABLE_LOW_MASK                      (0x00fff000) /* <12:23> R:RW:75: */
#define PDSS_HPD_CTRL3_STABLE_LOW_POS                       (12)


/*
 * Configures HPD module, irq spacing and unplug event
 */
#define PDSS_HPD_CTRL4_ADDRESS(m)                           (0x400a0168 + ((m) * (0x10000)))
#define PDSS_HPD_CTRL4(m)                                   (*(volatile uint32_t *)(0x400a0168 + ((m) * 0x10000)))
#define PDSS_HPD_CTRL4_DEFAULT                              (0x000004b0)

/*
 * Defines the minimum spacing between interrupts  Default = 2 ms. Assumes
 * 600 Khz clock.  If spacing of interrupts received is less than this value,
 * they will be ignored.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL4_IRQ_SPACING_MASK                     (0x00000fff) /* <0:11> R:RW:1200: */
#define PDSS_HPD_CTRL4_IRQ_SPACING_POS                      (0)


/*
 * Configures HPD module, Initial LOW and HIGH times
 */
#define PDSS_HPD_CTRL5_ADDRESS(m)                           (0x400a016c + ((m) * (0x10000)))
#define PDSS_HPD_CTRL5(m)                                   (*(volatile uint32_t *)(0x400a016c + ((m) * 0x10000)))
#define PDSS_HPD_CTRL5_DEFAULT                              (0x0ea50ea5)

/*
 * Defines the minimum time before plugged condion is detected for initial
 * connect.  Default = 100 ms. Assumes 600 Khz clock. This value is internally
 * multiplied by 16.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL5_LONG_HIGH_MASK                       (0x0000ffff) /* <0:15> R:RW:3749: */
#define PDSS_HPD_CTRL5_LONG_HIGH_POS                        (0)


/*
 * Defines the minimum width before unplugged condition is detected.  Default
 * = 100 ms. Assumes 600 Khz clock. This value is internally multiplied by
 * 16..  After LONG_LOW condition is detected, HPD input is expected to stay
 * high equal to LONG_HIGH before plugged condition is reached. This is the
 * optional 100 ms debounce time
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPD_CTRL5_LONG_LOW_MASK                        (0xffff0000) /* <16:31> R:RW:3749: */
#define PDSS_HPD_CTRL5_LONG_LOW_POS                         (16)


/*
 * HPD queue register
 */
#define PDSS_HPD_QUEUE_ADDRESS(m)                           (0x400a0170 + ((m) * (0x10000)))
#define PDSS_HPD_QUEUE(m)                                   (*(volatile uint32_t *)(0x400a0170 + ((m) * 0x10000)))
#define PDSS_HPD_QUEUE_DEFAULT                              (0x00000000)

/*
 * This is a read only register. Reading this register will clear the queue
 * in HW. This queue holds 4 locations.
 * Location 0 : bits 1:0.
 * Location 1 : bits 3:2.
 * Location 2 : bits 5:4.
 * Location 3 : bits 7:6.
 * This queue provides a FIFO function. First location written by HW is location
 * 0. The last location is location 3. HW will not write once this queue
 * is full. Reading the Queue will clear the contents. Reading an empty queue
 * will not cause FIFO under run.
 * This queue will hold hpd input events. These events are PLUGGED, UNPLUGGED
 * and IRQ.  Only 2 back to back IRQ will be written into the queue and additional
 * ones will be dropped. When UNPLUGGED is detected, the event will always
 * be written to location 0 of the queue and locations 1 to 3 will be cleared.
 *  The following codes are used to encode events in the queue.
 * 00  : Empty location.
 * 01  : UNPLUGGED.
 * 10  : PLUGGED.
 * 11  : IRQ.
 *
 * Hardware will clear this register when CPU reads it.
 */
#define PDSS_HPD_QUEUE_DATA_OUT_MASK                        (0x000000ff) /* <0:7> RW:R:0: */
#define PDSS_HPD_QUEUE_DATA_OUT_POS                         (0)


/*
 * HPDT module configure and IRQ width and delay parameters
 */
#define PDSS_HPDT_CTRL1_ADDRESS(m)                          (0x400a0174 + ((m) * (0x10000)))
#define PDSS_HPDT_CTRL1(m)                                  (*(volatile uint32_t *)(0x400a0174 + ((m) * 0x10000)))
#define PDSS_HPDT_CTRL1_DEFAULT                             (0x404af1c2)

/*
 * Defines the width of IRQ pulse in muliples of 600KHZ clock period. This
 * value sets the width at 0.75 ms.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPDT_CTRL1_IRQ_WIDTH_MASK                      (0x00000fff) /* <0:11> R:RW:450: */
#define PDSS_HPDT_CTRL1_IRQ_WIDTH_POS                       (0)


/*
 * Defines the delay to execute IRQ command in multiples of 600KHZ clock
 * period. The default value is 2 ms.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPDT_CTRL1_SET_IRQ_DELAY_MASK                  (0x00fff000) /* <12:23> R:RW:1199: */
#define PDSS_HPDT_CTRL1_SET_IRQ_DELAY_POS                   (12)


/*
 * Defines the initial level of the hpdt output.
 */
#define PDSS_HPDT_CTRL1_DEFAULT_LEVEL                       (1u << 24) /* <24:24> R:RW:0: */


/*
 * Source of data to hpdt_out signal. 0: comes from hpdt module.  1: comes
 * from DATA_SOURCE bit of this register
 */
#define PDSS_HPDT_CTRL1_DATA_SOURCE_SELECT                  (1u << 25) /* <25:25> R:RW:0: */


/*
 * Data source of hpdt_out output when DATA_SOURCE_SELECT is set high
 */
#define PDSS_HPDT_CTRL1_DATA_SOURCE                         (1u << 26) /* <26:26> R:RW:0: */


/*
 * Defines the commands to be executed by hpdt module.
 * 0: set hpdt_out to low.
 * 1: set hpdt_out to high.
 * 2: send an IRQ.
 * 3: unused.
 * Note: Send IRQ command does not set GPIO to high level before driving
 * IRQ pulse low. It assumes IRQ is being sent to a plugged device all the
 * time which would mean the GPIO is already at high level.  An IRQ command
 * is received before GPIO is driven to a plugged level, creates an error
 * condition that must be filtered by CPU.
 */
#define PDSS_HPDT_CTRL1_COMMAND_MASK                        (0x18000000) /* <27:28> R:RW:0: */
#define PDSS_HPDT_CTRL1_COMMAND_POS                         (27)


/*
 * When set, the commands specified by COMMAND bits will be executed. Execution
 * happens on the rising edge of this bit.
 * FW should not clear this bit until INTR2.HPDT_COMMAND_DONE interrupt is
 * set.
 * Hardware clears this bit when command_done is set.
 * Clearing of this command by FW is not necessary and is not recommended.
 */
#define PDSS_HPDT_CTRL1_COMMAND_START                       (1u << 29) /* <29:29> RW1C:RW:0: */


/*
 * When set, HPDT state machine is at Idle
 * This bit must be reset to zero for the module to operate.
 */
#define PDSS_HPDT_CTRL1_RESET_HPDT_STATE                    (1u << 30) /* <30:30> R:RW:1: */


/*
 * Defines the HIGH & LOW transition delays.
 */
#define PDSS_HPDT_CTRL2_ADDRESS(m)                          (0x400a0178 + ((m) * (0x10000)))
#define PDSS_HPDT_CTRL2(m)                                  (*(volatile uint32_t *)(0x400a0178 + ((m) * 0x10000)))
#define PDSS_HPDT_CTRL2_DEFAULT                             (0x014af4af)

/*
 * Defines the delay from HIGH -> LOW transition.  In multiples of 600KHZ
 * clock period. The defaul value is 2 ms.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPDT_CTRL2_SET_LOW_DELAY_MASK                  (0x00000fff) /* <0:11> R:RW:1199: */
#define PDSS_HPDT_CTRL2_SET_LOW_DELAY_POS                   (0)


/*
 * Defines the delay from LOW --> HIGH transition.  In multiples of 600KHZ
 * clock period. The defaul value is 2 ms.
 * This register should be adjusted when a different value is needed.
 */
#define PDSS_HPDT_CTRL2_SET_HIGH_DELAY_MASK                 (0x00fff000) /* <12:23> R:RW:1199: */
#define PDSS_HPDT_CTRL2_SET_HIGH_DELAY_POS                  (12)


/*
 * Defines the default level of HPDT transmit OE.
 * 0: HPD output goes IDLE (OE=0) when module is not selected or module is
 * IDLE state
 * 1: HPD output stays active.
 */
#define PDSS_HPDT_CTRL2_DEFAULT_OE                          (1u << 24) /* <24:24> R:RW:1: */


/*
 * Defines the source of HPD pin OE.
 * 0: Driven by HW
 * 1: Driven by bit CTRL.HPD_DIRECTION
 */
#define PDSS_HPDT_CTRL2_OE_SOURCE_SELECT                    (1u << 25) /* <25:25> R:RW:0: */


/*
 * Selects which pins to appear on HPDT debug bus
 *      2'h1     : debug_out   <= count_12_bits_q[3:0];
 *     2'h2     : debug_out   <= count_12_bits_q[7:4];
 *     2'h3     : debug_out   <= count_12_bits_q[11:8];
 *     default  : debug_out   <= {1'h0, state_q};
 */
#define PDSS_HPDT_CTRL2_HPDT_DEBUG_SEL_MASK                 (0x0c000000) /* <26:27> R:RW:0: */
#define PDSS_HPDT_CTRL2_HPDT_DEBUG_SEL_POS                  (26)


/*
 * The output of debug bus.
 */
#define PDSS_HPDT_CTRL2_HPDT_DEBUG_OUT_MASK                 (0xf0000000) /* <28:31> RW:R:0: */
#define PDSS_HPDT_CTRL2_HPDT_DEBUG_OUT_POS                  (28)


/*
 * ADC1 SAR Control Register
 * General Purpose voltgae measurement, Temperature Sceining
 */
#define PDSS_ADC1_SAR_CTRL_ADDRESS(m)                       (0x400a017c + ((m) * (0x10000)))
#define PDSS_ADC1_SAR_CTRL(m)                               (*(volatile uint32_t *)(0x400a017c + ((m) * 0x10000)))
#define PDSS_ADC1_SAR_CTRL_DEFAULT                          (0x00008000)

/*
 * Setting this bit will enable the HW SAR logic.
 * Once the SAR_EN is one, Hardware will update the  SAR_OUT register after
 * 8 cycles of clk_sar and clear this register.
 */
#define PDSS_ADC1_SAR_CTRL_SAR_EN                           (1u << 0) /* <0:0> RW1C:RW1S:0: */


/*
 * ADC starting mid value
 */
#define PDSS_ADC1_SAR_CTRL_MID_VAL_MASK                     (0x0000ff00) /* <8:15> R:RW:128: */
#define PDSS_ADC1_SAR_CTRL_MID_VAL_POS                      (8)


/*
 * ADC output resistance value
 * Stored 8-bit ADC value after the ID Pin voltage is sampled.
 */
#define PDSS_ADC1_SAR_CTRL_SAR_OUT_MASK                     (0x00ff0000) /* <16:23> RW:R:0: */
#define PDSS_ADC1_SAR_CTRL_SAR_OUT_POS                      (16)


/*
 * This register selects which edge of cmp_out pin of the s8usbpd_adc_top
 * should be used to generate INT0.CMP_OUT_DET
 */
#define PDSS_ADC1_SAR_CTRL_INTR_CMP_SEL_MASK                (0x03000000) /* <24:25> R:RW:0: */
#define PDSS_ADC1_SAR_CTRL_INTR_CMP_SEL_POS                 (24)


/*
 * The CMP_OUT to SOC GPIO can be selected to go through a digital filter
 * which runs at CLK_LF.
 * If direct connection to GPIO is needed then INTR_1_CFG.CMP_OUT1_FILT_EN
 * needs to be set to "0".
 * If filtering is required, then INTR_1_CFG.CMP_OUT1_FILT_EN needs to beset
 * to "1".
 * 1: Filter for one CLK_LF
 * 2: Filter for two CLK_LF
 * 3: Filter for three CLK_LF
 * 0, 4:7: Reserved
 */
#define PDSS_ADC1_SAR_CTRL_CMP_OUT_TO_GPIO_FILT_SEL_MASK    (0x1c000000) /* <26:28> R:RW:0: */
#define PDSS_ADC1_SAR_CTRL_CMP_OUT_TO_GPIO_FILT_SEL_POS     (26)


/*
 * S8USBPD DAC1 Control Register
 */
#define PDSS_ADC1_CTRL_ADDRESS(m)                           (0x400a0180 + ((m) * (0x10000)))
#define PDSS_ADC1_CTRL(m)                                   (*(volatile uint32_t *)(0x400a0180 + ((m) * 0x10000)))
#define PDSS_ADC1_CTRL_DEFAULT                              (0x80000200)

/*
 * Control bits for 8-bit DAC.
 * DAC_CNTRL register is used only if CPU wants to implement the SAR algorithm
 * in FW.
 */
#define PDSS_ADC1_CTRL_DAC_CNTRL_MASK                       (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_ADC1_CTRL_DAC_CNTRL_POS                        (0)


/*
 * ADC DFT Control:
 * 0: Normal operation
 * 1: DAC output voltage
 */
#define PDSS_ADC1_CTRL_DFT_MUXSEL                           (1u << 8) /* <8:8> R:RW:0: */


/*
 * This is for when high voltage supply for a port is not present. This bit
 * should be set when the high voltage is present,
 * in order to ensure that the outputs are set to know values.
 * 0: All outputs are isolated to a known value
 * 1: Normal operation
 */
#define PDSS_ADC1_CTRL_ADC_ISO_N                            (1u << 9) /* <9:9> R:RW:1: */


/*
 * Comparator Output.  If voltage on ID pin is less than DAC voltage, then
 * cmp_out is HIGH.
 */
#define PDSS_ADC1_CTRL_CMP_OUT                              (1u << 15) /* <15:15> RW:R:0: */


/*
 * Input Voltage select
 */
#define PDSS_ADC1_CTRL_VSEL_MASK                            (0x00060000) /* <17:18> R:RW:0: */
#define PDSS_ADC1_CTRL_VSEL_POS                             (17)


/*
 * ADC Power down control, active high.
 */
#define PDSS_ADC1_CTRL_PD_LV                                (1u << 31) /* <31:31> R:RW:1: */


/*
 * ADC12SAR Control Register
 * General Purpose voltgae measurement, Temperature Sceining
 */
#define PDSS_ADC2_SAR_CTRL_ADDRESS(m)                       (0x400a0184 + ((m) * (0x10000)))
#define PDSS_ADC2_SAR_CTRL(m)                               (*(volatile uint32_t *)(0x400a0184 + ((m) * 0x10000)))
#define PDSS_ADC2_SAR_CTRL_DEFAULT                          (0x00008000)

/*
 * Setting this bit will enable the HW SAR logic.
 * Once the SAR_EN is one, Hardware will update the  SAR_OUT register after
 * 8 cycles of clk_sar and clear this register.
 */
#define PDSS_ADC2_SAR_CTRL_SAR_EN                           (1u << 0) /* <0:0> RW1C:RW1S:0: */


/*
 * ADC starting mid value
 */
#define PDSS_ADC2_SAR_CTRL_MID_VAL_MASK                     (0x0000ff00) /* <8:15> R:RW:128: */
#define PDSS_ADC2_SAR_CTRL_MID_VAL_POS                      (8)


/*
 * ADC output resistance value
 * Stored 8-bit ADC value after the ID Pin voltage is sampled.
 */
#define PDSS_ADC2_SAR_CTRL_SAR_OUT_MASK                     (0x00ff0000) /* <16:23> RW:R:0: */
#define PDSS_ADC2_SAR_CTRL_SAR_OUT_POS                      (16)


/*
 * This register selects which edge of cmp_out pin of the s8usbpd_adc_top
 * should be used to generate INT0.CMP_OUT_DET
 */
#define PDSS_ADC2_SAR_CTRL_INTR_CMP_SEL_MASK                (0x03000000) /* <24:25> R:RW:0: */
#define PDSS_ADC2_SAR_CTRL_INTR_CMP_SEL_POS                 (24)


/*
 * The CMP_OUT to SOC GPIO can be selected to go through a digital filter
 * which runs at CLK_LF.
 * If direct connection to GPIO is needed then INTR_1_CFG.CMP_OUT2_FILT_EN
 * needs to be set to "0".
 * If filtering is required, then INTR_1_CFG.CMP_OUT2_FILT_EN needs to beset
 * to "1".
 * 1: Filter for one CLK_LF
 * 2: Filter for two CLK_LF
 * 3: Filter for three CLK_LF
 * 0, 4:7: Reserved
 */
#define PDSS_ADC2_SAR_CTRL_CMP_OUT_TO_GPIO_FILT_SEL_MASK    (0x1c000000) /* <26:28> R:RW:0: */
#define PDSS_ADC2_SAR_CTRL_CMP_OUT_TO_GPIO_FILT_SEL_POS     (26)


/*
 * S8USBPD DAC2 Control Register
 */
#define PDSS_ADC2_CTRL_ADDRESS(m)                           (0x400a0188 + ((m) * (0x10000)))
#define PDSS_ADC2_CTRL(m)                                   (*(volatile uint32_t *)(0x400a0188 + ((m) * 0x10000)))
#define PDSS_ADC2_CTRL_DEFAULT                              (0x80000200)

/*
 * Control bits for 8-bit DAC.
 * DAC_CNTRL register is used only if CPU wants to implement the SAR algorithm
 * in FW.
 */
#define PDSS_ADC2_CTRL_DAC_CNTRL_MASK                       (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_ADC2_CTRL_DAC_CNTRL_POS                        (0)


/*
 * ADC DFT Control:
 * 0: Normal operation
 * 1: DAC output voltage
 */
#define PDSS_ADC2_CTRL_DFT_MUXSEL                           (1u << 8) /* <8:8> R:RW:0: */


/*
 * This is for when high voltage supply for a port is not present. This bit
 * should be set when the high voltage is present,
 * in order to ensure that the outputs are set to know values.
 * 0: All outputs are isolated to a known value
 * 1: Normal operation
 */
#define PDSS_ADC2_CTRL_ADC_ISO_N                            (1u << 9) /* <9:9> R:RW:1: */


/*
 * Comparator Output.  If voltage on ID pin is less than DAC voltage, then
 * cmp_out is HIGH.
 */
#define PDSS_ADC2_CTRL_CMP_OUT                              (1u << 15) /* <15:15> RW:R:0: */


/*
 * Input Voltage select
 */
#define PDSS_ADC2_CTRL_VSEL_MASK                            (0x00060000) /* <17:18> R:RW:0: */
#define PDSS_ADC2_CTRL_VSEL_POS                             (17)


/*
 * ADC Power down control, active high.
 */
#define PDSS_ADC2_CTRL_PD_LV                                (1u << 31) /* <31:31> R:RW:1: */


/*
 * S8USBPD C-connector Control Register 0
 */
#define PDSS_CC_CTRL_0_ADDRESS(m)                           (0x400a018c + ((m) * (0x10000)))
#define PDSS_CC_CTRL_0(m)                                   (*(volatile uint32_t *)(0x400a018c + ((m) * 0x10000)))
#define PDSS_CC_CTRL_0_DEFAULT                              (0xb0000000)

/*
 * FW can only use this bit when the DEBUG_CC_0.TX_CC_DRIVE_SRC is set to
 * "1".
 * 0: Disables the Transceiver to transmit data
 * 1: Enables the Transceiver to transmit data
 * Notes: This bit is not needed for normal data transmit . This is reqiired
 * only for manual data tranmit .
 *
 */
#define PDSS_CC_CTRL_0_TX_EN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Enables the Transceiver to receive data, Active High
 * This bit should be set after CC Line active interrupt (wakeup) in DeepSleep.
 * FW should set this bit at init and not change after across deep sleep
 * and wake.
 * Notes: DFP_unatached : 1'b0
 *            DFP attached    :  1'b1
 *            UFP , Cable       : 1'b1  (It can se set to 1 from beginning
 * )
 */
#define PDSS_CC_CTRL_0_RX_EN                                (1u << 1) /* <1:1> R:RW:0: */


/*
 * Firmware detects cable attach and specifies whether CC1 or CC2 is connected
 * to the CC-line of the cable.
 * 0 - CC1
 * 1 - CC2
 * Notes:
 * DFP: 0 or 1 based on the plug orientation .
 * UFP: 0 or 1 based on the plug orientation.
 * AMA: 0 or 1 based on board , which ever line is being used for communication.
 * Cable: 0  (Only CC1 line is used)
 */
#define PDSS_CC_CTRL_0_CC_1V2                               (1u << 2) /* <2:2> R:RW:0: */


/*
 * CC line voltage Comparator enable
 * Notes: Set this bit as need basis . No need to set for DFP unattachd .
 */
#define PDSS_CC_CTRL_0_CMP_EN                               (1u << 3) /* <3:3> R:RW:0: */


/*
 * Connects cmp_dn comparator to CC1/CC2
 * 0 - CC1
 * 1 - CC2
 */
#define PDSS_CC_CTRL_0_CMP_DN_CC1V2                         (1u << 4) /* <4:4> R:RW:0: */


/*
 * Selects the voltage threshold for cmp_dn comparator
 * Notes: Pair of DN and UP comprator should be used to determine remote
 * Rp value when acting as UFP. Firmware can set DN comparator to 0.655V
 * in UFP mode. In DFP mode firmware can use this comparator to detect detach
 * by selecting 2.6V reference .
 */
#define PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK                     (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_DN_VSEL_POS                      (5)


/*
 * Connects cmp_up comparator to CC1/CC2
 * 0 - CC1
 * 1 - CC2
 */
#define PDSS_CC_CTRL_0_CMP_UP_CC1V2                         (1u << 8) /* <8:8> R:RW:0: */


/*
 * Selects the voltage threshold for cmp_up comparator.
 * Notes: Pair of DN and UP comprator should be used to determine remote
 * Rp value when acting as UFP. Firmware can set UP comparator to 1.235V
 * in UFP mode. In DFP mode firmware can use this comparator to detect detach
 * by selecting 2.6V reference .
 */
#define PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK                     (0x00000e00) /* <9:11> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_UP_VSEL_POS                      (9)


/*
 * Selects the cmp_up comparator offset:
 */
#define PDSS_CC_CTRL_0_CMP_UP_OFFSET_MASK                   (0x00007000) /* <12:14> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_UP_OFFSET_POS                    (12)


/*
 * Enables the offset generator for the cmp_up comparator
 * 0 - no offset
 * 1 - offset enabled, see CMP_UP_OFFSET register for value
 */
#define PDSS_CC_CTRL_0_CMP_UP_OFFSET_EN                     (1u << 15) /* <15:15> R:RW:0: */


/*
 * No effect
 */
#define PDSS_CC_CTRL_0_CMP_LA_CC1V2                         (1u << 16) /* <16:16> R:RW:0: */


/*
 * No effect. Spare register
 */
#define PDSS_CC_CTRL_0_CMP_LA_VSEL_MASK                     (0x000e0000) /* <17:19> R:RW:0: */
#define PDSS_CC_CTRL_0_CMP_LA_VSEL_POS                      (17)


/*
 * Disable Dead Battery Rd termination on CC1
 * Notes:
 * DFP  (Unattached and attached) : 1'b1
 * UFP_dead : 1'b0
 * UFP_attached: 1'b1
 * Cable: Don't care
 */
#define PDSS_CC_CTRL_0_RD_CC1_DB_DIS                        (1u << 20) /* <20:20> R:RW:0: */


/*
 * Disable Dead Battery Rd termination on CC2
 * Notes:
 * DFP  (Unattached and attached) : 1'b1
 * UFP_dead : 1'b0
 * UFP_attached: 1'b1
 * Cable: Don't care
 */
#define PDSS_CC_CTRL_0_RD_CC2_DB_DIS                        (1u << 21) /* <21:21> R:RW:0: */


/*
 * 0: Disable CC1 Trimmed Rd Termination
 * 1: Enable CC1 Trimmed Rd Termination
 * Notes:
 * DFP_UA , DFP_A : 0
 * UFP_DEAD: 0
 * UFP_A: 1
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_RD_CC1_EN                            (1u << 22) /* <22:22> R:RW:0: */


/*
 * 0: Disable CC2 Trimmed Rd Termination
 * 1: Enable CC2 Trimmed Rd Termination
 * Notes:
 * DFP_UA , DFP_A : 0
 * UFP_DEAD: 0
 * UFP_A: 1
 * Cable: 0
 * -------------
 * DFP_UA: DFP unattached
 * DFP_A: DFP attached
 * UFP_A: UFP attached
 */
#define PDSS_CC_CTRL_0_RD_CC2_EN                            (1u << 23) /* <23:23> R:RW:0: */


/*
 * 0: Disable CC1 Pull-up Termination (Rp)
 * 1: Enable CC1 Pull-up Termination
 * Notes:
 * DFP_UA  = 0
 * DFP_A : 1  (This bit must be set to 1'b0 if CC1 is not used for communication)
 * UFP_DEAD: 0
 * UFP_A: 0
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_RP_CC1_EN                            (1u << 24) /* <24:24> R:RW:0: */


/*
 * 0: Disable CC2 Pull-up Termination (Rp)
 * 1: Enable CC2 Pull-up Termination
 * Notes:
 * DFP_UA  = 0
 * DFP_A : 1 ( (This bit must be set to 1'b0 if CC2 is not used for communication)
 * UFP_DEAD: 0
 * UFP_A: 0
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_RP_CC2_EN                            (1u << 25) /* <25:25> R:RW:0: */


/*
 * Selects the Pull-up Termination current value
 * 0, 2 - 80uA (Default current broadcast)
 * 1 - 180uA (1.5A current broadcast)
 * 3 - 330uA (3.0A current broadcast)
 * Notes: This field matters only when RP_CC1/2_EN is set.
 */
#define PDSS_CC_CTRL_0_RP_MODE_MASK                         (0x0c000000) /* <26:27> R:RW:0: */
#define PDSS_CC_CTRL_0_RP_MODE_POS                          (26)


/*
 * Enables the hysteresis mode for the line activity comparator
 * This is a legacy control signal and should be driven low
 * Notes: (Common for all application)
 * This regsiter should be programmed to "0".
 */
#define PDSS_CC_CTRL_0_EN_HYST                              (1u << 28) /* <28:28> R:RW:1: */


/*
 * Selects the amount of line activity comparator hysteresis -> Also legacy
 * control, should be driven low
 * 0: 50mV hystersis
 * 1: 100mV hysteresis"
 * Notes: This filed matter onlt when EN_HYST is set.
 */
#define PDSS_CC_CTRL_0_HYST_MODE                            (1u << 29) /* <29:29> R:RW:1: */


/*
 * Controls the reference voltage generator for DFP vs. UFP/Cable operation
 * 0: UFP/Cable - voltage reference is 2.4V
 * 1: DFP - voltage reference is 2.6V
 * This register is a user configuration that must be set.
 * Notes: This bit should only be enablled for DFP
 */
#define PDSS_CC_CTRL_0_DFP_EN                               (1u << 30) /* <30:30> R:RW:0: */


/*
 * Disables all active circuitry and DC paths
 * DS_ATTACH_DET_EN is still active
 * Notes: Firmware can disable the PHY for DFP waiting for attach. This will
 * save power while waiting for attach. Friware sets DS_ATTACH_DET_EN bit
 * to enable resistor based pullup on both CC lines. This pullup can be enabled
 * independent of PHY state.
 * DFP_UA: 1
 * DFP_A: 0
 * UFP: 0
 * Cable: 0
 */
#define PDSS_CC_CTRL_0_PWR_DISABLE                          (1u << 31) /* <31:31> R:RW:1: */


/*
 * S8USBPD C-connector Control Register 1
 */
#define PDSS_CC_CTRL_1_ADDRESS(m)                           (0x400a0190 + ((m) * (0x10000)))
#define PDSS_CC_CTRL_1(m)                                   (*(volatile uint32_t *)(0x400a0190 + ((m) * 0x10000)))
#define PDSS_CC_CTRL_1_DEFAULT                              (0x00005000)

/*
 * Enables ADFT Mode
 */
#define PDSS_CC_CTRL_1_CC_ADFT_EN                           (1u << 0) /* <0:0> R:RW:0: */


/*
 * Selects ADFT connection
 * See s8usbpd BROS for decoding details
 */
#define PDSS_CC_CTRL_1_CC_ADFT_CTRL_MASK                    (0x0000001e) /* <1:4> R:RW:0: */
#define PDSS_CC_CTRL_1_CC_ADFT_CTRL_POS                     (1)


/*
 * Enables the offset generator for the RX comparator
 * 0 - no offset
 * 1 - offset enabled, see RX_OFFSET register for value
 * This is a legacy control signal and should be driven low
 */
#define PDSS_CC_CTRL_1_RX_OFFSET_EN                         (1u << 6) /* <6:6> R:RW:0: */


/*
 * Selects the RX comparator offset:
 */
#define PDSS_CC_CTRL_1_RX_OFFSET_MASK                       (0x00000380) /* <7:9> R:RW:0: */
#define PDSS_CC_CTRL_1_RX_OFFSET_POS                        (7)


/*
 * Enables the deepsleep attach detect pull-up resistor
 * Set HI for a DFP waiting for attach
 */
#define PDSS_CC_CTRL_1_DS_ATTACH_DET_EN                     (1u << 10) /* <10:10> R:RW:0: */


/*
 * Transmit voltage select
 */
#define PDSS_CC_CTRL_1_VTX_SEL_MASK                         (0x00003800) /* <11:13> R:RW:2: */
#define PDSS_CC_CTRL_1_VTX_SEL_POS                          (11)


/*
 * 0: All outputs are isolated to a known value
 * 1: Normal operation
 */
#define PDSS_CC_CTRL_1_CC_ISO_N                             (1u << 14) /* <14:14> R:RW:1: */


/*
 * S8USBPD DeepSleep-Reference Control Register
 */
#define PDSS_DPSLP_REF_CTRL_ADDRESS(m)                      (0x400a0194 + ((m) * (0x10000)))
#define PDSS_DPSLP_REF_CTRL(m)                              (*(volatile uint32_t *)(0x400a0194 + ((m) * 0x10000)))
#define PDSS_DPSLP_REF_CTRL_DEFAULT                         (0x00000030)

/*
 * Setting this bit will enable the deepsleep current reference outputs.
 */
#define PDSS_DPSLP_REF_CTRL_IGEN_EN                         (1u << 0) /* <0:0> R:RW:0: */


/*
 * Setting this bit will enable the deepsleep reference generator ADFT mode.
 */
#define PDSS_DPSLP_REF_CTRL_DPSLP_ADFT_EN                   (1u << 1) /* <1:1> R:RW:0: */


/*
 * Controls the Deep Sleep reference ADFT mode
 * 0: ganged 7 iref current sources
 * 1: vrefdpslp voltage reference
 */
#define PDSS_DPSLP_REF_CTRL_ADFT_CTRL                       (1u << 2) /* <2:2> R:RW:0: */


/*
 * Block enable input
 * 1 - All analog and DC paths cut off, outputs forced to known value
 *      This completely disables the CC Transceiver/Detect block.
 * 0 - Normal functionality
 * Notes: Firmware can disable the PHY for DFP waiting for attach. This will
 * save power while waiting for attach. Friware sets DS_ATTACH_DET_EN bit
 * to enable resistor based pullup on both CC lines. This pullup can be enabled
 * independent of PHY state.
 * DFP_UA: 1
 * DFP_A: 0
 * UFP: 0
 * Cable: 0
 */
#define PDSS_DPSLP_REF_CTRL_PD_DPSLP                        (1u << 3) /* <3:3> R:RW:0: */


/*
 * Control to "zero-out" PTAT/CTAT currents
 * 1. 00 : PTAT-CTAT Zeroed-out (ibias=0)
 * 2. 01 : CTAT zeroed-out; IBIAS = IPTAT
 * 3. 10 : PTAT zeroed-out; IBIAS-ICTAT
 * 4. 11 : Both enabled
 */
#define PDSS_DPSLP_REF_CTRL_PCTAT_CTRL_MASK                 (0x00000030) /* <4:5> R:RW:3: */
#define PDSS_DPSLP_REF_CTRL_PCTAT_CTRL_POS                  (4)


/*
 * S8USBPD PUMP control Register
 */
#define PDSS_PUMP_CTRL_ADDRESS(m)                           (0x400a019c + ((m) * (0x10000)))
#define PDSS_PUMP_CTRL(m)                                   (*(volatile uint32_t *)(0x400a019c + ((m) * 0x10000)))
#define PDSS_PUMP_CTRL_DEFAULT                              (0x00000014)

/*
 * ADFT control
 */
#define PDSS_PUMP_CTRL_ADFT_MASK                            (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_PUMP_CTRL_ADFT_POS                             (0)


/*
 * Bypasses the pumped output.  Pump output is connected to VDDD.
 * 0: Charge pump output
 * 1: Pump output shorted to VDDD
 */
#define PDSS_PUMP_CTRL_BYPASS_LV                            (1u << 2) /* <2:2> R:RW:1: */


/*
 * External clock select
 * 0: Internal oscillator used for charge pump
 * 1: External clock used for charge pump
 */
#define PDSS_PUMP_CTRL_CLK_SEL                              (1u << 3) /* <3:3> R:RW:0: */


/*
 * Pump powerdown signal
 * 0: Pump enabled
 * 1: Pump disabled, all current paths cutoff
 * Notes: This pump should beb enabled only when DFP is VDDD supply is 3.3V
 * and Rp value is 3A.
 * DFP_UA : 0
 * DFP_A:  1 (VDDD = 3.3.V and RP_MODE = 3A)
 * UFP: 0
 * AMA: 0
 * Cable: 0
 */
#define PDSS_PUMP_CTRL_PD_PUMP                              (1u << 4) /* <4:4> R:RW:1: */


/*
 * S8USBPD 300ma V5V Power FET Register
 */
#define PDSS_PFET300_CTRL_ADDRESS(m)                        (0x400a01a0 + ((m) * (0x10000)))
#define PDSS_PFET300_CTRL(m)                                (*(volatile uint32_t *)(0x400a01a0 + ((m) * 0x10000)))
#define PDSS_PFET300_CTRL_DEFAULT                           (0x00000000)

/*
 * V5V Comparator Enable, Active High.
 * Detects if V5V supply is present.
 */
#define PDSS_PFET300_CTRL_EN_COMP                           (1u << 0) /* <0:0> R:RW:0: */


/*
 * CC1 switch enable signal, Active High.
 */
#define PDSS_PFET300_CTRL_EN_SWITCH_CC1                     (1u << 1) /* <1:1> R:RW:0: */


/*
 * CC1 switch enable signal, Active High.
 */
#define PDSS_PFET300_CTRL_EN_SWITCH_CC2                     (1u << 2) /* <2:2> R:RW:0: */


/*
 * Configures SWAP control0
 */
#define PDSS_SWAP_CTRL0_ADDRESS(m)                          (0x400a01a4 + ((m) * (0x10000)))
#define PDSS_SWAP_CTRL0(m)                                  (*(volatile uint32_t *)(0x400a01a4 + ((m) * 0x10000)))
#define PDSS_SWAP_CTRL0_DEFAULT                             (0x00000000)

/*
 * Input source selection for SWAPT
 */
#define PDSS_SWAP_CTRL0_SWAPT_SOURCE_SEL_MASK               (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_SWAP_CTRL0_SWAPT_SOURCE_SEL_POS                (0)


/*
 * Input source selection for SWAPR
 * 0: Source is cmp_out of ADC1
 * 1: Source is cmp_out of ADC2
 */
#define PDSS_SWAP_CTRL0_SWAPR_SOURCE_SEL                    (1u << 2) /* <2:2> R:RW:0: */


/*
 * Polatiry of the SWAPT output
 * 0: No  Inverting the output of the swapt
 * 1: Inverting the output of the swapt
 */
#define PDSS_SWAP_CTRL0_SWAPT_POLARITY                      (1u << 3) /* <3:3> R:RW:0: */


/*
 * Pull down value for SWAPT_OUT_GPIO1
 */
#define PDSS_SWAP_CTRL0_SWAPT_EN_CC1_PULLDN                 (1u << 4) /* <4:4> R:RW:0: */


/*
 * Pull down value for SWAPT_OUT_GPIO2
 */
#define PDSS_SWAP_CTRL0_SWAPT_EN_CC2_PULLDN                 (1u << 5) /* <5:5> R:RW:0: */


/*
 * 0: Connect SWAPT_EN_CC1_PULLDN to SWAPT_OUT_GPIO1
 * 1: Connect SWAPT to SWAPT_OUT_GPIO1
 */
#define PDSS_SWAP_CTRL0_SWAPT_TO_GPIO1_EN                   (1u << 6) /* <6:6> R:RW:0: */


/*
 * 0: Connect SWAPT_EN_CC2_PULLDN to SWAPT_OUT_GPIO2
 * 1: Connect SWAPT to SWAPT_OUT_GPIO2
 */
#define PDSS_SWAP_CTRL0_SWAPT_TO_GPIO2_EN                   (1u << 7) /* <7:7> R:RW:0: */


/*
 * 0: Clocks is turn off for SWAP block
 * 1: Clock is runing in the SWAP block
 */
#define PDSS_SWAP_CTRL0_SWAP_ENABLED                        (1u << 31) /* <31:31> R:RW:0: */


/*
 * Configures SWAP module, IRQ_MIN, IRQ_MAX and ENABLE
 */
#define PDSS_SWAP_CTRL1_ADDRESS(m)                          (0x400a01a8 + ((m) * (0x10000)))
#define PDSS_SWAP_CTRL1(m)                                  (*(volatile uint32_t *)(0x400a01a8 + ((m) * 0x10000)))
#define PDSS_SWAP_CTRL1_DEFAULT                             (0x8025812c)

/*
 * Defines the min width of IRQ pulse. Spec = 0.5 ms. Assumes 600 Khz clock.
 */
#define PDSS_SWAP_CTRL1_IRQ_MIN_MASK                        (0x00000fff) /* <0:11> R:RW:300: */
#define PDSS_SWAP_CTRL1_IRQ_MIN_POS                         (0)


/*
 * Defines the max width of IRQ pulse. Spec = 1.0 ms. Assumes 600 Khz clock.
 */
#define PDSS_SWAP_CTRL1_IRQ_MAX_MASK                        (0x00fff000) /* <12:23> R:RW:600: */
#define PDSS_SWAP_CTRL1_IRQ_MAX_POS                         (12)


/*
 * When set it flushes the queue.
 */
#define PDSS_SWAP_CTRL1_FLUSH_QUEUE                         (1u << 24) /* <24:24> R:RW:0: */


/*
 * When set, input to SWAP module will come from output of SWAPt module.
 */
#define PDSS_SWAP_CTRL1_LOOPBACK_EN                         (1u << 30) /* <30:30> R:RW:0: */


/*
 * When set, SWAP state machine is at Idle
 */
#define PDSS_SWAP_CTRL1_RESET_SWAP_STATE                    (1u << 31) /* <31:31> R:RW:1: */


/*
 * Configures SWAP module, Glitch width.
 */
#define PDSS_SWAP_CTRL2_ADDRESS(m)                          (0x400a01ac + ((m) * (0x10000)))
#define PDSS_SWAP_CTRL2(m)                                  (*(volatile uint32_t *)(0x400a01ac + ((m) * 0x10000)))
#define PDSS_SWAP_CTRL2_DEFAULT                             (0x00096096)

/*
 * Defines the width of glitch. Spec = 0.25 ms. Assumes clk of 600 Khz on
 * the rising edge of SWAP_in input
 */
#define PDSS_SWAP_CTRL2_GLITCH_WIDTH_HIGH_MASK              (0x00000fff) /* <0:11> R:RW:150: */
#define PDSS_SWAP_CTRL2_GLITCH_WIDTH_HIGH_POS               (0)


/*
 * Defines the width of glitch. Spec = 0.25 ms. Assumes clk of 600 Khz on
 * the falling edge of SWAP_in input
 */
#define PDSS_SWAP_CTRL2_GLITCH_WIDTH_LOW_MASK               (0x00fff000) /* <12:23> R:RW:150: */
#define PDSS_SWAP_CTRL2_GLITCH_WIDTH_LOW_POS                (12)


/*
 * Configures SWAP module, stable high and stable low values
 */
#define PDSS_SWAP_CTRL3_ADDRESS(m)                          (0x400a01b0 + ((m) * (0x10000)))
#define PDSS_SWAP_CTRL3(m)                                  (*(volatile uint32_t *)(0x400a01b0 + ((m) * 0x10000)))
#define PDSS_SWAP_CTRL3_DEFAULT                             (0x0004b04b)

/*
 * Defines the minimum time before plugged condition is detected for subsequent
 * connect.  Spec = 2 ms. Assumes 600 Khz clock. This value is internally
 * multiplied by 16.
 */
#define PDSS_SWAP_CTRL3_STABLE_HIGH_MASK                    (0x00000fff) /* <0:11> R:RW:75: */
#define PDSS_SWAP_CTRL3_STABLE_HIGH_POS                     (0)


/*
 * Defines the minimum time before unplugged condition is detected for subsequent
 * connect.  Spec = 2 ms. Assumes 600 Khz clock. This value is internally
 * multiplied by 16.
 */
#define PDSS_SWAP_CTRL3_STABLE_LOW_MASK                     (0x00fff000) /* <12:23> R:RW:75: */
#define PDSS_SWAP_CTRL3_STABLE_LOW_POS                      (12)


/*
 * Configures SWAP module, irq spacing and unplug event
 */
#define PDSS_SWAP_CTRL4_ADDRESS(m)                          (0x400a01b4 + ((m) * (0x10000)))
#define PDSS_SWAP_CTRL4(m)                                  (*(volatile uint32_t *)(0x400a01b4 + ((m) * 0x10000)))
#define PDSS_SWAP_CTRL4_DEFAULT                             (0x000004b0)

/*
 * Defines the minimum spacing between interrupts  Spec = 2 ms. Assumes 600
 * Khz clock.
 */
#define PDSS_SWAP_CTRL4_IRQ_SPACING_MASK                    (0x00000fff) /* <0:11> R:RW:1200: */
#define PDSS_SWAP_CTRL4_IRQ_SPACING_POS                     (0)


/*
 * Configures SWAP module, Initial LOW and HIGH times
 */
#define PDSS_SWAP_CTRL5_ADDRESS(m)                          (0x400a01b8 + ((m) * (0x10000)))
#define PDSS_SWAP_CTRL5(m)                                  (*(volatile uint32_t *)(0x400a01b8 + ((m) * 0x10000)))
#define PDSS_SWAP_CTRL5_DEFAULT                             (0x0ea50ea5)

/*
 * Defines the minimum time before plugged condion is detected for initial
 * connect.  Spec = 100 ms. Assumes 600 Khz clock. This value is internally
 * multiplied by 16.
 */
#define PDSS_SWAP_CTRL5_LONG_HIGH_MASK                      (0x0000ffff) /* <0:15> R:RW:3749: */
#define PDSS_SWAP_CTRL5_LONG_HIGH_POS                       (0)


/*
 * Defines the minimum width before unplugged condition is detected.  Spec
 * = 100 ms. Assumes 600 Khz clock. This value is internally multiplied by
 * 16..  After LONG_LOW condition is detected, SWAP input is expected to
 * stay high equal to LONG_HIGH before plugged condition is reached. This
 * is the optional 100 ms debounce time
 */
#define PDSS_SWAP_CTRL5_LONG_LOW_MASK                       (0xffff0000) /* <16:31> R:RW:3749: */
#define PDSS_SWAP_CTRL5_LONG_LOW_POS                        (16)


/*
 * SWAP queue register
 */
#define PDSS_SWAP_QUEUE_ADDRESS(m)                          (0x400a01bc + ((m) * (0x10000)))
#define PDSS_SWAP_QUEUE(m)                                  (*(volatile uint32_t *)(0x400a01bc + ((m) * 0x10000)))
#define PDSS_SWAP_QUEUE_DEFAULT                             (0x00000000)

/*
 * This is a read only register. Reading this register will clear the queue
 * in HW. This queue holds 4 locations.
 * Location 0 : bits 1:0.
 * Location 1 : bits 3:2.
 * Location 2 : bits 5:4.
 * Location 3 : bits 7:6.
 * This queue provides a FIFO function. First location written by HW is location
 * 0. The last location is location 3. HW will not write once this queue
 * is full. Reading the Queue will clear the contents. Reading an empty queue
 * will not cause FIFO under run.
 * This queue will hold SWAP input events. These events are PLUGGED, UNPLUGGED
 * and IRQ.  Only 2 back to back IRQ will be written into the queue and additional
 * ones will be dropped. When UNPLUGGED is detected, the event will always
 * be written to location 0 of the queue and locations 1 to 3 will be cleared.
 *  The following codes are used to encode events in the queue.
 * 00  : Empty location.
 * 01  : UNPLUGGED.
 * 10  : PLUGGED.
 * 11  : IRQ.
 */
#define PDSS_SWAP_QUEUE_DATA_OUT_MASK                       (0x000000ff) /* <0:7> RW:R:0: */
#define PDSS_SWAP_QUEUE_DATA_OUT_POS                        (0)


/*
 * SWAPT module configure and IRQ width and delay parameters
 */
#define PDSS_SWAPT_CTRL1_ADDRESS(m)                         (0x400a01c0 + ((m) * (0x10000)))
#define PDSS_SWAPT_CTRL1(m)                                 (*(volatile uint32_t *)(0x400a01c0 + ((m) * 0x10000)))
#define PDSS_SWAPT_CTRL1_DEFAULT                            (0x404af1c2)

/*
 * Defines the width of IRQ pulse in muliples of 600KHZ clock period. This
 * value sets the width at 0.75 ms.
 */
#define PDSS_SWAPT_CTRL1_IRQ_WIDTH_MASK                     (0x00000fff) /* <0:11> R:RW:450: */
#define PDSS_SWAPT_CTRL1_IRQ_WIDTH_POS                      (0)


/*
 * Defines the delay to execute IRQ command in multiples of 600KHZ clock
 * period. The default value is 2 ms.
 */
#define PDSS_SWAPT_CTRL1_SET_IRQ_DELAY_MASK                 (0x00fff000) /* <12:23> R:RW:1199: */
#define PDSS_SWAPT_CTRL1_SET_IRQ_DELAY_POS                  (12)


/*
 * Defines the initial level of the SWAPt output.
 */
#define PDSS_SWAPT_CTRL1_DEFAULT_LEVEL                      (1u << 24) /* <24:24> R:RW:0: */


/*
 * Source of data to SWAPt_out signal. 0: comes from SWAPt module.  1: comes
 * from DATA_SOURCE bit of this register
 */
#define PDSS_SWAPT_CTRL1_DATA_SOURCE_SELECT                 (1u << 25) /* <25:25> R:RW:0: */


/*
 * Data source of SWAPt_out output when DATA_SOURCE_SELECT is set high
 */
#define PDSS_SWAPT_CTRL1_DATA_SOURCE                        (1u << 26) /* <26:26> R:RW:0: */


/*
 * Defines the commands to be executed by SWAPt module.
 * 0: set SWAPt_out to low.
 * 1: set SWAPt_out to high.
 * 2: send an IRQ.
 * 3: unused.
 */
#define PDSS_SWAPT_CTRL1_COMMAND_MASK                       (0x18000000) /* <27:28> R:RW:0: */
#define PDSS_SWAPT_CTRL1_COMMAND_POS                        (27)


/*
 * When set, the commands specified by COMMAND bits will be executed. Execution
 * happens on the rising edge of this bit.
 * FW should not clear this bit until INTR2.SWAPT_COMMAND_DONE interrupt
 * is set.
 * Hardware clears this bit when command_done is set.
 */
#define PDSS_SWAPT_CTRL1_COMMAND_START                      (1u << 29) /* <29:29> RW1C:RW:0: */


/*
 * When set, SWAPT state machine is at Idle
 */
#define PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE                  (1u << 30) /* <30:30> R:RW:1: */


/*
 * SWAPT HIGH & LOW transition delays.
 */
#define PDSS_SWAPT_CTRL2_ADDRESS(m)                         (0x400a01c4 + ((m) * (0x10000)))
#define PDSS_SWAPT_CTRL2(m)                                 (*(volatile uint32_t *)(0x400a01c4 + ((m) * 0x10000)))
#define PDSS_SWAPT_CTRL2_DEFAULT                            (0x014af4af)

/*
 * Defines the delay from HIGH -> LOW transition.  In multiples of 600KHZ
 * clock period. The defaul value is 2 ms.
 */
#define PDSS_SWAPT_CTRL2_SET_LOW_DELAY_MASK                 (0x00000fff) /* <0:11> R:RW:1199: */
#define PDSS_SWAPT_CTRL2_SET_LOW_DELAY_POS                  (0)


/*
 * Defines the delay from LOW --> HIGH transition.  In multiples of 600KHZ
 * clock period. The defaul value is 2 ms.
 */
#define PDSS_SWAPT_CTRL2_SET_HIGH_DELAY_MASK                (0x00fff000) /* <12:23> R:RW:1199: */
#define PDSS_SWAPT_CTRL2_SET_HIGH_DELAY_POS                 (12)


/*
 * Defines the default level of SWAPT transmit OE.
 * 0: SWAP output goes IDLE (OE=0) when module is not selected or module
 * is IDLE state
 * 1: SWAP output stays active.
 */
#define PDSS_SWAPT_CTRL2_DEFAULT_OE                         (1u << 24) /* <24:24> R:RW:1: */


/*
 * Defines the source of SWAP pin OE.
 * 0: Driven by HW
 * 1: Driven by bit CTRL.SWAP_DIRECTION
 */
#define PDSS_SWAPT_CTRL2_OE_SOURCE_SELECT                   (1u << 25) /* <25:25> R:RW:0: */


/*
 * Selects which pins to appear on SWAPT debug bus
 *      2'h1     : debug_out   <= count_12_bits_q[3:0];
 *     2'h2     : debug_out   <= count_12_bits_q[7:4];
 *     2'h3     : debug_out   <= count_12_bits_q[11:8];
 *     default  : debug_out   <= {1'h0, state_q};
 */
#define PDSS_SWAPT_CTRL2_SWAPT_DEBUG_SEL_MASK               (0x0c000000) /* <26:27> R:RW:0: */
#define PDSS_SWAPT_CTRL2_SWAPT_DEBUG_SEL_POS                (26)


/*
 * The output of debug bus.
 */
#define PDSS_SWAPT_CTRL2_SWAPT_DEBUG_OUT_MASK               (0xf0000000) /* <28:31> RW:R:0: */
#define PDSS_SWAPT_CTRL2_SWAPT_DEBUG_OUT_POS                (28)


/*
 * External NGDO control registers
 */
#define PDSS_NGDO_CTRL_0_ADDRESS(m)                         (0x400a01c8 + ((m) * (0x10000)))
#define PDSS_NGDO_CTRL_0(m)                                 (*(volatile uint32_t *)(0x400a01c8 + ((m) * 0x10000)))
#define PDSS_NGDO_CTRL_0_DEFAULT                            (0x00000000)

/*
 * NFET Gate Driver Control Signal, Active High
 * Bit 3 - VBUS_C_CTRL0 pin Control
 * Bit 4 - VBUS_C_CTRL1 pin Control
 */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_C_NGDO_EN_LV_MASK        (0x00000018) /* <3:4> R:RW:0: */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_C_NGDO_EN_LV_POS         (3)


/*
 * PFET Gate Driver Control Signal, Active High
 * Bit 5 - VBUS_C_CTRL0 pin Control
 * Bit 6 - VBUS_C_CTRL1 pin Control
 */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_C_PULLDN_EN_LV_MASK      (0x00000060) /* <5:6> R:RW:0: */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_C_PULLDN_EN_LV_POS       (5)


/*
 * NFET Gate Driver Control Signal, Active High
 * Bit 12 - VBUS_P_CTRL0 pin Control
 * Bit 13 - VBUS_P_CTRL1 pin Control
 * NOTE: In CCG3
 * Bit 12 - VBUS_P_CTRL1 pin Control
 * Bit 13 - VBUS_P_CTRL0 pin Control
 */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_P_NGDO_EN_LV_MASK        (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_P_NGDO_EN_LV_POS         (12)


/*
 * PFET Gate Driver Control Signal, Active High
 * Bit 14 - VBUS_P_CTRL0 pin Control
 * Bit 15 - VBUS_P_CTRL1 pin Control
 * NOTE: In CCG3
 * Bit 14 - VBUS_P_CTRL1 pin Control
 * Bit 15 - VBUS_P_CTRL0 pin Control
 */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_P_PULLDN_EN_LV_MASK      (0x0000c000) /* <14:15> R:RW:0: */
#define PDSS_NGDO_CTRL_0_NGDO_VBUS_P_PULLDN_EN_LV_POS       (14)


/*
 * RESERVED FOR FUTURE USE
 */
#define PDSS_NGDO_CTRL_0_NGDO_SPARE_MASK                    (0xff000000) /* <24:31> R:RW:0: */
#define PDSS_NGDO_CTRL_0_NGDO_SPARE_POS                     (24)


/*
 * Gate driver control for VBUS C
 */
#define PDSS_NGDO_CTRL_C_ADDRESS(m)                         (0x400a01cc + ((m) * (0x10000)))
#define PDSS_NGDO_CTRL_C(m)                                 (*(volatile uint32_t *)(0x400a01cc + ((m) * 0x10000)))
#define PDSS_NGDO_CTRL_C_DEFAULT                            (0x00000000)

/*
 * The gate driver control option.
 * 0: FW controlls the EN_LV and PULLDN_EN_PLV pins.
 * 1: HW controlls the EN_LV and PULLDN_EN_PLV pins
 */
#define PDSS_NGDO_CTRL_C_AUTO_MODE                          (1u << 0) /* <0:0> R:RW:0: */


/*
 * The gate driver control option.
 * Any write-one to this register will reset the edge detector in the NGDO
 * controller.
 * FW should cleared this register after the fault conditions(OV, UV, OC,
 * VBUS_LESS_5) are removed by writing a “1” to this register.
 */
#define PDSS_NGDO_CTRL_C_RST_EDGE_DET                       (1u << 1) /* <1:1> R:RW:0: */


/*
 * This bit selects which bit of EN_LV/PULLDN_EN_LV turns of first when the
 * TURN_OFF_SPACING is >0.
 * 0: Bit0 turns off first
 * 1: Bit1 turns off first
 */
#define PDSS_NGDO_CTRL_C_TURN_OFF_SEL                       (1u << 2) /* <2:2> R:RW:0: */


/*
 * Bypassing the synchronizer for fault detection in CLK_LF domain.
 * 0: No Bypass. The spacing take effect 2 CLK_LF after the fault detection.
 * 1: Bpass: The spacing takes effect right after fault detection
 */
#define PDSS_NGDO_CTRL_C_BYPASS_2DFF                        (1u << 3) /* <3:3> R:RW:0: */


/*
 * The EN_LV/PULLDN_EN_LV bit0 and bit 1 CLK_LF spacing.
 * 0:No Spacing. Both bit0 and bit1 turns off at the same time.
 * 1: One CLK_LF spacing between bit0 and bit1
 * 2: Two CLK_LF spacing between bit0 and bit1
 * .
 * .
 * .
 * 15: 15 CLK_LF spacing between bit0 and bit1
 */
#define PDSS_NGDO_CTRL_C_TURN_OFF_SPACING_MASK              (0x000000f0) /* <4:7> R:RW:0: */
#define PDSS_NGDO_CTRL_C_TURN_OFF_SPACING_POS               (4)


/*
 * Reserved
 */
#define PDSS_NGDO_CTRL_C_SEL_OV                             (1u << 8) /* <8:8> R:RW:0: */


/*
 * Reserved
 */
#define PDSS_NGDO_CTRL_C_SEL_UV                             (1u << 9) /* <9:9> R:RW:0: */


/*
 * Reserved
 */
#define PDSS_NGDO_CTRL_C_SEL_OC                             (1u << 10) /* <10:10> R:RW:0: */


/*
 * 0: SWAP detection & VBUS_LESS_5 is not selected for turning off the NGDO
 * 1: SWAP detection & VBUS_LESS_5 is       selected for turning off the
 * NGDO
 */
#define PDSS_NGDO_CTRL_C_SEL_SWAP_VBUS_LESS_5               (1u << 11) /* <11:11> R:RW:0: */


/*
 * The Off value used by Hardware in Automode to turn off the NGDO
 */
#define PDSS_NGDO_CTRL_C_EN_LV_OFF_VALUE_MASK               (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_NGDO_CTRL_C_EN_LV_OFF_VALUE_POS                (12)


/*
 * The Off value used by Hardware in Automode to turn off the NGDO
 */
#define PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_MASK        (0x0000c000) /* <14:15> R:RW:0: */
#define PDSS_NGDO_CTRL_C_PULLDN_EN_LV_OFF_VALUE_POS         (14)


/*
 * The ON value used by Hardware to turn on the NGDO in swap condition
 */
#define PDSS_NGDO_CTRL_C_EN_LV_ON_VALUE_MASK                (0x00030000) /* <16:17> R:RW:0: */
#define PDSS_NGDO_CTRL_C_EN_LV_ON_VALUE_POS                 (16)


/*
 * The ON value used by Hardware to turn on the NGDO in swap condition
 */
#define PDSS_NGDO_CTRL_C_PULLDN_EN_LV_ON_VALUE_MASK         (0x000c0000) /* <18:19> R:RW:0: */
#define PDSS_NGDO_CTRL_C_PULLDN_EN_LV_ON_VALUE_POS          (18)


/*
 * Gate driver control for VBUS P
 */
#define PDSS_NGDO_CTRL_P_ADDRESS(m)                         (0x400a01d0 + ((m) * (0x10000)))
#define PDSS_NGDO_CTRL_P(m)                                 (*(volatile uint32_t *)(0x400a01d0 + ((m) * 0x10000)))
#define PDSS_NGDO_CTRL_P_DEFAULT                            (0x00000000)

/*
 * The gate driver control option.
 * 0: FW controlls the EN_LV and PULLDN_EN_PLV pins.
 * 1: HW controlls the EN_LV and PULLDN_EN_PLV pins
 */
#define PDSS_NGDO_CTRL_P_AUTO_MODE                          (1u << 0) /* <0:0> R:RW:0: */


/*
 * The gate driver control option.
 * Any write-one to this register will reset the edge detector in the NGDO
 * controller.
 * FW should cleared this register after the fault conditions(OV, UV, OC,
 * VBUS_LESS_5) are removed by writing a “1” to this register.
 */
#define PDSS_NGDO_CTRL_P_RST_EDGE_DET                       (1u << 1) /* <1:1> R:RW:0: */


/*
 * This bit selects which bit of EN_LV/PULLDN_EN_LV turns of first when the
 * TURN_OFF_SPACING is >0.
 * 0: Bit0 turns off first
 * 1: Bit1 turns off first
 * Note: IN CCG3:
 * 0: Bit1 turns off first
 * 1: Bit0 turns off first
 */
#define PDSS_NGDO_CTRL_P_TURN_OFF_SEL                       (1u << 2) /* <2:2> R:RW:0: */


/*
 * Bypassing the synchronizer for fault detection in CLK_LF domain.
 * 0: No Bypass. The spacing take effect 2 CLK_LF after the fault detection.
 * 1: Bpass: The spacing takes effect right after fault detection
 */
#define PDSS_NGDO_CTRL_P_BYPASS_2DFF                        (1u << 3) /* <3:3> R:RW:0: */


/*
 * The EN_LV/PULLDN_EN_LV bit0 and bit 1 CLK_LF spacing.
 * 0:No Spacing. Both bit0 and bit1 turns off at the same time.
 * 1: One CLK_LF spacing between bit0 and bit1
 * 2: Two CLK_LF spacing between bit0 and bit1
 * .
 * .
 * .
 * 15: 15 CLK_LF spacing between bit0 and bit1
 */
#define PDSS_NGDO_CTRL_P_TURN_OFF_SPACING_MASK              (0x000000f0) /* <4:7> R:RW:0: */
#define PDSS_NGDO_CTRL_P_TURN_OFF_SPACING_POS               (4)


/*
 * Reserved
 */
#define PDSS_NGDO_CTRL_P_SEL_OV                             (1u << 8) /* <8:8> R:RW:0: */


/*
 * Reserved
 */
#define PDSS_NGDO_CTRL_P_SEL_UV                             (1u << 9) /* <9:9> R:RW:0: */


/*
 * Reserved
 */
#define PDSS_NGDO_CTRL_P_SEL_OC                             (1u << 10) /* <10:10> R:RW:0: */


/*
 * 0: SWAP detection & VBUS_LESS_5 is not selected for turning off the NGDO
 * 1: SWAP detection & VBUS_LESS_5 is       selected for turning off the
 * NGDO
 */
#define PDSS_NGDO_CTRL_P_SEL_SWAP_VBUS_LESS_5               (1u << 11) /* <11:11> R:RW:0: */


/*
 * The Off value used by Hardware in Automode to turn off the NGDO bit[1:0]
 * Note in CCG3:
 * EN_LV Off values for [0:1]
 */
#define PDSS_NGDO_CTRL_P_EN_LV_OFF_VALUE_MASK               (0x00003000) /* <12:13> R:RW:0: */
#define PDSS_NGDO_CTRL_P_EN_LV_OFF_VALUE_POS                (12)


/*
 * The Off value used by Hardware in Automode to turn off the NGDO bit[1:0]
 * Note in CCG3:
 * PULLDN_EN_LV Off values for [0:1]
 */
#define PDSS_NGDO_CTRL_P_PULLDN_EN_LV_OFF_VALUE_MASK        (0x0000c000) /* <14:15> R:RW:0: */
#define PDSS_NGDO_CTRL_P_PULLDN_EN_LV_OFF_VALUE_POS         (14)


/*
 * The ON value used by Hardware to turn on the NGDO in swap condition bit[1:0]
 * Note in CCG3:
 * EN_LV ON values for [0:1]
 */
#define PDSS_NGDO_CTRL_P_EN_LV_ON_VALUE_MASK                (0x00030000) /* <16:17> R:RW:0: */
#define PDSS_NGDO_CTRL_P_EN_LV_ON_VALUE_POS                 (16)


/*
 * The ON value used by Hardware to turn on the NGDO in swap condition bit[1:0]
 * Note in CCG3:
 * PULLDN_EN_LV ON values for [0:1]
 */
#define PDSS_NGDO_CTRL_P_PULLDN_EN_LV_ON_VALUE_MASK         (0x000c0000) /* <18:19> R:RW:0: */
#define PDSS_NGDO_CTRL_P_PULLDN_EN_LV_ON_VALUE_POS          (18)


/*
 * NCELL DDFT Selections
 */
#define PDSS_NCELL_DDFT_MUX_ADDRESS(m)                      (0x400a01d4 + ((m) * (0x10000)))
#define PDSS_NCELL_DDFT_MUX(m)                              (*(volatile uint32_t *)(0x400a01d4 + ((m) * 0x10000)))
#define PDSS_NCELL_DDFT_MUX_DEFAULT                         (0x00000000)

/*
 * 30      vbus_p_en_lv[1]
 * 29      vbus_p_en_lv[0]
 * 28      vbus_p_pulldn_en_lv[1]
 * 27      vbus_p_pulldn_en_lv[0]
 * 26      vbus_c_en_lv[1]
 * 25      vbus_c_en_lv[0]
 * 24      vbus_c_pulldn_en_lv[1]
 * 23      vbus_c_pulldn_en_lv[0]
 * 22      0
 * 21      0
 * 20      0
 * 19      0
 * 18      ngdo_vbus_p_ddft
 * 17      ngdo_vbus_c_ddft
 * 16      vbus_p_ddft_faults_masked
 * 15      vbus_p_ddft_async_fault_det
 * 14      0
 * 13      0
 * 12      0
 * 11      0
 * 10      0
 * 9       0
 * 8       0
 * 7       0
 * 6       vbus_c_ddft_faults_masked
 * 5       vbus_c_ddft_async_fault_det
 * 4       0
 * 3       0
 * 2       0
 * 1        0
 * 0       0
 */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT0_SEL_MASK            (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT0_SEL_POS             (0)


/*
 * 30      vbus_p_en_lv[1]
 * 29      vbus_p_en_lv[0]
 * 28      vbus_p_pulldn_en_lv[1]
 * 27      vbus_p_pulldn_en_lv[0]
 * 26      vbus_c_en_lv[1]
 * 25      vbus_c_en_lv[0]
 * 24      vbus_c_pulldn_en_lv[1]
 * 23      vbus_c_pulldn_en_lv[0]
 * 22      0
 * 21      0
 * 20      0
 * 19      0
 * 18      ngdo_vbus_p_ddft
 * 17      ngdo_vbus_c_ddft
 * 16      vbus_p_ddft_faults_masked
 * 15      vbus_p_ddft_async_fault_det
 * 14      0
 * 13      0
 * 12      0
 * 11      0
 * 10      0
 * 9       0
 * 8       0
 * 7       0
 * 6       vbus_c_ddft_faults_masked
 * 5       vbus_c_ddft_async_fault_det
 * 4       0
 * 3       0
 * 2       0
 * 1        0
 * 0       0
 */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT1_SEL_MASK            (0x00000fc0) /* <6:11> R:RW:0: */
#define PDSS_NCELL_DDFT_MUX_NCELL_DDFT1_SEL_POS             (6)


/*
 * Swap Spare coinfiguration 0
 */
#define PDSS_SWAP_CNTRL_0_ADDRESS(m)                        (0x400a01d8 + ((m) * (0x10000)))
#define PDSS_SWAP_CNTRL_0(m)                                (*(volatile uint32_t *)(0x400a01d8 + ((m) * 0x10000)))
#define PDSS_SWAP_CNTRL_0_DEFAULT                           (0x00000000)

/*
 * Controls the CC Charge Pump voltage references
 * 0 - Charge pump voltage is lower
 * 1 - Charge pump voltage is higher
 */
#define PDSS_SWAP_CNTRL_0_CP_REF_SEL                        (1u << 0) /* <0:0> R:RW:0: */


/*
 * Connects cmp_dn comparator to CC1/CC2
 * 0 - CC1
 * 1 - CC2
 */
#define PDSS_SWAP_CNTRL_0_CMP_FS_CC1V2                      (1u << 1) /* <1:1> R:RW:0: */


/*
 * Selects the voltage threshold for cmp_fs comparator
 * Notes: 0.52V reference should be used for Fast Swap Detection
 */
#define PDSS_SWAP_CNTRL_0_CMP_FS_VSEL_MASK                  (0x0000001c) /* <2:4> R:RW:0: */
#define PDSS_SWAP_CNTRL_0_CMP_FS_VSEL_POS                   (2)


/*
 * TBD
 */
#define PDSS_SWAP_CNTRL_0_SWAP_1_MASK                       (0x000000e0) /* <5:7> R:RW:0: */
#define PDSS_SWAP_CNTRL_0_SWAP_1_POS                        (5)


/*
 * TBD
 */
#define PDSS_SWAP_CNTRL_0_SWAP_2_MASK                       (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_SWAP_CNTRL_0_SWAP_2_POS                        (8)


/*
 * TBD
 */
#define PDSS_SWAP_CNTRL_0_SWAP_3_MASK                       (0x00ff0000) /* <16:23> R:RW:0: */
#define PDSS_SWAP_CNTRL_0_SWAP_3_POS                        (16)


/*
 * Swap Spare coinfiguration 1
 */
#define PDSS_SWAP_CNTRL_1_ADDRESS(m)                        (0x400a01dc + ((m) * (0x10000)))
#define PDSS_SWAP_CNTRL_1(m)                                (*(volatile uint32_t *)(0x400a01dc + ((m) * 0x10000)))
#define PDSS_SWAP_CNTRL_1_DEFAULT                           (0x00000000)

/*
 * TBD
 */
#define PDSS_SWAP_CNTRL_1_SWAP_5_MASK                       (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_SWAP_CNTRL_1_SWAP_5_POS                        (0)


/*
 * TBD
 */
#define PDSS_SWAP_CNTRL_1_SWAP_6_MASK                       (0x0000ff00) /* <8:15> R:RW:0: */
#define PDSS_SWAP_CNTRL_1_SWAP_6_POS                        (8)


/*
 * Counter for IDLE detection
 */
#define PDSS_BIT_EN_CNTR_CTRL_ADDRESS(m)                    (0x400a01e0 + ((m) * (0x10000)))
#define PDSS_BIT_EN_CNTR_CTRL(m)                            (*(volatile uint32_t *)(0x400a01e0 + ((m) * 0x10000)))
#define PDSS_BIT_EN_CNTR_CTRL_DEFAULT                       (0x00000000)

/*
 * The number of rx clock that Receiver does not see any rx_bit_en to detect
 * a idle condition
 * 0: Disabled
 */
#define PDSS_BIT_EN_CNTR_CTRL_BIT_EN_CNTR_MASK              (0x000003ff) /* <0:9> R:RW:0: */
#define PDSS_BIT_EN_CNTR_CTRL_BIT_EN_CNTR_POS               (0)


/*
 * An internal rx_bit_en is created when this counter is equal to BIT_EN_CNTR.
 */
#define PDSS_BIT_EN_CNTR_CTRL_GEN_BIT_EN_CNTR_MASK          (0x001f0000) /* <16:20> R:RW:0: */
#define PDSS_BIT_EN_CNTR_CTRL_GEN_BIT_EN_CNTR_POS           (16)


/*
 * S8USBPD C-connector Trim Register0. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_0_ADDRESS(m)                      (0x400aff00 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_0(m)                              (*(volatile uint32_t *)(0x400aff00 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_0_DEFAULT                         (0x00000040)

/*
 * Trim bits for Driver termination impedance
 */
#define PDSS_S8USBPD_TRIM_0_ZDRV_TRIM_MASK                  (0x00000003) /* <0:1> R:RW:0: */
#define PDSS_S8USBPD_TRIM_0_ZDRV_TRIM_POS                   (0)


/*
 * Trim bits for TX Driver rise/fall slew rate
 */
#define PDSS_S8USBPD_TRIM_0_TX_TRIM_MASK                    (0x0000000c) /* <2:3> R:RW:0: */
#define PDSS_S8USBPD_TRIM_0_TX_TRIM_POS                     (2)


/*
 * Trim bits for Rd pull-down resistor
 */
#define PDSS_S8USBPD_TRIM_0_RD_TRIM_MASK                    (0x000000f0) /* <4:7> R:RW:4: */
#define PDSS_S8USBPD_TRIM_0_RD_TRIM_POS                     (4)


/*
 * S8USBPD C-connector Trim Register1. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_1_ADDRESS(m)                      (0x400aff04 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_1(m)                              (*(volatile uint32_t *)(0x400aff04 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_1_DEFAULT                         (0x00000000)

/*
 * Trim bits for CC1 Pull-up current source
 * Firmware may need to read SFlash from location TBD and set this value
 * for each Rp value (RP_MODE)
 */
#define PDSS_S8USBPD_TRIM_1_RP_CC1_TRIM_MASK                (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_S8USBPD_TRIM_1_RP_CC1_TRIM_POS                 (0)


/*
 * DeepSleep 2.4uA current reference temperature coefficient trim bits <1:0>
 */
#define PDSS_S8USBPD_TRIM_1_TC_I_TRIM_9P_MASK               (0x000000c0) /* <6:7> R:RW:0:DPSLP_REF_EN */
#define PDSS_S8USBPD_TRIM_1_TC_I_TRIM_9P_POS                (6)


/*
 * S8USBPD C-connector Trim Register2. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_2_ADDRESS(m)                      (0x400aff08 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_2(m)                              (*(volatile uint32_t *)(0x400aff08 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_2_DEFAULT                         (0x00000000)

/*
 * [5:0]: Trim bits for CC2 Pull-up current source
 *           Firmware may need to read SFlash from location TBD and set this
 * value for each Rp value (RP_MODE)
 * [7:6]:  Deepsleep 2.4uA current reference temperature coefficient trim
 * bits <3:2>
 */
#define PDSS_S8USBPD_TRIM_2_RP_CC2_TRIM_MASK                (0x0000003f) /* <0:5> R:RW:0: */
#define PDSS_S8USBPD_TRIM_2_RP_CC2_TRIM_POS                 (0)


/*
 * Deepsleep 2.4uA current reference temperature coefficient trim bits <3:2>
 */
#define PDSS_S8USBPD_TRIM_2_TC_I_TRIM_4P_MASK               (0x000000c0) /* <6:7> R:RW:0:DPSLP_REF_EN */
#define PDSS_S8USBPD_TRIM_2_TC_I_TRIM_4P_POS                (6)


/*
 * S8USBPD C-connector Trim Register3. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_3_ADDRESS(m)                      (0x400aff0c + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_3(m)                              (*(volatile uint32_t *)(0x400aff0c + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_3_DEFAULT                         (0x00000000)

/*
 * Trim bits for 1.43V comparator reference
 * This should be programmed to 0x2 for CCG2*A Silicon. Leave default for
 * ** silicon.
 * Notes: This filed should be 0x2 for all application (DFP,UFP,AMA ,cable)
 */
#define PDSS_S8USBPD_TRIM_3_V0P55_TRIM_MASK                 (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_S8USBPD_TRIM_3_V0P55_TRIM_POS                  (0)


/*
 * Trim bits for 0.655V comparator reference
 */
#define PDSS_S8USBPD_TRIM_3_V0P655_TRIM_MASK                (0x00000038) /* <3:5> R:RW:0: */
#define PDSS_S8USBPD_TRIM_3_V0P655_TRIM_POS                 (3)


/*
 * S8USBPD C-connector Trim Register4. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_4_ADDRESS(m)                      (0x400aff10 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_4(m)                              (*(volatile uint32_t *)(0x400aff10 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_4_DEFAULT                         (0x00000000)

/*
 * Trim bits for 0.74V comparator reference
 */
#define PDSS_S8USBPD_TRIM_4_V0P74_TRIM_MASK                 (0x0000000f) /* <0:3> R:RW:0: */
#define PDSS_S8USBPD_TRIM_4_V0P74_TRIM_POS                  (0)


/*
 * Trim bits for 0.8V comparator reference
 */
#define PDSS_S8USBPD_TRIM_4_V0P8_TRIM_MASK                  (0x00000070) /* <4:6> R:RW:0: */
#define PDSS_S8USBPD_TRIM_4_V0P8_TRIM_POS                   (4)


/*
 * S8USBPD C-connector Trim Register5. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_5_ADDRESS(m)                      (0x400aff14 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_5(m)                              (*(volatile uint32_t *)(0x400aff14 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_5_DEFAULT                         (0x00000004)

/*
 * Trim bits for 1.125V comparator reference
 */
#define PDSS_S8USBPD_TRIM_5_V1P125_TRIM_MASK                (0x00000007) /* <0:2> R:RW:4: */
#define PDSS_S8USBPD_TRIM_5_V1P125_TRIM_POS                 (0)


/*
 * Trim bits for 1.235V comparator reference
 */
#define PDSS_S8USBPD_TRIM_5_V1P235_TRIM_MASK                (0x00000038) /* <3:5> R:RW:0: */
#define PDSS_S8USBPD_TRIM_5_V1P235_TRIM_POS                 (3)


/*
 * S8USBPD C-connector Trim and Deep Sleep Refgen. Register6. Production
 * trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_6_ADDRESS(m)                      (0x400aff18 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_6(m)                              (*(volatile uint32_t *)(0x400aff18 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_6_DEFAULT                         (0x00000000)

/*
 * Trim bits for 1.77V comparator reference
 * This should be programmed to 0x3 for CCG2*A Silicon. Leave default for
 * ** silicon.
 * Notes: This field should be 0x3 for all application (DFP,UFP,AMA ,cable)
 */
#define PDSS_S8USBPD_TRIM_6_V1P575_TRIM_MASK                (0x00000007) /* <0:2> R:RW:0: */
#define PDSS_S8USBPD_TRIM_6_V1P575_TRIM_POS                 (0)


/*
 * Beta multiplier reference trim bits.
 * Refer to GPM-541 for trim settings.
 */
#define PDSS_S8USBPD_TRIM_6_REF_TRIM_MASK                   (0x00000078) /* <3:6> R:RW:0:DPSLP_REF_EN */
#define PDSS_S8USBPD_TRIM_6_REF_TRIM_POS                    (3)


/*
 * S8USBPD Deep Sleep Refgen Trim Register7. Production trims stored in flash
 */
#define PDSS_S8USBPD_TRIM_7_ADDRESS(m)                      (0x400aff1c + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_7(m)                              (*(volatile uint32_t *)(0x400aff1c + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_7_DEFAULT                         (0x000000f1)

/*
 * DeepSleep 2.4uA current reference trim bit.
 * Refer to s8usbpd BROS for bit settings.
 */
#define PDSS_S8USBPD_TRIM_7_I_TRIM_MASK                     (0x000000ff) /* <0:7> R:RW:241: */
#define PDSS_S8USBPD_TRIM_7_I_TRIM_POS                      (0)


/*
 * S8USBPD Spare1 Trim Register
 */
#define PDSS_S8USBPD_TRIM_SPARE1_ADDRESS(m)                 (0x400aff20 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_SPARE1(m)                         (*(volatile uint32_t *)(0x400aff20 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_SPARE1_DEFAULT                    (0x00000000)

/*
 * Spare1 Trim
 */
#define PDSS_S8USBPD_TRIM_SPARE1_SPARE1_TRIM_MASK           (0x000000f8) /* <3:7> R:RW:0: */
#define PDSS_S8USBPD_TRIM_SPARE1_SPARE1_TRIM_POS            (3)


/*
 * S8USBPD Spare2 Trim Register
 */
#define PDSS_S8USBPD_TRIM_SPARE2_ADDRESS(m)                 (0x400aff24 + ((m) * (0x10000)))
#define PDSS_S8USBPD_TRIM_SPARE2(m)                         (*(volatile uint32_t *)(0x400aff24 + ((m) * 0x10000)))
#define PDSS_S8USBPD_TRIM_SPARE2_DEFAULT                    (0x00000000)

/*
 * Spare2 Trim
 */
#define PDSS_S8USBPD_TRIM_SPARE2_SPARE2_TRIM_MASK           (0x000000ff) /* <0:7> R:RW:0: */
#define PDSS_S8USBPD_TRIM_SPARE2_SPARE2_TRIM_POS            (0)


/*
 * Configuration register
 */
#define CPUSS_CONFIG_ADDRESS                                (0x40100000)
#define CPUSS_CONFIG                                        (*(volatile uint32_t *)(0x40100000))
#define CPUSS_CONFIG_DEFAULT                                (0x00000000)

/*
 * 0': Vector Table is located at 0x0000:0000 in flash
 * '1': Vector Table is located at 0x2000:0000 in SRAM
 * Note that vectors for RESET and FAULT are always fetched from ROM. Value
 * in flash/RAM is ignored for these vectors.
 */
#define CPUSS_CONFIG_VECT_IN_RAM                            (1u << 0) /* <0:0> R:RW:0: */


/*
 * SYSCALL control register
 * Used to make system requests to SROM code.  System Requests transition
 * from User Mode to Privileged Mode.  See SAS for more details.  Firmware/ATE
 * should write CPUSS_SYSARG first and CPUSS_SYSREQ register next.
 */
#define CPUSS_SYSREQ_ADDRESS                                (0x40100004)
#define CPUSS_SYSREQ                                        (*(volatile uint32_t *)(0x40100004))
#define CPUSS_SYSREQ_DEFAULT                                (0x30000000)

/*
 * Opcode of the system call being requested.
 */
#define CPUSS_SYSREQ_SYSCALL_COMMAND_MASK                   (0x0000ffff) /* <0:15> :RW:0: */
#define CPUSS_SYSREQ_SYSCALL_COMMAND_POS                    (0)


/*
 * Disable Reset Vector fetch relocation:
 * '0': CPU accesses to locations 0x0000:0000 - 0x0000:0007 are redirected
 * to ROM.
 * '1': CPU accesses to locations 0x0000:0000 - 0x0000:0007 are made to flash.
 * Note that this field defaults to '0' on reset, ensuring actual reset vector
 * fetches are always made to ROM. Note that this field does not affect DAP
 * accesses. Flash DfT routines may set this bit to '1' to enable uninhibited
 * read-back of programmed data in the first flash page.
 */
#define CPUSS_SYSREQ_DIS_RESET_VECT_REL                     (1u << 27) /* <27:27> R:RW:0: */


/*
 * Indicates whether the system is in privileged ('1') or user mode ('0').
 * Only CPU SW executing from ROM can set this field to '1' when ROM_ACCESS_EN
 * is '1' (the CPU is executing a SystemCall NMI interrupt handler). Any
 * other write to this field sets is to '0'. This field is used as the AHB-Lite
 * hprot[1] signal to implement Cypress proprietary user/privileged modes.
 * These modes are used to enable/disable access to specific MMIO registers
 * and memory regions.
 */
#define CPUSS_SYSREQ_PRIVILEGED                             (1u << 28) /* <28:28> A:RW:1: */


/*
 * Indicates that executing from Boot ROM is enabled. HW sets this field
 * to '1', on reset or when the SystemCall NMI vector is fetched from Boot
 * ROM. HW sets this field to '0', when the CPU is NOT executing from either
 * Boot or System ROM. This bit is used for debug purposes only.
 */
#define CPUSS_SYSREQ_ROM_ACCESS_EN                          (1u << 29) /* <29:29> RW:R:1: */


/*
 * Indicates the source of the write access to the SYSREQ register.
 * '0': CPU write access.
 * '1': DAP write access.
 * HW sets this field when the SYSREQ register is written to and SYSCALL_REQ
 * is '0' (the last time it is set is when SW sets SYSCALL_REQ from '0' to
 * '1').
 */
#define CPUSS_SYSREQ_HMASTER_0                              (1u << 30) /* <30:30> A:R:0: */


/*
 * CPU/DAP writes a '1' to this field to request a SystemCall. The HMASTER_0
 * field indicates the source of the write access. Setting this field to
 * '1' immediate results in a NMI. The SystemCall NMI interrupt handler sets
 * this field to '0' after servicing the request.
 */
#define CPUSS_SYSREQ_SYSCALL_REQ                            (1u << 31) /* <31:31> R:RW:0: */


/*
 * SYSARG control register
 * Used to make system requests to SROM code.  System Requests transition
 * from User Mode to Privileged Mode.  See SAS for more details.  Firmware/ATE
 * should write CPUSS_SYSARG first and CPUSS_SYSREQ register next.
 */
#define CPUSS_SYSARG_ADDRESS                                (0x40100008)
#define CPUSS_SYSARG                                        (*(volatile uint32_t *)(0x40100008))
#define CPUSS_SYSARG_DEFAULT                                (0x00000000)

/*
 * Argument to System Call specified in SYSREQ. Semantics of argument depends
 * on system call made. Typically a pointer to a parameter block.
 */
#define CPUSS_SYSARG_SYSCALL_ARG_MASK                       (0xffffffff) /* <0:31> :RW:0: */
#define CPUSS_SYSARG_SYSCALL_ARG_POS                        (0)


/*
 * Protection control register
 */
#define CPUSS_PROTECTION_ADDRESS                            (0x4010000c)
#define CPUSS_PROTECTION                                    (*(volatile uint32_t *)(0x4010000c))
#define CPUSS_PROTECTION_DEFAULT                            (0x0000000f)

/*
 * Current protection mode; this field is available as a global signal everywhere
 * in the system. Writes to this field are ignored when PROTECTION_LOCK is
 * '1':
 * 0b1xxx: BOOT
 * 0b01xx: KILL
 * 0b001x: PROTECTED
 * 0b0001: OPEN
 * 0b0000: VIRGIN (also used for DEAD mode, but then FLASH_LOCK is also set)
 */
#define CPUSS_PROTECTION_PROTECTION_MODE_MASK               (0x0000000f) /* <0:3> A:RW:15: */
#define CPUSS_PROTECTION_PROTECTION_MODE_POS                (0)


/*
 * Setting this bit will force SPCIF.ADDRESS.AXA to be ignored, which prevents
 * SM Flash from being erased or overwritten. It is used to indicate the
 * DEAD protection mode. Writes to this field are ignored when PROTECTION_LOCK
 * is '1'
 */
#define CPUSS_PROTECTION_FLASH_LOCK                         (1u << 30) /* <30:30> A:RW:0:SECURE_ACCESS */


/*
 * Setting this field will block (ignore) any further writes to the PROTECTION_MODE
 * field in this register. Once '1', this field cannot be cleared.
 */
#define CPUSS_PROTECTION_PROTECTION_LOCK                    (1u << 31) /* <31:31> R:RW1S:0: */


/*
 * ROM privilege register
 * ROM memory has a maximum capacity of 128 KByte. ROM is partitioned into
 * Boot ROM and System ROM. These two partitions are located back-to-back
 * in the system address space. The Boot ROM capacity is 4, 8, 16, or 32
 * KByte. The System ROM partition capacity equals the ROM capacity minus
 * the Boot ROM partition capacity.
 *
 * User mode accesses to a privileged address result in an AHB-Lite bus error.
 * If the ROM memory capacity is not a power of two, the ROM memory region
 * has an unpopulated/unaccounted memory are (at the end of the ROM memory
 * region). A user mode access to an unpopulated, privileged area (as indicated
 * by the LIMIT field(s)) address results in an AHB-Lite bus error. A user
 * mode access to a unpopulated area, without any access violations, behaves
 * as follows: Reads return "0" and writes are ignore (RZWI).
 */
#define CPUSS_PRIV_ROM_ADDRESS                              (0x40100010)
#define CPUSS_PRIV_ROM                                      (*(volatile uint32_t *)(0x40100010))
#define CPUSS_PRIV_ROM_DEFAULT                              (0x00000000)

/*
 * Indicates the limit where the privileged area of the Boot ROM partition
 * starts in increments of 256 Bytes.
 * "0":  Entire Boot ROM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 * ...
 * BROM_PROT_LIMIT >= "Boot ROM partition capacity": Entire Boot ROM partition
 * is user mode accessible.
 */
#define CPUSS_PRIV_ROM_BROM_PROT_LIMIT_MASK                 (0x000000ff) /* <0:7> R:RW:0: */
#define CPUSS_PRIV_ROM_BROM_PROT_LIMIT_POS                  (0)


/*
 * RAM privilege register
 * User mode accesses to a privileged address result in an AHB-Lite bus error.
 * If the RAM memory capacity is not a power of two, the RAM memory region
 * has an unpopulated/unaccounted memory are (at the end of the RAM memory
 * region). A user mode access to an unpopulated, privileged area (as indicated
 * by the LIMIT field(s)) address results in an AHB-Lite bus error. A user
 * mode access to a unpopulated area, without any access violations, behaves
 * as follows: Reads return "0" and writes are ignore (RZWI).
 */
#define CPUSS_PRIV_RAM_ADDRESS                              (0x40100014)
#define CPUSS_PRIV_RAM                                      (*(volatile uint32_t *)(0x40100014))
#define CPUSS_PRIV_RAM_DEFAULT                              (0x00000000)

/*
 * Indicates the limit where the privileged area of SRAM starts in increments
 * of 256 Bytes.
 * "0":  Entire SRAM is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the SRAM indicates that the entire
 * SRAM is user mode accessible.
 */
#define CPUSS_PRIV_RAM_RAM_PROT_LIMIT_MASK                  (0x000001ff) /* <0:8> R:RW:0: */
#define CPUSS_PRIV_RAM_RAM_PROT_LIMIT_POS                   (0)


/*
 * ROM privilege register
 * User mode accesses to a privileged address result in an+C48 AHB-Lite bus
 * error. If the regular flash memory capacity is not a power of two, the
 * regular flash memory region has an unpopulated/unaccounted memory are
 * (at the end of the regular flash memory region). A user mode access to
 * an unpopulated, privileged area (as indicated by the LIMIT field(s)) address
 * results in an AHB-Lite bus error. A user mode access to a unpopulated
 * area, without any access violations, behaves as follows: Reads return
 * "0" and writes are ignore (RZWI).
 */
#define CPUSS_PRIV_FLASH_ADDRESS                            (0x40100018)
#define CPUSS_PRIV_FLASH                                    (*(volatile uint32_t *)(0x40100018))
#define CPUSS_PRIV_FLASH_DEFAULT                            (0x00000000)

/*
 * Indicates the limit where the privileged area of flash starts in increments
 * of 256 Bytes.
 * "0":  Entire flash is Privileged.
 * "1":  First 256 Bytes are User accessable.
 *
 * Any number larger than the size of the flash indicates that the entire
 * flash is user mode accessible. Note that SuperVisory rows are always User
 * accessable.
 *
 * If FLASH_PROT_LIMIT defines a non-empty privileged area, the boot ROM
 * will assume that a system call table exists at the beginning of the Flash
 * privileged area and use it for all SystemCalls made using SYSREQ.
 */
#define CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_MASK              (0x000007ff) /* <0:10> R:RW:0: */
#define CPUSS_PRIV_FLASH_FLASH_PROT_LIMIT_POS               (0)


/*
 * Wounding register
 * Wounding is based on the FLASH/SRAM memory address range. This range is
 * always the next power of 2 multiple of the FLASH/SRAM memory capacity.
 * E.g., a 48 KByte SRAM capacity has a 64 KByte memory address range. With
 * RAM_WOUND is "0", all 48 KByte SRAM capacity is accessible. With RAM_WOUND
 * is "1", the first 32 KByte SRAM capacity is accessible. With RAM_WOUND
 * is "2", the first 16 KByte SRAM capacity is accessible.
 */
#define CPUSS_WOUNDING_ADDRESS                              (0x4010001c)
#define CPUSS_WOUNDING                                      (*(volatile uint32_t *)(0x4010001c))
#define CPUSS_WOUNDING_DEFAULT                              (0x00000000)

/*
 * Indicates the amount of accessible RAM 0 memory capacitty in this part.
 * The value in this field is effectively write-once (it is only possible
 * to set bits, not clear them). The remainder portion of SRAM is not accessible
 * and will return an AHB-Lite bus error.
 * "0": entire memory accessible
 * "1": first 1/2 of the memory accessible
 * "2": first 1/4 of the memory accessible
 * "3": first 1/8 of the memory accessible
 * "4": first 1/16 of the memory accessible
 * "5": first 1/32 of the memory accessible
 * "6": first 1/64 of the memory accessible
 * "7": first 1/128 of the memory accessible
 */
#define CPUSS_WOUNDING_RAM_WOUND_MASK                       (0x00070000) /* <16:18> R:RW1S:0: */
#define CPUSS_WOUNDING_RAM_WOUND_POS                        (16)


/*
 * Indicates the amount of accessible flash in this part. The value in this
 * field is effectively write-once (it is only possible to set bits, not
 * clear them). The remainder portion of flash is not accessible and will
 * return an AHB-Lite bus error.
 * "0": entire memory accessible
 * "1": first 1/2 of the memory accessible
 * "2": first 1/4 of the memory accessible
 * "3": first 1/8 of the memory accessible
 * "4": first 1/16 of the memory accessible
 * "5": first 1/32 of the memory accessible
 * "6": first 1/64 of the memory accessible
 * "7": first 1/128 of the memory accessible (used for the DEAD protection
 * mode)
 */
#define CPUSS_WOUNDING_FLASH_WOUND_MASK                     (0x00700000) /* <20:22> R:RW1S:0: */
#define CPUSS_WOUNDING_FLASH_WOUND_POS                      (20)


/*
 * DFT Select Register
 * Controls power switches of FLASH, ROM, SRAM macro(s) in CPUMEMSS. Note
 * that for StreetFighther this functionality is NOT available due to an
 * omission during IP development.
 */
#define CPUSS_DFT_ADDRESS                                   (0x4010002c)
#define CPUSS_DFT                                           (*(volatile uint32_t *)(0x4010002c))
#define CPUSS_DFT_DEFAULT                                   (0x00000000)

/*
 * Forcibly powers down Flash macro(s) when set and TEST_MODE is active.
 */
#define CPUSS_DFT_FLASH_PD                                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * Forcibly powers down system ROM macro(s) when set and TEST_MODE is active.
 */
#define CPUSS_DFT_ROM_PD                                    (1u << 1) /* <1:1> R:RW:0: */


/*
 * Forcibly powers down system SRAM macro(s) periphery logic when set and
 * TEST_MODE is active. Memory array is connected to Vcchib and cannot be
 * powered down.
 */
#define CPUSS_DFT_RAM_PD                                    (1u << 2) /* <2:2> R:RW:0: */


/*
 * Forcibly powers down DW/DMA controller SRAM macro(s) (memory array and
 * pheriphery) when set and TEST_MODE is active.
 */
#define CPUSS_DFT_DMAC_RAM_PD                               (1u << 3) /* <3:3> R:RW:0: */


/*
 * FLASH control register
 */
#define CPUSS_FLASH_CTL_ADDRESS                             (0x40100030)
#define CPUSS_FLASH_CTL                                     (*(volatile uint32_t *)(0x40100030))
#define CPUSS_FLASH_CTL_DEFAULT                             (0x00000000)

/*
 * Amount of ROM wait states:
 * "0": 0 wait states (fast flash: [0, 24] MHz system frequency, slow flash:
 * [0, 16] MHz system frequency)
 * "1": 1 wait state (fast flash: [24, 48] MHz system frequency, slow flash:
 * [16, 32] MHz system frequency)
 * "2": 2 wait states (slow flash: [32, 48] MHz system frequency)
 * "3": undefined
 */
#define CPUSS_FLASH_CTL_FLASH_WS_MASK                       (0x00000003) /* <0:1> R:RW:0: */
#define CPUSS_FLASH_CTL_FLASH_WS_POS                        (0)


/*
 * Prefetch enable:
 * '0': disabled. This is a desirable seeting when FLASH_WS is "0" or when
 * predictable execution behavior is required.
 * '1': enabled.
 */
#define CPUSS_FLASH_CTL_PREF_EN                             (1u << 4) /* <4:4> R:RW:0: */


/*
 * 1': Invalidates the content of the flash controller's buffers.
 */
#define CPUSS_FLASH_CTL_FLASH_INVALIDATE                    (1u << 8) /* <8:8> RW1C:RW:0: */


/*
 * ROM control register
 */
#define CPUSS_ROM_CTL_ADDRESS                               (0x40100034)
#define CPUSS_ROM_CTL                                       (*(volatile uint32_t *)(0x40100034))
#define CPUSS_ROM_CTL_DEFAULT                               (0x00000000)

/*
 * Amount of ROM wait states:
 * '0': 0 wait states. Use this setting for newer, faster ROM design. Use
 * this setting for older, slower ROM design and frequencies in the range
 * [0, 24] MHz.
 * '1': 1 wait state. Use this setting for older, slower ROM design and frequencies
 * in the range <24, 48] MHz.
 *
 * CPUSSv2 supports two types of ROM memory: an older, slower design (operating
 * at up to 24 MHz) and a newer, faster design (operating at up to 48 MHz).
 * The older design requires 1 wait state for frequencies above 24 MHz. The
 * newer design never requires wait states. All chips after Street Fighter
 * will use the newer design. As a result, all chips after Street Fighter
 * can always use 0 wait states.
 */
#define CPUSS_ROM_CTL_ROM_WS                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Bist command register
 */
#define CPUSS_BIST_CMD_ADDRESS                              (0x40100040)
#define CPUSS_BIST_CMD                                      (*(volatile uint32_t *)(0x40100040))
#define CPUSS_BIST_CMD_DEFAULT                              (0x00000000)

/*
 * 1': Start SRAM BIST. Hardware set this field to '0' when BIST is completed.
 * SRAM BIST is functional up to 48 MHz. Note that this field is mutually
 * exclusive with the "SROM_GO" field.
 */
#define CPUSS_BIST_CMD_SRAM_GO                              (1u << 0) /* <0:0> RW1C:RW:0: */


/*
 * 1': Start ROM BIST. Hardware set this field to '0' when BIST is completed.
 * SROM BIST is functional up to 24 MHz.
 */
#define CPUSS_BIST_CMD_SROM_GO                              (1u << 1) /* <1:1> RW1C:RW:0: */


/*
 * BIST data register
 */
#define CPUSS_BIST_DATA_ADDRESS                             (0x40100044)
#define CPUSS_BIST_DATA                                     (*(volatile uint32_t *)(0x40100044))
#define CPUSS_BIST_DATA_DEFAULT                             (0x00000000)

/*
 * Data that is written into a SRAM during a W0 substep (~BIST_DATA.DATA
 * is written into a SRAM during a W1 substep).
 */
#define CPUSS_BIST_DATA_DATA_MASK                           (0xffffffff) /* <0:31> R:RW:0: */
#define CPUSS_BIST_DATA_DATA_POS                            (0)


/*
 * BIST control register
 */
#define CPUSS_BIST_CTL_ADDRESS                              (0x4010004c)
#define CPUSS_BIST_CTL                                      (*(volatile uint32_t *)(0x4010004c))
#define CPUSS_BIST_CTL_DEFAULT                              (0x00000000)

/*
 * Hot-one mask for the SRAMs for which the BIST is performed.
 */
#define CPUSS_BIST_CTL_SRAMS_ENABLED_MASK                   (0x00003fff) /* <0:13> R:RW:0:BIST_SRAM_NR */
#define CPUSS_BIST_CTL_SRAMS_ENABLED_POS                    (0)


/*
 * Specifies how the SRAM BIST addresses are generated (should be set to
 * '0' for SROM BIST):
 * '0': Column address is incremented/decremented till it reaches its maximum/minimum
 * value. Once it reach its maximum/minimum value, it is set to its mimimum/maximum
 * value and only then is the row address incremented/decremented.
 * '1': Row address is incremented/decremented till it reaches its maximum/minimum
 * value. Once it reach its maximum/minimum value, it is set to its mimimum/maximum
 * value and only then is the column address incremented/decremented.
 */
#define CPUSS_BIST_CTL_ROW_FIRST                            (1u << 20) /* <20:20> R:RW:0: */


/*
 * BIST step 0 control register
 */
#define CPUSS_BIST_STEP0_CTL_ADDRESS                        (0x40100050)
#define CPUSS_BIST_STEP0_CTL                                (*(volatile uint32_t *)(0x40100050))
#define CPUSS_BIST_STEP0_CTL_DEFAULT                        (0x00000000)

/*
 * Specifies what sequence of SRAM BIST steps (R0, R1, W0, W1) is performed
 * (not used for SROM BIST):
 * "0": W0. Write SRAM with BIST_DATA.DATA.
 * "1": W1. Write SRAM with ~BIST_DATA.DATA.
 * "2": R0. Read SRAM and compare output to BIST_DATA.DATA.
 * "3": R1. Read SRAM and compare output to ~BIST_DATA.DATA.
 * "4": W0, R0. Write SRAM with BIST_DATA.DATA, followed by read SRAM and
 * compare output to BIST_DATA.DATA (all to the same address).
 * "5": R0, W1.
 * "6": R1, W0.
 * "7": R0, W1, R1.
 * "8": R1, W0, R0.
 * "9": R0, W1, W0.
 * "10": R1, W0, W1.
 * "11": R0, W1, W0, W1.
 * "12": R1, W0, W1, W0.
 * "13": R0, W1, R1, W0.
 * "14": R1, W0, R0, W1.
 * "15": R0, W1, R1, W0, R0, W1.
 */
#define CPUSS_BIST_STEP0_CTL_OPCODE_MASK                    (0x0000000f) /* <0:3> R:RW:0: */
#define CPUSS_BIST_STEP0_CTL_OPCODE_POS                     (0)


/*
 * Specifies direction in which SRAM BIST steps through addresses (not used
 * for SROM BIST):
 * ''0': BIST steps through the SRAM from the maximum row and column addresses
 * (as specified by a design time configurtion parameter when ADDR_START_ENABLED
 * is '0' and as specified by BIST_ADDR_START when ADDR_START_ENABLED is
 * '1')  to the minimum row and column addresses.
 * '1': BIST steps through the SRAM from the minimum row and column addresses
 * ("0" when ADDR_START_ENABLED is '0' and as specified by BIST_ADDR_START
 * when ADDR_START_ENABLED is '1') to the maximum row and column addresses.
 */
#define CPUSS_BIST_STEP0_CTL_UP                             (1u << 4) /* <4:4> R:RW:0: */


/*
 * BIST status register
 */
#define CPUSS_BIST_STATUS_ADDRESS                           (0x40100080)
#define CPUSS_BIST_STATUS                                   (*(volatile uint32_t *)(0x40100080))
#define CPUSS_BIST_STATUS_DEFAULT                           (0x00000000)

/*
 * BIST substep (either R0, R1, W0, or W1).
 */
#define CPUSS_BIST_STATUS_SUB_STEP_MASK                     (0x00000007) /* <0:2> W:R:Undefined: */
#define CPUSS_BIST_STATUS_SUB_STEP_POS                      (0)


/*
 * BIST step (step i uses BIST_STEPi_CTL).
 */
#define CPUSS_BIST_STATUS_STEP_MASK                         (0x00000700) /* <8:10> W:R:Undefined: */
#define CPUSS_BIST_STATUS_STEP_POS                          (8)


/*
 * SRAM identifier.
 */
#define CPUSS_BIST_STATUS_SRAM_MASK                         (0x000f0000) /* <16:19> W:R:Undefined: */
#define CPUSS_BIST_STATUS_SRAM_POS                          (16)


/*
 * 0': BIST passed.
 * '1': BIST failed (SRAM match error or ROM MISR check error). BIST error
 * information is found in SRAM, STEP, SUB_STEP fields and BIST_DATA_ACT,
 * BIST_DATA_EXP and BIST_ADDR registers.
 */
#define CPUSS_BIST_STATUS_FAIL                              (1u << 24) /* <24:24> W1S:RW:0: */


/*
 * BIST data expected register
 */
#define CPUSS_BIST_DATA_ACT_ADDRESS                         (0x40100084)
#define CPUSS_BIST_DATA_ACT                                 (*(volatile uint32_t *)(0x40100084))
#define CPUSS_BIST_DATA_ACT_DEFAULT                         (0x00000000)

/*
 * For SRAM BIST, this field is the SRAM output data that caused a BIST failure.
 * For ROM BIST, this field is the calculated Mulitple Input Shift Register
 * (MISR).
 */
#define CPUSS_BIST_DATA_ACT_DATA_MASK                       (0xffffffff) /* <0:31> W:R:Undefined: */
#define CPUSS_BIST_DATA_ACT_DATA_POS                        (0)


/*
 * BIST data actual register
 */
#define CPUSS_BIST_DATA_EXP_ADDRESS                         (0x40100088)
#define CPUSS_BIST_DATA_EXP                                 (*(volatile uint32_t *)(0x40100088))
#define CPUSS_BIST_DATA_EXP_DEFAULT                         (0x00000000)

/*
 * For SRAM BIST. This field is the expected data from SRAM. This value is
 * BIST_DATA.DATA for a R0 substep and ~BIST_DATA.DATA for a R1 substep.
 */
#define CPUSS_BIST_DATA_EXP_DATA_MASK                       (0xffffffff) /* <0:31> W:R:Undefined: */
#define CPUSS_BIST_DATA_EXP_DATA_POS                        (0)


/*
 * BIST address register
 */
#define CPUSS_BIST_ADDR_ADDRESS                             (0x4010008c)
#define CPUSS_BIST_ADDR                                     (*(volatile uint32_t *)(0x4010008c))
#define CPUSS_BIST_ADDR_DEFAULT                             (0x00000000)

/*
 * Current column address.
 */
#define CPUSS_BIST_ADDR_COL_ADDR_MASK                       (0x00000fff) /* <0:11> W:R:Undefined: */
#define CPUSS_BIST_ADDR_COL_ADDR_POS                        (0)


/*
 * Current row address.
 */
#define CPUSS_BIST_ADDR_ROW_ADDR_MASK                       (0x0fff0000) /* <16:27> W:R:Undefined: */
#define CPUSS_BIST_ADDR_ROW_ADDR_POS                        (16)


/*
 * BIST SROM Multiple Input Shift Register (MISR)
 */
#define CPUSS_BIST_MISR_ADDRESS                             (0x40100090)
#define CPUSS_BIST_MISR                                     (*(volatile uint32_t *)(0x40100090))
#define CPUSS_BIST_MISR_DEFAULT                             (0x00000000)

/*
 * Current value of ROM Multiple Input Shift Register (MISR).
 */
#define CPUSS_BIST_MISR_DATA_MASK                           (0xffffffff) /* <0:31> W:R:Undefined: */
#define CPUSS_BIST_MISR_DATA_POS                            (0)


/*
 * Parallel Test Mode Control Register
 * This register determines the test interface (SWD or PTM) that connects
 * the ATE to the device.
 */
#define CPUSS_PTM_CTL_ADDRESS                               (0x401000c0)
#define CPUSS_PTM_CTL                                       (*(volatile uint32_t *)(0x401000c0))
#define CPUSS_PTM_CTL_DEFAULT                               (0x00000000)

/*
 * 0': SWD mode.
 * '1': PTM mode.
 * This bit is typically set to '1' through the SWD interface to switch to
 * the PTM interface and it is typically set to '0' through the PTM interface
 * to switch to the SWD interface. This bit replaces the m0s8tst IP's CTRL.PTM_MODE_EN
 * MMIO register field, which is rendered ineffective when using CPUSSv2.
 */
#define CPUSS_PTM_CTL_PTM_EN                                (1u << 0) /* <0:0> R:RW:0: */


/*
 * Flash/NVL geometry information
 * This register indicates the flash/NVLatch geometry parameters for the
 * current device. This does not take wounding into account, which may restrict
 * access to flash memory. Note that the default values of the read-only
 * fields are determined by the CPUSS design time configuration.
 */
#define SPCIF_GEOMETRY_ADDRESS                              (0x40110000)
#define SPCIF_GEOMETRY                                      (*(volatile uint32_t *)(0x40110000))
#define SPCIF_GEOMETRY_DEFAULT                              (0x00000000)

/*
 * Regular flash capacity in 256 Byte multiples (chip dependent). If multiple
 * flash macros are present, this field provides the flash capacity of all
 * flash macros together:
 * "0": 256 Bytes.
 * "1": 2*256 Bytes.
 * ...
 * "65535": 65536*256 Bytes.
 */
#define SPCIF_GEOMETRY_FLASH_MASK                           (0x0000ffff) /* <0:15> W:R:Undefined: */
#define SPCIF_GEOMETRY_FLASH_POS                            (0)


/*
 * Supervisory flash capacity in 256 Byte multiples (chip dependent). If
 * multiple flash macros are present, this field provides the supervisory
 * flash capacity of all flash macros together:
 * "0": 256 Bytes.
 * "1": 2*256 Bytes.
 * ...
 * "15": 16*256 Bytes.
 */
#define SPCIF_GEOMETRY_SFLASH_MASK                          (0x000f0000) /* <16:19> W:R:Undefined: */
#define SPCIF_GEOMETRY_SFLASH_POS                           (16)


/*
 * Number of flash macros (chip dependent):
 * "0": 1 flash macro
 * "1": 2 flash macros
 * "2": 3 flash macros
 * "3": 4 flash macros
 */
#define SPCIF_GEOMETRY_NUM_FLASH_MASK                       (0x00300000) /* <20:21> W:R:Undefined: */
#define SPCIF_GEOMETRY_NUM_FLASH_POS                        (20)


/*
 * Page size in 64 Byte multiples (chip dependent):
 * "0": 64 byte
 * "1": 128 byte
 * "2": 192 byte
 * "3": 256 byte
 *
 * The page size is used to detemine the number of Bytes in a page for Flash
 * page based operations (e.g. PGM_PAGE).
 *
 * Note: the field name FLASH_ROW is misleading, as this field specifies
 * the number of Bytes in a page, rather than the number of Bytes in a row.
 * In a single plane flash macro architecture, a page consists of a single
 * row. However, in a multi plane flash macro architecture, a page consists
 * of multiple rows from different planes.
 */
#define SPCIF_GEOMETRY_FLASH_ROW_MASK                       (0x00c00000) /* <22:23> W:R:Undefined: */
#define SPCIF_GEOMETRY_FLASH_ROW_POS                        (22)


/*
 * 0': SRAM busy wait loop has not been copied.
 * '1': Busy wait loop has been written into SRAM.
 */
#define SPCIF_GEOMETRY_DE_CPD_LP                            (1u << 31) /* <31:31> :RW:0: */


/*
 * Flash/NVL address register
 * Specifies the flash macro, flash partition, flash page and flash byte
 * address on which a flash operation is performed or the NVLatch byte address
 * to be read/written.
 */
#define SPCIF_ADDRESS_ADDRESS                               (0x40110004)
#define SPCIF_ADDRESS                                       (*(volatile uint32_t *)(0x40110004))
#define SPCIF_ADDRESS_DEFAULT                               (0x00000000)

/*
 * Specifies a Byte address:
 * - For flash, this is the byte in a (macro, page) as specified by PAGE_ADDR
 * (for write operations to page byte latches). MSBs are ignored when they
 * exceed the page range; e.g. for 64 byte pages, BYTE_ADDR[7:6] are ignored.
 * - For NVLatch, this is the byte in the NVLatch array (for read/write operations).
 * MSBs are ignored when they exceed the NVLatch array; e.g. for 32 byte
 * array, BYTE_ADDR[7:5] are ignored.
 */
#define SPCIF_ADDRESS_BYTE_ADDR_MASK                        (0x000000ff) /* <0:7> RW:RW:0: */
#define SPCIF_ADDRESS_BYTE_ADDR_POS                         (0)


/*
 * 0': Don't change BYTE_ADDR after a write to FLASH_WR_DATA.
 * '1': Increment BYTE_ADDR after a write to FLASH_WR_DATA.
 */
#define SPCIF_ADDRESS_INC                                   (1u << 8) /* <8:8> R:RW:0: */


/*
 * Flash page address for program/erase operations. Supports up to 2048 pages:
 * 512 KByte total flash capacity for 256 Byte pages, 256 KByte total flash
 * capacity for 128 Byte pages, 128 KByte total flash capacity for 64 Byte
 * pages.
 *
 * MSBs are used to identify a specific flash macro; e.g. for a 4 flash macro
 * configuration, with 64 pages per flash macro, PAGE_ADDR[7:6] identifies
 * the flash macro. Note that ADDRESS.ALL can be used to select all flash
 * macros.
 *
 * MSBs are ignored when they exceed the total flash capacity; e.g. for a
 * total page capacity of 256 pages, PAGE_ADDR[10:8] are ignored. In addition,
 * MSBs are ignored when ADDRESS.ALL is '1'
 */
#define SPCIF_ADDRESS_PAGE_ADDR_MASK                        (0x07ff0000) /* <16:26> R:RW:0: */
#define SPCIF_ADDRESS_PAGE_ADDR_POS                         (16)


/*
 * Indicates whether address pertains to regular or supervisory flash pages.
 * This bit has no effect for NVLatch operations.
 */
#define SPCIF_ADDRESS_AXA                                   (1u << 28) /* <28:28> R:RW:0: */


/*
 * Controls the flash macro(s) on which the flash operation is performed.
 * '0': Select flash macro identified by MSBs of ADDRESS.PAGE_ADDR.
 * '1': Select all flash macros; the same operation is performed on all flash
 * macros in parallel. This mode is useful for test purposes.
 */
#define SPCIF_ADDRESS_ALL                                   (1u << 29) /* <29:29> R:RW:0: */


/*
 * SPCIF timer register
 * Provides the timer period reload value
 */
#define SPCIF_TIMER_ADDRESS                                 (0x40110008)
#define SPCIF_TIMER                                         (*(volatile uint32_t *)(0x40110008))
#define SPCIF_TIMER_DEFAULT                                 (0x00000000)

/*
 * Period of timer in "clk_timer" clock cycles; start of the time counter
 * value. When SW sets FLASH_CONTROL.START_COUNT to '1', the timer counter
 * is counted down to "0" from the start counter value as provided by this
 * register field. Note that TIMER.PERIOD should only be written when FLASH_CONTROL.START_COUNT
 * is '0'. If TIMER.PERIOD is written when FLASH_CONTOL.START_COUNT is '1',
 * an AHB-Lite bus error is generated and the write is dropped/does not take
 * effect.
 *
 * For SRSSv2, the timer counter operates in asynchronous mode and "clk_timer"
 * is the 36 MHz IMO clock ("clk_imo"), which is asynchronous to the system
 * clock ("clk_sys"). In asynchronous mode, a read from this register field
 * returns "0".
 *
 * For SRSS-Lite, the timer operates in asynchronous mode and "clk_timer"
 * is the system clock ("clk_sys"). In synchronous mode (for SRSS-Lite and
 * "clk_timer" is "clk_sys"), a read from this register field returns the
 * current timer counter value.
 */
#define SPCIF_TIMER_PERIOD_MASK                             (0xffffffff) /* <0:31> A:RW:0: */
#define SPCIF_TIMER_PERIOD_POS                              (0)


/*
 * Flash control register
 * Controls program/erase operations on the Flash
 */
#define SPCIF_FLASH_CONTROL_ADDRESS                         (0x4011000c)
#define SPCIF_FLASH_CONTROL                                 (*(volatile uint32_t *)(0x4011000c))
#define SPCIF_FLASH_CONTROL_DEFAULT                         (0x00000000)

/*
 * Flash mode:
 */
#define SPCIF_FLASH_CONTROL_MODE_MASK                       (0x0000000f) /* <0:3> R:RW:0: */
#define SPCIF_FLASH_CONTROL_MODE_POS                        (0)


/*
 * Flash Seq input, see BROS for details on values.
 */
#define SPCIF_FLASH_CONTROL_SEQ_MASK                        (0x00000030) /* <4:5> R:RW:0: */
#define SPCIF_FLASH_CONTROL_SEQ_POS                         (4)


/*
 * Enables a single cycle high time of the Aclk to the flash macro(s).  This
 * bit only has
 * effect when FLASH_CONTROL.NO_READS = '1'.
 */
#define SPCIF_FLASH_CONTROL_ACLK_EN                         (1u << 6) /* <6:6> RW:RW:0: */


/*
 * Configures the timer to assert the PE output while the timer is active:
 * '0': Pumps off.
 * '1': Pumps on while timer is counting.
 */
#define SPCIF_FLASH_CONTROL_PE_ENABLE                       (1u << 7) /* <7:7> R:RW:0: */


/*
 * Manual override for pump enables:
 * '0': Pumps operate normally (per PE_ENABLE).
 * '1': Pumps are enabled as long as this bit is set.
 */
#define SPCIF_FLASH_CONTROL_PE_OVERRIDE                     (1u << 8) /* <8:8> R:RW:0: */


/*
 * This bit should be enabled so that the timer can start counting down.
 * This bit is cleared by hardware when counter period expires. The start
 * timer counter value is provided by TIMER.PERIOD.
 */
#define SPCIF_FLASH_CONTROL_START_COUNT                     (1u << 9) /* <9:9> RW1C:RW1S:0: */


/*
 * Selects which of the 3 pump enable outputs to drive with the signal driven
 * from the PE_ENABLE and PE_OVERRIDE bits.  (3: Reserved)
 */
#define SPCIF_FLASH_CONTROL_PE_SELECT_MASK                  (0x00000c00) /* <10:11> R:RW:0: */
#define SPCIF_FLASH_CONTROL_PE_SELECT_POS                   (10)


/*
 * Firmware sets this bit to block the flash controller from issuing more
 * requests to flash. When '1', the flash controller returns an AHB-Lite
 * bus error on any read requests. Firmware must ensure no read accesses
 * are pending or must not take control of the flash immediately with setting
 * this bit.
 */
#define SPCIF_FLASH_CONTROL_NO_READS                        (1u << 14) /* <14:14> R:RW:0: */


/*
 * Flash write data register
 * Writing to this register writes one byte into the Flash page latch (at
 * position ADDRESS.WR_ADDR).  Multiple bytes can be written in sequence
 * by using ADDRESS.INC. Writes to this register may block the CPU bus for
 * a few cycles to enable the flash page latches to keep up.
 */
#define SPCIF_FLASH_WR_DATA_ADDRESS                         (0x40110010)
#define SPCIF_FLASH_WR_DATA                                 (*(volatile uint32_t *)(0x40110010))
#define SPCIF_FLASH_WR_DATA_DEFAULT                         (0x00000000)

/*
 * Data byte for a specific page byte latch
 */
#define SPCIF_FLASH_WR_DATA_DATA_MASK                       (0x000000ff) /* <0:7> A:RW:0: */
#define SPCIF_FLASH_WR_DATA_DATA_POS                        (0)


/*
 * Bookmark pointer register
 * Used as storage space for the SROM to write a bookmark. Bookmarks are
 * used as a pointer for cases when the SPCIF timer is setup to generate
 * an interrupt and control of the CM0 is given back to user code. When the
 * SPCIF interrupt is activated, the SROM uses this value to know where to
 * resume its function execution.
 */
#define SPCIF_BOOKMARK_ADDRESS                              (0x40110028)
#define SPCIF_BOOKMARK                                      (*(volatile uint32_t *)(0x40110028))
#define SPCIF_BOOKMARK_DEFAULT                              (0x00000000)

/*
 * SROM bookmark.
 */
#define SPCIF_BOOKMARK_BOOKMARK_MASK                        (0xffffffff) /* <0:31> R:RW:0: */
#define SPCIF_BOOKMARK_BOOKMARK_POS                         (0)


/*
 * Flash-Lite test mode register
 * Configures test mode parameters and signals for Flash macro. See m0s8cpussv2/flash
 * memory BROS for the use of this register.
 */
#define SPCIF_FMLT_DFT_ADDRESS                              (0x40110030)
#define SPCIF_FMLT_DFT                                      (*(volatile uint32_t *)(0x40110030))
#define SPCIF_FMLT_DFT_DEFAULT                              (0x00000000)

/*
 * Flash Test Mode Select, see BROS for details on each value.
 */
#define SPCIF_FMLT_DFT_FMLT_TM_MASK                         (0x0000001f) /* <0:4> R:RW:0: */
#define SPCIF_FMLT_DFT_FMLT_TM_POS                          (0)


/*
 * Positive/Negative Margin Voltage select.
 */
#define SPCIF_FMLT_DFT_FMLT_PNB                             (1u << 5) /* <5:5> R:RW:0: */

/*
 * Use Vneg
 */
#define SPCIF_FMLT_DFT_FMLT_PNB_FMLT_PNB_VNEG               (0)
/*
 * Use Vpos
 */
#define SPCIF_FMLT_DFT_FMLT_PNB_FMLT_PNB_VPOS               (1)

/*
 * When DFT.TM_XYDEC = 1, if one and only one WL or Y select address is on,
 * this signal is asserted.
 */
#define SPCIF_FMLT_DFT_TM_XYOUT                             (1u << 17) /* <17:17> RW:R:0: */


/*
 * SystemCall routines in ROM provide flash programming functionality. Flash
 * programming requires proper trimming of certain on-chip components. This
 * trimming information is part of the supervisory flash memory. The supervisory
 * flash memory is programmed using the SystemCall routines. As a result,
 * the initial programming of trimming information in the supervisory flash
 * memory can NOT rely on the information in the supervisory flash memory
 * (supervisory flash memory trimming information is not initialized). Instead,
 * this trimming information is provided at specific SRAM locations (known
 * by the SystemCall routines). This bit field specifies where the SystemCall
 * routines can find the trimming information.
 */
#define SPCIF_FMLT_DFT_PARAM_LOC                            (1u << 31) /* <31:31> R:RW:0: */


/*
 * SPCIF interrupt request register
 */
#define SPCIF_INTR_ADDRESS                                  (0x401107f0)
#define SPCIF_INTR                                          (*(volatile uint32_t *)(0x401107f0))
#define SPCIF_INTR_DEFAULT                                  (0x00000000)

/*
 * Timer counter value reaches "0". Set to '1', when event is detected. Write
 * INTR field with '1', to clear bit. Write INTR_SET field with '1', to set
 * bit.
 */
#define SPCIF_INTR_TIMER                                    (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * SPCIF interrupt set request register
 * When read, this register reflects the interrupt request register.
 */
#define SPCIF_INTR_SET_ADDRESS                              (0x401107f4)
#define SPCIF_INTR_SET                                      (*(volatile uint32_t *)(0x401107f4))
#define SPCIF_INTR_SET_DEFAULT                              (0x00000000)

/*
 * Write INTR_SET field with '1' to set corresponding INTR field.
 */
#define SPCIF_INTR_SET_TIMER                                (1u << 0) /* <0:0> A:RW1S:0: */


/*
 * SPCIF interrupt mask register
 */
#define SPCIF_INTR_MASK_ADDRESS                             (0x401107f8)
#define SPCIF_INTR_MASK                                     (*(volatile uint32_t *)(0x401107f8))
#define SPCIF_INTR_MASK_DEFAULT                             (0x00000000)

/*
 * Mask for corresponding field in INTR register.
 */
#define SPCIF_INTR_MASK_TIMER                               (1u << 0) /* <0:0> R:RW:0: */


/*
 * SPCIF interrupt masked request register
 * When read, this register reflects a bitwise and between the interrupt
 * request and mask registers.
 */
#define SPCIF_INTR_MASKED_ADDRESS                           (0x401107fc)
#define SPCIF_INTR_MASKED                                   (*(volatile uint32_t *)(0x401107fc))
#define SPCIF_INTR_MASKED_DEFAULT                           (0x00000000)

/*
 * Logical and of corresponding request and mask fields.
 */
#define SPCIF_INTR_MASKED_TIMER                             (1u << 0) /* <0:0> W:R:0: */


/*
 * Flash macro 0 DAC trim register 0
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. Note
 * that IDAC and SLOPE values for flash macro 0 are provided by the NVLatch
 * structure if this structure is present (after a DeepSleep reset "rst_sys_dpslp_n",
 * the HW copies the NVLatch structure IDAC and SLOPE values into this MMIO
 * register). If the NVLatch structure is NOT present, the boot process reads
 * flash macro 0 supervisory memory and writes to this register. This flash
 * macro read is an uncontrolled/untrimmed flash macro read in which the
 * default control/trim values apply. See m0s8cpussv2/flash memory BROS for
 * the use of this register.
 */
#define SPCIF_TRIM_M0_DAC0_ADDRESS                          (0x4011ff00)
#define SPCIF_TRIM_M0_DAC0                                  (*(volatile uint32_t *)(0x4011ff00))
#define SPCIF_TRIM_M0_DAC0_DEFAULT                          (0x00000000)

/*
 * Used as "idac[4:0]" flash macro 0 input. It enables trimming of the sense
 * amplifier reference current. Each increment causes an increase of ~0.6
 * µA:
 * "0": (0 + 2) * 0.6 µA = 1.2 µA
 * "1": (1 + 2) * 0.6 µA = 1.8 µA
 * ...
 * "31": (31 + 2) * 0.6 µA = 19.8 µA
 */
#define SPCIF_TRIM_M0_DAC0_IDAC_MASK                        (0x0000001f) /* <0:4> RW:RW:0: */
#define SPCIF_TRIM_M0_DAC0_IDAC_POS                         (0)


/*
 * Used as "slope[2:0]" flash macro 0 input (for regular FLASH) or used as
 * "sdac[2:0]" flash macro 0 input (for FLASH-Lite). It determines the slope
 * of the sense amplifier reference current as a function of temperature.
 * Each increment cause an increase of 5 nA/C:
 * "0": -5 nA/C
 * "1": 0 nA/C (no temperature dependency)
 * ...
 * "7":  30 nA/C
 */
#define SPCIF_TRIM_M0_DAC0_SLOPE_MASK                       (0x000000e0) /* <5:7> RW:RW:0: */
#define SPCIF_TRIM_M0_DAC0_SLOPE_POS                        (5)


/*
 * Flash macro 0 DAC trim register 1
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. The
 * boot process reads flash macro 0 supervisory memory and writes to this
 * register. This flash macro read is an uncontrolled/untrimmed flash macro
 * read in which the default control/trim values apply. See m0s8cpussv2/flash
 * memory BROS for the use of this register.
 */
#define SPCIF_TRIM_M0_DAC1_ADDRESS                          (0x4011ff04)
#define SPCIF_TRIM_M0_DAC1                                  (*(volatile uint32_t *)(0x4011ff04))
#define SPCIF_TRIM_M0_DAC1_DEFAULT                          (0x00000000)

/*
 * Used as "mdac[7:0]" flash macro 0 input. It controls the output margin
 * voltage VMARG as a function of input supplies VPOS and VNEG. Each increment
 * increases the output margin voltage as a multiple of VPOS/255 (positive
 * margin mode) or VNEG/255 (negative margin mode):
 * "0": 0 * VPOS/255 (positive margin mode); 0 * VNEG/255 (negative margin
 * mode)
 * "1": 1 * VPOS/255 (positive margin mode); 1 * VNEG/255 (negative margin
 * mode)
 * ...
 * "255": VPOS (positive margin mode); VNEG (negative margin mode)
 *
 * The flash macro 0 pumps (VPOS and VNEG) are also used for the NVLatch
 * structure and therefore the MDAC field needs to be set for NVLatch operations.
 */
#define SPCIF_TRIM_M0_DAC1_MDAC_MASK                        (0x000000ff) /* <0:7> R:RW:0: */
#define SPCIF_TRIM_M0_DAC1_MDAC_POS                         (0)


/*
 * Flash macro 0 DAC trim register 2
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. The
 * boot process reads flash macro 0 supervisory memory and writes to this
 * register. This flash macro read is an uncontrolled/untrimmed flash macro
 * read in which the default control/trim values apply. See m0s8cpussv2/flash
 * memory BROS for the use of this register.
 */
#define SPCIF_TRIM_M0_DAC2_ADDRESS                          (0x4011ff08)
#define SPCIF_TRIM_M0_DAC2                                  (*(volatile uint32_t *)(0x4011ff08))
#define SPCIF_TRIM_M0_DAC2_DEFAULT                          (0x00000000)

/*
 * Used as "pdac[3:0]" flash macro 0 input. It control the trimming of the
 * positive pump output voltage VPOS. Each increment causes an increase of
 * ~0.1 V. In normal mode:
 * "0": VPOS = 5.9 + 0 * 0.1 V = 5.9 V
 * "1": VPOS = 5.9 + 1 * 0.1 V = 6.0 V
 * ...
 * "15": VPOS = 5.9 + 15 * 0.1 V = 7.4 V
 *
 * The flash macro 0 pumps (VPOS and VNEG) are also used for the NVLatch
 * structure and therefore the PDAC field needs to be set for NVLatch operations.
 */
#define SPCIF_TRIM_M0_DAC2_PDAC_MASK                        (0x0000000f) /* <0:3> R:RW:0: */
#define SPCIF_TRIM_M0_DAC2_PDAC_POS                         (0)


/*
 * Used as "ndac[3:0]" flash macro 0 input. It control the trimming of the
 * negative pump output voltage VNEG. Each increment causes an decrease of
 * ~0.1 V. In normal mode:
 * "0": VNEG = -3.0 - 0 * 0.1 V = -3.0 V
 * "1": VNEG = -3.0 - 1 * 0.1 V = -3.1 V
 * ...
 * "15": VNEG = -3.0 - 15 * 0.1 V = -4.5 V
 *
 * The flash macro 0 pumps (VPOS and VNEG) are also used for the NVLatch
 * structure and therefore the NDAC field needs to be set for NVLatch operations.
 */
#define SPCIF_TRIM_M0_DAC2_NDAC_MASK                        (0x000000f0) /* <4:7> R:RW:0: */
#define SPCIF_TRIM_M0_DAC2_NDAC_POS                         (4)


/*
 * Flash macro 0 DAC trim register 3
 * Control/trim values for flash macro 0 DACs. These values are determined
 * at manufacturing and written in flash macro 0 supervisory memory. The
 * boot process reads flash macro 0 supervisory memory and writes to this
 * register. See m0s8cpussv2/flash memory BROS for the use of this register.
 */
#define SPCIF_TRIM_M0_DAC3_ADDRESS                          (0x4011ff0c)
#define SPCIF_TRIM_M0_DAC3                                  (*(volatile uint32_t *)(0x4011ff0c))
#define SPCIF_TRIM_M0_DAC3_DEFAULT                          (0x00000000)

/*
 * Used as "bdac[3:0]" flash macro 0 input.
 */
#define SPCIF_TRIM_M0_DAC3_BDAC_MASK                        (0x0000000f) /* <0:3> R:RW:0: */
#define SPCIF_TRIM_M0_DAC3_BDAC_POS                         (0)


/*
 * Used as "cdac[2:0]" flash macro 0 input (for FLASH-Lite). Temperature
 * compensated trim DAC. It controls Vctat slope for VPOS.
 */
#define SPCIF_TRIM_M0_DAC3_CDAC_MASK                        (0x00000070) /* <4:6> R:RW:0:FMLT_OR_S8FS */
#define SPCIF_TRIM_M0_DAC3_CDAC_POS                         (4)


/*
 * Watchpoint Comparator Configuration
 * Defines the number of comparators implemented
 */
#define CM0_DWT_CTRL_ADDRESS                                (0xe0001000)
#define CM0_DWT_CTRL                                        (*(volatile uint32_t *)(0xe0001000))
#define CM0_DWT_CTRL_DEFAULT                                (0x20000000)

/*
 * Number of comparators available
 */
#define CM0_DWT_CTRL_NUMCOMP_MASK                           (0xf0000000) /* <28:31> :R:2: */
#define CM0_DWT_CTRL_NUMCOMP_POS                            (28)


/*
 * Watchpoint Comparator PC Sample
 * Samples the current value of the program counter.  Unless DWT_PCSR reads
 * as 0xFFFFFFFF, under the conditions described in Program counter sampling
 * support on page C1-344, bit [0] is RAZ. When RAZ, bit [0] does not reflect
 * instruction set state as is the case with similar functionality in
 * other ARM architecture profiles.
 */
#define CM0_DWT_PCSR_ADDRESS                                (0xe000101c)
#define CM0_DWT_PCSR                                        (*(volatile uint32_t *)(0xe000101c))
#define CM0_DWT_PCSR_DEFAULT                                (0x00000000)

/*
 * Executed Instruction Address sample value
 */
#define CM0_DWT_PCSR_EIASAMPLE_MASK                         (0xffffffff) /* <0:31> W:R:X: */
#define CM0_DWT_PCSR_EIASAMPLE_POS                          (0)


/*
 * Watchpoint Comparator Compare Value
 * Provides a reference value for use by comparator
 */
#define CM0_DWT_COMP0_ADDRESS                               (0xe0001020)
#define CM0_DWT_COMP0                                       (*(volatile uint32_t *)(0xe0001020))
#define CM0_DWT_COMP0_DEFAULT                               (0x00000000)

/*
 * Reference value for comparison. See The DWT comparators on page C1-341.
 */
#define CM0_DWT_COMP0_COMP_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_COMP0_COMP_POS                              (0)


/*
 * Watchpoint Comparator Mask
 * Provides the size of the ignore mask applied to the access address range
 * matching
 */
#define CM0_DWT_MASK0_ADDRESS                               (0xe0001024)
#define CM0_DWT_MASK0                                       (*(volatile uint32_t *)(0xe0001024))
#define CM0_DWT_MASK0_DEFAULT                               (0x00000000)

/*
 * The size of the ignore mask applied to address range matching. See The
 * DWT comparators on
 * page C1-341 for the usage model. The mask range is IMPLEMENTATION DEFINED.
 * Writing all ones to this field and reading it back can be used to determine
 * the maximum mask size supported.
 */
#define CM0_DWT_MASK0_MASK_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_MASK0_MASK_POS                              (0)


/*
 * Watchpoint Comparator Function
 * Controls the operation of the comparator
 */
#define CM0_DWT_FUNCTION0_ADDRESS                           (0xe0001028)
#define CM0_DWT_FUNCTION0                                   (*(volatile uint32_t *)(0xe0001028))
#define CM0_DWT_FUNCTION0_DEFAULT                           (0x00000000)

/*
 * Select action on comparator match.
 */
#define CM0_DWT_FUNCTION0_FUNCTION_MASK                     (0x0000000f) /* <0:3> R:RW:0: */
#define CM0_DWT_FUNCTION0_FUNCTION_POS                      (0)


/*
 * Comparator match. It indicates that the operation defined by FUNCTION
 * has occurred since the bit was last read:
 * 0 the associated comparator has matched.
 * 1 the associated comparator has not matched.
 * Reading the register clears this bit to 0.
 */
#define CM0_DWT_FUNCTION0_MATCHED                           (1u << 24) /* <24:24> RW:R:0: */


/*
 * Watchpoint Comparator Compare Value
 * Provides a reference value for use by comparator
 */
#define CM0_DWT_COMP1_ADDRESS                               (0xe0001030)
#define CM0_DWT_COMP1                                       (*(volatile uint32_t *)(0xe0001030))
#define CM0_DWT_COMP1_DEFAULT                               (0x00000000)

/*
 * Reference value for comparison. See The DWT comparators on page C1-341.
 */
#define CM0_DWT_COMP1_COMP_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_COMP1_COMP_POS                              (0)


/*
 * Watchpoint Comparator Mask
 * Provides the size of the ignore mask applied to the access address range
 * matching
 */
#define CM0_DWT_MASK1_ADDRESS                               (0xe0001034)
#define CM0_DWT_MASK1                                       (*(volatile uint32_t *)(0xe0001034))
#define CM0_DWT_MASK1_DEFAULT                               (0x00000000)

/*
 * The size of the ignore mask applied to address range matching. See The
 * DWT comparators on
 * page C1-341 for the usage model. The mask range is IMPLEMENTATION DEFINED.
 * Writing all ones to this field and reading it back can be used to determine
 * the maximum mask size supported.
 */
#define CM0_DWT_MASK1_MASK_MASK                             (0xffffffff) /* <0:31> R:RW:X: */
#define CM0_DWT_MASK1_MASK_POS                              (0)


/*
 * Watchpoint Comparator Function
 * Controls the operation of the comparator
 */
#define CM0_DWT_FUNCTION1_ADDRESS                           (0xe0001038)
#define CM0_DWT_FUNCTION1                                   (*(volatile uint32_t *)(0xe0001038))
#define CM0_DWT_FUNCTION1_DEFAULT                           (0x00000000)

/*
 * Select action on comparator match.
 */
#define CM0_DWT_FUNCTION1_FUNCTION_MASK                     (0x0000000f) /* <0:3> R:RW:0: */
#define CM0_DWT_FUNCTION1_FUNCTION_POS                      (0)


/*
 * Comparator match. It indicates that the operation defined by FUNCTION
 * has occurred since the bit was last read:
 * 0 the associated comparator has matched.
 * 1 the associated comparator has not matched.
 * Reading the register clears this bit to 0.
 */
#define CM0_DWT_FUNCTION1_MATCHED                           (1u << 24) /* <24:24> RW:R:0: */


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #4
 */
#define CM0_DWT_PID4_ADDRESS                                (0xe0001fd0)
#define CM0_DWT_PID4                                        (*(volatile uint32_t *)(0xe0001fd0))
#define CM0_DWT_PID4_DEFAULT                                (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_DWT_PID4_VALUE_MASK                             (0xffffffff) /* <0:31> :R:4: */
#define CM0_DWT_PID4_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #0
 */
#define CM0_DWT_PID0_ADDRESS                                (0xe0001fe0)
#define CM0_DWT_PID0                                        (*(volatile uint32_t *)(0xe0001fe0))
#define CM0_DWT_PID0_DEFAULT                                (0x0000000a)

/*
 * Peripheral ID #0
 */
#define CM0_DWT_PID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:10: */
#define CM0_DWT_PID0_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #1
 */
#define CM0_DWT_PID1_ADDRESS                                (0xe0001fe4)
#define CM0_DWT_PID1                                        (*(volatile uint32_t *)(0xe0001fe4))
#define CM0_DWT_PID1_DEFAULT                                (0x000000b0)

/*
 * Peripheral ID #1
 */
#define CM0_DWT_PID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:176: */
#define CM0_DWT_PID1_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #2
 */
#define CM0_DWT_PID2_ADDRESS                                (0xe0001fe8)
#define CM0_DWT_PID2                                        (*(volatile uint32_t *)(0xe0001fe8))
#define CM0_DWT_PID2_DEFAULT                                (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_DWT_PID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:11: */
#define CM0_DWT_PID2_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Peripheral ID #3
 */
#define CM0_DWT_PID3_ADDRESS                                (0xe0001fec)
#define CM0_DWT_PID3                                        (*(volatile uint32_t *)(0xe0001fec))
#define CM0_DWT_PID3_DEFAULT                                (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_DWT_PID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:0: */
#define CM0_DWT_PID3_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #0
 */
#define CM0_DWT_CID0_ADDRESS                                (0xe0001ff0)
#define CM0_DWT_CID0                                        (*(volatile uint32_t *)(0xe0001ff0))
#define CM0_DWT_CID0_DEFAULT                                (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_DWT_CID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:13: */
#define CM0_DWT_CID0_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #1
 */
#define CM0_DWT_CID1_ADDRESS                                (0xe0001ff4)
#define CM0_DWT_CID1                                        (*(volatile uint32_t *)(0xe0001ff4))
#define CM0_DWT_CID1_DEFAULT                                (0x000000e0)

/*
 * Component ID #1
 */
#define CM0_DWT_CID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:224: */
#define CM0_DWT_CID1_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #2
 */
#define CM0_DWT_CID2_ADDRESS                                (0xe0001ff8)
#define CM0_DWT_CID2                                        (*(volatile uint32_t *)(0xe0001ff8))
#define CM0_DWT_CID2_DEFAULT                                (0x00000005)

/*
 * Component ID #2
 */
#define CM0_DWT_CID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:5: */
#define CM0_DWT_CID2_VALUE_POS                              (0)


/*
 * Watchpoint Unit CoreSight ROM Table Component ID #3
 */
#define CM0_DWT_CID3_ADDRESS                                (0xe0001ffc)
#define CM0_DWT_CID3                                        (*(volatile uint32_t *)(0xe0001ffc))
#define CM0_DWT_CID3_DEFAULT                                (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_DWT_CID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:177: */
#define CM0_DWT_CID3_VALUE_POS                              (0)


/*
 * Breakpoint Unit Control
 * Provides BPU implementation information, and the global enable for the
 * BPU.
 */
#define CM0_BP_CTRL_ADDRESS                                 (0xe0002000)
#define CM0_BP_CTRL                                         (*(volatile uint32_t *)(0xe0002000))
#define CM0_BP_CTRL_DEFAULT                                 (0x00000040)

/*
 * Enables the BPU:
 * 0 BPU is disabled.
 * 1 BPU is enabled.
 */
#define CM0_BP_CTRL_ENABLE                                  (1u << 0) /* <0:0> R:RW:0: */


/*
 * RAZ on reads, SBO for writes. If written as zero, the write to the register
 * is ignored.
 */
#define CM0_BP_CTRL_KEY                                     (1u << 1) /* <1:1> R:RW:0: */


/*
 * The number of breakpoint comparators. If NUM_CODE is zero, the implementation
 * does not support any comparators.
 */
#define CM0_BP_CTRL_NUM_CODE_MASK                           (0x000000f0) /* <4:7> :R:4: */
#define CM0_BP_CTRL_NUM_CODE_POS                            (4)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP0_ADDRESS                                (0xe0002008)
#define CM0_BP_COMP0                                        (*(volatile uint32_t *)(0xe0002008))
#define CM0_BP_COMP0_DEFAULT                                (0x00000000)

/*
 * Enables the comparator:
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP0_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP0_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP0_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP0_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP0_MATCH_POS                              (30)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP1_ADDRESS                                (0xe000200c)
#define CM0_BP_COMP1                                        (*(volatile uint32_t *)(0xe000200c))
#define CM0_BP_COMP1_DEFAULT                                (0x00000000)

/*
 * Enables the comparator:
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP1_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP1_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP1_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP1_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP1_MATCH_POS                              (30)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP2_ADDRESS                                (0xe0002010)
#define CM0_BP_COMP2                                        (*(volatile uint32_t *)(0xe0002010))
#define CM0_BP_COMP2_DEFAULT                                (0x00000000)

/*
 * Enables the comparator:
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP2_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP2_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP2_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP2_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP2_MATCH_POS                              (30)


/*
 * Breakpoint Compare Register
 * Holds a breakpoint address for comparison with instruction addresses in
 * the Code
 * memory region, see The system address map on page B3-258 for more information.
 */
#define CM0_BP_COMP3_ADDRESS                                (0xe0002014)
#define CM0_BP_COMP3                                        (*(volatile uint32_t *)(0xe0002014))
#define CM0_BP_COMP3_DEFAULT                                (0x00000000)

/*
 * Enables the comparator.
 * Note BP_CTRL.ENABLE must also be set to 1 to enable a comparator.
 */
#define CM0_BP_COMP3_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Stores bits [28:2] of the comparison address. The comparison address is
 * compared with the address from the Code memory region. Bits [31:29] and
 * [1:0] of the comparison address are zero.
 */
#define CM0_BP_COMP3_COMP_ADDR_MASK                         (0x1ffffffc) /* <2:28> R:RW:X: */
#define CM0_BP_COMP3_COMP_ADDR_POS                          (2)


/*
 * BP_MATCH defines the behavior when the COMP address is matched.
 */
#define CM0_BP_COMP3_MATCH_MASK                             (0xc0000000) /* <30:31> R:RW:X: */
#define CM0_BP_COMP3_MATCH_POS                              (30)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #4
 */
#define CM0_BP_PID4_ADDRESS                                 (0xe0002fd0)
#define CM0_BP_PID4                                         (*(volatile uint32_t *)(0xe0002fd0))
#define CM0_BP_PID4_DEFAULT                                 (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_BP_PID4_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4: */
#define CM0_BP_PID4_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #0
 */
#define CM0_BP_PID0_ADDRESS                                 (0xe0002fe0)
#define CM0_BP_PID0                                         (*(volatile uint32_t *)(0xe0002fe0))
#define CM0_BP_PID0_DEFAULT                                 (0x0000000b)

/*
 * Peripheral ID #0
 */
#define CM0_BP_PID0_VALUE_MASK                              (0xffffffff) /* <0:31> :R:11: */
#define CM0_BP_PID0_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #1
 */
#define CM0_BP_PID1_ADDRESS                                 (0xe0002fe4)
#define CM0_BP_PID1                                         (*(volatile uint32_t *)(0xe0002fe4))
#define CM0_BP_PID1_DEFAULT                                 (0x000000b0)

/*
 * Peripheral ID #1
 */
#define CM0_BP_PID1_VALUE_MASK                              (0xffffffff) /* <0:31> :R:176: */
#define CM0_BP_PID1_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #2
 */
#define CM0_BP_PID2_ADDRESS                                 (0xe0002fe8)
#define CM0_BP_PID2                                         (*(volatile uint32_t *)(0xe0002fe8))
#define CM0_BP_PID2_DEFAULT                                 (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_BP_PID2_VALUE_MASK                              (0xffffffff) /* <0:31> :R:11: */
#define CM0_BP_PID2_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Peripheral ID #3
 */
#define CM0_BP_PID3_ADDRESS                                 (0xe0002fec)
#define CM0_BP_PID3                                         (*(volatile uint32_t *)(0xe0002fec))
#define CM0_BP_PID3_DEFAULT                                 (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_BP_PID3_VALUE_MASK                              (0xffffffff) /* <0:31> :R:0: */
#define CM0_BP_PID3_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #0
 */
#define CM0_BP_CID0_ADDRESS                                 (0xe0002ff0)
#define CM0_BP_CID0                                         (*(volatile uint32_t *)(0xe0002ff0))
#define CM0_BP_CID0_DEFAULT                                 (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_BP_CID0_VALUE_MASK                              (0xffffffff) /* <0:31> :R:13: */
#define CM0_BP_CID0_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #1
 */
#define CM0_BP_CID1_ADDRESS                                 (0xe0002ff4)
#define CM0_BP_CID1                                         (*(volatile uint32_t *)(0xe0002ff4))
#define CM0_BP_CID1_DEFAULT                                 (0x000000e0)

/*
 * Component ID #1
 */
#define CM0_BP_CID1_VALUE_MASK                              (0xffffffff) /* <0:31> :R:224: */
#define CM0_BP_CID1_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #2
 */
#define CM0_BP_CID2_ADDRESS                                 (0xe0002ff8)
#define CM0_BP_CID2                                         (*(volatile uint32_t *)(0xe0002ff8))
#define CM0_BP_CID2_DEFAULT                                 (0x00000005)

/*
 * Component ID #2
 */
#define CM0_BP_CID2_VALUE_MASK                              (0xffffffff) /* <0:31> :R:5: */
#define CM0_BP_CID2_VALUE_POS                               (0)


/*
 * Breakpoint Unit CoreSight ROM Table Component ID #3
 */
#define CM0_BP_CID3_ADDRESS                                 (0xe0002ffc)
#define CM0_BP_CID3                                         (*(volatile uint32_t *)(0xe0002ffc))
#define CM0_BP_CID3_DEFAULT                                 (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_BP_CID3_VALUE_MASK                              (0xffffffff) /* <0:31> :R:177: */
#define CM0_BP_CID3_VALUE_POS                               (0)


/*
 * Auxiliary Control Register
 * Provides configuration and control options.
 */
#define CM0_ACTLR_ADDRESS                                   (0xe000e008)
#define CM0_ACTLR                                           (*(volatile uint32_t *)(0xe000e008))
#define CM0_ACTLR_DEFAULT                                   (0x00000000)

/*
 * Always 0 for CM0r0p0
 */
#define CM0_ACTLR_ACTRL_MASK                                (0xffffffff) /* <0:31> :R:0: */
#define CM0_ACTLR_ACTRL_POS                                 (0)


/*
 * SysTick Control & Status
 * Controls the SysTick counter and provides status data. The SysTick counter
 * is functional in the Active and Sleep power modes (and not functional
 * in the DeepSleep, Hibernate and Stop power modes).
 */
#define CM0_SYST_CSR_ADDRESS                                (0xe000e010)
#define CM0_SYST_CSR                                        (*(volatile uint32_t *)(0xe000e010))
#define CM0_SYST_CSR_DEFAULT                                (0x00000000)

/*
 * Indicates the enabled status of the SysTick counter:
 * '0': counter is disabled.
 * '1': counter is operating.
 */
#define CM0_SYST_CSR_ENABLE                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Indicates whether counting to "0" causes the status of the SysTick exception
 * to change to pending:
 * '0': count to "0" does not affect the SysTick exception status.
 * '1': count to "0" changes the SysTick exception status to pending.
 *
 * Changing the value of the counter to "0" by writing zero to the SYST_CVR
 * register to "0" never changes the status of the SysTick exception.
 */
#define CM0_SYST_CSR_TICKINT                                (1u << 1) /* <1:1> R:RW:0: */


/*
 * Indicates the SysTick counter clock source:
 * '0': SysTick uses the low frequency clock "clk_lf". For this mode to function,
 * "clk_lf" should be less than half the frequency of "clk_sys". Note that
 * "clk_lf" is generated by a low accuracy ILO (Internal Low power Oscillator),
 * with a target frequency of 32.768 kHz (frequency can be as low as 15 KHz
 * and as high as 60 kHz).
 * '1': SysTick uses the system/processor clock "clk_sys".
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality on the
 * low frequency clock is provided. in SF, TSG6M products, this functionality
 * is not provided. For these products, this field should be set to '1',
 * such that SysTick uses the system clock "clk_sys".
 */
#define CM0_SYST_CSR_CLKSOURCE                              (1u << 2) /* <2:2> R:RW:0: */


/*
 * Indicates whether the counter has counted to "0" since the last read of
 * this register:
 * '0': counter has not counted to "0".
 * '1': counter has counted to "0".
 *
 * COUNTFLAG is set to '1' by a count transition from "1" to "0".
 * COUNTFLAG is cleared to '0' by a read of this register, and by any write
 * to the SYST_CVR register.
 */
#define CM0_SYST_CSR_COUNTFLAG                              (1u << 16) /* <16:16> RW:R:0: */


/*
 * SysTick Reload Value
 * Sets or reads the reload value of the SYST_CVR register.
 */
#define CM0_SYST_RVR_ADDRESS                                (0xe000e014)
#define CM0_SYST_RVR                                        (*(volatile uint32_t *)(0xe000e014))
#define CM0_SYST_RVR_DEFAULT                                (0x00000000)

/*
 * The value to load into the SYST_CVR register when the counter reaches
 * 0.
 */
#define CM0_SYST_RVR_RELOAD_MASK                            (0x00ffffff) /* <0:23> R:RW:X: */
#define CM0_SYST_RVR_RELOAD_POS                             (0)


/*
 * SysTick Current Value
 * Reads or clears the current counter value.  Any write to the register
 * clears the register to 0.
 */
#define CM0_SYST_CVR_ADDRESS                                (0xe000e018)
#define CM0_SYST_CVR                                        (*(volatile uint32_t *)(0xe000e018))
#define CM0_SYST_CVR_DEFAULT                                (0x00000000)

/*
 * Current counter value.
 * This is the value of the counter at the time it is sampled.
 */
#define CM0_SYST_CVR_CURRENT_MASK                           (0x00ffffff) /* <0:23> R:RW:X: */
#define CM0_SYST_CVR_CURRENT_POS                            (0)


/*
 * SysTick Calibration Value
 * Reads the calibration value and parameters for SysTick.
 */
#define CM0_SYST_CALIB_ADDRESS                              (0xe000e01c)
#define CM0_SYST_CALIB                                      (*(volatile uint32_t *)(0xe000e01c))
#define CM0_SYST_CALIB_DEFAULT                              (0x00000000)

/*
 * Optionally, holds a reload value to be used for 10ms (100Hz) timing, subject
 * to system clock skew errors. If this field is "0", the calibration value
 * is not known.
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality on the
 * low frequency clock is provided and this field is 0x00:00147. In earlier
 * products, SysTick counter functionality on the low frequency clock is
 * NOT provided and this field is 0x00:0000.
 */
#define CM0_SYST_CALIB_TENMS_MASK                           (0x00ffffff) /* <0:23> RW:R:X: */
#define CM0_SYST_CALIB_TENMS_POS                            (0)


/*
 * Indicates whether the 10ms calibration value is exact:
 * '0': 10ms calibration value is exact.
 * '1': 10ms calibration value is inexact, because of the clock frequency.
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality on the
 * low frequency clock is provided and this field is '1' (due to the low
 * accuracy ILO). In earlier products, SysTick counter functionality on the
 * low frequency clock is NOT provided and this field is '0'.
 */
#define CM0_SYST_CALIB_SKEW                                 (1u << 30) /* <30:30> RW:R:X: */


/*
 * Indicates whether a implementation defined reference clock is provided:
 * '0': the reference clock is provided.
 * '1': the reference clock is not provided.
 * When this bit is '1', the SYST_CSR.CLKSOURCEis forced to '1' and cannot
 * be cleared to '0'.
 *
 * In PSoC4A-BLE (and later products), SysTick counter functionality  on
 * the low frequency clock is provided and this field is '0'. In earlier
 * products, SysTick counter functionality on the low frequency clock is
 * NOT provided and this field is '1'.
 */
#define CM0_SYST_CALIB_NOREF                                (1u << 31) /* <31:31> :R:0: */


/*
 * Interrupt Set-Enable Register
 * Enables, or reads the enabled state of one or more interrupts.
 */
#define CM0_ISER_ADDRESS                                    (0xe000e100)
#define CM0_ISER                                            (*(volatile uint32_t *)(0xe000e100))
#define CM0_ISER_DEFAULT                                    (0x00000000)

/*
 * Enables, or reads the enabled state of one or more interrupts. Each bit
 * corresponds to the same numbered interrupt.
 */
#define CM0_ISER_SETENA_MASK                                (0xffffffff) /* <0:31> R:RW1S:0: */
#define CM0_ISER_SETENA_POS                                 (0)


/*
 * Interrupt Clear Enable Register
 * Disables, or reads the enabled state of one or more interrupts
 */
#define CM0_ICER_ADDRESS                                    (0xe000e180)
#define CM0_ICER                                            (*(volatile uint32_t *)(0xe000e180))
#define CM0_ICER_DEFAULT                                    (0x00000000)

/*
 * Disables, or reads the enabled state of one or more interrupts. Each bit
 * corresponds to the same numbered interrupt.
 */
#define CM0_ICER_CLRENA_MASK                                (0xffffffff) /* <0:31> R:RW1C:0: */
#define CM0_ICER_CLRENA_POS                                 (0)


/*
 * Interrupt Set-Pending Register
 * On writes, sets the status of one or more interrupts to pending. On reads,
 * shows the
 * pending status of the interrupts.
 */
#define CM0_ISPR_ADDRESS                                    (0xe000e200)
#define CM0_ISPR                                            (*(volatile uint32_t *)(0xe000e200))
#define CM0_ISPR_DEFAULT                                    (0x00000000)

/*
 * Changes the state of one or more interrupts to pending. Each bit corresponds
 * to the same numbered interrupt.
 */
#define CM0_ISPR_SETPEND_MASK                               (0xffffffff) /* <0:31> R:RW1S:0: */
#define CM0_ISPR_SETPEND_POS                                (0)


/*
 * Interrupt Clear-Pending Register
 * On writes, clears the status of one or more interrupts to pending. On
 * reads, shows
 * the pending status of the interrupts.
 */
#define CM0_ICPR_ADDRESS                                    (0xe000e280)
#define CM0_ICPR                                            (*(volatile uint32_t *)(0xe000e280))
#define CM0_ICPR_DEFAULT                                    (0x00000000)

/*
 * Changes the state of one or more interrupts to not pending. Each bit corresponds
 * to the same numbered interrupt.
 */
#define CM0_ICPR_CLRPEND_MASK                               (0xffffffff) /* <0:31> R:RW1C:0: */
#define CM0_ICPR_CLRPEND_POS                                (0)


/*
 * Interrupt Priority Registers
 * Sets or reads interrupt priorities. Register n contains priorities for
 * interrupts N=4n .. 4n+3
 */
#define CM0_IPR_ADDRESS(n)                                  (0xe000e400 + ((n) * (0x0004)))
#define CM0_IPR(n)                                          (*(volatile uint32_t *)(0xe000e400 + ((n) * 0x0004)))
#define CM0_IPR_DEFAULT                                     (0x00000000)

/*
 * Priority of interrupt number N.
 */
#define CM0_IPR_PRI_N0_MASK                                 (0x000000c0) /* <6:7> R:RW:0: */
#define CM0_IPR_PRI_N0_POS                                  (6)


/*
 * Priority of interrupt number N+1.
 */
#define CM0_IPR_PRI_N1_MASK                                 (0x0000c000) /* <14:15> R:RW:0: */
#define CM0_IPR_PRI_N1_POS                                  (14)


/*
 * Priority of interrupt number N+2.
 */
#define CM0_IPR_PRI_N2_MASK                                 (0x00c00000) /* <22:23> R:RW:0: */
#define CM0_IPR_PRI_N2_POS                                  (22)


/*
 * Priority of interrupt number N+3.
 */
#define CM0_IPR_PRI_N3_MASK                                 (0xc0000000) /* <30:31> R:RW:0: */
#define CM0_IPR_PRI_N3_POS                                  (30)


/*
 * CPUID Register
 * Contains the part number, version, and implementation information that
 * is specific to this processor.
 */
#define CM0_CPUID_ADDRESS                                   (0xe000ed00)
#define CM0_CPUID                                           (*(volatile uint32_t *)(0xe000ed00))
#define CM0_CPUID_DEFAULT                                   (0x410cc200)

/*
 * Indicates revision. In ARM implementations this is the minor revision
 * number n in the pn part of the rnpn revision status, see Product revision
 * status on page xii. For release r0p0.
 */
#define CM0_CPUID_REVISION_MASK                             (0x0000000f) /* <0:3> :R:0: */
#define CM0_CPUID_REVISION_POS                              (0)


/*
 * Indicates part number, Cortex-M0
 */
#define CM0_CPUID_PARTNO_MASK                               (0x0000fff0) /* <4:15> :R:3104: */
#define CM0_CPUID_PARTNO_POS                                (4)


/*
 * Indicates the architecture, ARMv6-M
 */
#define CM0_CPUID_CONSTANT_MASK                             (0x000f0000) /* <16:19> :R:12: */
#define CM0_CPUID_CONSTANT_POS                              (16)


/*
 * Implementation defined. In ARM implementations this is the major revision
 * number n in the rn part of the rnpn revision status, Product revision
 * status on page xii.
 */
#define CM0_CPUID_VARIANT_MASK                              (0x00f00000) /* <20:23> :R:0: */
#define CM0_CPUID_VARIANT_POS                               (20)


/*
 * Implementer code for ARM.
 */
#define CM0_CPUID_IMPLEMENTER_MASK                          (0xff000000) /* <24:31> :R:65: */
#define CM0_CPUID_IMPLEMENTER_POS                           (24)


/*
 * Interrupt Control State Register
 * Controls and provides status information for the ARMv6-M.
 */
#define CM0_ICSR_ADDRESS                                    (0xe000ed04)
#define CM0_ICSR                                            (*(volatile uint32_t *)(0xe000ed04))
#define CM0_ICSR_DEFAULT                                    (0x00000000)

/*
 * The exception number for the current executing exception.  0= Thread mode.
 * This is the same value as IPSR[8:0]
 */
#define CM0_ICSR_VECTACTIVE_MASK                            (0x000001ff) /* <0:8> RW:R:0: */
#define CM0_ICSR_VECTACTIVE_POS                             (0)


/*
 * The exception number for the highest priority pending exception. 0= No
 * pending exceptions.
 * The pending state includes the effect of memory-mapped enable and mask
 * registers. It does not include the PRIMASK special-purpose register qualifier.
 */
#define CM0_ICSR_VECTPENDING_MASK                           (0x001ff000) /* <12:20> RW:R:0: */
#define CM0_ICSR_VECTPENDING_POS                            (12)


/*
 * Indicates if an external configurable, NVIC generated, interrupt is pending.
 */
#define CM0_ICSR_ISRPENDING                                 (1u << 22) /* <22:22> RW:R:0: */


/*
 * Indicates whether a pending exception will be serviced on exit from debug
 * halt state.
 */
#define CM0_ICSR_ISRPREEMPT                                 (1u << 23) /* <23:23> RW:R:0: */


/*
 * Clears a pending SysTick, whether set here or by the timer hardware.
 */
#define CM0_ICSR_PENDSTCLR                                  (1u << 25) /* <25:25> R:RW1C:0: */


/*
 * Sets a pending SysTick or reads back the current state. Writing PENDSTSET
 * and PENDSTCLR to '1' concurrently is UNPREDICTABLE.
 */
#define CM0_ICSR_PENDSTSETB                                 (1u << 26) /* <26:26> RW:RW1S:0: */


/*
 * Clears a pending PendSV interrupt.
 */
#define CM0_ICSR_PENDSVCLR                                  (1u << 27) /* <27:27> R:RW1C:0: */


/*
 * Sets a pending PendSV interrupt or reads back the current state.  Use
 * this normally to request a context switch.  Writing PENDSVSET and PENDSVCLR
 * to '1' concurrently is UNPREDICTABLE.
 */
#define CM0_ICSR_PENDSVSET                                  (1u << 28) /* <28:28> RW:RW1S:0: */


/*
 * Activates an NMI exception or reads back the current state.
 * Because NMI is the highest priority exception, it activates as soon as
 * it is registered.
 */
#define CM0_ICSR_NMIPENDSET                                 (1u << 31) /* <31:31> RW:RW1S:0: */


/*
 * Application Interrupt and Reset Control Register
 * Sets or returns interrupt control data.
 */
#define CM0_AIRCR_ADDRESS                                   (0xe000ed0c)
#define CM0_AIRCR                                           (*(volatile uint32_t *)(0xe000ed0c))
#define CM0_AIRCR_DEFAULT                                   (0x00000000)

/*
 * Clears all active state information for fixed and configurable
 * exceptions.  The effect of writing a 1 to this bit if the processor is
 * not halted in Debug state is UNPREDICTABLE.
 */
#define CM0_AIRCR_VECTCLRACTIVE                             (1u << 1) /* <1:1> R:RW1C:0: */


/*
 * System Reset Request.  Writing 1 to this bit asserts a signal to request
 * a reset by the external system. This will cause a full system reset of
 * the CPU and all other components in the device.  See Reset management
 * on page B1-240 for more information.
 */
#define CM0_AIRCR_SYSRESETREQ                               (1u << 2) /* <2:2> R:RW1S:0: */


/*
 * Indicates the memory system data endianness:
 * 0 little endian
 * 1 big endian.
 * See Endian support on page A3-44 for more information.
 */
#define CM0_AIRCR_ENDIANNESS                                (1u << 15) /* <15:15> :R:0: */


/*
 * Vector Key. The value 0x05FA must be written to this register, otherwise
 * the register write is UNPREDICTABLE.  Readback value is UNKNOWN.
 */
#define CM0_AIRCR_VECTKEY_MASK                              (0xffff0000) /* <16:31> R:RW:X: */
#define CM0_AIRCR_VECTKEY_POS                               (16)


/*
 * System Control Register
 * Sets or returns system control data.
 */
#define CM0_SCR_ADDRESS                                     (0xe000ed10)
#define CM0_SCR                                             (*(volatile uint32_t *)(0xe000ed10))
#define CM0_SCR_DEFAULT                                     (0x00000000)

/*
 * Determines whether, on an exit from an ISR that returns to the base level
 * of execution priority, the processor enters a sleep state:
 * 0 do not enter sleep state.
 * 1 enter sleep state.
 * See Power management on page B1-240 for more information.
 */
#define CM0_SCR_SLEEPONEXIT                                 (1u << 1) /* <1:1> R:RW:0: */


/*
 * An implementation can use this bit to select DeepSleep/Hibernate power
 * modes upon execution of WFI/WFE:
 * 0: Select Sleep mode
 * 1: Select DeepSleep/Hibernate (depends on PWR_CONTROL.HIBERNATE)
 */
#define CM0_SCR_SLEEPDEEP                                   (1u << 2) /* <2:2> R:RW:0: */


/*
 * Determines whether an interrupt transition from inactive state to pending
 * state is a wakeup event:
 * 0: transitions from inactive to pending are not wakeup events.
 * 1: transitions from inactive to pending are wakeup events.
 * See WFE on page A6-197 for more information.
 */
#define CM0_SCR_SEVONPEND                                   (1u << 4) /* <4:4> R:RW:0: */


/*
 * Configuration and Control Register
 * Returns configuration and control data.
 */
#define CM0_CCR_ADDRESS                                     (0xe000ed14)
#define CM0_CCR                                             (*(volatile uint32_t *)(0xe000ed14))
#define CM0_CCR_DEFAULT                                     (0x00000208)

/*
 * 1: unaligned word and halfword accesses generate a HardFault exception.
 */
#define CM0_CCR_UNALIGN_TRP                                 (1u << 3) /* <3:3> :R:1: */


/*
 * 1: On exception entry, the SP used prior to the exception is adjusted
 * to be 8-byte aligned and the context to restore it is saved. The SP is
 * restored on the associated exception return.
 */
#define CM0_CCR_STKALIGN                                    (1u << 9) /* <9:9> :R:1: */


/*
 * System Handler Priority Register 2
 * Sets or returns priority for system handler 11
 */
#define CM0_SHPR2_ADDRESS                                   (0xe000ed1c)
#define CM0_SHPR2                                           (*(volatile uint32_t *)(0xe000ed1c))
#define CM0_SHPR2_DEFAULT                                   (0x00000000)

/*
 * Priority of system handler 11, SVCall
 */
#define CM0_SHPR2_PRI_11_MASK                               (0xc0000000) /* <30:31> R:RW:0: */
#define CM0_SHPR2_PRI_11_POS                                (30)


/*
 * System Handler Priority Register 3
 * Sets or returns priority for system handlers 14-15.
 */
#define CM0_SHPR3_ADDRESS                                   (0xe000ed20)
#define CM0_SHPR3                                           (*(volatile uint32_t *)(0xe000ed20))
#define CM0_SHPR3_DEFAULT                                   (0x00000000)

/*
 * Priority of system handler 14, PendSV
 */
#define CM0_SHPR3_PRI_14_MASK                               (0x00c00000) /* <22:23> R:RW:0: */
#define CM0_SHPR3_PRI_14_POS                                (22)


/*
 * Priority of system handler 15, SysTick
 */
#define CM0_SHPR3_PRI_15_MASK                               (0xc0000000) /* <30:31> R:RW:0: */
#define CM0_SHPR3_PRI_15_POS                                (30)


/*
 * System Handler Control and State Register
 * Controls and provides the status of system handlers.
 */
#define CM0_SHCSR_ADDRESS                                   (0xe000ed24)
#define CM0_SHCSR                                           (*(volatile uint32_t *)(0xe000ed24))
#define CM0_SHCSR_DEFAULT                                   (0x00000000)

/*
 * 0 SVCall is not pending.
 * 1 SVCall is pending.
 * This bit reflects the pending state on a read, and updates the pending
 * state, to the value written, on a write. (Pending state bits are set to
 * 1 when an exception occurs, and are cleared to 0 when an exception becomes
 * active.)
 */
#define CM0_SHCSR_SVCALLPENDED                              (1u << 15) /* <15:15> RW:RW:0: */


/*
 * Debug Fault Status Register
 * Provides the top level reason why a debug event has occurred.  Writing
 * 1 to a register bit clears that bit to 0. A read of the HALTED bit by
 * an instruction executed by stepping returns an UNKNOWN value, For more
 * information see Debug stepping on page C1-325.
 */
#define CM0_DFSR_ADDRESS                                    (0xe000ed30)
#define CM0_DFSR                                            (*(volatile uint32_t *)(0xe000ed30))
#define CM0_DFSR_DEFAULT                                    (0x00000000)

/*
 * Indicates a debug event generated by a C_HALT or C_STEP request, triggered
 * by a write to the DHCSR:
 * 0 no active halt request debug event.
 * 1 halt request debug event active.
 * See Debug Halting Control and Status Register, DHCSR for more information.
 */
#define CM0_DFSR_HALTED                                     (1u << 0) /* <0:0> RW1S:RW1C:0: */


/*
 * Indicates a debug event generated by BKPT instruction execution or a breakpoint
 * match in the BPU:
 * 0 no breakpoint debug event.
 * 1 at least one breakpoint debug event.
 */
#define CM0_DFSR_BKPT                                       (1u << 1) /* <1:1> RW1S:RW1C:0: */


/*
 * Indicates a debug event generated by the DWT:
 * 0 no debug events generated by the DWT.
 * 1 at least one debug event generated by the DWT.
 */
#define CM0_DFSR_DWTTRAP                                    (1u << 2) /* <2:2> RW1S:RW1C:0: */


/*
 * Indicates whether a vector catch debug event was generated:
 * 0 no vector catch debug event generated.
 * 1 vector catch debug event generated.
 * The corresponding FSR shows the primary cause of the exception.
 */
#define CM0_DFSR_VCATCH                                     (1u << 3) /* <3:3> RW1S:RW1C:0: */


/*
 * Indicates an asynchronous debug event generated because of EDBGRQ being
 * asserted:
 * 0 no EDBGRQ debug event.
 * 1 EDBGRQ debug event.
 */
#define CM0_DFSR_EXTERNAL                                   (1u << 4) /* <4:4> RW1S:RW1C:0: */


/*
 * Debug Halting Control and Status Register
 * Controls halting debug.  When C_DEBUGEN is set to 1, C_STEP and C_MASKINTS
 * must not be
 * modified when the processor is running.  Note: S_HALT is 0 when the processor
 * is running. When C_DEBUGEN is set to 0, the processor ignores the values
 * of all other bits in this register. For more information on the use of
 * DHCSR, see Debug stepping on page C1-325.
 * Note: any write to this register must have [31:16]=0xA05F - if not, the
 * write is ignored.
 */
#define CM0_DHCSR_ADDRESS                                   (0xe000edf0)
#define CM0_DHCSR                                           (*(volatile uint32_t *)(0xe000edf0))
#define CM0_DHCSR_DEFAULT                                   (0x02000000)

/*
 * Halting debug enable bit.  If a debugger writes to DHCSR to change the
 * value of this bit from 0 to 1, it must also write 0 to the C_MASKINTS
 * bit, otherwise behavior is UNPREDICTABLE.  This bit can only be written
 * from the DAP. Access to the DHCSR from
 * software running on the processor is IMPLEMENTATION DEFINED.  However,
 * writes to this bit from software running on the processor are ignored.
 */
#define CM0_DHCSR_C_DEBUGEN                                 (1u << 0) /* <0:0> R:RW:0: */


/*
 * Processor halt bit. The effects of writes to this bit are:
 * 0 Request a halted processor to run.
 * 1 Request a running processor to halt.
 * Table C1-7 on page C1-326 shows the effect of writes to this bit when
 * the processor is in Debug state.
 */
#define CM0_DHCSR_C_HALT                                    (1u << 1) /* <1:1> R:RW:X: */


/*
 * Processor step bit. The effects of writes to this bit are:
 * 0 Single-stepping disabled.
 * 1 Single-stepping enabled.
 * For more information about the use of this bit see Table C1-7 on page
 * C1-326.
 */
#define CM0_DHCSR_C_STEP                                    (1u << 2) /* <2:2> R:RW:X: */


/*
 * When debug is enabled, the debugger can write to this bit to mask PendSV,
 * SysTick and external configurable interrupts. The effect of any attempt
 * to change the value of this bit is UNPREDICTABLE unless both:
 * - before the write to DHCSR, the value of the C_HALT bit is 1
 * - the write to the DHCSR that changes the C_MASKINTS bit also writes 1
 * to the C_HALT bit
 * This means that a single write to DHCSR cannot set the C_HALT to 0 and
 * change the value of the C_MASKINTS bit. The bit does not affect NMI. When
 * DHCSR.C_DEBUGEN is set to 0, the
 * value of this bit is UNKNOWN. For more information about the use of this
 * bit see Table C1-7 on page C1-326.
 */
#define CM0_DHCSR_C_MASKINTS                                (1u << 3) /* <3:3> R:RW:X: */


/*
 * A handshake flag for transfers through the DCRDR:
 * - Writing to DCRSR clears the bit to 0.
 * - Completion of the DCRDR transfer then sets the bit to 1.
 * For more information about DCRDR transfers see Debug Core Register Data
 * Register, DCRDR on page C1-337.
 * 0: There has been a write to the DCRDR, but the transfer is
 * not complete.
 * 1: The transfer to or from the DCRDR is complete.
 * This bit is only valid when the processor is in Debug state, otherwise
 * the bit is UNKNOWN.
 */
#define CM0_DHCSR_S_REGRDY                                  (1u << 16) /* <16:16> RW:R:X: */


/*
 * Indicates whether the processor is in Debug state.
 */
#define CM0_DHCSR_S_HALT                                    (1u << 17) /* <17:17> RW:R:0: */


/*
 * Indicates whether the processor is sleeping.  The debugger must set the
 * DHCSR.C_HALT bit to 1 to gain control, or
 * wait for an interrupt or other wakeup event to wakeup the system.
 */
#define CM0_DHCSR_S_SLEEP                                   (1u << 18) /* <18:18> RW:R:0: */


/*
 * Indicates whether the processor is locked up because of an unrecoverable
 * exception.  See Unrecoverable exception cases on page B1-238 for more
 * information.  This bit can only read as 1 when accessed by a remote debugger
 * using the
 * DAP.  The bit clears to 0 when the processor enters Debug state.
 */
#define CM0_DHCSR_S_LOCKUP                                  (1u << 19) /* <19:19> RW:R:0: */


/*
 * Debug key:
 * Software must write 0xA05F to [31:16] to enable write accesses to bits
 * [15:0], otherwise the processor ignores the write access.
 */
#define CM0_DHCSR_DBG_KEY_P1_MASK                           (0x00f00000) /* <20:23> R:W:X: */
#define CM0_DHCSR_DBG_KEY_P1_POS                            (20)


/*
 * When not in Debug state, indicates whether the processor has completed
 * execution of at least one instruction since the last read of DHCSR. This
 * is a sticky bit, that clears to 0 on a read of DHCSR.
 * This bit is UNKNOWN:
 * - after a Local reset, but is set to 1 as soon as the processor completes
 * execution of an instruction
 * - when S_LOCKUP is set to 1
 * - when S_HALT is set to 1.
 * When the processor is not in Debug state, a debugger can check this bit
 * to determine if the processor is stalled on a load, store or fetch access.
 */
#define CM0_DHCSR_S_RETIRE_ST                               (1u << 24) /* <24:24> RW:R:X: */


/*
 * Indicates whether the processor has been reset since the last read of
 * DHCSR. This is a sticky bit, that clears to 0 on a read of DHCSR
 */
#define CM0_DHCSR_S_RESET_ST                                (1u << 25) /* <25:25> RW:R:1: */


/*
 * See desciption for Bit 20
 */
#define CM0_DHCSR_DBG_KEY_P2_MASK                           (0xfc000000) /* <26:31> R:W:X: */
#define CM0_DHCSR_DBG_KEY_P2_POS                            (26)


/*
 * Debug Core Register Selector Register
 * With the DCRDR, see Debug Core Register Data Register, DCRDR on page C1-337,
 * the DCRSR provides debug access to the ARM core registers and special-purpose
 * registers. A write to DCRSR specifies the register to transfer, whether
 * the transfer is a read or a write, and starts the transfer.  This register
 * is only accessible in Debug state.  When the processor is in Debug state,
 * the debugger must preserve the Exception number bits in the IPSR, otherwise
 * behavior is UNPREDICTABLE
 */
#define CM0_DCRSR_ADDRESS                                   (0xe000edf4)
#define CM0_DCRSR                                           (*(volatile uint32_t *)(0xe000edf4))
#define CM0_DCRSR_DEFAULT                                   (0x00000000)

/*
 * Specifies the ARM core register or special-purpose register to transfer.
 */
#define CM0_DCRSR_REGSEL_MASK                               (0x0000001f) /* <0:4> R:RW:X: */
#define CM0_DCRSR_REGSEL_POS                                (0)


/*
 * Specifies the type of access for the transfer.
 */
#define CM0_DCRSR_REGWNR                                    (1u << 16) /* <16:16> R:RW:X: */


/*
 * Debug Core Register Data Register
 * With the DCRSR, see Debug Core Register Selector Register, DCRSR on page
 * C1-335, the DCRDR provides debug access to the ARM core registers and
 * special-purpose registers. The DCRDR is the data register for these accesses.
 */
#define CM0_DCRDR_ADDRESS                                   (0xe000edf8)
#define CM0_DCRDR                                           (*(volatile uint32_t *)(0xe000edf8))
#define CM0_DCRDR_DEFAULT                                   (0x00000000)

/*
 * Data temporary cache, for reading and writing CPU registers.
 * This register is UNKNOWN:
 * - on reset
 * - when DHCSR.S_HALT = 0.
 * - when DHCSR.S_REGRDY = 0 during execution of a DCRSR based transaction
 * that updates the register
 */
#define CM0_DCRDR_DBGTMP_MASK                               (0xffffffff) /* <0:31> RW:RW:X: */
#define CM0_DCRDR_DBGTMP_POS                                (0)


/*
 * Debug Exception and Monitor Control Register
 * Manages vector catch behavior and enables the DWT.
 */
#define CM0_DEMCR_ADDRESS                                   (0xe000edfc)
#define CM0_DEMCR                                           (*(volatile uint32_t *)(0xe000edfc))
#define CM0_DEMCR_DEFAULT                                   (0x00000000)

/*
 * Enable Reset Vector Catch. This causes a Local reset to halt a running
 * system. If DHCSR.C_DEBUGEN is set to 0, the processor ignores the value
 * of this bit.
 */
#define CM0_DEMCR_VC_CORERESET                              (1u << 0) /* <0:0> R:RW:0: */


/*
 * Enable halting debug trap on a HardFault exception. If DHCSR.C_DEBUGEN
 * is set to 0, the processor ignores the value of this bit.
 */
#define CM0_DEMCR_VC_HARDERR                                (1u << 10) /* <10:10> R:RW:0: */


/*
 * Global enable for all features configured and controlled by the DWT unit.
 *  When DWTENA is set to 0 DWT registers return UNKNOWN values on reads.
 * In addition, it is IMPLEMENTATION DEFINED whether the processor ignores
 * writes to the DWT while DWTENA is 0.
 */
#define CM0_DEMCR_DWTENA                                    (1u << 24) /* <24:24> R:RW:0: */


/*
 * System Control Space ROM Table Peripheral ID #4
 */
#define CM0_SCS_PID4_ADDRESS                                (0xe000efd0)
#define CM0_SCS_PID4                                        (*(volatile uint32_t *)(0xe000efd0))
#define CM0_SCS_PID4_DEFAULT                                (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_SCS_PID4_VALUE_MASK                             (0xffffffff) /* <0:31> :R:4: */
#define CM0_SCS_PID4_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #0
 */
#define CM0_SCS_PID0_ADDRESS                                (0xe000efe0)
#define CM0_SCS_PID0                                        (*(volatile uint32_t *)(0xe000efe0))
#define CM0_SCS_PID0_DEFAULT                                (0x00000008)

/*
 * Peripheral ID #0
 */
#define CM0_SCS_PID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:8: */
#define CM0_SCS_PID0_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #1
 */
#define CM0_SCS_PID1_ADDRESS                                (0xe000efe4)
#define CM0_SCS_PID1                                        (*(volatile uint32_t *)(0xe000efe4))
#define CM0_SCS_PID1_DEFAULT                                (0x000000b0)

/*
 * Peripheral ID #1
 */
#define CM0_SCS_PID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:176: */
#define CM0_SCS_PID1_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #2
 */
#define CM0_SCS_PID2_ADDRESS                                (0xe000efe8)
#define CM0_SCS_PID2                                        (*(volatile uint32_t *)(0xe000efe8))
#define CM0_SCS_PID2_DEFAULT                                (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_SCS_PID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:11: */
#define CM0_SCS_PID2_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Peripheral ID #3
 */
#define CM0_SCS_PID3_ADDRESS                                (0xe000efec)
#define CM0_SCS_PID3                                        (*(volatile uint32_t *)(0xe000efec))
#define CM0_SCS_PID3_DEFAULT                                (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_SCS_PID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:0: */
#define CM0_SCS_PID3_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #0
 */
#define CM0_SCS_CID0_ADDRESS                                (0xe000eff0)
#define CM0_SCS_CID0                                        (*(volatile uint32_t *)(0xe000eff0))
#define CM0_SCS_CID0_DEFAULT                                (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_SCS_CID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:13: */
#define CM0_SCS_CID0_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #1
 */
#define CM0_SCS_CID1_ADDRESS                                (0xe000eff4)
#define CM0_SCS_CID1                                        (*(volatile uint32_t *)(0xe000eff4))
#define CM0_SCS_CID1_DEFAULT                                (0x000000e0)

/*
 * Component ID #1
 */
#define CM0_SCS_CID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:224: */
#define CM0_SCS_CID1_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #2
 */
#define CM0_SCS_CID2_ADDRESS                                (0xe000eff8)
#define CM0_SCS_CID2                                        (*(volatile uint32_t *)(0xe000eff8))
#define CM0_SCS_CID2_DEFAULT                                (0x00000005)

/*
 * Component ID #2
 */
#define CM0_SCS_CID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:5: */
#define CM0_SCS_CID2_VALUE_POS                              (0)


/*
 * System Control Space ROM Table Component ID #3
 */
#define CM0_SCS_CID3_ADDRESS                                (0xe000effc)
#define CM0_SCS_CID3                                        (*(volatile uint32_t *)(0xe000effc))
#define CM0_SCS_CID3_DEFAULT                                (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_SCS_CID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:177: */
#define CM0_SCS_CID3_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral #0
 */
#define CM0_ROM_SCS_ADDRESS                                 (0xe00ff000)
#define CM0_ROM_SCS                                         (*(volatile uint32_t *)(0xe00ff000))
#define CM0_ROM_SCS_DEFAULT                                 (0xfff0f003)

/*
 * Offset to SCS ROM Table
 */
#define CM0_ROM_SCS_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4293980163: */
#define CM0_ROM_SCS_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table Peripheral #1
 */
#define CM0_ROM_DWT_ADDRESS                                 (0xe00ff004)
#define CM0_ROM_DWT                                         (*(volatile uint32_t *)(0xe00ff004))
#define CM0_ROM_DWT_DEFAULT                                 (0xfff02003)

/*
 * Offset to DWT ROM Table
 */
#define CM0_ROM_DWT_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4293926915: */
#define CM0_ROM_DWT_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table Peripheral #2
 */
#define CM0_ROM_BPU_ADDRESS                                 (0xe00ff008)
#define CM0_ROM_BPU                                         (*(volatile uint32_t *)(0xe00ff008))
#define CM0_ROM_BPU_DEFAULT                                 (0xfff03003)

/*
 * Offset to BPU ROM Table
 */
#define CM0_ROM_BPU_VALUE_MASK                              (0xffffffff) /* <0:31> :R:4293931011: */
#define CM0_ROM_BPU_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table End Marker
 */
#define CM0_ROM_END_ADDRESS                                 (0xe00ff00c)
#define CM0_ROM_END                                         (*(volatile uint32_t *)(0xe00ff00c))
#define CM0_ROM_END_DEFAULT                                 (0x00000000)

/*
 * End marker in peripheral list
 */
#define CM0_ROM_END_VALUE_MASK                              (0xffffffff) /* <0:31> :R:0: */
#define CM0_ROM_END_VALUE_POS                               (0)


/*
 * CM0 CoreSight ROM Table Memory Type
 */
#define CM0_ROM_CSMT_ADDRESS                                (0xe00fffcc)
#define CM0_ROM_CSMT                                        (*(volatile uint32_t *)(0xe00fffcc))
#define CM0_ROM_CSMT_DEFAULT                                (0x00000001)

/*
 * Memory Type
 */
#define CM0_ROM_CSMT_VALUE_MASK                             (0xffffffff) /* <0:31> :R:1: */
#define CM0_ROM_CSMT_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #4
 */
#define CM0_ROM_PID4_ADDRESS                                (0xe00fffd0)
#define CM0_ROM_PID4                                        (*(volatile uint32_t *)(0xe00fffd0))
#define CM0_ROM_PID4_DEFAULT                                (0x00000004)

/*
 * Peripheral ID #4
 */
#define CM0_ROM_PID4_VALUE_MASK                             (0xffffffff) /* <0:31> :R:4: */
#define CM0_ROM_PID4_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #0
 */
#define CM0_ROM_PID0_ADDRESS                                (0xe00fffe0)
#define CM0_ROM_PID0                                        (*(volatile uint32_t *)(0xe00fffe0))
#define CM0_ROM_PID0_DEFAULT                                (0x00000071)

/*
 * Peripheral ID #0
 */
#define CM0_ROM_PID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:113: */
#define CM0_ROM_PID0_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #1
 */
#define CM0_ROM_PID1_ADDRESS                                (0xe00fffe4)
#define CM0_ROM_PID1                                        (*(volatile uint32_t *)(0xe00fffe4))
#define CM0_ROM_PID1_DEFAULT                                (0x000000b4)

/*
 * Peripheral ID #1
 */
#define CM0_ROM_PID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:180: */
#define CM0_ROM_PID1_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #2
 */
#define CM0_ROM_PID2_ADDRESS                                (0xe00fffe8)
#define CM0_ROM_PID2                                        (*(volatile uint32_t *)(0xe00fffe8))
#define CM0_ROM_PID2_DEFAULT                                (0x0000000b)

/*
 * Peripheral ID #2
 */
#define CM0_ROM_PID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:11: */
#define CM0_ROM_PID2_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Peripheral ID #3
 */
#define CM0_ROM_PID3_ADDRESS                                (0xe00fffec)
#define CM0_ROM_PID3                                        (*(volatile uint32_t *)(0xe00fffec))
#define CM0_ROM_PID3_DEFAULT                                (0x00000000)

/*
 * Peripheral ID #3
 */
#define CM0_ROM_PID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:0: */
#define CM0_ROM_PID3_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #0
 */
#define CM0_ROM_CID0_ADDRESS                                (0xe00ffff0)
#define CM0_ROM_CID0                                        (*(volatile uint32_t *)(0xe00ffff0))
#define CM0_ROM_CID0_DEFAULT                                (0x0000000d)

/*
 * Component ID #0
 */
#define CM0_ROM_CID0_VALUE_MASK                             (0xffffffff) /* <0:31> :R:13: */
#define CM0_ROM_CID0_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #1
 */
#define CM0_ROM_CID1_ADDRESS                                (0xe00ffff4)
#define CM0_ROM_CID1                                        (*(volatile uint32_t *)(0xe00ffff4))
#define CM0_ROM_CID1_DEFAULT                                (0x00000010)

/*
 * Component ID #1
 */
#define CM0_ROM_CID1_VALUE_MASK                             (0xffffffff) /* <0:31> :R:16: */
#define CM0_ROM_CID1_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #2
 */
#define CM0_ROM_CID2_ADDRESS                                (0xe00ffff8)
#define CM0_ROM_CID2                                        (*(volatile uint32_t *)(0xe00ffff8))
#define CM0_ROM_CID2_DEFAULT                                (0x00000005)

/*
 * Component ID #2
 */
#define CM0_ROM_CID2_VALUE_MASK                             (0xffffffff) /* <0:31> :R:5: */
#define CM0_ROM_CID2_VALUE_POS                              (0)


/*
 * CM0 CoreSight ROM Table Component ID #3
 */
#define CM0_ROM_CID3_ADDRESS                                (0xe00ffffc)
#define CM0_ROM_CID3                                        (*(volatile uint32_t *)(0xe00ffffc))
#define CM0_ROM_CID3_DEFAULT                                (0x000000b1)

/*
 * Component ID #3
 */
#define CM0_ROM_CID3_VALUE_MASK                             (0xffffffff) /* <0:31> :R:177: */
#define CM0_ROM_CID3_VALUE_POS                              (0)


/*
 * Link to Cortex M0 ROM Table.
 */
#define ROMTABLE_ADDR_ADDRESS                               (0xf0000000)
#define ROMTABLE_ADDR                                       (*(volatile uint32_t *)(0xf0000000))
#define ROMTABLE_ADDR_DEFAULT                               (0xf00ff003)

/*
 * Entry present.
 */
#define ROMTABLE_ADDR_PRESENT                               (1u << 0) /* <0:0> R:R:1: */


/*
 * ROM Table format:
 * '0: 8-bit format.
 * '1': 32-bit format.
 */
#define ROMTABLE_ADDR_FORMAT_32BIT                          (1u << 1) /* <1:1> R:R:1: */


/*
 * Address offset of the Cortex-M0 ROM Table base address (0xe00f:f000) wrt.
 * Cypress chip specific ROM Table base address (0xf000:0000). ADDR_OFFSET[19:0]
 * = 0xe00f:f - 0xf000:0 = 0xf00f:f.
 */
#define ROMTABLE_ADDR_ADDR_OFFSET_MASK                      (0xfffff000) /* <12:31> R:R:983295: */
#define ROMTABLE_ADDR_ADDR_OFFSET_POS                       (12)


/*
 * Device Type Identifier register.
 */
#define ROMTABLE_DID_ADDRESS                                (0xf0000fcc)
#define ROMTABLE_DID                                        (*(volatile uint32_t *)(0xf0000fcc))
#define ROMTABLE_DID_DEFAULT                                (0x00000001)

/*
 * .
 */
#define ROMTABLE_DID_VALUE_MASK                             (0xffffffff) /* <0:31> R:R:1: */
#define ROMTABLE_DID_VALUE_POS                              (0)


/*
 * Peripheral Identification Register 4.
 */
#define ROMTABLE_PID4_ADDRESS                               (0xf0000fd0)
#define ROMTABLE_PID4                                       (*(volatile uint32_t *)(0xf0000fd0))
#define ROMTABLE_PID4_DEFAULT                               (0x00000000)

/*
 * JEP106 continuation code.  This value is product specific and specified
 * as part of the product definition in the CPUSS.JEPCONTINUATION parameter.
 */
#define ROMTABLE_PID4_JEP_CONTINUATION_MASK                 (0x0000000f) /* <0:3> R:R:Undefined: */
#define ROMTABLE_PID4_JEP_CONTINUATION_POS                  (0)


/*
 * Size of ROM Table is 2^COUNT * 4 KByte.
 */
#define ROMTABLE_PID4_COUNT_MASK                            (0x000000f0) /* <4:7> R:R:0: */
#define ROMTABLE_PID4_COUNT_POS                             (4)


/*
 * Peripheral Identification Register 5.
 */
#define ROMTABLE_PID5_ADDRESS                               (0xf0000fd4)
#define ROMTABLE_PID5                                       (*(volatile uint32_t *)(0xf0000fd4))
#define ROMTABLE_PID5_DEFAULT                               (0x00000000)

/*
 * .
 */
#define ROMTABLE_PID5_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:0: */
#define ROMTABLE_PID5_VALUE_POS                             (0)


/*
 * Peripheral Identification Register 6.
 */
#define ROMTABLE_PID6_ADDRESS                               (0xf0000fd8)
#define ROMTABLE_PID6                                       (*(volatile uint32_t *)(0xf0000fd8))
#define ROMTABLE_PID6_DEFAULT                               (0x00000000)

/*
 * .
 */
#define ROMTABLE_PID6_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:0: */
#define ROMTABLE_PID6_VALUE_POS                             (0)


/*
 * Peripheral Identification Register 7.
 */
#define ROMTABLE_PID7_ADDRESS                               (0xf0000fdc)
#define ROMTABLE_PID7                                       (*(volatile uint32_t *)(0xf0000fdc))
#define ROMTABLE_PID7_DEFAULT                               (0x00000000)

/*
 * .
 */
#define ROMTABLE_PID7_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:0: */
#define ROMTABLE_PID7_VALUE_POS                             (0)


/*
 * Peripheral Identification Register 0.
 */
#define ROMTABLE_PID0_ADDRESS                               (0xf0000fe0)
#define ROMTABLE_PID0                                       (*(volatile uint32_t *)(0xf0000fe0))
#define ROMTABLE_PID0_DEFAULT                               (0x00000000)

/*
 * JEP106 part number. 4 lsbs of CPUSS.PARTNUMBER parameter.  These part
 * numbers are maintained in spec 40-9500.
 */
#define ROMTABLE_PID0_PN_MIN_MASK                           (0x000000ff) /* <0:7> R:R:Undefined: */
#define ROMTABLE_PID0_PN_MIN_POS                            (0)


/*
 * Peripheral Identification Register 1.
 */
#define ROMTABLE_PID1_ADDRESS                               (0xf0000fe4)
#define ROMTABLE_PID1                                       (*(volatile uint32_t *)(0xf0000fe4))
#define ROMTABLE_PID1_DEFAULT                               (0x00000000)

/*
 * JEP106 part number. 4 msbs of CPUSS.PARTNUMBER parameter.  These part
 * numbers are maintained in spec 40-9500.
 */
#define ROMTABLE_PID1_PN_MAJ_MASK                           (0x0000000f) /* <0:3> R:R:Undefined: */
#define ROMTABLE_PID1_PN_MAJ_POS                            (0)


/*
 * JEP106 vendor id. 4 lsbs of CPUSS.JEPID parameter.  This number is maintained
 * in spec 40-9500.
 */
#define ROMTABLE_PID1_JEPID_MIN_MASK                        (0x000000f0) /* <4:7> R:R:Undefined: */
#define ROMTABLE_PID1_JEPID_MIN_POS                         (4)


/*
 * Peripheral Identification Register 2.
 */
#define ROMTABLE_PID2_ADDRESS                               (0xf0000fe8)
#define ROMTABLE_PID2                                       (*(volatile uint32_t *)(0xf0000fe8))
#define ROMTABLE_PID2_DEFAULT                               (0x00000000)

/*
 * JEP106 vendor id. 4 msbs of CPUSS.JEPID parameter.  This number is maintained
 * in spec 40-9500.
 */
#define ROMTABLE_PID2_JEPID_MAJ_MASK                        (0x00000007) /* <0:2> R:R:Undefined: */
#define ROMTABLE_PID2_JEPID_MAJ_POS                         (0)


/*
 * Major REVision number (chip specific). Identifies the design iteration
 * of the component. For first tape out: 0x1. This field is implemented in
 * RTL by an ECO-able tie-off structure and is incremented on subsequent
 * tape outs.
 */
#define ROMTABLE_PID2_REV_MASK                              (0x000000f0) /* <4:7> R:R:Undefined: */
#define ROMTABLE_PID2_REV_POS                               (4)


/*
 * Peripheral Identification Register 3.
 */
#define ROMTABLE_PID3_ADDRESS                               (0xf0000fec)
#define ROMTABLE_PID3                                       (*(volatile uint32_t *)(0xf0000fec))
#define ROMTABLE_PID3_DEFAULT                               (0x00000000)

/*
 * Customer modified field. This field is used to track modifications to
 * the original component design as a result of componenet IP reuse.
 */
#define ROMTABLE_PID3_CM_MASK                               (0x0000000f) /* <0:3> R:R:0: */
#define ROMTABLE_PID3_CM_POS                                (0)


/*
 * Minor REVision number (chip specific). For first tape out: 0x1. This field
 * is implemented in RTL by an ECO-able tie-off structure and is incremented
 * on subsequent tape outs.
 */
#define ROMTABLE_PID3_REV_AND_MASK                          (0x000000f0) /* <4:7> R:R:Undefined: */
#define ROMTABLE_PID3_REV_AND_POS                           (4)


/*
 * Component Identification Register 0.
 */
#define ROMTABLE_CID0_ADDRESS                               (0xf0000ff0)
#define ROMTABLE_CID0                                       (*(volatile uint32_t *)(0xf0000ff0))
#define ROMTABLE_CID0_DEFAULT                               (0x0000000d)

/*
 * Component identification byte 0 of 4-byte component identification 0xB105:100D.
 */
#define ROMTABLE_CID0_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:13: */
#define ROMTABLE_CID0_VALUE_POS                             (0)


/*
 * Component Identification Register 1.
 */
#define ROMTABLE_CID1_ADDRESS                               (0xf0000ff4)
#define ROMTABLE_CID1                                       (*(volatile uint32_t *)(0xf0000ff4))
#define ROMTABLE_CID1_DEFAULT                               (0x00000010)

/*
 * Component identification byte 1 of 4-byte component identification 0xB105:100D.
 * Component class: "ROM Table".
 */
#define ROMTABLE_CID1_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:16: */
#define ROMTABLE_CID1_VALUE_POS                             (0)


/*
 * Component Identification Register 2.
 */
#define ROMTABLE_CID2_ADDRESS                               (0xf0000ff8)
#define ROMTABLE_CID2                                       (*(volatile uint32_t *)(0xf0000ff8))
#define ROMTABLE_CID2_DEFAULT                               (0x00000005)

/*
 * Component identification byte 2 of 4-byte component identification 0xB105:100D.
 */
#define ROMTABLE_CID2_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:5: */
#define ROMTABLE_CID2_VALUE_POS                             (0)


/*
 * Component Identification Register 3.
 */
#define ROMTABLE_CID3_ADDRESS                               (0xf0000ffc)
#define ROMTABLE_CID3                                       (*(volatile uint32_t *)(0xf0000ffc))
#define ROMTABLE_CID3_DEFAULT                               (0x000000b1)

/*
 * Component identification byte 3 of 4-byte component identification 0xB105:100D.
 */
#define ROMTABLE_CID3_VALUE_MASK                            (0xffffffff) /* <0:31> R:R:177: */
#define ROMTABLE_CID3_VALUE_POS                             (0)



#endif /* _CCG4PD_REGS_H_ */

