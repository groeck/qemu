/*
 * Allwinner A10 Clock Control Module
 *
 * Copyright (C) 2016 Guenter Roeck
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef AW_A10_CCM_H
#define AW_A10_CCM_H

#include "hw/sysbus.h"

#define TYPE_AW_A10_CCM "allwinner-A10-ccm"
#define AW_A10_CCM(obj) OBJECT_CHECK(AwA10CCMState, (obj), TYPE_AW_A10_CCM)

#define PLL1_CFG_REG		0x0000
#define PLL1_TUN_REG		0x0004
#define PLL2_CFG_REG		0x0008
#define PLL2_TUN_REG		0x000c
#define PLL3_CFG_REG		0x0010
#define PLL4_CFG_REG		0x0018
#define PLL5_CFG_REG		0x0020
#define PLL6_CFG_REG		0x0028
#define PLL6_TUN_REG		0x002c
#define PLL6_TUN_REG		0x002c
#define PLL7_CFG_REG		0x0030
#define PLL1_TUN2_REG		0x0038
#define PLL5_TUN2_REG		0x003c
#define OSC24M_CFG_REG		0x0050
#define CPU_AHB_APB0_CFG_REG	0x0054
#define APB1_CLK_DIV_REG	0x0058
#define AXI_GATING_REG		0x005c
#define AHB_GATING_REG0		0x0060
#define AHB_GATING_REG1		0x0064
#define APB0_GATING_REG		0x0068
#define APB1_GATING_REG		0x006c
#define NAND_SCLK_CFG_REG	0x0080
#define SD0_CLK_REG		0x0088
#define SD1_CLK_REG		0x008c
#define SD2_CLK_REG		0x0090
#define SD3_CLK_REG		0x0094
#define TS_CLK_REG		0x0098
#define SS_CLK_REG		0x009c
#define SPI0_CLK_REG		0x00a0
#define SPI1_CLK_REG		0x00a4
#define SPI2_CLK_REG		0x00a8
#define IR0_CLK_REG		0x00b0
#define IR1_CLK_REG		0x00b4
#define IIS_CLK_REG		0x00b8
#define AC97_CLK_REG		0x00bc
#define KEYPAD_CLK_REG		0x00c4

#define SPI3_CLK_REG		0x00d4
#define DRAM_CLK_REG		0x0100

typedef struct AwA10CCMState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion iomem;

    uint32_t pll1_cfg;
    uint32_t pll1_tun;
    uint32_t pll2_cfg;
    uint32_t pll2_tun;
    uint32_t pll3_cfg;
    uint32_t pll4_cfg;
    uint32_t pll5_cfg;
    uint32_t pll5_tun;
    uint32_t pll6_cfg;
    uint32_t pll6_tun;
    uint32_t pll7_cfg;
    uint32_t pll1_tun2;
    uint32_t pll5_tun2;
    uint32_t osc24m_cfg;
    uint32_t cpu_ahb_apb0_cfg;
    uint32_t apb1_clk_div;
    uint32_t ahb_gating0;
    uint32_t ahb_gating1;
    uint32_t apb0_gating;
    uint32_t apb1_gating;
    uint32_t nand_sclk_cfg;
    uint32_t sd0_clk;
    uint32_t sd1_clk;
    uint32_t sd2_clk;
    uint32_t sd3_clk;
    uint32_t ts_clk;
    uint32_t ss_clk;
    uint32_t spi0_clk;
    uint32_t spi1_clk;
    uint32_t spi2_clk;
    uint32_t ir0_clk;
    uint32_t ir1_clk;
    uint32_t iis_clk;
    uint32_t ac97_clk;
    uint32_t keypad_clk;
    uint32_t spi3_clk;
    uint32_t dram_clk;

#if 0
    /* Frequencies precalculated on register changes */
    uint32_t pll_refclk_freq;
    uint32_t mcu_clk_freq;
    uint32_t hsp_clk_freq;
    uint32_t ipg_clk_freq;
#endif
} AwA10CCMState;

// uint32_t imx_clock_frequency(DeviceState *s, AW_A10Clk clock);

#endif /* AW_A10_CCM_H */
