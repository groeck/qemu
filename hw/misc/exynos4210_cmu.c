/*
 *  exynos4210 Clock Management Units (CMUs) Emulation
 *
 *  Copyright (C) 2011 Samsung Electronics Co Ltd.
 *    Maksim Kozlov, <m.kozlov@samsung.com>
 *
 *  Copyright (c) 2015 Guenter Roeck <linux@roeck-us.net>
 *    Port to QEMU 2.4.0, simplifications
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "hw/sysbus.h"
#include "hw/arm/exynos4210.h"

#define DEBUG_CMU         0
#define DEBUG_CMU_EXTEND  0

#if DEBUG_CMU || DEBUG_CMU_EXTEND

    #define  PRINT_DEBUG(fmt, args...)  \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)

    #define  PRINT_DEBUG_SIMPLE(fmt, args...)  \
        do { \
            fprintf(stderr, fmt, ## args); \
        } while (0)

#if DEBUG_CMU_EXTEND

    #define  PRINT_DEBUG_EXTEND(fmt, args...) \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)
#else
    #define  PRINT_DEBUG_EXTEND(fmt, args...) \
        do {} while (0)
#endif /* EXTEND */

#else
    #define  PRINT_DEBUG(fmt, args...) \
        do {} while (0)
    #define  PRINT_DEBUG_SIMPLE(fmt, args...) \
        do {} while (0)
    #define  PRINT_DEBUG_EXTEND(fmt, args...) \
        do {} while (0)
#endif

#define  PRINT_ERROR(fmt, args...)                                          \
        do {                                                                \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)

/* function blocks */
#define LEFTBUS_BLK   0x0
#define RIGHTBUS_BLK  0x0
#define TOP_BLK       0x10
#define TOP0_BLK      0x10
#define TOP1_BLK      0x14
#define DMC_BLK       0x0
#define DMC0_BLK      0x0
#define DMC1_BLK      0x4
#define CPU_BLK       0x0
#define CPU0_BLK      0x0
#define CPU1_BLK      0x4

#define CAM_BLK       0x20
#define TV_BLK        0x24
#define MFC_BLK       0x28
#define G3D_BLK       0x2C
#define IMAGE_BLK     0x30
#define LCD0_BLK      0x34
#define LCD1_BLK      0x38
#define MAUDIO_BLK    0x3C
#define FSYS_BLK      0x40
#define FSYS0_BLK     0x40
#define FSYS1_BLK     0x44
#define FSYS2_BLK     0x48
#define FSYS3_BLK     0x4C
#define GPS_BLK       0x4C /*  CLK_GATE_IP_GPS in CMU_TOP */
#define PERIL_BLK     0x50
#define PERIL0_BLK    0x50
#define PERIL1_BLK    0x54
#define PERIL2_BLK    0x58
#define PERIL3_BLK    0x5C
#define PERIL4_BLK    0x60
#define PERIR_BLK     0x60 /* CLK_GATE_IP_PERIR in CMU_TOP */
#define PERIL5_BLK    0x64

#define BLOCK_MASK    0xFF

/* PLLs */
/* located in CMU_CPU block */
#define APLL  0x00
#define MPLL  0x08
/* located in CMU_TOP block */
#define EPLL  0x10
#define VPLL  0x20

/* groups of registers */
#define PLL_LOCK       0x000
#define PLL_CON        0x100
#define CLK_SRC        0x200
#define CLK_SRC_MASK   0x300
#define CLK_MUX_STAT   0x400
#define CLK_DIV        0x500
#define CLK_DIV_STAT   0x600
#define CLK_GATE_SCLK  0x800
#define CLK_GATE_IP    0x900

#define GROUP_MASK     0xF00

#define PLL_LOCK_(pll)  (PLL_LOCK + (pll))
#define PLL_CON0_(pll)  (PLL_CON  + (pll))
#define PLL_CON1_(pll)  (PLL_CON  + (pll) + 4)

#define CLK_SRC_(block)         (CLK_SRC       + (block))
#define CLK_SRC_MASK_(block)    (CLK_SRC_MASK  + (block))
#define CLK_MUX_STAT_(block)    (CLK_MUX_STAT  + (block))
#define CLK_DIV_(block)         (CLK_DIV       + (block))
#define CLKDIV2_RATIO           0x580 /* described for CMU_TOP only */
#define CLK_DIV_STAT_(block)    (CLK_DIV_STAT  + (block))
#define CLKDIV2_STAT            0x680 /* described for CMU_TOP only */
#define CLK_GATE_SCLK_(block)   (CLK_GATE_SCLK + (block))
/* For CMU_LEFTBUS and CMU_RIGHTBUS, CLK_GATE_IP_XXX
   registers are located at 0x800. */
#define CLK_GATE_IP_LR_BUS      0x800
#define CLK_GATE_IP_(block)     (CLK_GATE_IP + (block))
#define CLK_GATE_BLOCK          0x970 /* described for CMU_TOP only */
#define CLKOUT_CMU              0xA00
#define CLKOUT_CMU_DIV_STAT     0xA04

/*
 * registers which are located outside of 0xAFF region
 */
/* CMU_DMC */
#define DCGIDX_MAP0        0x01000
#define DCGIDX_MAP1        0x01004
#define DCGIDX_MAP2        0x01008
#define DCGPERF_MAP0       0x01020
#define DCGPERF_MAP1       0x01024
#define DVCIDX_MAP         0x01040
#define FREQ_CPU           0x01060
#define FREQ_DPM           0x01064
#define DVSEMCLK_EN        0x01080
#define MAXPERF            0x01084
/* CMU_CPU */
#define ARMCLK_STOPCTRL    0x01000
#define ATCLK_STOPCTRL     0x01004
#define PARITYFAIL_STATUS  0x01010
#define PARITYFAIL_CLEAR   0x01014
#define PWR_CTRL           0x01020
#define APLL_CON0_L8       0x01100
#define APLL_CON0_L7       0x01104
#define APLL_CON0_L6       0x01108
#define APLL_CON0_L5       0x0110C
#define APLL_CON0_L4       0x01110
#define APLL_CON0_L3       0x01114
#define APLL_CON0_L2       0x01118
#define APLL_CON0_L1       0x0111C
#define IEM_CONTROL        0x01120
#define APLL_CON1_L8       0x01200
#define APLL_CON1_L7       0x01204
#define APLL_CON1_L6       0x01208
#define APLL_CON1_L5       0x0120C
#define APLL_CON1_L4       0x01210
#define APLL_CON1_L3       0x01214
#define APLL_CON1_L2       0x01218
#define APLL_CON1_L1       0x0121C
#define CLKDIV_IEM_L8      0x01300
#define CLKDIV_IEM_L7      0x01304
#define CLKDIV_IEM_L6      0x01308
#define CLKDIV_IEM_L5      0x0130C
#define CLKDIV_IEM_L4      0x01310
#define CLKDIV_IEM_L3      0x01314
#define CLKDIV_IEM_L2      0x01318
#define CLKDIV_IEM_L1      0x0131C

#define EXTENDED_REGION_MASK    0x1000
#define EXTENDED_REGISTER_MASK  0xFFF

typedef struct {
    const char *name; /* for debugging */
    uint32_t    offset;
    uint32_t    reset_value;
} Exynos4210CmuReg;


static const Exynos4210CmuReg exynos4210_cmu_leftbus_regs[] = {
    /* CMU_LEFTBUS registers */
    { "CLK_SRC_LEFTBUS",             CLK_SRC_(LEFTBUS_BLK),        0x00000000 },
    { "CLK_MUX_STAT_LEFTBUS",        CLK_MUX_STAT_(LEFTBUS_BLK),   0x00000001 },
    { "CLK_DIV_LEFTBUS",             CLK_DIV_(LEFTBUS_BLK),        0x00000000 },
    { "CLK_DIV_STAT_LEFTBUS",        CLK_DIV_STAT_(LEFTBUS_BLK),   0x00000000 },
    { "CLK_GATE_IP_LEFTBUS",         CLK_GATE_IP_LR_BUS,           0xFFFFFFFF },
    { "CLKOUT_CMU_LEFTBUS",          CLKOUT_CMU,                   0x00010000 },
    { "CLKOUT_CMU_LEFTBUS_DIV_STAT", CLKOUT_CMU_DIV_STAT,          0x00000000 },
};

static const Exynos4210CmuReg exynos4210_cmu_rightbus_regs[] = {
    /* CMU_RIGHTBUS registers */
    { "CLK_SRC_RIGHTBUS",             CLK_SRC_(RIGHTBUS_BLK),      0x00000000 },
    { "CLK_MUX_STAT_RIGHTBUS",        CLK_MUX_STAT_(RIGHTBUS_BLK), 0x00000001 },
    { "CLK_DIV_RIGHTBUS",             CLK_DIV_(RIGHTBUS_BLK),      0x00000000 },
    { "CLK_DIV_STAT_RIGHTBUS",        CLK_DIV_STAT_(RIGHTBUS_BLK), 0x00000000 },
    { "CLK_GATE_IP_RIGHTBUS",         CLK_GATE_IP_LR_BUS,          0xFFFFFFFF },
    { "CLKOUT_CMU_RIGHTBUS",          CLKOUT_CMU,                  0x00010000 },
    { "CLKOUT_CMU_RIGHTBUS_DIV_STAT", CLKOUT_CMU_DIV_STAT,         0x00000000 },
};

static const Exynos4210CmuReg exynos4210_cmu_top_regs[] = {
    /* CMU_TOP registers */
    { "EPLL_LOCK",               PLL_LOCK_(EPLL),              0x00000FFF },
    { "VPLL_LOCK",               PLL_LOCK_(VPLL),              0x00000FFF },
    { "EPLL_CON0",               PLL_CON0_(EPLL),              0x00300301 },
    { "EPLL_CON1",               PLL_CON1_(EPLL),              0x00000000 },
    { "VPLL_CON0",               PLL_CON0_(VPLL),              0x00240201 },
    { "VPLL_CON1",               PLL_CON1_(VPLL),              0x66010464 },
    { "CLK_SRC_TOP0",            CLK_SRC_(TOP0_BLK),           0x00000000 },
    { "CLK_SRC_TOP1",            CLK_SRC_(TOP1_BLK),           0x00000000 },
    { "CLK_SRC_CAM",             CLK_SRC_(CAM_BLK),            0x11111111 },
    { "CLK_SRC_TV",              CLK_SRC_(TV_BLK),             0x00000000 },
    { "CLK_SRC_MFC",             CLK_SRC_(MFC_BLK),            0x00000000 },
    { "CLK_SRC_G3D",             CLK_SRC_(G3D_BLK),            0x00000000 },
    { "CLK_SRC_IMAGE",           CLK_SRC_(IMAGE_BLK),          0x00000000 },
    { "CLK_SRC_LCD0",            CLK_SRC_(LCD0_BLK),           0x00001111 },
    { "CLK_SRC_LCD1",            CLK_SRC_(LCD1_BLK),           0x00001111 },
    { "CLK_SRC_MAUDIO",          CLK_SRC_(MAUDIO_BLK),         0x00000005 },
    { "CLK_SRC_FSYS",            CLK_SRC_(FSYS_BLK),           0x00011111 },
    { "CLK_SRC_PERIL0",          CLK_SRC_(PERIL0_BLK),         0x00011111 },
    { "CLK_SRC_PERIL1",          CLK_SRC_(PERIL1_BLK),         0x01110055 },
    { "CLK_SRC_MASK_TOP",        CLK_SRC_MASK_(TOP_BLK),       0x00000001 },
    { "CLK_SRC_MASK_CAM",        CLK_SRC_MASK_(CAM_BLK),       0x11111111 },
    { "CLK_SRC_MASK_TV",         CLK_SRC_MASK_(TV_BLK),        0x00000111 },
    { "CLK_SRC_MASK_LCD0",       CLK_SRC_MASK_(LCD0_BLK),      0x00001111 },
    { "CLK_SRC_MASK_LCD1",       CLK_SRC_MASK_(LCD1_BLK),      0x00001111 },
    { "CLK_SRC_MASK_MAUDIO",     CLK_SRC_MASK_(MAUDIO_BLK),    0x00000001 },
    { "CLK_SRC_MASK_FSYS",       CLK_SRC_MASK_(FSYS_BLK),      0x01011111 },
    { "CLK_SRC_MASK_PERIL0",     CLK_SRC_MASK_(PERIL0_BLK),    0x00011111 },
    { "CLK_SRC_MASK_PERIL1",     CLK_SRC_MASK_(PERIL1_BLK),    0x01110111 },
    { "CLK_MUX_STAT_TOP",        CLK_MUX_STAT_(TOP_BLK),       0x11111111 },
    { "CLK_MUX_STAT_MFC",        CLK_MUX_STAT_(MFC_BLK),       0x00000111 },
    { "CLK_MUX_STAT_G3D",        CLK_MUX_STAT_(G3D_BLK),       0x00000111 },
    { "CLK_MUX_STAT_IMAGE",      CLK_MUX_STAT_(IMAGE_BLK),     0x00000111 },
    { "CLK_DIV_TOP",             CLK_DIV_(TOP_BLK),            0x00000000 },
    { "CLK_DIV_CAM",             CLK_DIV_(CAM_BLK),            0x00000000 },
    { "CLK_DIV_TV",              CLK_DIV_(TV_BLK),             0x00000000 },
    { "CLK_DIV_MFC",             CLK_DIV_(MFC_BLK),            0x00000000 },
    { "CLK_DIV_G3D",             CLK_DIV_(G3D_BLK),            0x00000000 },
    { "CLK_DIV_IMAGE",           CLK_DIV_(IMAGE_BLK),          0x00000000 },
    { "CLK_DIV_LCD0",            CLK_DIV_(LCD0_BLK),           0x00700000 },
    { "CLK_DIV_LCD1",            CLK_DIV_(LCD1_BLK),           0x00700000 },
    { "CLK_DIV_MAUDIO",          CLK_DIV_(MAUDIO_BLK),         0x00000000 },
    { "CLK_DIV_FSYS0",           CLK_DIV_(FSYS0_BLK),          0x00B00000 },
    { "CLK_DIV_FSYS1",           CLK_DIV_(FSYS1_BLK),          0x00000000 },
    { "CLK_DIV_FSYS2",           CLK_DIV_(FSYS2_BLK),          0x00000000 },
    { "CLK_DIV_FSYS3",           CLK_DIV_(FSYS3_BLK),          0x00000000 },
    { "CLK_DIV_PERIL0",          CLK_DIV_(PERIL0_BLK),         0x00000000 },
    { "CLK_DIV_PERIL1",          CLK_DIV_(PERIL1_BLK),         0x00000000 },
    { "CLK_DIV_PERIL2",          CLK_DIV_(PERIL2_BLK),         0x00000000 },
    { "CLK_DIV_PERIL3",          CLK_DIV_(PERIL3_BLK),         0x00000000 },
    { "CLK_DIV_PERIL4",          CLK_DIV_(PERIL4_BLK),         0x00000000 },
    { "CLK_DIV_PERIL5",          CLK_DIV_(PERIL5_BLK),         0x00000000 },
    { "CLKDIV2_RATIO",           CLKDIV2_RATIO,                0x11111111 },
    { "CLK_DIV_STAT_TOP",        CLK_DIV_STAT_(TOP_BLK),       0x00000000 },
    { "CLK_DIV_STAT_CAM",        CLK_DIV_STAT_(CAM_BLK),       0x00000000 },
    { "CLK_DIV_STAT_TV",         CLK_DIV_STAT_(TV_BLK),        0x00000000 },
    { "CLK_DIV_STAT_MFC",        CLK_DIV_STAT_(MFC_BLK),       0x00000000 },
    { "CLK_DIV_STAT_G3D",        CLK_DIV_STAT_(G3D_BLK),       0x00000000 },
    { "CLK_DIV_STAT_IMAGE",      CLK_DIV_STAT_(IMAGE_BLK),     0x00000000 },
    { "CLK_DIV_STAT_LCD0",       CLK_DIV_STAT_(LCD0_BLK),      0x00000000 },
    { "CLK_DIV_STAT_LCD1",       CLK_DIV_STAT_(LCD1_BLK),      0x00000000 },
    { "CLK_DIV_STAT_MAUDIO",     CLK_DIV_STAT_(MAUDIO_BLK),    0x00000000 },
    { "CLK_DIV_STAT_FSYS0",      CLK_DIV_STAT_(FSYS0_BLK),     0x00000000 },
    { "CLK_DIV_STAT_FSYS1",      CLK_DIV_STAT_(FSYS1_BLK),     0x00000000 },
    { "CLK_DIV_STAT_FSYS2",      CLK_DIV_STAT_(FSYS2_BLK),     0x00000000 },
    { "CLK_DIV_STAT_FSYS3",      CLK_DIV_STAT_(FSYS3_BLK),     0x00000000 },
    { "CLK_DIV_STAT_PERIL0",     CLK_DIV_STAT_(PERIL0_BLK),    0x00000000 },
    { "CLK_DIV_STAT_PERIL1",     CLK_DIV_STAT_(PERIL1_BLK),    0x00000000 },
    { "CLK_DIV_STAT_PERIL2",     CLK_DIV_STAT_(PERIL2_BLK),    0x00000000 },
    { "CLK_DIV_STAT_PERIL3",     CLK_DIV_STAT_(PERIL3_BLK),    0x00000000 },
    { "CLK_DIV_STAT_PERIL4",     CLK_DIV_STAT_(PERIL4_BLK),    0x00000000 },
    { "CLK_DIV_STAT_PERIL5",     CLK_DIV_STAT_(PERIL5_BLK),    0x00000000 },
    { "CLKDIV2_STAT",            CLKDIV2_STAT,                 0x00000000 },
    { "CLK_GATE_SCLK_CAM",       CLK_GATE_SCLK_(CAM_BLK),      0xFFFFFFFF },
    { "CLK_GATE_IP_CAM",         CLK_GATE_IP_(CAM_BLK),        0xFFFFFFFF },
    { "CLK_GATE_IP_TV",          CLK_GATE_IP_(TV_BLK),         0xFFFFFFFF },
    { "CLK_GATE_IP_MFC",         CLK_GATE_IP_(MFC_BLK),        0xFFFFFFFF },
    { "CLK_GATE_IP_G3D",         CLK_GATE_IP_(G3D_BLK),        0xFFFFFFFF },
    { "CLK_GATE_IP_IMAGE",       CLK_GATE_IP_(IMAGE_BLK),      0xFFFFFFFF },
    { "CLK_GATE_IP_LCD0",        CLK_GATE_IP_(LCD0_BLK),       0xFFFFFFFF },
    { "CLK_GATE_IP_LCD1",        CLK_GATE_IP_(LCD1_BLK),       0xFFFFFFFF },
    { "CLK_GATE_IP_FSYS",        CLK_GATE_IP_(FSYS_BLK),       0xFFFFFFFF },
    { "CLK_GATE_IP_GPS",         CLK_GATE_IP_(GPS_BLK),        0xFFFFFFFF },
    { "CLK_GATE_IP_PERIL",       CLK_GATE_IP_(PERIL_BLK),      0xFFFFFFFF },
    { "CLK_GATE_IP_PERIR",       CLK_GATE_IP_(PERIR_BLK),      0xFFFFFFFF },
    { "CLK_GATE_BLOCK",          CLK_GATE_BLOCK,               0xFFFFFFFF },
    { "CLKOUT_CMU_TOP",          CLKOUT_CMU,                   0x00010000 },
    { "CLKOUT_CMU_TOP_DIV_STAT", CLKOUT_CMU_DIV_STAT,          0x00000000 },
};

static const Exynos4210CmuReg exynos4210_cmu_dmc_regs[] = {
    /* CMU_DMC registers */
    { "CLK_SRC_DMC",             CLK_SRC_(DMC_BLK),            0x00010000 },
    { "CLK_SRC_MASK_DMC",        CLK_SRC_MASK_(DMC_BLK),       0x00010000 },
    { "CLK_MUX_STAT_DMC",        CLK_MUX_STAT_(DMC_BLK),       0x11100110 },
    { "CLK_DIV_DMC0",            CLK_DIV_(DMC0_BLK),           0x00000000 },
    { "CLK_DIV_DMC1",            CLK_DIV_(DMC1_BLK),           0x00000000 },
    { "CLK_DIV_STAT_DMC0",       CLK_DIV_STAT_(DMC0_BLK),      0x00000000 },
    { "CLK_DIV_STAT_DMC1",       CLK_DIV_STAT_(DMC1_BLK),      0x00000000 },
    { "CLK_GATE_IP_DMC",         CLK_GATE_IP_(DMC_BLK),        0xFFFFFFFF },
    { "CLKOUT_CMU_DMC",          CLKOUT_CMU,                   0x00010000 },
    { "CLKOUT_CMU_DMC_DIV_STAT", CLKOUT_CMU_DIV_STAT,          0x00000000 },
    { "DCGIDX_MAP0",             DCGIDX_MAP0,                  0xFFFFFFFF },
    { "DCGIDX_MAP1",             DCGIDX_MAP1,                  0xFFFFFFFF },
    { "DCGIDX_MAP2",             DCGIDX_MAP2,                  0xFFFFFFFF },
    { "DCGPERF_MAP0",            DCGPERF_MAP0,                 0xFFFFFFFF },
    { "DCGPERF_MAP1",            DCGPERF_MAP1,                 0xFFFFFFFF },
    { "DVCIDX_MAP",              DVCIDX_MAP,                   0xFFFFFFFF },
    { "FREQ_CPU",                FREQ_CPU,                     0x00000000 },
    { "FREQ_DPM",                FREQ_DPM,                     0x00000000 },
    { "DVSEMCLK_EN",             DVSEMCLK_EN,                  0x00000000 },
    { "MAXPERF",                 MAXPERF,                      0x00000000 },
};

static const Exynos4210CmuReg exynos4210_cmu_cpu_regs[] = {
    /* CMU_CPU registers */
    { "APLL_LOCK",               PLL_LOCK_(APLL),             0x00000FFF },
    { "MPLL_LOCK",               PLL_LOCK_(MPLL),             0x00000FFF },
    { "APLL_CON0",               PLL_CON0_(APLL),             0x00C80601 },
    { "APLL_CON1",               PLL_CON1_(APLL),             0x0000001C },
    { "MPLL_CON0",               PLL_CON0_(MPLL),             0x00C80601 },
    { "MPLL_CON1",               PLL_CON1_(MPLL),             0x0000001C },
    { "CLK_SRC_CPU",             CLK_SRC_(CPU_BLK),           0x00000000 },
    { "CLK_MUX_STAT_CPU",        CLK_MUX_STAT_(CPU_BLK),      0x00110101 },
    { "CLK_DIV_CPU0",            CLK_DIV_(CPU0_BLK),          0x00000000 },
    { "CLK_DIV_CPU1",            CLK_DIV_(CPU1_BLK),          0x00000000 },
    { "CLK_DIV_STAT_CPU0",       CLK_DIV_STAT_(CPU0_BLK),     0x00000000 },
    { "CLK_DIV_STAT_CPU1",       CLK_DIV_STAT_(CPU1_BLK),     0x00000000 },
    { "CLK_GATE_SCLK_CPU",       CLK_GATE_SCLK_(CPU_BLK),     0xFFFFFFFF },
    { "CLK_GATE_IP_CPU",         CLK_GATE_IP_(CPU_BLK),       0xFFFFFFFF },
    { "CLKOUT_CMU_CPU",          CLKOUT_CMU,                  0x00010000 },
    { "CLKOUT_CMU_CPU_DIV_STAT", CLKOUT_CMU_DIV_STAT,         0x00000000 },
    { "ARMCLK_STOPCTRL",         ARMCLK_STOPCTRL,             0x00000044 },
    { "ATCLK_STOPCTRL",          ATCLK_STOPCTRL,              0x00000044 },
    { "PARITYFAIL_STATUS",       PARITYFAIL_STATUS,           0x00000000 },
    { "PARITYFAIL_CLEAR",        PARITYFAIL_CLEAR,            0x00000000 },
    { "PWR_CTRL",                PWR_CTRL,                    0x00000033 },
    { "APLL_CON0_L8",            APLL_CON0_L8,                0x00C80601 },
    { "APLL_CON0_L7",            APLL_CON0_L7,                0x00C80601 },
    { "APLL_CON0_L6",            APLL_CON0_L6,                0x00C80601 },
    { "APLL_CON0_L5",            APLL_CON0_L5,                0x00C80601 },
    { "APLL_CON0_L4",            APLL_CON0_L4,                0x00C80601 },
    { "APLL_CON0_L3",            APLL_CON0_L3,                0x00C80601 },
    { "APLL_CON0_L2",            APLL_CON0_L2,                0x00C80601 },
    { "APLL_CON0_L1",            APLL_CON0_L1,                0x00C80601 },
    { "IEM_CONTROL",             IEM_CONTROL,                 0x00000000 },
    { "APLL_CON1_L8",            APLL_CON1_L8,                0x00000000 },
    { "APLL_CON1_L7",            APLL_CON1_L7,                0x00000000 },
    { "APLL_CON1_L6",            APLL_CON1_L6,                0x00000000 },
    { "APLL_CON1_L5",            APLL_CON1_L5,                0x00000000 },
    { "APLL_CON1_L4",            APLL_CON1_L4,                0x00000000 },
    { "APLL_CON1_L3",            APLL_CON1_L3,                0x00000000 },
    { "APLL_CON1_L2",            APLL_CON1_L2,                0x00000000 },
    { "APLL_CON1_L1",            APLL_CON1_L1,                0x00000000 },
    { "CLKDIV_IEM_L8",           CLKDIV_IEM_L8,               0x00000000 },
    { "CLKDIV_IEM_L7",           CLKDIV_IEM_L7,               0x00000000 },
    { "CLKDIV_IEM_L6",           CLKDIV_IEM_L6,               0x00000000 },
    { "CLKDIV_IEM_L5",           CLKDIV_IEM_L5,               0x00000000 },
    { "CLKDIV_IEM_L4",           CLKDIV_IEM_L4,               0x00000000 },
    { "CLKDIV_IEM_L3",           CLKDIV_IEM_L3,               0x00000000 },
    { "CLKDIV_IEM_L2",           CLKDIV_IEM_L2,               0x00000000 },
    { "CLKDIV_IEM_L1",           CLKDIV_IEM_L1,               0x00000000 },
};

#define EXYNOS4210_CMU_REGS_MEM_SIZE     0x4000

/*
 * for indexing register in the uint32_t array
 *
 * 'reg' - register offset (see offsets definitions above)
 *
 */
#define I_(reg) (reg / sizeof(uint32_t))

#define XOM_0 1 /* Select XXTI (0) or XUSBXTI (1) base clock source */

/*
 *  Offsets in CLK_SRC_CPU register
 *  for control MUXMPLL and MUXAPLL
 *
 *  0 = FINPLL, 1 = MOUTM(A)PLLFOUT
 */
#define MUX_APLL_SEL_SHIFT 0
#define MUX_MPLL_SEL_SHIFT 8
#define MUX_CORE_SEL_SHIFT 16
#define MUX_HPM_SEL_SHIFT  20

#define MUX_APLL_SEL  (1 << MUX_APLL_SEL_SHIFT)
#define MUX_MPLL_SEL  (1 << MUX_MPLL_SEL_SHIFT)
#define MUX_CORE_SEL  (1 << MUX_CORE_SEL_SHIFT)
#define MUX_HPM_SEL   (1 << MUX_HPM_SEL_SHIFT)

/* Offsets for fields in CLK_MUX_STAT_CPU register */
#define APLL_SEL_SHIFT         0
#define APLL_SEL_MASK          0x00000007
#define MPLL_SEL_SHIFT         8
#define MPLL_SEL_MASK          0x00000700
#define CORE_SEL_SHIFT         16
#define CORE_SEL_MASK          0x00070000
#define HPM_SEL_SHIFT          20
#define HPM_SEL_MASK           0x00700000

/* Offsets for fields in <pll>_CON0 register */
#define PLL_ENABLE_SHIFT 31
#define PLL_ENABLE_MASK  0x80000000 /* [31] bit */
#define PLL_LOCKED_MASK  0x20000000 /* [29] bit */

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

    /* registers values */
    uint32_t reg[EXYNOS4210_CMU_REGS_MEM_SIZE / sizeof(uint32_t)];

    /* which CMU it is */
    Exynos4210Cmu cmu_id;

    /* registers information for debugging and resetting */
    const Exynos4210CmuReg *regs;
    int regs_number;
} Exynos4210CmuState;

#define TYPE_EXYNOS4210_CMU "exynos4210.cmu"
#define EXYNOS4210_CMU(obj) \
    OBJECT_CHECK(Exynos4210CmuState, (obj), TYPE_EXYNOS4210_CMU)

#if DEBUG_CMU_EXTEND
/* The only meaning of life - debugging. This function should be only used
 * inside PRINT_DEBUG_EXTEND macros
 */
static const char *exynos4210_cmu_regname(Exynos4210CmuState *s,
                                          hwaddr  offset)
{
    int i;

    for (i = 0; i < s->regs_number; i++) {
        if (offset == s->regs[i].offset) {
            return s->regs[i].name;
        }
    }

    return NULL;
}
#endif

static uint64_t exynos4210_cmu_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    Exynos4210CmuState *s = opaque;
    uint64_t retval;

    if (offset > (EXYNOS4210_CMU_REGS_MEM_SIZE - sizeof(uint32_t))) {
        PRINT_ERROR("Bad offset: 0x%x\n", (int)offset);
        return 0;
    }

    if (offset & EXTENDED_REGION_MASK) {
        if (s->cmu_id == EXYNOS4210_CMU_DMC) {
            switch (offset & 0xFFF) {
            case DCGIDX_MAP0:
            case DCGIDX_MAP1:
            case DCGIDX_MAP2:
            case DCGPERF_MAP0:
            case DCGPERF_MAP1:
            case DVCIDX_MAP:
            case FREQ_CPU:
            case FREQ_DPM:
            case DVSEMCLK_EN:
            case MAXPERF:
                retval = s->reg[I_(offset)];
                break;
            default:
                PRINT_ERROR("Bad offset: 0x%x\n", (int)offset);
                retval = 0;
                break;
            }

            PRINT_DEBUG_EXTEND("<0x%05x> %s -> %08x\n", (unsigned int)offset,
                               exynos4210_cmu_regname(s, offset),
                               (unsigned int) retval);

            return retval;
        }

        if (s->cmu_id == EXYNOS4210_CMU_CPU) {
            switch (offset & 0xFFF) {
            case ARMCLK_STOPCTRL:
            case ATCLK_STOPCTRL:
            case PARITYFAIL_STATUS:
            case PARITYFAIL_CLEAR:
            case PWR_CTRL:
            case APLL_CON0_L8:
            case APLL_CON0_L7:
            case APLL_CON0_L6:
            case APLL_CON0_L5:
            case APLL_CON0_L4:
            case APLL_CON0_L3:
            case APLL_CON0_L2:
            case APLL_CON0_L1:
            case IEM_CONTROL:
            case APLL_CON1_L8:
            case APLL_CON1_L7:
            case APLL_CON1_L6:
            case APLL_CON1_L5:
            case APLL_CON1_L4:
            case APLL_CON1_L3:
            case APLL_CON1_L2:
            case APLL_CON1_L1:
            case CLKDIV_IEM_L8:
            case CLKDIV_IEM_L7:
            case CLKDIV_IEM_L6:
            case CLKDIV_IEM_L5:
            case CLKDIV_IEM_L4:
            case CLKDIV_IEM_L3:
            case CLKDIV_IEM_L2:
            case CLKDIV_IEM_L1:
                retval = s->reg[I_(offset)];
                break;
            default:
                PRINT_ERROR("Bad offset: 0x%x\n", (int)offset);
                retval = 0;
                break;
            }

            PRINT_DEBUG_EXTEND("<0x%05x> %s -> %08x\n", (unsigned int)offset,
                               exynos4210_cmu_regname(s, offset),
                               (unsigned int) retval);

            return retval;
        }
    }

    switch (offset & GROUP_MASK) {
    case PLL_LOCK:
    case PLL_CON:
    case CLK_SRC:
    case CLK_SRC_MASK:
    case CLK_MUX_STAT:
    case CLK_DIV:
    case CLK_DIV_STAT:
    case 0x700: /* Reserved */
    case CLK_GATE_SCLK: /* reserved? */
    case CLK_GATE_IP:
    case CLKOUT_CMU:
        retval = s->reg[I_(offset)];
        break;
    default:
        PRINT_ERROR("Bad offset: 0x%x\n", (int)offset);
        retval = 0;
        break;
    }

    PRINT_DEBUG_EXTEND("<0x%05x> %s -> %08x\n", (unsigned int)offset,
                       exynos4210_cmu_regname(s, offset), (unsigned int)retval);

    return retval;
}

static void exynos4210_cmu_write(void *opaque, hwaddr offset,
                                 uint64_t val, unsigned size)
{
    Exynos4210CmuState *s = opaque;
    uint32_t group, block;

    group = offset & GROUP_MASK;
    block = offset & BLOCK_MASK;

    switch (group) {
    case PLL_LOCK:
        /* TODO do something useful */
        break;
    case PLL_CON:
        switch (block) {
        case APLL:
        {
            uint32_t pre_val = s->reg[I_(offset)];
            s->reg[I_(offset)] = val;
            val = (val & ~PLL_LOCKED_MASK) | (pre_val & PLL_LOCKED_MASK);
            s->reg[I_(offset)] = val;
            break;
        }
        case MPLL:
        {
            uint32_t pre_val = s->reg[I_(offset)];
            s->reg[I_(offset)] = val;
            val = (val & ~PLL_LOCKED_MASK) | (pre_val & PLL_LOCKED_MASK);
            s->reg[I_(offset)] = val;
            break;
        }
        }
        break;
    case CLK_SRC:
        switch (block) {
        case CPU_BLK:
        {
            s->reg[I_(offset)] = val;

            if (val & MUX_APLL_SEL) {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(APLL_SEL_MASK)) |
                       (2 << APLL_SEL_SHIFT);
            } else {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(APLL_SEL_MASK)) |
                       (1 << APLL_SEL_SHIFT);
            }


            if (val & MUX_MPLL_SEL) {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(MPLL_SEL_MASK)) |
                       (2 << MPLL_SEL_SHIFT);
            } else {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(MPLL_SEL_MASK)) |
                       (1 << MPLL_SEL_SHIFT);
            }

            if (val & MUX_CORE_SEL) {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(CORE_SEL_MASK)) |
                       (2 << CORE_SEL_SHIFT);

            } else {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(CORE_SEL_MASK)) |
                        (1 << CORE_SEL_SHIFT);
            }

            if (val & MUX_HPM_SEL) {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                       (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(HPM_SEL_MASK)) |
                       (2 << HPM_SEL_SHIFT);
            } else {
                s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] =
                        (s->reg[I_(CLK_MUX_STAT_(CPU_BLK))] & ~(HPM_SEL_MASK)) |
                        (1 << HPM_SEL_SHIFT);
            }
        }
            break;
        case TOP0_BLK:
            s->reg[I_(offset)] = val;
            break;
        default:
            PRINT_ERROR("Unknown functional block: 0x%x\n", (int)block);
            break;
        }
        break;
    case CLK_SRC_MASK:
        break;
    case CLK_MUX_STAT:
        break;
    case CLK_DIV:
        switch (block) {
        case TOP_BLK:
            s->reg[I_(offset)] = val;
            break;
        case CPU0_BLK:
            s->reg[I_(offset)] = val;
            break;
        default:
            PRINT_ERROR("Unknown functional block: 0x%x\n", (int)block);
            break;
        }
        break;
    case CLK_DIV_STAT:
    case 0x700: /* Reserved */
    case CLK_GATE_SCLK: /* reserved? */
    case CLK_GATE_IP:
    case CLKOUT_CMU:
        break;
    default:
        PRINT_ERROR("Bad offset: 0x%x\n", (int)offset);
        break;
    }
}

static const MemoryRegionOps exynos4210_cmu_ops = {
    .read = exynos4210_cmu_read,
    .write = exynos4210_cmu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void exynos4210_cmu_reset(DeviceState *dev)
{
    Exynos4210CmuState *s = EXYNOS4210_CMU(dev);
    int i;
    uint32_t index = 0;

    for (i = 0; i < s->regs_number; i++) {
        index = (s->regs[i].offset) / sizeof(uint32_t);
        s->reg[index] = s->regs[i].reset_value;
    }

    PRINT_DEBUG("CMU %d reset completed\n", s->cmu_id);
}

static const VMStateDescription vmstate_exynos4210_cmu = {
    .name = TYPE_EXYNOS4210_CMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, Exynos4210CmuState,
                             EXYNOS4210_CMU_REGS_MEM_SIZE / sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    }
};

DeviceState *exynos4210_cmu_create(hwaddr addr, Exynos4210Cmu cmu_id)
{
    DeviceState  *dev;
    SysBusDevice *bus;

    dev = qdev_create(NULL, TYPE_EXYNOS4210_CMU);
    qdev_prop_set_int32(dev, "cmu_id", cmu_id);
    qdev_init_nofail(dev);
    bus = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(bus, 0, addr);

    return dev;
}

static int exynos4210_cmu_init(SysBusDevice *dev)
{
    Exynos4210CmuState *s = EXYNOS4210_CMU(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &exynos4210_cmu_ops, s,
                          TYPE_EXYNOS4210_CMU, EXYNOS4210_CMU_REGS_MEM_SIZE);
    sysbus_init_mmio(dev, &s->iomem);

    switch (s->cmu_id) {
    case EXYNOS4210_CMU_LEFTBUS:
        s->regs = exynos4210_cmu_leftbus_regs;
        s->regs_number = ARRAY_SIZE(exynos4210_cmu_leftbus_regs);
        break;
    case EXYNOS4210_CMU_RIGHTBUS:
        s->regs = exynos4210_cmu_rightbus_regs;
        s->regs_number = ARRAY_SIZE(exynos4210_cmu_rightbus_regs);
        break;
    case EXYNOS4210_CMU_TOP:
        s->regs = exynos4210_cmu_top_regs;
        s->regs_number = ARRAY_SIZE(exynos4210_cmu_top_regs);
        break;
    case EXYNOS4210_CMU_DMC:
        s->regs = exynos4210_cmu_dmc_regs;
        s->regs_number = ARRAY_SIZE(exynos4210_cmu_dmc_regs);
        break;
    case EXYNOS4210_CMU_CPU:
        s->regs = exynos4210_cmu_cpu_regs;
        s->regs_number = ARRAY_SIZE(exynos4210_cmu_cpu_regs);
        break;
    default:
        hw_error("Wrong CMU: %d\n", s->cmu_id);
    }

    return 0;
}

static Property exynos4210_cmu_properties[] = {
    DEFINE_PROP_INT32("cmu_id", Exynos4210CmuState, cmu_id, UNSPECIFIED_CMU),
    DEFINE_PROP_END_OF_LIST(),
};

static void exynos4210_cmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init   = exynos4210_cmu_init;
    dc->reset = exynos4210_cmu_reset;
    dc->props = exynos4210_cmu_properties;
    dc->vmsd  = &vmstate_exynos4210_cmu;
}

static const TypeInfo exynos4210_cmu_info = {
    .name          = TYPE_EXYNOS4210_CMU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Exynos4210CmuState),
    .class_init    = exynos4210_cmu_class_init,
};

static void exynos4210_cmu_register(void)
{
    type_register_static(&exynos4210_cmu_info);
}

type_init(exynos4210_cmu_register)
