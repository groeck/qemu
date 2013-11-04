/*
 * Toumaz Comet SoC with META21.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#ifndef hw_comet_h
# define hw_comet_h "comet.h"

#include "comet_regs.h"
#include "qemu-common.h"
#include "memory.h"
#include "img_mdc.h"
#include "img_scb.h"
#include "comet_spim.h"
#include "comet_pdp.h"
#include "meta_boot.h"
#include "dw_otg.h"
#include "dw_serial.h"

#define COMET_DEFAULT_RESET_CFG 0xf0000390

#define COMET_MDC_CHANNELS 8

typedef enum {
    COMET_DMACHANSEL_NOTACTIVE,
    COMET_DMACHANSEL_SDIO_WRITE,
    COMET_DMACHANSEL_SDIO_READ,
    COMET_DMACHANSEL_SPIMASTER_WRITE,
    COMET_DMACHANSEL_SPIMASTER_READ,
    COMET_DMACHANSEL_SPISLAVE_WRITE,
    COMET_DMACHANSEL_SPISLAVE_READ,
    COMET_DMACHANSEL_SPIMASTER_WRITE2,
    COMET_DMACHANSEL_SPIMASTER_READ2,
    COMET_DMACHANSEL_I2S_WRITE,
    COMET_DMACHANSEL_I2S_READ,
    COMET_DMACHANSEL_LCD,
    COMET_DMACHANSEL_SDHOST_WRITE,
    COMET_DMACHANSEL_SDHOST_READ,
    COMET_DMACHANSEL_MAX,
} CometDmaChanSel;

#define COMET_NUM_GPIOS 90

typedef enum {
    /* GPIO0 */
    COMET_GPIO_SPI1_CS2  = 17,
    COMET_GPIO_UART0_CTS = 22,
} CometGpio;

struct dw_mmc_state;
struct comet_top_state;

struct comet_state_s {
    MetaCore *core;
    struct MetaBootInfo boot;
    struct comet_top_state *top;

    struct img_mdc_state_s *mdc;
    struct img_scb_state_s *scb2;
    struct comet_spim_state_s *spim1;
    DwSerialState *uart0, *uart1;
    comet_pdp_state *pdp;
    struct dw_mmc_state *mmc;
    struct dw_otg_state *usb;

    qemu_irq *drq;

    qemu_irq *gpios;
    uint32_t gpio_dir[3];
    uint32_t gpio_select[3];
    uint32_t gpio_en[3];
    uint32_t gpio_din[3];
    uint32_t gpio_dout[3];
    uint32_t irq_plrt[3];
    uint32_t irq_type[3];
    uint32_t irq_en[3];
    uint32_t irq_sts[3];
    /* input into irq trigger logic */
    uint32_t irq_input[3];

    /* registers */
    uint32_t reset_cfg;

    MemoryRegion rom_iomem;
    MemoryRegion core_code_iomem[512];
    MemoryRegion core_code_slave_iomem;
    MemoryRegion core_data_iomem[512];
    MemoryRegion core_data_slave_iomem;
    MemoryRegion internal_iomem;
    MemoryRegion sdram_iomem;
    MemoryRegion ucc0_mtxsram_iomem;
    MemoryRegion ucc0_mcpsram16_iomem;
    MemoryRegion ucc0_mcpsram24_iomem;
    MemoryRegion ucc1_mtxsram_iomem;

    MemoryRegion peripherals_iomem;
    MemoryRegion sysctrl_iomem;
    MemoryRegion gpios_iomem;
    MemoryRegion hep_iomem;
    MemoryRegion sys_inf_iomem;
    MemoryRegion sys_event_iomem;
    MemoryRegion ucc0_iomem;
    MemoryRegion ucc1_iomem;
    MemoryRegion i2s_iomem;

    MemoryRegion pdi_iomem;
};

struct comet_state_s *comet_init(unsigned long sdram_size,
                                 const char *core,
                                 const char *kernel_filename,
                                 const char *kernel_cmdline);

#endif
