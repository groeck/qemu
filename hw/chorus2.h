/*
 * Frontier Silicon Chorus2 SoC with META122.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#ifndef hw_chorus2_h
# define hw_chorsu2_h "chorus2.h"

#include "chorus2_regs.h"
#include "qemu-common.h"
#include "meta_boot.h"

#define C2_DMAC_CHANNELS    12

struct chorus2_dmac_state_s;
struct chorus2_spi_state_s;

typedef enum {
    C2_DMACHANSEL_LOCALBUS,     /* 0x00 */
    C2_DMACHANSEL_REEDSOLOMONI,
    C2_DMACHANSEL_REEDSOLOMONO,
    C2_DMACHANSEL_MEMSTICK,
    C2_DMACHANSEL_4,            /* 0x04 */
    C2_DMACHANSEL_SCPI,
    C2_DMACHANSEL_SCPO,
    C2_DMACHANSEL_7,
    C2_DMACHANSEL_ECPO,         /* 0x08 */
    C2_DMACHANSEL_ATAPI,
    C2_DMACHANSEL_SPI1O,
    C2_DMACHANSEL_SPI1I,
    C2_DMACHANSEL_SPI2O,        /* 0x0C */
    C2_DMACHANSEL_SPI2I,
    C2_DMACHANSEL_NOISESHAPER,
    C2_DMACHANSEL_LCD,
    C2_DMACHANSEL_I2S1O,        /* 0x10 */
    C2_DMACHANSEL_I2S1I,
    C2_DMACHANSEL_SPDIFO,
    C2_DMACHANSEL_SPDIFI,
    C2_DMACHANSEL_I2S2O,        /* 0x14 */
    C2_DMACHANSEL_CORESCP,
    C2_DMACHANSEL_CORELOCAL,
    C2_DMACHANSEL_CORE,
    C2_DMACHANSEL_RDI,          /* 0x18 */
    C2_DMACHANSEL_NAND,
    C2_DMACHANSEL_MAX,
} Chorus2DmaChanSel;

struct chorus2_state_s {
    MetaCore *core;
    struct MetaBootInfo boot;
    struct chorus2_dmac_state_s *dmac;
    struct chorus2_spi_state_s *spi;

    qemu_irq *drq;

    /* registers */
    uint32_t revision;
    uint32_t boot_setup;
    uint16_t sw_reset_prot;
    uint8_t  sw_reset;
    Chorus2DmaChanSel dma_chan_sel[C2_DMAC_CHANNELS];
};

struct chorus2_state_s *chorus2_init(unsigned long sdram_size,
                                     const char *core,
                                     const char *kernel_filename,
                                     const char *kernel_cmdline);

#endif
