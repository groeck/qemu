/*
 * Frontier Silicon Chorus2 SPI controller.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#ifndef hw_chorus2_spi_h
# define hw_chorsu2_spi_h "chorus2_spi.h"

#include <inttypes.h>
#include "qemu-common.h"
#include "memory.h"
#include "irq.h"

#define C2_SPI_SIZE         0x1000

typedef void (*chorus2_spi_device_rxtx)(void *opaque, size_t len,
                                        uint8_t *rx, uint8_t *tx);

typedef void (*chorus2_spi_device_setcs)(void *opaque, unsigned int cs);

struct chorus2_spi_state_s {
    MemoryRegion iomem;

    qemu_irq txdrq;
    qemu_irq rxdrq;

    /*
     * next_dma is current register values
     * cur_dma is registers for dma in progress
     * idle if cur_dma.{send,get}_count == 0
     */
    struct chorus2_spi_dma {
        uint16_t send_count;
        uint16_t get_count;
        int8_t send, get, cs, cont;
    } next_dma, cur_dma;

    uint8_t dma_read_int_stat;
    uint8_t dma_read_int_en;
    uint8_t dma_write_int_stat;
    uint8_t dma_write_int_en;
    uint32_t reg[5];

    struct chorus2_spi_device_s {
        chorus2_spi_device_rxtx rxtx;
        chorus2_spi_device_setcs setcs;
        void *opaque;
    } devices[3];
};

void chorus2_spi_device_init(struct chorus2_spi_state_s *spi, int cs,
                             chorus2_spi_device_rxtx rxtx,
                             chorus2_spi_device_setcs setcs, void *opaque);
struct chorus2_spi_state_s *chorus2_spi_init(qemu_irq txdrq, qemu_irq rxdrq);
void chorus2_spi_reset(struct chorus2_spi_state_s *spi);

#endif
