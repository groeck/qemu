/*
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#ifndef hw_comet_spim_h
# define hw_comet_spim_h "comet_spi.h"

#include <inttypes.h>
#include "qemu-common.h"
#include "memory.h"
#include "irq.h"

#define COMET_SPIM_SIZE              0x100

typedef void (*comet_spim_device_rxtx)(void *opaque, size_t len,
                                       uint8_t *rx, uint8_t *tx);

typedef void (*comet_spim_device_setcs)(void *opaque, unsigned int cs);

struct comet_spim_state_s {
    MemoryRegion iomem;

    qemu_irq irq;
    qemu_irq txdrq;
    qemu_irq rxdrq;

    /*
     * next_dma is current register values
     * cur_dma is registers for dma in progress
     * idle if cur_dma.{send,get}_count == 0
     */
    struct comet_spim_dma {
        uint16_t send_count;
        uint16_t get_count;
        int8_t send, get, cs, cont;
    } next_dma, cur_dma;

    uint8_t send_buf[32];
    size_t send_bufridx, send_bufwidx, send_bufcount;

    uint8_t recv_buf[32];
    size_t recv_bufridx, recv_bufwidx, recv_bufcount;

    uint8_t dma_read_int_stat;
    uint8_t dma_read_int_en;
    uint8_t dma_write_int_stat;
    uint8_t dma_write_int_en;
    uint32_t reg[5];

    struct comet_spim_device_s {
        comet_spim_device_rxtx rxtx;
        comet_spim_device_setcs setcs;
        void *opaque;
    } devices[3];
};

void comet_spim_device_init(struct comet_spim_state_s *spi, int cs,
                            comet_spim_device_rxtx rxtx,
                            comet_spim_device_setcs setcs, void *opaque);
struct comet_spim_state_s *comet_spim_init(qemu_irq irq, qemu_irq txdrq,
                                           qemu_irq rxdrq);
void comet_spim_reset(struct comet_spim_state_s *spi);

#endif
