/*
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#include <stdio.h>
#include "qemu-log.h"
#include "comet_spim.h"

#include "boards.h"

#define SPI_DEBUG_LEVEL 1
#if SPI_DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif
#if SPI_DEBUG_LEVEL >= 1
#  define ERRLOG(...) fprintf(stderr, ## __VA_ARGS__)
#else
#  define ERRLOG(...) do { } while (0)
#endif

/* register numbers (<<2 for offset) */
typedef enum {
    COMET_SPIM_REG0             = 0x00 >> 2,
    COMET_SPIM_REG1             = 0x04 >> 2,
    COMET_SPIM_REG2             = 0x08 >> 2,
    COMET_SPIM_REG3             = 0x0c >> 2,
    COMET_SPIM_REG4             = 0x10 >> 2,
    COMET_SPIM_SENDDATA         = 0x14 >> 2,
    COMET_SPIM_GETDATA          = 0x18 >> 2,
    COMET_SPIM_DMAREADINTSTAT   = 0x1c >> 2,
    COMET_SPIM_DMAREADINTEN     = 0x20 >> 2,
    COMET_SPIM_DMAREADINTCL     = 0x24 >> 2,
    COMET_SPIM_DMAWRITEINTSTAT  = 0x28 >> 2,
    COMET_SPIM_DMAWRITEINTEN    = 0x2c >> 2,
    COMET_SPIM_DMAWRITEINTCL    = 0x30 >> 2,
    COMET_SPIM_DIAGNOSTIC       = 0x34 >> 2,
    COMET_SPIM_CMPDATACTRL      = 0x38 >> 2,
} CometSpiReg;

static const char *comet_spim_reg_names[] = {
    [COMET_SPIM_REG0]            = "REG0",
    [COMET_SPIM_REG1]            = "REG1",
    [COMET_SPIM_REG2]            = "REG2",
    [COMET_SPIM_REG3]            = "REG3",
    [COMET_SPIM_REG4]            = "REG4",
    [COMET_SPIM_SENDDATA]        = "SEND_DATA",
    [COMET_SPIM_GETDATA]         = "GET_DATA",
    [COMET_SPIM_DMAREADINTSTAT]  = "DMA_READ_INT_STAT",
    [COMET_SPIM_DMAREADINTEN]    = "DMA_READ_INT_EN",
    [COMET_SPIM_DMAREADINTCL]    = "DMA_READ_INT_CL",
    [COMET_SPIM_DMAWRITEINTSTAT] = "DMA_WRITE_INT_STAT",
    [COMET_SPIM_DMAWRITEINTEN]   = "DMA_WRITE_INT_EN",
    [COMET_SPIM_DMAWRITEINTCL]   = "DMA_WRITE_INT_CL",
    [COMET_SPIM_DIAGNOSTIC]      = "DIAGNOSTIC",
    [COMET_SPIM_CMPDATACTRL]     = "CMP_DATA_CTRL",
};

#define COMET_SPIM_REG0_MASK            0xffffffff
#define COMET_SPIM_REG1_MASK            0xffffffff
#define COMET_SPIM_REG2_MASK            0xffffffff

#define COMET_SPIM_REG3_CONT_MASK       (1 << 27)
#define COMET_SPIM_REG3_SRESET_MASK     (1 << 26)
#define COMET_SPIM_REG3_GETDMA_MASK     (1 << 25)
#define COMET_SPIM_REG3_SENDDMA_MASK    (1 << 24)
#define COMET_SPIM_REG3_CS_MASK         0x00030000
#define COMET_SPIM_REG3_CS_SHIFT        16
#define COMET_SPIM_REG3_XFERSIZE_MASK   0x00000fff
#define COMET_SPIM_REG3_MASK            0x3f030fff
#define COMET_SPIM_REG3_STORE_MASK      0x34000000

#define COMET_SPIM_REG4_MASK            0x00000fff

#define COMET_SPIM_DMA_READ_INT_MASK    0xf
#define COMET_SPIM_DMA_WRITE_INT_MASK   0x1f
#define COMET_SPIM_DMA_INT_ALLDONETRIG  (1 << 4)
#define COMET_SPIM_DMA_INT_FUL          (1 << 3)
#define COMET_SPIM_DMA_INT_HF           (1 << 2)
#define COMET_SPIM_DMA_INT_EX           (1 << 1)
#define COMET_SPIM_DMA_INT_TRIG         (1 << 0)

static void comet_spim_update_interrupts(struct comet_spim_state_s *spi)
{
    bool high = !!((spi->dma_write_int_stat & spi->dma_write_int_en)
                || (spi->dma_read_int_stat & spi->dma_read_int_en));

    if (high) {
        qemu_irq_raise(spi->irq);
    } else {
        qemu_irq_lower(spi->irq);
    }
}

static int comet_spim_idle(struct comet_spim_state_s *spi)
{
    return !spi->cur_dma.send_count && !spi->cur_dma.get_count && !spi->recv_bufcount;
}

static int comet_spim_send_pending(struct comet_spim_state_s *spi)
{
    return spi->next_dma.send_count && spi->next_dma.send;
}

static int comet_spim_get_pending(struct comet_spim_state_s *spi)
{
    return spi->next_dma.send_count && spi->next_dma.get;
}

static void comet_spim_update_dreq(struct comet_spim_state_s *spi)
{
    struct comet_spim_dma *dma = &spi->cur_dma;
    bool txdrq, rxdrq;

    txdrq = dma->send && dma->send_count && (spi->send_bufcount < sizeof(spi->send_buf));
    rxdrq = dma->get && spi->recv_bufcount;

    DBGLOG("spi txdrq=%d (s=%d:%d:%d) rxdrq=%d (g=%d:%d:%d)\n",
           txdrq, dma->send, dma->send_count, spi->send_bufcount < sizeof(spi->send_buf),
           rxdrq, dma->get, dma->get_count, !!spi->recv_bufcount);

    qemu_set_irq(spi->txdrq, txdrq);
    qemu_set_irq(spi->rxdrq, rxdrq);
}

static void comet_spim_clear_dreq(struct comet_spim_state_s *spi)
{
    qemu_irq_lower(spi->txdrq);
    qemu_irq_lower(spi->rxdrq);
}

static struct comet_spim_device_s *
comet_spim_get_device(struct comet_spim_state_s *spi)
{
    if (spi->cur_dma.cs < 0 || spi->cur_dma.cs >= ARRAY_SIZE(spi->devices)) {
        return NULL;
    }
    if (!spi->devices[spi->cur_dma.cs].rxtx) {
        return NULL;
    }
    if (!spi->devices[spi->cur_dma.cs].setcs) {
        return NULL;
    }
    return &spi->devices[spi->cur_dma.cs];
}

static void comet_spim_terminate_transaction(struct comet_spim_state_s *spi)
{
    struct comet_spim_device_s *dev;
    bool get = spi->cur_dma.get;
    bool send = spi->cur_dma.send;

    /* only terminate the transaction once */
    if (!send && !get) {
        return;
    }
    spi->cur_dma.send = 0;
    spi->cur_dma.get = 0;

    /* inform the device of the termination */
    dev = comet_spim_get_device(spi);
    if (dev) {
        dev->setcs(dev->opaque, false);
    }

    /* ready for a new chip select now */
    if (get) {
        spi->dma_write_int_stat |= COMET_SPIM_DMA_INT_ALLDONETRIG;
        comet_spim_update_interrupts(spi);
    }
}

static void comet_spim_dma_start(struct comet_spim_state_s *spi)
{
    /* changing the chip select ends the transaction */
    if (spi->cur_dma.cs != spi->next_dma.cs) {
        comet_spim_terminate_transaction(spi);
    }

    /* the next dma info is now the current dma */
    spi->cur_dma = spi->next_dma;
    /* move count from send_count into send_count/get_count */
    if (spi->cur_dma.get) {
        spi->cur_dma.get_count = spi->cur_dma.send_count;
    }
    if (!spi->cur_dma.send) {
        spi->cur_dma.send_count = 0;
    }
    /* next_dma send/get cleared so we don't keep requesting dma */
    spi->next_dma.send = 0;
    spi->next_dma.get = 0;
    comet_spim_update_dreq(spi);
}

static void comet_spim_dma_complete(struct comet_spim_state_s *spi)
{
    bool get = spi->cur_dma.get;
    bool send = spi->cur_dma.send;

    /* terminate transaction if not continuing */
    if (!spi->cur_dma.cont) {
        comet_spim_terminate_transaction(spi);
    }

    /* ready for another dma now */
    if (send)
        spi->dma_read_int_stat |= COMET_SPIM_DMA_INT_TRIG;
    if (get)
        spi->dma_write_int_stat |= COMET_SPIM_DMA_INT_TRIG;
    comet_spim_update_interrupts(spi);
}

static size_t comet_spim_trans_size(struct comet_spim_state_s *spi)
{
    size_t ravail, wavail, sz;

    sz = (size_t)~0;

    if (spi->cur_dma.get) {
        ravail = MIN(spi->cur_dma.get_count,
                     sizeof(spi->recv_buf) - spi->recv_bufcount);
        sz = MIN(sz, ravail);
    }

    if (spi->cur_dma.send) {
        wavail = MIN(spi->cur_dma.send_count, spi->send_bufcount);
        sz = MIN(sz, wavail);
    }

    return sz;
}

static void comet_spim_data_pump(struct comet_spim_state_s *spi)
{
    uint8_t rdat[sizeof(spi->recv_buf)];
    uint8_t wdat[sizeof(spi->send_buf)];
    struct comet_spim_device_s *dev;
    size_t sz, cp, xfered = 0;

    if (comet_spim_idle(spi)) {
        if ((comet_spim_send_pending(spi) || comet_spim_get_pending(spi))) {
            /* start dma if currently idle and pending */
            comet_spim_dma_start(spi);
        } else {
            /* there's nothing to do */
            return;
        }
    }

    while ((spi->cur_dma.get || spi->cur_dma.send) && (sz = comet_spim_trans_size(spi))) {
        DBGLOG("spi pump %d s=%d g=%d\n", sz, spi->cur_dma.send, spi->cur_dma.get);
        dev = comet_spim_get_device(spi);

        /* setup send data */
        if (spi->cur_dma.send) {
            cp = MIN(sz, sizeof(spi->send_buf) - spi->send_bufridx);
            memcpy(wdat, spi->send_buf + spi->send_bufridx, cp);
            if (cp < sz) {
                /* wrapped around the buffer */
                memcpy(wdat + cp, spi->send_buf, sz - cp);
            }
            spi->send_bufridx += sz;
            spi->send_bufridx %= sizeof(spi->send_buf);
            spi->send_bufcount -= sz;
            spi->cur_dma.send_count -= sz;
        } else {
            memset(wdat, 0xff, sz);
        }

        /* setup receive data */
        memset(rdat, 0xff, sz);

        /* perform transfer */
        if (dev) {
            dev->setcs(dev->opaque, true);
            dev->rxtx(dev->opaque, sz, wdat, rdat);
        }
        xfered += sz;

        /* buffer read data */
        if (spi->cur_dma.get) {
            cp = MIN(sz, sizeof(spi->recv_buf) - spi->recv_bufwidx);
            memcpy(spi->recv_buf + spi->recv_bufwidx, rdat, cp);
            if (cp < sz) {
                /* wrapped around the buffer */
                memcpy(spi->recv_buf, rdat + cp, sz - cp);
            }
            spi->recv_bufwidx += sz;
            spi->recv_bufwidx %= sizeof(spi->recv_buf);
            spi->recv_bufcount += sz;
            spi->cur_dma.get_count -= sz;

            /* something is now (momentarily) in fifo */
            spi->dma_write_int_stat |= COMET_SPIM_DMA_INT_EX;
            comet_spim_update_interrupts(spi);
        }
    }

    comet_spim_update_dreq(spi);

    /* complete dma if now idle */
    if (xfered && comet_spim_idle(spi))
        comet_spim_dma_complete(spi);
}

static void comet_spim_send_data(struct comet_spim_state_s *spi, uint8_t val)
{
    /* write to the buffer */
    spi->send_buf[spi->send_bufwidx++] = val;
    spi->send_bufwidx %= sizeof(spi->send_buf);
    spi->send_bufcount++;
    /* something is now in fifo */
    spi->dma_read_int_stat |= COMET_SPIM_DMA_INT_EX;
    comet_spim_update_interrupts(spi);
    /* pump out the data if possible */
    comet_spim_data_pump(spi);
}

static uint8_t comet_spim_get_data(struct comet_spim_state_s *spi)
{
    uint8_t buf = 0xff;

    if (!spi->recv_bufcount) {
        comet_spim_data_pump(spi);
    }

    if (spi->recv_bufcount) {
        buf = spi->recv_buf[spi->recv_bufridx++];
        spi->recv_bufridx %= sizeof(spi->recv_buf);
        spi->recv_bufcount--;
        comet_spim_update_dreq(spi);
        if (comet_spim_idle(spi))
            comet_spim_dma_complete(spi);
    } else {
        ERRLOG("SPI has no data for get_data\n");
    }

    return buf;
}

static uint64_t comet_spim_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    struct comet_spim_state_s *spi = opaque;
    unsigned int offset;
    uint32_t ret = 0;

    addr &= 0xff;
    offset = addr >> 2;

    if (unlikely(addr & 0x3)) {
        goto invalid;
    }
    if (unlikely(offset >= ARRAY_SIZE(comet_spim_reg_names))) {
        goto invalid;
    }
    if (unlikely(!comet_spim_reg_names[offset])) {
        goto invalid;
    }

    switch (offset) {
    case COMET_SPIM_REG0:
    case COMET_SPIM_REG1:
    case COMET_SPIM_REG2:
    case COMET_SPIM_REG4:
        ret = spi->reg[offset];
        break;
    case COMET_SPIM_REG3:
        ret = spi->reg[3];
        if (spi->next_dma.send) {
            ret |= COMET_SPIM_REG3_SENDDMA_MASK;
        }
        if (spi->next_dma.get) {
            ret |= COMET_SPIM_REG3_GETDMA_MASK;
        }
        if (spi->next_dma.cont) {
            ret |= COMET_SPIM_REG3_CONT_MASK;
        }
        ret |= spi->next_dma.cs << COMET_SPIM_REG3_CS_SHIFT;
        ret |= spi->next_dma.send_count;
        break;
    case COMET_SPIM_GETDATA:
        ret = comet_spim_get_data(spi);
        break;
    case COMET_SPIM_DMAREADINTSTAT:
        ret = spi->dma_read_int_stat;
        break;
    case COMET_SPIM_DMAREADINTEN:
        ret = spi->dma_read_int_en;
        break;
    case COMET_SPIM_DMAWRITEINTSTAT:
        ret = spi->dma_write_int_stat;
        break;
    case COMET_SPIM_DMAWRITEINTEN:
        ret = spi->dma_write_int_en;
        break;
    default:
        DBGLOG("unhandled ");
    }

    DBGLOG("spi read(%s @ 0x%02x) = 0x%08x\n",
           comet_spim_reg_names[offset], addr, ret);
    return ret;

invalid:
    ERRLOG("unhandled spi read(0x%02" HWADDR_PRIx ") = 0x%08x\n", addr, ret);
    return ret;
}

static void comet_spim_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    struct comet_spim_state_s *spi = opaque;
    unsigned int offset;

    addr &= 0xff;
    offset = addr >> 2;

    if (unlikely(addr & 0x3)) {
        goto invalid;
    }
    if (unlikely(offset >= ARRAY_SIZE(comet_spim_reg_names))) {
        goto invalid;
    }
    if (unlikely(!comet_spim_reg_names[offset])) {
        goto invalid;
    }

    switch (offset) {
    case COMET_SPIM_REG0:
        spi->reg[0] = val & COMET_SPIM_REG0_MASK;
        break;
    case COMET_SPIM_REG1:
        spi->reg[1] = val & COMET_SPIM_REG1_MASK;
        break;
    case COMET_SPIM_REG2:
        spi->reg[2] = val & COMET_SPIM_REG1_MASK;
        break;
    case COMET_SPIM_REG3:
        spi->reg[3] = val & COMET_SPIM_REG3_STORE_MASK;
        spi->next_dma.send_count = val & COMET_SPIM_REG3_XFERSIZE_MASK;
        if (!spi->next_dma.send_count) {
            spi->next_dma.send_count = 0x1000;
        }
        spi->next_dma.send = !!(val & COMET_SPIM_REG3_SENDDMA_MASK);
        spi->next_dma.get = !!(val & COMET_SPIM_REG3_GETDMA_MASK);
        spi->next_dma.cs = (val & COMET_SPIM_REG3_CS_MASK)
                                >> COMET_SPIM_REG3_CS_SHIFT;
        if (spi->next_dma.cs == 0x3) {
            spi->next_dma.cs = -1;
        }
        spi->next_dma.cont = !!(val & COMET_SPIM_REG3_CONT_MASK);
        DBGLOG("spi next DMA %d bytes g=%d s=%d cont=%d\n",
               spi->next_dma.send_count, spi->next_dma.get, spi->next_dma.send, spi->next_dma.cont);

        if (spi->reg[3] & COMET_SPIM_REG3_SRESET_MASK) {
            comet_spim_terminate_transaction(spi);
        }
        comet_spim_data_pump(spi);
        break;
    case COMET_SPIM_REG4:
        spi->reg[4] = val & COMET_SPIM_REG1_MASK;
        break;
    case COMET_SPIM_SENDDATA:
        comet_spim_send_data(spi, val);
        break;
    case COMET_SPIM_DMAREADINTEN:
        spi->dma_read_int_en = val & COMET_SPIM_DMA_READ_INT_MASK;
        comet_spim_update_interrupts(spi);
        break;
    case COMET_SPIM_DMAREADINTCL:
        spi->dma_read_int_stat &= ~val;
        comet_spim_update_interrupts(spi);
        break;
    case COMET_SPIM_DMAWRITEINTEN:
        spi->dma_write_int_en = val & COMET_SPIM_DMA_WRITE_INT_MASK;
        comet_spim_update_interrupts(spi);
        break;
    case COMET_SPIM_DMAWRITEINTCL:
        spi->dma_write_int_stat &= ~val;
        comet_spim_update_interrupts(spi);
        break;
    default:
        DBGLOG("unhandled ");
    }

    DBGLOG("spi write(%s @ 0x%02x, 0x%08x)\n",
           comet_spim_reg_names[offset], addr, val);
    return;

invalid:
    ERRLOG("spi write(0x%02" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    return;
}

static const MemoryRegionOps comet_spim_io_ops = {
    .read = comet_spim_io_read,
    .write = comet_spim_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void comet_spim_device_init(struct comet_spim_state_s *spi, int cs,
                            comet_spim_device_rxtx rxtx,
                            comet_spim_device_setcs setcs, void *opaque)
{
    if (cs < 0 || cs >= ARRAY_SIZE(spi->devices)) {
        return;
    }

    spi->devices[cs].rxtx = rxtx;
    spi->devices[cs].setcs = setcs;
    spi->devices[cs].opaque = opaque;
}

struct comet_spim_state_s *comet_spim_init(qemu_irq irq, qemu_irq txdrq,
                                           qemu_irq rxdrq)
{
    struct comet_spim_state_s *spi;

    spi = g_malloc0(sizeof(struct comet_spim_state_s));
    spi->reg[0] = 0x200c0c18; /* flash chip */
    spi->reg[1] = 0x200c0c18; /* flash chip */
    spi->reg[2] = 0x80010102; /* SD card */
    spi->reg[4] = 0x00010fff;

    memory_region_init_io(&spi->iomem, &comet_spim_io_ops, spi,
                          "comet-spim", COMET_SPIM_SIZE - 1);
    spi->irq = irq;
    spi->txdrq = txdrq;
    spi->rxdrq = rxdrq;

    comet_spim_reset(spi);
    return spi;
}

void comet_spim_reset(struct comet_spim_state_s *spi)
{
    comet_spim_terminate_transaction(spi);

    memset(&spi->next_dma, 0, sizeof(spi->next_dma));
    memset(&spi->cur_dma, 0, sizeof(spi->cur_dma));
    spi->dma_read_int_stat = COMET_SPIM_DMA_INT_TRIG;
    spi->dma_read_int_en = 0;
    spi->dma_write_int_stat = COMET_SPIM_DMA_INT_ALLDONETRIG
                            | COMET_SPIM_DMA_INT_TRIG;
    spi->dma_write_int_en = 0;
    memset(spi->reg, 0, sizeof(spi->reg));

    comet_spim_clear_dreq(spi);
}
