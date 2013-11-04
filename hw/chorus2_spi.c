/*
 * Frontier Silicon Chorus2 SPI controller.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#include "chorus2_spi.h"

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
    C2_SPI_REG0             = 0x000 >> 2,
    C2_SPI_REG1             = 0x004 >> 2,
    C2_SPI_REG2             = 0x008 >> 2,
    C2_SPI_REG3             = 0x00c >> 2,
    C2_SPI_REG4             = 0x010 >> 2,
    C2_SPI_SENDDATA         = 0x014 >> 2,
    C2_SPI_GETDATA          = 0x018 >> 2,
    C2_SPI_DMAREADINTSTAT   = 0x01c >> 2,
    C2_SPI_DMAREADINTEN     = 0x020 >> 2,
    C2_SPI_DMAREADINTCL     = 0x024 >> 2,
    C2_SPI_DMAWRITEINTSTAT  = 0x028 >> 2,
    C2_SPI_DMAWRITEINTEN    = 0x02c >> 2,
    C2_SPI_DMAWRITEINTCL    = 0x030 >> 2,
    C2_SPI_DIAGNOSTIC       = 0x034 >> 2,
    C2_SPI_CMPDATACTRL      = 0x038 >> 2,
} Chorus2SpiReg;

static const char *chorus2_spi_reg_names[] = {
    [C2_SPI_REG0]            = "REG0",
    [C2_SPI_REG1]            = "REG1",
    [C2_SPI_REG2]            = "REG2",
    [C2_SPI_REG3]            = "REG3",
    [C2_SPI_REG4]            = "REG4",
    [C2_SPI_SENDDATA]        = "SEND_DATA",
    [C2_SPI_GETDATA]         = "GET_DATA",
    [C2_SPI_DMAREADINTSTAT]  = "DMA_READ_INT_STAT",
    [C2_SPI_DMAREADINTEN]    = "DMA_READ_INT_EN",
    [C2_SPI_DMAREADINTCL]    = "DMA_READ_INT_CL",
    [C2_SPI_DMAWRITEINTSTAT] = "DMA_WRITE_INT_STAT",
    [C2_SPI_DMAWRITEINTEN]   = "DMA_WRITE_INT_EN",
    [C2_SPI_DMAWRITEINTCL]   = "DMA_WRITE_INT_CL",
    [C2_SPI_DIAGNOSTIC]      = "DIAGNOSTIC",
    [C2_SPI_CMPDATACTRL]     = "CMP_DATA_CTRL",
};

#define C2_SPI_REG0_MASK            0xffffffff
#define C2_SPI_REG1_MASK            0xffffffff
#define C2_SPI_REG2_MASK            0xffffffff

#define C2_SPI_REG3_CONT_MASK       (1 << 27)
#define C2_SPI_REG3_SRESET_MASK     (1 << 26)
#define C2_SPI_REG3_GETDMA_MASK     (1 << 25)
#define C2_SPI_REG3_SENDDMA_MASK    (1 << 24)
#define C2_SPI_REG3_CS_MASK         0x00030000
#define C2_SPI_REG3_CS_SHIFT        16
#define C2_SPI_REG3_XFERSIZE_MASK   0x00000fff
#define C2_SPI_REG3_MASK            0x3f030fff
#define C2_SPI_REG3_STORE_MASK      0x34000000

#define C2_SPI_REG4_MASK            0x00000fff

#define C2_SPI_DMA_READ_INT_MASK    0xf
#define C2_SPI_DMA_WRITE_INT_MASK   0x1f
#define C2_SPI_DMA_INT_ALLDONETRIG  (1 << 4)
#define C2_SPI_DMA_INT_FUL          (1 << 3)
#define C2_SPI_DMA_INT_HF           (1 << 2)
#define C2_SPI_DMA_INT_EX           (1 << 1)
#define C2_SPI_DMA_INT_TRIG         (1 << 0)

static void chorus2_spi_update_interrupts(struct chorus2_spi_state_s *spi)
{
}

static int chorus2_spi_idle(struct chorus2_spi_state_s *spi)
{
    return !spi->cur_dma.send_count && !spi->cur_dma.get_count;
}

static int chorus2_spi_send_pending(struct chorus2_spi_state_s *spi)
{
    return spi->next_dma.send_count && spi->next_dma.send;
}

static int chorus2_spi_get_pending(struct chorus2_spi_state_s *spi)
{
    return spi->next_dma.send_count && spi->next_dma.get;
}

static void chorus2_spi_update_dreq(struct chorus2_spi_state_s *spi)
{
    if (chorus2_spi_idle(spi)) {
        /* idle, so request depends on SENDDMA and GETDMA bits */
        qemu_set_irq(spi->txdrq, spi->next_dma.send);
        qemu_set_irq(spi->rxdrq, spi->next_dma.get);
        if (spi->next_dma.get) {
            /* something is now (momentarily) in fifo */
            spi->dma_write_int_stat |= C2_SPI_DMA_INT_EX;
            chorus2_spi_update_interrupts(spi);
        }
    } else {
        /* busy, so no current dma request */
        qemu_irq_lower(spi->txdrq);
        qemu_irq_lower(spi->rxdrq);
    }
}

static struct chorus2_spi_device_s *
chorus2_spi_get_device(struct chorus2_spi_state_s *spi)
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

static void chorus2_spi_terminate_transaction(struct chorus2_spi_state_s *spi)
{
    struct chorus2_spi_device_s *dev;

    /* only terminate the transaction once */
    if (!spi->cur_dma.send && !spi->cur_dma.get) {
        return;
    }
    spi->cur_dma.send = 0;
    spi->cur_dma.get = 0;

    /* inform the device of the termination */
    dev = chorus2_spi_get_device(spi);
    if (dev) {
        dev->setcs(dev->opaque, false);
    }

    /* ready for a new chip select now */
    spi->dma_write_int_stat |= C2_SPI_DMA_INT_ALLDONETRIG;
    chorus2_spi_update_interrupts(spi);
}

static void chorus2_spi_dma_start(struct chorus2_spi_state_s *spi)
{
    /* changing the chip select ends the transaction */
    if (spi->cur_dma.cs != spi->next_dma.cs) {
        chorus2_spi_terminate_transaction(spi);
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
    chorus2_spi_update_dreq(spi);
}

static void chorus2_spi_dma_complete(struct chorus2_spi_state_s *spi)
{
    /* terminate transaction if not continuing */
    if (!spi->cur_dma.cont) {
        chorus2_spi_terminate_transaction(spi);
    }

    /* start next dma if one in queue */
    chorus2_spi_update_dreq(spi);

    /* ready for another dma now */
    spi->dma_read_int_stat |= C2_SPI_DMA_INT_TRIG;
    spi->dma_write_int_stat |= C2_SPI_DMA_INT_TRIG;
    chorus2_spi_update_interrupts(spi);
}

static void chorus2_spi_send_data(struct chorus2_spi_state_s *spi, uint8_t val)
{
    struct chorus2_spi_device_s *dev;
    uint8_t rxbuf = 0xff;

    /* start dma if currently idle and pending */
    if (chorus2_spi_send_pending(spi) && chorus2_spi_idle(spi)) {
        chorus2_spi_dma_start(spi);
    }
    if (spi->cur_dma.send_count) {
        --spi->cur_dma.send_count;
        /* write a byte to the device */
        dev = chorus2_spi_get_device(spi);
        if (dev) {
            dev->rxtx(dev->opaque, 1, &val, &rxbuf);
        }
        /* complete dma if now idle */
        if (chorus2_spi_idle(spi)) {
            chorus2_spi_dma_complete(spi);
        }
    } else {
        ERRLOG("SPI unexpected send_data\n");
    }
    /* something is now (momentarily) in fifo */
    spi->dma_read_int_stat |= C2_SPI_DMA_INT_EX;
    chorus2_spi_update_interrupts(spi);
}

static uint8_t chorus2_spi_get_data(struct chorus2_spi_state_s *spi)
{
    struct chorus2_spi_device_s *dev;
    uint8_t txbuf = 0xff;
    uint8_t rxbuf = 0xff;

    /* start dma if currently idle and pending */
    if (chorus2_spi_get_pending(spi) && chorus2_spi_idle(spi)) {
        chorus2_spi_dma_start(spi);
    }
    if (spi->cur_dma.get_count) {
        --spi->cur_dma.get_count;
        /* read a byte from the device */
        dev = chorus2_spi_get_device(spi);
        if (dev) {
            dev->rxtx(dev->opaque, 1, &txbuf, &rxbuf);
        }
        /* complete dma if now idle */
        if (chorus2_spi_idle(spi)) {
            chorus2_spi_dma_complete(spi);
        }
    } else {
        ERRLOG("SPI unexpected send_data\n");
    }
    return rxbuf;
}

static uint64_t chorus2_spi_io_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    struct chorus2_spi_state_s *spi = opaque;
    unsigned int offset = addr >> 2;
    uint32_t ret = 0;

    if (unlikely(addr & 0x3)) {
        goto invalid;
    }
    if (unlikely(offset >= ARRAY_SIZE(chorus2_spi_reg_names))) {
        goto invalid;
    }
    if (unlikely(!chorus2_spi_reg_names[offset])) {
        goto invalid;
    }

    switch (offset) {
    case C2_SPI_REG0:
    case C2_SPI_REG1:
    case C2_SPI_REG2:
    case C2_SPI_REG4:
        ret = spi->reg[offset];
        break;
    case C2_SPI_REG3:
        ret = spi->reg[3];
        if (spi->next_dma.send) {
            ret |= C2_SPI_REG3_SENDDMA_MASK;
        }
        if (spi->next_dma.get) {
            ret |= C2_SPI_REG3_GETDMA_MASK;
        }
        if (spi->next_dma.cont) {
            ret |= C2_SPI_REG3_CONT_MASK;
        }
        ret |= spi->next_dma.cs << C2_SPI_REG3_CS_SHIFT;
        ret |= spi->next_dma.send_count;
        break;
    case C2_SPI_GETDATA:
        ret = chorus2_spi_get_data(spi);
        break;
    case C2_SPI_DMAREADINTSTAT:
        ret = spi->dma_read_int_stat;
        break;
    case C2_SPI_DMAREADINTEN:
        ret = spi->dma_read_int_en;
        break;
    case C2_SPI_DMAWRITEINTSTAT:
        ret = spi->dma_write_int_stat;
        break;
    case C2_SPI_DMAWRITEINTEN:
        ret = spi->dma_write_int_en;
        break;
    default:
        DBGLOG("unhandled ");
    }

    DBGLOG("spi read(%s @ 0x%03x) = 0x%08x\n",
           chorus2_spi_reg_names[offset], addr, ret);
    return ret;

invalid:
    ERRLOG("unhandled spi read(0x%03" HWADDR_PRIx ") = 0x%08x\n", addr, ret);
    return ret;
}

static void chorus2_spi_io_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    struct chorus2_spi_state_s *spi = opaque;
    unsigned int offset = addr >> 2;

    if (unlikely(addr & 0x3)) {
        goto invalid;
    }
    if (unlikely(offset >= ARRAY_SIZE(chorus2_spi_reg_names))) {
        goto invalid;
    }
    if (unlikely(!chorus2_spi_reg_names[offset])) {
        goto invalid;
    }

    switch (offset) {
    case C2_SPI_REG0:
        spi->reg[0] = val & C2_SPI_REG0_MASK;
        break;
    case C2_SPI_REG1:
        spi->reg[1] = val & C2_SPI_REG1_MASK;
        break;
    case C2_SPI_REG2:
        spi->reg[2] = val & C2_SPI_REG1_MASK;
        break;
    case C2_SPI_REG3:
        spi->reg[3] = val & C2_SPI_REG3_STORE_MASK;
        spi->next_dma.send_count = val & C2_SPI_REG3_XFERSIZE_MASK;
        if (!spi->next_dma.send_count) {
            spi->next_dma.send_count = 0x1000;
        }
        spi->next_dma.send = !!(val & C2_SPI_REG3_SENDDMA_MASK);
        spi->next_dma.get = !!(val & C2_SPI_REG3_GETDMA_MASK);
        spi->next_dma.cs = (val & C2_SPI_REG3_CS_MASK) >> C2_SPI_REG3_CS_SHIFT;
        if (spi->next_dma.cs == 0x3) {
            spi->next_dma.cs = -1;
        }
        spi->next_dma.cont = !!(val & C2_SPI_REG3_CONT_MASK);

        if (spi->reg[3] & C2_SPI_REG3_SRESET_MASK) {
            chorus2_spi_terminate_transaction(spi);
        }
        chorus2_spi_update_dreq(spi);
        break;
    case C2_SPI_REG4:
        spi->reg[4] = val & C2_SPI_REG1_MASK;
        break;
    case C2_SPI_SENDDATA:
        chorus2_spi_send_data(spi, val);
        break;
    case C2_SPI_DMAREADINTEN:
        spi->dma_read_int_en = val & C2_SPI_DMA_READ_INT_MASK;
        chorus2_spi_update_interrupts(spi);
        break;
    case C2_SPI_DMAREADINTCL:
        spi->dma_read_int_stat &= ~val;
        chorus2_spi_update_interrupts(spi);
        break;
    case C2_SPI_DMAWRITEINTEN:
        spi->dma_write_int_en = val & C2_SPI_DMA_WRITE_INT_MASK;
        chorus2_spi_update_interrupts(spi);
        break;
    case C2_SPI_DMAWRITEINTCL:
        spi->dma_write_int_stat &= ~val;
        chorus2_spi_update_interrupts(spi);
        break;
    default:
        DBGLOG("unhandled ");
    }

    DBGLOG("spi write(%s @ 0x%03x, 0x%08x)\n",
           chorus2_spi_reg_names[offset], addr, val);
    return;

invalid:
    ERRLOG("spi write(0x%03" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    return;
}

static const MemoryRegionOps chorus2_spi_io_ops = {
    .read = chorus2_spi_io_read,
    .write = chorus2_spi_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void chorus2_spi_device_init(struct chorus2_spi_state_s *spi, int cs,
                             chorus2_spi_device_rxtx rxtx,
                             chorus2_spi_device_setcs setcs, void *opaque)
{
    if (cs < 0 || cs >= ARRAY_SIZE(spi->devices)) {
        return;
    }

    spi->devices[cs].rxtx = rxtx;
    spi->devices[cs].setcs = setcs;
    spi->devices[cs].opaque = opaque;
}

struct chorus2_spi_state_s *chorus2_spi_init(qemu_irq txdrq, qemu_irq rxdrq)
{
    struct chorus2_spi_state_s *spi;

    spi = g_malloc0(sizeof(struct chorus2_spi_state_s));
    spi->reg[0] = 0x200c0c18; /* flash chip */
    spi->reg[1] = 0x200c0c18; /* flash chip */
    spi->reg[2] = 0x80010102; /* SD card */
    spi->reg[4] = 0x00010fff;

    memory_region_init_io(&spi->iomem, &chorus2_spi_io_ops, spi,
                          "chorus2-spi", C2_SPI_SIZE);

    spi->txdrq = txdrq;
    spi->rxdrq = rxdrq;

    chorus2_spi_reset(spi);
    return spi;
}

void chorus2_spi_reset(struct chorus2_spi_state_s *spi)
{
    chorus2_spi_terminate_transaction(spi);

    memset(&spi->next_dma, 0, sizeof(spi->next_dma));
    memset(&spi->cur_dma, 0, sizeof(spi->cur_dma));
    spi->dma_read_int_stat = C2_SPI_DMA_INT_TRIG;
    spi->dma_read_int_en = 0;
    spi->dma_write_int_stat = C2_SPI_DMA_INT_ALLDONETRIG | C2_SPI_DMA_INT_TRIG;
    spi->dma_write_int_en = 0;
    memset(spi->reg, 0, sizeof(spi->reg));

    chorus2_spi_update_dreq(spi);
}
