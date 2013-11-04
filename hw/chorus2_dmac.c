/*
 * Frontier Silicon Chorus2 DMA controller.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#include "chorus2_dmac.h"

#include "boards.h"
#include "soc_dma.h"
#include "qemu-log.h"

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

struct chorus2_dmac_channel_s {
    struct soc_dma_ch_s *dma;

    uint8_t  word_size;
    uint8_t  drq, dir;
    uint16_t count;
    uint32_t cnt;
    uint32_t irq_stat;

    unsigned int perip;
    hwaddr base;
    hwaddr offset;
    hwaddr start;
    unsigned char burst;
};

struct chorus2_dmac_state_s {
    MemoryRegion iomem;
    unsigned int num_channels;
    unsigned int num_perips;
    struct soc_dma_s *dma;

    struct chorus2_dmac_channel_s *chans;
};

/* register numbers (<<2 for offset) */
typedef enum {
    C2_DMAC_SETUP    = 0x000 >> 2,
    C2_DMAC_CNT      = 0x004 >> 2,
    C2_DMAC_PHADDR   = 0x008 >> 2,
    C2_DMAC_IRQSTAT  = 0x00C >> 2,
    C2_DMAC_2DMODE   = 0x010 >> 2,
} Chorus2DmacReg;

#define C2_DMAC_STRIDE_SHIFT        5
#define C2_DMAC_STRIDE              0x20

#define C2_DMAC_CNT_PW_SHIFT        27
#define C2_DMAC_CNT_PW_MASK         (0x3 << C2_DMAC_CNT_PW_SHIFT)
#define C2_DMAC_CNT_PW_8            (2 << C2_DMAC_CNT_PW_SHIFT)
#define C2_DMAC_CNT_PW_16           (1 << C2_DMAC_CNT_PW_SHIFT)
#define C2_DMAC_CNT_PW_32           0
#define C2_DMAC_CNT_LIEN_MASK       0x80000000
#define C2_DMAC_CNT_IEN_MASK        0x20000000
#define C2_DMAC_CNT_DIR_MASK        0x04000000
#define C2_DMAC_CNT_DREQ_MASK       0x00100000
#define C2_DMAC_CNT_SRST_MASK       0x00080000
#define C2_DMAC_CNT_LEN_MASK        0x00040000
#define C2_DMAC_CNT_EN_MASK         0x00010000
#define C2_DMAC_CNT_CNT_MASK        0x0000FFFF

#define C2_DMAC_PHADDR_ADDR_MASK    0x007FFFFF
#define C2_DMAC_PHADDR_BURST_MASK   0x07000000
#define C2_DMAC_PHADDR_BURST_SHIFT  24

#define C2_DMAC_IRQSTAT_FIN_MASK    (1<<17)

static void chorus2_dma_channel_load(struct chorus2_dmac_channel_s *ch)
{
    if (ch->dir) { /* perip to mem */
        ch->dma->type[0] = soc_dma_access_const;
        ch->dma->type[1] = soc_dma_access_linear;
        ch->dma->vaddr[0] = ch->base + ch->offset;
        ch->dma->vaddr[1] = ch->start;
    } else { /* mem to perip */
        ch->dma->type[0] = soc_dma_access_linear;
        ch->dma->type[1] = soc_dma_access_const;
        ch->dma->vaddr[0] = ch->start;
        ch->dma->vaddr[1] = ch->base + ch->offset;
    }
    soc_dma_ch_update(ch->dma);
}

static void chorus2_dma_update_interrupts(struct chorus2_dmac_channel_s *ch)
{
    /* FIXME trigger interrupt if ch->irq_stat */
}

static void chorus2_dmac_start_dma(struct chorus2_dmac_state_s *dmac,
                                   struct chorus2_dmac_channel_s *ch)
{
    chorus2_dma_channel_load(ch);
    soc_dma_set_request(ch->dma, 1);
}

static void chorus2_dmac_stop_dma(struct chorus2_dmac_state_s *dmac,
                                  struct chorus2_dmac_channel_s *ch)
{
    soc_dma_set_request(ch->dma, 0);
}

static uint64_t chorus2_dmac_io_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    struct chorus2_dmac_state_s *dmac = opaque;
    unsigned int chan = addr >> C2_DMAC_STRIDE_SHIFT;
    unsigned int offset = (addr & (C2_DMAC_STRIDE - 1)) >> 2;
    struct chorus2_dmac_channel_s *ch = &dmac->chans[chan];
    uint32_t ret = 0;

    if (unlikely(addr & 0x3)) {
        return 0;
    }
    if (unlikely(chan >= dmac->num_channels)) {
        return 0;
    }

    switch (offset) {
    case C2_DMAC_SETUP:
        return ch->start;
        break;
    case C2_DMAC_CNT:
        ret = ch->cnt | ch->count;
        if (dmac->dma->drqbmp & (1 << ch->perip)) {
            ret |= C2_DMAC_CNT_DREQ_MASK;
        }
        break;
    case C2_DMAC_PHADDR:
        ret = ch->offset >> 2;
        ret |= ch->burst << C2_DMAC_PHADDR_BURST_SHIFT;
        break;
    case C2_DMAC_IRQSTAT:
        ret = ch->irq_stat;
        break;
    case C2_DMAC_2DMODE:
        break;
    default:
        DBGLOG("dmac[%d] read(0x%03" HWADDR_PRIx ") = 0x%08x\n", chan, addr, ret);
    }

    return ret;
}

static void chorus2_dmac_io_write(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    struct chorus2_dmac_state_s *dmac = opaque;
    unsigned int chan = addr >> C2_DMAC_STRIDE_SHIFT;
    unsigned int offset = (addr & (C2_DMAC_STRIDE - 1)) >> 2;
    struct chorus2_dmac_channel_s *ch = &dmac->chans[chan];

    if (unlikely(addr & 0x3)) {
        return;
    }
    if (unlikely(chan >= dmac->num_channels)) {
        return;
    }

    switch (offset) {
    case C2_DMAC_SETUP:
        ch->start = val;
        break;
    case C2_DMAC_CNT:
        {
            uint32_t xor = ch->cnt ^ val;
            /* FIXME handle word_size of 0 (error) */
            ch->word_size = 4 >> ((val & C2_DMAC_CNT_PW_MASK)
                                        >> C2_DMAC_CNT_PW_SHIFT);
            ch->cnt = val & ~(C2_DMAC_CNT_DREQ_MASK | C2_DMAC_CNT_CNT_MASK);
            ch->dir = !!(val & C2_DMAC_CNT_DIR_MASK);
            ch->count = val;    /* 16 bits */
            if (val & C2_DMAC_CNT_SRST_MASK) {
                /* disable DMA */
                chorus2_dmac_stop_dma(dmac, ch);
                break;
            }
            if (xor & C2_DMAC_CNT_EN_MASK) {
                if (val & C2_DMAC_CNT_EN_MASK) {
                    if (dmac->dma->drqbmp & (1 << ch->perip)) {
                        /* enable DMA */
                        /* start reading, size pw, direction dir, cnt=cnt */
                        chorus2_dmac_start_dma(dmac, ch);
                    }
                } else {
                    /* disable DMA */
                    chorus2_dmac_stop_dma(dmac, ch);
                }
            }
        }
        break;
    case C2_DMAC_PHADDR:
        ch->offset = (val & C2_DMAC_PHADDR_ADDR_MASK) << 2;
        ch->burst = (val & C2_DMAC_PHADDR_BURST_MASK)
                            >> C2_DMAC_PHADDR_BURST_SHIFT;
        break;
    case C2_DMAC_IRQSTAT:
        ch->irq_stat = val & 0xf;
        chorus2_dma_update_interrupts(ch);
        break;
    case C2_DMAC_2DMODE:
        break;
    default:
        DBGLOG("dmac[%d] write(0x%03" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", chan, addr, val);
    }

    return;
}

static const MemoryRegionOps chorus2_dmac_io_ops = {
    .read = chorus2_dmac_io_read,
    .write = chorus2_dmac_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

MemoryRegion *chorus2_dmac_iomem(struct chorus2_dmac_state_s *dmac)
{
    return &dmac->iomem;
}

struct soc_dma_s *chorus2_dmac_dma(struct chorus2_dmac_state_s *dmac)
{
    return dmac->dma;
}

static void chorus2_dmac_process_request(void *opaque, int drq)
{
    struct chorus2_dmac_state_s *dmac = opaque;
    struct chorus2_dmac_channel_s *ch = dmac->chans;
    int i;

    for (i = 0; i < dmac->num_channels; ++i, ++ch) {
        if (ch->perip == drq) {
            if (ch->cnt & C2_DMAC_CNT_EN_MASK) {
                chorus2_dmac_start_dma(dmac, ch);
            }
        }
    }

}

static void chorus2_dma_request(void *opaque, int drq, int req)
{
    struct chorus2_dmac_state_s *dmac = opaque;
    if (req) {
        chorus2_dmac_process_request(opaque, drq);
        dmac->dma->drqbmp |= (1 << drq);
    } else {
        dmac->dma->drqbmp &= ~(1 << drq);
    }
}

static void chorus2_dma_setup(struct soc_dma_ch_s *dma)
{
    /* FIXME do something here, at least do some error checking */
}

static void chorus2_dma_transfer(struct soc_dma_ch_s *dma)
{
    struct chorus2_dmac_channel_s *ch = dma->opaque;
    uint8_t value[4];
    if (ch->count) {
#if DEBUG_LEVEL >= 3
        DBGLOG("DMA [%08x]---[%d * %d]-->[%08x]\n",
                dma->vaddr[0],
                ch->count, ch->word_size,
                dma->vaddr[1]);
#endif
        while (ch->count) {
            cpu_physical_memory_read(dma->vaddr[0], value, ch->word_size);
#if DEBUG_LEVEL >= 4
            {
                int i;
                DBGLOG("\tDMA [%08x]---(", dma->vaddr[0]);
                for (i = 0; i < ch->word_size; ++i) {
                    DBGLOG("%02x", value[i]);
                }
                DBGLOG(")-->[%08x]\n", dma->vaddr[1]);
            }
#endif
            cpu_physical_memory_write(dma->vaddr[1], value, ch->word_size);
            dma->vaddr[ch->dir] += ch->word_size;
            --ch->count;
        }
    }

    ch->cnt &= ~C2_DMAC_CNT_EN_MASK;
    if (ch->cnt & C2_DMAC_CNT_IEN_MASK) {
        ch->irq_stat |= C2_DMAC_IRQSTAT_FIN_MASK;
        chorus2_dma_update_interrupts(ch);
    }
}

struct chorus2_dmac_state_s *chorus2_dmac_init(unsigned int chans,
                                               unsigned int perips,
                                               hwaddr base)
{
    struct chorus2_dmac_state_s *dmac;
    int i;

    dmac = g_malloc0(sizeof(struct chorus2_dmac_state_s));
    dmac->chans = g_malloc0(sizeof(struct chorus2_dmac_channel_s)*chans);
    dmac->num_channels = chans;

    memory_region_init_io(&dmac->iomem, &chorus2_dmac_io_ops, dmac,
                          "chorus2-dmac", C2_DMAC_SIZE);

    dmac->num_perips = perips;

    dmac->dma = soc_dma_init(chans);
    dmac->dma->freq = 100000000;
    dmac->dma->transfer_fn = chorus2_dma_transfer;
    dmac->dma->setup_fn = chorus2_dma_setup;
    dmac->dma->drq = qemu_allocate_irqs(chorus2_dma_request, dmac, perips);
    dmac->dma->opaque = dmac;
    for (i = 0; i < dmac->num_channels; ++i) {
        dmac->chans[i].dma = &dmac->dma->ch[i];
        dmac->dma->ch[i].opaque = &dmac->chans[i];
        dmac->chans[i].base = base;
    }

    return dmac;
}

void chorus2_dmac_reset(struct chorus2_dmac_state_s *dmac)
{
    int i;
    for (i = 0; i < dmac->num_channels; ++i) {
        chorus2_dmac_stop_dma(dmac, &dmac->chans[i]);
        dmac->chans[i].word_size = 0;
        dmac->chans[i].drq = 0;
        dmac->chans[i].dir = 0;
        dmac->chans[i].count = 0;
        dmac->chans[i].cnt = 0;
        dmac->chans[i].irq_stat = 0;
        dmac->chans[i].perip = 0;
        dmac->chans[i].offset = 0;
        dmac->chans[i].start = 0;
        dmac->chans[i].burst = 0;
    }
}

void chorus2_dmac_set_perip(struct chorus2_dmac_state_s *dmac,
                            unsigned int chan, unsigned int perip)
{
    dmac->chans[chan].perip = perip;
}
