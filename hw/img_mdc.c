#include "img_mdc.h"

#include "boards.h"
#include "soc_dma.h"
#include "qemu-log.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define CHAN_CFG_LIST_IEN          (1 << 31)
#define CHAN_CFG_BSWAP             (1 << 30)
#define CHAN_CFG_IEN               (1 << 29)
#define CHAN_CFG_LEVEL_INT         (1 << 28)
#define CHAN_CFG_CHANNEL_MASK      0x3f
#define CHAN_CFG_CHANNEL_SHIFT     20
#define CHAN_CFG_ACC_DEL_MASK      0x7
#define CHAN_CFG_ACC_DEL_SHIFT     16
#define CHAN_CFG_WAIT_UNPACK       (1 << 13)
#define CHAN_CFG_INC_W             (1 << 12)
#define CHAN_CFG_WAIT_PACK         (1 << 9)
#define CHAN_CFG_INC_R             (1 << 8)
#define CHAN_CFG_PHYSICAL_W        (1 << 7)
#define CHAN_CFG_WIDTH_W_MASK      0x7
#define CHAN_CFG_WIDTH_W_SHIFT     4
#define CHAN_CFG_PHYSICAL_R        (1 << 3)
#define CHAN_CFG_WIDTH_R_MASK      0x7
#define CHAN_CFG_WIDTH_R_SHIFT     0

#define CHAN_READ_STHREAD_MASK     0xf
#define CHAN_READ_STHREAD_SHIFT    28
#define CHAN_READ_RTHREAD_MASK     0xf
#define CHAN_READ_RTHREAD_SHIFT    24
#define CHAN_READ_PRIORITY_MASK    0xf
#define CHAN_READ_PRIORITY_SHIFT   20
#define CHAN_READ_WTHREAD_MASK     0xf
#define CHAN_READ_WTHREAD_SHIFT    16
#define CHAN_READ_HOLD_OFF_MASK    0xf
#define CHAN_READ_HOLD_OFF_SHIFT   12
#define CHAN_READ_BURST_SIZE_MASK  0xff
#define CHAN_READ_BURST_SIZE_SHIFT 4
#define CHAN_READ_DREQ_ENABLE      (1 << 1)
#define CHAN_READ_READBACK         (1 << 0)

#define CHAN_CMDP_CMDP_MASK        0x3f
#define CHAN_CMDP_CMDP_SHIFT       16
#define CHAN_CMDP_INT_ACTIVE       (1 << 8)
#define CHAN_CMDP_CMDS_DONE_MASK   0x3f
#define CHAN_CMDP_CMDS_DONE_SHIFT  0

#define CHAN_CTRL_TAG_MASK         0xf
#define CHAN_CTRL_TAG_SHIFT        24
#define CHAN_CTRL_CANCEL           (1 << 20)
#define CHAN_CTRL_DREQ             (1 << 16)
#define CHAN_CTRL_FIFO_DEPTH       (1 << 8)
#define CHAN_CTRL_LIST_EN          (1 << 4)
#define CHAN_CTRL_EN               (1 << 0)

struct img_mdc_chan_regs_s {
    uint32_t cfg;
    uint32_t cfg_read;
    uint32_t read_addr;
    uint32_t write_addr;
    uint32_t transfer_size;
    uint32_t node_addr;
    uint32_t cmds_processed;
    uint32_t ctrl;
};

struct img_mdc_channel_s {
    struct img_mdc_state_s *mdc;
    struct soc_dma_ch_s *dma;
    qemu_irq irq;
    bool irq_state;
    unsigned int perip;
    uint8_t value[16];
    size_t value_sz;

#if DEBUG_LEVEL > 0
    unsigned int idx;
#endif

    struct img_mdc_chan_regs_s shadow;
    struct img_mdc_chan_regs_s active;

    bool list_ien;
};

struct img_mdc_channel_group_s {
};

struct img_mdc_state_s {
    MemoryRegion iomem;
    unsigned int num_channels;
    unsigned int num_perips;
    struct soc_dma_s *dma;

    struct img_mdc_channel_s *chans;
    struct img_mdc_channel_group_s *chan_groups;
};

#define MDC_CHAN_BASE      0x000
#define MDC_CHANGROUP_BASE 0x800
#define MDC_COMMON_BASE   0x900

typedef enum {
    MDC_CHAN_GENERAL_CONFIG     = 0x00,
    MDC_CHAN_READ_CONFIG        = 0x04,
    MDC_CHAN_READ_ADDRESS       = 0x08,
    MDC_CHAN_WRITE_ADDRESS      = 0x0c,
    MDC_CHAN_TRANSFER_SIZE      = 0x10,
    MDC_CHAN_LIST_NODE_ADDRESS  = 0x14,
    MDC_CHAN_CMDS_PROCESSED     = 0x18,
    MDC_CHAN_CTRL_STATUS        = 0x1c,
} ImgMDCchanReg;

typedef enum {
    MDC_CHANGRP_CMD_DONE_FLAGS = 0x00,
    MDC_CHANGRP_READY_STATUS   = 0x04,
} ImgMDCchanGroupReg;

typedef enum {
    MDC_GLOBAL_CONFIG_A = 0x00,
    MDC_GLOBAL_CONFIG_B = 0x04,
    MDC_GLOBAL_STATUS   = 0x08,
} ImgMDCCommonReg;

static void img_mdc_update_interrupts(struct img_mdc_channel_s *chan)
{
    bool high = !!(chan->active.cmds_processed & CHAN_CMDP_INT_ACTIVE);
    if (high && !chan->irq_state) {
        qemu_irq_raise(chan->irq);
    } else if (!high && chan->irq_state) {
        qemu_irq_lower(chan->irq);
    }
    chan->irq_state = high;
}

static void img_mdc_cmd_done(struct img_mdc_channel_s *chan)
{
    uint8_t cmds_done;
    bool interrupt = false;

    cmds_done = (chan->active.cmds_processed >> CHAN_CMDP_CMDS_DONE_SHIFT) & CHAN_CMDP_CMDS_DONE_MASK;
    cmds_done++;

    chan->active.cmds_processed &= ~(CHAN_CMDP_CMDS_DONE_MASK << CHAN_CMDP_CMDS_DONE_SHIFT);
    chan->active.cmds_processed |= cmds_done;

    if (chan->active.ctrl & CHAN_CTRL_LIST_EN) {
        /* interrupt only if LIST_IEN was set in the shadow register */
        interrupt = chan->list_ien;
    } else {
        /* interrupt if IEN is set */
        interrupt = chan->active.cfg & CHAN_CFG_IEN;
    }

    DBGLOG("MDC[%d] cmds_done=%d interrupt=%d\n", chan->idx, cmds_done, !!interrupt);

    if (interrupt) {
        if (chan->active.cfg & CHAN_CFG_LEVEL_INT)
            chan->active.cmds_processed |= CHAN_CMDP_INT_ACTIVE;
        else {
            /* TODO: what? */
            fprintf(stderr, "MDC unimplemented edge interrupt\n");
        }
        img_mdc_update_interrupts(chan);
    }
}

static bool img_mdc_channel_ready(struct img_mdc_state_s *mdc,
                                  struct img_mdc_channel_s *chan)
{
    if (chan->dma->enable) {
        /* already running */
        return false;
    }

    if (chan->irq_state) {
        /* interrupt raised */
        DBGLOG("MDC[%d] waiting for interrupt clear\n", chan->idx);
        return false;
    }

    if (!(chan->active.ctrl & (CHAN_CTRL_EN | CHAN_CTRL_LIST_EN))) {
        /* not enabled */
        DBGLOG("MDC[%d] not enabled\n", chan->idx);
        return false;
    }

    if (chan->active.cfg_read & CHAN_READ_DREQ_ENABLE) {
        if (!chan->perip) {
            DBGLOG("DREQ_ENABLE on channel %d without peripheral!\n",
                    chan->idx);
            return false;
        }

        if (!(mdc->dma->drqbmp & (1 << chan->perip))) {
#if DEBUG_LEVEL >= 3
            DBGLOG("MDC[%d] waiting for DREQ_ENABLE %d\n", chan->idx, chan->perip);
#endif
            return false;
        }
    }

    return true;
}

static bool img_mdc_load_list_node(struct img_mdc_channel_s *chan)
{
    if (!(chan->active.ctrl & CHAN_CTRL_LIST_EN))
        return false;

    if (!chan->active.node_addr)
        return false;

    DBGLOG("MDC[%d] list node at 0x%08x\n", chan->idx, chan->active.node_addr);
    cpu_physical_memory_read(chan->active.node_addr, (uint8_t*)&chan->active, 32);


#if DEBUG_LEVEL >= 3
    {
        size_t i;
        uint32_t *dat = (uint32_t*)&chan->active;
        for (i = 0; i < 8; i++)
            DBGLOG("  0x%02x: 0x%08x\n", i << 2, dat[i]);
    }
#endif

    if (chan->active.cfg & CHAN_CFG_LIST_IEN)
        img_mdc_cmd_done(chan);

    return true;
}

static bool img_mdc_start_dma(struct img_mdc_state_s *mdc,
                              struct img_mdc_channel_s *chan)
{
    if (!(chan->active.ctrl & CHAN_CTRL_EN) && chan->active.ctrl & CHAN_CTRL_LIST_EN)
        img_mdc_load_list_node(chan);

    if (!img_mdc_channel_ready(mdc, chan))
        return false;

    DBGLOG("MDC[%d] begin\n", chan->idx);
    soc_dma_set_request(chan->dma, 1);
    return true;
}

static uint64_t img_mdc_io_read(void *opaque, hwaddr addr,
	                        unsigned size)
{
    struct img_mdc_state_s *mdc = opaque;
    struct img_mdc_channel_s *chan;
    unsigned int chan_num, chan_gidx, cg_num;
    uint32_t ret = 0;
    uint8_t cmds_done, cmd_processed;
    bool active;
    hwaddr off;

    if (unlikely(addr & 0x3)) {
        return ret;
    }

    if (addr >= MDC_COMMON_BASE) {
        switch (addr - MDC_COMMON_BASE) {
        case MDC_GLOBAL_CONFIG_A:
            ret |= 1 << 16;
            ret |= mdc->num_channels << 8;
            ret |= 5;
            break;
        case MDC_GLOBAL_CONFIG_B:
            break;
        case MDC_GLOBAL_STATUS:
            ret |= 0xffff << 16; /* READY */
            ret |= 1 << 8; /* ENABLE */
            break;
        default:
            DBGLOG("unhandled ");
        }
        DBGLOG("MDC common read(0x%03x) = 0x%08x\n", addr, ret);
        return ret;
    }

    if (addr >= MDC_CHANGROUP_BASE) {
        addr -= MDC_CHANGROUP_BASE;
        cg_num = addr / 0x40;
        addr %= 0x40;

        for (chan_gidx = 0; chan_gidx < 32; chan_gidx++) {
            chan_num = (cg_num << 5) + chan_gidx;
            if (chan_num >= mdc->num_channels)
                break;
            chan = &mdc->chans[chan_num];

            switch (addr) {
            case MDC_CHANGRP_CMD_DONE_FLAGS:
                cmds_done = (chan->active.cmds_processed >> CHAN_CMDP_CMDS_DONE_SHIFT) & CHAN_CMDP_CMDS_DONE_MASK;
                cmd_processed = (chan->active.cmds_processed >> CHAN_CMDP_CMDP_SHIFT) & CHAN_CMDP_CMDP_MASK;
                if (cmds_done != cmd_processed)
                    ret |= (1 << chan_gidx);
                break;
            case MDC_CHANGRP_READY_STATUS:
                if (img_mdc_channel_ready(mdc, chan))
                    ret |= (1 << chan_gidx);
                break;
            }
        }

        DBGLOG("MDC[cg%d] read(0x%02x) = 0x%08x\n", cg_num, addr, ret);
        return ret;
    }

    chan_num = addr >> 6;
    chan = &mdc->chans[chan_num];

    if (unlikely(chan_num >= mdc->num_channels)) {
        return ret;
    }

    active = !!(addr & 0x20);
    off = addr & 0x3f;

    if (off <= MDC_CHAN_CTRL_STATUS) {
        if (active || off >= MDC_CHAN_CMDS_PROCESSED)
            ret = ((uint32_t*)(&chan->active))[off >> 2];
        else
            ret = ((uint32_t*)(&chan->shadow))[off >> 2];
        DBGLOG("MDC[%d] read(0x%02x) = 0x%08x\n", chan_num, off, ret);
    } else {
        DBGLOG("MDC[%d] unhandled read(0x%03x)\n", chan_num, off);
    }

    return ret;
}

static void img_mdc_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    struct img_mdc_state_s *mdc = opaque;
    struct img_mdc_channel_s *chan;
    unsigned int chan_num;
    bool active;
    uint32_t old, update_mask = 0xffffffff;
    hwaddr off;

    if (unlikely(addr & 0x3)) {
        return;
    }

    if (addr >= MDC_COMMON_BASE) {
        return;
    }

    if (addr >= MDC_CHANGROUP_BASE) {
        return;
    }

    chan_num = addr >> 6;
    chan = &mdc->chans[chan_num];

    if (unlikely(chan_num >= mdc->num_channels)) {
        DBGLOG("MDC write to invalid channel %d\n", chan_num);
        return;
    }

    active = !!(addr & 0x20);
    off = addr & 0x3f;

    if (active && off < MDC_CHAN_CMDS_PROCESSED) {
        /* it's only valid to write to the shadow registers */
        DBGLOG("MDC[%d] write to active register 0x%x\n", chan_num, addr);
        return;
    }

    if (off >= MDC_CHAN_CMDS_PROCESSED) {
        old = ((uint32_t*)(&chan->active))[off >> 2];

        switch (off) {
        case MDC_CHAN_CMDS_PROCESSED:
            update_mask = 0x00ff0100;
            break;

        case MDC_CHAN_CTRL_STATUS:
            if (val & CHAN_CTRL_CANCEL)
                soc_dma_set_request(chan->dma, 0);
            chan->list_ien = val & CHAN_CFG_LIST_IEN;
            chan->active = chan->shadow;
            break;
        }

        val = (old & ~update_mask) | (val & update_mask);
        ((uint32_t*)(&chan->active))[off >> 2] = val;
        DBGLOG("MDC[%d] 0x%02x = 0x%08x\n", chan_num, off, val);

        switch (off) {
        case MDC_CHAN_CMDS_PROCESSED:
            img_mdc_update_interrupts(chan);
            img_mdc_start_dma(mdc, chan);
            break;

        case MDC_CHAN_CTRL_STATUS:
            img_mdc_start_dma(mdc, chan);
            break;
        }

        return;
    }

    if (off <= MDC_CHAN_CTRL_STATUS) {
        DBGLOG("MDC[%d] 0x%02x = 0x%08x\n", chan_num, off, val);

        ((uint32_t*)(&chan->shadow))[off >> 2] = val;

        switch (off) {
        case MDC_CHAN_TRANSFER_SIZE:
            chan->shadow.transfer_size &= 0x00ffffff;
            break;
	}

        return;
    }

    DBGLOG("MDC[%d] unhandled write(0x%02x, 0x%08x)\n",
           chan_num, off, val);
    return;
}

static const MemoryRegionOps img_mdc_io_ops = {
    .read = img_mdc_io_read,
    .write = img_mdc_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

MemoryRegion *img_mdc_iomem(struct img_mdc_state_s *mdc)
{
    return &mdc->iomem;
}

struct soc_dma_s *img_mdc_dma(struct img_mdc_state_s *mdc)
{
    return mdc->dma;
}

static void img_mdc_request(void *opaque, int drq, int req)
{
    struct img_mdc_state_s *mdc = opaque;
    struct img_mdc_channel_s *chan = mdc->chans;
    int i;

    if (req) {
        mdc->dma->drqbmp |= (1 << drq);

        for (i = 0; i < mdc->num_channels; ++i, ++chan) {
            if (chan->perip == drq)
                img_mdc_start_dma(mdc, chan);
        }
    } else {
        mdc->dma->drqbmp &= ~(1 << drq);
    }
}

static void img_mdc_setup(struct soc_dma_ch_s *dma)
{
    struct img_mdc_channel_s *chan = dma->opaque;

    DBGLOG("MDC[%d] setup\n", chan->idx);

    dma->bytes = chan->active.transfer_size + 1;
    chan->dma->vaddr[0] = chan->active.read_addr;
    chan->dma->vaddr[1] = chan->active.write_addr;
    soc_dma_ch_update(chan->dma);
}

static bool img_mdc_transfer_dreq_wait(struct soc_dma_ch_s *dma)
{
    struct img_mdc_channel_s *chan = dma->opaque;

    if (dma->bytes && chan->active.cfg_read & CHAN_READ_DREQ_ENABLE && chan->perip) {
        if (!(chan->mdc->dma->drqbmp & (1 << chan->perip))) {
            DBGLOG("MDC[%d] waiting for DREQ_ENABLE %d, %d bytes remaining\n",
                   chan->idx, chan->perip, dma->bytes);
            return true;
        }
    }

    return false;
}

static void img_mdc_transfer(struct soc_dma_ch_s *dma)
{
    struct img_mdc_channel_s *chan = dma->opaque;
    uint8_t sz_t, sz_r, sz_w;
    uint32_t r_rem, r_idx, r_pad, w_rem, w_idx;

    sz_r = (chan->active.cfg >> CHAN_CFG_WIDTH_R_SHIFT) & CHAN_CFG_WIDTH_R_MASK;
    sz_r = 1 << sz_r;
    sz_w = (chan->active.cfg >> CHAN_CFG_WIDTH_W_SHIFT) & CHAN_CFG_WIDTH_W_MASK;
    sz_w = 1 << sz_w;
    sz_t = MAX(sz_r, sz_w);

#if DEBUG_LEVEL >= 3
    DBGLOG("MDC[%d] %d bytes 0x%08x:%d-->0x%08x:%d\n",
           chan->idx, dma->bytes, dma->vaddr[0], sz_r, dma->vaddr[1], sz_w);
#endif

    while (dma->bytes) {
        r_idx = chan->value_sz;
        w_idx = 0;
        w_rem = MIN(sz_t - r_idx, dma->bytes) + r_idx;
        r_rem = w_rem - r_idx;
        r_pad = sz_t - r_rem;

        while (r_rem) {
            cpu_physical_memory_read(dma->vaddr[0], chan->value + r_idx, sz_r);
            r_idx += sz_r;
            if (chan->active.cfg & CHAN_CFG_INC_R) {
                dma->vaddr[0] += sz_r;
                chan->active.read_addr = dma->vaddr[0];
            }
            r_rem -= MIN(sz_r, r_rem);
            if (r_rem && img_mdc_transfer_dreq_wait(dma))
                break;
        }

        /* in case DREQ was deasserted */
        w_rem -= r_rem;

        while (r_pad--)
            chan->value[r_idx++] = 0x00;

        if (chan->active.cfg & CHAN_CFG_BSWAP) {
            int i;
            for (i = 0; i < (sz_r / 2); i++) {
                uint8_t tmp = chan->value[i];
                chan->value[i] = chan->value[(sz_r-1)-i];
                chan->value[(sz_r-1)-i] = tmp;
            }
        }

        while (w_rem) {
            if (w_rem < sz_w && r_rem) {
                chan->value_sz = w_rem;
                DBGLOG("MDC[%d] breaking transfer mid-write, value_sz=%u\n", chan->idx, chan->value_sz);
                break;
            }

#if DEBUG_LEVEL >= 4
            {
                int i;
                DBGLOG("\tMDC[%d] [0x%08x]---(", chan->idx, dma->vaddr[0]);
                for (i = 0; i < sz_w; i++)
                    DBGLOG(" %02x", chan->value[w_idx+i]);
                DBGLOG(" )-->[0x%08x]\n", dma->vaddr[1]);
            }
#endif

            cpu_physical_memory_write(dma->vaddr[1], chan->value + w_idx, sz_w);
            w_idx += sz_w;
            if (chan->active.cfg & CHAN_CFG_INC_W) {
                dma->vaddr[1] += sz_w;
                chan->active.write_addr = dma->vaddr[1];
            }
            w_rem -= MIN(sz_w, w_rem);
            chan->value_sz = 0;
        }

        dma->bytes -= MIN(sz_t, dma->bytes) - (r_rem + chan->value_sz);
        chan->active.transfer_size = dma->bytes - 1;

        if (chan->value_sz || img_mdc_transfer_dreq_wait(dma))
            return;
    }

    if (!img_mdc_load_list_node(chan)) {
        DBGLOG("MDC[%d] transfer complete\n", chan->idx);
        chan->active.ctrl &= ~(CHAN_CTRL_EN | CHAN_CTRL_LIST_EN);
        soc_dma_set_request(dma, 0);
        img_mdc_cmd_done(chan);
    }
}

struct img_mdc_state_s *img_mdc_init(unsigned int chans, unsigned int perips,
                                     hwaddr base, qemu_irq *irqs)
{
    struct img_mdc_state_s *mdc;
    int i;

    mdc = g_malloc0(sizeof(struct img_mdc_state_s));
    mdc->chans = g_malloc0(sizeof(struct img_mdc_channel_s) * chans);
    mdc->num_channels = chans;

    memory_region_init_io(&mdc->iomem, &img_mdc_io_ops, mdc,
                          "img-mdc", IMG_MDC_SIZE);

    mdc->num_perips = perips;

    mdc->dma = soc_dma_init(chans);
    mdc->dma->freq = 100000000;
    mdc->dma->transfer_fn = img_mdc_transfer;
    mdc->dma->setup_fn = img_mdc_setup;
    mdc->dma->drq = qemu_allocate_irqs(img_mdc_request, mdc, perips);
    mdc->dma->opaque = mdc;
    for (i = 0; i < mdc->num_channels; ++i) {
        mdc->chans[i].mdc = mdc;
        mdc->chans[i].irq = irqs[i];
        mdc->chans[i].dma = &mdc->dma->ch[i];
        mdc->chans[i].dma->bytes = 0;
        mdc->chans[i].dma->type[0] = soc_dma_access_linear;
        mdc->chans[i].dma->type[1] = soc_dma_access_linear;
        mdc->chans[i].value_sz = 0;
        mdc->dma->ch[i].opaque = &mdc->chans[i];
#if DEBUG_LEVEL > 0
        mdc->chans[i].idx = i;
#endif
    }

    return mdc;
}

void img_mdc_reset(struct img_mdc_state_s *mdc)
{
    int i;
    for (i = 0; i < mdc->num_channels; ++i) {
        soc_dma_set_request(mdc->chans[i].dma, 0);
        memset(&mdc->chans[i].active, 0, sizeof(mdc->chans[i].active));
        memset(&mdc->chans[i].shadow, 0, sizeof(mdc->chans[i].shadow));
        mdc->chans[i].perip = 0;
        mdc->chans[i].value_sz = 0;
        img_mdc_update_interrupts(&mdc->chans[i]);
    }
}

unsigned int img_mdc_get_perip(struct img_mdc_state_s *mdc, unsigned int chan)
{
    return mdc->chans[chan].perip;
}

void img_mdc_set_perip(struct img_mdc_state_s *mdc, unsigned int chan,
                       unsigned int perip)
{
    mdc->chans[chan].perip = perip;
}

