/*
 * Imagination Technologies PDP in Toumaz XENIF SoC (Comet).
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 *
 * Comet specific PDP register interface.
 */

#include "comet_pdp.h"
#include "hw.h"
#include "qemu-log.h"

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

typedef enum {
    PDP_STR1SURF    = 0x000,
    PDP_STR1ADDR    = 0x060,
    PDP_STR1POSN    = 0x0C0,
    PDP_PALETTE1    = 0x14C,
    PDP_PALETTE2    = 0x150,
    PDP_SYNCCTRL    = 0x154,
    /* interrupts */
    PDP_INTSTAT     = 0x178,
    PDP_INTENAB     = 0x17C,
    PDP_INTCLR      = 0x1AC,
} CometPDPRegs;

#define PDP_STR1SURF_USELUT_MASK        0x80000000
#define PDP_STR1SURF_USELUT_SHIFT       31
#define PDP_STR1SURF_PIXFMT_MASK        0x78000000
#define PDP_STR1SURF_PIXFMT_SHIFT       27
typedef enum {
    PDP_STR1SURF_PIXFMT_RGB8        =   0x0,
    PDP_STR1SURF_PIXFMT_ARGB4444    =   0x4,
    PDP_STR1SURF_PIXFMT_ARGB1555    =   0x5,
    PDP_STR1SURF_PIXFMT_RGB888      =   0x6,
    PDP_STR1SURF_PIXFMT_RGB565      =   0x7,
    PDP_STR1SURF_PIXFMT_ARGB8888    =   0x8,
} comet_pdp_pixefmts;
#define PDP_STRXSURF_WIDTH_MASK         0x001ff800
#define PDP_STRXSURF_WIDTH_SHIFT        11
#define PDP_STRXSURF_HEIGHT_MASK        0x000007ff
#define PDP_STRXSURF_HEIGHT_SHIFT       0

#define PDP_STRXADDR_STREAMEN_MASK      0x80000000
#define PDP_STRXADDR_BASEADDR_MASK      0x003fffff

#define PDP_STRXPOSN_SRCSTRIDE_MASK     0xffc00000
#define PDP_STRXPOSN_SRCSTRIDE_SHIFT    22
#define PDP_STRXPOSN_XSTART_MASK        0x003ff800
#define PDP_STRXPOSN_XSTART_SHIFT       11
#define PDP_STRXPOSN_YSTART_MASK        0x000007ff
#define PDP_STRXPOSN_YSTART_SHIFT       0

#define PDP_PALETTE1_LUTADDR_MASK       0xff000000
#define PDP_PALETTE1_LUTADDR_SHIFT      24
#define PDP_PALETTE2_LUTDATA_MASK       0x00ffffff
#define PDP_PALETTE2_LUTDATA_SHIFT      0

#define PDP_SYNCCTRL_SYNCACTIVE_MASK    0x80000000
#define PDP_SYNCCTRL_SYNCACTIVE_SHIFT   31
#define PDP_SYNCCTRL_POWERDN_MASK       0x10000000
#define PDP_SYNCCTRL_POWERDN_SHIFT      28

static void comet_pdp_recalc_gfx_base(comet_pdp_state *cpdp)
{
    cpdp->gfx.plane.base = cpdp->base_addr + cpdp->gfx.offset;
}

static uint64_t comet_pdp_io_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    comet_pdp_state *cpdp = opaque;
    comet_pdp_gfx_plane *cgfx = &cpdp->gfx;
    pdp_state *pdp = &cpdp->pdp;
    uint32_t ret = 0;

    switch (addr) {
    case PDP_STR1SURF:
        if (cgfx->use_lut) {
            ret |= PDP_STR1SURF_USELUT_MASK;
        }
        /* FIXME complete */
        break;
    case PDP_STR1ADDR:
        ret = cgfx->offset >> 4;
        if (cgfx->plane.en) {
            ret |= PDP_STRXADDR_STREAMEN_MASK;
        }
        break;
    case PDP_STR1POSN:
        ret = ((cgfx->plane.stride >> 4) - 1)
                << PDP_STRXPOSN_SRCSTRIDE_SHIFT;
        ret |= cgfx->plane.xstart
                << PDP_STRXPOSN_XSTART_SHIFT;
        ret |= cgfx->plane.ystart
                << PDP_STRXPOSN_YSTART_SHIFT;
        break;
    case PDP_PALETTE1:
        ret = cpdp->palette_addr << PDP_PALETTE1_LUTADDR_SHIFT;
        break;
    case PDP_PALETTE2:
        ret = (pdp->palettes[0].entries[cpdp->palette_addr][0]
                << PDP_PALETTE2_LUTDATA_SHIFT) << 16;
        ret |= (pdp->palettes[0].entries[cpdp->palette_addr][1]
                << PDP_PALETTE2_LUTDATA_SHIFT) << 8;
        ret |= pdp->palettes[0].entries[cpdp->palette_addr][2]
                << PDP_PALETTE2_LUTDATA_SHIFT;
        break;
    case PDP_SYNCCTRL:
        if (pdp->sync_active) {
            ret |= PDP_SYNCCTRL_SYNCACTIVE_MASK;
        }
        if (pdp->power_dn) {
            ret |= PDP_SYNCCTRL_POWERDN_MASK;
        }
        break;
    case PDP_INTSTAT:
        ret = pdp->irq_stat;
        break;
    case PDP_INTENAB:
        ret = pdp->irq_mask;
        break;
    case PDP_INTCLR:
        /* FIXME what does reading this reg actually gove? */
        ret = 0;
        break;
    default:
        DBGLOG("unhandled pdp read(0x%08" HWADDR_PRIx ")\n", addr);
    }
    return ret;
}

static void comet_pdp_io_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    comet_pdp_state *cpdp = opaque;
    comet_pdp_gfx_plane *cgfx = &cpdp->gfx;
    pdp_plane *gfx = &cgfx->plane;
    pdp_state *pdp = &cpdp->pdp;

    comet_pdp_pixefmts cpixfmt;
    pdp_pixfmt pixfmt;

    switch (addr) {
    case PDP_STR1SURF:
        cgfx->use_lut = val & PDP_STR1SURF_USELUT_MASK;
        cpixfmt = (val & PDP_STR1SURF_PIXFMT_MASK)
                        >> PDP_STR1SURF_PIXFMT_SHIFT;
        gfx->width = 1 + ((val & PDP_STRXSURF_WIDTH_MASK)
                            >> PDP_STRXSURF_WIDTH_SHIFT);
        gfx->height = 1 + ((val & PDP_STRXSURF_HEIGHT_MASK)
                            >> PDP_STRXSURF_HEIGHT_SHIFT);
        gfx->src_width = gfx->width;
        gfx->src_height = gfx->height;
        gfx->xend = gfx->xstart + gfx->width;
        gfx->yend = gfx->ystart + gfx->height;
        switch (cpixfmt) {
        case PDP_STR1SURF_PIXFMT_RGB8:
            if (cgfx->use_lut) {
                pixfmt = PDP_PIXFMT_INDEX8;
            } else {
                pixfmt = PDP_PIXFMT_GREYSCALE;
            }
            break;
        case PDP_STR1SURF_PIXFMT_ARGB4444:
            pixfmt = PDP_PIXFMT_ARGB4444;
            break;
        case PDP_STR1SURF_PIXFMT_ARGB1555:
            pixfmt = PDP_PIXFMT_ARGB1555;
            break;
        case PDP_STR1SURF_PIXFMT_RGB888:
            pixfmt = PDP_PIXFMT_RGB888;
            break;
        case PDP_STR1SURF_PIXFMT_RGB565:
            pixfmt = PDP_PIXFMT_RGB565;
            break;
        case PDP_STR1SURF_PIXFMT_ARGB8888:
            pixfmt = PDP_PIXFMT_ARGB8888;
            break;
        default:
            DBGLOG("Comet PDP: bad pixel format %x\n", cpixfmt);
            pixfmt = PDP_PIXFMT_RGB888;
            break;
        }
        gfx->pixfmt = pixfmt;
        break;
    case PDP_STR1ADDR:
        cgfx->offset = (val & PDP_STRXADDR_BASEADDR_MASK) << 4;
        comet_pdp_recalc_gfx_base(cpdp);
        cgfx->plane.en = !!(val & PDP_STRXADDR_STREAMEN_MASK);
        pdp_update_planes(&cpdp->pdp);
        break;
    case PDP_STR1POSN:
        cgfx->plane.stride = (((val & PDP_STRXPOSN_SRCSTRIDE_MASK)
                            >> PDP_STRXPOSN_SRCSTRIDE_SHIFT) + 1) << 4;
        cgfx->plane.xstart = (val & PDP_STRXPOSN_XSTART_MASK)
                            >> PDP_STRXPOSN_XSTART_SHIFT;
        cgfx->plane.ystart = (val & PDP_STRXPOSN_YSTART_MASK)
                            >> PDP_STRXPOSN_YSTART_SHIFT;
        break;
    case PDP_PALETTE1:
        cpdp->palette_addr = (val & PDP_PALETTE1_LUTADDR_MASK)
                            >> PDP_PALETTE1_LUTADDR_SHIFT;
        break;
    case PDP_PALETTE2:
        val = (val & PDP_PALETTE2_LUTDATA_MASK)
                >> PDP_PALETTE2_LUTDATA_SHIFT;
        pdp_palette_set(pdp, 0, cpdp->palette_addr,
                        0xff & (val >> 16),
                        0xff & (val >> 8),
                        0xff & val);
        break;
    case PDP_SYNCCTRL:
        pdp->sync_active = !!(val & PDP_SYNCCTRL_SYNCACTIVE_MASK);
        pdp->power_dn = !!(val & PDP_SYNCCTRL_POWERDN_MASK);
        break;
    case PDP_INTSTAT:
        pdp_irqs(pdp, val);
        break;
    case PDP_INTENAB:
        pdp_mask_irqs(pdp, val);
        break;
    case PDP_INTCLR:
        pdp_clear_irqs(pdp, val);
        break;
    default:
        DBGLOG("unhandled pdp write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    }
}

static const MemoryRegionOps comet_pdp_io_ops = {
    .read = comet_pdp_io_read,
    .write = comet_pdp_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void comet_pdp_reset(void *opaque)
{
    comet_pdp_state *cpdp = opaque;

    pdp_reset(&cpdp->pdp);
}

comet_pdp_state *comet_pdp_init(MemoryRegion *address_space,
                                hwaddr iomem, target_ulong len,
                                qemu_irq irq, CPUArchState *env)
{
    comet_pdp_state *cpdp;
    pdp_state *pdp;
    pdp_plane *gfx;

    /* Initialise PDP state */
    cpdp = g_malloc0(sizeof(comet_pdp_state));
    pdp = &cpdp->pdp;
    gfx = &cpdp->gfx.plane;
    pdp_init(pdp, irq, 0x00000000, 0x00000000, 1, env);
    pdp_set_plane(pdp, 0, &cpdp->gfx.plane);

    /* hard coded settings */
    pdp->de_width = 640;
    pdp->de_height = 480;
    pdp->sync_active = 1;
    gfx->zorder = 0;
    gfx->xstart = 0;
    gfx->ystart = 0;
    gfx->width = pdp->de_width - gfx->xstart;
    gfx->height = pdp->de_height - gfx->ystart;
    gfx->xend = gfx->xstart + gfx->width;
    gfx->yend = gfx->ystart + gfx->height;
    gfx->src_width = gfx->width;
    gfx->src_height = gfx->height;
    gfx->pixfmt = PDP_PIXFMT_ARGB8888;
    gfx->stride = gfx->src_width * 2;
    comet_pdp_recalc_gfx_base(cpdp);
    pdp_update_planes(pdp);

    /* Initialise register region */
    memory_region_init_io(&cpdp->iomem, &comet_pdp_io_ops, cpdp,
                          "comet-pdp", len - 1);
    memory_region_add_subregion(address_space, iomem, &cpdp->iomem);

    qemu_register_reset(comet_pdp_reset, cpdp);

    return cpdp;
}

/* Set base address of PDP */
void comet_pdp_set_base(comet_pdp_state *cpdp, hwaddr base)
{
    cpdp->base_addr = base;
    comet_pdp_recalc_gfx_base(cpdp);
}

void comet_pdp_set_res(comet_pdp_state *cpdp, int width, int height)
{
    cpdp->pdp.de_width = width;
    cpdp->pdp.de_height = height;
}
