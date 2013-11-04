/*
 * Imagination Technologies Pixel Display Pipeline (PDP).
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#include "pdp.h"
#include "pixel_ops.h"
#include "console.h"
#include "qemu-log.h"
#include "vga_int.h"

/*
 * The length of the scan chunk temporary arrays in pixels.
 * When pixels need converting, the source data is read into an array on the
 * stack in chunks of this many pixels. Ideally this should be the width of the
 * screen so as not to waste space or require multiple separate reads.
 */
#define SCAN_CHUNK_LEN 640

/* find the planes intersecting a row y and the next y where it will change */
static void pdp_planes_under_y(pdp_state *pdp, pdp_plane **planes,
                               uint16_t y, uint16_t *change)
{
    uint16_t next = pdp->de_height;
    int z;
    int i = 0;
    for (z = 0; z < pdp->num_planes && pdp->sorted_planes[z]; ++z) {
        pdp_plane *plane = pdp->sorted_planes[z];
        if (y < plane->ystart) {
            if (plane->ystart < next) {
                next = plane->ystart;
            }
        } else if (y < plane->yend) {
            if (plane->yend < next) {
                next = plane->yend;
            }
            *change = next;
            planes[i++] = plane;
        }
    }
    if (i < pdp->num_planes) {
        planes[i] = NULL;
    }
    *change = next;
}

/*
 * find the next plane at (x,y) under level z and the next x where it will
 * change
 */
static pdp_plane *pdp_plane_under_x(pdp_state *pdp, pdp_plane **planes,
                                    uint16_t y, uint16_t x, int z,
                                    uint16_t *change)
{
    uint16_t next = pdp->de_width;
    for (; z < pdp->num_planes && planes[z]; ++z) {
        pdp_plane *plane = planes[z];
        if (x < plane->xstart) {
            if (plane->xstart < next) {
                next = plane->xstart;
            }
        } else if (x < plane->xend) {
            if (plane->xend < next) {
                next = plane->xend;
            }
            *change = next;
            return plane;
        }
    }
    *change = next;
    return NULL;
}

/* read a line of greyscale pixels into a destination buffer */
static void pdp_scanline_g8(pdp_state *pdp, uint8_t *dest,
                            hwaddr src, int nr)
{
    int dw = pdp->dw;
    int count, i;
    uint8_t tmp[SCAN_CHUNK_LEN];
    while (nr) {
        /* read some data into buffer */
        count = nr;
        if (count > sizeof(tmp)) {
            count = sizeof(tmp);
        }
        cpu_physical_memory_rw(src, tmp, count, 0);
        /* run through palette */
        switch (dw) {
        case 1:
            for (i = 0; i < count; ++i) {
                *dest++ = pdp->gs_dentries[tmp[i]];
            }
            break;
        case 2:
            for (i = 0; i < count; ++i, dest += 2) {
                *(uint16_t *)dest = pdp->gs_dentries[tmp[i]];
            }
            break;
        case 3:
            for (i = 0; i < count; ++i) {
                uint32_t dentry = pdp->gs_dentries[tmp[i]];
                *dest++ = dentry >> 16;
                *dest++ = dentry >> 8;
                *dest++ = dentry;
            }
            break;
        case 4:
            for (i = 0; i < count; ++i, dest += 4) {
                *(uint32_t *)dest = pdp->gs_dentries[tmp[i]];
            }
            break;
        }
        src += count;
        nr -= count;
    }
}

/* read a line of indexed pixels into a destination buffer */
static void pdp_scanline_i8(pdp_state *pdp, uint8_t *dest,
                            hwaddr src, int nr)
{
    int dw = pdp->dw;
    int count, i;
    uint8_t tmp[SCAN_CHUNK_LEN];
    pdp_palette *pal = &pdp->palettes[0];
    while (nr) {
        /* read some data into buffer */
        count = nr;
        if (count > sizeof(tmp)) {
            count = sizeof(tmp);
        }
        cpu_physical_memory_rw(src, tmp, count, 0);

        /* run through palette */
        switch (dw) {
        case 1:
            for (i = 0; i < count; ++i) {
                *dest++ = pal->dentries[tmp[i]];
            }
            break;
        case 2:
            for (i = 0; i < count; ++i, dest += 2) {
                *(uint16_t *)dest = pal->dentries[tmp[i]];
            }
            break;
        case 3:
            for (i = 0; i < count; ++i) {
                uint32_t dentry = pal->dentries[tmp[i]];
                *dest++ = dentry >> 16;
                *dest++ = dentry >> 8;
                *dest++ = dentry;
            }
            break;
        case 4:
            for (i = 0; i < count; ++i, dest += 4) {
                *(uint32_t *)dest = pal->dentries[tmp[i]];
            }
            break;
        }
        src += count;
        nr -= count;
    }
}

/* convert an rgb565 pixel to separate r, g, b components */
static void pdp_rgb16_to_rgb(const pdp_state *pdp, const uint8_t *src,
                             uint8_t *rp, uint8_t *gp, uint8_t *bp)
{
    uint16_t val = *(const uint16_t *)src;
    uint8_t r, g, b;
    r = (val >> 8) & 0xf8;
    g = (val >> 3) & 0xfc;
    b = (val << 3) & 0xf8;
    *rp = r | (r >> 5);
    *gp = g | (g >> 6);
    *bp = b | (b >> 5);
}

/* read a line of 15bit rgb565 pixels into a destination buffer */
static void pdp_scanline_rgb16(pdp_state *pdp, uint8_t *dest,
                                hwaddr src, int nr)
{
    int dw = pdp->dw;
    int count, i;
    uint8_t tmp[SCAN_CHUNK_LEN*2];

    /* native format, easy peasy */
    if (dw == 2) {
        cpu_physical_memory_rw(src, dest, nr*dw, 0);
        return;
    }
    while (nr) {
        /* read some data into buffer */
        count = nr;
        if (count > sizeof(tmp)/dw) {
            count = sizeof(tmp)/dw;
        }
        cpu_physical_memory_rw(src, tmp, 2*count, 0);

        /* run through converter */
        switch (dw) {
        case 1:
            for (i = 0; i < count; ++i) {
                uint8_t r, g, b;
                pdp_rgb16_to_rgb(pdp, &tmp[i*2], &r, &g, &b);
                *dest++ = pdp->rgb_to_pixel(r, g, b);
            }
            break;
        case 3:
            for (i = 0; i < count; ++i) {
                uint8_t r, g, b;
                uint32_t col;
                pdp_rgb16_to_rgb(pdp, &tmp[i*2], &r, &g, &b);
                col = pdp->rgb_to_pixel(r, g, b);
                *dest++ = col >> 16;
                *dest++ = col >> 8;
                *dest++ = col;
            }
            break;
        case 4:
            for (i = 0; i < count; ++i, dest += 4) {
                uint8_t r, g, b;
                pdp_rgb16_to_rgb(pdp, &tmp[i*2], &r, &g, &b);
                *(uint32_t *)dest = pdp->rgb_to_pixel(r, g, b);
            }
            break;
        }
        src += 2*count;
        nr -= count;
    }
}

/* convert an argb8888 pixel to separate a, r, g, b components */
static void pdp_rgb32_to_argb(const pdp_state *pdp, const uint8_t *src,
                              uint8_t *ap, uint8_t *rp, uint8_t *gp, uint8_t *bp)
{
    uint32_t val = *(const uint32_t *)src;
    *ap = (val >> 24) & 0xff;
    *rp = (val >> 16) & 0xff;
    *gp = (val >> 8) & 0xff;
    *bp = (val >> 0) & 0xff;
}

/* read a line of 32bit argb8888 pixels into a destination buffer */
static void pdp_scanline_argb32(pdp_state *pdp, uint8_t *dest,
                                hwaddr src, int nr)
{
    int dw = pdp->dw;
    int count, i;
    uint8_t tmp[SCAN_CHUNK_LEN*4];

    /* native format, easy peasy */
    if (dw == 4) {
        cpu_physical_memory_rw(src, dest, nr*dw, 0);
        return;
    }
    while (nr) {
        /* read some data into buffer */
        count = nr;
        if (count > sizeof(tmp)/dw) {
            count = sizeof(tmp)/dw;
        }
        cpu_physical_memory_rw(src, tmp, 4*count, 0);

        /* run through converter */
        switch (dw) {
        case 1:
            for (i = 0; i < count; ++i) {
                uint8_t a, r, g, b;
                pdp_rgb32_to_argb(pdp, &tmp[i*4], &a, &r, &g, &b);
                *dest++ = pdp->rgb_to_pixel(r, g, b);
            }
            break;
        case 2:
            for (i = 0; i < count; ++i) {
                uint8_t a, r, g, b;
                uint32_t col;
                uint16_t px;
                pdp_rgb32_to_argb(pdp, &tmp[i*4], &a, &r, &g, &b);
                col = pdp->rgb_to_pixel(r, g, b);
                px = (((col >> 16) & 0x1f) << 11) |
                     (((col >> 8) & 0x3f) << 6) |
                     (((col >> 0) & 0x1f) << 0);
                *dest++ = px >> 8;
                *dest++ = px;
            }
            break;
        case 3:
            for (i = 0; i < count; ++i) {
                uint8_t a, r, g, b;
                uint32_t col;
                pdp_rgb32_to_argb(pdp, &tmp[i*4], &a, &r, &g, &b);
                col = pdp->rgb_to_pixel(r, g, b);
                *dest++ = col >> 16;
                *dest++ = col >> 8;
                *dest++ = col;
            }
            break;
        }
        src += 4*count;
        nr -= count;
    }
}

/* use a lookup table to find the scanline function for a given pixel format */
typedef void (*pdp_scanline_p)(pdp_state *pdp, uint8_t *dest,
                                hwaddr src, int nr);

static const pdp_scanline_p pdp_scanlines[PDP_PIXFMT_MAX] = {
    [PDP_PIXFMT_GREYSCALE] = pdp_scanline_g8,
    [PDP_PIXFMT_INDEX8] = pdp_scanline_i8,
    [PDP_PIXFMT_RGB565] = pdp_scanline_rgb16,
    [PDP_PIXFMT_ARGB8888] = pdp_scanline_argb32,
    /* FIXME implement scanline functions for the other pixel formats */
#if 0
    [PDP_PIXFMT_RGB888] = pdp_scanline_rgb24,
    [PDP_PIXFMT_ARGB1555] = pdp_scanline_argb16_1555,
    [PDP_PIXFMT_ARGB4444] = pdp_scanline_argb16_4444,
#endif
};

/* perform a scanline of the PDP data output */
static void pdp_do_scanline(pdp_state *pdp, pdp_plane **planes, uint16_t y,
                            uint8_t *dest)
{
    uint16_t next = 0;
    int dw = pdp->dw;
    while (next < pdp->de_width) {
        uint16_t x = next;
        hwaddr src;
        /*
         * Find the top plane and the number of pixels this remains the top
         * plane.
         */
        pdp_plane *plane = pdp_plane_under_x(pdp, planes, y, next, 0, &next);
        if (!plane) {
            /* fill with background colour */
            while (x++ < next) {
                memcpy(dest, &pdp->dbg, dw);
                dest += dw;
            }
            continue;
        }
        /* Scan the pixel data depending on the pixel format */
        src = plane->base + plane->stride * (y-plane->ystart)
            + (x - plane->xstart);
        if (pdp_scanlines[plane->pixfmt]) {
            pdp_scanlines[plane->pixfmt](pdp, dest, src, next - x);
        } else {
            memset(dest, 0, (next - x)*dw);
        }
        dest += (next - x)*dw;
    };
}

/* scan the entire PDP data output */
static void pdp_do_scan(pdp_state *pdp)
{
    uint8_t *dest;
    int linesize;
    uint16_t next = 0;

    dest = ds_get_data(pdp->ds);
    linesize = ds_get_linesize(pdp->ds);

    pdp_plane **planes = pdp->row_planes;
    while (next < pdp->de_height) {
        uint16_t y = next;
        /*
         * Find the planes intersecting this row and the number of rows the
         * intersecting planes will remain unchanged.
         */
        pdp_planes_under_y(pdp, planes, y, &next);
        /* Do a scanline for each of the rows in the range */
        for (; y < next; ++y) {
            pdp_do_scanline(pdp, planes, y, dest);
            dest += linesize;
        }
    }
}

static void pdp_update_display(void *opaque)
{
    pdp_state *pdp = opaque;

    if (!pdp || !pdp->sync_active || pdp->power_dn
             || !pdp->de_width || !pdp->de_height) {
        return;
    }

    /* Resolution */
    if (pdp->de_width != ds_get_width(pdp->ds) ||
            pdp->de_height != ds_get_height(pdp->ds)) {
        qemu_console_resize(pdp->ds, pdp->de_width, pdp->de_height);
        pdp->invalidate = 1;
    }

    /* Detect altered pixel format */
    if (ds_get_bits_per_pixel(pdp->ds) != 8*pdp->dw) {
        pdp_change_dst_format(pdp);
    }

    pdp_do_scan(pdp);
    dpy_gfx_update(pdp->ds, 0, 0, pdp->de_width, pdp->de_height);
    pdp->invalidate = 0;
    pdp_irqs(pdp, PDP_INT_VEVENT0);
}

static void pdp_invalidate_display(void *opaque)
{
    pdp_state *pdp = opaque;
    pdp->invalidate = 1;
}

static void pdp_screen_dump(void *opaque, const char *filename, bool cswitch,
                            Error **errp)
{
    pdp_state *pdp = opaque;
    pdp_update_display(pdp);
    ppm_save(filename, pdp->ds->surface, errp);
}

/*
 * PUBLIC PDP INTERFACE FOR SOC REGISTER INTERFACES TO USE
 */

/* Give a plane an id */
int pdp_set_plane(pdp_state *pdp, unsigned int id, pdp_plane *plane)
{
    pdp->planes[id] = plane;
    return 0;
}

/* Update the order of the visible planes */
void pdp_update_planes(pdp_state *pdp)
{
    unsigned int i, j;
    unsigned int nr = pdp->num_planes;
    pdp_plane **planes = pdp->planes;
    pdp_plane **sorted = pdp->sorted_planes;
    /* clear sorted plane list */
    memset(sorted, 0, sizeof(pdp_plane *)*nr);
    /* add visible planes into list */
    for (i = 0; i < nr; ++i) {
        pdp_plane *plane = planes[i];
        if (!plane || !plane->en) {
            continue;
        }
        if (plane->xstart > pdp->de_width || plane->ystart > pdp->de_height) {
            continue;
        }
        if (!plane->width || !plane->height) {
            continue;
        }
        /* skip out of range zorders, and overwrite repeated zorders */
        if (plane->zorder < pdp->num_planes) {
            sorted[plane->zorder] = plane;
        }
    }
    /* push visible planes towards beginning of list */
    for (j = i = 0; i < nr; ++i) {
        if (sorted[i]) {
            sorted[j++] = sorted[i];
        }
    }
    if (j < nr) {
        sorted[j] = NULL;
    }
    /* do a full redraw */
    pdp->invalidate = 1;
}

/* Initialise PDP state */
void pdp_init(pdp_state *pdp, qemu_irq irq, uint32_t core_id,
              uint32_t core_rev, unsigned int planes, CPUArchState *env)
{
    memset(pdp, 0, sizeof(*pdp));

    pdp->core_id        = core_id;
    pdp->core_rev       = core_rev;
    pdp->num_planes     = planes;
    pdp->planes         = g_malloc0(sizeof(pdp_plane *)*planes);
    pdp->sorted_planes  = g_malloc0(sizeof(pdp_plane *)*planes);
    pdp->row_planes     = g_malloc0(sizeof(pdp_plane *)*planes);

    pdp->env = env;
    pdp->irq = irq;
    pdp->ds = graphic_console_init(pdp_update_display,
                                   pdp_invalidate_display,
                                   pdp_screen_dump, NULL, pdp);
    pdp_change_dst_format(pdp);

    pdp_bg_set(pdp, 0, 0, 0);
}

/* update cached values which depend on destination pixel format */
void pdp_change_dst_format(pdp_state *pdp)
{
    int i, j;

    /* update pixel format and width */
    switch (ds_get_bits_per_pixel(pdp->ds)) {
    case 8:
        pdp->rgb_to_pixel = rgb_to_pixel8;
        pdp->dw = 1;
        break;
    case 15:
        pdp->rgb_to_pixel = rgb_to_pixel15;
        pdp->dw = 2;
        break;
    case 16:
        pdp->rgb_to_pixel = rgb_to_pixel16;
        pdp->dw = 2;
        break;
    case 32:
        pdp->rgb_to_pixel = rgb_to_pixel32;
        pdp->dw = 4;
        break;
    default:
        qemu_log("unknown host depth %d\n", ds_get_bits_per_pixel(pdp->ds));
        return;
    }

    /* update background colour */
    pdp->dbg = pdp->rgb_to_pixel(pdp->bg[0], pdp->bg[1], pdp->bg[2]);

    /* update greyscale palette */
    for (i = 0; i < 256; ++i) {
        pdp->gs_dentries[i] = pdp->rgb_to_pixel(i, i, i);
    }

    /* update palettes */
    for (i = 0; i < ARRAY_SIZE(pdp->palettes); ++i) {
        pdp_palette *pal = &pdp->palettes[i];
        for (j = 0; j < ARRAY_SIZE(pal->dentries); ++j) {
            pal->dentries[j] = pdp->rgb_to_pixel(pal->entries[j][0],
                                                 pal->entries[j][1],
                                                 pal->entries[j][2]);
        }
    }

    /* need to do a full redraw */
    pdp->invalidate = 1;
}

/* Reset the PDP state */
void pdp_reset(pdp_state *pdp)
{
}

/* fire interrupts */
void pdp_irqs(pdp_state *pdp, uint32_t irqs)
{
    uint32_t old_high, new_high;

    old_high = pdp->irq_mask & pdp->irq_stat;
    pdp->irq_stat |= irqs;
    new_high = pdp->irq_mask & pdp->irq_stat;
    if (!old_high && new_high) {
        qemu_irq_raise(pdp->irq);
    }
}

/* mask interrupts */
void pdp_mask_irqs(pdp_state *pdp, uint32_t irqs)
{
    uint32_t old_high, new_high;

    old_high = pdp->irq_mask & pdp->irq_stat;
    pdp->irq_mask = irqs;
    new_high = pdp->irq_mask & pdp->irq_stat;
    if (!old_high && new_high) {
        qemu_irq_raise(pdp->irq);
    } else if (old_high && !new_high) {
        qemu_irq_lower(pdp->irq);
    }
}

/* clear a set of interrupts */
void pdp_clear_irqs(pdp_state *pdp, uint32_t irqs)
{
    uint32_t old_high, new_high;

    old_high = pdp->irq_mask & pdp->irq_stat;
    pdp->irq_stat &= ~irqs;
    new_high = pdp->irq_mask & pdp->irq_stat;
    if (old_high && !new_high) {
        qemu_irq_lower(pdp->irq);
    }
}

/* Set the background colour */
void pdp_bg_set(pdp_state *pdp, unsigned int r, unsigned int g, unsigned int b)
{
    pdp->bg[0] = r;
    pdp->bg[1] = g;
    pdp->bg[2] = b;
    pdp->dbg = pdp->rgb_to_pixel(r, g, b);
}

/* Set a palette colour */
void pdp_palette_set(pdp_state *pdp, int pali, uint8_t index,
                     unsigned int r, unsigned int g, unsigned int b)
{
    pdp_palette *pal = &pdp->palettes[pali];
    pal->entries[index][0] = r;
    pal->entries[index][1] = g;
    pal->entries[index][2] = b;
    pal->dentries[index] = pdp->rgb_to_pixel(r, g, b);
}
