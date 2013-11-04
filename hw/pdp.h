/*
 * Imagination Technologies Pixel Display Pipeline (PDP).
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 *
 * Base PDP state and functions. Individual PDP register interfaces should
 * interface with this API to control the PDP.
 */

#ifndef hw_pdp_h
# define hw_pdp_h "pdp.h"

#include "qemu-common.h"
#include "irq.h"

/* Pixel formats */
typedef enum pdp_pixfmt {
    PDP_PIXFMT_GREYSCALE,
    PDP_PIXFMT_INDEX8,

    PDP_PIXFMT_RGB565,
    PDP_PIXFMT_RGB888,

    PDP_PIXFMT_ARGB1555,
    PDP_PIXFMT_ARGB4444,
    PDP_PIXFMT_ARGB8888,

    PDP_PIXFMT_MAX,
} pdp_pixfmt;

/* Basic plane */
typedef struct pdp_plane_s {
    /* stream enable */
    int en;

    /* stream position */
    unsigned int zorder;
    uint16_t xstart;
    uint16_t ystart;
    uint16_t xend;
    uint16_t yend;
    uint16_t width;
    uint16_t height;

    /* pixel format */
    pdp_pixfmt pixfmt;
    /* stride */
    uint16_t stride;
    /* dimentions */
    uint16_t src_width;
    uint16_t src_height;

    /* physical base pointer */
    hwaddr base;
} pdp_plane;

/* PDP palette */
#define PDP_MAX_PALETTES 1
typedef struct pdp_palette_s {
    /* [256 entries][R,G,B] */
    uint8_t entries[256][3];
    /* converted into current format */
    uint32_t dentries[256];
} pdp_palette;

/* Interrupts */
#define PDP_INT_VEVENT0 0x00000004

/* Main PDP state */
typedef struct pdp_state_s {
    /* id registers */
    uint32_t core_id;
    uint32_t core_rev;

    /* sync and power control */
    int sync_active;
    int power_dn;

    /* data enable width/height */
    uint16_t de_width;
    uint16_t de_height;

    /* background colour */
    uint8_t bg[3];
    uint32_t dbg;

    /* greyscale palette */
    uint32_t gs_dentries[256];
    /* palettes */
    pdp_palette palettes[PDP_MAX_PALETTES];

    /* the PDP consists of a number of different planes */
    unsigned int num_planes;
    struct pdp_plane_s **planes;

    /* visible planes sorted from top to bottom */
    struct pdp_plane_s **sorted_planes;

    /* row planes (temporary */
    struct pdp_plane_s **row_planes;

    /* interrupt output */
    qemu_irq irq;
    uint32_t irq_stat;
    uint32_t irq_mask;

    /* QEMU display state */
    DisplayState *ds;
    int dw;
    unsigned int (*rgb_to_pixel)(unsigned int r, unsigned int g,
                                 unsigned int b);
    int invalidate;
    CPUArchState *env;
} pdp_state;

/* Give a plane an id */
int pdp_set_plane(pdp_state *pdp, unsigned int id, pdp_plane *plane);

/* Update the order of the visible planes */
void pdp_update_planes(pdp_state *pdp);

/* Initialise PDP state */
void pdp_init(pdp_state *pdp, qemu_irq irq, uint32_t core_id,
              uint32_t core_rev, unsigned int planes, CPUArchState *env);
void pdp_change_dst_format(pdp_state *pdp);

/* Reset the PDP state */
void pdp_reset(pdp_state *pdp);

/* Interrupt control */
void pdp_irqs(pdp_state *pdp, uint32_t irqs);
void pdp_mask_irqs(pdp_state *pdp, uint32_t irqs);
void pdp_clear_irqs(pdp_state *pdp, uint32_t irqs);

/* Fixed colours control */
void pdp_bg_set(pdp_state *pdp, unsigned int r, unsigned int g, unsigned int b);
void pdp_palette_set(pdp_state *pdp, int pali, uint8_t index,
                     unsigned int r, unsigned int g, unsigned int b);

#endif
