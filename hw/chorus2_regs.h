/*
 * Register definitions for Frontier Silicon Chorus2 SoC with META122.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#ifndef hw_chorus2_regs_h
# define hw_chorsu2_regs_h "chorus2_regs.h"

/* C2_SOCID */
#define C2_SOCID_DEFAULT                0xdb20

/* C2_BOOT_SETUP */
#define C2_BOOTSETUP_BOOTSRC_MASK       0x0007  /* Boot source selector */
#define C2_BOOTSETUP_BOOTSRC_SPI1       0x0007  /*  x11: Master SPI (Default) */
#define C2_BOOTSETUP_BOOTSRC_SCP1       0x0006  /*  110: Slave mode */
#define C2_BOOTSETUP_BOOTSRC_SPI2       0x0005  /*  101: Slave SPI (using NF
                                                         pins) */
#define C2_BOOTSETUP_BOOTSRC_SCP2       0x0004  /*  100: Slave SPI (using S2 and
                                                         SCP2 pins) */
#define C2_BOOTSETUP_BOOTSRC_C000       0x0002  /*  010: 0xC0000000 - CS[0]n */
#define C2_BOOTSETUP_BOOTSRC_C100       0x0001  /*  001: 0xC1000000 - CS[1]n */
#define C2_BOOTSETUP_BOOTSRC_NOBOOT     0x0000  /*  000: No boot - Wait for
                                                         JTAG */
#define C2_BOOTSETUP_BOOTEN_MASK        0x0008  /* Boot enable */
#define C2_BOOTSETUP_BOOTADDR_MASK      0x0010  /* Boot address: */
#define C2_BOOTSETUP_BOOTADDR_EXT       0x0000  /*  0: Boot from external bus at
                                                       0xc0000000 */
#define C2_BOOTSETUP_BOOTADDR_ROM       0x0010  /*  1: Boot from boot ROM at
                                                       0x80000000 */
#define C2_BOOTSETUP_BOOTROMOPT_MASK    0x00e0  /* Boot ROM option */
#define C2_BOOTSETUP_STAREN_MASK        0x0100  /* Self Test and Repair (STAR)
                                                   enable */
#define C2_BOOTSETUP_HREN_MASK          0x0200  /* Hard repair enable */
#define C2_BOOTSETUP_BISTEN_MASK        0x0400  /* BIST enable */
#define C2_BOOTSETUP_SREN_MASK          0x0800  /* Soft repair enable */
#define C2_BOOTSETUP_STARCLKSEL_MASK    0x1000  /* STAR clock select */
#define C2_BOOTSETUP_STARCLKSEL_TCK     0x0000  /*  0: JTAG (TCK) */
#define C2_BOOTSETUP_STARCLKSEL_1MHZ    0x1000  /*  1: Internally generated
                                                       1MHz */
#define C2_BOOTSETUP_BISTCLKSEL_MASK    0x2000  /* BIST clock select */
#define C2_BOOTSETUP_BISTCLKSEL_PROD    0x0000  /*  0: Production test only */
#define C2_BOOTSETUP_BISTCLKSEL_NORM    0x1000  /*  1: Normal */
#define C2_BOOTSETUP_DEFAULT            0xffff

#endif
