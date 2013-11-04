/*
 *  META Linux bootloader
 *
 *  Copyright (c) 2011 Imagination Technologies
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef META_BOOTLOADER_H
#define META_BOOTLOADER_H

#include "core.h"

extern const char *ldr_filename;
extern unsigned int ldr_pagesize;
extern int boot_thread;

struct MetaBootInfo {
    /* First, the LDR of a bootloader is loaded first to set up MMU and TBI */
    const char *ldr_filename;
    unsigned int ldr_pagesize;
    /* Then the kernel is loaded, and the bootloader will boot it */
    const char *kernel_filename;
    /* Cmdline is put after kernel, and pointed to by GP_SECURE0 register */
    const char *kernel_cmdline;

    /* RAM information */
    hwaddr ram_phys;
    hwaddr ram_size;

    /* Callback to setup up SoC hardware in the event of a direct kernel boot */
    void (*setup_direct_boot)(CPUArchState *env, void *opaque);
    void *setup_direct_boot_opaque;

    /* If the above fails, just use a certain entry point */
    target_ulong entry;

    /* The thread to boot the kernel on */
    int boot_thread;

    /* Some boot options */
    uint32_t flags;
#define META_BOOT_MINIM     (1 << 0)
#define META_BOOT_MMU_HTP   (1 << 1)
};

/* Setup the core loading of a Linux kernel image on reset */
int meta_setup_boot(MetaCore *core, MetaBootInfo *boot);

#endif
