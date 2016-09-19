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

/* META elf support (loading into virtual space instead of physical) */
#include "hw.h"
#include "disas.h"
#include "monitor.h"
#include "sysemu.h"
#include "uboot_image.h"
#include "loader.h"
#include "fw_cfg.h"
#include "dasim.h"
#include "meta_boot_mmu.h"
#include "meta_boot_tbi.h"

/* ELF loader */

static void *load_at(int fd, int offset, int size)
{
    void *ptr;
    if (lseek(fd, offset, SEEK_SET) < 0) {
        return NULL;
    }
    ptr = g_malloc(size);
    if (read(fd, ptr, size) != size) {
        g_free(ptr);
        return NULL;
    }
    return ptr;
}


#ifdef ELF_CLASS
#undef ELF_CLASS
#endif

#define ELF_CLASS   ELFCLASS32
#include "elf.h"

static CPUArchState *elf_env;
#define SZ              32
#define elf_word        uint32_t
#define elf_sword       int32_t
#define bswapSZs        bswap32s
#undef rom_add_blob_fixed
#define rom_add_blob_fixed(label, data, size, addr) \
                    meta_dasim_rw(elf_env, addr, data, size, 1)
#define load_elf32 load_elf32v
#include "elf_ops.h"

#undef elfhdr
#undef elf_phdr
#undef elf_shdr
#undef elf_sym
#undef elf_note
#undef elf_word
#undef elf_sword
#undef bswapSZs
#undef SZ
#undef rom_add_blob_fixed
#undef load_elf32

#include "qemu-common.h"
#include "loader.h"
#include "meta_boot.h"
#include "dasim.h"
#include "elf.h"
#include "ldr.h"

/* return < 0 if error, otherwise the number of bytes loaded in memory */
static int _meta_load_elf(const char *filename,
                          uint64_t (*translate_fn)(void *, uint64_t),
                          void *translate_opaque, uint64_t *pentry, uint64_t
                          *lowaddr, uint64_t *highaddr, int big_endian, int
                          elf_machine, int clear_lsb)
{
    int fd, data_order, target_data_order, must_swab, ret;
    uint8_t e_ident[EI_NIDENT];

    fd = open(filename, O_RDONLY | O_BINARY);
    if (fd < 0) {
        perror(filename);
        return -1;
    }
    if (read(fd, e_ident, sizeof(e_ident)) != sizeof(e_ident)) {
        goto fail;
    }
    if (e_ident[0] != ELFMAG0 ||
        e_ident[1] != ELFMAG1 ||
        e_ident[2] != ELFMAG2 ||
        e_ident[3] != ELFMAG3) {
        goto fail;
    }
#ifdef HOST_WORDS_BIGENDIAN
    data_order = ELFDATA2MSB;
#else
    data_order = ELFDATA2LSB;
#endif
    must_swab = data_order != e_ident[EI_DATA];
    if (big_endian) {
        target_data_order = ELFDATA2MSB;
    } else {
        target_data_order = ELFDATA2LSB;
    }

    if (target_data_order != e_ident[EI_DATA]) {
        goto fail;
    }

    lseek(fd, 0, SEEK_SET);
    if (e_ident[EI_CLASS] != ELFCLASS64) {
        ret = load_elf32v(filename, fd, translate_fn, translate_opaque,
                          must_swab, pentry, lowaddr, highaddr, elf_machine,
                          clear_lsb);
    } else {
        ret = -1;
    }

    close(fd);
    return ret;

 fail:
    close(fd);
    return -1;
}

/* load elf, return image size */
static int meta_load_elf(CPUArchState *env, const char *elf, target_ulong *entry)
{
    uint64_t entryp, high;
    int ret;

    entryp = 0;
    elf_env = env;
    ret = _meta_load_elf(elf, NULL, NULL, &entryp, NULL, &high, 0, ELF_MACHINE,
                         0);
    if (ret < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                elf);
        exit(1);
    }

    if (entry) {
        *entry = entryp;
    }

    return high - entryp;
}

/* Loading using LDR file */

static bool meta_load_ldr_error(ldr_ctx_t *ctx, int err, void *user)
{
    const char *errstr;
    switch (err) {
    case EHEADER:
        errstr = "header";
        break;
    case ECRC:
        errstr = "crc - have you set the page size correctly?";
        break;
    case EUNSUPPORTED:
        errstr = "unsupported";
        break;
    case EUNKNOWN:
        errstr = "unknown";
        break;
    case EREADLDR:
        errstr = "readldr";
        break;
    default:
        errstr = "other";
        break;
    };
    fprintf(stderr, "Error loading LDR: %d (%s)\n", err, errstr);
    return false;
}

static bool meta_load_ldr_loadmem(ldr_ctx_t *ctx, uint32_t addr, uint8_t *data,
                                  size_t sz, void *user)
{
    CPUArchState *env = user;

    if (meta_dasim_rw(env, addr, data, sz, 1) < 0) {
        fprintf(stderr, "ldr_loadmem 0x%08x (%u bytes) failed\n",
                addr, (unsigned int)sz);
        return false;
    }

    return true;
}

static bool meta_load_ldr_loadcore(ldr_ctx_t *ctx, ldr_reg_t *regs, size_t num,
                                   void *user)
{
    size_t i;

    fprintf(stderr, "ldr_loadcore unimplemented:\n");
    for (i = 0; i < num; i++) {
        fprintf(stderr, "  0x%08x |= (0x%08x & 0x%08x)\n", regs[i].reg_desc,
               regs[i].wr_mask, regs[i].val);
    }
    fprintf(stderr, "\n");

    return false;
}

static bool meta_load_ldr_loadmmreg(ldr_ctx_t *ctx, ldr_reg_t *regs, size_t num,
                                    void *user)
{
    size_t i;

    fprintf(stderr, "ldr_loadmmreg unimplemented:\n");
    for (i = 0; i < num; i++) {
        fprintf(stderr, "  0x%08x |= (0x%08x & 0x%08x)\n", regs[i].reg_desc,
               regs[i].wr_mask, regs[i].val);
    }
    fprintf(stderr, "\n");

    return true;
}

static bool meta_load_ldr_startthreads(ldr_ctx_t *ctx,
                                       ldr_startthread_t *threads, size_t num,
                                       void *user)
{
    size_t i;
    CPUArchState *env = user;
    MetaCore *core = META_THREAD2CORE(env);

    for (i = 0; i < num; i++) {
        if (unlikely(threads[i].thread >= core->num_threads)) {
            fprintf(stderr, "ldr_startthreads trying to start T%d > max T%d\n",
                    threads[i].thread,
                    core->num_threads - 1);
            return false;
        }
    }

    for (i = 0; i < num; i++) {
        env = &core->threads[threads[i].thread].env;
        env->pc[META_PC] = threads[i].pc;
        env->aregs[0][0] = threads[i].sp;
        meta_core_intreg_write(env, META_UNIT_CT, META_TXENABLE, 1);
    }

    return true;
}

static bool meta_load_ldr_zeromem(ldr_ctx_t *ctx, uint32_t addr, uint32_t sz,
                                  void *user)
{
    CPUArchState *env = user;
    uint8_t buf[64];
    uint32_t len;
    addr &= -4;

    len = (sz > sizeof(buf)) ? sizeof(buf) : sz;
    memset(buf, 0, len);

    /* first get aligned */
    len = -addr & 0x3;
    if (len) {
        if (meta_dasim_rw(env, addr, buf, len, 1) < 0) {
            fprintf(stderr, "ldr_zeromem %#x failed\n", addr);
        }
        sz -= len;
        addr += len;
    }
    /* then use zero buffer to write chunks */
    while (sz) {
        len = (sz > sizeof(buf)) ? sizeof(buf) : sz;
        if (meta_dasim_rw(env, addr, buf, len, 1) < 0) {
            fprintf(stderr, "ldr_zeromem %#x failed\n", addr);
        }
        sz -= len;
        addr += len;
    }

    return true;
}

static bool meta_load_ldr_read(ldr_ctx_t *ctx, uint32_t addr, void *user)
{
    CPUArchState *env = user;
    uint8_t val[4];
    if (meta_dasim_rw(env, addr, val, sizeof(val), 0) < 0) {
        fprintf(stderr, "ldr_read *0x%08x failed\n", addr);
        return false;
    }
    return true;
}

static bool meta_load_ldr_write(ldr_ctx_t *ctx, uint32_t addr, uint32_t val,
                                void *user)
{
    CPUArchState *env = user;
    if (meta_dasim_rw(env, addr, (uint8_t *)&val, sizeof(val), 1) < 0) {
        fprintf(stderr, "ldr_write *0x%08x = %#x failed\n", addr, val);
        return false;
    }
    return true;
}

static bool meta_load_ldr_memset(ldr_ctx_t *ctx, uint32_t addr, uint32_t count,
                                 uint32_t val, uint32_t inc, void *user)
{
    CPUArchState *env = user;

    addr &= -4;
    /* This could be optimised a lot */
    for (; count; --count) {
        if (meta_dasim_rw(env, addr, (uint8_t *)&val, 4, 1) < 0) {
            fprintf(stderr, "MemSet %#x=%#x failed\n", addr, val);
        }
        val += inc;
        addr += 4;
    }

    return true;
}


static ldr_cb_t callbacks = {
    .on_error = meta_load_ldr_error,
    .on_loadmem = meta_load_ldr_loadmem,
    .on_loadcore = meta_load_ldr_loadcore,
    .on_loadmmreg = meta_load_ldr_loadmmreg,
    .on_startthreads = meta_load_ldr_startthreads,
    .on_zeromem = meta_load_ldr_zeromem,
    /* no need to do anything for on_pause */
    .on_read = meta_load_ldr_read,
    .on_write = meta_load_ldr_write,
    .on_memset = meta_load_ldr_memset,
};

/* Load a flat LDR file created by LDLK */
static int meta_load_ldr(CPUArchState *env, const char *ldrname,
                         unsigned int page_size)
{
    ldr_ctx_t *ctx;

    ctx = ldr_open(ldrname, page_size);
    if (!ctx) {
        fprintf(stderr, "Failed to open input LDR '%s'\n", ldrname);
        goto err_ret;
    }

    if (!ldr_parse_header(ctx, &callbacks, env)) {
        fprintf(stderr, "Failed to parse header in LDR '%s'\n", ldrname);
        goto err_close;
    }

    if (!ldr_parse_slcode(ctx, &callbacks, env)) {
        fprintf(stderr, "Failed to parse secload code in LDR '%s' "
                "(page size = %u B)\n", ldrname, page_size);
        goto err_close;
    }

    if (!ldr_parse_sldata(ctx, &callbacks, env)) {
        fprintf(stderr, "Failed to parse secload data in LDR '%s' "
                "(page size = %u B)\n", ldrname, page_size);
        goto err_close;
    }

    ldr_close(ctx);
    return 0;

err_close:
    ldr_close(ctx);
err_ret:
    return 1;
}

static bool meta_ldr_boot_thread_startthreads(ldr_ctx_t *ctx,
                                              ldr_startthread_t *threads,
                                              size_t num, void *user)
{
    uint8_t *threadmask = user;
    size_t i;
    for (i = 0; i < num; i++) {
        *threadmask |= 1 << threads[i].thread;
    }
    return true;
}

static ldr_cb_t boot_thread_callbacks = {
    .on_startthreads = meta_ldr_boot_thread_startthreads,
};

/* Find boot thread from a flat LDR file created by LDLK */
static int meta_ldr_boot_thread(const char *ldrname, unsigned int page_size)
{
    ldr_ctx_t *ctx;
    uint8_t threadmask = 0;

    ctx = ldr_open(ldrname, page_size);
    if (!ctx) {
        fprintf(stderr, "Failed to open input LDR '%s'\n", ldrname);
        goto err_ret;
    }

    if (!ldr_parse_header(ctx, &boot_thread_callbacks, &threadmask)) {
        fprintf(stderr, "Failed to parse header in LDR '%s'\n", ldrname);
        goto err_close;
    }

    if (!ldr_parse_sldata(ctx, &boot_thread_callbacks, &threadmask)) {
        fprintf(stderr, "Failed to parse secload data in LDR '%s' "
                "(page size = %u B)\n", ldrname, page_size);
        goto err_close;
    }

    ldr_close(ctx);

    if (!threadmask) {
        /* no threads */
        return 0;
    }
    if (threadmask & (threadmask - 1)) {
        fprintf(stderr, "LDR starts multiple threads, "
                "please specify -bootthread.\n");
        return -1;
    }

    return ffs(threadmask) - 1;

err_close:
    ldr_close(ctx);
err_ret:
    return 0;
}

static void main_cpu_reset(void *opaque)
{
    CPUArchState *env = opaque;
    MetaCore *core = META_THREAD2CORE(env);
    MetaBootInfo *boot = env->boot_info;
    target_ulong entry;
    unsigned int len;

    if (!boot) {
        return;
    }

    if (boot->ldr_filename) {
        /* Load the LDR first */
        if (meta_load_ldr(env, boot->ldr_filename, boot->ldr_pagesize)) {
            exit(1);
        }

        if (boot->kernel_filename) {
            /* Load the kernel binary */
            len = meta_load_elf(env, boot->kernel_filename, &entry);

            /* Copy command line after the kernel */
            if (meta_dasim_rw(env, entry + len, (uint8_t *)boot->kernel_cmdline,
                              qemu_strnlen(boot->kernel_cmdline, 256), 1) < 0) {
                fprintf(stderr, "Could not copy kernel command line\n");
                core->gp_secure[0] = 0;
            }

            /* Use a GP secure register for storing pointer to cmdline */
            core->gp_secure[0] = entry + len;
        }
    } else if (boot->kernel_filename) {
        struct meta_mmu_seg local_segs[2], global_segs[2];
        target_ulong lheap_addr = 0x40000000;
        target_ulong gheap_addr = 0xc0000000;
        target_ulong gheap_brk = gheap_addr;
        target_ulong lheap_size, gheap_size, mmutable_size;
        hwaddr gheap_phys, gheap_v2p;
        target_ulong tbi, segs, args;

        /*
         * Allow the SoC to set up hardware that would normally be done by the
         * bootloader, such as DDR and clock registers.
         */
        if (boot->setup_direct_boot) {
            boot->setup_direct_boot(env, boot->setup_direct_boot_opaque);
        }

        /* Reserve either 32K (HTP) or 512K (ATP) for init page tables */
        mmutable_size = (boot->flags & META_BOOT_MMU_HTP) ? 0x8000 : 0x80000;

        /* Reserve a single 4K page for the global heap (TBI stuff) */
        gheap_size = 0x1000;

        /*
         * Local heap maps all of RAM except a bit at the end which is used for
         * page tables and global heap.
         */
        lheap_size = boot->ram_size - mmutable_size - gheap_size;
        gheap_phys = boot->ram_phys + lheap_size;
        gheap_v2p = gheap_phys - gheap_addr;
        meta_mmu_init_seg(&local_segs[0], lheap_addr, lheap_addr + lheap_size,
                          boot->ram_phys,
                          meta_tbi_seg_id(env->thread_num, SEG_SCOPE_LOCAL,
                                          SEG_TYPE_HEAP));
        meta_mmu_null_seg(&local_segs[1]);
        /* Global heap (for TBI structures) */
        meta_mmu_init_seg(&global_segs[0], gheap_addr, gheap_addr + gheap_size,
                          gheap_phys,
                          meta_tbi_seg_id(env->thread_num, SEG_SCOPE_GLOBAL,
                                          SEG_TYPE_HEAP));
        meta_mmu_null_seg(&global_segs[1]);

        /* Set up MMU tables */
        meta_mmu_setup(env, boot->flags & META_BOOT_MMU_HTP,
                       boot->ram_phys + boot->ram_size, local_segs,
                       global_segs);

        /* Set up TBI structures */
        tbi = meta_tbi_setup(env, &gheap_brk, gheap_v2p);
        segs = meta_tbi_seg_setup(env, &gheap_brk, gheap_v2p, local_segs,
                                  global_segs);

        /* Load the kernel binary */
        len = meta_load_elf(env, boot->kernel_filename, &entry);

        /* Copy command line after the kernel */
        args = entry + len;
        if (meta_dasim_rw(env, entry + len, (uint8_t *)boot->kernel_cmdline,
                          qemu_strnlen(boot->kernel_cmdline, 256), 1) < 0) {
            fprintf(stderr, "Could not copy kernel command line\n");
            args = 0;
        }

        /* Set up kernel entry point */
        env->pc[META_PC] = entry;   /* PC */
        env->dregs[1][3] = tbi;     /* D1Ar1: TBI (ISTAT) */
        env->dregs[0][3] = 0;       /* D0Ar2: TBI */
        env->dregs[1][2] = segs;    /* D1Ar3: TBI Segments */
        env->dregs[0][2] = args;    /* D0Ar4: Arguments or DTB */
        env->dregs[1][4] = 0;       /* D1RtP: Return pointer */
        meta_core_intreg_write(env, META_UNIT_CT, META_TXENABLE, 1);
    } else if (boot->entry) {
        if (META_COREMAJOR(env) >= 2 && boot->flags & META_BOOT_MINIM) {
            env->cregs[META_TXPRIVEXT] |= META_TXPRIVEXT_MINIMENABLE_MASK;
        }
        env->pc[META_PC] = boot->entry;
        env->cregs[META_TXENABLE] = META_TXENABLE_ENABLED;
        env->halted = 0;
        env->block = META_BLOCK_NONE;
    }

}

/* Setup the core loading of a Linux kernel image on reset */
int meta_setup_boot(MetaCore *core, MetaBootInfo *boot)
{
    CPUArchState *env;

    if (!boot->ldr_filename) {
        boot->ldr_filename = ldr_filename;
        boot->ldr_pagesize = ldr_pagesize;
    }

    boot->boot_thread = boot_thread;
    if (boot->boot_thread == -1) {
        if (boot->ldr_filename) {
            boot->boot_thread = meta_ldr_boot_thread(boot->ldr_filename,
                                                     boot->ldr_pagesize);
            if (boot->boot_thread == -1) {
                return 1;
            }
        } else {
            boot->boot_thread = 0;
        }
    }
    if (boot->boot_thread < 0 || boot->boot_thread >= core->num_threads) {
        fprintf(stderr, "Invalid boot thread %d\n", boot->boot_thread);
        return 1;
    }
    env = &core->threads[boot->boot_thread].env;

    env->boot_info = boot;
    qemu_register_reset(main_cpu_reset, env);

    return 0;
}
