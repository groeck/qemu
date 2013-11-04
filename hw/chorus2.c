/*
 * Frontier Silicon Chorus2 SoC with META122.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#include "boards.h"
#include "loader.h"
#include "soc_dma.h"
#include "sysemu.h"
#include "qemu-log.h"
#include "exec-memory.h"

#include "chorus2.h"
#include "chorus2_dmac.h"
#include "chorus2_spi.h"
#include "core.h"
#include "dasim.h"

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define C2_ROM_BASE         0x80000000
#define C2_ROM_SIZE         0x20000

#define C2_CORECODE_BASE    0x80020000
#define C2_CORECODE_SIZE    0x10000

#define C2_COREDATA_BASE    0x82000000
#define C2_COREDATA_SIZE    0x10000

#define C2_INTRAM_BASE      0xe0000000
#define C2_INTRAM_SIZE      0x50000

#define C2_SDRAM_BASE       0xb0000000
#define C2_SDRAM_SIZE       0x10000000

#define C2_SYSBUS_BASE      0x02000000
#define C2_SYSBUS_SIZE      0x1000

#define C2_DMAC_BASE        (C2_SYSBUS_BASE + 0x01000)
#define C2_DMAC_CHANNELS    12

#define C2_SCP_BASE         (C2_SYSBUS_BASE + 0x05000)
#define C2_SCP_SIZE         0x1000

#define C2_SPI_BASE         (C2_SYSBUS_BASE + 0x08000)
#define C2_SPI_SIZE         0x1000

#define C2_PINCTRL_BASE     (C2_SYSBUS_BASE + 0x24000)
#define C2_PINCTRL_SIZE     0x1000

#define C2_SDRAMCTRL_BASE   (C2_SYSBUS_BASE + 0x28000)
#define C2_SDRAMCTRL_SIZE   0x1000

typedef enum {
    C2_IRQ_MAX      = 128,
} Chorus2Irq;

static void chorus2_reset(void *opaque)
{
    struct chorus2_state_s *soc = opaque;
    int i;

    cpu_meta_core_reset(soc->core);
    soc->sw_reset = 0;
    for (i = 0; i < C2_DMAC_CHANNELS; ++i) {
        soc->dma_chan_sel[i] = 0;
    }
    chorus2_spi_reset(soc->spi);
    chorus2_dmac_reset(soc->dmac);
}

/* register numbers (<<2 for offset) */
typedef enum {
    C2_SOCID            = 0x000 >> 2,
    C2_SW_RESET         = 0x004 >> 2,
    C2_SW_RESET_PROT    = 0x008 >> 2,
    C2_BOOT_SETUP       = 0x038 >> 2,
    C2_DMACHANSEL_0_3   = 0x048 >> 2,
    C2_DMACHANSEL_4_7   = 0x04c >> 2,
    C2_DMACHANSEL_8_11  = 0x050 >> 2,
    C2_DMASPIDENDSEL    = 0x058 >> 2,
} Chorus2LocalBusReg;

static uint64_t chorus2_sysctrl_io_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    struct chorus2_state_s *soc = opaque;
    unsigned int offset = addr >> 2;
    uint32_t ret = 0;

    /* FIXME handle bad widths */

    if (unlikely(addr & 0x3)) {
        return 0;
    }

    switch (offset) {
    case C2_SOCID:
        ret = soc->revision;
        break;
    case C2_SW_RESET_PROT:
        ret = soc->sw_reset_prot;
        break;
    case C2_BOOT_SETUP:
        ret = soc->boot_setup;
        break;
    case C2_DMACHANSEL_0_3:
    case C2_DMACHANSEL_4_7:
    case C2_DMACHANSEL_8_11:
        {
            int chan = 4 * (offset - C2_DMACHANSEL_0_3);
            int i;
            for (i = 0; i < 32; i += 8, ++chan) {
                ret |= soc->dma_chan_sel[chan] << i;
            }
            break;
        }
    default:
        DBGLOG("unhandled sys read(0x%08" HWADDR_PRIx ")\n", C2_SYSBUS_BASE + addr);
    }

    return ret;
}

static void chorus2_sysctrl_io_write(void *opaque, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    struct chorus2_state_s *soc = opaque;
    unsigned int offset = addr >> 2;

    /* FIXME handle bad widths */

    if (unlikely(addr & 0x3)) {
        return;
    }

    switch (offset) {
    case C2_SW_RESET:
        val &= 1;
        soc->sw_reset = val;
        if (val) {
            qemu_system_reset_request();
        }
        break;
    case C2_SW_RESET_PROT:
        soc->sw_reset_prot = val;
        break;
    case C2_DMACHANSEL_0_3:
    case C2_DMACHANSEL_4_7:
    case C2_DMACHANSEL_8_11:
        {
            int chan = 4 * (offset - C2_DMACHANSEL_0_3);
            int i;
            for (i = 0; i < 32; i += 8, ++chan) {
                Chorus2DmaChanSel sel = (val >> i) & 0x3f;
                soc->dma_chan_sel[chan] = sel;
                chorus2_dmac_set_perip(soc->dmac, chan, sel);
            }
        }
        break;
    case C2_DMASPIDENDSEL:
        /*
         * SPI already works in the boot sequence without this register
         * being implemented, but it may need implementing in the future.
         */
        break;
    default:
        DBGLOG("unhandled sys write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
               C2_SYSBUS_BASE + addr, val);
    }

    return;
}

static const MemoryRegionOps chorus2_sysctrl_io_ops = {
    .read = chorus2_sysctrl_io_read,
    .write = chorus2_sysctrl_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/* SCP Registers */

static uint64_t chorus2_scp_io_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    DBGLOG("unhandled scp write(0x%08" HWADDR_PRIx ")\n", C2_SCP_BASE + addr);
    return 0;
}

static void chorus2_scp_io_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    DBGLOG("unhandled scp write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", C2_SCP_BASE + addr, val);
}

static const MemoryRegionOps chorus2_scp_io_ops = {
    .read = chorus2_scp_io_read,
    .write = chorus2_scp_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/* Pin Function Control Registers */

static uint64_t chorus2_pinctrl_io_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    DBGLOG("unhandled pinctrl write(0x%08" HWADDR_PRIx ")\n", C2_PINCTRL_BASE + addr);
    return 0;
}

static void chorus2_pinctrl_io_write(void *opaque, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    DBGLOG("unhandled pinctrl write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
           C2_PINCTRL_BASE + addr, val);
}

static const MemoryRegionOps chorus2_pinctrl_io_ops = {
    .read = chorus2_pinctrl_io_read,
    .write = chorus2_pinctrl_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/* SDRAM Control Registers */

static uint64_t chorus2_sdramctrl_io_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    DBGLOG("unhandled sdramctrl write(0x%08" HWADDR_PRIx ")\n", C2_SDRAMCTRL_BASE + addr);
    return 0;
}

static void chorus2_sdramctrl_io_write(void *opaque, hwaddr addr,
                                       uint64_t val, unsigned size)
{
    DBGLOG("unhandled sdramctrl write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
           C2_SDRAMCTRL_BASE + addr, val);
}

static const MemoryRegionOps chorus2_sdramctrl_io_ops = {
    .read = chorus2_sdramctrl_io_read,
    .write = chorus2_sdramctrl_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void chorus2_setup_io(struct chorus2_state_s *soc, MemoryRegion *memory)
{
    MemoryRegion sysctrl_iomem;
    MemoryRegion scp_iomem;
    MemoryRegion pinctrl_iomem;
    MemoryRegion sdramctrl_iomem;
    struct soc_dma_s *dma;

    memory_region_init_io(&sysctrl_iomem, &chorus2_sysctrl_io_ops, soc,
                          "chorus2-sysctrl", C2_SYSBUS_SIZE);
    memory_region_add_subregion(memory, C2_SYSBUS_BASE, &sysctrl_iomem);

    memory_region_init_io(&scp_iomem, &chorus2_scp_io_ops, soc,
                          "chorus2-scp", C2_SCP_SIZE);
    memory_region_add_subregion(memory, C2_SCP_BASE, &scp_iomem);

    memory_region_init_io(&pinctrl_iomem, &chorus2_pinctrl_io_ops, soc,
                          "chorus2-pinctrl", C2_PINCTRL_SIZE);
    memory_region_add_subregion(memory, C2_PINCTRL_BASE, &pinctrl_iomem);

    memory_region_init_io(&sdramctrl_iomem, &chorus2_sdramctrl_io_ops, soc,
                          "chorus2-sdramctrl", C2_SDRAMCTRL_SIZE);
    memory_region_add_subregion(memory, C2_SDRAMCTRL_BASE, &sdramctrl_iomem);

    soc->dmac = chorus2_dmac_init(C2_DMAC_CHANNELS, C2_DMACHANSEL_MAX,
                                  C2_SYSBUS_BASE);
    dma = chorus2_dmac_dma(soc->dmac);
    soc->drq = dma->drq;

    soc->spi = chorus2_spi_init(soc->drq[C2_DMACHANSEL_SPI1O],
                                soc->drq[C2_DMACHANSEL_SPI1I]);

    memory_region_add_subregion(memory, C2_DMAC_BASE, chorus2_dmac_iomem(soc->dmac));
    memory_region_add_subregion(memory, C2_SPI_BASE, &soc->spi->iomem);
}

struct chorus2_state_s *chorus2_init(unsigned long sdram_size,
                                     const char *core,
                                     const char *kernel_filename,
                                     const char *kernel_cmdline)
{
    struct chorus2_state_s *s = (struct chorus2_state_s *)
            g_malloc0(sizeof(struct chorus2_state_s));
    struct soc_dma_s *dma;
    MemoryRegion *memory = get_system_memory();
    MemoryRegion rom_iomem;
    MemoryRegion core_code_iomem;
    MemoryRegion core_data_iomem;
    MemoryRegion internal_iomem;
    MemoryRegion sdram_iomem;

    if (!core) {
        core = "meta122";
    }

    s->core = cpu_meta_core_init(core, C2_IRQ_MAX);
    if (!s->core) {
        return NULL;
    }

    /* setup io */
    s->revision = C2_SOCID_DEFAULT;
    s->boot_setup = C2_BOOTSETUP_DEFAULT;
    chorus2_setup_io(s, memory);
    dma = chorus2_dmac_dma(s->dmac);

    /* setup memory */
    /* FIXME ROM is read from META as all zeros */
    memory_region_init_ram(&rom_iomem, "meta_core_rom.ram", C2_ROM_SIZE);
    memory_region_add_subregion(memory, C2_ROM_BASE, &rom_iomem);
    load_image_targphys("roms/meta/c2_rom.bin", C2_ROM_BASE, C2_ROM_SIZE);

    memory_region_init_ram(&core_code_iomem, "meta_code.ram", C2_CORECODE_SIZE);
    memory_region_add_subregion(memory, C2_CORECODE_BASE, &core_code_iomem);

    memory_region_init_ram(&core_data_iomem, "meta_data.ram", C2_COREDATA_SIZE);
    memory_region_add_subregion(memory, C2_COREDATA_BASE, &core_data_iomem);

    /* register core memories with core */
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_CODE_ROM,
                           C2_ROM_BASE, C2_ROM_SIZE);
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_CODE,
                           C2_CORECODE_BASE, C2_CORECODE_SIZE);
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_DATA,
                           C2_COREDATA_BASE, C2_COREDATA_SIZE);

    memory_region_init_ram(&internal_iomem, "meta_internal.ram", C2_INTRAM_SIZE);
    memory_region_add_subregion(memory, C2_INTRAM_BASE, &internal_iomem);
    soc_dma_port_add_mem(dma, memory_region_get_ram_ptr(&internal_iomem), C2_INTRAM_BASE,
                         C2_INTRAM_SIZE);

    memory_region_init_ram(&sdram_iomem, "sdram.ram", sdram_size);
    memory_region_add_subregion(memory, C2_SDRAM_BASE, &sdram_iomem);
    soc_dma_port_add_mem(dma, memory_region_get_ram_ptr(&sdram_iomem), C2_SDRAM_BASE,
                         C2_SDRAM_SIZE);

    qemu_register_reset(chorus2_reset, s);

    /* set up boot */
    s->boot.kernel_filename = kernel_filename;
    s->boot.kernel_cmdline = kernel_cmdline;
    s->boot.ram_phys = C2_SDRAM_BASE;
    s->boot.ram_size = sdram_size;
    if (!sap_comms) {
        s->boot.entry = 0x80000000;
    }
    meta_setup_boot(s->core, &s->boot);

    return s;
}
