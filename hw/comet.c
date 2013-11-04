/*
 * Toumaz Comet SoC with META21.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#include "blockdev.h"
#include "boards.h"
#include "exec-memory.h"
#include "loader.h"
#include "pc.h"
#include "soc_dma.h"
#include "sysemu.h"
#include "qemu-log.h"

#include "comet.h"
#include "core.h"
#include "meta_boot.h"
#include "comet_pdc.h"
#include "dw_mmc.h"
#include "dasim.h"
#include "dw_serial.h"

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define COMET_ROM_BASE               0xe0000000
#define COMET_ROM_SIZE               0xc000

#define COMET_CORECODE_BASE          0x80000000
#define COMET_CORECODE_SLAVE_BASE    0x02040000
#define COMET_CORECODE_SIZE          0x10000
#define COMET_CORECODE_END           0x82000000

#define COMET_COREDATA_BASE          0x82000000
#define COMET_COREDATA_SLAVE_BASE    0x02030000
#define COMET_COREDATA_SIZE          0x10000
#define COMET_COREDATA_END           0x84000000

#define COMET_INTRAM_BASE            0xe0200000
#define COMET_INTRAM_SIZE            0x60000

#define COMET_UCC0_MTXSRAM_BASE      0xe0260000
#define COMET_UCC0_MTXSRAM_SIZE      0x50000
#define COMET_UCC0_MCPSRAM16_BASE    0xe02b0000
#define COMET_UCC0_MCPSRAM16_SIZE    0x8e00
#define COMET_UCC0_MCPSRAM24_BASE    0xe02c0000
#define COMET_UCC0_MCPSRAM24_SIZE    0x2b800

#define COMET_UCC1_MTXSRAM_BASE      0xe0300000
#define COMET_UCC1_MTXSRAM_SIZE      0x50000

#define COMET_SDRAM_BASE             0xb0000000
#define COMET_SDRAM_SIZE             0x10000000

#define COMET_PERIPHERALS_BASE       0x02004000
#define COMET_PERIPHERALS_SIZE       0x400

#define COMET_SCB0_BASE              0x02004400
#define COMET_SCB1_BASE              0x02004600
#define COMET_SCB2_BASE              0x02004800

#define COMET_UART0_BASE             0x02004B00
#define COMET_UART0_SIZE             0xff

#define COMET_UART1_BASE             0x02004C00
#define COMET_UART1_SIZE             0xff

#define COMET_SPIM1_BASE             0x02004E00

#define COMET_SDIO_HOST_BASE         0x02005400
#define COMET_SDIO_HOST_SIZE         0x200

#define COMET_GPIOS_BASE             0x02005800
#define COMET_GPIOS_SIZE             0x200

#define COMET_PDC_BASE               0x02006000
#define COMET_PDC_SIZE               0x1000

#define COMET_PDP_BASE               0x02008000
#define COMET_PDP_SIZE               0x800

#define COMET_PDI_BASE               0x02008800
#define COMET_PDI_SIZE               0x100

#define COMET_HEP_BASE               0x02008c00
#define COMET_HEP_SIZE               0x400

#define COMET_SYS_INF_BASE           0x02009000
#define COMET_SYS_INF_SIZE           0x100

#define COMET_MDC_BASE               0x0200c000
#define COMET_MDC_SIZE               0x1000

#define COMET_UCC0_BASE              0x02010000
#define COMET_UCC0_SIZE              0x3000

#define COMET_UCC1_BASE              0x02014000
#define COMET_UCC1_SIZE              0x3000

#define COMET_I2S_BASE               0x02004F00
#define COMET_I2S_SIZE               0x200

#define COMET_USB_BASE               0x02020000

#define COMET_SYS_EVENT_BASE         0x04000000
#define COMET_SYS_EVENT_SIZE         0x400000

typedef enum {
    CR_PDP_MEM_BASE_ADDR = 0x20,
} CometHep;

typedef enum {
    COMET_IRQ_SCB0      = 0,
    COMET_IRQ_SCB1      = 1,
    COMET_IRQ_SCB2      = 2,
    COMET_IRQ_SDIO_DEV  = 3,
    COMET_IRQ_UART0     = 4,
    COMET_IRQ_UART1     = 5,
    COMET_IRQ_SPIM0     = 6,
    COMET_IRQ_SPIS      = 7,
    COMET_IRQ_SPIM1     = 8,
    COMET_IRQ_GPIO0     = 13,
    COMET_IRQ_GPIO1     = 14,
    COMET_IRQ_GPIO2     = 15,
    COMET_IRQ_PDC       = 18,
    COMET_IRQ_SDIO_HOST = 20,
    COMET_IRQ_MDC0      = 21,
    COMET_IRQ_MDC1      = 22,
    COMET_IRQ_MDC2      = 23,
    COMET_IRQ_MDC3      = 24,
    COMET_IRQ_MDC4      = 25,
    COMET_IRQ_MDC5      = 26,
    COMET_IRQ_MDC6      = 27,
    COMET_IRQ_MDC7      = 28,
    COMET_IRQ_IR        = 29,
    COMET_IRQ_RTC       = 30,
    COMET_IRQ_WDOG      = 31,
    COMET_IRQ_PDP       = 35,
    COMET_IRQ_MAX       = 64,
} CometIrq;

struct comet_top_state {
    uint32_t clkswitch;
    uint32_t clkenab;
    uint32_t meta_clkdiv;
    uint32_t meta_clkdelete;
    uint32_t sysclk_div;
    uint32_t syspll_ctl[2];
};

static void comet_gpio_irq_bank_update(struct comet_state_s *s,
                                       unsigned int bank);
static void comet_gpio_irq_bank_clear(struct comet_state_s *s,
                                      unsigned int bank, uint32_t clr);

static uint64_t comet_peripheral_io_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    struct comet_state_s *soc = opaque;
    int stream;
    uint32_t ret = 0;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return 0;
    }

    switch (addr) {
    case CR_PERIP_RESET_CFG:
        ret = soc->reset_cfg;
        break;
    case CR_PERIP_DMA_ROUTE_SEL_2:
        for (stream = 0; stream < 8; stream++) {
            CometDmaChanSel sel = img_mdc_get_perip(soc->mdc, stream);
            ret |= sel << (stream << 2);
        }
        break;
    case CR_COMET_CORE_REV:
        /* PS1 */
        ret = 0x00010000;
        break;
    case CR_PERIP_SDHOST_DMA_WRITE_DATA:
        ret = dw_mmc_get_dma_wdata(soc->mmc);
        break;
    default:
        DBGLOG("unhandled perip read(0x%08" HWADDR_PRIx ")\n", COMET_PERIPHERALS_BASE + addr);
    }

    return ret;
}

static void comet_peripheral_io_write(void *opaque, hwaddr addr,
                                      uint64_t val, unsigned size)
{
    struct comet_state_s *soc = opaque;
    int stream;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return;
    }

    switch (addr) {
    case CR_PERIP_DMA_ROUTE_SEL_2:
        for (stream = 0; stream < 8; stream++) {
            CometDmaChanSel sel = (val >> (stream << 2)) & 0xf;
            img_mdc_set_perip(soc->mdc, stream, sel);
        }
        break;
    case CR_PERIP_SDHOST_DMA_RDATA:
        dw_mmc_set_dma_rdata(soc->mmc, val);
        break;

    default:
        DBGLOG("unhandled perip write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
               COMET_PERIPHERALS_BASE + addr, val);
    }

    return;
}

static const MemoryRegionOps comet_peripherals_io_ops = {
    .read = comet_peripheral_io_read,
    .write = comet_peripheral_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_gpio_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    struct comet_state_s *soc = opaque;
    uint32_t ret = 0;
    unsigned int bank = (addr >> 2) & 0x3;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return 0;
    }

    switch (addr) {
    case CR_PADS_GPIO_DIR0 ... CR_PADS_GPIO_DIR2:
        ret = soc->gpio_dir[bank];
        break;
    case CR_PADS_GPIO_SELECT0 ... CR_PADS_GPIO_SELECT2:
        ret = soc->gpio_select[bank];
        break;
    case CR_PADS_GPIO_BIT_EN0 ... CR_PADS_GPIO_BIT_EN2:
        ret = soc->gpio_en[bank];
        break;
    case CR_PADS_GPIO_DIN0 ... CR_PADS_GPIO_DIN2:
        ret = soc->gpio_din[bank]
            & soc->gpio_select[bank]; /* in gpio mode */
        /*
         * GPIO_SDH_CD needs to be consistent with card detect reported
         * through MMC block.
         */
        if (addr == CR_PADS_GPIO_DIN0 && dw_mmc_get_cd(soc->mmc, 0)) {
            ret |= (1 << 6);
        }
        break;
    case CR_PADS_GPIO_DOUT0 ... CR_PADS_GPIO_DOUT2:
        ret = soc->gpio_dout[bank];
        break;
    case CR_PADS_IRQ_PLRT0 ... CR_PADS_IRQ_PLRT2:
        ret = soc->irq_plrt[bank];
        break;
    case CR_PADS_IRQ_TYPE0 ... CR_PADS_IRQ_TYPE2:
        ret = soc->irq_type[bank];
        break;
    case CR_PADS_IRQ_EN0 ... CR_PADS_IRQ_EN2:
        ret = soc->irq_en[bank];
        break;
    case CR_PADS_IRQ_STS0 ... CR_PADS_IRQ_STS2:
        ret = soc->irq_sts[bank];
        break;
    case CR_TOP_CLKSWITCH:
        ret = soc->top->clkswitch;
        break;
    case CR_TOP_CLKENAB:
        ret = soc->top->clkenab;
        break;
    case CR_TOP_META_CLKDIV:
        ret = soc->top->meta_clkdiv;
        break;
    case CR_TOP_META_CLKDELETE:
        ret = soc->top->meta_clkdelete;
        break;
    case CR_TOP_SYSCLK_DIV:
        ret = soc->top->sysclk_div;
        break;
    case CR_TOP_SYSPLL_CTL0:
        ret = soc->top->syspll_ctl[0];
        break;
    case CR_TOP_SYSPLL_CTL1:
        ret = soc->top->syspll_ctl[1];
        break;
    default:
        DBGLOG("unhandled gpio read(0x%08" HWADDR_PRIx ")\n", COMET_GPIOS_BASE + addr);
    }

    return ret;
}

static void comet_gpio_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    struct comet_state_s *soc = opaque;
    unsigned int bank = (addr >> 2) & 0x3;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return;
    }

    switch (addr) {
    case CR_PADS_GPIO_DIR0 ... CR_PADS_GPIO_DIR2:
        soc->gpio_dir[bank] = val & 0x3fffffff;
        break;
    case CR_PADS_GPIO_SELECT0 ... CR_PADS_GPIO_SELECT2:
        soc->gpio_select[bank] = val & 0x3fffffff;
        comet_gpio_irq_bank_update(soc, bank);
        break;
    case CR_PADS_GPIO_BIT_EN0 ... CR_PADS_GPIO_BIT_EN2:
        soc->gpio_en[bank] = val & 0x3fffffff;
        comet_gpio_irq_bank_update(soc, bank);
        break;
    case CR_PADS_IRQ_PLRT0 ... CR_PADS_IRQ_PLRT2:
        soc->irq_plrt[bank] = val & 0x3fffffff;
        comet_gpio_irq_bank_update(soc, bank);
        break;
    case CR_PADS_IRQ_TYPE0 ... CR_PADS_IRQ_TYPE2:
        soc->irq_type[bank] = val & 0x3fffffff;
        comet_gpio_irq_bank_update(soc, bank);
        break;
    case CR_PADS_IRQ_EN0 ... CR_PADS_IRQ_EN2:
        soc->irq_en[bank] = val & 0x3fffffff;
        comet_gpio_irq_bank_update(soc, bank);
        break;
    case CR_PADS_IRQ_STS0 ... CR_PADS_IRQ_STS2:
        comet_gpio_irq_bank_clear(soc, bank, val);
        break;
    case CR_TOP_CLKSWITCH:
        soc->top->clkswitch = val & 0x3fff7fff;
        break;
    case CR_TOP_CLKENAB:
        soc->top->clkenab = val & 0x220b7330;
        break;
    case CR_TOP_META_CLKDIV:
        soc->top->meta_clkdiv = val & 0x00000001;
        break;
    case CR_TOP_META_CLKDELETE:
        soc->top->meta_clkdelete = val & 0x000003ff;
        break;
    case CR_TOP_SYSCLK_DIV:
        soc->top->sysclk_div = val & 0x000000ff;
        break;
    case CR_TOP_SYSPLL_CTL0:
        soc->top->syspll_ctl[0] = val & 0xfff1fff7;
        break;
    case CR_TOP_SYSPLL_CTL1:
        soc->top->syspll_ctl[1] = val & 0x1f00003f;
        break;
    default:
        DBGLOG("unhandled gpio write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
               COMET_GPIOS_BASE + addr, val);
    }

    return;
}

static const MemoryRegionOps comet_gpios_io_ops = {
    .read = comet_gpio_io_read,
    .write = comet_gpio_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_hep_io_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    struct comet_state_s *soc = opaque;
    uint32_t ret = 0;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return 0;
    }

    switch (addr) {
    case CR_PDP_MEM_BASE_ADDR:
        if (soc->pdp) {
            ret = soc->pdp->base_addr;
        }
        break;
    default:
        DBGLOG("unhandled hep read(0x%08x+%#" HWADDR_PRIx ")\n", COMET_HEP_BASE, addr);
    }

    return ret;
}

static void comet_hep_io_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    struct comet_state_s *soc = opaque;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return;
    }

    switch (addr) {
    case CR_PDP_MEM_BASE_ADDR:
        if (soc->pdp) {
            comet_pdp_set_base(soc->pdp, val << 2);
        }
        break;
    default:
        DBGLOG("unhandled hep write(0x%08x+%#" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
               COMET_HEP_BASE, addr, val);
    }

    return;
}

static const MemoryRegionOps comet_hep_io_ops = {
    .read = comet_hep_io_read,
    .write = comet_hep_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_sys_inf_io_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    uint32_t ret = 0;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return 0;
    }

    switch (addr) {
    default:
        DBGLOG("unhandled sys inf read(0x%08x+%#" HWADDR_PRIx ")\n",
               COMET_SYS_INF_BASE, addr);
    }

    return ret;
}

static void comet_sys_inf_io_write(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return;
    }

    switch (addr) {
    default:
        DBGLOG("unhandled sys inf write(0x%08x+%#" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
               COMET_SYS_INF_BASE, addr, val);
    }

    return;
}

static const MemoryRegionOps comet_sys_inf_io_ops = {
    .read = comet_sys_inf_io_read,
    .write = comet_sys_inf_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_sys_event_io_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    uint32_t ret = 0;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return 0;
    }

    switch (addr) {
    default:
        DBGLOG("unhandled sys event read(0x%08" HWADDR_PRIx ")\n",
               COMET_SYS_EVENT_BASE + addr);
    }

    return ret;
}

static void comet_sys_event_io_write(void *opaque, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    struct comet_state_s *soc = (struct comet_state_s *) opaque;

    /* FIXME handle bad width */

    if (unlikely(addr & 0x3)) {
        return;
    }

    switch (addr) {
    case LINSYSCFLUSH_DCACHE_LINE ... LINSYSCFLUSH_DCACHE_LINE + LINSYSCFLUSH_ADDR_BITS:
        trace_flush(NULL, false, true, (addr - LINSYSCFLUSH_DCACHE_LINE),
                    soc->core->threads[0].env.cregs[META_TXTACTCYC]);
        break;
    case LINSYSCFLUSH_ICACHE_LINE ... LINSYSCFLUSH_ICACHE_LINE + LINSYSCFLUSH_ADDR_BITS:
        trace_flush(NULL, true, true, (addr - LINSYSCFLUSH_ICACHE_LINE),
                    soc->core->threads[0].env.cregs[META_TXTACTCYC]);
        break;
    default:
        DBGLOG("unhandled sys event write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n",
               COMET_SYS_EVENT_BASE + addr, val);
    }

    return;
}

static const MemoryRegionOps comet_sys_event_io_ops = {
    .read = comet_sys_event_io_read,
    .write = comet_sys_event_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_uccp_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    uint32_t ret = 0;

    switch (addr) {
    default:
        DBGLOG("unhandled uccp read(0x%08" HWADDR_PRIx ")\n", addr);
    }

    return ret;
}

static void comet_uccp_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    switch (addr) {
    default:
        DBGLOG("unhandled uccp write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    }
}

static const MemoryRegionOps comet_uccp_io_ops = {
    .read = comet_uccp_io_read,
    .write = comet_uccp_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_pdi_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    uint32_t ret = 0;

    switch (addr) {
    default:
        DBGLOG("unhandled pdi read(0x%08" HWADDR_PRIx ")\n", addr);
    }

    return ret;
}

static void comet_pdi_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    switch (addr) {
    default:
        DBGLOG("unhandled pdi write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    }
}

static const MemoryRegionOps comet_pdi_io_ops = {
    .read = comet_pdi_io_read,
    .write = comet_pdi_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t comet_i2s_io_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    uint32_t ret = 0;

    switch (addr) {
    default:
        DBGLOG("unhandled i2s read(0x%08" HWADDR_PRIx ")\n", addr);
    }

    return ret;
}

static void comet_i2s_io_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    switch (addr) {
    default:
        DBGLOG("unhandled i2s write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    }
}

static const MemoryRegionOps comet_i2s_io_ops = {
    .read = comet_i2s_io_read,
    .write = comet_i2s_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void comet_setup_io(struct comet_state_s *soc, MemoryRegion *memory)
{
    struct soc_dma_s *dma;
    int uart_baudbase = 1843200;
    int i;
    /* this determines the number of SD cards configured */
    BlockDriverState *bd[1];
    DriveInfo *dinfo;

    memory_region_init_io(&soc->peripherals_iomem, &comet_peripherals_io_ops, soc,
                          "comet-peripherals", COMET_PERIPHERALS_SIZE - 1);
    memory_region_add_subregion(memory, COMET_PERIPHERALS_BASE, &soc->peripherals_iomem);

    memory_region_init_io(&soc->gpios_iomem, &comet_gpios_io_ops, soc,
                          "comet-gpios", COMET_GPIOS_SIZE - 1);
    memory_region_add_subregion(memory, COMET_GPIOS_BASE, &soc->gpios_iomem);

    memory_region_init_io(&soc->hep_iomem, &comet_hep_io_ops, soc,
                          "comet-hep", COMET_HEP_SIZE - 1);
    memory_region_add_subregion(memory, COMET_HEP_BASE, &soc->hep_iomem);

    memory_region_init_io(&soc->sys_inf_iomem, &comet_sys_inf_io_ops, soc,
                          "comet-sys-inf", COMET_SYS_INF_SIZE - 1);
    memory_region_add_subregion(memory, COMET_SYS_INF_BASE, &soc->sys_inf_iomem);

    memory_region_init_io(&soc->sys_event_iomem, &comet_sys_event_io_ops, soc,
                          "comet-sys-event", COMET_SYS_EVENT_SIZE - 1);
    memory_region_add_subregion(memory, COMET_SYS_EVENT_BASE, &soc->sys_event_iomem);

    /* UCCP sys bus registers */
    memory_region_init_io(&soc->ucc0_iomem, &comet_uccp_io_ops, soc,
                          "comet-ucc0", COMET_UCC0_SIZE - 1);
    memory_region_add_subregion(memory, COMET_UCC0_BASE, &soc->ucc0_iomem);

    memory_region_init_io(&soc->ucc1_iomem, &comet_uccp_io_ops, soc,
                          "comet-ucc1", COMET_UCC1_SIZE - 1);
    memory_region_add_subregion(memory, COMET_UCC1_BASE, &soc->ucc1_iomem);

    /* I2S audio in/out regions */
    memory_region_init_io(&soc->i2s_iomem, &comet_i2s_io_ops, soc,
                          "comet-i2s", COMET_I2S_SIZE - 1);
    memory_region_add_subregion(memory, COMET_I2S_BASE, &soc->i2s_iomem);

    /* MDC */
    soc->mdc = img_mdc_init(COMET_MDC_CHANNELS, COMET_DMACHANSEL_MAX,
                            COMET_MDC_BASE, &soc->core->triggers.irqs[COMET_IRQ_MDC0]);
    dma = img_mdc_dma(soc->mdc);
    soc->drq = dma->drq;
    memory_region_add_subregion(memory, COMET_MDC_BASE, img_mdc_iomem(soc->mdc));

    /* SCB */
    soc->scb2 = img_scb_init(COMET_SCB2_BASE, soc->core->triggers.irqs[COMET_IRQ_SCB2]);
    memory_region_add_subregion(memory, COMET_SCB2_BASE, img_scb_iomem(soc->scb2));

    /* SPI */
    soc->spim1 = comet_spim_init(soc->core->triggers.irqs[COMET_IRQ_SPIM1],
                                 soc->drq[COMET_DMACHANSEL_SPIMASTER_READ2],
                                 soc->drq[COMET_DMACHANSEL_SPIMASTER_WRITE2]);
    memory_region_add_subregion(memory, COMET_SPIM1_BASE, &soc->spim1->iomem);

    /* USB */
    soc->usb = dw_otg_init(COMET_USB_BASE);
    memory_region_add_subregion(memory, COMET_USB_BASE, dw_otg_iomem(soc->usb));

    soc->uart0 = dw_serial_mm_init(get_system_memory(), COMET_UART0_BASE,
                                   soc->core->triggers.irqs[COMET_IRQ_UART0],
                                   uart_baudbase,
                                   serial_hds[0] ?:
                                       qemu_chr_new("null0", "null", NULL),
                                   DEVICE_NATIVE_ENDIAN);
    soc->uart1 = dw_serial_mm_init(get_system_memory(), COMET_UART1_BASE,
                                  soc->core->triggers.irqs[COMET_IRQ_UART1],
                                  uart_baudbase,
                                  serial_hds[1] ?:
                                      qemu_chr_new("null1", "null", NULL),
                                  DEVICE_NATIVE_ENDIAN);

    soc->pdp = comet_pdp_init(memory, COMET_PDP_BASE, COMET_PDP_SIZE,
                              soc->core->triggers.irqs[COMET_IRQ_PDP],
                              &soc->core->threads[0].env);

    memory_region_init_io(&soc->pdi_iomem, &comet_pdi_io_ops, soc,
                          "comet-pdi", COMET_PDI_SIZE - 1);
    memory_region_add_subregion(memory, COMET_PDI_BASE, &soc->pdi_iomem);

    comet_pdc_init(memory, COMET_PDC_BASE, COMET_PDC_SIZE,
                   soc->core->triggers.irqs[COMET_IRQ_PDC],
                   soc->core->triggers.irqs[COMET_IRQ_RTC],
                   soc->core->triggers.irqs[COMET_IRQ_IR],
                   soc->core->triggers.irqs[COMET_IRQ_WDOG],
                   4);

    /* SD Card */
    for (i = 0; i < ARRAY_SIZE(bd); ++i) {
        dinfo = drive_get(IF_SD, 0, i);
        if (!dinfo) {
            fprintf(stderr, "qemu: missing SecureDigital device\n");
            exit(1);
        }
        bd[i] = dinfo->bdrv;
    }
    soc->mmc = dw_mmc_init(memory, COMET_SDIO_HOST_BASE, COMET_SDIO_HOST_SIZE,
                           ARRAY_SIZE(bd), 32, bd,
                           soc->core->triggers.irqs[COMET_IRQ_SDIO_HOST],
                           dma->drq[COMET_DMACHANSEL_SDHOST_READ],
                           dma->drq[COMET_DMACHANSEL_SDHOST_WRITE]);
}

static void comet_top_reset(struct comet_top_state *top)
{
    top->clkswitch = 0;
    top->clkenab = 0x00001000;
    top->meta_clkdiv = 0;
    top->meta_clkdelete = 0;
    top->sysclk_div = 0;
    top->syspll_ctl[0] = 0x0c201853;
    top->syspll_ctl[1] = 0x06000017;
}

static void comet_gpio_reset(struct comet_state_s *s, bool hard)
{
    unsigned int i;

    s->gpio_dir[0] = s->gpio_dir[1] = s->gpio_dir[2] = 0x3fffffff;
    s->gpio_select[0] = 0x3fffffc0;
    s->gpio_select[1] = s->gpio_select[2] = 0x3fffffff;
    memset(s->gpio_en, 0, sizeof(s->gpio_en));
    /* raw inputs are controlled by other devices. it's up to them to reset. */
    if (hard) {
        memset(s->gpio_din, 0, sizeof(s->gpio_din));
    }
    memset(s->gpio_dout, 0, sizeof(s->gpio_dout));
    memset(s->irq_plrt, 0, sizeof(s->irq_plrt));
    memset(s->irq_type, 0, sizeof(s->irq_type));
    memset(s->irq_en, 0, sizeof(s->irq_en));
    memset(s->irq_sts, 0, sizeof(s->irq_sts));
    memset(s->irq_input, 0, sizeof(s->irq_input));

    /* clear output interrupts */
    for (i = 0; i < 3; ++i) {
        qemu_set_irq(s->core->triggers.irqs[COMET_IRQ_GPIO0 + i], 0);
    }
}

static void comet_reset(void *opaque)
{
    struct comet_state_s *soc = (struct comet_state_s *) opaque;

    cpu_meta_core_reset(soc->core);
    comet_top_reset(soc->top);
    comet_gpio_reset(soc, false);
    dw_mmc_reset(soc->mmc);
}

/* provide a new input to a gpio bank's irq logic */
static void comet_gpio_irq_in(struct comet_state_s *s, unsigned int bank,
                              uint32_t irq_in)
{
    uint32_t level_irqs, prev, changed_irqs, active;
    level_irqs = ~s->irq_type[bank];

    /* compare and update irq input */
    prev = s->irq_input[bank];
    s->irq_input[bank] = irq_in;

    /* trigger edge and level interrupts */
    changed_irqs = prev ^ irq_in;
    s->irq_sts[bank] |= irq_in & (level_irqs | changed_irqs);

    /* active interrupts are both triggered and enabled, and trigger bank irq */
    active = s->irq_sts[bank] & s->irq_en[bank];
    qemu_set_irq(s->core->triggers.irqs[COMET_IRQ_GPIO0 + bank], !!active);
}

static void comet_gpio_irq_bank_update(struct comet_state_s *s,
                                       unsigned int bank)
{
    uint32_t irq_in;

    /* find the input into the gpio irq trigger logic */
    irq_in = s->gpio_select[bank]                      /* GPIO */
           & s->gpio_en[bank]                          /* bit enable */
           & ~(s->gpio_din[bank] ^ s->irq_plrt[bank]); /* match polarity */

    comet_gpio_irq_in(s, bank, irq_in);
}

static void comet_gpio_irq_bank_clear(struct comet_state_s *s,
                                      unsigned int bank, uint32_t clr)
{
    s->irq_sts[bank] &= clr;

    comet_gpio_irq_bank_update(s, bank);
}

static void comet_gpio_handler(void *opaque, int n, int level)
{
    struct comet_state_s *s = opaque;
    unsigned int bank = n / 30;
    unsigned int idx = n % 30;
    uint32_t bit, inbit, din;

    din = s->gpio_din[bank];
    bit = 1 << idx;
    inbit = !!level << idx;

    /* update raw input bit */
    s->gpio_din[bank] = (din & ~bit) | inbit;

    /* and update irq triggering */
    comet_gpio_irq_bank_update(s, bank);
}

struct comet_state_s *comet_init(unsigned long sdram_size,
                                 const char *core,
                                 const char *kernel_filename,
                                 const char *kernel_cmdline)
{
    struct comet_state_s *s = (struct comet_state_s *)
            g_malloc0(sizeof(struct comet_state_s));
    int sz, i;
    struct soc_dma_s *dma;
    hwaddr addr;
    MemoryRegion *memory = get_system_memory();

    s->reset_cfg = COMET_DEFAULT_RESET_CFG;

    if (!core) {
        core = "harrier";
    }

    s->core = cpu_meta_core_init(core, COMET_IRQ_MAX);
    if (!s->core) {
        return NULL;
    }

    /* top level registers */
    s->top = g_malloc0(sizeof(*s->top));

    /* GPIOs */
    s->gpios = qemu_allocate_irqs(comet_gpio_handler, s, COMET_NUM_GPIOS);
    comet_gpio_reset(s, true);

    /* setup io */
    comet_setup_io(s, memory);
    dma = img_mdc_dma(s->mdc);

    /* setup memory */
    memory_region_init_ram(&s->rom_iomem, "meta_core_rom.ram", COMET_ROM_SIZE);
    memory_region_add_subregion(memory, COMET_ROM_BASE, &s->rom_iomem);
    sz = load_image_targphys("roms/meta/comet_rom.bin", COMET_ROM_BASE,
                             COMET_ROM_SIZE);
    if (sz == -1) {
        fprintf(stderr, "Unable to load ROM!\n");
    }

    memory_region_init_ram(&s->core_code_iomem[0], "meta_code.ram", COMET_CORECODE_SIZE);
    memory_region_add_subregion(memory, COMET_CORECODE_BASE, &s->core_code_iomem[0]);
    for (addr = COMET_CORECODE_BASE + COMET_CORECODE_SIZE, i = 1;
         addr < COMET_CORECODE_END;
         addr += COMET_CORECODE_SIZE, i++) {
        assert(i < ARRAY_SIZE(s->core_code_iomem));
        memory_region_init_alias(&s->core_code_iomem[i], "meta_code.ram",
                                 &s->core_code_iomem[0], 0, COMET_CORECODE_SIZE);
        memory_region_add_subregion(memory, addr, &s->core_code_iomem[i]);
    }
    memory_region_init_alias(&s->core_code_slave_iomem, "meta_code_slave.iomem",
                             &s->core_code_iomem[0], 0, COMET_CORECODE_SIZE);
    memory_region_add_subregion(memory, COMET_CORECODE_SLAVE_BASE, &s->core_code_slave_iomem);

    memory_region_init_ram(&s->core_data_iomem[0], "meta_data.ram", COMET_COREDATA_SIZE);
    memory_region_add_subregion(memory, COMET_COREDATA_BASE, &s->core_data_iomem[0]);
    for (addr = COMET_COREDATA_BASE + COMET_COREDATA_SIZE, i = 1;
         addr < COMET_COREDATA_END;
         addr += COMET_COREDATA_SIZE, i++) {
        assert(i < ARRAY_SIZE(s->core_data_iomem));
        memory_region_init_alias(&s->core_data_iomem[i], "meta_data.ram",
                                 &s->core_data_iomem[0], 0, COMET_COREDATA_SIZE);
        memory_region_add_subregion(memory, addr, &s->core_data_iomem[i]);
    }
    memory_region_init_alias(&s->core_data_slave_iomem, "meta_data_slave.iomem",
                             &s->core_data_iomem[0], 0, COMET_COREDATA_SIZE);
    memory_region_add_subregion(memory, COMET_COREDATA_SLAVE_BASE, &s->core_data_slave_iomem);

    /* register core memories with core */
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_CODE,
                           COMET_CORECODE_BASE, COMET_CORECODE_SIZE);
    cpu_meta_core_mem_init(s->core, META_CORE_MEM_DATA,
                           COMET_COREDATA_BASE, COMET_COREDATA_SIZE);

    memory_region_init_ram(&s->internal_iomem, "meta_internal.ram", COMET_INTRAM_SIZE);
    memory_region_add_subregion(memory, COMET_INTRAM_BASE, &s->internal_iomem);
    soc_dma_port_add_mem(dma, memory_region_get_ram_ptr(&s->internal_iomem),
                         COMET_INTRAM_BASE, COMET_INTRAM_SIZE);

    memory_region_init_ram(&s->ucc0_mtxsram_iomem, "meta_ucc0_mtx.ram",
                           COMET_UCC0_MTXSRAM_SIZE);
    memory_region_add_subregion(memory, COMET_UCC0_MTXSRAM_BASE, &s->ucc0_mtxsram_iomem);

    memory_region_init_ram(&s->ucc0_mcpsram16_iomem, "meta_ucc0_mcp16.ram",
                           COMET_UCC0_MCPSRAM16_SIZE);
    memory_region_add_subregion(memory, COMET_UCC0_MCPSRAM16_BASE, &s->ucc0_mcpsram16_iomem);

    memory_region_init_ram(&s->ucc0_mcpsram24_iomem, "meta_ucc0_mcp24.ram",
                           COMET_UCC0_MCPSRAM24_SIZE);
    memory_region_add_subregion(memory, COMET_UCC0_MCPSRAM24_BASE, &s->ucc0_mcpsram24_iomem);

    memory_region_init_ram(&s->ucc1_mtxsram_iomem, "meta_ucc1_mtx.ram",
                           COMET_UCC1_MTXSRAM_SIZE);
    memory_region_add_subregion(memory, COMET_UCC1_MTXSRAM_BASE, &s->ucc1_mtxsram_iomem);

    memory_region_init_ram(&s->sdram_iomem, "sdram.ram", sdram_size);
    memory_region_add_subregion(memory, COMET_SDRAM_BASE, &s->sdram_iomem);
    soc_dma_port_add_mem(dma, memory_region_get_ram_ptr(&s->sdram_iomem), COMET_SDRAM_BASE,
                         COMET_SDRAM_SIZE);

    qemu_register_reset(comet_reset, s);

    /* set up boot */
    s->boot.kernel_filename = kernel_filename;
    s->boot.kernel_cmdline = kernel_cmdline;
    if (!sap_comms) {
        s->boot.entry = 0xe0000000;
        s->boot.flags |= META_BOOT_MINIM;
    }
    meta_setup_boot(s->core, &s->boot);

    return s;
}
