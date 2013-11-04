/*
 * Synopsys DesignWare APB UART emulation.
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#include "dw_serial.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

/* the size of the iomem from serial.c */
#define UART_8250_SZ 0x20

struct DwSerialState
{
    SerialState *uart;
    MemoryRegion dw_iomem;
    uint16_t version;
};

enum {
    DW_CPR = 0xf4, /* Component Parameter Register */
    DW_UCV = 0xf8, /* UART Component Version */
    DW_CTR = 0xfc, /* Component Type Register */
} DwSerialReg;

static uint64_t dw_serial_io_read(void *opaque, hwaddr addr,
	                          unsigned size)
{
    DwSerialState *dw = opaque;

    if (unlikely(addr & 0x3) || unlikely(size != 4)) {
        /* unaligned or not 32b */
        DBGLOG("invalid dw_serial read 0x%08" HWADDR_PRIx "\n", addr);
        return 0;
    }

    switch (addr + UART_8250_SZ) {
    case DW_UCV:
        return (('0' + ((dw->version >> 8) & 0xf)) << 24) |
               (('0' + ((dw->version >> 4) & 0xf)) << 16) |
               (('0' + ((dw->version >> 0) & 0xf)) << 8) |
               '*';

    case DW_CTR:
        return 0x44570110;

    default:
        DBGLOG("unhandled dw_serial read 0x%08" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void dw_serial_io_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    if (unlikely(addr & 0x3) || unlikely(size != 4)) {
        /* unaligned or not 32b */
        DBGLOG("invalid dw_serial write 0x%08" HWADDR_PRIx
               " = 0x%08" PRIx64 "\n", addr, val);
        return;
    }

    switch (addr + UART_8250_SZ) {
    default:
        DBGLOG("unhandled dw_serial write 0x%08" HWADDR_PRIx
               " = 0x%08" PRIx64 "\n", addr, val);
    }
}

static const MemoryRegionOps dw_serial_io_ops = {
    .read = dw_serial_io_read,
    .write = dw_serial_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

DwSerialState *dw_serial_mm_init(MemoryRegion *address_space,
                                 hwaddr base, qemu_irq irq, int baudbase,
                                 CharDriverState *chr, enum device_endian end)
{
    DwSerialState *dw;

    dw = g_malloc0(sizeof(DwSerialState));
    dw->version = 0x201;
    dw->uart = serial_mm_init(address_space, base, 2, irq, baudbase, chr, end);

    memory_region_init_io(&dw->dw_iomem, &dw_serial_io_ops, dw,
                          "dw_serial", 0x100 - UART_8250_SZ);
    memory_region_add_subregion(address_space, base + UART_8250_SZ,
                                &dw->dw_iomem);

    return dw;
}
