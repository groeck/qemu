#include "dw_otg.h"

#include "qemu-log.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

struct dw_otg_state {
    MemoryRegion iomem;
};

static uint64_t dw_otg_io_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    /*struct dw_otg_state *d = opaque;*/

    DBGLOG("dw_otg unhandled read(0x%x)\n", addr);
    return 0;
}

static void dw_otg_io_write(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    /*struct dw_otg_state *d = opaque;*/

    DBGLOG("dw_otg unhandled write(0x%x,0x%x)\n", addr, val);
}

static const MemoryRegionOps dw_otg_io_ops = {
    .read = dw_otg_io_read,
    .write = dw_otg_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

struct dw_otg_state *dw_otg_init(hwaddr base)
{
    struct dw_otg_state *d;

    d = g_malloc0(sizeof(struct dw_otg_state));

    memory_region_init_io(&d->iomem, &dw_otg_io_ops, d,
                          "dw-otg", DW_OTG_SIZE - 1);

    return d;
}

MemoryRegion *dw_otg_iomem(struct dw_otg_state *d)
{
    return &d->iomem;
}

