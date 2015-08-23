/*
 * IMX watchdog
 *
 * Copyright (C) 2015 Guenter Roeck
 *
 * This code is licensed under GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 */

#include "hw/hw.h"
#include "qemu/bitops.h"
#include "hw/sysbus.h"
#include "hw/arm/imx.h"
#include "qemu/main-loop.h"
#include "sysemu/sysemu.h"

#define TYPE_IMX_WDOG "imx.wdog"

#define DEBUG_WDOG 0

#if DEBUG_WDOG

#  define DPRINTF(fmt, args...) \
    do { fprintf(stderr, "%s: " fmt , __func__, ##args); } while (0)
#else
#  define DPRINTF(fmt, args...) do {} while (0)
#endif

#define IMX_WDOG(obj) \
        OBJECT_CHECK(IMXWDOGState, (obj), TYPE_IMX_WDOG)

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    DeviceState *ccm;

    uint32_t wcr;
    uint32_t wsr;
    uint32_t wrsr;
    uint32_t wmcr;
    uint32_t cnt;
} IMXWDOGState;

#define IMX2_WDT_WCR            0x00            /* Control Register */
#define IMX2_WDT_WCR_WT         (0xFF << 8)     /* -> Watchdog Timeout Field */
#define IMX2_WDT_WCR_WRE        (1 << 3)        /* -> WDOG Reset Enable */
#define IMX2_WDT_WCR_WDE        (1 << 2)        /* -> Watchdog Enable */
#define IMX2_WDT_WCR_WDZST      (1 << 0)        /* -> Watchdog timer Suspend */

#define IMX2_WDT_WSR            0x02            /* Service Register */
#define IMX2_WDT_SEQ1           0x5555          /* -> service sequence 1 */
#define IMX2_WDT_SEQ2           0xAAAA          /* -> service sequence 2 */

#define IMX2_WDT_WRSR           0x04            /* Reset Status Register */
#define IMX2_WDT_WRSR_TOUT      (1 << 1)        /* -> Reset due to Timeout */

#define IMX2_WDT_WMCR           0x08            /* Misc Register */

#define IMX2_WDT_MAX_TIME       128
#define IMX2_WDT_DEFAULT_TIME   60              /* in seconds */

#define WDOG_SEC_TO_COUNT(s)    (((s) * 2 - 1) << 8)

static void imx_wdog_reset(DeviceState *dev)
{
    IMXWDOGState *s = IMX_WDOG(dev);

    s->wcr = 0;
    s->wsr = 0;
    s->wrsr = 0;
    s->wmcr = 0;
}

static uint64_t imx_wdog_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXWDOGState *s = IMX_WDOG(opaque);

    DPRINTF("read(offset=%lx)", offset);
    switch (offset) {
    case IMX2_WDT_WCR:
        DPRINTF(" wcr = 0x%x\n", s->wcr);
        return s->wcr;
    case IMX2_WDT_WSR:
        DPRINTF(" wsr = 0x%x\n", s->wsr);
        return s->wsr;
    case IMX2_WDT_WRSR:
        DPRINTF(" wrsr = 0x%x\n", s->wrsr);
        return s->wrsr;
    case IMX2_WDT_WMCR:
        DPRINTF(" wmcr = 0x%x\n", s->wmcr);
        return s->wmcr;
    }
    DPRINTF(" return 0\n");
    return 0;
}

static void imx_wdog_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    IMXWDOGState *s = IMX_WDOG(opaque);

    DPRINTF("write(offset=%lx, value = %x)\n", offset, (unsigned int)value);
    switch (offset) {
    case IMX2_WDT_WCR:
        s->wcr = value & (IMX2_WDT_WCR_WT | IMX2_WDT_WCR_WRE | IMX2_WDT_WCR_WDE
                          | IMX2_WDT_WCR_WDZST);
        if ((s->wcr & IMX2_WDT_WCR_WT) == 0 && (s->wcr & IMX2_WDT_WCR_WDE) &&
            !(s->wcr & IMX2_WDT_WCR_WRE))
                qemu_system_reset_request();

        break;
    case IMX2_WDT_WSR:
        s->wsr = value;
        break;
    case IMX2_WDT_WRSR:
        break;
    case IMX2_WDT_WMCR:
        s->wmcr = value;
        break;
    default:
        break;
    }
}

void imx_timerw_create(const hwaddr addr, DeviceState *ccm)
{
    IMXWDOGState *s;
    DeviceState *dev;

    dev = sysbus_create_simple(TYPE_IMX_WDOG, addr, NULL);
    s = IMX_WDOG(dev);
    s->ccm = ccm;
}

static const MemoryRegionOps imx_wdog_ops = {
  .read = imx_wdog_read,
  .write = imx_wdog_write,
  .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_imx_timer_wdog = {
    .name = "imx.wdog",

    .fields = (VMStateField[]) {
        VMSTATE_UINT32(wcr, IMXWDOGState),
        VMSTATE_UINT32(wsr, IMXWDOGState),
        VMSTATE_UINT32(wrsr, IMXWDOGState),
        VMSTATE_UINT32(wmcr, IMXWDOGState),
        VMSTATE_END_OF_LIST()
    }
};

static void imx_wdog_realize(DeviceState *dev, Error **errp)
{
    IMXWDOGState *s = IMX_WDOG(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &imx_wdog_ops, s, TYPE_IMX_WDOG,
                          0x00001000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void imx_wdog_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc  = DEVICE_CLASS(klass);

    dc->realize = imx_wdog_realize;
    dc->reset = imx_wdog_reset;
    dc->vmsd = &vmstate_imx_timer_wdog;
    dc->desc = "i.MX watchdog timer";
}

static const TypeInfo imx_wdog_info = {
    .name = TYPE_IMX_WDOG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXWDOGState),
    .class_init = imx_wdog_class_init,
};

static void imx_wdog_register_types(void)
{
    type_register_static(&imx_wdog_info);
}

type_init(imx_wdog_register_types)
