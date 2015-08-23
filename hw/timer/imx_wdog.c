/*
 * i.MX watchdog
 *
 * Copyright (C) 2015 Guenter Roeck
 *
 * This code is licensed under GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qemu/event_notifier.h"
#include "qemu/main-loop.h"
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/timer/imx_wdog.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/runstate.h"

#define DEBUG_WDOG 0

#if DEBUG_WDOG

#  define DPRINTF(fmt, args...) \
    do { fprintf(stderr, "%s: " fmt , __func__, ##args); } while (0)
#else
#  define DPRINTF(fmt, args...) do {} while (0)
#endif

static void imx_wdog_expired(void *opaque)
{
    IMXWDOGState *s = IMX_WDOG(opaque);

    s->wrsr |= IMX2_WDT_WRSR_TOUT;

    /* Reset system if watchdog is enabled and if WRE is not set */
    if ((s->wcr & IMX2_WDT_WCR_WDE) && !(s->wcr & IMX2_WDT_WCR_WRE)) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void imx_wdog_reset(DeviceState *dev)
{
    IMXWDOGState *s = IMX_WDOG(dev);

    s->wcr = 0;
    s->wsr = 0;
    s->wrsr &= ~IMX2_WDT_WRSR_TOUT;
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
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);

        ptimer_transaction_begin(s->timer);
        ptimer_stop(s->timer);
        if ((s->wcr & IMX2_WDT_WCR_WDE) && (s->wcr & IMX2_WDT_WCR_WT)) {
            ptimer_set_count(s->timer, (s->wcr & IMX2_WDT_WCR_WT) >> 8);
            ptimer_run(s->timer, 1);
        }
        ptimer_transaction_commit(s->timer);
        break;
    case IMX2_WDT_WSR:
        if (s->wsr == IMX2_WDT_SEQ1 && value == IMX2_WDT_SEQ2) {
            ptimer_transaction_begin(s->timer);
            ptimer_set_count(s->timer, (s->wcr & IMX2_WDT_WCR_WT) >> 8);
            ptimer_transaction_commit(s->timer);
        }
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

    s->timer = ptimer_init(imx_wdog_expired, s, PTIMER_POLICY_DEFAULT);
    ptimer_transaction_begin(s->timer);
    ptimer_set_freq(s->timer, 2);
    ptimer_set_limit(s->timer, 0xff, 1);
    ptimer_transaction_commit(s->timer);
}

static void imx_wdog_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

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
