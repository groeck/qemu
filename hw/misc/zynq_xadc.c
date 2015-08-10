/*
 * ADC registers for Xilinx Zynq Platform
 *
 * Copyright (c) 2015 Guenter Roeck
 * Based on hw/misc/zynq_slcr.c, written by Michal Simek
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/hw.h"
#include "hw/misc/zynq_xadc.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"

enum {
    CFG                = 0x000 / 4,
    INTSTS,
    INTMSK,
    STATUS,
    CFIFO,
    DFIFO,
    CTL,
};

#define XADC_ZYNQ_CFG_ENABLE            BIT(31)
#define XADC_ZYNQ_CFG_CFIFOTH_RD(x)     (((x) >> 20) & 0x0f)
#define XADC_ZYNQ_CFG_DFIFOTH_RD(x)     (((x) >> 16) & 0x0f)
#define XADC_ZYNQ_CFG_WEDGE             BIT(13)
#define XADC_ZYNQ_CFG_REDGE             BIT(12)
#define XADC_ZYNQ_CFG_TCKRATE_DIV2      (0x0 << 8)
#define XADC_ZYNQ_CFG_TCKRATE_DIV4      (0x1 << 8)
#define XADC_ZYNQ_CFG_TCKRATE_DIV8      (0x2 << 8)
#define XADC_ZYNQ_CFG_TCKRATE_DIV16     (0x3 << 8)
#define XADC_ZYNQ_CFG_IGAP_MASK         0x1f
#define XADC_ZYNQ_CFG_IGAP(x)           ((x) & XADC_ZYNQ_CFG_IGAP_MASK)

#define XADC_ZYNQ_INT_CFIFO_LTH         BIT(9)
#define XADC_ZYNQ_INT_DFIFO_GTH         BIT(8)
#define XADC_ZYNQ_INT_ALARM_MASK        0xff
#define XADC_ZYNQ_INT_ALARM_OFFSET      0

#define XADC_ZYNQ_STATUS_DFIFO_LVL(x)   (((x) & 0x0f) << 12)
#define XADC_ZYNQ_STATUS_CFIFOF         BIT(11)
#define XADC_ZYNQ_STATUS_CFIFOE         BIT(10)
#define XADC_ZYNQ_STATUS_DFIFOF         BIT(9)
#define XADC_ZYNQ_STATUS_DFIFOE         BIT(8)
#define XADC_ZYNQ_STATUS_OT             BIT(7)
#define XADC_ZYNQ_STATUS_ALM(x)         BIT(x)

#define XADC_ZYNQ_CTL_RESET             BIT(4)

#define XADC_ZYNQ_CMD_NOP               0x00
#define XADC_ZYNQ_CMD_READ              0x01
#define XADC_ZYNQ_CMD_WRITE             0x02

static void zynq_xadc_reset(DeviceState *d)
{
    ZynqXADCState *s = ZYNQ_XADC(d);
    int i;

    s->regs[CFG] = XADC_ZYNQ_CFG_IGAP(0x14) | XADC_ZYNQ_CFG_TCKRATE_DIV4 |
        XADC_ZYNQ_CFG_REDGE;
    s->regs[INTSTS] = XADC_ZYNQ_INT_CFIFO_LTH;
    s->regs[INTMSK] = 0xffffffff;
    s->regs[STATUS] = 0;
    s->regs[CFIFO] = 0;
    s->regs[DFIFO] = 0;
    s->regs[CTL] = XADC_ZYNQ_CTL_RESET;

    for (i = 0; i < ARRAY_SIZE(s->xadc_regs); i++) {
        s->xadc_regs[i] = 0;
    }
    for (i = 0; i < ARRAY_SIZE(s->xadc_dfifo); i++) {
        s->xadc_dfifo[i] = 0;
    }
    s->xadc_dfifo_entries = 0;
}

static bool zynq_xadc_check_offset(hwaddr offset, bool rnw)
{
    switch (offset) {
    case CFG:
    case INTMSK:
    case INTSTS:
    case CTL:
        return true;
    case DFIFO:
    case STATUS:
        return rnw;     /* read only */
    case CFIFO:
        return !rnw;    /* write only */
    default:
        return false;
    }
}

static void zynq_xadc_update_ints(ZynqXADCState *s)
{
    qemu_set_irq(s->qemu_irq, !!(s->regs[INTSTS] & ~s->regs[INTMSK]));
}

static uint64_t zynq_xadc_read(void *opaque, hwaddr offset, unsigned size)
{
    ZynqXADCState *s = opaque;
    int reg = offset / 4;
    uint64_t rv = 0;
    int i;

    if (!zynq_xadc_check_offset(reg, true)) {
        qemu_log_mask(LOG_GUEST_ERROR, "zynq_xadc: Invalid read access to "
                      " addr %" HWADDR_PRIx "\n", offset);
    }

    switch (reg) {
    case CFG:
    case INTMSK:
    case INTSTS:
    case CTL:
        rv = s->regs[reg];
        break;
    case STATUS:
        rv = XADC_ZYNQ_STATUS_CFIFOE;
        rv |= XADC_ZYNQ_STATUS_DFIFO_LVL(s->xadc_dfifo_entries);
        if (!s->xadc_dfifo_entries) {
            rv |= XADC_ZYNQ_STATUS_DFIFOE;
        } else if (s->xadc_dfifo_entries >= ARRAY_SIZE(s->xadc_dfifo)) {
            rv |= XADC_ZYNQ_STATUS_DFIFOF;
        }
        break;
    case DFIFO:
        rv = s->xadc_dfifo[0];
        if (s->xadc_dfifo_entries > 0) {
            s->xadc_dfifo_entries--;
        }
        for (i = 0; i < s->xadc_dfifo_entries; i++) {
            s->xadc_dfifo[i] = s->xadc_dfifo[i + 1];
        }
        s->xadc_dfifo[s->xadc_dfifo_entries] = 0;
        zynq_xadc_update_ints(s);
        break;
    }
    return rv;
}

static void xadc_add_dfifo(ZynqXADCState *s, uint16_t regval)
{
    if (s->xadc_dfifo_entries < ARRAY_SIZE(s->xadc_dfifo)) {
            s->xadc_dfifo[s->xadc_dfifo_entries++] = s->xadc_read_reg_previous;
    }
    s->xadc_read_reg_previous = regval;
    if (s->xadc_dfifo_entries > XADC_ZYNQ_CFG_DFIFOTH_RD(s->regs[CFG])) {
        s->regs[INTSTS] |= XADC_ZYNQ_INT_DFIFO_GTH;
    }
    zynq_xadc_update_ints(s);
}

static void zynq_xadc_write(void *opaque, hwaddr offset, uint64_t val,
                            unsigned size)
{
    ZynqXADCState *s = (ZynqXADCState *)opaque;
    int reg = offset / 4;
    int xadc_reg;
    int xadc_cmd;
    int xadc_data;

    if (!zynq_xadc_check_offset(reg, false)) {
        qemu_log_mask(LOG_GUEST_ERROR, "zynq_xadc: Invalid write access to "
                      "addr %" HWADDR_PRIx "\n", offset);
        return;
    }

    switch (reg) {
    case CFG:
        s->regs[CFG] = val;
        break;
    case INTSTS:
        s->regs[INTSTS] &= ~val;
        zynq_xadc_update_ints(s);
        break;
    case INTMSK:
        s->regs[INTMSK] = val & 0x003ff;
        zynq_xadc_update_ints(s);
        break;
    case CFIFO:
        xadc_cmd = extract32(val, 26, 4);
        xadc_reg = extract32(val, 16, 7);
        xadc_data = extract32(val, 0, 16);

        switch (xadc_cmd) {
        case XADC_ZYNQ_CMD_READ:
            xadc_add_dfifo(s, s->xadc_regs[xadc_reg]);
            break;
        case XADC_ZYNQ_CMD_WRITE:
            s->xadc_regs[xadc_reg] = xadc_data;
            break;
        case XADC_ZYNQ_CMD_NOP:
            xadc_add_dfifo(s, 0);
            break;
        }
        s->regs[INTSTS] |= XADC_ZYNQ_INT_CFIFO_LTH;
        zynq_xadc_update_ints(s);
        break;
    case CTL:
        if (val & XADC_ZYNQ_CTL_RESET) {
            zynq_xadc_reset(DEVICE(s));
        }
        s->regs[CTL] = val & 0x00fffeff;
        break;
    }
}

static const MemoryRegionOps xadc_ops = {
    .read = zynq_xadc_read,
    .write = zynq_xadc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void zynq_xadc_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    ZynqXADCState *s = ZYNQ_XADC(obj);

    memory_region_init_io(&s->iomem, obj, &xadc_ops, s, "xadc",
                          ZYNQ_XADC_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->qemu_irq);
}

static const VMStateDescription vmstate_zynq_xadc = {
    .name = "zynq_xadc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, ZynqXADCState, ZYNQ_XADC_NUM_IO_REGS),
        VMSTATE_UINT16_ARRAY(xadc_regs, ZynqXADCState, ZYNQ_XADC_NUM_ADC_REGS),
        VMSTATE_UINT16_ARRAY(xadc_dfifo, ZynqXADCState, ZYNQ_XADC_FIFO_DEPTH),
        VMSTATE_UINT16(xadc_read_reg_previous, ZynqXADCState),
        VMSTATE_UINT16(xadc_dfifo_entries, ZynqXADCState),
        VMSTATE_END_OF_LIST()
    }
};

static void zynq_xadc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_zynq_xadc;
    dc->reset = zynq_xadc_reset;
}

static const TypeInfo zynq_xadc_info = {
    .class_init = zynq_xadc_class_init,
    .name  = TYPE_ZYNQ_XADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(ZynqXADCState),
    .instance_init = zynq_xadc_init,
};

static void zynq_xadc_register_types(void)
{
    type_register_static(&zynq_xadc_info);
}

type_init(zynq_xadc_register_types)
