/*
 * Microchip PolarFire SoC SYSREG module emulation
 *
 * Copyright (c) 2020 Wind River Systems, Inc.
 *
 * Author:
 *   Bin Meng <bin.meng@windriver.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/mchp_pfsoc_sysreg.h"

#define CLOCK_CONFIG_CR 0x08
#define RTC_CLOCK_CR    0x0c
#define SUBBLK_CLOCK_CR 0x84
#define ENVM_CR         0xb8
#define MESSAGE_INT     0x118c

static uint64_t mchp_pfsoc_sysreg_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    MchpPfSoCSysregState *s = opaque;
    uint32_t val;

    switch (offset) {
    case CLOCK_CONFIG_CR:
    case RTC_CLOCK_CR:
    case SUBBLK_CLOCK_CR:
    case ENVM_CR:
        val = s->reg[offset];
        break;
    default:
        val = s->reg[offset];
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented device read "
                      "(size %d, offset 0x%" HWADDR_PRIx ")\n",
                      __func__, size, offset);
        break;
    }
    return val;
}

static void mchp_pfsoc_sysreg_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    MchpPfSoCSysregState *s = opaque;

    switch (offset) {
    case MESSAGE_INT:
        qemu_irq_lower(s->irq);
        break;
    case CLOCK_CONFIG_CR:
    case RTC_CLOCK_CR:
    case SUBBLK_CLOCK_CR:
    case ENVM_CR:
        s->reg[offset] = value;
        break;
    default:
        s->reg[offset] = value;
        qemu_log_mask(LOG_UNIMP, "%s: unimplemented device write "
                      "(size %d, value 0x%" PRIx64
                      ", offset 0x%" HWADDR_PRIx ")\n",
                      __func__, size, value, offset);
        break;
    }
}

static const MemoryRegionOps mchp_pfsoc_sysreg_ops = {
    .read = mchp_pfsoc_sysreg_read,
    .write = mchp_pfsoc_sysreg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void mchp_pfsoc_sysreg_realize(DeviceState *dev, Error **errp)
{
    MchpPfSoCSysregState *s = MCHP_PFSOC_SYSREG(dev);

    memory_region_init_io(&s->sysreg, OBJECT(dev),
                          &mchp_pfsoc_sysreg_ops, s,
                          "mchp.pfsoc.sysreg",
                          MCHP_PFSOC_SYSREG_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->sysreg);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static void mchp_pfsoc_sysreg_reset(DeviceState *dev)
{
    MchpPfSoCSysregState *s = MCHP_PFSOC_SYSREG(dev);

    memset(s->reg, 0, sizeof(uint32_t) * MCHP_PFSOC_SYSREG_REG_SIZE);

    s->reg[CLOCK_CONFIG_CR] = BIT(4);
    s->reg[RTC_CLOCK_CR] = 0x00010064;
    s->reg[SUBBLK_CLOCK_CR] = 0x00000001; /* ENVM enabled */
    s->reg[ENVM_CR] = 0x40000000 | BIT(6) | 0x0f;
}

static void mchp_pfsoc_sysreg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Microchip PolarFire SoC SYSREG module";
    dc->realize = mchp_pfsoc_sysreg_realize;
    dc->reset = mchp_pfsoc_sysreg_reset;
}

static const TypeInfo mchp_pfsoc_sysreg_info = {
    .name          = TYPE_MCHP_PFSOC_SYSREG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MchpPfSoCSysregState),
    .class_init    = mchp_pfsoc_sysreg_class_init,
};

static void mchp_pfsoc_sysreg_register_types(void)
{
    type_register_static(&mchp_pfsoc_sysreg_info);
}

type_init(mchp_pfsoc_sysreg_register_types)
