/*
 * Allwinner A10 Clock Control Module
 *
 * Copyright (C) 2016 Guenter Roeck
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/misc/allwinner-a10-ccm.h"

static void update_clocks(AwA10CCMState *s)
{
}

static int aw_a10_ccm_post_load(void *opaque, int version_id)
{
    AwA10CCMState *s = (AwA10CCMState *)opaque;

    update_clocks(s);
    return 0;
}

static const VMStateDescription vmstate_aw_a10_ccm = {
    .name = TYPE_AW_A10_CCM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(pll1_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll1_tun, AwA10CCMState),
        VMSTATE_UINT32(pll2_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll2_tun, AwA10CCMState),
        VMSTATE_UINT32(pll3_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll4_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll5_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll5_tun, AwA10CCMState),
        VMSTATE_UINT32(pll6_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll6_tun, AwA10CCMState),
        VMSTATE_UINT32(pll7_cfg, AwA10CCMState),
        VMSTATE_UINT32(pll1_tun2, AwA10CCMState),
        VMSTATE_UINT32(pll5_tun2, AwA10CCMState),
        VMSTATE_UINT32(osc24m_cfg, AwA10CCMState),
        VMSTATE_UINT32(cpu_ahb_apb0_cfg, AwA10CCMState),
        VMSTATE_UINT32(apb1_clk_div, AwA10CCMState),
        VMSTATE_UINT32(ahb_gating0, AwA10CCMState),
        VMSTATE_UINT32(ahb_gating1, AwA10CCMState),
        VMSTATE_UINT32(apb0_gating, AwA10CCMState),
        VMSTATE_UINT32(apb1_gating, AwA10CCMState),
        VMSTATE_UINT32(nand_sclk_cfg, AwA10CCMState),
        VMSTATE_UINT32(sd0_clk, AwA10CCMState),
        VMSTATE_UINT32(sd1_clk, AwA10CCMState),
        VMSTATE_UINT32(sd2_clk, AwA10CCMState),
        VMSTATE_UINT32(sd3_clk, AwA10CCMState),
        VMSTATE_UINT32(ts_clk, AwA10CCMState),
        VMSTATE_UINT32(ss_clk, AwA10CCMState),
        VMSTATE_UINT32(spi0_clk, AwA10CCMState),
        VMSTATE_UINT32(spi1_clk, AwA10CCMState),
        VMSTATE_UINT32(spi2_clk, AwA10CCMState),
        VMSTATE_UINT32(ir0_clk, AwA10CCMState),
        VMSTATE_UINT32(ir1_clk, AwA10CCMState),
        VMSTATE_UINT32(iis_clk, AwA10CCMState),
        VMSTATE_UINT32(ac97_clk, AwA10CCMState),
        VMSTATE_UINT32(keypad_clk, AwA10CCMState),
        VMSTATE_UINT32(spi3_clk, AwA10CCMState),
        VMSTATE_UINT32(dram_clk, AwA10CCMState),
        VMSTATE_END_OF_LIST()
    },
    .post_load = aw_a10_ccm_post_load,
};

static void aw_a10_ccm_reset(DeviceState *dev)
{
    AwA10CCMState *s = AW_A10_CCM(dev);

    s->pll1_cfg = 0x21005000;
    s->pll1_tun = 0;
    s->pll2_cfg = 0x008100010;
    s->pll2_tun = 0;
    s->pll3_cfg = 0x0010d063;
    s->pll4_cfg = 0x21081000;
    s->pll5_cfg = 0x11049280;
    s->pll6_cfg = 0x21009911;
    s->pll6_tun = 0;
    s->pll7_cfg = 0x0010d063;
    s->osc24m_cfg = 0x00138013;
    s->cpu_ahb_apb0_cfg = 0x00010010;
    s->ac97_clk = 0x00030000;
    s->keypad_clk = 0x0000001f;

    update_clocks(s);
}

static uint64_t aw_a10_ccm_read(void *opaque, hwaddr offset,
                                unsigned size)
{
    AwA10CCMState *s = (AwA10CCMState *)opaque;

    switch (offset & ~3) {
    case PLL1_CFG_REG:
        return s->pll1_cfg;
    case PLL1_TUN_REG:
        return s->pll1_tun;
    case PLL2_CFG_REG:
        return s->pll2_cfg;
    case PLL2_TUN_REG:
        return s->pll2_tun;
    case PLL3_CFG_REG:
	return s->pll3_cfg;
    case PLL4_CFG_REG:
	return s->pll4_cfg;
    case PLL5_CFG_REG:
	return s->pll5_cfg;
    case PLL6_CFG_REG:
	return s->pll6_cfg;
    case PLL6_TUN_REG:
	return s->pll6_tun;
    case PLL7_CFG_REG:
	return s->pll7_cfg;
    case PLL1_TUN2_REG:
	return s->pll1_tun2;
    case PLL5_TUN2_REG:
	return s->pll5_tun2;
    case OSC24M_CFG_REG:
	return s->osc24m_cfg;
    case CPU_AHB_APB0_CFG_REG:
	return s->cpu_ahb_apb0_cfg;
    case APB1_CLK_DIV_REG:
	return s->apb1_clk_div;
    case AHB_GATING_REG0:
	return s->ahb_gating0;
    case AHB_GATING_REG1:
	return s->ahb_gating1;
    case APB0_GATING_REG:
	return s->apb0_gating;
    case APB1_GATING_REG:
	return s->apb1_gating;
    case NAND_SCLK_CFG_REG:
	return s->nand_sclk_cfg;
    case SD0_CLK_REG:
	return s->sd0_clk;
    case SD1_CLK_REG:
	return s->sd1_clk;
    case SD2_CLK_REG:
	return s->sd2_clk;
    case SD3_CLK_REG:
	return s->sd3_clk;
    case TS_CLK_REG:
	return s->ts_clk;
    case SS_CLK_REG:
	return s->ss_clk;
    case SPI0_CLK_REG:
	return s->spi0_clk;
    case SPI1_CLK_REG:
	return s->spi1_clk;
    case SPI2_CLK_REG:
	return s->spi2_clk;
    case IR0_CLK_REG:
	return s->ir0_clk;
    case IR1_CLK_REG:
	return s->ir1_clk;
    case AC97_CLK_REG:
	return s->ac97_clk;
    case KEYPAD_CLK_REG:
	return s->keypad_clk;
    case SPI3_CLK_REG:
	return s->spi3_clk;
    case DRAM_CLK_REG:
	return s->dram_clk;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad register at offset 0x%"
                      HWADDR_PRIx "\n", TYPE_AW_A10_CCM, __func__, offset);
        return 0;
    }
}

static void aw_a10_ccm_write(void *opaque, hwaddr offset,
			     uint64_t value, unsigned size)
{
    AwA10CCMState *s = (AwA10CCMState *)opaque;

    switch (offset & ~3) {
    case PLL1_CFG_REG:
        s->pll1_cfg = value;
	break;
    case PLL1_TUN_REG:
        s->pll1_tun = value;
	break;
    case PLL2_CFG_REG:
        s->pll2_cfg = value;
	break;
    case PLL2_TUN_REG:
        s->pll2_tun = value;
	break;
    case PLL3_CFG_REG:
	s->pll3_cfg = value;
	break;
    case PLL4_CFG_REG:
	s->pll4_cfg = value;
	break;
    case PLL5_CFG_REG:
	s->pll5_cfg = value;
	break;
    case PLL6_CFG_REG:
	s->pll6_cfg = value;
	break;
    case PLL6_TUN_REG:
	s->pll6_tun = value;
	break;
    case PLL7_CFG_REG:
	s->pll7_cfg = value;
	break;
    case PLL1_TUN2_REG:
	s->pll1_tun2 = value;
	break;
    case PLL5_TUN2_REG:
	s->pll5_tun2 = value;
	break;
    case OSC24M_CFG_REG:
	s->osc24m_cfg = value;
	break;
    case CPU_AHB_APB0_CFG_REG:
	s->cpu_ahb_apb0_cfg = value;
	break;
    case APB1_CLK_DIV_REG:
	s->apb1_clk_div = value;
	break;
    case AHB_GATING_REG0:
	s->ahb_gating0 = value;
	break;
    case AHB_GATING_REG1:
	s->ahb_gating1 = value;
	break;
    case APB0_GATING_REG:
	s->apb0_gating = value;
	break;
    case APB1_GATING_REG:
	s->apb1_gating = value;
	break;
    case NAND_SCLK_CFG_REG:
	s->nand_sclk_cfg = value;
	break;
    case SD0_CLK_REG:
	s->sd0_clk = value;
	break;
    case SD1_CLK_REG:
	s->sd1_clk = value;
	break;
    case SD2_CLK_REG:
	s->sd2_clk = value;
	break;
    case SD3_CLK_REG:
	s->sd3_clk = value;
	break;
    case TS_CLK_REG:
	s->ts_clk = value;
	break;
    case SS_CLK_REG:
	s->ss_clk = value;
	break;
    case SPI0_CLK_REG:
	s->spi0_clk = value;
	break;
    case SPI1_CLK_REG:
	s->spi1_clk = value;
	break;
    case SPI2_CLK_REG:
	s->spi2_clk = value;
	break;
    case IR0_CLK_REG:
	s->ir0_clk = value;
	break;
    case IR1_CLK_REG:
	s->ir1_clk = value;
	break;
    case AC97_CLK_REG:
	s->ac97_clk = value;
	break;
    case KEYPAD_CLK_REG:
	s->keypad_clk = value;
	break;
    case SPI3_CLK_REG:
	s->spi3_clk = value;
	break;
    case DRAM_CLK_REG:
	s->dram_clk = value;
	break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad register at offset 0x%"
                      HWADDR_PRIx "\n", TYPE_AW_A10_CCM, __func__, offset);
        return;
    }
    update_clocks(s);
}

static const struct MemoryRegionOps aw_a10_ccm_ops = {
    .read = aw_a10_ccm_read,
    .write = aw_a10_ccm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void aw_a10_ccm_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    AwA10CCMState *s = AW_A10_CCM(dev);

    memory_region_init_io(&s->iomem, OBJECT(sbd), &aw_a10_ccm_ops, s,
                          TYPE_AW_A10_CCM, 0x400);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void aw_a10_ccm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = aw_a10_ccm_realize;
    dc->reset = aw_a10_ccm_reset;
    dc->vmsd = &vmstate_aw_a10_ccm;
    dc->desc = "Allwinner A10 Clock Control Module";
}

static const TypeInfo aw_a10_ccm_info = {
    .name = TYPE_AW_A10_CCM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AwA10CCMState),
    .class_init = aw_a10_ccm_class_init,
};

static void aw_a10_ccm_register_types(void)
{
    type_register_static(&aw_a10_ccm_info);
}

type_init(aw_a10_ccm_register_types)
