/*
 * Copyright (c) 2015 Guenter Roeck
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "cpu-qom.h"
#include "exec/helper-proto.h"
#include "kvm-consts.h"
#include "sysemu/sysemu.h"
#include "internals.h"

void arm_handle_smc_call(ARMCPU *cpu)
{
    // CPUState *cs = CPU(cpu);
    CPUARMState *env = &cpu->env;
    uint32_t cmd;
    uint32_t val;

    cmd = env->regs[12];
    val = env->regs[0];

    fprintf(stderr, "SMC cmd 0x%x val 0x%x\n", cmd, val);

    if (cmd == 0x102) {
	// write val into L2X0_CTRL
    }
}
