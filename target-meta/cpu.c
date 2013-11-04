/*
 *  META virtual CPU
 *
 *  Copyright (c) 2011 Imagination Technologies
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "cpu.h"
#include "qemu-common.h"

const char *meta_freason_names[4] = {
    [META_FREASON_NOERROR] = "no error",
    [META_FREASON_GENERAL] = "general violation",
    [META_FREASON_PAGE]    = "page fault",
    [META_FREASON_PROTECT] = "protection violation",
};

const char *meta_hreason_names[4] = {
    [META_HREASON_SWITCH]  = "SWITCH",
    [META_HREASON_UNKNOWN] = "unknown instruction",
    [META_HREASON_PRIV]    = "privilege violation",
    [META_HREASON_FAULT]   = "memory fault",
};

/* CPUClass::reset() */
static void meta_cpu_reset(CPUState *s)
{
    METACPU *cpu = META_CPU(s);
    METACPUClass *mcc = META_CPU_GET_CLASS(cpu);
    CPUMETAState *env = &cpu->env;

    mcc->parent_reset(s);

    cpu_state_reset(env);
}

static void meta_cpu_initfn(Object *obj)
{
    METACPU *cpu = META_CPU(obj);
    CPUMETAState *env = &cpu->env;

    cpu_exec_init(env);
}

static void meta_cpu_class_init(ObjectClass *c, void *data)
{
    METACPUClass *mcc = META_CPU_CLASS(c);
    CPUClass *cc = CPU_CLASS(c);

    mcc->parent_reset = cc->reset;
    cc->reset = meta_cpu_reset;
}

static const TypeInfo meta_cpu_type_info = {
    .name = TYPE_META_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(METACPU),
    .instance_init = meta_cpu_initfn,
    .abstract = false,
    .class_size = sizeof(METACPUClass),
    .class_init = meta_cpu_class_init,
};

static void meta_cpu_register_types(void)
{
    type_register_static(&meta_cpu_type_info);
}

type_init(meta_cpu_register_types)
