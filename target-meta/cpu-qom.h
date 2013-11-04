/*
 * QEMU META CPU
 *
 * Copyright (c) 2012 SUSE LINUX Products GmbH
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */
#ifndef QEMU_META_CPU_QOM_H
#define QEMU_META_CPU_QOM_H

#include "qemu/cpu.h"

#ifdef TARGET_META64
#define TYPE_META_CPU "meta64-cpu"
#else
#define TYPE_META_CPU "meta-cpu"
#endif

#define META_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(METACPUClass, (klass), TYPE_META_CPU)
#define META_CPU(obj) \
    OBJECT_CHECK(METACPU, (obj), TYPE_META_CPU)
#define META_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(METACPUClass, (obj), TYPE_META_CPU)

/**
 * METACPUClass:
 * @parent_reset: The parent class' reset handler.
 *
 * A META CPU model.
 */
typedef struct METACPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    void (*parent_reset)(CPUState *cpu);
} METACPUClass;

/**
 * METACPU:
 * @env: #CPUMETAState
 *
 * A META CPU.
 */
typedef struct METACPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUMETAState env;
} METACPU;

static inline METACPU *meta_env_get_cpu(CPUMETAState *env)
{
    return META_CPU(container_of(env, METACPU, env));
}

#define ENV_GET_CPU(e) CPU(meta_env_get_cpu(e))


#endif
