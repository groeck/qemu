/*
 * Texas Instruments TMP108 Temperature Sensor I2C messages
 *
 * Browse the data sheet:
 *
 *    http://www.ti.com/lit/ds/symlink/tmp108.pdf
 *
 * Copyright (C) 2016 Guenter Roeck <linux@roeck-us.net>
 *
 * Derived from tmp105_regs.h.
 *
 * Copyright (C) 2012 Alex Horn <alex.horn@cs.ox.ac.uk>
 * Copyright (C) 2008-2012 Andrzej Zaborowski <balrogg@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */

#ifndef TMP108_REGS_H
#define TMP108_REGS_H

/**
 * TMP108Reg:
 * @TMP108_REG_TEMPERATURE: Temperature register
 * @TMP108_REG_CONFIG: Configuration register
 * @TMP108_REG_T_LOW: Low temperature register
 * @TMP108_REG_T_HIGH: High temperature register
 **/
typedef enum TMP108Reg {
    TMP108_REG_TEMPERATURE = 0,
    TMP108_REG_CONFIG,
    TMP108_REG_T_LOW,
    TMP108_REG_T_HIGH,
} TMP108Reg;

#endif
