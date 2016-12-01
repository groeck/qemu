/*
 * Texas Instruments TMP108 Temperature Sensor
 *
 * Browse the data sheet:
 *
 *    http://www.ti.com/lit/gpn/tmp108
 *
 * Copyright (C) 2012 Alex Horn <alex.horn@cs.ox.ac.uk>
 * Copyright (C) 2008-2012 Andrzej Zaborowski <balrogg@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */
#ifndef QEMU_TMP108_H
#define QEMU_TMP108_H

#include "hw/i2c/i2c.h"
#include "hw/sensor/tmp108_regs.h"

#define TYPE_TMP108 "tmp108"
#define TMP108(obj) OBJECT_CHECK(TMP108State, (obj), TYPE_TMP108)

/**
 * TMP108State:
 * @config: Bits 5 and 6 (value 32 and 64) determine the precision of the
 * temperature. See Table 8 in the data sheet.
 *
 * @see_also: http://www.ti.com/lit/ds/symlink/tmp108.pdf
 */
typedef struct TMP108State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq pin;

    uint8_t pointer;
    uint16_t config;
    int16_t temperature;
    int16_t limit[2];
    int faults;
    uint8_t alarm;
    uint8_t low_alarm;
    uint8_t high_alarm;
} TMP108State;

#endif
