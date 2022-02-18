/*
 * Texas Instruments TMP464/TMP468 temperature sensor.
 *
 * Copyright (c) 2022 Guenter Roeck.
 *
 * Largely inspired by :
 *
 * Texas Instruments TMP421 temperature sensor.
 *
 * Copyright (c) 2016 IBM Corporation.
 *
 * Largely inspired by :
 *
 * Texas Instruments TMP105 temperature sensor.
 *
 * Copyright (C) 2008 Nokia Corporation
 * Written by Andrzej Zaborowski <andrew@openedhand.com>
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
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "qom/object.h"

/* Manufacturer / Device ID's */
#define TMP464_MANUFACTURER_ID          0x5449
#define TMP464_DEVICE_ID                0x1468
#define TMP468_DEVICE_ID                0x0468

typedef struct DeviceInfo {
    int model;
    const char *name;
} DeviceInfo;

static const DeviceInfo devices[] = {
    { TMP464_DEVICE_ID, "tmp464" },
    { TMP468_DEVICE_ID, "tmp468" },
};

struct TMP464State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    int16_t temperature[9];
    int16_t therm_limit[9];
    int16_t therm2_limit[9];
    int16_t offset[8];
    uint8_t nfactor[8];

    uint16_t status[3];
    uint16_t config;
    uint16_t hysteresis;

    bool locked;
    uint8_t channels;

    uint8_t len;
    uint8_t pointer;
    uint8_t buf[2];

    qemu_irq therm_pin;
    qemu_irq therm2_pin;
};

struct TMP464Class {
    I2CSlaveClass parent_class;
    DeviceInfo *dev;
};

#define TYPE_TMP464 "tmp464-generic"
OBJECT_DECLARE_TYPE(TMP464State, TMP464Class, TMP464)

/* the TMP464 registers */
#define TMP464_THERM_STATUS_REG         0x21
#define TMP464_THERM2_STATUS_REG        0x22
#define TMP464_OPEN_STATUS_REG          0x23
#define TMP464_CONFIG_REG               0x30
#define    TMP464_BUSY                    (1 << 1)
#define    TMP464_CONVRATE_B0             (1 << 2)
#define    TMP464_CONVRATE_B1             (1 << 3)
#define    TMP464_CONVRATE_B2             (1 << 4)
#define    TMP464_CONFIG_SHUTDOWN         (1 << 5)
#define    TMP464_CONFIG_ONESHOT          (1 << 6)

#define    TMP464_CHANNEL_MASK(ch)      (1 << ((ch) + 7))

#define TMP464_HYSTERESIS_REG           0x38

#define TMP464_RESET_REG                0x20
#define    TMP464_RESET                 (1 << 15)
#define TMP464_LOCK_REG                 0xc4
#define    TMP464_LOCK_VAL              0x5ca6
#define    TMP464_UNLOCK_VAL            0xeb19
#define    TMP464_LOCKED                0x8000

#define TMP464_MANUFACTURER_ID_REG      0xFE
#define TMP464_DEVICE_ID_REG            0xFF

#define TMP464_TEMP_REG(ch)             ((ch) + 0x00)
#define TMP464_THERM_LIMIT_REG(ch)      ((ch) ? ((ch) - 1) * 0x08 + 0x42 : 0x39)
#define TMP464_THERM2_LIMIT_REG(ch)     ((ch) ? ((ch) - 1) * 0x08 + 0x43 : 0x3a)
#define TMP464_TEMP_OFFSET_REG(ch)      (((ch) - 1) * 0x08 + 0x40)
#define TMP464_TEMP_NFACTOR_REG(ch)     (((ch) - 1) * 0x08 + 0x41)

static void tmp464_interrupt_update(TMP464State *s)
{
    if (s->config & TMP464_CONFIG_SHUTDOWN) {
        qemu_set_irq(s->therm_pin, 1);
        qemu_set_irq(s->therm2_pin, 1);
    } else {
        qemu_set_irq(s->therm_pin, !(s->status[0] & (s->config & 0xff80)));
        qemu_set_irq(s->therm2_pin, !(s->status[1] & (s->config & 0xff80)));
    }
}

static void tmp464_alarm_update(TMP464State *s)
{
    int16_t temp, temp_hyst;
    int ch;

    if (s->config & TMP464_CONFIG_SHUTDOWN) {
        tmp464_interrupt_update(s);
        return;
    }

    if (s->config & TMP464_CONFIG_ONESHOT) {
        s->config &= ~TMP464_CONFIG_ONESHOT;
    }

    for (ch = 0; ch < s->channels; ch++) {
        if (!(s->config & TMP464_CHANNEL_MASK(ch))) {
            s->status[0] &= ~TMP464_CHANNEL_MASK(ch);
            s->status[1] &= ~TMP464_CHANNEL_MASK(ch);
            s->status[2] &= ~TMP464_CHANNEL_MASK(ch);
            continue;
        }
        temp = s->temperature[ch];
        temp_hyst = temp - s->hysteresis;
        if (temp >= s->therm_limit[ch]) {
            s->status[0] |= TMP464_CHANNEL_MASK(ch);
        } else if (temp_hyst <= s->therm_limit[ch]) {
            s->status[0] &= ~TMP464_CHANNEL_MASK(ch);
        }
        if (temp >= s->therm2_limit[ch]) {
            s->status[1] |= TMP464_CHANNEL_MASK(ch);
        } else if (temp_hyst <= s->therm2_limit[ch]) {
            s->status[1] &= ~TMP464_CHANNEL_MASK(ch);
        }
    }

    tmp464_interrupt_update(s);
}

static void tmp464_get_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMP464State *s = TMP464(obj);
    int64_t value;
    int tempid;

    if (sscanf(name, "temperature%d", &tempid) != 1) {
        error_setg(errp, "error reading %s: %s", name, g_strerror(errno));
        return;
    }

    if (tempid >= s->channels || tempid < 0) {
        error_setg(errp, "error reading %s", name);
        return;
    }

    value = (s->temperature[tempid] * 625 + 40) / 80;

    visit_type_int(v, name, &value, errp);
}

/*
 * Units are 0.001 centigrades relative to 0 C.
 */
static void tmp464_set_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMP464State *s = TMP464(obj);
    int64_t temp;
    int tempid;

    if (!visit_type_int(v, name, &temp, errp)) {
        return;
    }

    if (temp > 255000 || temp < -255000) {
        error_setg(errp, "value %" PRId64 ".%03" PRIu64 " C is out of range",
                   temp / 1000, temp % 1000);
        return;
    }

    if (sscanf(name, "temperature%d", &tempid) != 1) {
        error_setg(errp, "error reading %s: %s", name, g_strerror(errno));
        return;
    }

    if (tempid >= s->channels || tempid < 0) {
        error_setg(errp, "error reading %s", name);
        return;
    }

    s->temperature[tempid] = (int16_t) (((temp * 10 + 312) / 625) << 3);

    tmp464_alarm_update(s);
}

static void tmp464_read(TMP464State *s)
{
    TMP464Class *sc = TMP464_GET_CLASS(s);
    int channel;

    s->len = 0;

    switch (s->pointer) {
    case TMP464_TEMP_REG(0) ... TMP464_TEMP_REG(8):
        s->buf[s->len++] = s->temperature[s->pointer] >> 8;
        s->buf[s->len++] = s->temperature[s->pointer] & 0xff;
        break;
    case TMP464_HYSTERESIS_REG:
        s->buf[s->len++] = s->hysteresis >> 8;
        s->buf[s->len++] = s->hysteresis & 0xff;
        break;
    case TMP464_THERM_LIMIT_REG(0):
        s->buf[s->len++] = s->therm_limit[0] >> 8;
        s->buf[s->len++] = s->therm_limit[0] & 0xff;
        break;
    case TMP464_THERM_LIMIT_REG(1):
    case TMP464_THERM_LIMIT_REG(2):
    case TMP464_THERM_LIMIT_REG(3):
    case TMP464_THERM_LIMIT_REG(4):
    case TMP464_THERM_LIMIT_REG(5):
    case TMP464_THERM_LIMIT_REG(6):
    case TMP464_THERM_LIMIT_REG(7):
    case TMP464_THERM_LIMIT_REG(8):
        channel = (s->pointer - TMP464_THERM_LIMIT_REG(1)) / 8 + 1;
        s->buf[s->len++] = s->therm_limit[channel] >> 8;
        s->buf[s->len++] = s->therm_limit[channel] & 0xff;
        break;
    case TMP464_THERM2_LIMIT_REG(0):
        s->buf[s->len++] = s->therm2_limit[0] >> 8;
        s->buf[s->len++] = s->therm2_limit[0] & 0xff;
        break;
    case TMP464_THERM2_LIMIT_REG(1):
    case TMP464_THERM2_LIMIT_REG(2):
    case TMP464_THERM2_LIMIT_REG(3):
    case TMP464_THERM2_LIMIT_REG(4):
    case TMP464_THERM2_LIMIT_REG(5):
    case TMP464_THERM2_LIMIT_REG(6):
    case TMP464_THERM2_LIMIT_REG(7):
    case TMP464_THERM2_LIMIT_REG(8):
        channel = (s->pointer - TMP464_THERM2_LIMIT_REG(1)) / 8 + 1;
        s->buf[s->len++] = s->therm2_limit[channel] >> 8;
        s->buf[s->len++] = s->therm2_limit[channel] & 0xff;
        break;
    case TMP464_TEMP_OFFSET_REG(1):
    case TMP464_TEMP_OFFSET_REG(2):
    case TMP464_TEMP_OFFSET_REG(3):
    case TMP464_TEMP_OFFSET_REG(4):
    case TMP464_TEMP_OFFSET_REG(5):
    case TMP464_TEMP_OFFSET_REG(6):
    case TMP464_TEMP_OFFSET_REG(7):
    case TMP464_TEMP_OFFSET_REG(8):
        channel = (s->pointer - TMP464_TEMP_OFFSET_REG(1)) / 8;
        s->buf[s->len++] = s->offset[channel] >> 8;
        s->buf[s->len++] = s->offset[channel] & 0xff;
        break;
    case TMP464_TEMP_NFACTOR_REG(1):
    case TMP464_TEMP_NFACTOR_REG(2):
    case TMP464_TEMP_NFACTOR_REG(3):
    case TMP464_TEMP_NFACTOR_REG(4):
    case TMP464_TEMP_NFACTOR_REG(5):
    case TMP464_TEMP_NFACTOR_REG(6):
    case TMP464_TEMP_NFACTOR_REG(7):
    case TMP464_TEMP_NFACTOR_REG(8):
        channel = (s->pointer - TMP464_TEMP_NFACTOR_REG(1)) / 8;
        s->buf[s->len++] = s->nfactor[channel];
        s->buf[s->len++] = 0;
        break;
    case TMP464_MANUFACTURER_ID_REG:
        s->buf[s->len++] = TMP464_MANUFACTURER_ID >> 8;
        s->buf[s->len++] = TMP464_MANUFACTURER_ID & 0xff;
        break;
    case TMP464_DEVICE_ID_REG:
        s->buf[s->len++] = sc->dev->model >> 8;
        s->buf[s->len++] = sc->dev->model & 0xff;
        break;
    case TMP464_CONFIG_REG:
        s->buf[s->len++] = s->config >> 8;
        s->buf[s->len++] = s->config & 0xff;
        break;
    case TMP464_THERM_STATUS_REG:
        s->buf[s->len++] = s->status[0] >> 8;
        s->buf[s->len++] = s->status[0] & 0xff;
        break;
    case TMP464_THERM2_STATUS_REG:
        s->buf[s->len++] = s->status[1] >> 8;
        s->buf[s->len++] = s->status[1] & 0xff;
        break;
    case TMP464_OPEN_STATUS_REG:
        s->buf[s->len++] = s->status[2] >> 8;
        s->buf[s->len++] = s->status[2] & 0xff;
        break;
    case TMP464_LOCK_REG:
        if (s->locked) {
            s->buf[s->len++] = TMP464_LOCKED >> 8;
            s->buf[s->len++] = TMP464_LOCKED & 0xff;
        } else {
            s->buf[s->len++] = 0;
            s->buf[s->len++] = 0;
        }
        break;
    default:
        s->buf[s->len++] = 0;
        s->buf[s->len++] = 0;
        break;
    }
}

static void tmp464_reset(I2CSlave *i2c);

static void tmp464_write(TMP464State *s)
{
    uint16_t regval = s->buf[0] << 8 | s->buf[1];
    int16_t temp;
    int channel;

    switch (s->pointer) {
    case TMP464_CONFIG_REG:
        if (s->channels == 5) {
            s->config = regval & 0x0ffc;
        } else {
            s->config = regval & 0xfffc;
        }
        tmp464_alarm_update(s);
        break;
    case TMP464_HYSTERESIS_REG:
        s->hysteresis = regval & 0x7f80;
        break;
    case TMP464_THERM_LIMIT_REG(0):
        s->therm_limit[0] = regval & 0xffc0;
        tmp464_alarm_update(s);
        break;
    case TMP464_THERM_LIMIT_REG(1):
    case TMP464_THERM_LIMIT_REG(2):
    case TMP464_THERM_LIMIT_REG(3):
    case TMP464_THERM_LIMIT_REG(4):
    case TMP464_THERM_LIMIT_REG(5):
    case TMP464_THERM_LIMIT_REG(6):
    case TMP464_THERM_LIMIT_REG(7):
    case TMP464_THERM_LIMIT_REG(8):
        channel = (s->pointer - TMP464_THERM_LIMIT_REG(1)) / 8 + 1;
        s->therm_limit[channel] = regval & 0xffc0;
        tmp464_alarm_update(s);
        break;
    case TMP464_THERM2_LIMIT_REG(0):
        s->therm2_limit[0] = regval & 0xffc0;
        tmp464_alarm_update(s);
        break;
    case TMP464_THERM2_LIMIT_REG(1):
    case TMP464_THERM2_LIMIT_REG(2):
    case TMP464_THERM2_LIMIT_REG(3):
    case TMP464_THERM2_LIMIT_REG(4):
    case TMP464_THERM2_LIMIT_REG(5):
    case TMP464_THERM2_LIMIT_REG(6):
    case TMP464_THERM2_LIMIT_REG(7):
    case TMP464_THERM2_LIMIT_REG(8):
        channel = (s->pointer - TMP464_THERM2_LIMIT_REG(1)) / 8 + 1;
        s->therm2_limit[channel] = regval & 0xffc0;
        tmp464_alarm_update(s);
        break;
    case TMP464_TEMP_OFFSET_REG(1):
    case TMP464_TEMP_OFFSET_REG(2):
    case TMP464_TEMP_OFFSET_REG(3):
    case TMP464_TEMP_OFFSET_REG(4):
    case TMP464_TEMP_OFFSET_REG(5):
    case TMP464_TEMP_OFFSET_REG(6):
    case TMP464_TEMP_OFFSET_REG(7):
    case TMP464_TEMP_OFFSET_REG(8):
        channel = (s->pointer - TMP464_TEMP_OFFSET_REG(1)) / 8;
	temp = s->temperature[channel + 1] - s->offset[channel];
        s->offset[channel] = regval & 0xfff8;
	temp += s->offset[channel];
	s->temperature[channel + 1] = temp;
        tmp464_alarm_update(s);
        break;
    case TMP464_TEMP_NFACTOR_REG(1):
    case TMP464_TEMP_NFACTOR_REG(2):
    case TMP464_TEMP_NFACTOR_REG(3):
    case TMP464_TEMP_NFACTOR_REG(4):
    case TMP464_TEMP_NFACTOR_REG(5):
    case TMP464_TEMP_NFACTOR_REG(6):
    case TMP464_TEMP_NFACTOR_REG(7):
    case TMP464_TEMP_NFACTOR_REG(8):
        channel = (s->pointer - TMP464_TEMP_NFACTOR_REG(1)) / 8;
        s->nfactor[channel] = regval >> 8;
        break;
    case TMP464_LOCK_REG:
        if (regval == TMP464_LOCK_VAL) {
            s->locked = true;
        } else if (regval == TMP464_UNLOCK_VAL) {
            s->locked = false;
        }
        break;
    case TMP464_RESET_REG:
        if (regval & TMP464_RESET) {
            tmp464_reset(I2C_SLAVE(s));
        }
        break;
    default:
        break;
    }
}

static uint8_t tmp464_rx(I2CSlave *i2c)
{
    TMP464State *s = TMP464(i2c);

    if (s->len < 2) {
        return s->buf[s->len++];
    } else {
        return 0xff;
    }
}

static int tmp464_tx(I2CSlave *i2c, uint8_t data)
{
    TMP464State *s = TMP464(i2c);

    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else if (s->len <= 2) {
        s->buf[s->len - 1] = data;
        s->len++;
        if (s->len >= 3) {
            tmp464_write(s);
        }
    }
    return 0;
}

static int tmp464_event(I2CSlave *i2c, enum i2c_event event)
{
    TMP464State *s = TMP464(i2c);

    if (event == I2C_START_RECV) {
        tmp464_read(s);
    }

    s->len = 0;
    return 0;
}

static const VMStateDescription vmstate_tmp464 = {
    .name = "TMP464",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, TMP464State),
        VMSTATE_UINT8_ARRAY(buf, TMP464State, 2),
        VMSTATE_UINT8(pointer, TMP464State),
        VMSTATE_UINT16(config, TMP464State),
        VMSTATE_UINT16(hysteresis, TMP464State),
        VMSTATE_BOOL(locked, TMP464State),
        VMSTATE_UINT8(channels, TMP464State),
        VMSTATE_UINT16_ARRAY(status, TMP464State, 3),
        VMSTATE_INT16_ARRAY(temperature, TMP464State, 9),
        VMSTATE_INT16_ARRAY(therm_limit, TMP464State, 9),
        VMSTATE_INT16_ARRAY(therm2_limit, TMP464State, 9),
        VMSTATE_INT16_ARRAY(offset, TMP464State, 8),
        VMSTATE_UINT8_ARRAY(nfactor, TMP464State, 8),
        VMSTATE_I2C_SLAVE(i2c, TMP464State),
        VMSTATE_END_OF_LIST()
    }
};

static void tmp464_reset(I2CSlave *i2c)
{
    TMP464State *s = TMP464(i2c);
    TMP464Class *sc = TMP464_GET_CLASS(s);
    int i;

    s->pointer = 0;

    s->locked = true;

    switch (sc->dev->model) {
    case TMP464_DEVICE_ID:
        s->config = 0x0f9c;
        s->channels = 5;
        break;
    case TMP468_DEVICE_ID:
        s->config = 0xff9c;
        s->channels = 9;
        break;
    }

    for (i = 0; i < s->channels; i++) {
        s->therm_limit[i] = 0x7fc0;
        s->therm2_limit[i] = 0x7fc0;
    }

    for (i = 0; i < s->channels - 1; i++) {
        s->offset[i] = 0;
        s->nfactor[i] = 0;
    }
    s->status[0] = 0;
    s->status[1] = 0;
    s->status[2] = 0;
}

static void tmp464_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    TMP464State *s = TMP464(dev);

    qdev_init_gpio_out(&i2c->qdev, &s->therm_pin, 1);
    qdev_init_gpio_out(&i2c->qdev, &s->therm2_pin, 1);

    tmp464_reset(&s->i2c);
}

static void tmp464_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    TMP464Class *sc = TMP464_CLASS(klass);

    dc->realize = tmp464_realize;
    k->event = tmp464_event;
    k->recv = tmp464_rx;
    k->send = tmp464_tx;
    dc->vmsd = &vmstate_tmp464;
    sc->dev = (DeviceInfo *) data;

    object_class_property_add(klass, "temperature0", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature1", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature2", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature3", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature4", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature5", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature6", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature7", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
    object_class_property_add(klass, "temperature8", "int",
                              tmp464_get_temperature,
                              tmp464_set_temperature, NULL, NULL);
}

static const TypeInfo tmp464_info = {
    .name          = TYPE_TMP464,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TMP464State),
    .class_size    = sizeof(TMP464Class),
    .abstract      = true,
};

static void tmp464_register_types(void)
{
    int i;

    type_register_static(&tmp464_info);
    for (i = 0; i < ARRAY_SIZE(devices); ++i) {
        TypeInfo ti = {
            .name       = devices[i].name,
            .parent     = TYPE_TMP464,
            .class_init = tmp464_class_init,
            .class_data = (void *) &devices[i],
        };
        type_register(&ti);
    }
}

type_init(tmp464_register_types)
