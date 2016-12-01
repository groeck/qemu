/*
 * Texas Instruments TMP108 temperature sensor.
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
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "tmp108.h"
#include "qapi/error.h"
#include "qapi/visitor.h"

static void tmp108_interrupt_update(TMP108State *s)
{
    qemu_set_irq(s->pin, s->alarm ^ ((~s->config >> 15) & 1));        /* POL */
}

static void tmp108_alarm_update(TMP108State *s)
{
    if (((s->config >> 0) & 3) == 0) {                                /* SD */
        return;
    }
    if (((s->config >> 0) & 3) == 1) {                                /* OS */
        s->config &= ~1;
    }

    if ((s->config >> 2) & 1) {                         /* TM, interrrupt mode */
        if (s->temperature >= s->limit[1] && !s->high_alarm) {
            s->alarm = 1;
            s->high_alarm = 1;
            s->low_alarm = 0;
            s->config |= (1 << 4);
        } else if (s->temperature < s->limit[0] && !s->low_alarm) {
            s->alarm = 1;
            s->low_alarm = 1;
            s->high_alarm = 0;
            s->config |= (1 << 3);
        }
    } else {                                            /* comparator mode */
        int hyst;

        if (s->temperature > s->limit[1]) {
            s->alarm = 1;
            s->config |= (1 << 4);
        } else if (s->temperature < s->limit[0]) {
            s->alarm = 1;
            s->config |= (1 << 3);
        }
        hyst = (s->config >> 12) & 0x03;
        if (hyst == 3)        /* 0, 1, 2, 4 */
            hyst = 4;
        if (s->temperature <= s->limit[1] - hyst &&
            s->temperature >= s->limit[0] + hyst) {
            s->alarm = 0;
            s->config &= ~((1 << 3) | (1 << 4));
        }
    }

    tmp108_interrupt_update(s);
}

static void tmp108_get_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMP108State *s = TMP108(obj);
    int64_t value = s->temperature * 1000 / 256;

    visit_type_int(v, name, &value, errp);
}

/* Units are 0.001 centigrades relative to 0 C.  s->temperature is 8.8
 * fixed point, so units are 1/256 centigrades.  A simple ratio will do.
 */
static void tmp108_set_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMP108State *s = TMP108(obj);
    Error *local_err = NULL;
    int64_t temp;

    visit_type_int(v, name, &temp, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    if (temp >= 128000 || temp < -128000) {
        error_setg(errp, "value %" PRId64 ".%03" PRIu64 " °C is out of range",
                   temp / 1000, temp % 1000);
        return;
    }

    s->temperature = (int16_t) (temp * 256 / 1000);

    tmp108_alarm_update(s);
}

static void tmp108_read(TMP108State *s)
{
    s->len = 0;

    switch (s->pointer & 3) {
    case TMP108_REG_TEMPERATURE:
        s->buf[s->len++] = (((uint16_t) s->temperature) >> 8);
        s->buf[s->len++] = (((uint16_t) s->temperature) >> 0) & 0xf0;
        break;

    case TMP108_REG_CONFIG:
        s->buf[s->len++] = (s->config >> 8) & 0xb0;
        s->buf[s->len++] = s->config & 0xff;
        if ((s->config >> 2) & 1) {                        /* TM, interrupt mode */
            s->alarm = 0;
            s->config &= ~((1 << 3) | (1 << 4));
            tmp108_interrupt_update(s);
        }
        break;

    case TMP108_REG_T_LOW:
        s->buf[s->len++] = ((uint16_t) s->limit[0]) >> 8;
        s->buf[s->len++] = (((uint16_t) s->limit[0]) >> 0) & 0xf0;
        break;

    case TMP108_REG_T_HIGH:
        s->buf[s->len++] = ((uint16_t) s->limit[1]) >> 8;
        s->buf[s->len++] = (((uint16_t) s->limit[1]) >> 0) & 0xf0;
        break;
    }
}

static void tmp108_write(TMP108State *s)
{
    switch (s->pointer & 3) {
    case TMP108_REG_TEMPERATURE:
        break;

    case TMP108_REG_CONFIG:
        if (s->len >= 3) {
            s->config = ((((uint16_t)s->buf[0]) & 0xb0) << 8) | s->buf[1];
        }
        tmp108_alarm_update(s);
        break;

    case TMP108_REG_T_LOW:
    case TMP108_REG_T_HIGH:
        if (s->len >= 3) {
            s->limit[s->pointer & 1] = (int16_t)
                    ((((uint16_t) s->buf[0]) << 8) | (s->buf[1] & 0xf0));
        }
        tmp108_alarm_update(s);
        break;
    }
}

static uint8_t tmp108_rx(I2CSlave *i2c)
{
    TMP108State *s = TMP108(i2c);

    if (s->len < 2) {
        return s->buf[s->len++];
    } else {
        return 0xff;
    }
}

static int tmp108_tx(I2CSlave *i2c, uint8_t data)
{
    TMP108State *s = TMP108(i2c);

    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
        }
        s->len++;
        tmp108_write(s);
    }

    return 0;
}

static int tmp108_event(I2CSlave *i2c, enum i2c_event event)
{
    TMP108State *s = TMP108(i2c);

    if (event == I2C_START_RECV) {
        tmp108_read(s);
    }

    s->len = 0;

    return 0;
}

static int tmp108_post_load(void *opaque, int version_id)
{
    TMP108State *s = opaque;

    tmp108_interrupt_update(s);
    return 0;
}

static const VMStateDescription vmstate_tmp108 = {
    .name = "TMP108",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = tmp108_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, TMP108State),
        VMSTATE_UINT8_ARRAY(buf, TMP108State, 2),
        VMSTATE_UINT8(pointer, TMP108State),
        VMSTATE_UINT16(config, TMP108State),
        VMSTATE_INT16(temperature, TMP108State),
        VMSTATE_INT16_ARRAY(limit, TMP108State, 2),
        VMSTATE_UINT8(alarm, TMP108State),
        VMSTATE_UINT8(low_alarm, TMP108State),
        VMSTATE_UINT8(high_alarm, TMP108State),
        VMSTATE_I2C_SLAVE(i2c, TMP108State),
        VMSTATE_END_OF_LIST()
    }
};

static void tmp108_reset(I2CSlave *i2c)
{
    TMP108State *s = TMP108(i2c);

    s->temperature = 0;
    s->limit[0] = 0x8000;
    s->limit[1] = 0x7ff0;
    s->pointer = 0;
    s->config = 0x0126;
    s->alarm = 0;
    s->low_alarm = 0;
    s->high_alarm = 0;

    tmp108_interrupt_update(s);
}

static void tmp108_realize(DeviceState *dev, Error **errp)
{
    TMP108State *s = TMP108(dev);
    I2CSlave *i2c = I2C_SLAVE(dev);

    qdev_init_gpio_out(&i2c->qdev, &s->pin, 1);
    tmp108_reset(i2c);
}

static void tmp108_initfn(Object *obj)
{
    object_property_add(obj, "temperature", "int",
                        tmp108_get_temperature,
                        tmp108_set_temperature, NULL, NULL, NULL);
}

static void tmp108_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = tmp108_event;
    k->recv = tmp108_rx;
    k->send = tmp108_tx;
    dc->realize = tmp108_realize;
    dc->vmsd = &vmstate_tmp108;
}

static const TypeInfo tmp108_info = {
    .name          = TYPE_TMP108,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TMP108State),
    .instance_init = tmp108_initfn,
    .class_init    = tmp108_class_init,
};

static void tmp108_register_types(void)
{
    type_register_static(&tmp108_info);
}

type_init(tmp108_register_types)
