/*
 * Freescale / NXP PFUZE100, 200, 300 emulation.
 * Supported devices:
 *   PFUZE100
 *   PFUZE200
 *   PFUZE300
 *
 * Copyright (C) 2024 Guenter Roeck <linux@roeck-us.net>
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"

#define TYPE_PFUZE      "pfuze"
#define TYPE_PFUZE100   "pfuze100"
#define TYPE_PFUZE200   "pfuze200"
#define TYPE_PFUZE3000  "pfuze3000"
#define TYPE_PFUZE3001  "pfuze3001"

OBJECT_DECLARE_TYPE(PFUZExxxI2CState, PFUZExxxClass, PFUZE)

#define NR_REGS         0x100

/* A simple I2C slave which returns values of ID or CNT register. */
typedef struct PFUZExxxI2CState {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/
    uint8_t regs[NR_REGS];  /* peripheral registers */
    uint8_t ptr;            /* current register index */
    uint8_t count;          /* counter used for tx/rx */
} PFUZExxxI2CState;

typedef struct PFUZExxxClass {
    /*< private >*/
    I2CSlaveClass parent_class;
    /*< public >*/
    void (*reset_enter)(PFUZExxxI2CState *s, ResetType type);
} PFUZExxxClass;

#define PFUZE100_DEVICEID       0x00
#define PFUZE100_REVID          0x10

#define PFUZE200_DEVICEID       0x01
#define PFUZE200_REVID          0x10

#define PFUZE3000_DEVICEID      0x30
#define PFUZE3000_REVID         0x10

#define PFUZE3001_DEVICEID      0x31
#define PFUZE3001_REVID         0x10

/* Reset all counters and load ID register */
static void pfuze100_reset_enter(PFUZExxxI2CState *s, ResetType type)
{
    s->regs[0x00] = PFUZE100_DEVICEID;
    s->regs[0x01] = PFUZE100_REVID;

    s->regs[0x06] = 0x3f;
    s->regs[0x09] = 0x7f;
    s->regs[0x0f] = 0x81;
    s->regs[0x12] = 0x3f;
    s->regs[0x1b] = 0x10;
    s->regs[0x23] = 0x08;
    s->regs[0x31] = 0x08;
    s->regs[0x38] = 0x08;
    s->regs[0x39] = 0x10;
    s->regs[0x3f] = 0x08;
    s->regs[0x40] = 0x20;
    s->regs[0x46] = 0x08;
    s->regs[0x47] = 0x20;
    s->regs[0x4d] = 0x08;
    s->regs[0x4e] = 0x30;
    s->regs[0x66] = 0x08;
}


/* Reset all counters and load ID register */
static void pfuze200_reset_enter(PFUZExxxI2CState *s, ResetType type)
{
    s->regs[0x00] = PFUZE200_DEVICEID;
    s->regs[0x01] = PFUZE200_REVID;

    s->regs[0x06] = 0x3f;
    s->regs[0x09] = 0x7f;
    s->regs[0x0f] = 0x81;
    s->regs[0x12] = 0x3f;
    s->regs[0x1b] = 0x10;
    s->regs[0x23] = 0x08;
    s->regs[0x38] = 0x08;
    s->regs[0x39] = 0x10;
    s->regs[0x3f] = 0x08;
    s->regs[0x40] = 0x20;
    s->regs[0x46] = 0x08;
    s->regs[0x47] = 0x20;
    s->regs[0x4e] = 0x30;
    s->regs[0x66] = 0x08;
}

/* Reset all counters and load ID register */
static void pfuze3000_reset_enter(PFUZExxxI2CState *s, ResetType type)
{
    s->regs[0x00] = PFUZE3000_DEVICEID;
    s->regs[0x01] = PFUZE3000_REVID;

    s->regs[0x06] = 0x3f;
    s->regs[0x09] = 0x7f;
    s->regs[0x0f] = 0xc5;
    s->regs[0x12] = 0x3f;
    s->regs[0x1b] = 0x10;
    s->regs[0x24] = 0x08;
    s->regs[0x2e] = 0x12;
    s->regs[0x2f] = 0x12;
    s->regs[0x30] = 0x12;
    s->regs[0x31] = 0x18;
    s->regs[0x32] = 0x44;
    s->regs[0x35] = 0x06;
    s->regs[0x38] = 0x28;
    s->regs[0x39] = 0x14;
    s->regs[0x3c] = 0x0c;
    s->regs[0x3d] = 0x0c;
    s->regs[0x3e] = 0x0c;
    s->regs[0x3f] = 0x38;
    s->regs[0x40] = 0x24;
    s->regs[0x66] = 0x08;
    s->regs[0x6b] = 0x06;
    s->regs[0x6c] = 0x4e;
    s->regs[0x6d] = 0x08;
    s->regs[0x6e] = 0x02;
    s->regs[0x6f] = 0x02;
    s->regs[0xff] = 0x08;
}

/* Reset all counters and load ID register */
static void pfuze3001_reset_enter(PFUZExxxI2CState *s, ResetType type)
{
    s->regs[0x00] = PFUZE3001_DEVICEID;
    s->regs[0x01] = PFUZE3001_REVID;

    s->regs[0x06] = 0x3f;
    s->regs[0x09] = 0x7f;
    s->regs[0x0f] = 0xc5;
    s->regs[0x12] = 0x3f;
    s->regs[0x1b] = 0x10;
    s->regs[0x31] = 0x10;
    s->regs[0x32] = 0x40;
    s->regs[0x35] = 0x06;
    s->regs[0x38] = 0x28;
    s->regs[0x39] = 0x14;
    s->regs[0x3c] = 0x0c;
    s->regs[0x3d] = 0x0c;
    s->regs[0x3e] = 0x0c;
    s->regs[0x3f] = 0x38;
    s->regs[0x40] = 0x20;
    s->regs[0x6b] = 0x06;
    s->regs[0x6c] = 0x4e;
    s->regs[0x6d] = 0x08;
    s->regs[0x6e] = 0x02;
    s->regs[0x6f] = 0x02;
    s->regs[0x70] = 0x40;
}

static void pfuze_reset_enter(Object *obj, ResetType type)
{
    PFUZExxxI2CState *s = PFUZE(obj);
    PFUZExxxClass *sc = PFUZE_GET_CLASS(s);

    memset(s->regs, 0, NR_REGS);
    s->ptr = 0;
    s->count = 0;

    sc->reset_enter(s, type);
}

/* Handle events from master. */
static int pfuze_event(I2CSlave *i2c, enum i2c_event event)
{
    PFUZExxxI2CState *s = PFUZE(i2c);

    s->count = 0;

    return 0;
}

/* Called when master requests read */
static uint8_t pfuze_rx(I2CSlave *i2c)
{
    PFUZExxxI2CState *s = PFUZE(i2c);

    return s->regs[s->ptr++];
}

/*
 * Called when master sends write.
 * Update ptr with byte 0, then perform write with second byte.
 */
static int pfuze_tx(I2CSlave *i2c, uint8_t data)
{
    PFUZExxxI2CState *s = PFUZE(i2c);

    if (s->count == 0) {
        /* Store register address */
        s->ptr = data;
        s->count++;
    } else {
        s->regs[s->ptr++] = data;
    }

    return 0;
}

static const VMStateDescription vmstate_pfuze = {
    .name = TYPE_PFUZE,
    .version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, PFUZExxxI2CState, NR_REGS),
        VMSTATE_UINT8(ptr, PFUZExxxI2CState),
        VMSTATE_UINT8(count, PFUZExxxI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void pfuze_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.enter = pfuze_reset_enter;
    dc->vmsd = &vmstate_pfuze;
    isc->event = pfuze_event;
    isc->recv = pfuze_rx;
    isc->send = pfuze_tx;
}

static const TypeInfo pfuze_info = {
    .name = TYPE_PFUZE,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PFUZExxxI2CState),
    .class_size = sizeof(PFUZExxxClass),
    .class_init = pfuze_class_init,
    .abstract = true,
};

static void pfuze100_class_init(ObjectClass *oc, void *data)
{
    PFUZExxxClass *sc = PFUZE_CLASS(oc);

    sc->reset_enter = pfuze100_reset_enter;
}

static const TypeInfo pfuze100_info = {
    .name = TYPE_PFUZE100,
    .parent = TYPE_PFUZE,
    .class_init = pfuze100_class_init
};

static void pfuze200_class_init(ObjectClass *oc, void *data)
{
    PFUZExxxClass *sc = PFUZE_CLASS(oc);

    sc->reset_enter = pfuze200_reset_enter;
}

static const TypeInfo pfuze200_info = {
    .name = TYPE_PFUZE200,
    .parent = TYPE_PFUZE,
    .class_init = pfuze200_class_init
};

static void pfuze3000_class_init(ObjectClass *oc, void *data)
{
    PFUZExxxClass *sc = PFUZE_CLASS(oc);

    sc->reset_enter = pfuze3000_reset_enter;
}

static const TypeInfo pfuze3000_info = {
    .name = TYPE_PFUZE3000,
    .parent = TYPE_PFUZE,
    .class_init = pfuze3000_class_init,
};

static void pfuze3001_class_init(ObjectClass *oc, void *data)
{
    PFUZExxxClass *sc = PFUZE_CLASS(oc);

    sc->reset_enter = pfuze3001_reset_enter;
}

static const TypeInfo pfuze3001_info = {
    .name = TYPE_PFUZE3001,
    .parent = TYPE_PFUZE,
    .class_init = pfuze3001_class_init,
};

static void pfuze_register_devices(void)
{
    type_register_static(&pfuze_info);
    type_register_static(&pfuze100_info);
    type_register_static(&pfuze200_info);
    type_register_static(&pfuze3000_info);
    type_register_static(&pfuze3001_info);
}

type_init(pfuze_register_devices);
