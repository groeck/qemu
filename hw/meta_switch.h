#ifndef __META_SWITCH_H__
#define __META_SWITCH_H__

#include "qdev.h"

typedef enum {
    META_SWITCH_USER     = 0x0,
    META_SWITCH_INT      = 0x1,
    META_SWITCH_THREAD   = 0x2,
    META_SWITCH_EXTERNAL = 0x3,
} MetaSwitchScope;

typedef enum {
    META_SWITCH_FILE     = 0x00,
    META_SWITCH_LOGF     = 0x01,
    META_SWITCH_UNEXP    = 0x02,
    META_SWITCH_MISC     = 0x03,
    META_SWITCH_LINUX    = 0x04,

    META_SWITCH_BREAK    = 0x3f,
    META_SWITCH_SUBGROUP_MAX,
} MetaSwitchSubgroup;

typedef struct MetaSwitchBus MetaSwitchBus;
typedef struct MetaSwitchDevice MetaSwitchDevice;
typedef struct MetaSwitchDeviceClass MetaSwitchDeviceClass;

struct MetaSwitchBus {
    BusState qbus;
    MetaSwitchDevice *devices[META_SWITCH_SUBGROUP_MAX][META_MAX_THREADS];
};

MetaSwitchBus *meta_switch_bus_new(DeviceState *dev);
bool meta_switch_bus_has_device(MetaSwitchBus *bus);
MetaSwitchDevice *meta_switch_bus_lookup_device(MetaSwitchBus *bus,
                                                CPUArchState *env,
                                                uint8_t subgroup);

#define TYPE_META_SWITCH_BUS "META_SWITCH"
#define META_SWITCH_BUS(obj) OBJECT_CHECK(MetaSwitchBus, (obj), \
                                          TYPE_META_SWITCH_BUS)

struct MetaSwitchDevice {
    DeviceState qdev;
    uint8_t thread_mask;
    int (*handler)(MetaSwitchDevice *dev, CPUArchState *env,
                   const uint32_t *args, int nargs,
                   uint32_t *ret, int nret);
};

struct MetaSwitchDeviceClass {
    DeviceClass parent_class;
    uint32_t subgroup;
    bool per_thread;
    int (*init)(MetaSwitchDevice *dev);
    int (*destroy)(MetaSwitchDevice *dev);
};

#define TYPE_META_SWITCH_DEVICE "meta-switch-device"
#define META_SWITCH_DEVICE(obj) \
     OBJECT_CHECK(MetaSwitchDevice, (obj), TYPE_META_SWITCH_DEVICE)
#define META_SWITCH_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(MetaSwitchDeviceClass, (klass), TYPE_META_SWITCH_DEVICE)
#define META_SWITCH_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(MetaSwitchDeviceClass, (obj), TYPE_META_SWITCH_DEVICE)

int meta_switch_handle(CPUArchState *env, uint32_t swtch, target_ulong stp);
char *meta_switch_arg_string(CPUArchState *env, target_ulong addr, uint32_t max_len);

static inline MetaSwitchBus *meta_switch_bus_from_device(MetaSwitchDevice *dev)
{
    return FROM_QBUS(MetaSwitchBus, qdev_get_parent_bus(&dev->qdev));
}

static inline MetaSwitchDevice *meta_switch_sibling(MetaSwitchDevice *dev,
                                                    CPUArchState *env,
                                                    uint8_t subgroup)
{
    MetaSwitchBus *bus = meta_switch_bus_from_device(dev);
    return meta_switch_bus_lookup_device(bus, env, subgroup);
}

#endif /* __META_SWITCH_H__ */
