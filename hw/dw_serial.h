#ifndef __hw_dw_serial_h__
#define __hw_dw_serial_h__

#include "serial.h"

typedef struct DwSerialState DwSerialState;

DwSerialState *dw_serial_mm_init(MemoryRegion *address_space,
                                 hwaddr base, qemu_irq irq, int baudbase,
                                 CharDriverState *chr, enum device_endian end);

#endif /* __hw_dw_serial_h__ */
