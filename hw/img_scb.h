#ifndef hw_img_scb_h
# define hw_img_scb_h "img_scb.h"

#include <inttypes.h>
#include "hwaddr.h"
#include "qemu-common.h"
#include "memory.h"
#include "i2c.h"

#define IMG_SCB_SIZE              0x200

struct img_scb_state_s;

struct img_scb_state_s *img_scb_init(hwaddr base, qemu_irq irq);
MemoryRegion *img_scb_iomem(struct img_scb_state_s *scb);
i2c_bus *img_scb_bus(struct img_scb_state_s *scb);

#endif
