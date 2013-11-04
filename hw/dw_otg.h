#ifndef hw_dw_otg_h
# define hw_dw_otg_h "dw_otg.h"

#include <inttypes.h>
#include "hwaddr.h"
#include "qemu-common.h"
#include "memory.h"

#define DW_OTG_SIZE               0xf000

struct dw_otg_state;

struct dw_otg_state *dw_otg_init(hwaddr base);
MemoryRegion *dw_otg_iomem(struct dw_otg_state *d);

#endif
