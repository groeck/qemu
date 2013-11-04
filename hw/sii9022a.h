#ifndef hw_sii9022a_h
# define hw_sii9022a_h "sii9022a.h"

#include "console.h"

typedef void (sii9022a_cb_reschange)(int width, int height, void *user);

void sii9022a_setup(DeviceState *dev, qemu_irq irq, sii9022a_cb_reschange *reschange, void *cb_user);

#endif
