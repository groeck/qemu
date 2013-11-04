#ifndef __META_DAMISC_H__
#define __META_DAMISC_H__

#include "meta_switch.h"

ssize_t meta_damisc_chan_read(MetaSwitchDevice *dev, CPUArchState *env,
                              int chan_idx, uint8_t *buf, size_t sz);
ssize_t meta_damisc_chan_write(MetaSwitchDevice *dev, CPUArchState *env,
                               int chan_idx, const uint8_t *buf, size_t sz);

int meta_damisc_chan_errno(int err);

#endif /* __META_DAMISC_H__ */
