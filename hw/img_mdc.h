#ifndef hw_img_mdc_h
# define hw_img_mdc_h "img_mdc.h"

#include <inttypes.h>
#include "hwaddr.h"
#include "qemu-common.h"
#include "memory.h"
#include "irq.h"

#define IMG_MDC_SIZE               0x1000

struct img_mdc_state_s;

MemoryRegion *img_mdc_iomem(struct img_mdc_state_s *mdc);
struct soc_dma_s *img_mdc_dma(struct img_mdc_state_s *mdc);

struct img_mdc_state_s *img_mdc_init(unsigned int chans, unsigned int perips,
                                     hwaddr base, qemu_irq *irqs);
void img_mdc_reset(struct img_mdc_state_s *mdc);
unsigned int img_mdc_get_perip(struct img_mdc_state_s *mdc, unsigned int chan);
void img_mdc_set_perip(struct img_mdc_state_s *mdc, unsigned int chan,
                       unsigned int perip);

#endif
