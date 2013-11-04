/*
 * Atmel DataFlash over SPI (AT45)
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#ifndef hw_dataflash_h
# define hw_dataflash_h "dataflash.h"

#include "qemu-common.h"
#include <inttypes.h>

struct dataflash_s;

enum {
    AT45DB321x,
    AT45DB642x,
};

void dataflash_spi_setcs(void *opaque, unsigned int cs);
void dataflash_spi_rxtx(void *opaque, size_t len,
                        uint8_t *rx, uint8_t *tx);
struct dataflash_s *dataflash_init(int type);
void dataflash_otp_set(struct dataflash_s *at, const uint8_t *otp, size_t sz);

#endif
