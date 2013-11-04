/*
 * METAmorph development board with Comet SoC.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#include "boards.h"
#include "loader.h"

#include "comet.h"
#include "comet_spim.h"
#include "dataflash.h"
#include "img_scb.h"
#include "qt5480.h"
#include "sii9022a.h"
#include "comet_pdp.h"

static const uint8_t otp[64] = {
    0x30, 0x31, 0x53, 0x50, 0x4d, 0x4e, 0x32, 0x32, /* 01SPMN22 */
    0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, /* 000000.. */
    0x32, 0x30, 0x31, 0x31, 0x2d, 0x30, 0x38, 0x2d, /* 2011-08- */
    0x30, 0x35, 0x00, 0x00, 0x32, 0x30, 0x31, 0x31, /* 05..2011 */
    0x2d, 0x30, 0x38, 0x2d, 0x30, 0x35, 0x00, 0x00, /* -08-05.. */
    0x49, 0x6d, 0x61, 0x67, 0x69, 0x6e, 0x61, 0x74, /* Imaginat */
    0x69, 0x6f, 0x6e, 0x54, 0x65, 0x63, 0x68, 0x6e, /* ionTechn */
    0x6f, 0x6c, 0x6f, 0x67, 0x69, 0x65, 0x73, 0x00, /* ologies. */
};

static void zero1sp_reschange(int width, int height, void *user)
{
    struct comet_state_s *soc = user;

    comet_pdp_set_res(soc->pdp, width, height);
}

static void zero1sp_init(QEMUMachineInitArgs *args)
{
    const char *kernel_filename = args->kernel_filename;
    const char *kernel_cmdline = args->kernel_cmdline;
    const char *cpu_model = args->cpu_model;

    struct comet_state_s *soc;
    struct dataflash_s *flash;
    DeviceState *qt5480, *sii9022a;

    soc = comet_init(256 << 20, cpu_model, kernel_filename, kernel_cmdline);
    if (!soc) {
        if (cpu_model) {
            fprintf(stderr, "Comet SoC failed to init with cpu '%s'\n",
                    cpu_model);
        } else {
            fprintf(stderr, "Comet SoC failed to init\n");
        }
        exit(1);
    }

    flash = dataflash_init(AT45DB642x);
    comet_spim_device_init(soc->spim1, 0, dataflash_spi_rxtx, dataflash_spi_setcs, flash);
    dataflash_otp_set(flash, otp, sizeof(otp));

    qt5480 = i2c_create_slave(img_scb_bus(soc->scb2), "qt5480", 0);
    i2c_set_slave_address((I2CSlave *)qt5480, 0x30);
    qt5480_setup(qt5480, 255<<2, 255<<2, soc->gpios[COMET_GPIO_UART0_CTS]);

    sii9022a = i2c_create_slave(img_scb_bus(soc->scb2), "sii9022a", 0);
    i2c_set_slave_address((I2CSlave *)sii9022a, 0x39);
    sii9022a_setup(sii9022a, soc->gpios[COMET_GPIO_SPI1_CS2], zero1sp_reschange, soc);
}


static QEMUMachine zero1sp_machine = {
    .name = "01sp",
    .desc = "Comet METAmorph development platform",
    .init = zero1sp_init,
    .is_default = 1,
};

static void zero1sp_machine_init(void)
{
    qemu_register_machine(&zero1sp_machine);
}

machine_init(zero1sp_machine_init);
