/*
 * METAmorph development board with Chorus2 SoC.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 */

#include "boards.h"
#include "loader.h"

#include "chorus2.h"
#include "chorus2_spi.h"
#include "dataflash.h"

static void zero1dg_init(QEMUMachineInitArgs *args)
{
    const char *kernel_filename = args->kernel_filename;
    const char *kernel_cmdline = args->kernel_cmdline;
    const char *cpu_model = args->cpu_model;

    struct chorus2_state_s *soc;
    struct dataflash_s *flash;

    soc = chorus2_init(64 << 20, cpu_model, kernel_filename, kernel_cmdline);
    if (!soc) {
        if (cpu_model) {
            fprintf(stderr, "Chorus2 SoC failed to init with cpu '%s'\n",
                    cpu_model);
        } else {
            fprintf(stderr, "Chorus2 SoC failed to init\n");
        }
        exit(1);
    }

    flash = dataflash_init(AT45DB321x);
    chorus2_spi_device_init(soc->spi, 0, dataflash_spi_rxtx, dataflash_spi_setcs, flash);
}


static QEMUMachine zero1dg_machine = {
    .name = "01dg",
    .desc = "METAmorph development platform",
    .init = zero1dg_init,
    .is_default = 0,
};

static void zero1dg_machine_init(void)
{
    qemu_register_machine(&zero1dg_machine);
}

machine_init(zero1dg_machine_init);
