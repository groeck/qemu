/* Communication with DA-Sim and CODESCAPE via SAP */

#include "dasim.h"
#include "sap.h"
#include "core.h"
#include "qemu-thread.h"
#include "sysemu.h"

#define STUBAPI_IOCTL_RESET 0x4

static QemuThread meta_dasim_thread;

static SAP_uint32 meta_dasim_get_processor_family(SAP_Target *self)
{
    return SAP_PROCESSOR_FAMILY_IMG_META;
}

static SAP_uint32 meta_dasim_get_processor_version(SAP_Target *self)
{
    MetaCore *core = self->UserData;

    return core->global.core_rev;
}

/*
 * virtual memory access for debug (includes writing to IO).
 * This is based on cpu_memory_rw_debug, but doesn't use
 * cpu_physical_memory_write_rom for writing as it doesn't work with register
 * memory.
 */
int meta_dasim_rw(CPUArchState *env, target_ulong addr,
                  uint8_t *buf, int len, int is_write)
{
    int l;
    hwaddr phys_addr;

    while (len > 0) {
        phys_addr = cpu_get_phys_page_debug(env, addr);
        /* if no physical page mapped, return an error */
        if (phys_addr == -1) {
            return -1;
        }
        l = ((addr & TARGET_PAGE_MASK) + TARGET_PAGE_SIZE) - addr;
        if (l > len) {
            l = len;
        }
        cpu_physical_memory_rw(phys_addr, buf, l, is_write);
        len -= l;
        buf += l;
        addr += l;
    }
    return 0;
}

static void meta_dasim_read(SAP_Target *self, SAP_uint32 thread,
                            SAP_uint32 address, SAP_uint32 size,
                            SAP_uint32 type, void *data)
{
    MetaCore *core = self->UserData;
    CPUArchState *env = &core->threads[thread].env;

    if (unlikely(thread >= core->num_threads)) {
        memset(data, 0, size);
        return;
    }

    if (meta_dasim_rw(env, address, data, size, 0) < 0) {
        memset(data, 0, size);
    }
}

static void meta_dasim_write(SAP_Target *self, SAP_uint32 thread,
                             SAP_uint32 address, SAP_uint32 size,
                             SAP_uint32 type, const void *data)
{
    MetaCore *core = self->UserData;
    CPUArchState *env = &core->threads[thread].env;

    if (unlikely(thread >= core->num_threads)) {
        return;
    }

    meta_dasim_rw(env, address, (void *)data, size, 1);
}

static void meta_dasim_ioctl(struct SAP_TargetTag *self, SAP_uint32 thread,
                             SAP_uint32 ioctl_cmd, SAP_uint32 count,
                             SAP_uint32 *pioctl)
{
    switch (ioctl_cmd) {
    case STUBAPI_IOCTL_RESET:
        /* request a full reset */
        qemu_system_reset_request();
        break;

    default:
        fprintf(stderr, "Unhandled DA-sim ioctl T%d cmd=%#x", thread, ioctl_cmd);
        while (count--) {
            fprintf(stderr, " %08x", *pioctl);
            ++pioctl;
        }
        fprintf(stderr, "\n");
    };
}

static void meta_dasim_cleanup(void)
{
    SAP_Cleanup();
}

static void *meta_dasim_handler(void *opaque)
{
    /* FIXME exit gracefully, not mid-send! */
    atexit(&meta_dasim_cleanup);
    for (;;) {
        SAP_HandleMessage(30000);
    }
    return NULL;
}

int meta_dasim_setup(MetaCore *core)
{
    SAP_Target **targets;
    char arg_buf[64];
    char *argv[] = { (char *)"qemu", &arg_buf[0] };

    /* pass -sap argument on to SAP library (except shared which is default) */
    if (!sap_comms) {
        return 1;
    }
    if (strcmp(sap_comms, "shared")) {
        snprintf(arg_buf, sizeof(arg_buf), "-sap-comms=%s", sap_comms);
        if (SAP_ParseCommandLine(ARRAY_SIZE(argv), argv)) {
            goto fail;
        }
    }

    targets = g_malloc(sizeof(SAP_Target *)*2);
    targets[0] = g_malloc(sizeof(SAP_Target));
    targets[1] = NULL;

    memset(targets[0], 0, sizeof(SAP_Target));
    targets[0]->UserData = core;
    targets[0]->GetProcessorFamily = &meta_dasim_get_processor_family;
    targets[0]->GetProcessorVersion = &meta_dasim_get_processor_version;
    targets[0]->Read = &meta_dasim_read;
    targets[0]->Write = &meta_dasim_write;
    targets[0]->IOCtl = &meta_dasim_ioctl;

    if (!SAP_Initialise(targets)) {
        goto fail;
    }

    /* start a dedicated thread to handle SAP messages */
    qemu_thread_create(&meta_dasim_thread, &meta_dasim_handler, core,
                       QEMU_THREAD_JOINABLE);

    return 0;
fail:
    sap_comms = NULL;
    return 1;
}
