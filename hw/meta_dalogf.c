/*
 * ImgTec Debug Adapter (DA) LogF driver
 *
 * Copyright (C) 2013 Imagination Technologies Ltd.
 *
 * Authors:
 *  James Hogan <james.hogan@imgtec.com>
 *  Paul Burton <paul.burton@imgtec.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <stdarg.h>
#include <stdio.h>
#include "meta_damisc.h"
#include "meta_switch.h"

typedef struct MetaDALogFState {
    MetaSwitchDevice dev;
    char *filename;
    FILE *file;
} MetaDALogFState;

#define LOGF_MAX_LEN 0x1000

static void meta_da_printf(MetaDALogFState *lf, CPUArchState *env,
                           const char *fmt, ...)
{
    MetaSwitchDevice *damisc;
    va_list ap;

    damisc = meta_switch_sibling(&lf->dev, env, META_SWITCH_MISC);
    va_start(ap, fmt);

    if (damisc && (!lf->filename || !lf->file)) {
        uint8_t buf[LOGF_MAX_LEN];
        va_list ap_chan;
        ssize_t wrote;
        int sz;
        va_copy(ap_chan, ap);
        sz = vsnprintf((char *)buf, sizeof(buf), fmt, ap_chan);
        va_end(ap_chan);
        if (sz >= 0) {
            wrote = meta_damisc_chan_write(damisc, env, 1, buf, sz);
            if (wrote >= 0) {
                goto done;
            }
        }
    }

    vfprintf(lf->file, fmt, ap);
done:
    va_end(ap);
}

static void meta_logf(MetaDALogFState *lf, CPUArchState *env,
                      char *fmt, const uint32_t *args, int nargs)
{
    char *cur, *next;
    unsigned int fmts = 0;
    char chr;
    char fmt_type = 0;
    char len_type = 0;
    uint32_t dummy = 0xdeadbeef;
    char *str;
    for (cur = fmt; ; cur++) {
        chr = cur[0];
        /* skip "%%" */
        if (chr == '%' && cur[1] == '%') {
            cur++;
            continue;
        }
        /* start of new format, or end of string */
        if (!chr || chr == '%') {
            fmts++;
            if (!chr || fmts > 1) {
                if (!nargs) {
                    args = &dummy;
                }
                /* do a single chunk of format */
                cur[0] = '\0';
                switch (fmt_type) {
                case 'i':
                    switch (len_type) {
                    case 'H':
                        meta_da_printf(lf, env, fmt, (char)*args);
                        break;
                    case 'h':
                        meta_da_printf(lf, env, fmt, (short)*args);
                        break;
                    case 'l':
                        meta_da_printf(lf, env, fmt, (long)*args);
                        break;
                    case 'j': /* intmax_t */
                        meta_da_printf(lf, env, fmt, (intmax_t)*args);
                        break;
                    case 'z': /* size_t */
                        meta_da_printf(lf, env, fmt, (size_t)*args);
                        break;
                    case 't': /* ptrdiff_t */
                        meta_da_printf(lf, env, fmt, (ptrdiff_t)*args);
                        break;
                    case '\0':
                        meta_da_printf(lf, env, fmt, (int)*args);
                        break;
                    case 'L':
                        /* FIXME align/combine with next argument */
                    default:
                        meta_da_printf(lf, env, "%s", fmt);
                        break;
                    }
                    break;
                case 'f':
                    switch (len_type) {
                    case 'd': /* long double */
                        /* FIXME */
                    case '\0':
                        meta_da_printf(lf, env, fmt, (int)*args);
                        break;
                    default:
                        meta_da_printf(lf, env, "%s", fmt);
                        break;
                    }
                    break;
                case 'c':
                    meta_da_printf(lf, env, fmt, (char)*args);
                    break;
                case 'p':
                    meta_da_printf(lf, env, fmt, (void *)(uintptr_t)*args);
                    break;
                case 's':
                    str = meta_switch_arg_string(env, *args, 0x1000);
                    if (str) {
                        meta_da_printf(lf, env, fmt, str);
                        g_free(str);
                    } else {
                        meta_da_printf(lf, env, fmt, "<BADSTRPTR>");
                    }
                    break;
                default:
                    meta_da_printf(lf, env, "%s", fmt);
                    break;
                }
                cur[0] = chr;
                fmt = cur;
                if (!chr) {
                    return;
                }
                if (nargs) {
                    args++;
                    nargs--;
                }
            }
            fmt_type = 0;
            len_type = 0;
            for (next = cur; *next; next++) {
                switch (*next) {
                /* CONVERSIONS */
                case 'd':
                case 'i':
                case 'o':
                case 'u':
                case 'x':
                case 'X':
                    fmt_type = 'i';
                    goto end_fmt;
                case 'e':
                case 'E':
                case 'f':
                case 'F':
                case 'g':
                case 'G':
                case 'a':
                case 'A':
                    fmt_type = 'f';
                    goto end_fmt;
                case 'c':
                    fmt_type = 'c';
                    goto end_fmt;
                case 's':
                    fmt_type = 's';
                    goto end_fmt;
                case 'p':
                    fmt_type = 'p';
                    goto end_fmt;
                /* ASSUME LENGTH MODIFIERS ETC */
                case 'h':
                    if (len_type == 'h') {
                        len_type = 'H';
                    } else {
                        len_type = 'h';
                    }
                    break;
                case 'l':
                    if (len_type == 'l') {
                        len_type = 'L';
                    } else {
                        len_type = 'l';
                    }
                    break;
                case 'L': /* long double */
                    len_type = 'd';
                    break;
                case 'j': /* intmax_t */
                case 'z': /* size_t */
                case 't': /* ptrdiff_t */
                    len_type = *next;
                    break;
                default:
                    break;
                }
            }
            next--;
end_fmt:
            cur = next;
        }
    }
}

static int meta_dalogf_handler(MetaSwitchDevice *dev, CPUArchState *env,
                               const uint32_t *args, int nargs,
                               uint32_t *ret, int nret)
{
    MetaDALogFState *lf = DO_UPCAST(MetaDALogFState, dev, dev);
    char *fmt;

    fmt = meta_switch_arg_string(env, args[0], LOGF_MAX_LEN);
    if (fmt) {
        meta_logf(lf, env, fmt, args + 1, nargs - 1);
        g_free(fmt);
    }

    return 0;
}

static int meta_dalogf_initfn(MetaSwitchDevice *dev)
{
    MetaDALogFState *lf = DO_UPCAST(MetaDALogFState, dev, dev);

    if (lf->filename) {
        lf->file = fopen(lf->filename, "w");
        if (!lf->file) {
            lf->file = stdout;
            fprintf(stderr, "Failed to open LogF output file '%s'\n",
                    lf->filename);
            return -1;
        }
    } else {
        lf->file = stdout;
    }

    dev->handler = meta_dalogf_handler;

    return 0;
}

static int meta_dalogf_destroy(MetaSwitchDevice *dev)
{
    MetaDALogFState *lf = DO_UPCAST(MetaDALogFState, dev, dev);

    if (lf->filename && lf->file) {
        fclose(lf->file);
    }

    return 0;
}

static Property meta_dalogf_properties[] = {
    DEFINE_PROP_STRING("filename", MetaDALogFState, filename),
    DEFINE_PROP_END_OF_LIST(),
};

static void meta_dalogf_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    MetaSwitchDeviceClass *sc = META_SWITCH_DEVICE_CLASS(klass);
    sc->init = meta_dalogf_initfn;
    sc->destroy = meta_dalogf_destroy;
    sc->subgroup = META_SWITCH_LOGF;
    dc->props = meta_dalogf_properties;
}

static TypeInfo meta_dalogf_info = {
    .name          = "dalogf",
    .parent        = TYPE_META_SWITCH_DEVICE,
    .instance_size = sizeof(MetaDALogFState),
    .class_init    = meta_dalogf_class_initfn,
};

static void meta_dalogf_register(void)
{
    type_register_static(&meta_dalogf_info);
}

type_init(meta_dalogf_register)
