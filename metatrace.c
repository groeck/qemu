#include "metatrace.h"
#include "config.h"

#include <string.h>

#if !defined(TARGET_META) || defined(CONFIG_USER_ONLY)

int metatrace_parse_events(const char *events, uint32_t *add, uint32_t *rem)
{
    *add = 0;
    if (rem) {
        *rem = 0;
    }
    return 0;
}

void metatrace_usage(metatrace_usage_printf pf, void *opaque)
{
}

int metatrace_setfile(const char *file)
{
    return 0;
}

unsigned int metatrace_getlimit(void)
{
    return 0;
}

void metatrace_setlimit(unsigned int limit)
{
}

void metatrace_setmask(uint32_t events)
{
}

uint32_t metatrace_getmask(void)
{
    return 0;
}

#else /* TARGET_META */

static const char *metatrace_file;
static uint32_t metatrace_event_mask;
static unsigned int metatrace_limit;

static struct metatrace_info {
    uint32_t mask;
    const char *name;
    const char *desc;
} metatrace_events[] = {
    { METATRACE_TT,       "tt",       "Trace unit events" },
    { METATRACE_DATA,     "data",     "Data loads and stores" },
    { METATRACE_IFETCH,   "ifetch",   "Instruction fetches" },
    { METATRACE_FLUSH,    "flush",    "Cache flushes" },
    { METATRACE_MMSWITCH, "mmswitch", "Virtual memory switch" },
    { METATRACE_PREFETCH, "prefetch", "Software prefetch" },
    { ~0u,                "all",      "All trace events" },
    {},
};

int metatrace_parse_events(const char *events, uint32_t *add, uint32_t *rem)
{
    struct metatrace_info *info;
    int neg, len;
    const char *sep;

    while (events) {
        /* events starting with '-' are added to removal mask (*rem) */
        neg = 0;
        if (*events == '-') {
            ++events;
            neg = 1;
        }
        /* find the next separator */
        sep = strchr(events, ',');
        if (sep) {
            len = sep - events;
            ++sep;
        } else {
            len = 255;
        }
        /* find a matching event type */
        for (info = metatrace_events; info->name; ++info) {
            if (!strncmp(info->name, events, len)) {
                if (!neg) {
                    *add |= info->mask;
                } else if (rem) {
                    *rem |= info->mask;
                } else {
                    /* removals aren't allowed unless rem != NULL */
                    return 1;
                }
                goto next_event;
            }
        }
        /* invalid event name */
        return 1;
next_event:
        events = sep;
    }
    return 0;
}

void metatrace_usage(metatrace_usage_printf pf, void *opaque)
{
    struct metatrace_info *info;

    pf(opaque, "available metatrace events:\n");
    for (info = metatrace_events; info->name; ++info) {
        pf(opaque, "%-8s %s\n", info->name, info->desc);
    }
}

int metatrace_setfile(const char *file)
{
    if (metatrace_file) {
        /* close file */
        metatrace_stop();
    }
    metatrace_file = file;
    if (metatrace_file) {
        /* open file */
        if (metatrace_start(metatrace_file)) {
            metatrace_file = NULL;
            return 1;
        }
    }
    return 0;
}

unsigned int metatrace_getlimit(void)
{
    return metatrace_limit;
}

void metatrace_setlimit(unsigned int limit)
{
    unsigned int old = metatrace_limit;
    metatrace_limit = limit;
    if (old != limit) {
        metatrace_set_limit(limit);
    }
}

void metatrace_setmask(uint32_t events)
{
    uint32_t diff;
    diff = metatrace_event_mask ^ events;
    metatrace_event_mask = events;
    if (diff) {
        metatrace_set_events(events);
    }
}

uint32_t metatrace_getmask(void)
{
    return metatrace_event_mask;
}

#endif
