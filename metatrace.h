#ifndef METATRACE_H
#define METATRACE_H

#include <stdint.h>
#include <stdio.h>

/* internal interface */
extern int metatrace_start(const char *file);
extern void metatrace_stop(void);
extern void metatrace_set_events(uint32_t mask);
extern void metatrace_set_limit(unsigned int limit);

#define METATRACE_TT       (1 << 0)  /* any trace unit event */
#define METATRACE_DATA     (1 << 1)  /* data loads and stores */
#define METATRACE_IFETCH   (1 << 2)  /* instruction fetches */
#define METATRACE_FLUSH    (1 << 3)  /* cache flushes */
#define METATRACE_MMSWITCH (1 << 4)  /* virtual memory switch */
#define METATRACE_PREFETCH (1 << 5)  /* data/code software prefetch */

/*
 * parse trace events list and output in *add and *rem
 * returns 0 on success
 */
int metatrace_parse_events(const char *events, uint32_t *add, uint32_t *rem);

/*
 * print usage information to a file
 */
typedef int (*metatrace_usage_printf)(void *, const char *, ...);
void metatrace_usage(metatrace_usage_printf pf, void *opaque);

/*
 * set the trace output file
 * returns 0 on success
 */
int metatrace_setfile(const char *file);

/* get the event limit (0 indicates no limit) */
unsigned int metatrace_getlimit(void);

/* set the event limit (0 indicates no limit) */
void metatrace_setlimit(unsigned int limit);

/* set the event mask */
void metatrace_setmask(uint32_t events);

/* get the current event mask */
uint32_t metatrace_getmask(void);

/* test the current event mask for certain bits */
static inline uint32_t metatrace_mask(uint32_t mask)
{
    return metatrace_getmask() & mask;
}

/* update the current event mask */
static inline void metatrace_updmask(uint32_t add, uint32_t rem)
{
    metatrace_setmask((metatrace_getmask() & ~rem) | add);
}

#endif
