#ifndef __TRACE_FORMAT_H__
#define __TRACE_FORMAT_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct trace_ctx_s trace_ctx_t;

#define TRACE_MAGIC_FILE      0xcac4
#define TRACE_MAGIC_BLOCK     0xb10c
#define TRACE_FORMAT_VERSION  1

typedef struct {
    uint32_t paddr;
    uint32_t ptaddr;
    uint8_t pgshift;
    uint8_t mmuctrl;
    uint8_t cachectrl;
} trace_mmu_t;

typedef struct {
    uint32_t pc;
    uint32_t actcyc;
} trace_state_t;

trace_ctx_t *trace_new(const char *filename);
trace_ctx_t *trace_open(const char *filename);
void trace_close(trace_ctx_t *ctx);
void trace_reset(trace_ctx_t *ctx);

void trace_state(trace_ctx_t *ctx, trace_state_t *ts);
void trace_state_pc(trace_ctx_t *ctx, uint32_t pc);
void trace_state_actcyc(trace_ctx_t *ctx, uint32_t actcyc);
void trace_mmu(trace_ctx_t *ctx, trace_mmu_t *mmu);
void trace_mmu_all(trace_ctx_t *ctx, uint32_t paddr, uint32_t ptaddr, uint8_t pgshift, uint8_t mmuctrl, uint8_t cachectrl);

void trace_event_dload(trace_ctx_t *ctx, uint32_t vaddr, uint8_t szlog2);
void trace_event_dstore(trace_ctx_t *ctx, uint32_t vaddr, uint8_t szlog2);
void trace_event_dflush(trace_ctx_t *ctx, uint32_t vaddr, uint32_t sz);
void trace_event_dflushline(trace_ctx_t *ctx, uint32_t line, uint32_t sz);
void trace_event_ifetch(trace_ctx_t *ctx, uint32_t pc);
void trace_event_iflush(trace_ctx_t *ctx, uint32_t vaddr, uint32_t sz);
void trace_event_iflushline(trace_ctx_t *ctx, uint32_t line, uint32_t sz);
void trace_event_ttevent(trace_ctx_t *ctx, uint16_t header, int n, uint32_t d1, uint32_t d2, uint32_t d3, uint32_t d4);
void trace_event_mmswitch(trace_ctx_t *ctx, uint8_t thread, uint8_t global, uint32_t pgdbase);
void trace_event_dprefetch(trace_ctx_t *ctx, uint32_t vaddr, uint32_t size, bool priority);
void trace_event_dprefetchexit(trace_ctx_t *ctx);
void trace_event_iprefetch(trace_ctx_t *ctx, uint32_t vaddr, uint32_t size, bool priority);
void trace_event_iprefetchexit(trace_ctx_t *ctx);

typedef struct {
    bool (*on_event_dload)     (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint8_t szlog2, trace_mmu_t *mmu);
    bool (*on_event_dstore)    (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint8_t szlog2, trace_mmu_t *mmu);
    bool (*on_event_dflush)    (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint32_t sz, trace_mmu_t *mmu);
    bool (*on_event_dflushline)(trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t line, uint32_t sz);
    bool (*on_event_ifetch)    (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t pc, trace_mmu_t *mmu);
    bool (*on_event_iflush)    (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint32_t sz, trace_mmu_t *mmu);
    bool (*on_event_iflushline)(trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t line, uint32_t sz);
    bool (*on_event_ttevent)   (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint16_t header, int n, uint32_t *data);
    bool (*on_event_mmswitch)  (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint8_t thread, uint8_t global, uint32_t pgdbase);
    bool (*on_event_dprefetch)     (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint32_t size, bool priority, trace_mmu_t *mmu);
    bool (*on_event_dprefetchexit) (trace_ctx_t *ctx, void *user, trace_state_t *ts);
    bool (*on_event_iprefetch)     (trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint32_t size, bool priority, trace_mmu_t *mmu);
    bool (*on_event_iprefetchexit) (trace_ctx_t *ctx, void *user, trace_state_t *ts);
} trace_cb_t;

bool trace_read_event(trace_ctx_t *ctx, trace_cb_t *cb, void *user);

#endif /* __TRACE_FORMAT_H__ */
