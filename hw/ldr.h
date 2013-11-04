#ifndef __LDR_H__
#define __LDR_H__

#include <stdbool.h>
#include <stdint.h>

#define EHEADER      1
#define ECRC         2
#define EUNSUPPORTED 3
#define EUNKNOWN     4
#define EREADLDR     5

typedef struct ldr_ctx_s ldr_ctx_t;

typedef struct {
    uint32_t dev_id;
    uint32_t sl_code;
    uint32_t sl_data;
    uint16_t pl_ctrl;
    uint16_t crc;
} ldr_hdr_t;

#define SLCSECURE      (1 << 31)
#define SLCCRITICAL    (1 << 30)
#define SLCOFFSET_MASK ((1 << 30) - 1)

typedef struct {
    uint32_t reg_desc;
    uint32_t wr_mask;
    uint32_t val;
} ldr_reg_t;

typedef struct {
    uint32_t thread;
    uint32_t sp;
    uint32_t pc;
    uint32_t catch_state;
} ldr_startthread_t;

typedef struct {
    bool (*on_error)(ldr_ctx_t *ctx, int err, void *user);
    bool (*on_header)(ldr_ctx_t *ctx, ldr_hdr_t *hdr, void *user);
    bool (*on_slcode)(ldr_ctx_t *ctx, uint8_t *data, size_t sz, void *user);
    bool (*on_loadmem)(ldr_ctx_t *ctx, uint32_t addr, uint8_t *data, size_t sz, void *user);
    bool (*on_loadcore)(ldr_ctx_t *ctx, ldr_reg_t *regs, size_t num, void *user);
    bool (*on_loadmmreg)(ldr_ctx_t *ctx, ldr_reg_t *regs, size_t num, void *user);
    bool (*on_startthreads)(ldr_ctx_t *ctx, ldr_startthread_t *threads, size_t num, void *user);
    bool (*on_zeromem)(ldr_ctx_t *ctx, uint32_t addr, uint32_t sz, void *user);
    bool (*on_config)(ldr_ctx_t *ctx, uint8_t *data, uint32_t sz, void *user);
    bool (*on_pause)(ldr_ctx_t *ctx, uint32_t count, void *user);
    bool (*on_read)(ldr_ctx_t *ctx, uint32_t addr, void *user);
    bool (*on_write)(ldr_ctx_t *ctx, uint32_t addr, uint32_t val, void *user);
    bool (*on_memset)(ldr_ctx_t *ctx, uint32_t addr, uint32_t count, uint32_t val, uint32_t inc, void *user);
    bool (*on_memchk)(ldr_ctx_t *ctx, uint32_t addr, uint32_t count, uint32_t val, uint32_t inc, void *user);
} ldr_cb_t;

ldr_ctx_t *ldr_open(const char *filename, size_t page_size);
void ldr_close(ldr_ctx_t *ctx);
bool ldr_parse_header(ldr_ctx_t *ctx, ldr_cb_t *cb, void *user);
bool ldr_parse_slcode(ldr_ctx_t *ctx, ldr_cb_t *cb, void *user);
bool ldr_parse_sldata(ldr_ctx_t *ctx, ldr_cb_t *cb, void *user);

#endif /* __LDR_H__ */

