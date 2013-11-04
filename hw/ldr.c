#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ldr.h"

struct ldr_ctx_s {
    FILE * file;
    size_t page_size;
    size_t page_bits;
    ldr_hdr_t hdr;
};

typedef struct {
    uint32_t next;
    uint16_t len;
    uint16_t crc;
} ldr_slcode_ftr_t;

#define DEV_ID_INIT  0xffffffff
#define DEV_ID_VALID 0x01aa5500

typedef struct {
    uint16_t cmd;
    uint16_t len;
    uint32_t next;
} ldr_l1_hdr_t;

typedef struct {
    uint32_t l2off;
    uint16_t l2len;
    uint16_t crc;
} ldr_l1_ftr_t;

#define L1_NEXT_END ((uint32_t)0xffffffff)

typedef struct {
    uint16_t tag;
    uint16_t len;
} ldr_l2_hdr_t;

typedef struct {
    uint16_t crc;
} ldr_l2_ftr_t;

#define TAG_L2BLOCK      (1 << 15)
#define TAG_COMMENT      (1 << 4)
#define TAG_L1CMD_MASK   0xf

#define CMD_LOADMEM      0
#define CMD_LOADCORE     1
#define CMD_LOADMMREG    2
#define CMD_STARTTHREADS 3
#define CMD_ZEROMEM      4
#define CMD_CONFIG       5

#define CFG_PAUSE        0
#define CFG_READ         1
#define CFG_WRITE        2
#define CFG_MEMSET       3
#define CFG_MEMCHK       4

static const uint16_t x25_tab[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

static bool read_ldr(ldr_ctx_t *ctx, void *buf, off_t off, size_t sz)
{
    off_t linear_off;
   
    if (ctx->page_size) {
        linear_off = ((off / (1 << ctx->page_bits)) * ctx->page_size)
                   + (off % (1 << ctx->page_bits));
    } else {
        linear_off = off;
    }

    if (fseek(ctx->file, linear_off, SEEK_SET))
        return false;

    if (fread(buf, sz, 1, ctx->file) != 1)
        return false;

    return true;
}

static uint16_t x25calc(void *data, size_t sz, uint16_t crc)
{
    uint16_t tmp;
    uint8_t val, *ptr = (uint8_t *)data;

    while (sz > 0) {
        val = *ptr++;
        tmp = crc;

        tmp >>= 12;
        tmp ^= val >> 4;
        tmp &= 0xf;
        tmp = crc = (uint16_t)((crc << 4) ^ x25_tab[tmp]);

        tmp >>= 12;
        tmp ^= val;
        tmp &= 0xf;
        crc = (uint16_t)((crc << 4) ^ x25_tab[tmp]);

        sz--;
    }

    return crc;
}

#define eswap16(x) (((x & 0xff) << 8) | ((x & 0xff00) >> 8))

static uint16_t crc(void *data, size_t sz, uint16_t init)
{
    uint16_t tmp = ~eswap16(init);
    tmp = ~x25calc(data, sz, tmp);
    return eswap16(tmp);
}

ldr_ctx_t *ldr_open(const char *filename, size_t page_size)
{
    ldr_ctx_t *ctx;

    if (!(ctx = malloc(sizeof(*ctx))))
        goto err;

    if (!(ctx->file = fopen(filename, "rb")))
        goto err;

    ctx->page_size = page_size;

    if (ctx->page_size) {
        ctx->page_bits = 1;
        while ((1 << ctx->page_bits) < ctx->page_size)
            ctx->page_bits++;
    }

    ctx->hdr.dev_id = DEV_ID_INIT;

    return ctx;

err:
    if (ctx) free(ctx);
    return NULL;
}

void ldr_close(ldr_ctx_t *ctx)
{
    fclose(ctx->file);
    free(ctx);
}

bool ldr_parse_header(ldr_ctx_t *ctx, ldr_cb_t *cb, void *user)
{
    uint16_t sum;

    if (ctx->hdr.dev_id == DEV_ID_INIT && !read_ldr(ctx, &ctx->hdr, 0, sizeof(ctx->hdr)))
        goto err_hdr;
    if (ctx->hdr.dev_id != DEV_ID_VALID)
        goto err_hdr;

    sum = crc(&ctx->hdr, sizeof(ctx->hdr) - 2, 0);
    if (sum != ctx->hdr.crc) {
        if (cb->on_error) cb->on_error(ctx, ECRC, user);
        return false;
    }

    if (cb->on_header) cb->on_header(ctx, &ctx->hdr, user);

    return true;

err_hdr:
    if (cb->on_error) cb->on_error(ctx, EHEADER, user);
    return false;
}

bool ldr_parse_slcode(ldr_ctx_t *ctx, ldr_cb_t *cb, void *user)
{
    off_t off;
    uint8_t buf[120];
    uint16_t sum;
    ldr_slcode_ftr_t footer;

    if (ctx->hdr.dev_id == DEV_ID_INIT && !ldr_parse_header(ctx, cb, user))
        return false;
    if (ctx->hdr.dev_id != DEV_ID_VALID) {
        if (cb->on_error) cb->on_error(ctx, EHEADER, user);
        return false;
    }

    if (ctx->hdr.sl_code & SLCSECURE) {
        if (cb->on_error) cb->on_error(ctx, EUNSUPPORTED, user);
        return false;
    }

    off = ctx->hdr.sl_code & ((1 << 29) - 1);

    while (off) {
        if (!read_ldr(ctx, buf, off, sizeof(buf)))
            return false;
        if (!read_ldr(ctx, &footer, off + sizeof(buf), sizeof(footer)))
            return false;

        sum = crc(buf, sizeof(buf), 0);
        sum = crc(&footer, sizeof(footer) - 2, sum);
        if (sum != footer.crc) {
            if (cb->on_error) cb->on_error(ctx, ECRC, user);
            return false;
        }

        if (cb->on_slcode) {
            if (!cb->on_slcode(ctx, buf, sizeof(buf), user))
                return false;
        }

        off = footer.next;
    }

    return true;
}

static bool parse_unknown(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    if (cb->on_error) cb->on_error(ctx, EUNKNOWN, user);
    return false;
}

static bool parse_loadmem(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    uint32_t addr;

    if (l1len != 1)
        return false;

    addr = l1buf[0];

    if (cb->on_loadmem) {
        if (!cb->on_loadmem(ctx, addr, l2buf, l2len, user))
            return false;
    }

    return true;
}

static bool parse_loadcore(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    ldr_reg_t *regs = (ldr_reg_t *)l2buf;

    if (cb->on_loadcore) {
        if (!cb->on_loadcore(ctx, regs, l2len / sizeof(ldr_reg_t), user))
            return false;
    }

    return true;
}

static bool parse_loadmmreg(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    ldr_reg_t *regs = (ldr_reg_t *)l2buf;

    if (cb->on_loadmmreg) {
        if (!cb->on_loadmmreg(ctx, regs, l2len / sizeof(ldr_reg_t), user))
            return false;
    }

    return true;
}

static bool parse_startthreads(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    ldr_startthread_t *threads = (ldr_startthread_t*)l2buf;

    if (cb->on_startthreads) {
        if (!cb->on_startthreads(ctx, threads, l2len / sizeof(ldr_startthread_t), user))
            return false;
    }

    return true;
}

static bool parse_zeromem(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    uint32_t addr, sz;

    if (l1len != 2)
        return false;

    addr = l1buf[0];
    sz = l1buf[1];

    if (cb->on_zeromem) {
        if (!cb->on_zeromem(ctx, addr, sz, user))
            return false;
    }

    return true;
}

static bool parse_config(ldr_ctx_t *ctx, ldr_cb_t *cb, uint32_t *l1buf, size_t l1len, uint8_t *l2buf, size_t l2len, void *user)
{
    size_t off = 0;
    uint32_t *l2buf32 = (uint32_t*)l2buf;
    uint32_t cmd, count, addr, val, inc;

    if (cb->on_config) {
        return cb->on_config(ctx, l2buf, l2len, user);
    }

    while (off < (l2len >> 2)) {
        cmd = l2buf32[off++];

        switch (cmd) {
        case CFG_PAUSE:
            count = l2buf32[off++];
            if (cb->on_pause) {
                if (!cb->on_pause(ctx, count, user))
                    return false;
            }
            break;
        case CFG_READ:
            addr = l2buf32[off++];
            if (cb->on_read) {
                if (!cb->on_read(ctx, addr, user))
                    return false;
            }
            break;
        case CFG_WRITE:
            addr = l2buf32[off++];
            val = l2buf32[off++];
            if (cb->on_write) {
                if (!cb->on_write(ctx, addr, val, user))
                    return false;
            }
            break;
        case CFG_MEMSET:
            addr = l2buf32[off++];
            count = l2buf32[off++];
            val = l2buf32[off++];
            inc = l2buf32[off++];
            if (cb->on_memset) {
                if (!cb->on_memset(ctx, addr, count, val, inc, user))
                    return false;
            }
            break;
        case CFG_MEMCHK:
            addr = l2buf32[off++];
            count = l2buf32[off++];
            val = l2buf32[off++];
            inc = l2buf32[off++];
            if (cb->on_memchk) {
                if (!cb->on_memchk(ctx, addr, count, val, inc, user))
                    return false;
            }
            break;
        default:
            return false;
        }
    }

    return true;
}

bool ldr_parse_sldata(ldr_ctx_t *ctx, ldr_cb_t *cb, void *user)
{
    off_t off;
    ldr_l1_hdr_t l1_hdr;
    ldr_l1_ftr_t l1_ftr;
    ldr_l2_hdr_t l2_hdr;
    ldr_l2_ftr_t l2_ftr;
    size_t l1buf_len, l2buf_len;
    uint8_t *l1buf, *l2buf;
    uint16_t sum;
    bool (*fn_parse)(ldr_ctx_t*, ldr_cb_t*, uint32_t*, size_t, uint8_t*, size_t, void*);

    if (ctx->hdr.dev_id == DEV_ID_INIT && !ldr_parse_header(ctx, cb, user))
        return false;
    if (ctx->hdr.dev_id != DEV_ID_VALID) {
        if (cb->on_error) cb->on_error(ctx, EHEADER, user);
        return false;
    }

    off = ctx->hdr.sl_data;

    while (off != L1_NEXT_END) {
        if (!read_ldr(ctx, &l1_hdr, off, sizeof(l1_hdr))) {
            if (cb->on_error) cb->on_error(ctx, EREADLDR, user);
            return false;
        }

        l1buf_len = l1_hdr.len - (sizeof(l1_hdr) + sizeof(l1_ftr));
        if (l1buf_len) {
            if (!(l1buf = malloc(l1buf_len))) {
                if (cb->on_error) cb->on_error(ctx, EUNKNOWN, user);
                return false;
            }

            if (!read_ldr(ctx, l1buf, off + sizeof(l1_hdr), l1buf_len)) {
                free(l1buf);
                if (cb->on_error) cb->on_error(ctx, EREADLDR, user);
                return false;
            }
        } else
            l1buf = NULL;

        if (!read_ldr(ctx, &l1_ftr, off + sizeof(l1_hdr) + l1buf_len, sizeof(l1_ftr))) {
            if (l1buf) free(l1buf);
            if (cb->on_error) cb->on_error(ctx, EREADLDR, user);
            return false;
        }

        sum = crc(&l1_hdr, sizeof(l1_hdr), 0);
        if (l1buf) sum = crc(l1buf, l1buf_len, sum);
        sum = crc(&l1_ftr, sizeof(l1_ftr) - 2, sum);
        if (sum != l1_ftr.crc) {
            if (l1buf) free(l1buf);
            if (cb->on_error) cb->on_error(ctx, ECRC, user);
            return false;
        }

        if (!read_ldr(ctx, &l2_hdr, l1_ftr.l2off, sizeof(l2_hdr))) {
            if (l1buf) free(l1buf);
            if (cb->on_error) cb->on_error(ctx, EREADLDR, user);
            return false;
        }

        l2buf_len = l1_ftr.l2len - (sizeof(l2_hdr) + sizeof(l2_ftr));
        if (l2buf_len) {
            if (!(l2buf = malloc(l2buf_len))) {
                if (l1buf) free(l1buf);
                if (cb->on_error) cb->on_error(ctx, EUNKNOWN, user);
                return false;
            }

            if (!read_ldr(ctx, l2buf, l1_ftr.l2off + sizeof(l2_hdr), l2buf_len)) {
                if (l1buf) free(l1buf);
                free(l2buf);
                if (cb->on_error) cb->on_error(ctx, EREADLDR, user);
                return false;
            }
        } else
            l2buf = NULL;

        if (!read_ldr(ctx, &l2_ftr, l1_ftr.l2off + sizeof(l2_hdr) + l2buf_len, sizeof(l2_ftr))) {
            if (l1buf) free(l1buf);
            if (l2buf) free(l2buf);
            if (cb->on_error) cb->on_error(ctx, EREADLDR, user);
            return false;
        }

        sum = crc(&l2_hdr, sizeof(l2_hdr), 0);
        if (l2buf) sum = crc(l2buf, l2buf_len, sum);
        sum = crc(&l2_ftr, sizeof(l2_ftr) - 2, sum);
        if (sum != l2_ftr.crc) {
            if (l1buf) free(l1buf);
            if (l2buf) free(l2buf);
            if (cb->on_error) cb->on_error(ctx, ECRC, user);
            return false;
        }

        switch (l1_hdr.cmd & TAG_L1CMD_MASK) {
        case CMD_LOADMEM: fn_parse = parse_loadmem; break;
        case CMD_LOADCORE: fn_parse = parse_loadcore; break;
        case CMD_LOADMMREG: fn_parse = parse_loadmmreg; break;
        case CMD_STARTTHREADS: fn_parse = parse_startthreads; break;
        case CMD_ZEROMEM: fn_parse = parse_zeromem; break;
        case CMD_CONFIG: fn_parse = parse_config; break;
        default: fn_parse = parse_unknown; break;
        }

        if (l2_hdr.tag & TAG_COMMENT) {
            /* ignore comments */
        } else {
            if (!fn_parse(ctx, cb, (uint32_t*)l1buf, l1buf_len >> 2, l2buf, l2buf_len, user)) {
                if (l1buf) free(l1buf);
                if (l2buf) free(l2buf);
                return false;
            }
        }

        if (l1buf) free(l1buf);
        if (l2buf) free(l2buf);
        off = l1_hdr.next;
    }

    return true;
}

