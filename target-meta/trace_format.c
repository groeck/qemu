#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "trace_format.h"

typedef enum {
    /* main event types */
    TRACE_EVENT_DLOAD,
    TRACE_EVENT_DSTORE,
    TRACE_EVENT_IFETCH,
    TRACE_EVENT_OTHER,

    /* 'other' event types */
    TRACE_EVENT_CACHEFLUSH = TRACE_EVENT_OTHER + (0x0 << 2),
    TRACE_EVENT_TTEVENT = TRACE_EVENT_OTHER + (0x1 << 2),
    TRACE_EVENT_MMSWITCH = TRACE_EVENT_OTHER + (0x2 << 2),
    TRACE_EVENT_PREFETCH = TRACE_EVENT_OTHER + (0x3 << 2),
    TRACE_EVENT_PREFETCHEXIT = TRACE_EVENT_OTHER + (0x4 << 2),
    TRACE_EVENT_PADTOBYTE  = TRACE_EVENT_OTHER + (0xf << 2),
} trace_event_type_t;

typedef enum {
    TRACE_ENC_SREL,
    TRACE_ENC_UREL,
    TRACE_ENC_BOOL,
} trace_enc_type_t;

typedef enum {
    TRACE_DATA_VADDR,
    TRACE_DATA_PADDR,
    TRACE_DATA_PC,
    TRACE_DATA_SIZE,
    TRACE_DATA_MISC,
    TRACE_DATA_ACTCYC,
    TRACE_DATA_TTHEADER,
    TRACE_DATA_TTDATA,
    TRACE_DATA_PGDBASE,   /* physical address of 1st level page table */
    TRACE_DATA_PTADDR,    /* physical address of 2nd level page table */
    TRACE_DATA_PGSHIFT,   /* page shift - 12 */
    TRACE_DATA_MMUCTRL,   /* mmu control bits */
    TRACE_DATA_CACHECTRL, /* cache control bits */
    TRACE_DATA_THREAD,    /* thread number */
    TRACE_DATA_GLOBAL,    /* global as opposed to local */
    TRACE_DATA_MAX,
} trace_data_type_t;

typedef struct {
    uint8_t data_type;
    uint8_t enc;
    uint8_t shift;
    bool shift_by_width;
    char *name;
} trace_event_field_desc_t;

typedef struct {
    uint8_t type;
    uint8_t num_fields;
    uint8_t num_opt_fields;
    char *name;
    trace_event_field_desc_t **fields;
} trace_event_desc_t;

typedef struct {
    uint16_t magic;
    uint16_t version;
    uint8_t num_event_descs;
    trace_event_desc_t **event_descs;
} trace_file_hdr_t;

struct trace_ctx_s {
    FILE *file;
    trace_file_hdr_t hdr;
    size_t reset_pos;
    trace_event_desc_t *evd_dload, *evd_dstore;
    trace_event_desc_t *evd_ifetch;
    trace_event_desc_t *evd_cacheflush, *evd_padtobyte;
    trace_event_desc_t *evd_ttevent;
    trace_event_desc_t *evd_mmswitch;
    trace_event_desc_t *evd_prefetch, *evd_prefetchexit;

    uint8_t write_buf[32];
    size_t write_buf_idx;

    uint8_t read_buf;
    size_t read_buf_bits;

    uint32_t last_vals[TRACE_DATA_MAX];

    trace_state_t state;
    trace_mmu_t mmu;
};

static void trace_event_padtobyte(trace_ctx_t *ctx);

#define assert(x) do { \
    if (!(x)) { \
        fprintf(stderr, "%s:%d failed assert: %s\n", __FILE__, __LINE__, #x); \
    } \
} while (0)

#ifndef MIN
#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#endif

#define TRACE_MMU_FIELDS 5

/* add field descriptions for MMU mapping data */
static void trace_gen_desc_mmu_fields(trace_event_desc_t *desc,
                                      trace_event_field_desc_t **fields)
{
    trace_event_field_desc_t *field;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PADDR;
    field->enc = TRACE_ENC_SREL;
    field->shift = 12;
    field->shift_by_width = false;
    field->name = strdup("PAddr");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PTADDR;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PTAddr");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PGSHIFT;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("PgShift");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_MMUCTRL;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("MMUCtrl");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_CACHECTRL;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("CacheCtrl");
}

/* fill out fields for MMU mapping data */
static uint32_t *trace_event_mmu(trace_ctx_t *ctx, uint32_t *vals)
{
    *vals++ = ctx->mmu.paddr;
    *vals++ = ctx->mmu.ptaddr;
    *vals++ = ctx->mmu.pgshift;
    *vals++ = ctx->mmu.mmuctrl;
    *vals++ = ctx->mmu.cachectrl;
    return vals;
}

/* update mmu structure from fields */
static uint32_t *trace_mmu_fields(trace_ctx_t *ctx, uint32_t vaddr, uint32_t *vals)
{
    trace_mmu_all(ctx, vals[0] | (vaddr & 0xfff), vals[1], vals[2], vals[3], vals[4]);
    return vals + TRACE_MMU_FIELDS;
}

static void trace_flush_write_buf(trace_ctx_t *ctx)
{
    size_t sz = ctx->write_buf_idx >> 3;

    if (!sz)
        return;

    if (fwrite(ctx->write_buf, 1, sz, ctx->file) != sz) {
        fprintf(stderr, "Failed to write to trace file\n");
        return;
    }

    memset(ctx->write_buf, 0, sz);

    if (ctx->write_buf_idx % 8) {
        ctx->write_buf[0] = ctx->write_buf[sz];
        ctx->write_buf[sz] = 0;
    }

    ctx->write_buf_idx -= sz << 3;
}

static void trace_write(trace_ctx_t *ctx, uint32_t val, size_t bits)
{
    uint32_t remval = val;
    uint8_t writeval;
    size_t rembits = bits;
    size_t byteoff, writebits;

    while (rembits) {
        byteoff = ctx->write_buf_idx % 8;
        writebits = MIN(8 - byteoff, rembits);
        writeval = (remval & ((1 << writebits) - 1)) << byteoff;
        ctx->write_buf[ctx->write_buf_idx >> 3] |= writeval;
        ctx->write_buf_idx += writebits;
        rembits -= writebits;
        remval >>= writebits;

        if (ctx->write_buf_idx >= ((sizeof(ctx->write_buf) - 1) << 3))
            trace_flush_write_buf(ctx);
    }
}

static uint32_t trace_try_read(trace_ctx_t *ctx, size_t bits, bool *success)
{
    uint32_t val = 0, readval = 0, tmpval;
    size_t rembits = bits;
    size_t readbits, readbytes, donebits = 0;

    if (success)
        *success = true;

    while (rembits) {
        if (ctx->read_buf_bits) {
            readbits = MIN(ctx->read_buf_bits, rembits);
            readval = ctx->read_buf & ((1 << readbits) - 1);
            val |= readval << donebits;
            donebits += readbits;
            ctx->read_buf >>= readbits;
            ctx->read_buf_bits -= readbits;
            rembits -= readbits;
            continue;
        }

        readbytes = rembits >> 3;
        if (rembits % 8) readbytes++;
        assert(readbytes <= 4);
        readbits = readbytes << 3;
        if (fread(&readval, 1, readbytes, ctx->file) != readbytes) {
            if (success)
                *success = false;
            else
                fprintf(stderr, "Failed to read trace file\n");
            return 0;
        }
        tmpval = readval;
        if (rembits < 32)
            tmpval &= (1 << rembits) - 1;
        val |= tmpval << donebits;
        donebits += rembits;
        ctx->read_buf = readval >> rembits;
        ctx->read_buf_bits = readbits - rembits;
        rembits = 0;
    }

    return val;
}

static uint32_t trace_read(trace_ctx_t *ctx, size_t bits)
{
    return trace_try_read(ctx, bits, NULL);
}

static void trace_write_str(trace_ctx_t *ctx, const char *str)
{
    size_t i;

    trace_write(ctx, strlen(str), 8);
    for (i = 0; i < strlen(str); i++)
        trace_write(ctx, str[i], 8);
}

static char *trace_read_str(trace_ctx_t *ctx)
{
    size_t len, i;
    char *str;

    len = trace_read(ctx, 8);
    if (!(str = malloc(len + 1)))
        return NULL;

    for (i = 0; i < len; i++)
        str[i] = trace_read(ctx, 8);

    str[len] = 0x00;
    return str;
}

static size_t trace_pos(trace_ctx_t *ctx)
{
    trace_flush_write_buf(ctx);
    return ftell(ctx->file);
}

static trace_event_desc_t *trace_gen_desc_dload(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;

    desc->type = TRACE_EVENT_DLOAD;
    desc->name = strdup("Data Load");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_VADDR;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = true;
    field->name = strdup("VAddr");

    trace_gen_desc_mmu_fields(desc, fields);

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_dstore(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;

    desc->type = TRACE_EVENT_DSTORE;
    desc->name = strdup("Data Store");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_VADDR;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = true;
    field->name = strdup("VAddr");

    trace_gen_desc_mmu_fields(desc, fields);

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_ifetch(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;

    desc->type = TRACE_EVENT_IFETCH;
    desc->name = strdup("Instruction Fetch");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    trace_gen_desc_mmu_fields(desc, fields);

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_cacheflush(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;

    desc->type = TRACE_EVENT_CACHEFLUSH;
    desc->name = strdup("Cache Flush");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_MISC;
    field->enc = TRACE_ENC_BOOL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Cache");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_MISC;
    field->enc = TRACE_ENC_BOOL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Line");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PADDR;
    field->enc = TRACE_ENC_SREL;
    field->shift = 12;
    field->shift_by_width = false;
    field->name = strdup("Addr");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_SIZE;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Size");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    trace_gen_desc_mmu_fields(desc, fields);

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_ttevent(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;
    char buf[64];

    desc->type = TRACE_EVENT_TTEVENT;
    snprintf(buf, sizeof(buf), "TTEvent");
    desc->name = strdup(buf);
    desc->num_fields = 0;
    desc->num_opt_fields = 4;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_TTHEADER;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Header");

    for (i = 0; i < desc->num_opt_fields; ++i) {
        field = fields[desc->num_fields++] = malloc(sizeof(*field));
        field->data_type = TRACE_DATA_TTDATA;
        field->enc = TRACE_ENC_SREL;
        field->shift = 0;
        field->shift_by_width = false;
        snprintf(buf, sizeof(buf), "Data%zd", i);
        field->name = strdup(buf);
    }

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_mmswitch(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;
    char buf[64];

    desc->type = TRACE_EVENT_MMSWITCH;
    snprintf(buf, sizeof(buf), "MMSwitch");
    desc->name = strdup(buf);
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_THREAD;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Thread");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_GLOBAL;
    field->enc = TRACE_ENC_BOOL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Global");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PGDBASE;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PGDBase");

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_prefetch(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;

    desc->type = TRACE_EVENT_PREFETCH;
    desc->name = strdup("Prefetch");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_MISC;
    field->enc = TRACE_ENC_BOOL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Cache");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_MISC;
    field->enc = TRACE_ENC_BOOL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Priority");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_VADDR;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = true;
    field->name = strdup("VAddr");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_SIZE;
    field->enc = TRACE_ENC_SREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Size");

    trace_gen_desc_mmu_fields(desc, fields);

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_prefetchexit(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));
    trace_event_field_desc_t *field, *fields[32];
    size_t i;

    desc->type = TRACE_EVENT_PREFETCHEXIT;
    desc->name = strdup("Prefetch Exit");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_PC;
    field->enc = TRACE_ENC_SREL;
    field->shift = 2;
    field->shift_by_width = false;
    field->name = strdup("PC");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_ACTCYC;
    field->enc = TRACE_ENC_UREL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("ActCyc");

    field = fields[desc->num_fields++] = malloc(sizeof(*field));
    field->data_type = TRACE_DATA_MISC;
    field->enc = TRACE_ENC_BOOL;
    field->shift = 0;
    field->shift_by_width = false;
    field->name = strdup("Cache");

    desc->fields = malloc(sizeof(field) * desc->num_fields);
    for (i = 0; i < desc->num_fields; i++)
        desc->fields[i] = fields[i];

    return desc;
}

static trace_event_desc_t *trace_gen_desc_padtobyte(trace_ctx_t *ctx)
{
    trace_event_desc_t *desc = malloc(sizeof(*desc));

    desc->type = TRACE_EVENT_PADTOBYTE;
    desc->name = strdup("Padding");
    desc->num_fields = 0;
    desc->num_opt_fields = 0;
    desc->fields = NULL;

    return desc;
}

static void trace_free_event_field_desc(trace_ctx_t *ctx, trace_event_field_desc_t *fld)
{
    if (fld->name)
        free(fld->name);

    free(fld);
}

static void trace_write_event_field_desc(trace_ctx_t *ctx, trace_event_field_desc_t *fld)
{
    size_t off_sz, off_end;
    uint32_t sz;

    /* dummy size, corrected later */
    off_sz = trace_pos(ctx);
    trace_write(ctx, 0, 32);

    trace_write(ctx, fld->data_type, 8);
    trace_write(ctx, fld->enc, 8);
    trace_write(ctx, fld->shift, 7);
    trace_write(ctx, fld->shift_by_width, 1);
    trace_write_str(ctx, fld->name);

    /* correct the size */
    off_end = trace_pos(ctx);
    sz = off_end - off_sz;
    assert(!fseek(ctx->file, off_sz, SEEK_SET));
    assert(fwrite(&sz, sizeof(sz), 1, ctx->file) == 1);
    assert(!fseek(ctx->file, off_end, SEEK_SET));
}

static trace_event_field_desc_t *trace_read_event_field_desc(trace_ctx_t *ctx)
{
    trace_event_field_desc_t *fld;
    size_t off_end;

    if (!(fld = calloc(sizeof(*fld), 1)))
        goto err;

    off_end = trace_pos(ctx) + trace_read(ctx, 32);

    fld->data_type = trace_read(ctx, 8);
    fld->enc = trace_read(ctx, 8);
    fld->shift = trace_read(ctx, 7);
    fld->shift_by_width = trace_read(ctx, 1);
    fld->name = trace_read_str(ctx);

    assert(!fseek(ctx->file, off_end, SEEK_SET));
    return fld;

err:
    if (fld) trace_free_event_field_desc(ctx, fld);
    return NULL;
}

static void trace_free_event_desc(trace_ctx_t *ctx, trace_event_desc_t *evd)
{
    size_t i;

    if (evd->fields) {
        for (i = 0; i < evd->num_fields; i++) {
            if (evd->fields[i])
                trace_free_event_field_desc(ctx, evd->fields[i]);
        }
        free(evd->fields);
    }

    if (evd->name)
        free(evd->name);

    free(evd);
}

static void trace_write_event_desc(trace_ctx_t *ctx, trace_event_desc_t *evd)
{
    size_t i, off_sz, off_end;
    uint32_t sz;

    /* dummy size, corrected later */
    off_sz = trace_pos(ctx);
    trace_write(ctx, 0, 32);

    trace_write(ctx, evd->type & 0x3, 8);
    switch (evd->type & 0x3) {
    case TRACE_EVENT_DLOAD:
    case TRACE_EVENT_DSTORE:
    case TRACE_EVENT_IFETCH:
        break;
    case TRACE_EVENT_OTHER:
        trace_write(ctx, evd->type >> 2, 8);
        break;
    }

    trace_write_str(ctx, evd->name);

    trace_write(ctx, evd->num_fields, 8);
    trace_write(ctx, evd->num_opt_fields, 8);
    for (i = 0; i < evd->num_fields; i++)
        trace_write_event_field_desc(ctx, evd->fields[i]);

    /* correct the size */
    off_end = trace_pos(ctx);
    sz = off_end - off_sz;
    assert(!fseek(ctx->file, off_sz, SEEK_SET));
    assert(fwrite(&sz, sizeof(sz), 1, ctx->file) == 1);
    assert(!fseek(ctx->file, off_end, SEEK_SET));
}

static trace_event_desc_t *trace_read_event_desc(trace_ctx_t *ctx)
{
    trace_event_desc_t *evd;
    size_t i, off_end;

    if (!(evd = calloc(sizeof(*evd), 1)))
        goto err;

    off_end = trace_pos(ctx) + trace_read(ctx, 32);

    evd->type = trace_read(ctx, 8);
    switch (evd->type) {
    case TRACE_EVENT_DLOAD:
    case TRACE_EVENT_DSTORE:
    case TRACE_EVENT_IFETCH:
        break;
    case TRACE_EVENT_OTHER:
        evd->type |= trace_read(ctx, 8) << 2;
        break;
    default:
        fprintf(stderr, "Unknown event type %d\n", evd->type);
        goto err;
    }

    evd->name = trace_read_str(ctx);

    evd->num_fields = trace_read(ctx, 8);
    evd->num_opt_fields = trace_read(ctx, 8);
    if (evd->num_opt_fields > evd->num_fields)
        goto err;
    if (!(evd->fields = calloc(sizeof(trace_event_field_desc_t *), evd->num_fields)))
        goto err;
    for (i = 0; i < evd->num_fields; i++) {
        if (!(evd->fields[i] = trace_read_event_field_desc(ctx)))
            goto err;
    }

    assert(!fseek(ctx->file, off_end, SEEK_SET));
    return evd;

err:
    if (evd) trace_free_event_desc(ctx, evd);
    return NULL;
}

static void trace_write_file_header(trace_ctx_t *ctx)
{
    size_t i, off_sz, off_end;
    uint32_t sz;

    trace_write(ctx, ctx->hdr.magic, 16);
    trace_write(ctx, ctx->hdr.version, 16);

    /* dummy header size, corrected later */
    off_sz = trace_pos(ctx);
    trace_write(ctx, 0, 32);

    trace_write(ctx, ctx->hdr.num_event_descs, 8);
    for (i = 0; i < ctx->hdr.num_event_descs; i++)
        trace_write_event_desc(ctx, ctx->hdr.event_descs[i]);

    /* correct the header size */
    off_end = trace_pos(ctx);
    sz = off_end - off_sz;
    assert(!fseek(ctx->file, off_sz, SEEK_SET));
    assert(fwrite(&sz, sizeof(sz), 1, ctx->file) == 1);
    assert(!fseek(ctx->file, off_end, SEEK_SET));
}

static bool trace_read_file_header(trace_ctx_t *ctx)
{
    size_t off_end, i;

    ctx->hdr.magic = trace_read(ctx, 16);
    if (ctx->hdr.magic != TRACE_MAGIC_FILE) {
        fprintf(stderr, "Invalid trace file magic 0x%04x\n", ctx->hdr.magic);
        return false;
    }

    ctx->hdr.version = trace_read(ctx, 16);
    if (ctx->hdr.version > TRACE_FORMAT_VERSION)
        fprintf(stderr, "Warning: trace file of unknown version %d\n", ctx->hdr.version);

    off_end = trace_pos(ctx) + trace_read(ctx, 32);

    ctx->hdr.num_event_descs = trace_read(ctx, 8);
    if (!(ctx->hdr.event_descs = calloc(sizeof(trace_event_desc_t *), ctx->hdr.num_event_descs)))
        return false;
    for (i = 0; i < ctx->hdr.num_event_descs; i++) {
        if (!(ctx->hdr.event_descs[i] = trace_read_event_desc(ctx)))
            return false;
    }

    assert(!fseek(ctx->file, off_end, SEEK_SET));

    return true;
}

trace_ctx_t *trace_new(const char *filename)
{
    trace_ctx_t *ctx;

    if (!(ctx = malloc(sizeof(*ctx))))
        goto err0;

    ctx->write_buf_idx = ctx->read_buf_bits = 0;
    memset(ctx->write_buf, 0, sizeof(ctx->write_buf));
    memset(ctx->last_vals, 0, sizeof(ctx->last_vals));

    ctx->hdr.magic = TRACE_MAGIC_FILE;
    ctx->hdr.version = TRACE_FORMAT_VERSION;
    ctx->hdr.num_event_descs = 0;

    if (!(ctx->file = fopen(filename, "wb")))
        goto err1;

    ctx->hdr.num_event_descs = 9;
    ctx->hdr.event_descs = malloc(sizeof(trace_event_desc_t *) * ctx->hdr.num_event_descs);
    ctx->hdr.event_descs[0] = ctx->evd_dload = trace_gen_desc_dload(ctx);
    ctx->hdr.event_descs[1] = ctx->evd_dstore = trace_gen_desc_dstore(ctx);
    ctx->hdr.event_descs[2] = ctx->evd_ifetch = trace_gen_desc_ifetch(ctx);
    ctx->hdr.event_descs[3] = ctx->evd_cacheflush = trace_gen_desc_cacheflush(ctx);
    ctx->hdr.event_descs[4] = ctx->evd_ttevent = trace_gen_desc_ttevent(ctx);
    ctx->hdr.event_descs[5] = ctx->evd_mmswitch = trace_gen_desc_mmswitch(ctx);
    ctx->hdr.event_descs[6] = ctx->evd_prefetch = trace_gen_desc_prefetch(ctx);
    ctx->hdr.event_descs[7] = ctx->evd_prefetchexit = trace_gen_desc_prefetchexit(ctx);
    ctx->hdr.event_descs[8] = ctx->evd_padtobyte = trace_gen_desc_padtobyte(ctx);

    trace_write_file_header(ctx);

    return ctx;

err1:
    free(ctx);
err0:
    return NULL;
}

trace_ctx_t *trace_open(const char *filename)
{
    trace_ctx_t *ctx;

    if (!(ctx = malloc(sizeof(*ctx))))
        goto err;

    ctx->write_buf_idx = ctx->read_buf_bits = 0;
    ctx->hdr.event_descs = NULL;
    memset(ctx->last_vals, 0, sizeof(ctx->last_vals));

    if (!(ctx->file = fopen(filename, "rb")))
        goto err;

    if (!trace_read_file_header(ctx))
        goto err;

    ctx->reset_pos = trace_pos(ctx);

    return ctx;

err:
    if (ctx)
        trace_close(ctx);
    return NULL;
}

void trace_close(trace_ctx_t *ctx)
{
    size_t i;

    /* ensure any remaning data is written to the file */
    if (ctx->write_buf_idx % 8)
        trace_event_padtobyte(ctx);
    trace_flush_write_buf(ctx);
    assert(!ctx->write_buf_idx);

    if (ctx->hdr.event_descs) {
        for (i = 0; i < ctx->hdr.num_event_descs; i++) {
            if (ctx->hdr.event_descs[i]) {
                trace_free_event_desc(ctx, ctx->hdr.event_descs[i]);
            }
        }
        free(ctx->hdr.event_descs);
    }

    if (ctx->file) {
        fclose(ctx->file);
    }
    free(ctx);
}

void trace_reset(trace_ctx_t *ctx)
{
    /* this relies upon the header occupying a whole number of bytes */
    assert(!fseek(ctx->file, ctx->reset_pos, SEEK_SET));
    ctx->read_buf_bits = 0;
    memset(ctx->last_vals, 0, sizeof(ctx->last_vals));
}

static void trace_event(trace_ctx_t *ctx, trace_event_desc_t *evd, uint32_t param, uint32_t *vals)
{
    trace_event_field_desc_t *fld;
    uint32_t last;
    int32_t fld_val;
    size_t i, bits, nibbles, width = 0;
    int fields;

    trace_write(ctx, evd->type & 0x3, 2);

    switch (evd->type & 0x3) {
    case TRACE_EVENT_DLOAD:
    case TRACE_EVENT_DSTORE:
        width = param;
        assert(width < 4);
        trace_write(ctx, width, 2);
        break;

    case TRACE_EVENT_IFETCH:
        break;

    case TRACE_EVENT_OTHER:
        trace_write(ctx, evd->type >> 2, 4);
        break;
    }

    /* if there are optional fields, specify how many we have */
    fields = evd->num_fields - evd->num_opt_fields;
    if (evd->num_opt_fields) {
        assert(param <= evd->num_opt_fields);
        bits = 32 - __builtin_clz((unsigned int)evd->num_opt_fields);
        trace_write(ctx, param, bits);
        fields += param;
    }

    for (i = 0; i < fields; i++) {
        fld = evd->fields[i];
        if (fld->enc == TRACE_ENC_SREL || fld->enc == TRACE_ENC_UREL) {
            last = ctx->last_vals[fld->data_type];
            bits = fld->shift;
            if (fld->shift_by_width)
                bits += width;
            last &= -(1 << bits);
            fld_val = (vals[i] - last) >> fld->shift;
            ctx->last_vals[fld->data_type] = vals[i];
        } else {
            fld_val = vals[i] >> fld->shift;
        }
        if (fld->shift_by_width)
            fld_val >>= width;
        switch (fld->enc) {
        case TRACE_ENC_SREL:
            bits = 1;
            if (fld_val)
                bits += 32 - __builtin_clz(fld_val ^ (fld_val >> 31));
            nibbles = (bits + 3) >> 2;
            trace_write(ctx, nibbles - 1, 3);
            trace_write(ctx, fld_val, nibbles << 2);
            break;

        case TRACE_ENC_UREL:
            bits = 1;
            if (fld_val)
                bits += 31 - __builtin_clz(fld_val);
            nibbles = (bits + 3) >> 2;
            trace_write(ctx, nibbles - 1, 3);
            trace_write(ctx, fld_val, nibbles << 2);
            break;

        case TRACE_ENC_BOOL:
            trace_write(ctx, !!fld_val, 1);
            break;

        default:
            fprintf(stderr, "Unhandled encoding %d\n", fld->enc);
            assert(0);
        }
    }

    /* end on a byte boundary */
    if (evd->type == TRACE_EVENT_PADTOBYTE && ctx->write_buf_idx % 8)
        trace_write(ctx, 0, 8 - (ctx->write_buf_idx % 8));
}

void trace_state(trace_ctx_t *ctx, trace_state_t *ts)
{
    ctx->state = *ts;
}

void trace_state_pc(trace_ctx_t *ctx, uint32_t pc)
{
    ctx->state.pc = pc;
}

void trace_state_actcyc(trace_ctx_t *ctx, uint32_t actcyc)
{
    ctx->state.actcyc = actcyc;
}

void trace_mmu(trace_ctx_t *ctx, trace_mmu_t *mmu)
{
    ctx->mmu = *mmu;
}
void trace_mmu_all(trace_ctx_t *ctx, uint32_t paddr, uint32_t ptaddr, uint8_t pgshift, uint8_t mmuctrl, uint8_t cachectrl)
{
    ctx->mmu.paddr = paddr;
    ctx->mmu.ptaddr = ptaddr;
    ctx->mmu.pgshift = pgshift;
    ctx->mmu.mmuctrl = mmuctrl;
    ctx->mmu.cachectrl = cachectrl;
}

void trace_event_dload(trace_ctx_t *ctx, uint32_t vaddr, uint8_t szlog2)
{
    uint32_t vals[3 + TRACE_MMU_FIELDS];
    vals[0] = ctx->state.pc;
    vals[1] = ctx->state.actcyc;
    vals[2] = vaddr;
    trace_event_mmu(ctx, vals + 3);
    trace_event(ctx, ctx->evd_dload, szlog2, vals);
}

void trace_event_dstore(trace_ctx_t *ctx, uint32_t vaddr, uint8_t szlog2)
{
    uint32_t vals[3 + TRACE_MMU_FIELDS];
    vals[0] = ctx->state.pc;
    vals[1] = ctx->state.actcyc;
    vals[2] = vaddr;
    trace_event_mmu(ctx, vals + 3);
    trace_event(ctx, ctx->evd_dstore, szlog2, vals);
}

void trace_event_ifetch(trace_ctx_t *ctx, uint32_t pc)
{
    uint32_t vals[2 + TRACE_MMU_FIELDS];
    trace_state_pc(ctx, pc);
    vals[0] = ctx->state.pc;
    vals[1] = ctx->state.actcyc;
    trace_event_mmu(ctx, vals + 2);
    trace_event(ctx, ctx->evd_ifetch, 0, vals);
}

static void trace_event_cacheflush(trace_ctx_t *ctx, bool icache, bool lineaddr, uint32_t addr, uint32_t size)
{
    uint32_t vals[6 + TRACE_MMU_FIELDS];
    vals[0] = icache;
    vals[1] = lineaddr;
    vals[2] = ctx->state.pc;
    vals[3] = addr;
    vals[4] = size;
    vals[5] = ctx->state.actcyc;
    trace_event_mmu(ctx, vals + 6);
    trace_event(ctx, ctx->evd_cacheflush, 0, vals);
}

void trace_event_dflush(trace_ctx_t *ctx, uint32_t addr, uint32_t size)
{
    trace_event_cacheflush(ctx, false, false, addr, size);
}

void trace_event_iflush(trace_ctx_t *ctx, uint32_t addr, uint32_t size)
{
    trace_event_cacheflush(ctx, true, false, addr, size);
}

void trace_event_dflushline(trace_ctx_t *ctx, uint32_t line, uint32_t size)
{
    trace_event_cacheflush(ctx, false, true, line, size);
}

void trace_event_iflushline(trace_ctx_t *ctx, uint32_t line, uint32_t size)
{
    trace_event_cacheflush(ctx, true, true, line, size);
}

void trace_event_ttevent(trace_ctx_t *ctx, uint16_t header, int n, uint32_t d1, uint32_t d2, uint32_t d3, uint32_t d4)
{
    uint32_t vals[6];
    vals[0] = ctx->state.actcyc;
    vals[1] = header;
    vals[2] = d1;
    vals[3] = d2;
    vals[4] = d3;
    vals[5] = d4;
    trace_event(ctx, ctx->evd_ttevent, n, vals);
}

void trace_event_mmswitch(trace_ctx_t *ctx, uint8_t thread, uint8_t global, uint32_t base)
{
    uint32_t vals[5];
    vals[0] = ctx->state.pc;
    vals[1] = ctx->state.actcyc;
    vals[2] = thread;
    vals[3] = global;
    vals[4] = base;
    trace_event(ctx, ctx->evd_mmswitch, 0, vals);
}

static void trace_event_prefetch(trace_ctx_t *ctx, bool icache, uint32_t vaddr, uint32_t size, bool priority)
{
    uint32_t vals[6 + TRACE_MMU_FIELDS];
    vals[0] = ctx->state.pc;
    vals[1] = ctx->state.actcyc;
    vals[2] = icache;
    vals[3] = priority;
    vals[4] = vaddr;
    vals[5] = size;
    trace_event_mmu(ctx, vals + 6);
    trace_event(ctx, ctx->evd_prefetch, 0, vals);
}

static void trace_event_prefetchexit(trace_ctx_t *ctx, bool icache)
{
    uint32_t vals[3];
    vals[0] = ctx->state.pc;
    vals[1] = ctx->state.actcyc;
    vals[2] = icache;
    trace_event(ctx, ctx->evd_prefetchexit, 0, vals);
}

void trace_event_dprefetch(trace_ctx_t *ctx, uint32_t vaddr, uint32_t size, bool priority)
{
    trace_event_prefetch(ctx, false, vaddr, size, priority);
}

void trace_event_dprefetchexit(trace_ctx_t *ctx)
{
    trace_event_prefetchexit(ctx, false);
}

void trace_event_iprefetch(trace_ctx_t *ctx, uint32_t vaddr, uint32_t size, bool priority)
{
    trace_event_prefetch(ctx, true, vaddr, size, priority);
}

void trace_event_iprefetchexit(trace_ctx_t *ctx)
{
    trace_event_prefetchexit(ctx, true);
}

static void trace_event_padtobyte(trace_ctx_t *ctx)
{
    trace_event(ctx, ctx->evd_padtobyte, 0, NULL);
}

bool trace_read_event(trace_ctx_t *ctx, trace_cb_t *cb, void *user)
{
    trace_event_desc_t *evd = NULL;
    trace_event_field_desc_t *fld;
    size_t i;
    uint8_t type, bits, nibbles, width = 0;
    uint32_t fld_val, last, vals[11];
    bool ret = true, read_success, icache, lineaddr;
    bool (*flush_func)(trace_ctx_t *ctx, void *user, trace_state_t *state, uint32_t vaddr, uint32_t sz, trace_mmu_t *mmu);
    bool (*flushline_func)(trace_ctx_t *ctx, void *user, trace_state_t *state, uint32_t line, uint32_t sz);
    bool (*prefetch_func)(trace_ctx_t *ctx, void *user, trace_state_t *ts, uint32_t vaddr, uint32_t lines, bool priority, trace_mmu_t *mmu);
    bool (*prefetchexit_func)(trace_ctx_t *ctx, void *user, trace_state_t *ts);
    int fields, opt_fields;

    type = trace_try_read(ctx, 2, &read_success);
    if (!read_success)
        return false;

    switch (type) {
    case TRACE_EVENT_DLOAD:
    case TRACE_EVENT_DSTORE:
        width = trace_read(ctx, 2);
        break;

    case TRACE_EVENT_IFETCH:
        break;

    case TRACE_EVENT_OTHER:
        type |= trace_read(ctx, 4) << 2;
        break;
    }

    for (i = 0; i < ctx->hdr.num_event_descs; i++) {
        if (ctx->hdr.event_descs[i]->type != type)
            continue;
        evd = ctx->hdr.event_descs[i];
        break;
    }
    if (!evd)
        return false;

    /* if there are optional fields, find how many we have */
    fields = evd->num_fields - evd->num_opt_fields;
    opt_fields = 0;
    if (evd->num_opt_fields) {
        bits = 32 - __builtin_clz((unsigned int)evd->num_opt_fields);
        opt_fields = trace_read(ctx, bits);
        if (opt_fields > evd->num_opt_fields)
            opt_fields = evd->num_opt_fields;
        fields += opt_fields;
    }


    for (i = 0; i < fields; i++) {
        fld = evd->fields[i];
        last = ctx->last_vals[fld->data_type];

        switch (fld->enc) {
        case TRACE_ENC_SREL:
            nibbles = trace_read(ctx, 3) + 1;
            bits = nibbles << 2;
            fld_val = trace_read(ctx, bits);
            fld_val = (uint32_t)(((int32_t)fld_val << (32 - bits)) >> (32 - bits));
            fld_val <<= fld->shift;
            bits = fld->shift;
            if (fld->shift_by_width) {
                fld_val <<= width;
                bits += width;
            }
            last &= -(1 << bits);
            fld_val += last;
            ctx->last_vals[fld->data_type] = fld_val;
            vals[i] = fld_val;
            break;

        case TRACE_ENC_UREL:
            nibbles = trace_read(ctx, 3) + 1;
            bits = nibbles << 2;
            fld_val = trace_read(ctx, bits);
            fld_val <<= fld->shift;
            bits = fld->shift;
            if (fld->shift_by_width) {
                fld_val <<= width;
                bits += width;
            }
            last &= -(1 << bits);
            fld_val += last;
            ctx->last_vals[fld->data_type] = fld_val;
            vals[i] = fld_val;
            break;

        case TRACE_ENC_BOOL:
            vals[i] = trace_read(ctx, 1) << fld->shift;
            break;

        default:
            fprintf(stderr, "Unhandled encoding %d\n", fld->enc);
            assert(0);
        }
    }

    switch (type) {
    case TRACE_EVENT_DLOAD:
        trace_state_pc(ctx, vals[0]);
        trace_state_actcyc(ctx, vals[1]);
        trace_mmu_fields(ctx, vals[2], vals + 3);
        if (cb->on_event_dload) ret = cb->on_event_dload(ctx, user, &ctx->state, vals[2], width, &ctx->mmu);
        break;
    case TRACE_EVENT_DSTORE:
        trace_state_pc(ctx, vals[0]);
        trace_state_actcyc(ctx, vals[1]);
        trace_mmu_fields(ctx, vals[2], vals + 3);
        if (cb->on_event_dstore) ret = cb->on_event_dstore(ctx, user, &ctx->state, vals[2], width, &ctx->mmu);
        break;
    case TRACE_EVENT_IFETCH:
        trace_state_pc(ctx, vals[0]);
        trace_state_actcyc(ctx, vals[1]);
        trace_mmu_fields(ctx, vals[0], vals + 2);
        if (cb->on_event_ifetch) ret = cb->on_event_ifetch(ctx, user, &ctx->state, vals[0], &ctx->mmu);
        break;
    case TRACE_EVENT_CACHEFLUSH:
        icache = vals[0];
        lineaddr = vals[1];
        trace_state_pc(ctx, vals[2]);
        trace_state_actcyc(ctx, vals[5]);
        if (lineaddr) {
            flushline_func = icache ? cb->on_event_iflushline : cb->on_event_dflushline;
            if (flushline_func) {
                ret = flushline_func(ctx, user, &ctx->state, vals[3], vals[4]);
            }
        } else {
            flush_func = icache ? cb->on_event_iflush : cb->on_event_dflush;
            trace_mmu_fields(ctx, vals[3], vals + 6);
            if (flush_func) {
                ret = flush_func(ctx, user, &ctx->state, vals[3], vals[4], &ctx->mmu);
            }
        }
        break;
    case TRACE_EVENT_TTEVENT:
        if (opt_fields && vals[1] & (1 << 1)) {
            trace_state_pc(ctx, vals[2]);
        } else {
            trace_state_pc(ctx, 0);
        }
        trace_state_actcyc(ctx, vals[0]);
        if (cb->on_event_ttevent) ret = cb->on_event_ttevent(ctx, user, &ctx->state, vals[1], opt_fields, vals + 2);
        break;
    case TRACE_EVENT_MMSWITCH:
        trace_state_pc(ctx, vals[0]);
        trace_state_actcyc(ctx, vals[1]);
        if (cb->on_event_mmswitch) ret = cb->on_event_mmswitch(ctx, user, &ctx->state, vals[2], vals[3], vals[4]);
        break;
    case TRACE_EVENT_PREFETCH:
        trace_state_pc(ctx, vals[0]);
        trace_state_actcyc(ctx, vals[1]);
        icache = vals[2];
        trace_mmu_fields(ctx, vals[4], vals + 6);
        prefetch_func = icache ? cb->on_event_iprefetch : cb->on_event_dprefetch;
        if (prefetch_func) ret = prefetch_func(ctx, user, &ctx->state, vals[4], vals[5], vals[3], &ctx->mmu);
        break;
    case TRACE_EVENT_PREFETCHEXIT:
        trace_state_pc(ctx, vals[0]);
        trace_state_actcyc(ctx, vals[1]);
        icache = vals[2];
        prefetchexit_func = icache ? cb->on_event_iprefetchexit : cb->on_event_dprefetchexit;
        if (prefetchexit_func) ret = prefetchexit_func(ctx, user, &ctx->state);
        break;
    case TRACE_EVENT_PADTOBYTE:
        if (ctx->read_buf_bits)
            trace_read(ctx, ctx->read_buf_bits);
        break;
    default:
        fprintf(stderr, "unknown event type %d\n", type);
        break;
    }

    return ret;
}

