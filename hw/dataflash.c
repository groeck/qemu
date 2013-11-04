/*
 * Atmel DataFlash over SPI (AT45)
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#include "hw/dataflash.h"

#include "qemu-common.h"
#include "qemu-log.h"
#include "blockdev.h"
#include "host-utils.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

enum {
    OP_BLOCK_ERASE              = 0x50,
    OP_READ_SECURITY            = 0x77,
    OP_PAGE_ERASE               = 0x81,
    OP_BUFFER1_TO_MAIN_ERASE    = 0x83,
    OP_BUFFER1_WRITE            = 0x84,
    OP_BUFFER2_TO_MAIN_ERASE    = 0x86,
    OP_BUFFER2_WRITE            = 0x87,
    OP_BUFFER1_TO_MAIN_NOERASE  = 0x88,
    OP_BUFFER2_TO_MAIN_NOERASE  = 0x89,
    OP_CHIP_ID                  = 0x9f,
    OP_PAGE_READ                = 0xd2,
    OP_STATUS_READ              = 0xd7,
    OP_CONT_READ                = 0xe8,
};

struct dataflash_s {
    uint8_t id;
    uint8_t density;

    /* flash memory */
    unsigned int len;
    unsigned int page_size;
    unsigned int page_bits;
    unsigned int block_size;
    unsigned int block_bits;
    BlockDriverState *bdrv;
    uint8_t otp[64];

    /* buffers (all 1 page) */
    uint8_t *buf[3]; /* 2 standard, 1 for internal use */

    /* state machine */
    unsigned int cs;
    uint8_t opcode;
    int error;
    uint32_t addr;
    uint32_t offset;

    size_t rx_len;
    uint8_t rxbuf[32];
    size_t rxbuf_count, rxbuf_ridx, rxbuf_widx;

    uint8_t txbuf[32];
    size_t txbuf_count, txbuf_ridx, txbuf_widx;
};

struct type_info_s {
    uint8_t id;
    uint8_t density;
    unsigned int page_size;
    unsigned int page_count;
};

struct type_info_s type_info[] = {
    [AT45DB321x] = { 0x27, 0x6, 528, 8129 },
    [AT45DB642x] = { 0x28, 0x7, 1056, 8192 },
};

#include "cpu.h"

static bool dataflash_tx_write(struct dataflash_s *at, uint8_t dat)
{
    if (at->txbuf_count >= sizeof(at->txbuf))
        return false;

    at->txbuf[at->txbuf_widx++] = dat;
    at->txbuf_widx %= sizeof(at->txbuf);
    at->txbuf_count++;

    return true;
}

static uint8_t dataflash_tx_read(struct dataflash_s *at)
{
    uint8_t dat;

    if (!at->txbuf_count)
        return 0xff;

    dat = at->txbuf[at->txbuf_ridx++];
    at->txbuf_ridx %= at->txbuf_count;

    return dat;
}

static bool dataflash_rx_write(struct dataflash_s *at, uint8_t dat)
{
    if (at->rxbuf_count >= sizeof(at->rxbuf))
        return false;

    at->rxbuf[at->rxbuf_widx++] = dat;
    at->rxbuf_widx %= sizeof(at->rxbuf);
    at->rxbuf_count++;

    return true;
}

static uint8_t dataflash_rx_read(struct dataflash_s *at)
{
    uint8_t dat;

    if (!at->rxbuf_count)
        return 0xff;

    dat = at->rxbuf[at->rxbuf_ridx++];
    at->rxbuf_ridx %= sizeof(at->rxbuf);
    at->rxbuf_count--;

    return dat;
}

static void dataflash_cmd_start(struct dataflash_s *at)
{
    DBGLOG("dataflash cmd 0x%02x start\n", at->opcode);

    switch (at->opcode) {
    case OP_BLOCK_ERASE:
    case OP_READ_SECURITY:
    case OP_PAGE_ERASE:
    case OP_BUFFER1_TO_MAIN_ERASE:
    case OP_BUFFER1_WRITE:
    case OP_BUFFER2_TO_MAIN_ERASE:
    case OP_BUFFER2_WRITE:
    case OP_BUFFER1_TO_MAIN_NOERASE:
    case OP_BUFFER2_TO_MAIN_NOERASE:
        at->rx_len = 3;
        break;

    case OP_CHIP_ID:
        dataflash_tx_write(at, 0x1f);
        dataflash_tx_write(at, at->id);
        dataflash_tx_write(at, 0x00);
        break;

    case OP_STATUS_READ:
        break;

    case OP_PAGE_READ:
    case OP_CONT_READ:
        at->rx_len = 7;
        break;

    default:
        DBGLOG("dataflash unknown command 0x%02x\n", at->opcode);
        break;
    }
}

static void dataflash_cmd_rxdone(struct dataflash_s *at)
{
    uint32_t pagenum, pageoffset;

    DBGLOG("dataflash cmd 0x%02x RX done\n", at->opcode);

    switch (at->opcode) {
    case OP_BLOCK_ERASE:
        at->addr =
            (dataflash_rx_read(at) << 16) |
            (dataflash_rx_read(at) << 8) |
            dataflash_rx_read(at);
        /* count in blocks */
        at->addr = at->addr >> at->block_bits;
        break;

    case OP_READ_SECURITY:
        at->offset = 0;
        break;

    case OP_PAGE_ERASE:
    case OP_BUFFER1_TO_MAIN_ERASE:
    case OP_BUFFER1_TO_MAIN_NOERASE:
    case OP_BUFFER2_TO_MAIN_ERASE:
    case OP_BUFFER2_TO_MAIN_NOERASE:
        at->addr =
            (dataflash_rx_read(at) << 16) |
            (dataflash_rx_read(at) << 8) |
            dataflash_rx_read(at);
        pagenum = at->addr >> at->page_bits;
        at->offset = pagenum*at->page_size;
        break;

    case OP_BUFFER1_WRITE:
    case OP_BUFFER2_WRITE:
        at->addr =
            (dataflash_rx_read(at) << 16) |
            (dataflash_rx_read(at) << 8) |
            dataflash_rx_read(at);
        at->addr &= (1 << at->page_bits) - 1;
        break;

    case OP_PAGE_READ:
    case OP_CONT_READ:
        at->addr = 
            (dataflash_rx_read(at) << 16) |
            (dataflash_rx_read(at) << 8) |
            dataflash_rx_read(at);

        pagenum = at->addr >> at->page_bits;
        pageoffset = at->addr & ((1 << at->page_bits) - 1);
        at->offset = pagenum*at->page_size + pageoffset;
 
        /*
         * FIXME is this how the hardware handles out of range
         * addresses?
         */
        if (at->offset >= at->len)
            at->error = 1;

        break;

    default:
        DBGLOG("dataflash unexpected RX for command 0x%02x\n", at->opcode);
        break;
    }
}

static void dataflash_cmd_complete(struct dataflash_s *at)
{
    int i;

    DBGLOG("dataflash cmd 0x%02x complete\n", at->opcode);

    if (at->bdrv) {
        switch (at->opcode) {
        case OP_BLOCK_ERASE:
            for (i = 0; i < 8; ++i) {
                if (bdrv_pwrite(at->bdrv, at->addr * at->block_size
                                               + i * at->page_size,
                                at->buf[2], at->page_size) == -1) {
                    DBGLOG("%s: block erase error\n", __func__);
                    break;
                }
            }
            break;

        case OP_PAGE_ERASE:
            if (bdrv_pwrite(at->bdrv, at->offset, at->buf[2],
                            at->page_size) == -1) {
                DBGLOG("%s: page erase error\n", __func__);
            }
            break;

        case OP_BUFFER1_TO_MAIN_ERASE:
        case OP_BUFFER1_TO_MAIN_NOERASE:
            if (bdrv_pwrite(at->bdrv, at->offset, at->buf[0],
                            at->page_size) == -1) {
                DBGLOG("%s: buffer1 to main error\n", __func__);
            }
            break;

        case OP_BUFFER2_TO_MAIN_ERASE:
        case OP_BUFFER2_TO_MAIN_NOERASE:
            if (bdrv_pwrite(at->bdrv, at->offset, at->buf[1],
                            at->page_size) == -1) {
                DBGLOG("%s: buffer2 to main error\n", __func__);
            }
            break;
        }
    }

    at->opcode = 0x00;
    at->error = 0;
    at->rx_len = 0;
    at->rxbuf_count = at->rxbuf_ridx = at->rxbuf_widx = 0;
    at->txbuf_count = at->txbuf_ridx = at->txbuf_widx = 0;
}

void dataflash_spi_setcs(void *opaque, unsigned int cs)
{
    struct dataflash_s *at = opaque;

    if (at->cs && !cs)
        dataflash_cmd_complete(at);

    at->cs = cs;
}

void dataflash_spi_rxtx(void *opaque, size_t len,
                        uint8_t *rx, uint8_t *tx)
{
    struct dataflash_s *at = opaque;
    size_t sz, rxidx = 0, txidx = 0;

    if (!at->cs) {
        DBGLOG("dataflash rxtx without cs\n");
        return;
    }

    while (rxidx < len) {
        if (!at->opcode) {
            at->opcode = rx[rxidx++];
            dataflash_cmd_start(at);
            tx[txidx++] = 0xff;
            continue;
        }

        if (at->rx_len) {
            dataflash_rx_write(at, rx[rxidx++]);
            tx[txidx++] = 0xff;
            if (!--at->rx_len)
                dataflash_cmd_rxdone(at);
            continue;
        } else {
            switch (at->opcode) {
            case OP_BUFFER1_WRITE:
                if (at->addr >= at->page_size) {
                    at->addr = 0;
                }
                at->buf[0][at->addr++] = rx[rxidx++];
                continue;
            case OP_BUFFER2_WRITE:
                if (at->addr >= at->page_size) {
                    at->addr = 0;
                }
                at->buf[1][at->addr++] = rx[rxidx++];
                continue;
            }
        }

        rxidx = len;
    }

    while (txidx < len) {
        if (at->error) {
            dataflash_cmd_complete(at);
            memset(&tx[txidx], 0xff, len - txidx);
            txidx = len;
            break;
        }

        switch (at->opcode) {
        case OP_READ_SECURITY:
            sz = MIN(len - txidx, sizeof(at->otp) - at->offset);
            memcpy(&tx[txidx], &at->otp[at->offset], sz);
            at->offset = (at->offset + sz) % sizeof(at->otp);
            txidx += sz;
            memset(&tx[txidx], 0xff, len - txidx);
            txidx = len;
            break;

        case OP_STATUS_READ:
            memset(&tx[txidx], 0xc4 | ((at->density) << 3), len - txidx);
            txidx = len;
            break;

        case OP_PAGE_READ:
        case OP_CONT_READ:
            sz = MIN(len - txidx, at->len - at->offset);
            if (!at->bdrv ||
                    bdrv_pread(at->bdrv, at->offset, &tx[txidx], sz) == -1) {
                DBGLOG("%s: read error\n", __func__);
                memset(&tx[txidx], 0xff, sz);
            }
            at->offset += sz;
            txidx += sz;
            memset(&tx[txidx], 0xff, len - txidx);
            txidx = len;
            break;

        default:
            tx[txidx++] = dataflash_tx_read(at);
            break;
        }
    }
}

struct dataflash_s *dataflash_init(int type)
{
    struct type_info_s *tinfo = &type_info[type];
    struct dataflash_s *at = g_malloc0(sizeof(struct dataflash_s));
    DriveInfo *dinfo;
    int i;

    at->id = tinfo->id;
    at->density = tinfo->density;
    at->len = tinfo->page_count * tinfo->page_size;
    at->page_size = tinfo->page_size;
    at->page_bits = 32 - clz32(at->page_size - 1);
    at->block_size = at->page_size * 8;
    at->block_bits = at->page_bits + 3;
    memset(at->otp, 0xff, sizeof(at->otp));
    for (i = 0; i < ARRAY_SIZE(at->buf); ++i) {
        at->buf[i] = g_malloc(at->page_size);
        memset(at->buf[i], 0xff, at->page_size);
    }

    /* preload from the mtd block if one has been specified */
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        at->bdrv = dinfo->bdrv;
    }

    return at;
}

void dataflash_otp_set(struct dataflash_s *at, const uint8_t *otp, size_t sz)
{
    size_t cp = MIN(sz, sizeof(at->otp));
    memcpy(at->otp, otp, cp);
    memset(at->otp + sz, 0xff, sizeof(at->otp) - cp);
}
