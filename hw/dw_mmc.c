/*
 * Synopsys DesignWare Mobile Storage Host Devices.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#include "dw_mmc.h"

#include "hw.h"
#include "qemu-log.h"
#include "sd.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

enum {
    DW_MMC_INPUT_READONLY,
    DW_MMC_INPUT_INSERT,
    DW_MMC_INPUT_NR,
};

/* Internal DW-MMC state */
struct dw_mmc_state {
    /* register region */
    MemoryRegion iomem;

    /* outgoing interrupt */
    qemu_irq irq;
    /* incoming signals (see DW_MMC_INPUT_*) */
    qemu_irq *inputs;
    /* DMA DREQ lines */
    qemu_irq rxdrq;
    qemu_irq txdrq;

    /* SD cards */
    SDState **cards;

    /* hardware configuration */
    /* CARD_TYPE */
    uint8_t num_cards;
    /* H_BUS_TYPE */
    uint8_t data_width;
    /* H_ADDR_WIDTH */
    /* INTERNAL_DMAC */
    /* DMA_INTERFACE */
    /* GE_DMA_DATA_WIDTH */
    uint16_t fifo_depth;
    /* FIFO_RAM_INSIDE */
    /* NUM_CLK_DIVIDERS */
    /* IMPLEMENT_HOLD_REG */
    /* UID_REG */
    /* SET_CLK_FALSE_PATH */
    /* AREA_OPTIMIZED */

    /* registers */
    uint32_t ctrl;
    uint32_t clkdiv;
    uint32_t clksrc;
    uint32_t clkena;
    uint32_t tmout;
    uint32_t ctype;
    uint32_t blksiz;
    uint32_t bytcnt;
    uint32_t intmask;
    uint32_t cmdarg;
    uint32_t cmd;
    uint32_t resp[4];
    uint32_t rintsts;
    uint32_t fifoth;
    uint32_t cdetect;
    uint32_t wrtprt;

    /* current command state */
    uint8_t card_number;
    SDState *card;
    uint32_t cur_cmd;
    uint32_t cur_cmdarg;
    uint32_t cur_bytcnt;
    uint32_t cur_flags;
#define DW_MMC_FLAG_COUNTING_M  (1 << 0)

    /* fifo */
    uint32_t *fifo;
    uint8_t fifo_count;
    uint8_t fifo_tx;
    uint8_t fifo_rx;
};

enum {
    DW_MMC_CTRL    = 0x00,  /* Control */
    DW_MMC_PWREN   = 0x04,  /* Power-enable */
    DW_MMC_CLKDIV  = 0x08,  /* Clock-divider */
    DW_MMC_CLKSRC  = 0x0c,  /* Clock-source */
    DW_MMC_CLKENA  = 0x10,  /* Clock-enable */
    DW_MMC_TMOUT   = 0x14,  /* Time-out */
    DW_MMC_CTYPE   = 0x18,  /* Card-type */
    DW_MMC_BLKSIZ  = 0x1c,  /* Block-size */
    DW_MMC_BYTCNT  = 0x20,  /* Byte-count */
    DW_MMC_INTMASK = 0x24,  /* Interrupt-mask */
    DW_MMC_CMDARG  = 0x28,  /* Command-argument */
    DW_MMC_CMD     = 0x2c,  /* Command */
    DW_MMC_RESP0   = 0x30,  /* Response-0 */
    DW_MMC_RESP1   = 0x34,  /* Response-1 */
    DW_MMC_RESP2   = 0x38,  /* Response-2 */
    DW_MMC_RESP3   = 0x3c,  /* Response-3 */
    DW_MMC_MINTSTS = 0x40,  /* Masked interrupt-status */
    DW_MMC_RINTSTS = 0x44,  /* Raw interrupt-status */
    DW_MMC_STATUS  = 0x48,  /* Status */
    DW_MMC_FIFOTH  = 0x4c,  /* FIFO threshold */
    DW_MMC_CDETECT = 0x50,  /* Card-detect */
    DW_MMC_WRTPRT  = 0x54,  /* Write-protect */
    DW_MMC_GPIO    = 0x58,  /* GPIO */
    DW_MMC_TCBCNT  = 0x5c,  /* Transferred CIU card byte count */
    DW_MMC_TBBCNT  = 0x60,  /* Transferred host/DMA BIU-FIFO byte count */
    DW_MMC_DEBNCE  = 0x64,  /* Card detect debounce */
    DW_MMC_USRID   = 0x68,  /* User ID */
    DW_MMC_VERID   = 0x6c,  /* Synopsys version ID */
    DW_MMC_HCON    = 0x70,  /* Hardware configuration */
    DW_MMC_BMOD    = 0x80,  /* Bus Mode */
    DW_MMC_PLDMND  = 0x84,  /* Poll Demand */
    DW_MMC_DBADDR  = 0x88,  /* Descriptor List Base Address */
    DW_MMC_IDSTS   = 0x8c,  /* Internal DMAC Status */
    DW_MMC_IDINTEN = 0x90,  /* Internal DMAC Interrupt Enable */
    DW_MMC_DSCADDR = 0x94,  /* Current Host Descriptor Address */
    DW_MMC_BUFADDR = 0x98,  /* Current Host Buffer Address */
    DW_MMC_DATA    = 0x100, /* Data FIFO read/write */
};

#define DW_MMC_INT_END_BIT_ERROR_M      (0x1 << 15)
#define DW_MMC_INT_AUTO_COMMAND_DONE_M  (0x1 << 14)
#define DW_MMC_INT_START_BIT_ERROR_M    (0x1 << 13)
#define DW_MMC_INT_HW_LOCKED_ERROR_M    (0x1 << 12)
#define DW_MMC_INT_FIFO_UNDERUN_M       (0x1 << 11)
#define DW_MMC_INT_HOST_TIMEOUT_M       (0x1 << 10)
#define DW_MMC_INT_DATA_READ_TIMEOUT_M  (0x1 << 9)
#define DW_MMC_INT_RESPONSE_TIMEOUT_M   (0x1 << 8)
#define DW_MMC_INT_DATA_CRC_ERROR_M     (0x1 << 7)
#define DW_MMC_INT_RESPONSE_CRC_ERROR_M (0x1 << 6)
#define DW_MMC_INT_RX_FIFO_DATA_RQ_M    (0x1 << 5)
#define DW_MMC_INT_TX_FIFO_DATA_RQ_M    (0x1 << 4)
#define DW_MMC_INT_DATA_TRANSFER_OVER_M (0x1 << 3)
#define DW_MMC_INT_COMMAND_DONE_M       (0x1 << 2)
#define DW_MMC_INT_RESPONSE_ERROR_M     (0x1 << 1)
#define DW_MMC_INT_CARD_DETECT_M        (0x1 << 0)

#define DW_MMC_CTRL_ABORT_READ_DATA_M   (0x1 << 8)
#define DW_MMC_CTRL_SEND_IRQ_RESPONSE_M (0x1 << 7)
#define DW_MMC_CTRL_READ_WAIT_M         (0x1 << 6)
#define DW_MMC_CTRL_DMA_ENABLE_M        (0x1 << 5)
#define DW_MMC_CTRL_INT_ENABLE_M        (0x1 << 4)
#define DW_MMC_CTRL_DMA_RESET_M         (0x1 << 2)
#define DW_MMC_CTRL_FIFO_RESET_M        (0x1 << 1)
#define DW_MMC_CTRL_CONTROLLER_RESET_M  (0x1 << 0)

#define DW_MMC_TMOUT_M                  0xffff00ff

#define DW_MMC_CMD_START_CMD_M                      (0x1 << 31)
#define DW_MMC_CMD_UPDATE_CLOCK_REGISTERS_ONLY_M    (0x1 << 21)
#define DW_MMC_CMD_CARD_NUMBER_S                    16
#define DW_MMC_CMD_CARD_NUMBER_M                    (0x1f << \
                                                     DW_MMC_CMD_CARD_NUMBER_S)
#define DW_MMC_CMD_SEND_INITIALISATION_M            (0x1 << 15)
#define DW_MMC_CMD_STOP_ABORT_CMD_M                 (0x1 << 14)
#define DW_MMC_CMD_WAIT_PRVDATA_COMPLETE_M          (0x1 << 13)
#define DW_MMC_CMD_TRANSFER_MODE_M                  (0x1 << 11)
#define DW_MMC_CMD_WRITE_M                          (0x1 << 10)
#define DW_MMC_CMD_DATA_EXPECTED_M                  (0x1 << 9)
#define DW_MMC_CMD_CHECK_RESPONSE_CRC_M             (0x1 << 8)
#define DW_MMC_CMD_RESPONSE_LENGTH_M                (0x1 << 7)
#define DW_MMC_CMD_RESPONSE_EXPECT_M                (0x1 << 6)
#define DW_MMC_CMD_INDEX_S                          0
#define DW_MMC_CMD_INDEX_M                          (0x3f << DW_MMC_CMD_INDEX_S)
#define DW_MMC_CMD_IMPLEMENTED_M                    0x8020e7ff

#define DW_MMC_STATUS_DMA_REQ_M             (0x1 << 31)
#define DW_MMC_STATUS_DMA_ACK_M             (0x1 << 30)
#define DW_MMC_STATUS_FIFO_COUNT_S          17
#define DW_MMC_STATUS_FIFO_COUNT_M          (0x1fff << \
                                             DW_MMC_STATUS_FIFO_COUNT_S)
#define DW_MMC_STATUS_RESP_INDEX_S          11
#define DW_MMC_STATUS_RESP_INDEX_M          (0x3f << \
                                             DW_MMC_STATUS_RESP_INDEX_S)
#define DW_MMC_STATUS_DATA_STATE_MC_BUSY_M  (0x1 << 10)
#define DW_MMC_STATUS_DATA_BUSY_M           (0x1 << 9)
#define DW_MMC_STATUS_DATA_3_STATUS_M       (0x1 << 8)
#define DW_MMC_STATUS_CMD_FSM_STATES_S      4
#define DW_MMC_STATUS_CMD_FSM_STATES_M      (0xf << \
                                             DW_MMC_STATUS_CMD_FSM_STATES_S)
#define DW_MMC_STATUS_FIFO_FULL_M           (0x1 << 3)
#define DW_MMC_STATUS_FIFO_EMPTY_M          (0x1 << 2)
#define DW_MMC_STATUS_TX_WATERMARK_M        (0x1 << 1)
#define DW_MMC_STATUS_RX_WATERMARK_M        (0x1 << 0)

#define DW_MMC_FIFOTH_RX_WMARK_S        16
#define DW_MMC_FIFOTH_RX_WMARK_M        (0xfff << DW_MMC_FIFOTH_RX_WMARK_S)
#define DW_MMC_FIFOTH_TX_WMARK_S        0
#define DW_MMC_FIFOTH_TX_WMARK_M        (0xfff << DW_MMC_FIFOTH_TX_WMARK_S)
#define DW_MMC_FIFOTH_M                 0x7fffffff

#define DW_MMC_HCON_NUM_CARDS_S         1
#define DW_MMC_HCON_DATA_WIDTH_S        7

static bool dw_mmc_start_cmd(struct dw_mmc_state *s);

static void dw_mmc_update_irq(struct dw_mmc_state *s)
{
    uint32_t irq = 0;

    if (s->ctrl & DW_MMC_CTRL_INT_ENABLE_M) {
        irq = s->rintsts & s->intmask;
    }

    qemu_set_irq(s->irq, !!irq);
}

static void dw_mmc_fire_irqs(struct dw_mmc_state *s, uint32_t irqs)
{
    DBGLOG("dw_mmc: irqs(%#x)\n", irqs);
    s->rintsts |= irqs;
    dw_mmc_update_irq(s);
}

/* FIFO manipulation */

static int dw_mmc_rx_ready(struct dw_mmc_state *s)
{
    return s->fifo_count;
}

static int dw_mmc_tx_ready(struct dw_mmc_state *s)
{
    return s->fifo_depth - s->fifo_count;
}

/* is the rx watermark hit? */
static int dw_mmc_rx_watermark(struct dw_mmc_state *s)
{
    int rx_wmark = (s->fifoth & DW_MMC_FIFOTH_RX_WMARK_M)
                            >> DW_MMC_FIFOTH_RX_WMARK_S;
    return s->fifo_count > rx_wmark;
}

/* is the tx watermark hit? */
static int dw_mmc_tx_watermark(struct dw_mmc_state *s)
{
    int tx_wmark = (s->fifoth & DW_MMC_FIFOTH_TX_WMARK_M)
                            >> DW_MMC_FIFOTH_TX_WMARK_S;
    return s->fifo_count <= tx_wmark;
}

static uint32_t dw_mmc_rx(struct dw_mmc_state *s)
{
    uint32_t data;
    if (unlikely(!dw_mmc_rx_ready(s))) {
        return 0;
    }
    data = s->fifo[s->fifo_rx++];
    s->fifo_rx &= s->fifo_depth - 1;
    --s->fifo_count;
    DBGLOG("rx -> %08x [%d,%d,%d]\n",
           data, s->fifo_count, s->fifo_rx, s->fifo_tx);
    return data;
}

static void dw_mmc_tx(struct dw_mmc_state *s, uint32_t data)
{
    if (unlikely(!dw_mmc_tx_ready(s))) {
        return;
    }
    s->fifo[s->fifo_tx++] = data;
    s->fifo_tx &= s->fifo_depth - 1;
    ++s->fifo_count;
    DBGLOG("tx <- %08x [%d,%d,%d]\n",
           data, s->fifo_count, s->fifo_rx, s->fifo_tx);
}

/* fire relevant fifo irqs */
static void dw_mmc_fifo_irq_update(struct dw_mmc_state *s)
{
    uint32_t irqs = 0;

    if (s->cur_cmd & DW_MMC_CMD_WRITE_M) {
        if (dw_mmc_tx_watermark(s)) {
            irqs |= DW_MMC_INT_TX_FIFO_DATA_RQ_M;
        }
    } else {
        if (dw_mmc_rx_watermark(s)) {
            irqs |= DW_MMC_INT_RX_FIFO_DATA_RQ_M;
        }
    }

    if (s->ctrl & DW_MMC_CTRL_DMA_ENABLE_M) {
        qemu_set_irq(s->rxdrq, !!(irqs & DW_MMC_INT_TX_FIFO_DATA_RQ_M));
        qemu_set_irq(s->txdrq, !!(irqs & DW_MMC_INT_RX_FIFO_DATA_RQ_M));
    } else {
        qemu_set_irq(s->rxdrq, 0);
        qemu_set_irq(s->txdrq, 0);
    }

    if (!(s->cur_cmd & DW_MMC_CMD_START_CMD_M)) {
        return;
    }

    if (irqs)
        dw_mmc_fire_irqs(s, irqs);
}

/* fill/empty the fifo */

static void dw_mmc_transfer_over(struct dw_mmc_state *s)
{
    s->cur_cmd &= ~DW_MMC_CMD_START_CMD_M;
    dw_mmc_fire_irqs(s, DW_MMC_INT_DATA_TRANSFER_OVER_M);
    dw_mmc_start_cmd(s);
}

static void dw_mmc_fill_fifo(struct dw_mmc_state *s)
{
    uint32_t fifo_item;
    int t_over = 0;
    int shift;

    /* if there's nothing to read, the transaction is over */
    if (!sd_data_ready(s->card)) {
        dw_mmc_transfer_over(s);
        return;
    }

    /* don't enter the loop if we've exhausted the byte count */
    if (s->cur_flags & DW_MMC_FLAG_COUNTING_M && !s->cur_bytcnt) {
        return;
    }

    /* don't enter the loop if there's no space */
    if (!dw_mmc_tx_ready(s)) {
        return;
    }

    do {
        /* accumulate bytes starting at most significant */
        fifo_item = 0;
        for (shift = 0; shift <= 24; shift += 8) {
            fifo_item |= (uint32_t)sd_read_data(s->card) << shift;
            if (s->cur_flags & DW_MMC_FLAG_COUNTING_M) {
                if (!--s->cur_bytcnt) {
                    t_over = 1;
                    break;
                }
            }
            if (!sd_data_ready(s->card)) {
                t_over = 1;
                break;
            }
        }
        dw_mmc_tx(s, fifo_item);
        if (t_over) {
            /* we've just read the last byte, data transfer is over */
            dw_mmc_transfer_over(s);
            break;
        }
    } while (dw_mmc_tx_ready(s));

    dw_mmc_fifo_irq_update(s);
}

static void dw_mmc_empty_fifo(struct dw_mmc_state *s)
{
    uint32_t fifo_item;
    int shift;

    /* don't enter the loop if we've exhausted the byte count */
    if (s->cur_flags & DW_MMC_FLAG_COUNTING_M && !s->cur_bytcnt) {
        return;
    }

    /* don't enter the loop unless there's data to send */
    if (!dw_mmc_rx_ready(s)) {
        return;
    }

    do {
        fifo_item = dw_mmc_rx(s);
        for (shift = 0; shift <= 24; shift += 8) {
            sd_write_data(s->card, (fifo_item >> shift) & 0xff);
            if (s->cur_flags & DW_MMC_FLAG_COUNTING_M && !--s->cur_bytcnt) {
                dw_mmc_transfer_over(s);
                goto done;
            }
        }
    } while (dw_mmc_rx_ready(s));
done:
    dw_mmc_fifo_irq_update(s);
}

static bool dw_mmc_run_cmd(struct dw_mmc_state *s)
{
    SDRequest request;
    uint8_t response[16];
    uint8_t *response_p;
    int expected_len = 0;
    int len, i;
    uint32_t irqs = 0;

    if (!(s->cur_cmd & DW_MMC_CMD_START_CMD_M))
        return false;

    s->cur_cmd &= ~DW_MMC_CMD_START_CMD_M;

    if (s->cur_cmd & DW_MMC_CMD_RESPONSE_EXPECT_M) {
        if (s->cur_cmd & DW_MMC_CMD_RESPONSE_LENGTH_M) {
            expected_len = 4;
        } else {
            expected_len = 1;
        }
    }

    request.cmd = (s->cur_cmd & DW_MMC_CMD_INDEX_M) >> DW_MMC_CMD_INDEX_S;
    request.arg = s->cur_cmdarg;
    request.crc = 0; /* FIXME */

    memset(response, 0, sizeof(response));
    len = sd_do_command(s->card, &request, response);

    for (i = expected_len - 1, response_p = response;
            i >= 0; --i, response_p += 4) {
        s->resp[i] = (response_p[0] << 24) | (response_p[1] << 16)
                   | (response_p[2] << 8)  | (response_p[3] << 0);
    }

    if (s->cur_cmd & DW_MMC_CMD_CHECK_RESPONSE_CRC_M) {
        /* FIXME check CRC */
    }

    if (len != expected_len*4) {
        /* if we got the wrong response, report a response error */
        irqs |= DW_MMC_INT_RESPONSE_ERROR_M;
        dw_mmc_transfer_over(s);
    } else if (expected_len) {
        /* data is to be transferred, start filling fifo if it's a read */
        if (!(s->cur_cmd & DW_MMC_CMD_WRITE_M)) {
            dw_mmc_fill_fifo(s);
        }
        dw_mmc_fifo_irq_update(s);
    } else if (s->cur_cmd & DW_MMC_CMD_STOP_ABORT_CMD_M) {
        dw_mmc_transfer_over(s);
    } else if (!len) {
        dw_mmc_transfer_over(s);
    }
    dw_mmc_fire_irqs(s, irqs);
    return true;
}

static bool dw_mmc_start_cmd(struct dw_mmc_state *s)
{
    uint32_t cmd = s->cmd;

    if (!(s->cmd & DW_MMC_CMD_START_CMD_M)) {
        /* nothing valid to start */
        return false;
    }
    if (s->cur_cmd & DW_MMC_CMD_START_CMD_M) {
        /* wait until this command completes */
        return false;
    }

    /* auto-cleared after command handed off */
    s->cmd &= ~DW_MMC_CMD_START_CMD_M;

    if (cmd & DW_MMC_CMD_UPDATE_CLOCK_REGISTERS_ONLY_M) {
        /* nothing much should be done */
        dw_mmc_fire_irqs(s, DW_MMC_INT_COMMAND_DONE_M);
        return true;
    }

    s->card_number = (cmd & DW_MMC_CMD_CARD_NUMBER_M)
                         >> DW_MMC_CMD_CARD_NUMBER_S;
    if (s->card_number > s->num_cards) {
        s->card_number = 0;
        DBGLOG("dw_mmc: SD card number %d out of range (should be < %d)\n",
                s->card_number, s->num_cards);
        return false;
    }
    s->card = s->cards[s->card_number];
    s->cur_cmd = cmd;
    s->cur_cmdarg = s->cmdarg;
    s->cur_bytcnt = s->bytcnt;
    s->cur_flags = 0;
    if (s->cur_bytcnt) {
        s->cur_flags |= DW_MMC_FLAG_COUNTING_M;
    }

    dw_mmc_fire_irqs(s, DW_MMC_INT_COMMAND_DONE_M);
    dw_mmc_run_cmd(s);
    return true;
}

static uint64_t dw_mmc_io_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    struct dw_mmc_state *s = opaque;
    uint32_t ret = 0;

    if (addr < DW_MMC_DATA) {
        switch (addr) {
        case DW_MMC_CTRL:
            ret = s->ctrl;
            break;
        case DW_MMC_CLKDIV:
            ret = s->clkdiv;
            break;
        case DW_MMC_CLKSRC:
            ret = s->clksrc;
            break;
        case DW_MMC_CLKENA:
            ret = s->clkena;
            break;
        case DW_MMC_TMOUT:
            ret = s->tmout;
            break;
        case DW_MMC_CTYPE:
            ret = s->ctype;
            break;
        case DW_MMC_INTMASK:
            ret = s->intmask;
            break;
        case DW_MMC_CMDARG:
            ret = s->cmdarg;
            break;
        case DW_MMC_CMD:
            ret = s->cmd;
            break;
        case DW_MMC_RESP0:
        case DW_MMC_RESP1:
        case DW_MMC_RESP2:
        case DW_MMC_RESP3:
            ret = s->resp[(addr - DW_MMC_RESP0) >> 2];
            break;
        case DW_MMC_MINTSTS:
            ret = s->rintsts & s->intmask;
            break;
        case DW_MMC_RINTSTS:
            ret = s->rintsts;
            break;
        case DW_MMC_STATUS:
            ret |= s->fifo_count << DW_MMC_STATUS_FIFO_COUNT_S;
            if (!(s->cdetect & (1 << s->card_number))) {
                ret |= DW_MMC_STATUS_DATA_3_STATUS_M;
            }
            if (!dw_mmc_tx_ready(s)) {
                ret |= DW_MMC_STATUS_FIFO_FULL_M;
            }
            if (!dw_mmc_rx_ready(s)) {
                ret |= DW_MMC_STATUS_FIFO_EMPTY_M;
            }
            if (dw_mmc_tx_watermark(s)) {
                ret |= DW_MMC_STATUS_TX_WATERMARK_M;
            }
            if (dw_mmc_rx_watermark(s)) {
                ret |= DW_MMC_STATUS_RX_WATERMARK_M;
            }
            break;
        case DW_MMC_FIFOTH:
            ret = s->fifoth;
            break;
        case DW_MMC_CDETECT:
            ret = s->cdetect;
            break;
        case DW_MMC_WRTPRT:
            ret = s->wrtprt;
            break;
        case DW_MMC_VERID:
            ret = 0x5342210a;
            break;
        case DW_MMC_HCON:
            /*
             * Default configuration:
             * SD_MMC
             * AHB bus
             * 9bits h_addr_width
             * generic_dma
             * 32bit ge_dma_data_width
             * fifo ram inside
             * no hold register
             * no false path
             * num_clk_divider=1
             * area optimization
             */
            ret = 0x04262041;
            ret |= (s->num_cards - 1) << DW_MMC_HCON_NUM_CARDS_S;
            ret |= (s->data_width == 16 ? 0 :
                    s->data_width == 32 ? 1 :
                    s->data_width == 64 ? 2 : 7) << DW_MMC_HCON_DATA_WIDTH_S;
            break;
        default:
            DBGLOG("dw_mmc: unimplemented read(%#x)\n", addr);
            break;
        }
    } else {
        if (dw_mmc_rx_ready(s)) {
            /* Read from FIFO */
            ret = dw_mmc_rx(s);
            dw_mmc_fill_fifo(s);
            dw_mmc_fifo_irq_update(s);
        } else {
            /* User error! */
            dw_mmc_fire_irqs(s, DW_MMC_INT_FIFO_UNDERUN_M);
        }
    }
    return ret;
}

uint32_t dw_mmc_get_dma_wdata(struct dw_mmc_state *s)
{
    uint32_t ret = 0;

    if (!(s->ctrl & DW_MMC_CTRL_DMA_ENABLE_M)) {
        DBGLOG("dw_mmc DMA wdata whilst DMA disabled\n");
        return ret;
    }

    if (dw_mmc_rx_ready(s)) {
        /* Read from FIFO */
        ret = dw_mmc_rx(s);
        dw_mmc_fill_fifo(s);
        dw_mmc_fifo_irq_update(s);
    } else {
        /* User error! */
        dw_mmc_fire_irqs(s, DW_MMC_INT_FIFO_UNDERUN_M);
    }

    return ret;
}

static void dw_mmc_io_write(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    struct dw_mmc_state *s = opaque;
    uint32_t diff;

    if (addr < DW_MMC_DATA) {
        switch (addr) {
        case DW_MMC_CTRL:
            diff = s->ctrl ^ val;
            s->ctrl = val;

            if (diff & DW_MMC_CTRL_INT_ENABLE_M) {
                dw_mmc_update_irq(s);
            }
            if (diff & DW_MMC_CTRL_DMA_ENABLE_M) {
                DBGLOG("dw_mmc DMA enable = %d\n", !!(val & DW_MMC_CTRL_DMA_ENABLE_M));
                dw_mmc_fifo_irq_update(s);
            }

            if (val & DW_MMC_CTRL_DMA_RESET_M) {
                /* reset internal dma */

                /* the bit auto-clears */
                s->ctrl &= ~DW_MMC_CTRL_DMA_RESET_M;
            }
            if (val & DW_MMC_CTRL_FIFO_RESET_M) {
                s->fifo_count = 0;
                s->fifo_tx = 0;
                s->fifo_rx = 0;

                /* the bit auto-clears */
                s->ctrl &= ~DW_MMC_CTRL_FIFO_RESET_M;
            }
            if (val & DW_MMC_CTRL_CONTROLLER_RESET_M) {
                /* reset biu/ciu interface, ciu and state machines */
                s->cur_cmd = 0;
                s->cur_bytcnt = 0;
                s->cur_flags = 0;

                /* reset register fields */
                s->ctrl &= ~(DW_MMC_CTRL_ABORT_READ_DATA_M   |
                             DW_MMC_CTRL_SEND_IRQ_RESPONSE_M |
                             DW_MMC_CTRL_READ_WAIT_M);
                s->cmd &= ~DW_MMC_CMD_START_CMD_M;

                /* the bit auto-clears */
                s->ctrl &= ~DW_MMC_CTRL_CONTROLLER_RESET_M;
            }
            break;
        case DW_MMC_CLKDIV:
            s->clkdiv = val;
            break;
        case DW_MMC_CLKSRC:
            s->clksrc = val;
            break;
        case DW_MMC_CLKENA:
            s->clkena = val;
            break;
        case DW_MMC_TMOUT:
            s->tmout = val & DW_MMC_TMOUT_M;
            break;
        case DW_MMC_CTYPE:
            s->ctype = val;
            break;
        case DW_MMC_BLKSIZ:
            s->blksiz = val;
            break;
        case DW_MMC_BYTCNT:
            s->bytcnt = val;
            break;
        case DW_MMC_INTMASK:
            s->intmask = val;
            dw_mmc_update_irq(s);
            break;
        case DW_MMC_CMDARG:
            s->cmdarg = val;
            break;
        case DW_MMC_CMD:
            s->cmd = val;
#if DEBUG_LEVEL >= 2
            if (val & ~DW_MMC_CMD_IMPLEMENTED_M) {
                cpu_abort(cpu_single_env,
                          "dw_mmc: unimplemented CMD bits %08x (%08x)\n",
                          val & ~DW_MMC_CMD_IMPLEMENTED_M, val);
            }
#endif
            dw_mmc_start_cmd(s);
            break;
        case DW_MMC_RINTSTS:
            s->rintsts &= ~val;
            dw_mmc_update_irq(s);
            break;
        case DW_MMC_FIFOTH:
            s->fifoth = val & DW_MMC_FIFOTH_M;
            break;
        default:
            DBGLOG("dw_mmc: unimplemented write(%#x, %#x)\n", addr, val);
            break;
        }
    } else {
        if (dw_mmc_tx_ready(s)) {
            /* Write to FIFO */
            dw_mmc_tx(s, val);
            dw_mmc_empty_fifo(s);
            dw_mmc_fifo_irq_update(s);
        } else {
            /* User error! */
            dw_mmc_fire_irqs(s, DW_MMC_INT_FIFO_UNDERUN_M);
        }
    }
}

void dw_mmc_set_dma_rdata(struct dw_mmc_state *s, uint32_t val)
{
    if (!(s->ctrl & DW_MMC_CTRL_DMA_ENABLE_M)) {
        DBGLOG("dw_mmc DMA rdata whilst DMA disabled\n");
        return;
    }

    if (dw_mmc_tx_ready(s)) {
        /* Write to FIFO */
        dw_mmc_tx(s, val);
        dw_mmc_empty_fifo(s);
        dw_mmc_fifo_irq_update(s);
    } else {
        /* User error! */
        dw_mmc_fire_irqs(s, DW_MMC_INT_FIFO_UNDERUN_M);
    }
}

static const MemoryRegionOps dw_mmc_io_ops = {
    .read = dw_mmc_io_read,
    .write = dw_mmc_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void dw_mmc_reset(struct dw_mmc_state *s)
{
    /* registers */
    s->ctrl     = 0;
    s->clkdiv   = 0;
    s->clksrc   = 0;
    s->clkena   = 0;
    s->tmout    = 0xffffff40;
    s->ctype    = 0;
    s->blksiz   = 0x200;
    s->bytcnt   = 0x200;
    s->intmask  = 0;
    s->cmdarg   = 0;
    s->cmd      = 0;
    memset(s->resp, 0, sizeof(s->resp));
    s->rintsts  = 0;
    s->fifoth   = (s->fifo_depth - 1) << DW_MMC_FIFOTH_RX_WMARK_S;

    s->card_number = 0;
    s->card = s->cards[0];
    s->cur_bytcnt = 0;
    s->cur_flags = 0;

    s->fifo_count = 0;
    s->fifo_tx = 0;
    s->fifo_rx = 0;

    dw_mmc_fifo_irq_update(s);
}

static void dw_mmc_input_handler(void *opaque, int n, int level)
{
    struct dw_mmc_state *s = opaque;
    int card = n / DW_MMC_INPUT_NR;
    uint32_t tmp;

    switch (n) {
    case DW_MMC_INPUT_READONLY:
        tmp = s->wrtprt;
        if (level) {
            s->wrtprt |= (1 << card);
        } else {
            s->wrtprt &= ~(1 << card);
        }
        break;
    case DW_MMC_INPUT_INSERT:
        tmp = s->cdetect;
        /* cdetect is inverse logic */
        if (level) {
            s->cdetect &= ~(1 << card);
        } else {
            s->cdetect |= (1 << card);
        }
        if (tmp != s->cdetect) {
            dw_mmc_fire_irqs(s, DW_MMC_INT_CARD_DETECT_M);
        }
        break;
    }
}

struct dw_mmc_state *dw_mmc_init(MemoryRegion *address_space,
                                 hwaddr iomem, target_ulong len,
                                 unsigned int num_cards,
                                 unsigned int fifo_depth,
                                 BlockDriverState **bd,
                                 qemu_irq irq, qemu_irq rxdrq, qemu_irq txdrq)
{
    struct dw_mmc_state *s;
    int i;

    /* num_cards should be in range */
    if (num_cards < 1 || num_cards > 16) {
        return NULL;
    }
    /* fifo_depth should be a power of 2 and in range */
    if (fifo_depth & (fifo_depth - 1)) {
        return NULL;
    }
    if (fifo_depth < 8 || fifo_depth > 4096) {
        return NULL;
    }

    /* hardware configuration */
    s = g_malloc0(sizeof(struct dw_mmc_state));
    s->num_cards = num_cards;
    s->fifo_depth = fifo_depth;
    s->data_width = 8*sizeof(s->fifo[0]);
    s->cards = g_malloc(sizeof(s->cards[0])*num_cards);
    s->fifo = g_malloc(sizeof(s->fifo[0])*fifo_depth);
    s->rxdrq = rxdrq;
    s->txdrq = txdrq;

    /* connect it up to the rest of the system */
    s->irq = irq;
    s->inputs = qemu_allocate_irqs(dw_mmc_input_handler, s,
                                   DW_MMC_INPUT_NR * s->num_cards);
    for (i = 0; i < num_cards; ++i) {
        s->cards[i] = sd_init(bd[i], 0);
    }

    /* reset registers, nothing initialy inserted */
    s->cdetect = (1 << s->num_cards) - 1;
    s->wrtprt = (1 << s->num_cards) - 1;
    dw_mmc_reset(s);
    /* set input signals, it's safe now the registers are reset */
    for (i = 0; i < num_cards; ++i) {
        sd_set_cb(s->cards[i],
                  s->inputs[i*DW_MMC_INPUT_NR + DW_MMC_INPUT_READONLY],
                  s->inputs[i*DW_MMC_INPUT_NR + DW_MMC_INPUT_INSERT]);
    }

    /* initialise register region */
    memory_region_init_io(&s->iomem, &dw_mmc_io_ops, s,
                          "dw-mmc", len - 1);
    memory_region_add_subregion(address_space, iomem, &s->iomem);

    return s;
}

int dw_mmc_get_cd(struct dw_mmc_state *s, int card)
{
    return s->cdetect & (1 << card);
}
