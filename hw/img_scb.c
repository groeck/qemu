#include "img_scb.h"

#include "i2c.h"
#include "qemu-log.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

typedef enum {
    SCB_LINE_STATUS                 = 0x00,
    SCB_LINE_OVERRIDE               = 0x04,
    SCB_MASTER_READ_ADDRESS         = 0x08,
    SCB_MASTER_READ_COUNT           = 0x0c,
    SCB_MASTER_WRITE_ADDRESS        = 0x10,
    SCB_MASTER_READ_DATA            = 0x14,
    SCB_MASTER_WRITE_DATA           = 0x18,
    SCB_MASTER_FILL_STATUS          = 0x1c,
    SCB_SLAVE_ADDRESS_BASE          = 0x20,
    SCB_SLAVE_ADDRESS_MASK          = 0x24,
    SCB_SLAVE_ADDRESS_CAUGHT        = 0x28,
    SCB_SLAVE_READ_DATA             = 0x30,
    SCB_SLAVE_FILL_STATUS           = 0x34,
    SCB_SLAVE_WRITE_DATA            = 0x38,
    SCB_CLK_SET                     = 0x3c,
    SCB_INTERRUPT_STATUS            = 0x40,
    SCB_INTERRUPT_CLEAR             = 0x44,
    SCB_INTERRUPT_MASK              = 0x48,
    SCB_GENERAL_CONTROL             = 0x4c,
    SCB_TIME_TPL                    = 0x50,
    SCB_TIME_TPH                    = 0x54,
    SCB_TIME_TP2S                   = 0x58,
    SCB_TIME_TBI                    = 0x60,
    SCB_TIME_TSL                    = 0x64,
    SCB_TIME_TDL                    = 0x68,
    SCB_TIME_TSDL                   = 0x6c,
    SCB_TIME_TSDH                   = 0x70,
    SCB_MASTER_EXTEND_READ_ADDRESS  = 0x74,
    SCB_MASTER_EXTEND_WRITE_ADDRESS = 0x78,
    SCB_MASTER_WRITE_COUNT          = 0x7c,
    CR_SCB_CORE_REV                 = 0x80,
    SCB_TIME_TCKH                   = 0x84,
    SCB_TIME_TCKL                   = 0x88,
    SCB_SLAVE_DATA_WRITTEN          = 0x90,
    SCB_MASTER_DATA_READ            = 0x94,
    SCB_CLEAR_STATUS                = 0x98,
} ImgSCBReg;

typedef enum {
    LINE_SCLK                       = (1 << 0),
    LINE_SCLK_EN                    = (1 << 1),
    LINE_SDAT                       = (1 << 2),
    LINE_SDAT_EN                    = (1 << 3),
    LINE_DET_START                  = (1 << 4),
    LINE_DET_STOP                   = (1 << 5),
    LINE_DET_ACK                    = (1 << 6),
    LINE_DET_NACK                   = (1 << 7),
    LINE_BUS_IDLE                   = (1 << 8),
    LINE_TRANS_DONE                 = (1 << 9),
    LINE_SCLK_OUT                   = (1 << 10),
    LINE_SDAT_OUT                   = (1 << 11),
    LINE_GEN_LINE_MASK              = (1 << 12),
    LINE_START_BIT_DET              = (1 << 13),
    LINE_STOP_BIT_DET               = (1 << 14),
    LINE_ACK_DET                    = (1 << 15),
    LINE_NACK_DET                   = (1 << 16),
    LINE_INPUT_HELD_VALID_DET       = (1 << 17),
    LINE_ABORT_DET                  = (1 << 18),
    LINE_TRANS_HALTED               = (1 << 19),
} ImgSCBLineStatus;

typedef enum {
    CLEAR_START_DET                 = (1 << 0),
    CLEAR_STOP_DET                  = (1 << 1),
    CLEAR_ACK_DET                   = (1 << 2),
    CLEAR_NACK_DET                  = (1 << 3),
    CLEAR_INPUT_HELD_VALID_DET      = (1 << 4),
    CLEAR_ABORT_DET                 = (1 << 5),
} ImgSCBClearStatus;

typedef enum {
    OVERRIDE_SCLK                   = (1 << 0),
    OVERRIDE_SCLK_EN                = (1 << 1),
    OVERRIDE_SDAT                   = (1 << 2),
    OVERRIDE_SDAT_EN                = (1 << 3),
    OVERRIDE_MASTER                 = (1 << 9),
    OVERRIDE_LINES                  = (1 << 10),
    OVERRIDE_DIRECT                 = (1 << 11),
} ImgSCBOverride;

typedef enum {
    OVERRIDE_CMD_PAUSE              = 0x00,
    OVERRIDE_CMD_GEN_DATA           = 0x01,
    OVERRIDE_CMD_GEN_START          = 0x02,
    OVERRIDE_CMD_GEN_STOP           = 0x03,
    OVERRIDE_CMD_GEN_ACK            = 0x04,
    OVERRIDE_CMD_GEN_NACK           = 0x05,
    OVERRIDE_CMD_RETRIEVE_DATA      = 0x08,
    OVERRIDE_CMD_RETRIEVE_ACK       = 0x09,
} ImgSCBOverrideCmd;

#define OVERRIDE_CMD_SHIFT  4
#define OVERRIDE_CMD_MASK   0x1f
#define OVERRIDE_DATA_SHIFT 24
#define OVERRIDE_DATA_MASK  0xff

typedef enum {
    INT_BUS_INACTIVE                = (1 << 0),
    INT_UNEXPECTED_START_BIT        = (1 << 1),
    INT_SCLK_LOW_TIMEOUT            = (1 << 2),
    INT_SDAT_LOW_TIMEOUT            = (1 << 3),
    INT_WRITE_ACK_ERROR             = (1 << 4),
    INT_ADDR_ACK_ERROR              = (1 << 5),
    INT_MASTER_WAIT_READ_DATA       = (1 << 6),
    INT_SLAVE_WAIT_WRITE_DATA       = (1 << 7),
    INT_SLAVE_WRITE_FIFO_FULL       = (1 << 8),
    INT_MASTER_READ_FIFO_FULL       = (1 << 9),
    INT_MASTER_READ_FIFO_FILLING    = (1 << 10),
    INT_MASTER_WRITE_FIFO_EMPTY     = (1 << 11),
    INT_MASTER_WRITE_FIFO_ALMOST    = (1 << 12),
    INT_SLAVE_TRANS_HALT_WAIT_READ  = (1 << 13),
    INT_SLAVE_HALT_WAIT_DATA        = (1 << 14),
    INT_T_DONE                      = (1 << 15),
    INT_SLAVE_EVENT                 = (1 << 16),
    INT_MASTER_HALT_BIT             = (1 << 17),
    INT_TIMING                      = (1 << 18),
} ImgSCBInterrupt;

typedef enum {
    CTL_RESET_GENDATA               = (1 << 0),
    CTL_RESET_DETDATA               = (1 << 1),
    CTL_RESET_MASTER                = (1 << 2),
    CTL_RESET_SLAVEDATA             = (1 << 3),
    CTL_RESET_REGISTER              = (1 << 4),
    CTL_CLKEN_DATA                  = (1 << 5),
    CTL_CLKEN_MASTER                = (1 << 6),
    CTL_CLKEN_SLAVE                 = (1 << 7),
    CTL_CLKEN_REGISTER              = (1 << 8),
    CTL_TRANS_HALT                  = (1 << 9),
} ImgSCBGenCtl;

typedef enum {
    STATE_IDLE = 0,
    STATE_READ = 1,
    STATE_WRITE = 2,
} ImgSCBState;

struct img_scb_state_s {
    MemoryRegion iomem;
    qemu_irq irq;
    i2c_bus *bus;

    int state;
    bool pending_stop;

    uint16_t ctl;
    uint32_t int_state, int_mask;
    bool irq_state;
    uint32_t clk, line_status;

    uint16_t master_read_address, master_read_count;
    uint16_t master_write_address, master_write_count;
    uint8_t master_read_data[16], master_write_data[16];
    size_t master_read_data_ridx, master_write_data_ridx;
    size_t master_read_data_widx, master_write_data_widx;
    size_t master_read_data_count, master_write_data_count;

    uint8_t time_tpl, time_tph, time_tp2s;
    uint16_t time_tbi, time_tsl, time_tdl;
    uint8_t time_tsdl, time_tsdh, time_tckh, time_tckl;
};

static void img_scb_irq_update(struct img_scb_state_s *scb)
{
    DBGLOG("SCB int state 0x%x mask 0x%x\n", scb->int_state, scb->int_mask);
    if (!scb->irq_state && (scb->int_state & scb->int_mask)) {
        DBGLOG("SCB IRQ raise\n");
        scb->irq_state = true;
        qemu_irq_raise(scb->irq);
    } else if (scb->irq_state && !(scb->int_state & scb->int_mask)) {
        DBGLOG("SCB IRQ lower\n");
        scb->irq_state = false;
        qemu_irq_lower(scb->irq);
    }
}

static void img_scb_int_raise(struct img_scb_state_s *scb, uint32_t int_val)
{
    scb->int_state |= int_val;
    img_scb_irq_update(scb);
}

static void img_scb_int_clear(struct img_scb_state_s *scb, uint32_t int_val)
{
    scb->int_state &= ~int_val;
    img_scb_irq_update(scb);
}

static bool img_scb_master_read_fifo_full(struct img_scb_state_s *scb)
{
    return scb->master_read_data_count == sizeof(scb->master_read_data);
}

static void img_scb_data_transfer(struct img_scb_state_s *scb)
{
#if DEBUG_LEVEL >= 3
    bool newline;
#endif

    DBGLOG("transfer state %d write_count=%d read_count=%d\n", scb->state, scb->master_write_count, scb->master_read_count);

    if (scb->state == STATE_IDLE && scb->master_write_count) {
        if (!i2c_start_transfer(scb->bus, scb->master_write_address, 0)) {
            DBGLOG("SCB write start, address 0x%x\n", scb->master_write_address);
            scb->state = STATE_WRITE;
            img_scb_int_clear(scb, INT_BUS_INACTIVE);

            scb->line_status |= LINE_START_BIT_DET;
            img_scb_int_raise(scb, INT_SLAVE_EVENT);
        } else {
            DBGLOG("SCB failed to start write transfer\n");
        }
    }
    if (scb->state == STATE_IDLE && scb->master_read_count) {
        if (!i2c_start_transfer(scb->bus, scb->master_read_address, 1)) {
            DBGLOG("SCB read start, address 0x%x\n", scb->master_read_address);
            scb->state = STATE_READ;
            img_scb_int_clear(scb, INT_BUS_INACTIVE);

            scb->line_status |= LINE_START_BIT_DET;
            img_scb_int_raise(scb, INT_SLAVE_EVENT);
        } else {
            DBGLOG("SCB failed to start read transfer\n");
        }
    }
    if (scb->state == STATE_IDLE) {
        img_scb_int_raise(scb, INT_BUS_INACTIVE);
        return;
    }

    if (scb->state == STATE_WRITE) {
#if DEBUG_LEVEL >= 3
        if (scb->master_write_count && scb->master_write_data_count) {
            DBGLOG("SCB >>");
            newline = true;
        }
#endif

        while (scb->master_write_count && scb->master_write_data_count) {
#if DEBUG_LEVEL >= 3
            DBGLOG(" 0x%02x", scb->master_write_data[scb->master_write_data_ridx]);
#endif
            i2c_send(scb->bus, scb->master_write_data[scb->master_write_data_ridx++]);
            scb->master_write_data_ridx %= sizeof(scb->master_write_data);
            scb->master_write_data_count--;
            scb->master_write_count--;
        }

#if DEBUG_LEVEL >= 3
        if (newline)
            DBGLOG("\n");
#endif

        if (!scb->master_write_count) {
            DBGLOG("SCB write done\n");
            i2c_end_transfer(scb->bus);
            scb->state = STATE_IDLE;
            scb->pending_stop = true;
            scb->line_status |= LINE_TRANS_DONE | LINE_STOP_BIT_DET;
            img_scb_int_raise(scb, INT_SLAVE_EVENT | INT_T_DONE | INT_MASTER_WRITE_FIFO_EMPTY);

            img_scb_data_transfer(scb);
            return;
        }

        if (!scb->master_write_data_count) {
            img_scb_int_raise(scb, INT_MASTER_WRITE_FIFO_EMPTY);
        }

        return;
    }
    
    if (scb->state == STATE_READ) {
#if DEBUG_LEVEL >= 3
        if (scb->master_read_count && !img_scb_master_read_fifo_full(scb)) {
            DBGLOG("SCB <<");
            newline = true;
        }
#endif

        while (scb->master_read_count && !img_scb_master_read_fifo_full(scb)) {
            scb->master_read_data[scb->master_read_data_widx++] = (uint8_t)i2c_recv(scb->bus);
#if DEBUG_LEVEL >= 3
            DBGLOG(" 0x%02x", scb->master_read_data[scb->master_read_data_widx - 1]);
#endif
            scb->master_read_data_widx %= sizeof(scb->master_read_data);
            scb->master_read_data_count++;
            scb->master_read_count--;
        }

#if DEBUG_LEVEL >= 3
        if (newline)
            DBGLOG("\n");
#endif

        if (!scb->master_read_count) {
            DBGLOG("SCB read done\n");
            i2c_end_transfer(scb->bus);
            scb->state = STATE_IDLE;
            scb->pending_stop = true;
            scb->line_status |= LINE_TRANS_DONE | LINE_STOP_BIT_DET;
            img_scb_int_raise(scb, INT_SLAVE_EVENT | INT_T_DONE | INT_MASTER_READ_FIFO_FILLING);

            img_scb_data_transfer(scb);
            return;
        }

        if (img_scb_master_read_fifo_full(scb)) {
            img_scb_int_raise(scb, INT_MASTER_READ_FIFO_FULL);
        } else if (scb->master_read_data_count) {
            img_scb_int_raise(scb, INT_MASTER_READ_FIFO_FILLING);
        }
    }
}

static uint64_t img_scb_io_read(void *opaque, hwaddr addr,
	                        unsigned size)
{
    struct img_scb_state_s *scb = opaque;
    uint32_t ret = 0;

    addr &= 0xff;

    switch (addr) {
    case SCB_LINE_STATUS:
        ret = scb->line_status | (scb->master_read_data[scb->master_read_data_ridx] << 24);
        break;
    case SCB_CLEAR_STATUS:
    case SCB_LINE_OVERRIDE:
        /* read makes no sense */
        ret = 0;
        break;

    case SCB_MASTER_READ_ADDRESS:
        ret = scb->master_read_address;
        break;
    case SCB_MASTER_READ_COUNT:
        ret = scb->master_read_count;
        break;
    case SCB_MASTER_READ_DATA:
        ret = scb->master_read_data[scb->master_read_data_ridx];
        break;
    case SCB_MASTER_DATA_READ:
        /* read makes no sense */
        break;

    case SCB_MASTER_WRITE_ADDRESS:
        ret = scb->master_write_address;
        break;
    case SCB_MASTER_WRITE_COUNT:
        ret = scb->master_write_count;
        break;
    case SCB_MASTER_WRITE_DATA:
        ret = scb->master_write_data[scb->master_write_data_ridx];
        break;

    case SCB_MASTER_FILL_STATUS:
        ret = 0;

        if (scb->master_read_data_count == sizeof(scb->master_read_data))
            ret |= (1 << 0);
        else if (!scb->master_read_data_count)
            ret |= (1 << 1);

        if (scb->master_write_data_count == sizeof(scb->master_write_data))
            ret |= (1 << 2);
        else if (!scb->master_write_data_count)
            ret |= (1 << 3);

        break;

    case SCB_CLK_SET:
        ret = scb->clk;
        break;

    case SCB_INTERRUPT_STATUS:
        ret = scb->int_state & scb->int_mask;
        break;
    case SCB_INTERRUPT_CLEAR:
        /* read makes no sense */
        ret = 0;
        break;
    case SCB_INTERRUPT_MASK:
        ret = scb->int_mask;
        break;

    case SCB_GENERAL_CONTROL:
        ret = scb->ctl;
        break;

    case SCB_TIME_TPL:
        ret = scb->time_tpl;
        break;
    case SCB_TIME_TPH:
        ret = scb->time_tph;
        break;
    case SCB_TIME_TP2S:
        ret = scb->time_tp2s;
        break;
    case SCB_TIME_TBI:
        ret = scb->time_tbi;
        break;
    case SCB_TIME_TSL:
        ret = scb->time_tsl;
        break;
    case SCB_TIME_TDL:
        ret = scb->time_tdl;
        break;
    case SCB_TIME_TSDL:
        ret = scb->time_tsdl;
        break;
    case SCB_TIME_TSDH:
        ret = scb->time_tsdh;
        break;
    case SCB_TIME_TCKH:
        ret = scb->time_tckh;
        break;
    case SCB_TIME_TCKL:
        ret = scb->time_tckl;
        break;

    case CR_SCB_CORE_REV:
        ret = 0x00020201; /* 0.2.2.1 as in comet */
        break;

    default:
        DBGLOG("unhandled ");
        ret = 0;
    }

    DBGLOG("scb read(0x%x) = 0x%08x\n", addr, ret);
    return ret;
}

static void img_scb_io_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    struct img_scb_state_s *scb = opaque;
    uint8_t cmd;

    addr &= 0xff;

    switch (addr) {
    case SCB_LINE_STATUS:
        /* write makes no sense */
        break;
    case SCB_CLEAR_STATUS:
        if (val & CLEAR_START_DET)
            scb->line_status &= ~LINE_START_BIT_DET;
        if (val & CLEAR_STOP_DET)
            scb->line_status &= ~LINE_STOP_BIT_DET;
        if (val & CLEAR_ACK_DET)
            scb->line_status &= ~LINE_ACK_DET;
        if (val & CLEAR_NACK_DET)
            scb->line_status &= ~LINE_NACK_DET;
        if (val & CLEAR_INPUT_HELD_VALID_DET)
            scb->line_status &= ~LINE_INPUT_HELD_VALID_DET;
        if (val & CLEAR_ABORT_DET)
            scb->line_status &= ~LINE_ABORT_DET;
        break;
    case SCB_LINE_OVERRIDE:
        if (val & OVERRIDE_LINES) {
            scb->line_status &= ~(LINE_SCLK | LINE_SCLK_EN | LINE_SDAT | LINE_SDAT_EN);
            if (val & OVERRIDE_SCLK)
                scb->line_status |= LINE_SCLK;
            if (val & OVERRIDE_SCLK_EN)
                scb->line_status |= LINE_SCLK_EN;
            if (val & OVERRIDE_SDAT)
                scb->line_status |= LINE_SDAT;
            if (val & OVERRIDE_SDAT_EN)
                scb->line_status |= LINE_SDAT_EN;
        }
        if (val & OVERRIDE_DIRECT) {
            cmd = (val >> OVERRIDE_CMD_SHIFT) & OVERRIDE_CMD_MASK;
            switch (cmd) {
            case OVERRIDE_CMD_GEN_DATA:
                DBGLOG("SCB generate data 0x%02x\n",
                       (val >> OVERRIDE_DATA_SHIFT) & OVERRIDE_DATA_MASK);
                scb->line_status |= LINE_INPUT_HELD_VALID_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
            case OVERRIDE_CMD_GEN_START:
                scb->line_status |= LINE_START_BIT_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
            case OVERRIDE_CMD_GEN_STOP:
                scb->line_status |= LINE_STOP_BIT_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
            case OVERRIDE_CMD_GEN_ACK:
                scb->line_status |= LINE_ACK_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
            case OVERRIDE_CMD_GEN_NACK:
                scb->line_status |= LINE_NACK_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
            case OVERRIDE_CMD_RETRIEVE_DATA:
                scb->line_status |= LINE_INPUT_HELD_VALID_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
            case OVERRIDE_CMD_RETRIEVE_ACK:
                scb->line_status |= LINE_ACK_DET;
                img_scb_int_raise(scb, INT_SLAVE_EVENT);
                break;
	    default:
                DBGLOG("SCB unhandled atomic command %d\n", cmd);
                break;
            }
            img_scb_int_raise(scb, INT_T_DONE);
        }
        break;

    case SCB_MASTER_READ_ADDRESS:
        scb->master_read_address = val & 0x3ff;
        break;
    case SCB_MASTER_READ_COUNT:
        scb->master_read_count += val & 0xffff;
        img_scb_data_transfer(scb);
        break;
    case SCB_MASTER_READ_DATA:
        /* write makes no sense */
        break;
    case SCB_MASTER_DATA_READ:
        scb->master_read_data_ridx++;
        scb->master_read_data_ridx %= sizeof(scb->master_read_data);
        scb->master_read_data_count--;
        img_scb_data_transfer(scb);
        break;

    case SCB_MASTER_WRITE_ADDRESS:
        scb->master_write_address = val & 0x3ff;
        break;
    case SCB_MASTER_WRITE_COUNT:
        scb->master_write_count += val & 0xffff;
        img_scb_data_transfer(scb);
        break;
    case SCB_MASTER_WRITE_DATA:
        scb->master_write_data[scb->master_write_data_widx++] = val;
        scb->master_write_data_widx %= sizeof(scb->master_write_data);
        scb->master_write_data_count++;
        img_scb_int_clear(scb, INT_MASTER_WRITE_FIFO_EMPTY);
        img_scb_data_transfer(scb);
        break;

    case SCB_MASTER_FILL_STATUS:
        /* write makes no sense */
        break;

    case SCB_CLK_SET:
        scb->clk = val;
        break;

    case SCB_INTERRUPT_STATUS:
        /* write makes no sense */
        break;
    case SCB_INTERRUPT_CLEAR:
        img_scb_int_clear(scb, val & 0x3ffff);

        if (scb->pending_stop) {
            scb->line_status |= LINE_STOP_BIT_DET;
            img_scb_int_raise(scb, INT_SLAVE_EVENT);
        }
        break;
    case SCB_INTERRUPT_MASK:
        scb->int_mask = val & 0x3ffff;
        img_scb_irq_update(scb);
        break;

    case SCB_GENERAL_CONTROL:
        scb->ctl = val & 0x3ff;
        if (val & CTL_TRANS_HALT)
            scb->line_status |= LINE_TRANS_HALTED;
        else
            scb->line_status &= ~LINE_TRANS_HALTED;
        break;

    case SCB_TIME_TPL:
        scb->time_tpl = val & 0xf;
        break;
    case SCB_TIME_TPH:
        scb->time_tph = val & 0xf;
        break;
    case SCB_TIME_TP2S:
        scb->time_tp2s = val & 0xf;
        break;
    case SCB_TIME_TBI:
        scb->time_tbi = val & 0xffff;
        break;
    case SCB_TIME_TSL:
        scb->time_tsl = val & 0xffff;
        break;
    case SCB_TIME_TDL:
        scb->time_tdl = val & 0xffff;
        break;
    case SCB_TIME_TSDL:
        scb->time_tsdl = val & 0xf;
        break;
    case SCB_TIME_TSDH:
        scb->time_tsdh = val & 0xf;
        break;
    case SCB_TIME_TCKH:
        scb->time_tckh = val & 0xf;
        break;
    case SCB_TIME_TCKL:
        scb->time_tckl = val & 0xf;
        break;

    case CR_SCB_CORE_REV:
        /* read only - writes ignored */
        break;

    default:
        DBGLOG("unhandled ");
    }

    DBGLOG("scb write(0x%x, 0x%08x)\n", addr, val);
}

static const MemoryRegionOps img_scb_io_ops = {
    .read = img_scb_io_read,
    .write = img_scb_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

struct img_scb_state_s *img_scb_init(hwaddr base, qemu_irq irq)
{
    struct img_scb_state_s *scb;

    scb = g_malloc0(sizeof(struct img_scb_state_s));

    memory_region_init_io(&scb->iomem, &img_scb_io_ops, scb,
                          "img-scb", IMG_SCB_SIZE);

    scb->irq = irq;
    scb->bus = i2c_init_bus(NULL, "i2c");

    return scb;
}

MemoryRegion *img_scb_iomem(struct img_scb_state_s *scb)
{
    return &scb->iomem;
}

i2c_bus *img_scb_bus(struct img_scb_state_s *scb)
{
    return scb->bus;
}

