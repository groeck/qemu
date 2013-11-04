#include "qemu-common.h"
#include "qemu-log.h"
#include "console.h"
#include "hw.h"
#include "i2c.h"
#include "qt5480.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define ADDR_INVALID (-1)
#define IDX_INVALID 0xffff

#define ARRAY_LENGTH(a) (sizeof(a)/sizeof(a[0]))

typedef struct {
    I2CSlave i2c;
    int addr;
    bool trans_write;
    uint8_t mem[1<<10];

    qemu_irq nchange;
    bool nchange_state;

    uint8_t write_data[32];
    size_t write_data_len, write_data_idx;

    uint8_t read_addr;
    size_t read_idx;

    uint16_t change_queue[8];
    size_t change_queue_ridx, change_queue_widx, change_queue_count;

    int btns;
    size_t xres, yres;
} QT5480State;

typedef enum {
    REG_CHIP_ID               = 0x00,
    REG_CODE_VERSION          = 0x01,
    REG_CALIBRATE             = 0x02,
    REG_RESET                 = 0x03,
    REG_EEPROM_BACKUP_REQUEST = 0x04,
    REG_READ_POINTER          = 0x05,
    REG_EEPROM_CHK_LSB        = 0x06,
    REG_EEPROM_CHK_MSB        = 0x07,
    REG_KEY_STATUS_0          = 0x08,
    REG_KEY_STATUS_1          = 0x09,
    REG_KEY_STATUS_2          = 0x0a,
    REG_KEY_STATUS_3          = 0x0b,
    REG_KEY_STATUS_4          = 0x0c,
    REG_KEY_STATUS_5          = 0x0d,
    REG_GEN_STATUS_1          = 0x0e,
    REG_GEN_STATUS_2          = 0x0f,
    REG_TOUCH0_X_MSB          = 0x10,
    REG_TOUCH0_X_LSB          = 0x11,
    REG_TOUCH0_Y_MSB          = 0x12,
    REG_TOUCH0_Y_LSB          = 0x13,
    REG_TOUCH1_X_MSB          = 0x14,
    REG_TOUCH1_X_LSB          = 0x15,
    REG_TOUCH1_Y_MSB          = 0x16,
    REG_TOUCH1_Y_LSB          = 0x17,
    REG_SLIDER0_POS           = REG_TOUCH1_X_MSB,
    REG_SLIDER1_POS           = REG_TOUCH1_X_LSB,
    REG_SLIDER2_POS           = REG_TOUCH1_Y_MSB,
    REG_SLIDER3_POS           = REG_TOUCH1_Y_LSB,
    REG_SLIDER4_POS           = 0x18,
    REG_SLIDER5_POS           = 0x19,
    REG_FORCE_SENSOR          = 0x1a,
    REG_CHAN_GATING_STATUS    = 0x1b,
    REG_TOUCH0_GESTURE_0      = 0x1c,
    REG_TOUCH0_GESTURE_1      = 0x1d,
    REG_TOUCH0_GESTURE_2      = 0x1e,
    REG_TOUCH0_GESTURE_3      = 0x1f,
    REG_TOUCH1_GESTURE_0      = 0x20,
    REG_TOUCH1_GESTURE_1      = 0x21,
    REG_TOUCH1_GESTURE_2      = 0x22,
    REG_TOUCH1_GESTURE_3      = 0x23,
    /* 0x24 - 0xff reserved */
    REG_CHAN0_DELTA_LSB       = 0x100,
    REG_CHAN0_DELTA_MSB       = 0x101,
    /* repeat delta LSB/MSB for channels 1..47 */
    REG_CHAN0_REF_LSB         = 0x160,
    REG_CHAN0_REF_MSB         = 0x160,
    /* repeat ref LSB/MSB for channels 1..47 */
    REG_CHAN0_CTL             = 0x200,
    /* repeat ctl for channels 1..47 */
    REG_CHAN0_NTHR            = 0x230,
    /* repeat nthr for channels 1..47 */
    REG_CHAN0_BURST_LEN       = 0x260,
    /* repeat burst len for channels 1..47 */
    REG_LP_MODE               = 0x290,
    REG_MIN_CYCLE_TIME        = 0x291,
    REG_AWAKE_TIMEOUT         = 0x292,
    REG_TRIGGER_CTL           = 0x293,
    REG_GUARD_CHAN_EN         = 0x294,
    REG_TS_SETUP              = 0x295,
    REG_TS_LEN                = 0x296,
    REG_SLIDER0_SETUP         = REG_TS_LEN,
    REG_SLIDER1_SETUP         = 0x297,
    REG_SLIDER2_SETUP         = 0x298,
    REG_SLIDER3_SETUP         = 0x299,
    REG_SLIDER4_SETUP         = 0x29a,
    REG_SLIDER5_SETUP         = 0x29b,
    REG_HYSTERESIS_0          = 0x29c,
    REG_HYSTERESIS_1          = 0x29d,
    REG_HYSTERESIS_2          = 0x29e,
    REG_HYSTERESIS_3          = 0x29f,
    REG_HYSTERESIS_4          = 0x2a0,
    REG_HYSTERESIS_5          = 0x2a1,
    REG_GPO_CTL               = 0x2a2,
    REG_NDRIFT                = 0x2a3,
    REG_PDRIFT                = 0x2a4,
    REG_NDIL                  = 0x2a5,
    REG_SDIL                  = 0x2a6,
    REG_NRECAL_DELAY          = 0x2a7,
    REG_DRIFT_HOLD_TIME       = 0x2a8,
    REG_FORCE_SENSOR_THRESH   = 0x2a9,
    REG_POS_CLIP_0            = 0x2aa,
    REG_POS_CLIP_1            = 0x2ab,
    REG_LINTBL_XOFF_LSB       = 0x2ac,
    REG_LINTBL_XOFF_MSB       = 0x2ad,
    REG_LINTBL_XSEG_1         = 0x2ae,
    /* repeat xseg for 2-16 */
    REG_LINTBL_YOFF_LSB       = 0x2be,
    REG_LINTBL_YOFF_MSB       = 0x2bf,
    REG_LINTBL_YSEG_1         = 0x2c0,
    /* repeat yseg for 2-16 */
    REG_BURST_CTL             = 0x2d0,
    REG_STATUS_MASK           = 0x2d1,
    REG_POS_FILTER_CTL        = 0x2d2,
    REG_TS_RES_CTL            = 0x2d3,
    REG_TS_PLATEAU_CTL        = 0x2d4,
    REG_SLEW_RATE_FILTER_CTL  = 0x2d5,
    REG_MEDIAN_FILTER_LEN     = 0x2d6,
    REG_IIR_FILTER_CTL        = 0x2d7,
    REG_TDOWN_HYSTERESIS      = 0x2d8,
    REG_GESTURE_CFG_0         = 0x2d9,
} QT5480Register;

static void qt5480_reset(QT5480State *s);
static void qt5480_enqueue_change(QT5480State *s, uint16_t addr, bool coalesce);

static void qt5480_write(QT5480State *s, uint16_t addr, uint8_t *data, size_t sz)
{
    if (addr + sz >= sizeof(s->mem)) {
        DBGLOG("qt5480 out of range write to address 0x%04x\n", addr);
        return;
    }

    DBGLOG("qt5480 write 0x%04x:%d = 0x%02x\n", addr, sz, data[0]);
    memcpy(&s->mem[addr], data, sz);

#define affected(x) ((addr <= x) && ((addr + sz) > x))

    if (affected(REG_RESET) && s->mem[REG_RESET]) {
        qt5480_reset(s);
        return;
    }

    if (affected(REG_READ_POINTER)) {
        qt5480_enqueue_change(s, s->mem[REG_READ_POINTER] << 2, false);
    }

#undef affected
}

static void qt5480_set_change(QT5480State *s, bool change)
{
    if (s->nchange_state != !change) {
        s->nchange_state = !change;
        qemu_set_irq(s->nchange, !change);
    }
}

static void qt5480_report_change(QT5480State *s)
{
    if (!s->change_queue_count) {
        qt5480_set_change(s, false);
        return;
    }
    if (!s->nchange_state) {
        /* wait for this one to be read first */
        return;
    }

    qt5480_set_change(s, true);
}

static void qt5480_enqueue_change(QT5480State *s, uint16_t addr, bool coalesce)
{
    if (coalesce && s->change_queue_count &&
        s->change_queue[s->change_queue_ridx] == addr) {
        /* the last change report is for this same address */
        return;
    }

    if (s->change_queue_count >= sizeof(s->change_queue)) {
        /* queue is full - drop */
        return;
    }

    s->change_queue[s->change_queue_widx++] = addr;
    s->change_queue_widx %= ARRAY_LENGTH(s->change_queue);
    s->change_queue_count++;

    qt5480_report_change(s);
}

static void qt5480_reset(QT5480State *s)
{
    DBGLOG("qt5480 reset\n");
    memset(s->mem, 0, sizeof(s->mem));
    s->change_queue_count = 0;
    s->change_queue_ridx = s->change_queue_widx = 0;

    s->mem[REG_CHIP_ID] = 0x40;
    s->mem[REG_CODE_VERSION] = 0x50;
    s->mem[REG_GEN_STATUS_1] = 0x80; /* RESET */

    qt5480_set_change(s, true);
}

static int qt5480_i2c_init(I2CSlave *i2c)
{
    QT5480State *s = FROM_I2C_SLAVE(QT5480State, i2c);

    s->addr = ADDR_INVALID;
    s->xres = 2048;
    s->yres = 2048;

    return 0;
}

static void qt5480_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    QT5480State *s = FROM_I2C_SLAVE(QT5480State, i2c);
    uint16_t addr;

    switch (event) {
    case I2C_START_SEND:
        s->trans_write = true;
        if (s->mem[REG_GEN_STATUS_1] & 0x80) {
            /* leave reset */
            s->mem[REG_GEN_STATUS_1] &= ~0x80;
        }
        s->write_data_idx = 0;
        qt5480_set_change(s, false);
        break;

    case I2C_START_RECV:
        s->trans_write = false;
        if (s->mem[REG_GEN_STATUS_1] & 0x80) {
            /* reset mode */
            s->read_idx = IDX_INVALID;
        } else if (s->nchange_state) {
            s->read_idx = IDX_INVALID;
            break;
        } else if (s->change_queue_count) {
            s->read_addr = s->change_queue[s->change_queue_ridx];
            s->read_idx = 0;
        } else {
            DBGLOG("qt5480 unexpected read\n");
            s->read_idx = IDX_INVALID;
        }
        qt5480_set_change(s, false);
        break;

    case I2C_FINISH:
        if (s->trans_write) {
            if (s->write_data_len < 3) {
                DBGLOG("qt5480 invalid write of length %d\n", s->write_data_len);
                break;
            }
            addr = (s->write_data[1] << 8) | s->write_data[0];
            qt5480_write(s, addr, s->write_data + 2, s->write_data_len - 2);
            s->write_data_len = 0;
        } else {
            if (!s->change_queue_count) {
                DBGLOG("qt5480 invalid read\n");
                break;
            }
            /* a data packet was read */
            /* advance the read index */
            s->change_queue_ridx++;
            s->change_queue_ridx %= ARRAY_LENGTH(s->change_queue);
            s->change_queue_count--;
        }
        qt5480_report_change(s);
        break;

    case I2C_NACK:
        DBGLOG("qt5480 I2C NACK\n");
        break;
    }
}

static int qt5480_i2c_rx(I2CSlave *i2c)
{
    QT5480State *s = FROM_I2C_SLAVE(QT5480State, i2c);
    if (s->read_idx == IDX_INVALID) {
        return 0xff;
    }
    if (!s->read_idx) {
        s->read_idx++;
        return s->read_addr >> 2;
    }
    return s->mem[s->read_addr + s->read_idx++ - 1];
}

static int qt5480_i2c_tx(I2CSlave *i2c, uint8_t data)
{
    QT5480State *s = FROM_I2C_SLAVE(QT5480State, i2c);
    if (s->write_data_idx >= sizeof(s->write_data)) {
        DBGLOG("qt5480 write overflow!\n");
        return 1;
    }
    s->write_data[s->write_data_idx++] = data;
    s->write_data_len++;
    return 0;
}

static void qt5480_touchscreen_event(void *opaque,
                int x, int y, int z, int buttons_state)
{
    QT5480State *s = (QT5480State *)opaque;
    uint16_t xpos, ypos;
    uint8_t size, area;

    xpos = (uint16_t)(((uint64_t)x * s->xres) / 32768);
    ypos = (uint16_t)(((uint64_t)y * s->yres) / 32768);
    xpos &= ((1 << 10) - 1);
    ypos &= ((1 << 10) - 1);
    size = 1;
    area = 1;

    s->mem[REG_TOUCH0_X_MSB] = xpos >> 2;
    s->mem[REG_TOUCH0_X_LSB] = ((xpos & 0x3) << 6) | size;
    s->mem[REG_TOUCH0_Y_MSB] = ypos >> 2;
    s->mem[REG_TOUCH0_Y_LSB] = ((ypos & 0x3) << 6) | area;

    if (s->btns ^ buttons_state) {
        /* update TS0DET */
        if (buttons_state)
            s->mem[REG_GEN_STATUS_2] |= 0x01;
        else
            s->mem[REG_GEN_STATUS_2] &= ~0x01;

        /* update TSCRDET */
        if (s->mem[REG_GEN_STATUS_2] & 0x03)
            s->mem[REG_GEN_STATUS_2] |= 0x80;
        else
            s->mem[REG_GEN_STATUS_2] &= ~0x80;

        s->btns = buttons_state;
        qt5480_enqueue_change(s, REG_KEY_STATUS_4, true);
    }

    if (s->btns) {
        qt5480_enqueue_change(s, REG_TOUCH0_X_MSB, true);
    }
}

void qt5480_setup(DeviceState *dev, size_t xres, size_t yres, qemu_irq nchange)
{
    QT5480State *s = FROM_I2C_SLAVE(QT5480State, I2C_SLAVE_FROM_QDEV(dev));

    s->xres = xres;
    s->yres = yres;
    s->nchange = nchange;

    qemu_add_mouse_event_handler(qt5480_touchscreen_event, s, 1,
                    "QEMU QT5480-driven Touchscreen");
}

static void qt5480_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->init = qt5480_i2c_init;
    sc->event = qt5480_i2c_event;
    sc->recv = qt5480_i2c_rx;
    sc->send = qt5480_i2c_tx;
}

static TypeInfo qt5480_info = {
    .name          = "qt5480",
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(QT5480State),
    .class_init    = qt5480_class_init,
};

static void qt5480_register_types(void)
{
    type_register_static(&qt5480_info);
}

type_init(qt5480_register_types)
