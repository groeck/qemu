#include "qemu-common.h"
#include "qemu-log.h"
#include "console.h"
#include "hw.h"
#include "i2c.h"
#include "sii9022a.h"

#define DEBUG_LEVEL 0

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define ARRAY_LENGTH(a) (sizeof(a)/sizeof(a[0]))

enum {
    SII9022A_PXLCLK_LSB    = 0x00,
    SII9022A_PXLCLK_MSB    = 0x01,
    SII9022A_VFREQ_LSB     = 0x02,
    SII9022A_VFREQ_MSB     = 0x03,
    SII9022A_PIXELS_LSB    = 0x04,
    SII9022A_PIXELS_MSB    = 0x05,
    SII9022A_LINES_LSB     = 0x06,
    SII9022A_LINES_MSB     = 0x07,
    SII9022A_INPUT_FORMAT  = 0x09,
    SII9022A_OUTPUT_FORMAT = 0x0a,
    SII9022A_AVIINFO       = 0x0c,
    SII9022A_TPI_SYS_CTL   = 0x1a,
    SII9022A_DEV_ID        = 0x1b,
    SII9022A_REV_PROD_ID   = 0x1c,
    SII9022A_REV_TPI_ID    = 0x1d,
    SII9022A_POWER_STATE   = 0x1e,
    SII9022A_REV_HDCP      = 0x30,
    SII9022A_INT_ENABLE    = 0x3c,
    SII9022A_INT_STATUS    = 0x3d,
    SII9022A_SYNC_METHOD   = 0x60,
    SII9022A_SYNC_POLARITY = 0x61,
    SII9022A_INTERNAL_PAGE = 0xbc,
    SII9022A_INDEXED_REG   = 0xbd,
    SII9022A_IND_REG_VAL   = 0xbe,
    SII9022A_RESET         = 0xc7,
};

enum {
    INT_HOTPLUG            = (1 << 0),
    INT_RX_SENSE           = (1 << 1),
    INT_STAT_HOTPLUG       = (1 << 2),
    INT_STAT_RX_SENSE      = (1 << 3),
    INT_AUDIO_ERR          = (1 << 4),
    INT_SECURITY_STATUS    = (1 << 5),
    INT_HDCP_VALUE_READY   = (1 << 6),
    INT_HDCP_AUTH_STATUS   = (1 << 7),
};

typedef struct {
    I2CSlave i2c;
    int addr;
    qemu_irq irq;
    sii9022a_cb_reschange *reschange;
    void *cb_user;

    int reg_addr;
    bool just_set_addr;
    bool host_req_bus, host_has_bus;
    int internal_page;
    int indexed_reg;

    uint8_t input_format, output_format;
    uint16_t pxlclk, vfreq, pixels, lines;
    bool res_change;
    uint8_t int_status, int_enable;
    bool irq_state;
    uint8_t aviinfo[14];
} Sii9022aState;

static void sii9022a_update_irq(Sii9022aState *s)
{
    uint8_t low = s->int_status & s->int_enable;

    if (!low && !s->irq_state)
        qemu_irq_raise(s->irq);
    else if (low && s->irq_state)
        qemu_irq_lower(s->irq);

    s->irq_state = !low;
}

static void sii9022a_int(Sii9022aState *s, uint8_t i)
{
    s->irq_state |= i;
    sii9022a_update_irq(s);
}

static void sii9022a_reset(Sii9022aState *s)
{
    DBGLOG("sii9022a: reset\n");

    s->int_enable = 0;
    s->int_status = 0;
    s->pixels = 640;
    s->lines = 480;
    sii9022a_int(s, INT_HOTPLUG);
}

static void sii9022a_res_change(Sii9022aState *s)
{
    const int hsync_len = 96;
    const int left_margin = 48;
    const int right_margin = 16;
    const int vsync_len = 2;
    const int upper_margin = 33;
    const int lower_margin = 10;
    int w, h;

    w = s->pixels - (hsync_len + left_margin + right_margin);
    h = s->lines - (vsync_len + upper_margin + lower_margin);

    DBGLOG("sii9022a: res change %ux%u\n", w, h);

    if (s->reschange)
        s->reschange(w, h, s->cb_user);
}

static int sii9022a_read_internal_reg(Sii9022aState *s)
{
    switch (s->internal_page) {
    case 0x01:
        switch (s->indexed_reg) {
        case 0x82: /* used to enable source termination */
            return 0;
        }
    }

    DBGLOG("sii9022a: read internal reg 0x%02x:0x%02x\n", s->internal_page, s->indexed_reg);
    return 0;
}

static void sii9022a_write_internal_reg(Sii9022aState *s, int val)
{
    switch (s->internal_page) {
    case 0x01:
        switch (s->indexed_reg) {
        case 0x82: /* used to enable source termination */
            return;
        }
    }

    DBGLOG("sii9022a: write internal reg 0x%02x:0x%02x\n", s->internal_page, s->indexed_reg);
}

static int sii9022a_i2c_init(I2CSlave *i2c)
{
    Sii9022aState *s = FROM_I2C_SLAVE(Sii9022aState, i2c);

    s->reg_addr = -1;

    return 0;
}

static void sii9022a_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    Sii9022aState *s = FROM_I2C_SLAVE(Sii9022aState, i2c);

    switch (event) {
    case I2C_START_SEND:
        break;

    case I2C_START_RECV:
        break;

    case I2C_FINISH:
        if (!s->just_set_addr)
            s->reg_addr = -1;
        s->just_set_addr = false;
        if (s->res_change)
            sii9022a_res_change(s);
        s->res_change = false;
        break;

    case I2C_NACK:
        DBGLOG("sii9022a: I2C NACK\n");
        break;
    }
}

static int sii9022a_i2c_rx(I2CSlave *i2c)
{
    Sii9022aState *s = FROM_I2C_SLAVE(Sii9022aState, i2c);
    int ret;

    if (s->reg_addr == -1) {
        DBGLOG("sii9022a: unexpected read\n");
        return 0x00;
    }

    switch (s->reg_addr) {
    case SII9022A_PXLCLK_LSB:
        ret = (s->pxlclk >> 0) & 0xff;
        break;
    case SII9022A_PXLCLK_MSB:
        ret = (s->pxlclk >> 8) & 0xff;
        break;
    case SII9022A_VFREQ_LSB:
        ret = (s->vfreq >> 0) & 0xff;
        break;
    case SII9022A_VFREQ_MSB:
        ret = (s->vfreq >> 8) & 0xff;
        break;
    case SII9022A_PIXELS_LSB:
        ret = (s->pixels >> 0) & 0xff;
        break;
    case SII9022A_PIXELS_MSB:
        ret = (s->pixels >> 8) & 0xff;
        break;
    case SII9022A_LINES_LSB:
        ret = (s->lines >> 0) & 0xff;
        break;
    case SII9022A_LINES_MSB:
        ret = (s->lines >> 8) & 0xff;
        break;
    case SII9022A_INPUT_FORMAT:
        ret = s->input_format;
        break;
    case SII9022A_OUTPUT_FORMAT:
        ret = s->output_format;
        break;
    case SII9022A_AVIINFO ... (SII9022A_AVIINFO+13):
        ret = s->aviinfo[s->reg_addr - SII9022A_AVIINFO];
        break;
    case SII9022A_TPI_SYS_CTL:
        ret = (!!s->host_req_bus << 2) |
              (!!s->host_has_bus << 1);
        break;
    case SII9022A_DEV_ID:
        ret = 0xb0;
        break;
    case SII9022A_REV_PROD_ID:
        ret = 0x02;
        break;
    case SII9022A_REV_TPI_ID:
        ret = 0x03;
        break;
    case SII9022A_REV_HDCP:
        ret = 0x00;
        break;
    case SII9022A_INT_ENABLE:
        ret = s->int_enable;
        break;
    case SII9022A_INT_STATUS:
        ret = s->int_status;
        break;
    case SII9022A_SYNC_POLARITY:
        ret = 0x00;
        break;
    case SII9022A_IND_REG_VAL:
        ret = sii9022a_read_internal_reg(s);
        break;
    default:
        DBGLOG("sii9022a: unhandled read from register 0x%02x\n", s->reg_addr);
        ret = 0x00;
    }

    s->reg_addr++;
    return ret;
}

static int sii9022a_i2c_tx(I2CSlave *i2c, uint8_t data)
{
    Sii9022aState *s = FROM_I2C_SLAVE(Sii9022aState, i2c);

    if (s->reg_addr == -1) {
        s->reg_addr = data;
        s->just_set_addr = true;
        return 0;
    }

    s->just_set_addr = false;

    switch (s->reg_addr) {
    case SII9022A_PXLCLK_LSB:
        s->pxlclk &= 0xff00;
        s->pxlclk |= data;
        break;
    case SII9022A_PXLCLK_MSB:
        s->pxlclk &= 0xff;
        s->pxlclk |= data << 8;
        break;
    case SII9022A_VFREQ_LSB:
        s->vfreq &= 0xff00;
        s->vfreq |= data;
        break;
    case SII9022A_VFREQ_MSB:
        s->vfreq &= 0xff;
        s->vfreq |= data << 8;
        break;
    case SII9022A_PIXELS_LSB:
        s->pixels &= 0xff00;
        s->pixels |= data;
        break;
    case SII9022A_PIXELS_MSB:
        s->pixels &= 0xff;
        s->pixels |= data << 8;
        break;
    case SII9022A_LINES_LSB:
        s->lines &= 0xff00;
        s->lines |= data;
        s->res_change = true;
        break;
    case SII9022A_LINES_MSB:
        s->lines &= 0xff;
        s->lines |= data << 8;
        s->res_change = true;
        break;
    case SII9022A_INPUT_FORMAT:
        s->input_format = data;
        break;
    case SII9022A_OUTPUT_FORMAT:
        s->output_format = data;
        break;
    case SII9022A_AVIINFO ... (SII9022A_AVIINFO+13):
        s->aviinfo[s->reg_addr - SII9022A_AVIINFO] = data;
        break;
    case SII9022A_TPI_SYS_CTL:
        s->host_req_bus = !!(data & (1 << 2));
        s->host_has_bus = !!(data & (1 << 2));
        s->host_has_bus |= s->host_req_bus;
        break;
    case SII9022A_RESET:
        if (data)
            DBGLOG("sii9022a: unexpected reset write 0x%x\n", data);
        sii9022a_reset(s);
        break;
    case SII9022A_INT_ENABLE:
        s->int_enable = data;
        sii9022a_update_irq(s);
        break;
    case SII9022A_INT_STATUS:
        s->int_status &= ~data;
        sii9022a_update_irq(s);
        break;
    case SII9022A_INTERNAL_PAGE:
        s->internal_page = data;
        break;
    case SII9022A_INDEXED_REG:
        s->indexed_reg = data;
        break;
    case SII9022A_IND_REG_VAL:
        sii9022a_write_internal_reg(s, data);
        break;
    case SII9022A_POWER_STATE:
    case SII9022A_SYNC_METHOD:
        break;
    default:
        DBGLOG("sii9022a: unhandled write to register 0x%02x\n", s->reg_addr);
    }

    s->reg_addr++;
    return 0;
}

void sii9022a_setup(DeviceState *dev, qemu_irq irq, sii9022a_cb_reschange *reschange, void *user)
{
    Sii9022aState *s = FROM_I2C_SLAVE(Sii9022aState, I2C_SLAVE_FROM_QDEV(dev));

    s->irq = irq;
    s->reschange = reschange;
    s->cb_user = user;
    sii9022a_update_irq(s);
}

static void sii9022a_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->init = sii9022a_i2c_init;
    sc->event = sii9022a_i2c_event;
    sc->recv = sii9022a_i2c_rx;
    sc->send = sii9022a_i2c_tx;
}

static TypeInfo sii9022a_info = {
    .name          = "sii9022a",
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Sii9022aState),
    .class_init    = sii9022a_class_init,
};

static void sii9022a_register_types(void)
{
    type_register_static(&sii9022a_info);
}

type_init(sii9022a_register_types)
