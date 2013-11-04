/*
 * Imagination Technologies PDP in Toumaz XENIF SoC (Comet).
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 *
 * This code is licenced under the LGPL
 *
 * IMG Powerdown Controller in XENIF (Comet).
 */

#include "comet_pdc.h"
#include "hw.h"
#include "qemu-log.h"
#include "sysemu.h"
#include "console.h"
#include "memory.h"

#define DEBUG_LEVEL 1

#if DEBUG_LEVEL >= 2
#  define DBGLOG(...) fprintf(stderr, ## __VA_ARGS__)
#elif DEBUG_LEVEL >= 1
#  define DBGLOG(...) qemu_log(__VA_ARGS__)
#else
#  define DBGLOG(...) do { } while (0)
#endif

#define MAX_SYSWAKES 4

enum {
    /* Watch dog timer */
    PDC_WD_SW_RESET      = 0x000,
    /* Real time clock */
    PDC_RTC_CONTROL      = 0x100,
    PDC_RTC_SEC          = 0x104,
    PDC_RTC_MIN          = 0x108,
    PDC_RTC_HOUR         = 0x10c,
    PDC_RTC_DAY          = 0x110,
    PDC_RTC_MON          = 0x114,
    PDC_RTC_YEAR         = 0x118,
    PDC_RTC_ASEC         = 0x11c,
    PDC_RTC_AMIN         = 0x120,
    PDC_RTC_AHOUR        = 0x124,
    PDC_RTC_ADAY         = 0x128,
    PDC_RTC_AMON         = 0x12c,
    PDC_RTC_AYEAR        = 0x130,
    PDC_RTC_IRQ_STATUS   = 0x134,
    PDC_RTC_IRQ_CLEAR    = 0x138,
    PDC_RTC_IRQ_EN       = 0x13c,
    /* GPIOs */
    PDC_IRQ_STATUS       = 0x310,
    PDC_IRQ_ENABLE       = 0x314,
    PDC_IRQ_CLEAR        = 0x318,
    PDC_IRQ_ROUTE        = 0x31c,
    PDC_SYS_WAKE0        = 0x330,
    PDC_SYS_WAKE0_CONFIG = 0x334,
    /* SoC GPIOs */
    PDC_SOC_GPIO_STATUS  = 0x580,
};

#define PDC_RTC_EPOCH           2000

#define PDC_RTC_CONTROL_GAE_M    (1 << 3)
#define PDC_RTC_CONTROL_FAST_M   (1 << 2) /* not supported */
#define PDC_RTC_CONTROL_UPDATE_M (1 << 1)
#define PDC_RTC_CONTROL_CE_M     (1 << 0)
#define PDC_RTC_CONTROL_M        0x9

#define PDC_RTC_SEC_B           6
#define PDC_RTC_SEC_M           0x3f
#define PDC_RTC_MIN_B           6
#define PDC_RTC_MIN_M           0x3f
#define PDC_RTC_HOUR_B          5
#define PDC_RTC_HOUR_M          0x1f
#define PDC_RTC_DAY_B           5
#define PDC_RTC_DAY_M           0x1f
#define PDC_RTC_MON_B           4
#define PDC_RTC_MON_M           0xf
#define PDC_RTC_YEAR_B          7
#define PDC_RTC_YEAR_M          0x7f

#define PDC_RTC_IRQ_ALARM_M     (1 << 2)
#define PDC_RTC_IRQ_MIN_M       (1 << 1)
#define PDC_RTC_IRQ_SEC_M       (1 << 0)
#define PDC_RTC_IRQ_M           0x7

#define PDC_IRQ_ROUTE_M         0x0f070f07

#define SYS_WAKE_STRIDE 0x8

enum {
    SW_IRQ_LEVEL_LOW  = 0x0,
    SW_IRQ_LEVEL_HIGH = 0x1,
    SW_IRQ_EDGE_LOW   = 0x2,
    SW_IRQ_EDGE_HIGH  = 0x3,
    SW_IRQ_NONE0      = 0x4,
    SW_IRQ_NONE1      = 0x5,
    SW_IRQ_EDGE0      = 0x6,
    SW_IRQ_EDGE1      = 0x7,
};

struct pdc_rtc_time_s {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t mon;
    uint8_t year;
};

struct pdc_rtc_s {
    /* rtc irq to PDC */
    int irq_routed;
    qemu_irq irq;
    /* rtc clock source */
    QEMUTimer *clk;
    uint64_t tick;

    /* register values */
    uint8_t control;
    uint8_t irq_status;
    uint8_t irq_clear;
    uint8_t irq_en;
    /* currently tracked time */
    struct pdc_rtc_time_s time;
    /* last set time values */
    struct pdc_rtc_time_s update;
    /* alarm enable bits and values */
    uint8_t alarm_en_bits;
    struct pdc_rtc_time_s alarm;
};

/* alarm_en_bits */
#define PDC_RTC_AEN_SEC_M  (1 << 0)
#define PDC_RTC_AEN_MIN_M  (1 << 1)
#define PDC_RTC_AEN_HOUR_M (1 << 2)
#define PDC_RTC_AEN_DAY_M  (1 << 3)
#define PDC_RTC_AEN_MON_M  (1 << 4)
#define PDC_RTC_AEN_YEAR_M (1 << 5)

enum {
    PDC_INIRQ_RTC,
    PDC_INIRQ_IR,
    PDC_INIRQ_WD,
    PDC_INIRQ_MAX
};

struct comet_pdc_state_s {
    MemoryRegion iomem;
    struct pdc_rtc_s rtc;

    qemu_irq irq;
    uint8_t num_syswakes;

    uint32_t irq_status;
    uint32_t irq_enable;
    uint32_t irq_edge_mask;
    uint32_t irq_route;

    uint8_t syswake_state;
    uint8_t syswake_int_modes[MAX_SYSWAKES];
};


static void pdc_rtc_update_irq(struct pdc_rtc_s *rtc)
{
    if (rtc->irq_routed) {
        qemu_irq_lower(rtc->irq);
    } else if (rtc->irq_status & rtc->irq_en) {
        qemu_irq_raise(rtc->irq);
    } else {
        qemu_irq_lower(rtc->irq);
    }
}

/* Increment the clock by 1 second */
static void pdc_rtc_inc_clock(struct pdc_rtc_s *rtc)
{
    /* leap years are handled specially */
    static uint8_t days_per_month[12] = {
    /* Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec */
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    /* next second */
    rtc->irq_status |= rtc->irq_en & PDC_RTC_IRQ_SEC_M;
    ++rtc->time.sec;
    if (rtc->time.sec < 60) {
        return;
    }
    /* next minute */
    rtc->irq_status |= rtc->irq_en & PDC_RTC_IRQ_MIN_M;
    rtc->time.sec = 0;
    ++rtc->time.min;
    if (rtc->time.min < 60) {
        return;
    }
    /* next hour */
    rtc->time.min = 0;
    ++rtc->time.hour;
    if (rtc->time.min < 24) {
        return;
    }
    /* next day */
    rtc->time.hour = 0;
    ++rtc->time.day;
    if (!(rtc->time.year & 0x3)) {
        /* leap year */
        if (rtc->time.day <= 29) {
            return;
        }
    } else if (rtc->time.mon >= 1 && rtc->time.mon <= 12) {
        if (rtc->time.day <= days_per_month[rtc->time.mon-1]) {
            return;
        }
    }
    /* next month */
    rtc->time.day = 1;
    ++rtc->time.mon;
    if (rtc->time.mon <= 12) {
        return;
    }
    /* next year */
    rtc->time.mon = 0;
    ++rtc->time.year;
    /* wrap the year */
    rtc->time.year &= (1 << PDC_RTC_YEAR_B) - 1;
}

/* Check whether the alarm time matches the clock, and emit an interrupt */
static void pdc_rtc_check_alarm(struct pdc_rtc_s *rtc)
{
    /* is the alarm even enabled and unmasked */
    if (!(rtc->control & PDC_RTC_CONTROL_GAE_M)) {
        return;
    }
    if (!(rtc->irq_en & PDC_RTC_IRQ_ALARM_M)) {
        return;
    }
    /* compare the requested alarm fields against the new time */
    if (rtc->alarm_en_bits & PDC_RTC_AEN_SEC_M
            && rtc->time.sec != rtc->alarm.sec) {
        return;
    }
    if (rtc->alarm_en_bits & PDC_RTC_AEN_MIN_M
            && rtc->time.min != rtc->alarm.min) {
        return;
    }
    if (rtc->alarm_en_bits & PDC_RTC_AEN_HOUR_M
            && rtc->time.hour != rtc->alarm.hour) {
        return;
    }
    if (rtc->alarm_en_bits & PDC_RTC_AEN_DAY_M
            && rtc->time.day != rtc->alarm.day) {
        return;
    }
    if (rtc->alarm_en_bits & PDC_RTC_AEN_MON_M
            && rtc->time.mon != rtc->alarm.mon) {
        return;
    }
    if (rtc->alarm_en_bits & PDC_RTC_AEN_YEAR_M
            && rtc->time.year != rtc->alarm.year) {
        return;
    }
    /* if they all match, emit an alarm interrupt */
    rtc->irq_status |= PDC_RTC_IRQ_ALARM_M;
    pdc_rtc_update_irq(rtc);
}

/* A tick of the clock every second */
static void pdc_rtc_tick(void *opaque)
{
    struct pdc_rtc_s *rtc = opaque;

    /* increment the clock */
    pdc_rtc_inc_clock(rtc);

    /* check for an alarm match */
    pdc_rtc_check_alarm(rtc);

    /* tick again 1 second later */
    rtc->tick += 1000;
    qemu_mod_timer(rtc->clk, rtc->tick);
}

static void pdc_rtc_enable(struct pdc_rtc_s *rtc)
{
    rtc->tick = qemu_get_clock_ms(rt_clock) + 1000;
    qemu_mod_timer(rtc->clk, rtc->tick);
}

static void pdc_rtc_disable(struct pdc_rtc_s *rtc)
{
    qemu_del_timer(rtc->clk);
}

static void pdc_rtc_init(struct pdc_rtc_s *rtc, qemu_irq irq)
{
    struct tm tm;

    rtc->irq = irq;

    /* set time to the current rtc time (see -rtc option) */
    qemu_get_timedate(&tm, 0);
    rtc->time.sec = tm.tm_sec;
    rtc->time.min = tm.tm_min;
    rtc->time.hour = tm.tm_hour;
    rtc->time.day = tm.tm_mday;
    rtc->time.mon = tm.tm_mon + 1;
    rtc->time.year = tm.tm_year + (1900 - PDC_RTC_EPOCH);
    /* initialise update time registers */
    rtc->update.sec = 0;
    rtc->update.min = 0;
    rtc->update.hour = 0;
    rtc->update.day = 1;
    rtc->update.mon = 1;
    rtc->update.year = 0;
    /* initialise alarm time registers */
    rtc->alarm_en_bits = 0;
    rtc->alarm.sec = 0;
    rtc->alarm.min = 0;
    rtc->alarm.hour = 0;
    rtc->alarm.day = 1;
    rtc->alarm.mon = 1;
    rtc->alarm.year = 0;
    /* initialise irq registers */
    rtc->irq_status = 0;
    rtc->irq_clear = 0;
    rtc->irq_en = 0;
    /* set the clock ticking */
    rtc->clk = qemu_new_timer_ms(rt_clock, pdc_rtc_tick, rtc);
    rtc->control = PDC_RTC_CONTROL_CE_M;
    pdc_rtc_enable(rtc);
}

static void comet_pdc_update_irq(struct comet_pdc_state_s *pdc)
{
    if (pdc->irq_status & pdc->irq_enable) {
        qemu_irq_raise(pdc->irq);
    } else {
        qemu_irq_lower(pdc->irq);
    }
}

static uint64_t comet_pdc_io_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    struct comet_pdc_state_s *pdc = opaque;
    uint32_t syswake, ret = 0;
    struct pdc_rtc_time_s *visible_time;

    addr &= -4;

    if (addr >= PDC_SYS_WAKE0 &&
        addr < (PDC_SYS_WAKE0_CONFIG + pdc->num_syswakes * SYS_WAKE_STRIDE)) {
        syswake = (addr - PDC_SYS_WAKE0) / SYS_WAKE_STRIDE;

        if (addr & 0x4) {
            /* SYS_WAKEn_CONFIG */
            DBGLOG("SYS_WAKE%d_CONFIG read\n", syswake);
        } else {
            /* SYS_WAKEn*/
            ret = (pdc->syswake_int_modes[syswake] << 1) |
                  !!(pdc->syswake_state & (1 << syswake));
        }

        return ret;
    }

    if (pdc->rtc.control & PDC_RTC_CONTROL_CE_M) {
        visible_time = &pdc->rtc.time;
    } else {
        visible_time = &pdc->rtc.update;
    }

    switch (addr) {
    /* Real time clock */
    case PDC_RTC_CONTROL:
        ret = pdc->rtc.control;
        break;
    case PDC_RTC_SEC:
        ret = visible_time->sec;
        break;
    case PDC_RTC_MIN:
        ret = visible_time->min;
        break;
    case PDC_RTC_HOUR:
        ret = visible_time->hour;
        break;
    case PDC_RTC_DAY:
        ret = visible_time->day;
        break;
    case PDC_RTC_MON:
        ret = visible_time->mon;
        break;
    case PDC_RTC_YEAR:
        ret = visible_time->year;
        break;
    case PDC_RTC_ASEC:
        ret = pdc->rtc.alarm.sec;
        if (pdc->rtc.alarm_en_bits & PDC_RTC_AEN_SEC_M) {
            ret |= (1 << PDC_RTC_SEC_B);
        }
        break;
    case PDC_RTC_AMIN:
        ret = pdc->rtc.alarm.min;
        if (pdc->rtc.alarm_en_bits & PDC_RTC_AEN_MIN_M) {
            ret |= (1 << PDC_RTC_MIN_B);
        }
        break;
    case PDC_RTC_AHOUR:
        ret = pdc->rtc.alarm.hour;
        if (pdc->rtc.alarm_en_bits & PDC_RTC_AEN_HOUR_M) {
            ret |= (1 << PDC_RTC_HOUR_B);
        }
        break;
    case PDC_RTC_ADAY:
        ret = pdc->rtc.alarm.day;
        if (pdc->rtc.alarm_en_bits & PDC_RTC_AEN_DAY_M) {
            ret |= (1 << PDC_RTC_DAY_B);
        }
        break;
    case PDC_RTC_AMON:
        ret = pdc->rtc.alarm.mon;
        if (pdc->rtc.alarm_en_bits & PDC_RTC_AEN_MON_M) {
            ret |= (1 << PDC_RTC_MON_B);
        }
        break;
    case PDC_RTC_AYEAR:
        ret = pdc->rtc.alarm.year;
        if (pdc->rtc.alarm_en_bits & PDC_RTC_AEN_YEAR_M) {
            ret |= (1 << PDC_RTC_YEAR_B);
        }
        break;
    case PDC_RTC_IRQ_STATUS:
        ret = pdc->rtc.irq_status;
        break;
    case PDC_RTC_IRQ_CLEAR:
        ret = pdc->rtc.irq_clear;
        break;
    case PDC_RTC_IRQ_EN:
        ret = pdc->rtc.irq_en;
        break;

    case PDC_IRQ_STATUS:
        ret = pdc->irq_status & pdc->irq_enable;
        break;
    case PDC_IRQ_ENABLE:
        ret = pdc->irq_enable;
        break;
    case PDC_IRQ_CLEAR:
        break;
    case PDC_IRQ_ROUTE:
        ret = pdc->irq_route;
        break;

    case PDC_SOC_GPIO_STATUS:
        /* there are other GPIO bits in this register which aren't implemnted */
        ret = pdc->syswake_state << 2;
        break;

    default:
        DBGLOG("unhandled pdc read(0x%08" HWADDR_PRIx ")\n", addr);
    }

    return ret;
}

static void comet_pdc_io_write(void *opaque, hwaddr addr,
                               uint64_t val, unsigned size)
{
    struct comet_pdc_state_s *pdc = opaque;
    uint32_t syswake;
    uint32_t diff;

    addr &= -4;

    if (addr >= PDC_SYS_WAKE0 &&
        addr < (PDC_SYS_WAKE0_CONFIG + pdc->num_syswakes * SYS_WAKE_STRIDE)) {
        syswake = (addr - PDC_SYS_WAKE0) / SYS_WAKE_STRIDE;

        if (addr & 0x4) {
            /* SYS_WAKEn_CONFIG */
            DBGLOG("SYS_WAKE%d_CONFIG = 0x%08" PRIx64 "\n", syswake, val);
        } else {
            /* SYS_WAKEn*/
            pdc->syswake_int_modes[syswake] = (val >> 1) & 0x7;

            switch (pdc->syswake_int_modes[syswake]) {
            case SW_IRQ_EDGE_LOW:
            case SW_IRQ_EDGE_HIGH:
            case SW_IRQ_EDGE0:
            case SW_IRQ_EDGE1:
                pdc->irq_edge_mask |= (1 << syswake);
                break;

            default:
                pdc->irq_edge_mask &= ~(1 << syswake);
                break;
            }
        }

        return;
    }

    switch (addr) {
    /* Watch dog timer */
    case PDC_WD_SW_RESET:
        if (val & 1) {
            qemu_system_reset_request();
        }
        break;

    /* Real time clock */
    case PDC_RTC_CONTROL:
        diff = val ^ pdc->rtc.control;
        pdc->rtc.control = val & PDC_RTC_CONTROL_M;
        /* update time */
        if (val & PDC_RTC_CONTROL_UPDATE_M) {
            pdc->rtc.time = pdc->rtc.update;
        }
        /* enable clock */
        if (diff & PDC_RTC_CONTROL_CE_M) {
            if (val & PDC_RTC_CONTROL_CE_M) {
                /* clear alarm interrupt if requested */
                if (pdc->rtc.irq_status & pdc->rtc.irq_clear
                        & PDC_RTC_IRQ_ALARM_M) {
                    pdc->rtc.irq_status &= ~PDC_RTC_IRQ_ALARM_M;
                    pdc->rtc.irq_clear &= pdc->rtc.irq_status;
                    pdc_rtc_update_irq(&pdc->rtc);
                }
                /* apply the update time and start the clock */
                pdc->rtc.time = pdc->rtc.update;
                pdc_rtc_enable(&pdc->rtc);
            } else {
                pdc_rtc_disable(&pdc->rtc);
            }
        }
        break;
    case PDC_RTC_SEC:
        pdc->rtc.update.sec = val & PDC_RTC_SEC_M;
        break;
    case PDC_RTC_MIN:
        pdc->rtc.update.min = val & PDC_RTC_MIN_M;
        break;
    case PDC_RTC_HOUR:
        pdc->rtc.update.hour = val & PDC_RTC_HOUR_M;
        break;
    case PDC_RTC_DAY:
        pdc->rtc.update.day = val & PDC_RTC_DAY_M;
        break;
    case PDC_RTC_MON:
        pdc->rtc.update.mon = val & PDC_RTC_MON_M;
        break;
    case PDC_RTC_YEAR:
        pdc->rtc.update.year = val & PDC_RTC_YEAR_M;
        break;
    case PDC_RTC_ASEC:
        pdc->rtc.alarm.sec = val & PDC_RTC_SEC_M;
        if (val & (1 << PDC_RTC_SEC_B)) {
            pdc->rtc.alarm_en_bits |= ~PDC_RTC_AEN_SEC_M;
        } else {
            pdc->rtc.alarm_en_bits &= ~PDC_RTC_AEN_SEC_M;
        }
        break;
    case PDC_RTC_AMIN:
        pdc->rtc.alarm.min = val & PDC_RTC_MIN_M;
        if (val & (1 << PDC_RTC_MIN_B)) {
            pdc->rtc.alarm_en_bits |= ~PDC_RTC_AEN_MIN_M;
        } else {
            pdc->rtc.alarm_en_bits &= ~PDC_RTC_AEN_MIN_M;
        }
        break;
    case PDC_RTC_AHOUR:
        pdc->rtc.alarm.hour = val & PDC_RTC_HOUR_M;
        if (val & (1 << PDC_RTC_HOUR_B)) {
            pdc->rtc.alarm_en_bits |= ~PDC_RTC_AEN_HOUR_M;
        } else {
            pdc->rtc.alarm_en_bits &= ~PDC_RTC_AEN_HOUR_M;
        }
        break;
    case PDC_RTC_ADAY:
        pdc->rtc.alarm.day = val & PDC_RTC_DAY_M;
        if (val & (1 << PDC_RTC_DAY_B)) {
            pdc->rtc.alarm_en_bits |= ~PDC_RTC_AEN_DAY_M;
        } else {
            pdc->rtc.alarm_en_bits &= ~PDC_RTC_AEN_DAY_M;
        }
        break;
    case PDC_RTC_AMON:
        pdc->rtc.alarm.mon = val & PDC_RTC_MON_M;
        if (val & (1 << PDC_RTC_MON_B)) {
            pdc->rtc.alarm_en_bits |= ~PDC_RTC_AEN_MON_M;
        } else {
            pdc->rtc.alarm_en_bits &= ~PDC_RTC_AEN_MON_M;
        }
        break;
    case PDC_RTC_AYEAR:
        pdc->rtc.alarm.year = val & PDC_RTC_YEAR_M;
        if (val & (1 << PDC_RTC_YEAR_B)) {
            pdc->rtc.alarm_en_bits |= ~PDC_RTC_AEN_YEAR_M;
        } else {
            pdc->rtc.alarm_en_bits &= ~PDC_RTC_AEN_YEAR_M;
        }
        break;
    case PDC_RTC_IRQ_STATUS:
        pdc->rtc.irq_status |= val & PDC_RTC_IRQ_M;
        pdc_rtc_update_irq(&pdc->rtc);
        break;
    case PDC_RTC_IRQ_CLEAR:
        val &= PDC_RTC_IRQ_M;
        pdc->rtc.irq_clear |= val;
        /* alarm interrupt only gets cleared when clock is enabled */
        pdc->rtc.irq_status &= ~(pdc->rtc.irq_clear & ~PDC_RTC_IRQ_ALARM_M);
        if (pdc->rtc.control & PDC_RTC_CONTROL_CE_M) {
            pdc->rtc.irq_status &= ~(pdc->rtc.irq_clear & PDC_RTC_IRQ_ALARM_M);
        }
        /* leave bits set that didn't take effect */
        pdc->rtc.irq_clear &= pdc->rtc.irq_status;
        pdc_rtc_update_irq(&pdc->rtc);
        break;
    case PDC_RTC_IRQ_EN:
        pdc->rtc.irq_en = val & PDC_RTC_IRQ_M;
        pdc_rtc_update_irq(&pdc->rtc);
        break;

    case PDC_IRQ_STATUS:
        break;
    case PDC_IRQ_ENABLE:
        pdc->irq_enable = val;
        comet_pdc_update_irq(pdc);
        break;
    case PDC_IRQ_CLEAR:
        pdc->irq_status &= ~(val & pdc->irq_edge_mask);
        comet_pdc_update_irq(pdc);
        break;
    case PDC_IRQ_ROUTE:
        diff = pdc->irq_route ^ val;
        pdc->irq_route = val & PDC_IRQ_ROUTE_M;
        if (diff & (1 << PDC_INIRQ_RTC)) {
            pdc->rtc.irq_routed = !!(val & (1 << PDC_INIRQ_RTC));
            pdc_rtc_update_irq(&pdc->rtc);
        }
        break;
    default:
        DBGLOG("unhandled pdc write(0x%08" HWADDR_PRIx ", 0x%08" PRIx64 ")\n", addr, val);
    };
}

static const MemoryRegionOps comet_pdc_io_ops = {
    .read = comet_pdc_io_read,
    .write = comet_pdc_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void comet_pdc_reset(void *opaque)
{
    struct comet_pdc_state_s *pdc = opaque;
    size_t i;

    pdc_rtc_init(&pdc->rtc, 0);

    for (i = 0; i < pdc->num_syswakes; i++)
        pdc->syswake_int_modes[i] = SW_IRQ_NONE0;

    pdc->irq_status = pdc->irq_enable = 0;
    pdc->irq_edge_mask = 0;
    pdc->irq_route = 0x0f010000;
    pdc->syswake_state = 0;
}

static void comet_pdc_keyboard_handler(void *opaque, int keycode)
{
    struct comet_pdc_state_s *pdc = opaque;
    int code = keycode & 0x7f;
    int syswake = -1;
    bool down = !(keycode & 0x80);
    bool interrupt = false;

    switch (code) {
    case 0x2c: /* z / SYS_WAKE2 */
        syswake = 2;
        break;
    case 0x2d: /* x / SYS_WAKE1 */
        syswake = 1;
        break;
    case 0x2e: /* c / SYS_WAKE0 */
        syswake = 0;
        break;
    }

    if (syswake == -1)
        return;

    if (!!(pdc->syswake_state & (1 << syswake)) == down) {
        /* value didn't change */
        return;
    }

    if (down)
        pdc->syswake_state |= (1 << syswake);
    else
        pdc->syswake_state &= ~(1 << syswake);

    switch (pdc->syswake_int_modes[syswake]) {
    case SW_IRQ_LEVEL_LOW:
    case SW_IRQ_EDGE_LOW:
        interrupt = !down;
        break;

    case SW_IRQ_LEVEL_HIGH:
    case SW_IRQ_EDGE_HIGH:
        interrupt = down;
        break;

    case SW_IRQ_EDGE0:
    case SW_IRQ_EDGE1:
        interrupt = true;
        break;
    }

    if (interrupt) {
        pdc->irq_status |= (1 << syswake);
        comet_pdc_update_irq(pdc);
    }
}

void comet_pdc_init(MemoryRegion *address_space, hwaddr iomem,
                    target_ulong len, qemu_irq irq, qemu_irq rtc_irq,
                    qemu_irq ir_irq, qemu_irq wd_irq, uint8_t num_syswakes)
{
    struct comet_pdc_state_s *pdc;

    pdc = g_malloc0(sizeof(struct comet_pdc_state_s));

    /* Initialise register region */
    memory_region_init_io(&pdc->iomem, &comet_pdc_io_ops, pdc,
                          "comet-pdc", len - 1);
    memory_region_add_subregion(address_space, iomem, &pdc->iomem);

    pdc->irq = irq;
    pdc->rtc.irq = rtc_irq;
    pdc->num_syswakes = num_syswakes;

    qemu_register_reset(comet_pdc_reset, pdc);

    qemu_add_kbd_event_handler(comet_pdc_keyboard_handler, pdc);
}
