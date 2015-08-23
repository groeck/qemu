/*
 * i.MX watchdog
 *
 * Copyright (C) 2015 Guenter Roeck
 *
 * This code is licensed under GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 */

#ifndef IMX_WDOG_H
#define IMX_WDOG_H

#include "hw/sysbus.h"
#include "hw/ptimer.h"

#define IMX2_WDT_WCR            0x00            /* Control Register */
#define IMX2_WDT_WCR_WT         (0xFF << 8)     /* -> Watchdog Timeout Field */
#define IMX2_WDT_WCR_WRE        (1 << 3)        /* -> WDOG Reset Enable */
#define IMX2_WDT_WCR_WDE        (1 << 2)        /* -> Watchdog Enable */
#define IMX2_WDT_WCR_WDZST      (1 << 0)        /* -> Watchdog timer Suspend */

#define IMX2_WDT_WSR            0x02            /* Service Register */
#define IMX2_WDT_SEQ1           0x5555          /* -> service sequence 1 */
#define IMX2_WDT_SEQ2           0xAAAA          /* -> service sequence 2 */

#define IMX2_WDT_WRSR           0x04            /* Reset Status Register */
#define IMX2_WDT_WRSR_TOUT      (1 << 1)        /* -> Reset due to Timeout */

#define IMX2_WDT_WMCR           0x08            /* Misc Register */

#define TYPE_IMX_WDOG   "imx.wdog"
#define IMX_WDOG(obj)   OBJECT_CHECK(IMXWDOGState, (obj), TYPE_IMX_WDOG)

typedef struct {
    /*< private >*/
    SysBusDevice busdev;

    /*< public >*/
    MemoryRegion iomem;
    DeviceState *ccm;

    struct ptimer_state *timer;

    uint32_t wcr;
    uint32_t wsr;
    uint32_t wrsr;
    uint32_t wmcr;
    uint32_t cnt;
} IMXWDOGState;

#endif /* IMX_WDOG_H */
