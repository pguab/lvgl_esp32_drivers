/**
 * @file il3897.h
 *
 */

#ifndef IL3897_H
#define IL3897_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "sdkconfig.h"

/*********************
 *      DEFINES
 *********************/

#define EPD_PANEL_WIDTH         CONFIG_LV_DISP_IL3897_EPD_PANEL_WIDTH   /* 122 */
#define EPD_PANEL_HEIGHT        CONFIG_LV_DISP_IL3897_EPD_PANEL_HEIGHT  /* 250 */
#define IL3897_WIDTH_IN_BYTES   (EPD_PANEL_WIDTH % 8 != 0 ? (EPD_PANEL_WIDTH >> 3) + 1 : EPD_PANEL_WIDTH >> 3)
#define IL3897_BUFFER_WIDTH     (IL3897_WIDTH_IN_BYTES << 3)
#define IL3897_BUFFER_SIZE      (IL3897_WIDTH_IN_BYTES * EPD_PANEL_HEIGHT)

/**********************
 *      TYPEDEFS
 **********************/

typedef enum {
    IL3897_POWER_OFF = 0,   /* Power off voltage generator circuit after each update */
    IL3897_POWER_TIMED,     /* Use il3897_poweroff_monitor_cb to power off after defined timeout */
    IL3897_POWER_MANUAL,    /* Never power off automatically, call il3897_set_power(false) required */
    IL3897_POWER_SLEEP,     /* Got to deep sleep after each update */
} il3897_power_mode_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void il3897_init(void);
void il3897_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void il3897_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area);
void il3897_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w,
                      lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa);

void il3897_sleep_in(void);
void il3897_set_power(bool on);

/**
 * @brief il3897_set_power_mode
 * @param mode[in]
 * @param timeout[in] - For IL3897_POWER_TIMED mode. Number of milliseconds from last refresh
 *                      after display is powered off.
 */
void il3897_set_power_mode(il3897_power_mode_t mode, int32_t timeout);

/**
 * @brief il3897_set_max_partial_refresh
 * @param count[in] - Number of partial refresh between automatic one time full refresh
 *                    <0 for partial refresh only
 */
void il3897_set_max_partial_refresh(int count);

void il3897_enforce_full_refresh();

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // IL3897_H
