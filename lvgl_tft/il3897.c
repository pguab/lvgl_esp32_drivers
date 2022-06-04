/**
@file il3897.c
@brief   Waveshare e-paper 2.13in v2 with GoodDisplay GDEH0213B72 display
@version 1.0
@date    2022-04-30
@author  Peter Guba

@section LICENSE

MIT License

Copyright (c) 2022 Peter Guba

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

/*********************
 *      INCLUDES
 *********************/
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "il3897.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "IL3897"

#define IL3897_DC_PIN           CONFIG_LV_DISP_PIN_DC
#define IL3897_RST_PIN          CONFIG_LV_DISP_PIN_RST
#define IL3897_USE_RST          CONFIG_LV_DISP_USE_RST
#define IL3897_BUSY_PIN         CONFIG_LV_DISP_PIN_BUSY
#define IL3897_BUSY_LEVEL       1

/* IL3897 commands */
#define IL3897_CMD_DRIVER_OUTPUT_CTRL			0x01
#define IL3897_CMD_GATE_DRIVING_VOLTAGE_CTRL	0x03
#define IL3897_CMD_SOURCE_DRIVING_VOLTAGE_CTRL	0x04
#define IL3897_CMD_SOFTSTART_CTRL               0x0C
#define IL3897_CMD_DEEP_SLEEP_MODE              0x10
#define IL3897_CMD_DATA_ENTRY_MODE              0x11
#define IL3897_CMD_SW_RESET                     0x12
#define IL3897_CMD_HV_READY_DETECTION       	0x14
#define IL3897_CMD_VCI_DETECTION        		0x15
#define IL3897_CMD_TEMPERATURE_SENSOR_SEL		0x18
#define IL3897_CMD_TEMPERATURE_SENSOR_CTRL		0x1A
#define IL3897_CMD_TEMPERATURE_SENSOR_READ		0x1B
#define IL3897_CMD_MASTER_ACTIVATION			0x20
#define IL3897_CMD_UPDATE_CTRL1     			0x21
#define IL3897_CMD_UPDATE_CTRL2     			0x22
#define IL3897_CMD_WRITE_RAM                    0x24
#define IL3897_CMD_WRITE_RED_RAM            	0x26
#define IL3897_CMD_READ_RAM         			0x27
#define IL3897_CMD_ACVCOM                       0x2B
#define IL3897_CMD_WRITE_VCOM                   0x2C
#define IL3897_CMD_OTP_READ                     0x2D
#define IL3897_CMD_READ_USER_ID                 0x2E
#define IL3897_CMD_READ_STATUS_BIT              0x2F
#define IL3897_CMD_WRITE_LUT                    0x32
#define IL3897_CMD_PROGRAM_OTP_SEL              0x36
#define IL3897_CMD_WRITE_USER_ID                0x38
#define IL3897_CMD_OTP_PROGRAM_MODE             0x39
#define IL3897_CMD_DUMMY_LINE_PERIOD            0x3A
#define IL3897_CMD_GATE_LINE_WIDTH              0x3B
#define IL3897_CMD_BORDER_WAVEFORM_CTRL         0x3C
#define IL3897_CMD_READ_RAM_OPTION              0x41
#define IL3897_CMD_RAM_XPOS_CTRL        		0x44
#define IL3897_CMD_RAM_YPOS_CTRL        		0x45
#define IL3897_CMD_RAM_XPOS_CNTR                0x4E
#define IL3897_CMD_RAM_YPOS_CNTR                0x4F
#define IL3897_CMD_ANALOG_BLOCK_CTRL            0x74
#define IL3897_CMD_DIGITAL_BLOCK_CTRL           0x7E

/* Data entry sequence modes */
#define IL3897_DATA_ENTRY_MASK		0x07
#define IL3897_DATA_ENTRY_XDYDX		0x00
#define IL3897_DATA_ENTRY_XIYDX		0x01
#define IL3897_DATA_ENTRY_XDYIX		0x02
#define IL3897_DATA_ENTRY_XIYIX		0x03
#define IL3897_DATA_ENTRY_XDYDY		0x04
#define IL3897_DATA_ENTRY_XIYDY		0x05
#define IL3897_DATA_ENTRY_XDYIY		0x06
#define IL3897_DATA_ENTRY_XIYIY		0x07

/* time constants in ms */
#define IL3897_RESET_DELAY			10
// full refresh wait time
#define IL3897_WAIT                 2000

/**********************
 *      MACROS
 **********************/

#define BIT_SET(a,b)                    ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b)                  ((a) &= ~(1U<<(b)))

/**********************
 *      TYPEDEFS
 **********************/

typedef struct il3897_data_struct {
    bool partial;
    bool is_init;
    bool is_sleep;
    bool is_poweron;
    il3897_power_mode_t  mode;
    int32_t    poweroff_delay;
    lv_task_t *poweroff_task;
    uint32_t   last_refresh_time;
    int32_t    partial_refresh_count;
    int32_t    max_partial_refresh;
} il3897_data_t;

/**********************
 *  STATIC VARIABLES
 **********************/

static const uint8_t il3897_lut_full_update[]= {
/*  0 */    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT0: BB:     VS 0 ~7
/*  7 */    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT1: BW:     VS 0 ~7
/* 14 */    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT2: WB:     VS 0 ~7
/* 21 */    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT3: WW:     VS 0 ~7
/* 28 */    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT4: VCOM:   VS 0 ~7

/* 35 */    0x03,0x03,0x00,0x00,0x02,                       // TP0 A~D, RP0
/* 40 */    0x09,0x09,0x00,0x00,0x02,                       // TP1 A~D, RP1
/* 45 */    0x03,0x03,0x00,0x00,0x02,                       // TP2 A~D, RP2
/* 50 */    0x00,0x00,0x00,0x00,0x00,                       // TP3 A~D, RP3
/* 55 */    0x00,0x00,0x00,0x00,0x00,                       // TP4 A~D, RP4
/* 60 */    0x00,0x00,0x00,0x00,0x00,                       // TP5 A~D, RP5
/* 65 */    0x00,0x00,0x00,0x00,0x00,                       // TP6 A~D, RP6

/* 70 */    0x15,0x41,0xA8,0x32,0x30,0x0A,                  // VGH, VSH1, VSH2, VSL, dummy line period, gate line width
};

static const uint8_t il3897_lut_partial_update[]= {
/*  0 */    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT0: BB:     VS 0 ~7
/*  7 */    0x80,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT1: BW:     VS 0 ~7
/* 14 */    0x40,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT2: WB:     VS 0 ~7
/* 21 */    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT3: WW:     VS 0 ~7
/* 28 */    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT4: VCOM:   VS 0 ~7

/* 35 */    0x0A,0x00,0x00,0x00,0x00,                       // TP0 A~D RP0
/* 40 */    0x00,0x00,0x00,0x00,0x00,                       // TP1 A~D RP1
/* 45 */    0x00,0x00,0x00,0x00,0x00,                       // TP2 A~D RP2
/* 50 */    0x00,0x00,0x00,0x00,0x00,                       // TP3 A~D RP3
/* 55 */    0x00,0x00,0x00,0x00,0x00,                       // TP4 A~D RP4
/* 60 */    0x00,0x00,0x00,0x00,0x00,                       // TP5 A~D RP5
/* 65 */    0x00,0x00,0x00,0x00,0x00,                       // TP6 A~D RP6

/* 70 */    0x15,0x41,0xA8,0x32,0x30,0x0A,
};

static il3897_data_t il3897_data = {
    .partial = false,
    .is_init = false,
    .is_sleep = true,
    .is_poweron = false,
    .mode = IL3897_POWER_OFF,
    .poweroff_delay = -1,
    .poweroff_task = NULL,
    .last_refresh_time = 0,
    .partial_refresh_count = 0,
    .max_partial_refresh = 5
};

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void il3897_panel_init();
#if IL3897_USE_RST
static inline void il3897_hw_reset();
#endif
static void il3897_set_full_update();
static void il3897_set_partial_update();
static void il3897_waitbusy(int wait_ms);
static inline void il3897_command_mode(void);
static inline void il3897_data_mode(void);
static inline void il3897_write_cmd(uint8_t cmd, const uint8_t *data, size_t len);
//static void il3897_send_data(uint8_t *data, uint16_t length);
static inline void il3897_set_window( uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye);
static inline void il3897_set_cursor(uint16_t sx, uint16_t ys);
static void il3897_update_display(void);
static void il3897_clear_cntlr_mem(bool update);
static void il3897_set_absolute_px(lv_disp_drv_t * disp_drv, uint8_t* buf,
                                   lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                                   lv_color_t color, lv_opa_t opa);
static void il3897_poweroff_task(lv_task_t * task);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void il3897_init(void)
{
    /* Initialize non-SPI GPIOs */
    gpio_pad_select_gpio(IL3897_DC_PIN);
    gpio_set_direction(IL3897_DC_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(IL3897_BUSY_PIN);
    gpio_set_direction(IL3897_BUSY_PIN,  GPIO_MODE_INPUT);

#if IL3897_USE_RST
    gpio_pad_select_gpio(IL3897_RST_PIN);
    gpio_set_direction(IL3897_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level( IL3897_RST_PIN, 1);

    il3897_hw_reset();
#endif

    il3897_panel_init();

    il3897_set_full_update();

    il3897_clear_cntlr_mem(true);

    //il3897_waitbusy(IL3897_WAIT);   // in example

    // Switch to partial update
    il3897_set_partial_update();

//    il3897_update_display();
}

/* Required by LVGL */
void il3897_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    ESP_LOGD( TAG, "il3897_flush(x1=%d, y1=%d, x2=%d, y2=%d)",
              area->x1, area->y1, area->x2, area->y2);

    bool full_update = false;
#if IL3897_USE_RST
    if (il3897_data.is_sleep) {
        il3897_hw_reset();
        il3897_panel_init();
        if ( il3897_data.partial_refresh_count < 0 ||       /* Enforce full refresh */
             ( il3897_data.max_partial_refresh > 0 &&
               il3897_data.partial_refresh_count >= il3897_data.max_partial_refresh) ) {
            ESP_LOGD( TAG, "Use full refresh after meximum partial refresh count");
            il3897_set_full_update();
            full_update = true;
        } else {
            il3897_set_partial_update();
        }
    } else {
        if ( il3897_data.partial_refresh_count < 0 ||       /* Enforce full refresh */
             ( il3897_data.max_partial_refresh > 0 &&
               il3897_data.partial_refresh_count >= il3897_data.max_partial_refresh) ) {
            ESP_LOGD( TAG, "Use full refresh after meximum partial refresh count");
            il3897_panel_init();
            il3897_set_full_update();
            full_update = true;
        }
    }
#else
    if ( il3897_data.partial_refresh_count < 0 ||       /* Enforce full refresh */
         ( il3897_data.max_partial_refresh > 0 &&
           il3897_data.partial_refresh_count >= il3897_data.max_partial_refresh) ) {
        il3897_panel_init();
        il3897_set_full_update();
        full_update = true;
    }
#endif

    uint8_t *buffer = (uint8_t*) color_map;
    uint16_t x_addr_counter = 0;
    uint16_t y_addr_counter = 0;
    size_t area_width = IL3897_WIDTH_IN_BYTES;
    size_t area_height = EPD_PANEL_HEIGHT;
    uint8_t entry_mode = IL3897_DATA_ENTRY_XIYIX;
    lv_area_t window;

    window.x1 = 0;
    window.x2 = IL3897_BUFFER_WIDTH - 1;
    window.y1 = 0;
    window.y2 = EPD_PANEL_HEIGHT - 1;

    ESP_LOGV( TAG, "window(xs=%d, xe=%d, ys=%d, ye=%d), start(x=%d, y=%d), width=%d, height=%d",
              window.x1, window.x2, window.y1, window.y2,
              x_addr_counter, y_addr_counter, area_width, area_height);

    il3897_write_cmd(IL3897_CMD_DATA_ENTRY_MODE, &entry_mode, 1);

    il3897_set_window(window.x1, window.x2, window.y1, window.y2);

    il3897_set_cursor(x_addr_counter, y_addr_counter);

    il3897_write_cmd(IL3897_CMD_WRITE_RAM, buffer, area_width * area_height);

    // Also RED RAM
    il3897_set_window(window.x1, window.x2, window.y1, window.y2);

    il3897_set_cursor(x_addr_counter, y_addr_counter);

    il3897_write_cmd(IL3897_CMD_WRITE_RED_RAM, buffer, area_width * area_height);

    il3897_update_display();

    il3897_data.last_refresh_time = lv_tick_get();
    if (il3897_data.poweroff_task) {
        lv_task_reset(il3897_data.poweroff_task);
        lv_task_set_prio(il3897_data.poweroff_task, LV_TASK_PRIO_LOW);
    }

    if (full_update) {
        il3897_set_partial_update();
    }

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    lv_disp_flush_ready(drv);
}


/* Rotate the display by "software" when using PORTRAIT orientation.
 * BIT_SET(byte_index, bit_index) clears the bit_index pixel at byte_index of
 * the display buffer.
 * BIT_CLEAR(byte_index, bit_index) sets the bit_index pixel at the byte_index
 * of the display buffer. */
void il3897_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t* buf,
    lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa)
{
    /* Pixel coordinates are relative to current area. As we always send whole buffer,
     * we need to transform them to absolute. */
    lv_disp_buf_t *vdb = disp_drv->buffer;
    x += vdb->area.x1;
    y += vdb->area.y1;

#if defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE)
    il3897_set_absolute_px(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
    /* Right bottom corner has coordinates [0,0] */
    x = EPD_PANEL_WIDTH - x - 1;
    y = EPD_PANEL_HEIGHT - y - 1;
    il3897_set_absolute_px(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT)
    /* Left bottom corner has coordinates [0,0] */
    lv_coord_t point_temp = x;
    x = y;
    y = EPD_PANEL_HEIGHT - point_temp - 1;
    il3897_set_absolute_px(disp_drv, buf, buf_w, x, y, color, opa);
#elif defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED)
    /* Right top corner has coordinates [0,0] */
    lv_coord_t point_temp = x;
    x = EPD_PANEL_WIDTH - y - 1;
    y = point_temp;
    il3897_set_absolute_px(disp_drv, buf, buf_w, x, y, color, opa);
#else
#error "Unsupported orientation used"
#endif
}


/* Required by LVGL */
void il3897_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area) {
//    ESP_LOGD( TAG, "il3897_rounder(x1=%d, y1=%d, x2=%d, y2=%d)",
//              area->x1, area->y1, area->x2, area->y2);

#if defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE) || defined (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
    area->x1 = 0;
    area->x2 = EPD_PANEL_WIDTH - 1;
    area->y1 = 0;
    area->y2 = EPD_PANEL_HEIGHT - 1;
#elif defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT) || defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED)
    area->x1 = 0;
    area->x2 = EPD_PANEL_HEIGHT - 1;
    area->y1 = 0;
    area->y2 = EPD_PANEL_WIDTH - 1;
#else
    #error "Unsupported orientation used"
#endif
}


/* Enter deep sleep mode */
void il3897_sleep_in(void)
{
    ESP_LOGD( TAG, "il3897_sleep_in()");

    uint8_t data;

    il3897_set_power(false);

#if IL3897_USE_RST
    data = 0x01;
    il3897_write_cmd(IL3897_CMD_DEEP_SLEEP_MODE, &data, 1);

    il3897_data.is_sleep = true;
    il3897_data.partial = false;
#else
    /* Without RESET, display can not be woke up, so don't go to sleep */
#endif
}

void il3897_set_power(bool on)
{
    ESP_LOGD( TAG, "il3897_set_power(%d)", on);

    uint8_t tmp = 0;

    if ( on ) {
        tmp = 0xC0;
    } else {
        tmp = 0x03;     // in GxEPD2 0xC3
    }
    il3897_data.is_poweron = on;

    il3897_write_cmd(IL3897_CMD_UPDATE_CTRL2, &tmp, 1);
    ESP_LOGD( TAG, "IL3897_CMD_UPDATE_CTRL2=0x%02X", tmp);

    il3897_write_cmd(IL3897_CMD_MASTER_ACTIVATION, NULL, 0);

    il3897_waitbusy(IL3897_WAIT);
}

void il3897_set_power_mode(il3897_power_mode_t mode, int32_t timeout)
{
    ESP_LOGD( TAG, "il3897_set_power_mode(%d)", mode);
    il3897_data.mode = mode;
    il3897_data.poweroff_delay = timeout;
    if (il3897_data.mode == IL3897_POWER_TIMED) {
        if (il3897_data.poweroff_task) {
            lv_task_set_period(il3897_data.poweroff_task, timeout);
        } else {
            il3897_data.poweroff_task = lv_task_create(il3897_poweroff_task,
                                                       timeout, LV_TASK_PRIO_OFF, NULL);
            if ( il3897_data.poweroff_task == NULL ) {
                ESP_LOGE( TAG, "Unable to create il3897_poweroff_task");
                il3897_data.mode = IL3897_POWER_OFF;
            }
        }
    }
}

void il3897_set_max_partial_refresh(int count)
{
    ESP_LOGD( TAG, "il3897_set_max_partial_refresh(%d)", count);
    il3897_data.max_partial_refresh = count;
}

void il3897_enforce_full_refresh()
{
    ESP_LOGD( TAG, "il3897_enforce_full_refresh()");
    il3897_data.partial_refresh_count = -1;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void il3897_panel_init()
{
    uint8_t tmp[3] = {0};

    /* Software reset */
    //il3897_write_cmd(IL3897_CMD_SW_RESET, NULL, 0);

    /* Busy wait for the BUSY signal to go low */
    //il3897_waitbusy(IL3897_WAIT);

    tmp[0] = 0x54;
    il3897_write_cmd(IL3897_CMD_ANALOG_BLOCK_CTRL, tmp, 1);

    tmp[0] = 0x3B;
    il3897_write_cmd(IL3897_CMD_DIGITAL_BLOCK_CTRL, tmp, 1);

    tmp[0] = ( EPD_PANEL_HEIGHT - 1) & 0xFF;
    tmp[1] = ( EPD_PANEL_HEIGHT >> 8 );
    tmp[2] = 0; // GD = 0; SM = 0; TB = 0;
    il3897_write_cmd(IL3897_CMD_DRIVER_OUTPUT_CTRL, tmp, 3);

    tmp[0] = 0x03;
    il3897_write_cmd(IL3897_CMD_BORDER_WAVEFORM_CTRL, tmp, 1);

    tmp[0] = 0x70;  // 0x70 in GxEPD2, 0x55 in GoodDisplay example
    il3897_write_cmd(IL3897_CMD_WRITE_VCOM, tmp, 1);

    il3897_write_cmd(IL3897_CMD_GATE_DRIVING_VOLTAGE_CTRL, &il3897_lut_full_update[70], 1);

    il3897_write_cmd(IL3897_CMD_SOURCE_DRIVING_VOLTAGE_CTRL,
                     &il3897_lut_full_update[71], 3);

    il3897_write_cmd(IL3897_CMD_DUMMY_LINE_PERIOD, &il3897_lut_full_update[74], 1);
    il3897_write_cmd(IL3897_CMD_GATE_LINE_WIDTH, &il3897_lut_full_update[75], 1);

    tmp[0] = 0x80;  // Inverse RED RAM | Normal BW RAM
    il3897_write_cmd(IL3897_CMD_UPDATE_CTRL1, tmp, 1);
}

#if IL3897_USE_RST
static inline void il3897_hw_reset()
{
    ESP_LOGD( TAG, "il3897_hw_reset() start");

    // Example code: 1/200ms - 0/10ms - 1/200ms
    gpio_set_level( IL3897_RST_PIN, 0);
    vTaskDelay(IL3897_RESET_DELAY / portTICK_RATE_MS);
    gpio_set_level( IL3897_RST_PIN, 1);
    vTaskDelay(IL3897_RESET_DELAY / portTICK_RATE_MS);
    il3897_data.is_sleep = false;
}
#endif

static void il3897_set_full_update()
{
    /* Write full update LUT */
    il3897_write_cmd(IL3897_CMD_WRITE_LUT, il3897_lut_full_update, 70);
    il3897_data.partial = false;
}

static void il3897_set_partial_update()
{
    uint8_t tmp = 0x26;
    il3897_write_cmd(IL3897_CMD_WRITE_VCOM, &tmp, 1);

    il3897_write_cmd(IL3897_CMD_WRITE_LUT, il3897_lut_partial_update, 70);
    il3897_data.partial = true;

//    il3897_waitbusy(IL3897_WAIT);
}

/* TODO: Remove the busy waiting */
static void il3897_waitbusy(int wait_ms)
{
    ESP_LOGD( TAG, "il3897_waitbusy() start");
    int i = 0;

    vTaskDelay(10 / portTICK_RATE_MS); // 10ms delay

    for(i = 0; i < wait_ms; i+=10) {
        if(gpio_get_level(IL3897_BUSY_PIN) != IL3897_BUSY_LEVEL) {
            ESP_LOGD( TAG, "il3897_waitbusy() end");
            return;
        }

        vTaskDelay(10 / portTICK_RATE_MS);
    }

    ESP_LOGE( TAG, "busy exceeded %dms", i*10 );
}

/* Set DC signal to command mode */
static inline void il3897_command_mode(void)
{
    gpio_set_level(IL3897_DC_PIN, 0);
}

/* Set DC signal to data mode */
static inline void il3897_data_mode(void)
{
    gpio_set_level(IL3897_DC_PIN, 1);
}

static inline void il3897_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();

    il3897_command_mode();
    disp_spi_send_data(&cmd, 1);

    if (data != NULL) {
        il3897_data_mode();
        disp_spi_send_data(data, len);
    }
}

// TODO:
/* Send length bytes of data to the display */
//static void il3897_send_data(uint8_t *data, uint16_t length)
//{
//    disp_wait_for_pending_transactions();

//    il3897_data_mode();
//    disp_spi_send_colors(data, length);
//}

/* Specify the start/end positions of the window address in the X and Y
 * directions by an address unit.
 *
 * @param sx: X Start position.
 * @param ex: X End position.
 * @param ys: Y Start position.
 * @param ye: Y End position.
 */
static inline void il3897_set_window( uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye)
{
    uint8_t tmp[4] = {0};

    tmp[0] = sx >> 3;
    tmp[1] = ex >> 3;

    /* Set X address start/end */
    il3897_write_cmd(IL3897_CMD_RAM_XPOS_CTRL, tmp, 2);

    tmp[0] = ys & 0xFF;
    tmp[1] = ys >> 8;
    tmp[2] = ye & 0xFF;
    tmp[3] = ye >> 8;
    /* Set Y address start/end */
    il3897_write_cmd(IL3897_CMD_RAM_YPOS_CTRL, tmp, 4);
}

/* Make initial settings for the RAM X and Y addresses in the address counter
 * (AC).
 *
 * @param sx: RAM X address counter.
 * @param ys: RAM Y address counter.
 */
static inline void il3897_set_cursor(uint16_t sx, uint16_t ys)
{
    uint8_t tmp[2] = {0};

    tmp[0] = sx >> 8;
    il3897_write_cmd(IL3897_CMD_RAM_XPOS_CNTR, tmp, 1);

    tmp[0] = ys & 0xFF;
    tmp[1] = ys >> 8;
    il3897_write_cmd(IL3897_CMD_RAM_YPOS_CNTR, tmp, 2);
}

/* After sending the RAM content we need to send the commands:
 * - Display Update Control 2
 * - Master Activation
 */
static void il3897_update_display(void)
{
    uint8_t tmp = 0x04;

    if ( !il3897_data.is_poweron ) {
        tmp |= 0xC0;
        il3897_data.is_poweron = true;
    }
    if ( il3897_data.mode == IL3897_POWER_OFF ) {
        tmp |= 0x03;
        il3897_data.is_poweron = false;
    }

    il3897_write_cmd(IL3897_CMD_UPDATE_CTRL2, &tmp, 1);
    ESP_LOGD( TAG, "IL3897_CMD_UPDATE_CTRL2=0x%02X", tmp);

    il3897_write_cmd(IL3897_CMD_MASTER_ACTIVATION, NULL, 0);

    il3897_waitbusy(IL3897_WAIT);

    if (il3897_data.partial) {
        ++il3897_data.partial_refresh_count;
    } else {
        il3897_data.partial_refresh_count = 0;
    }

    if ( il3897_data.mode == IL3897_POWER_SLEEP) {
        il3897_sleep_in();
    }
}

/* Clear the graphic RAM. */
static void il3897_clear_cntlr_mem(bool update)
{
    ESP_LOGD( TAG, "il3897_clear_cntlr_mem() start");

    uint8_t tmp;

    /* Arrays used by SPI must be word alligned */
    // If stack memory is problem, can be one row
    WORD_ALIGNED_ATTR uint8_t clear_page[IL3897_BUFFER_SIZE];
    memset(clear_page, 0xff, sizeof(clear_page));   // All bits 1 for white

    /* Configure entry mode */
    tmp = IL3897_DATA_ENTRY_XIYIX;
    il3897_write_cmd(IL3897_CMD_DATA_ENTRY_MODE, &tmp, 1);

    /* Set Black-White memory */
    /* Configure the window */
    il3897_set_window(0, IL3897_BUFFER_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);

    /* Set starting postition */
    il3897_set_cursor(0, 0);

    il3897_write_cmd(IL3897_CMD_WRITE_RAM, clear_page, sizeof(clear_page));

    /* Set Red memory */
    memset(clear_page, 0x00, sizeof(clear_page));   // All bits 0 for non-red

    /* Configure the window */
    il3897_set_window(0, IL3897_BUFFER_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);

    /* Set starting postition */
    il3897_set_cursor(0, 0);

    il3897_write_cmd(IL3897_CMD_WRITE_RED_RAM, clear_page, sizeof(clear_page));

    if (update) {
        il3897_update_display();
    }
}

void il3897_set_absolute_px(lv_disp_drv_t * disp_drv, uint8_t* buf,
    lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa)
{
    if (x < 0 || x >= EPD_PANEL_WIDTH || y < 0 || y >= EPD_PANEL_HEIGHT) {
        return;
    }
    uint16_t byte_index = (x + y * (IL3897_WIDTH_IN_BYTES << 3)) >> 3;
    uint8_t  bit_index = x & 0x7;

#if defined(CONFIG_LV_INVERT_COLORS)
    if (color.full) {
        BIT_CLEAR(buf[byte_index], 7 - bit_index);
    } else {
        BIT_SET(buf[byte_index], 7 - bit_index);
    }
#else
    if (color.full) {
        BIT_SET(buf[byte_index], 7 - bit_index);
    } else {
        BIT_CLEAR(buf[byte_index], 7 - bit_index);
    }
#endif
}

static void il3897_poweroff_task(lv_task_t * task)
{
    ESP_LOGD( TAG, "il3897_poweroff_task()");

    if ( il3897_data.is_poweron &&
         il3897_data.poweroff_delay > 0 ) {
        int32_t now = lv_tick_get();
        if ( il3897_data.last_refresh_time + il3897_data.poweroff_delay <= now ) {
            ESP_LOGD( TAG, "Power off after delay");
            il3897_set_power(false);
            lv_task_set_prio(il3897_data.poweroff_task, LV_TASK_PRIO_OFF);
        }
    }
}
