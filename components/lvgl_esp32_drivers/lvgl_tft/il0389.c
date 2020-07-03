/**
@file il0389.c
@brief   Waveshare e-paper 4.2in b/w display
@version 1.0
@date    2020-07-03
@author  Kong Fanming


@section LICENSE

MIT License

Copyright (c) 2020 Kong Fanming

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

#include "il0389.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "IL0389"

/**
 * IL0389 compatible EPD controller driver.
 */

#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))

#define EPD_PANEL_WIDTH            CONFIG_LVGL_DISPLAY_WIDTH
#define EPD_PANEL_HEIGHT        CONFIG_LVGL_DISPLAY_HEIGHT

#define IL0389_PIXELS_PER_BYTE        8

static bool il0389_partial = false;

static void il0389_waitbusy(int wait_ms)
{
    int i;
    vTaskDelay(10 / portTICK_RATE_MS); // 10ms delay
    for(i = 0; i < (wait_ms * 10); i++){
        if(gpio_get_level(IL0389_BUSY_PIN) != IL0389_BUSY_LEVEL) {
            //ESP_LOGI(TAG, "busy %dms", i*10);
            return;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    ESP_LOGE(TAG, "busy exceeded %dms", i * 10);
}



static inline void il0389_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(IL0389_DC_PIN, 0);  // command mode
    disp_spi_send_data(&cmd, 1);

    if (data != NULL) {
        gpio_set_level(IL0389_DC_PIN, 1); // data mode
        disp_spi_send_data((uint8_t *)data, len);
    }
}

static inline void il0389_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(IL0389_DC_PIN, 0);  // command mode
    disp_spi_send_data(&cmd, 1);
}


static void il0389_send_data(const uint8_t *data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(IL0389_DC_PIN, 1) ;  // data mode
    disp_spi_send_colors((uint8_t *)data, length); // requires the flush
}


static inline void il0389_set_window(uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye)
{
    uint8_t tmp[9];

    il0389_send_cmd(IL0389_CMD_PARTIAL_IN);
    tmp[0] = sx >> 8;
    tmp[1] = sx & 0xF8;
    tmp[2] = ex >> 8;
    tmp[3] = (ex & 0xF8) | 0x07;

    tmp[4] = ys >> 8;
    tmp[5] = ys & 0xFF;
    tmp[6] = ye >> 8;
    tmp[7] = ye & 0xFF;

    tmp[7] = 0x01; // Gates scan both inside and outside of the partial window. (default)
    il0389_write_cmd(IL0389_CMD_PARTIAL_WINDOW, tmp, 9);
    vTaskDelay(2 / portTICK_RATE_MS);
}

void il0389_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    size_t linelen = (area->x2 - area->x1 + 1)/8;
    uint8_t *buffer = (uint8_t*)color_map;
    // skip lines
    size_t address = (area->y1 * IL0389_COLUMNS) + (area->x1/8);

    ESP_LOGI(TAG, "flush: %d,%d at %d,%d", area->x1, area->x2, area->y1, area->y2);

    il0389_set_window(area->x1, area->x2, area->y1, area->y2);
    il0389_send_cmd(IL0389_CMD_DATA_START_TRANSMISSION_1);
    for(size_t row = area->y1; row <= area->y2; row++){
        il0389_send_data(buffer + address, linelen); // reverses bits in byte
        buffer += IL0389_COLUMNS; // next line down
    }

    // il0389_set_window(area->x1, area->x2, area->y1, area->y2);
    il0389_send_cmd(IL0389_CMD_PARTIAL_OUT);

    il0389_send_cmd(IL0389_CMD_DISPLAY_REFRESH);// DISPLAY_REFRESH
    vTaskDelay(100 / portTICK_RATE_MS);
    il0389_waitbusy(IL0389_WAIT);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing */
    lv_disp_flush_ready(drv);
}

static void il0389_clear_cntlr_mem(uint8_t ram_cmd, bool update)
{
    // DMA buffers in the stack is still allowed but externl ram enable SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY
    WORD_ALIGNED_ATTR uint8_t clear_page[EPD_PANEL_WIDTH / 8]; // used by SPI, must be word alligned
    uint8_t *buffer = (uint8_t*)clear_page;
    // skip lines
    size_t address = 0;
    size_t linelen = EPD_PANEL_WIDTH / 8;

    ESP_LOGI(TAG, "clear" );
    memset(clear_page, 0xff, sizeof(clear_page));

    il0389_set_window(0, EPD_PANEL_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1); // start/end
    il0389_send_cmd(ram_cmd);
    for(size_t row = 0; row <= EPD_PANEL_HEIGHT - 1; row++){
        il0389_send_data(buffer + address, linelen); // reverses bits in byte
        buffer += IL0389_COLUMNS; // next line down
    }

    // il0389_set_window(area->x1, area->x2, area->y1, area->y2);
    il0389_send_cmd(IL0389_CMD_PARTIAL_OUT);
    il0389_send_cmd(IL0389_CMD_DISPLAY_REFRESH);// DISPLAY_REFRESH
}

void il0389_set_px_cb(struct _disp_drv_t * disp_drv, uint8_t* buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa) {

    /* buf_w will be ignored, the configured CONFIG_LVGL_DISPLAY_HEIGHT and _WIDTH,
       and CONFIG_LVGL_DISPLAY_ORIENTATION_LANDSCAPE and _PORTRAIT will be used. */
    uint16_t byte_index = 0;
    uint8_t  bit_index = 0;

    byte_index = y + ((x>>3) * CONFIG_LVGL_DISPLAY_HEIGHT);
    bit_index  = x & 0x7;

    if (color.full != 0) {
        BIT_SET(buf[byte_index], 7 - bit_index);
    } else {
        BIT_CLEAR(buf[byte_index], 7 - bit_index);
    }
}



void il0389_rounder(struct _disp_drv_t * disp_drv, lv_area_t *area) {
    area->x1 = area->x1 & ~(0x7);
    area->x2 = area->x2 |  (0x7);
}


void il0389_sleep_in(void) {
    uint8_t tmp[4];

    ESP_LOGI(TAG, "sleep in");
    tmp[0] = 0xF7; // border floating
    il0389_write_cmd(IL0389_CMD_VCOM_AND_DATA_INTERVAL_SETTING, tmp, 1);
    il0389_write_cmd(IL0389_CMD_POWER_OFF, NULL, 0);
    il0389_waitbusy(IL0389_WAIT);
    tmp[0] = 0xA5; // check cod
    il0389_write_cmd(IL0389_CMD_DEEP_SLEEP, tmp, 1);

    tmp[0] = 0x17;
    il0389_write_cmd(IL0389_CMD_VCOM_AND_DATA_INTERVAL_SETTING, tmp, 1);
    tmp[0] = 0x00;
    il0389_write_cmd(IL0389_CMD_VCM_DC_SETTING, tmp, 1);// to solve Vcom drop
    tmp[0] = 0x02;
    tmp[1] = tmp[2] = tmp[3] = 0x00;
    il0389_write_cmd(IL0389_CMD_POWER_SETTING, tmp, 4);// POWER_SETTING
    il0389_waitbusy(IL0389_WAIT);
    il0389_send_cmd(IL0389_CMD_POWER_OFF);// POWER_OFF
}



// main initialize
void il0389_init(void)
{
    uint8_t tmp[3];

    ESP_LOGI(TAG, "init");

    //Initialize non-SPI GPIOs
    gpio_set_direction(IL0389_DC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(IL0389_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(IL0389_BUSY_PIN,  GPIO_MODE_INPUT);

    gpio_set_level(IL0389_RST_PIN, 0);
    vTaskDelay(IL0389_RESET_DELAY / portTICK_RATE_MS);
    gpio_set_level(IL0389_RST_PIN, 1);
    vTaskDelay(IL0389_RESET_DELAY / portTICK_RATE_MS);

    tmp[0] = 0x17;
    tmp[1] = 0x17;
    tmp[2] = 0x17;
    il0389_write_cmd(IL0389_CMD_BOOSTER_SOFT_START, tmp, 3); //already hardware reset
    il0389_waitbusy(IL0389_WAIT);
    il0389_send_cmd(IL0389_CMD_POWER_ON); //POWER_ON
    il0389_waitbusy(IL0389_WAIT);
    tmp[0] = 0x0F; // PANEL_SETTING LUT from OTP
    il0389_write_cmd(IL0389_CMD_PANEL_SETTING, tmp, 1);

    tmp[0] = 0xF7; // border floating
    il0389_write_cmd(IL0389_CMD_VCOM_AND_DATA_INTERVAL_SETTING, tmp, 1);
    // il0389_clear_cntlr_mem(IL0389_CMD_DATA_START_TRANSMISSION_1, true);
    // il0389_clear_cntlr_mem(IL0389_CMD_DATA_START_TRANSMISSION_2, true);
    // allow partial updates now
    il0389_partial = true;

}




