/**
 * @file il0389.h
 *
 */

#ifndef IL0389_H
#define IL0389_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl/lvgl.h"
#include "sdkconfig.h"

// Defined in lvgl_helpers.h
// #define DISP_BUF_SIZE       (CONFIG_LVGL_DISPLAY_HEIGHT*IL0389_COLUMNS)

#define IL0389_COLUMNS      (CONFIG_LVGL_DISPLAY_WIDTH / 8)
#define IL0389_PIXEL        (CONFIG_LVGL_DISPLAY_WIDTH * CONFIG_LVGL_DISPLAY_HEIGHT)

#define IL0389_DC_PIN          CONFIG_LVGL_DISP_PIN_DC
#define IL0389_RST_PIN         CONFIG_LVGL_DISP_PIN_RST
#define IL0389_BUSY_PIN        CONFIG_LVGL_DISP_PIN_BUSY
#define IL0389_BUSY_LEVEL      1




/* IL0389 commands */
#define IL0389_CMD_PANEL_SETTING                       0x00
#define IL0389_CMD_POWER_SETTING                       0x01
#define IL0389_CMD_POWER_OFF                           0x02
#define IL0389_CMD_POWER_OFF_SEQUENCE_SETTING          0x03
#define IL0389_CMD_POWER_ON                            0x04
#define IL0389_CMD_POWER_ON_MEASURE                    0x05
#define IL0389_CMD_BOOSTER_SOFT_START                  0x06
#define IL0389_CMD_DEEP_SLEEP                          0x07
#define IL0389_CMD_DATA_START_TRANSMISSION_1           0x10
#define IL0389_CMD_DATA_STOP                           0x11
#define IL0389_CMD_DISPLAY_REFRESH                     0x12
#define IL0389_CMD_DATA_START_TRANSMISSION_2           0x13
#define IL0389_CMD_LUT_FOR_VCOM                        0x20
#define IL0389_CMD_LUT_WHITE_TO_WHITE                  0x21
#define IL0389_CMD_LUT_BLACK_TO_WHITE                  0x22
#define IL0389_CMD_LUT_WHITE_TO_BLACK                  0x23
#define IL0389_CMD_LUT_BLACK_TO_BLACK                  0x24
#define IL0389_CMD_PLL_CONTROL                         0x30
#define IL0389_CMD_TEMPERATURE_SENSOR_COMMAND          0x40
#define IL0389_CMD_TEMPERATURE_SENSOR_SELECTION        0x41
#define IL0389_CMD_TEMPERATURE_SENSOR_WRITE            0x42
#define IL0389_CMD_TEMPERATURE_SENSOR_READ             0x43
#define IL0389_CMD_VCOM_AND_DATA_INTERVAL_SETTING      0x50
#define IL0389_CMD_LOW_POWER_DETECTION                 0x51
#define IL0389_CMD_TCON_SETTING                        0x60
#define IL0389_CMD_RESOLUTION_SETTING                  0x61
#define IL0389_CMD_GSST_SETTING                        0x65
#define IL0389_CMD_GET_STATUS                          0x71
#define IL0389_CMD_AUTO_MEASUREMENT_VCOM               0x80
#define IL0389_CMD_READ_VCOM_VALUE                     0x81
#define IL0389_CMD_VCM_DC_SETTING                      0x82
#define IL0389_CMD_PARTIAL_WINDOW                      0x90
#define IL0389_CMD_PARTIAL_IN                          0x91
#define IL0389_CMD_PARTIAL_OUT                         0x92
#define IL0389_CMD_PROGRAM_MODE                        0xA0
#define IL0389_CMD_ACTIVE_PROGRAMMING                  0xA1
#define IL0389_CMD_READ_OTP                            0xA2
#define IL0389_CMD_POWER_SAVING                        0xE3

/* Data entry sequence modes */
#define IL0389_DATA_ENTRY_MASK                         0x07
#define IL0389_DATA_ENTRY_XDYDX                        0x00
#define IL0389_DATA_ENTRY_XIYDX                        0x01
#define IL0389_DATA_ENTRY_XDYIX                        0x02
#define IL0389_DATA_ENTRY_XIYIX                        0x03
#define IL0389_DATA_ENTRY_XDYDY                        0x04
#define IL0389_DATA_ENTRY_XIYDY                        0x05
#define IL0389_DATA_ENTRY_XDYIY                        0x06
#define IL0389_DATA_ENTRY_XIYIY                        0x07

/* Options for display update */


/* Options for display update sequence */


/* time constants in ms */
#define IL0389_RESET_DELAY         200
#define IL0389_BUSY_DELAY          1
// normal wait time max 200ms
#define IL0389_WAIT                20

void il0389_init(void);
void il0389_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void il0389_fullflush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void il0389_rounder(struct _disp_drv_t * disp_drv, lv_area_t *area);
void il0389_set_px_cb(struct _disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa);
void il0389_sleep_in(void);

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* __IL0389_REGS_H__ */

