/**
 * @file lv_conf.h
 * Configuration file for LVGL 8.3.x
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/* Color depth: 16-bit RGB565 */
#define LV_COLOR_DEPTH 16

/* Enable logging via Serial */
#define LV_USE_LOG 0

/* Memory settings */
#define LV_MEM_CUSTOM 0
#if LV_MEM_CUSTOM == 0
#  define LV_MEM_SIZE    (32U * 1024U)
#endif

/* Tick source: use millis() from Arduino */
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
#  define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#  define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

/* Display refresh period in ms */
#define LV_DISP_DEF_REFR_PERIOD 30

/* Default display and themes */
#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 0

/* Enable animations */
#define LV_USE_ANIMATION 1

/* Fonts used in the project */
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_20  1
#define LV_FONT_MONTSERRAT_28  0
#define LV_FONT_MONTSERRAT_48  0

/* Widgets used */
#define LV_USE_LABEL    1
#define LV_USE_IMG      1
#define LV_USE_ARC      1
#define LV_USE_BTN      1
#define LV_USE_SLIDER   0
#define LV_USE_ROLLER   0

/* GPU acceleration */
#define LV_USE_GPU 0

/* File system (not used) */
#define LV_USE_FILESYSTEM 0

/* Miscellaneous */
#define LV_USE_OBJ_REALIGN          1
#define LV_USE_MSG                  0
#define LV_USE_ASSERT_NULL          0
#define LV_SPRINTF_CUSTOM           0
#define LV_SPRINTF_USE_FLOAT        0

/* Buffer size for display drawing */
#define LV_USE_DRAW_SW 1

/* Use internal memory for display buffer */
#define LV_USE_DRAW_STM32_DMA2D 0

#endif /*LV_CONF_H*/