/*
 * DeskKnob hardware pin map (ESP32-S3).
 *
 * These values describe the physical DeskKnob v1 board and must stay in sync
 * with the schematic. Every other component includes this header instead of
 * hard-coding GPIO numbers.
 */
#pragma once

#include "driver/gpio.h"

/* -------- Round LCD (GC9A01 240x240, SPI) -------- */
#define BOARD_LCD_SPI_HOST   SPI2_HOST
#define BOARD_LCD_SCLK       GPIO_NUM_12
#define BOARD_LCD_MOSI       GPIO_NUM_13
#define BOARD_LCD_CS         GPIO_NUM_11
#define BOARD_LCD_DC         GPIO_NUM_10
#define BOARD_LCD_RST        GPIO_NUM_14
#define BOARD_LCD_BL         GPIO_NUM_9   /* backlight, active high (LEDC) */
#define BOARD_LCD_H_RES      240
#define BOARD_LCD_V_RES      240

/* -------- AS5600 magnetic angle sensor (I2C) -------- */
#define BOARD_I2C_PORT       I2C_NUM_0
#define BOARD_I2C_SDA        GPIO_NUM_19
#define BOARD_I2C_SCL        GPIO_NUM_8
#define BOARD_AS5600_ADDR    0x36

/* -------- Input -------- */
#define BOARD_PIN_HX710_DOUT GPIO_NUM_35  /* pressure film data (input)  */
#define BOARD_PIN_HX710_SCK  GPIO_NUM_36  /* pressure film clock (output) */
#define BOARD_PIN_BUTTON     GPIO_NUM_4   /* push button, active low (back/exit) */

/* -------- WS2812 ring -------- */
#define BOARD_PIN_LED_RING   GPIO_NUM_47
#define BOARD_LED_RING_NUM   8

/* -------- TMC6300 / BLDC 6-PWM --------
 * Pin order matches BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, en)
 */
#define BOARD_MOTOR_POLE_PAIRS 7
#define BOARD_MOTOR_PWM_AH   GPIO_NUM_40
#define BOARD_MOTOR_PWM_AL   GPIO_NUM_38
#define BOARD_MOTOR_PWM_BH   GPIO_NUM_41
#define BOARD_MOTOR_PWM_BL   GPIO_NUM_37
#define BOARD_MOTOR_PWM_CH   GPIO_NUM_42
#define BOARD_MOTOR_PWM_CL   GPIO_NUM_39
#define BOARD_MOTOR_EN       GPIO_NUM_45
