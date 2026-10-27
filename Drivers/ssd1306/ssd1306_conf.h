#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

/* STM32 microcontroller family */
#define STM32F1

/* Communication interface */
#define SSD1306_USE_I2C

/* I2C configuration */
#define SSD1306_I2C_PORT hi2c1
#define SSD1306_I2C_ADDR (0x3C << 1)

/* OLED resolution */
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

/* Fonts */
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26
#define SSD1306_INCLUDE_FONT_16x24
#define SSD1306_INCLUDE_FONT_16x15

#endif
