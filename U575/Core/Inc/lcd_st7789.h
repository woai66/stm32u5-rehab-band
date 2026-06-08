#ifndef __LCD_ST7789_H__
#define __LCD_ST7789_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_WIDTH   240U
#define LCD_HEIGHT  284U

#define LCD_COLOR_WHITE      0xFFFFU
#define LCD_COLOR_BLACK      0x0000U
#define LCD_COLOR_BLUE       0x001FU
#define LCD_COLOR_RED        0xF800U
#define LCD_COLOR_GREEN      0x07E0U
#define LCD_COLOR_CYAN       0x07FFU
#define LCD_COLOR_MAGENTA    0xF81FU
#define LCD_COLOR_YELLOW     0xFFE0U
#define LCD_COLOR_GRAY       0x8410U
#define LCD_COLOR_DARKGRAY   0x4208U

void LCD_Init(void);
void LCD_SetBacklight(uint8_t on);
void LCD_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void LCD_FillScreen(uint16_t color);
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif