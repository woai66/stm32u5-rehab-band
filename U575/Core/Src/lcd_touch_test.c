#include "lcd_touch_test.h"
#include "lcd_st7789.h"
#include "touch_cst816t.h"

static void LCD_DrawCross(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= 6U) {
        LCD_Fill(x - 6U, y, x + 6U, y, color);
    }
    if (y >= 6U) {
        LCD_Fill(x, y - 6U, x, y + 6U, color);
    }
    LCD_DrawRect((x > 8U) ? (x - 8U) : 0U, (y > 8U) ? (y - 8U) : 0U, 17U, 17U, color);
}

void LCD_ColorBarTest(void)
{
    const uint16_t colors[] = {
        LCD_COLOR_RED,
        LCD_COLOR_GREEN,
        LCD_COLOR_BLUE,
        LCD_COLOR_WHITE,
        LCD_COLOR_BLACK,
        LCD_COLOR_YELLOW,
        LCD_COLOR_CYAN,
        LCD_COLOR_MAGENTA
    };
    const uint16_t bar_h = LCD_HEIGHT / (sizeof(colors) / sizeof(colors[0]));

    for (uint16_t i = 0; i < (sizeof(colors) / sizeof(colors[0])); i++) {
        uint16_t y0 = i * bar_h;
        uint16_t y1 = (i == ((sizeof(colors) / sizeof(colors[0])) - 1U)) ? (LCD_HEIGHT - 1U) : (y0 + bar_h - 1U);
        LCD_Fill(0, y0, LCD_WIDTH - 1U, y1, colors[i]);
    }

    LCD_DrawRect(0, 0, LCD_WIDTH, LCD_HEIGHT, LCD_COLOR_WHITE);
    LCD_DrawRect(8, 8, LCD_WIDTH - 16U, LCD_HEIGHT - 16U, LCD_COLOR_BLACK);
}

void LCD_TouchPointTest(void)
{
    CST816T_TouchPoint_t point;
    static uint16_t last_x = 0;
    static uint16_t last_y = 0;
    static uint8_t has_last = 0;

    if (CST816T_ReadPoint(&point) != HAL_OK) {
        return;
    }

    if (point.detected != 0U) {
        if (has_last != 0U) {
            LCD_DrawCross(last_x, last_y, LCD_COLOR_BLACK);
        }
        LCD_DrawCross(point.x, point.y, LCD_COLOR_WHITE);
        last_x = point.x;
        last_y = point.y;
        has_last = 1U;
    }
}

void LCD_Touch_TestTask(void)
{
    LCD_Init();
    (void)CST816T_Init();
    LCD_ColorBarTest();

    for (;;) {
        LCD_TouchPointTest();
        HAL_Delay(20);
    }
}