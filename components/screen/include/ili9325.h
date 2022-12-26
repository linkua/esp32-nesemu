// Copyright 2020 Espressif Systems (Shanghai) Co. Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _IOT_LCD_ILI9325_H_
#define _IOT_LCD_ILI9325_H_

#define ILI9325_TFTWIDTH		240
#define ILI9325_TFTHEIGHT		320

/* Color definitions, RGB565 format */
#define COLOR_BLACK       0x0000
#define COLOR_NAVY        0x000F
#define COLOR_DARKGREEN   0x03E0
#define COLOR_DARKCYAN    0x03EF
#define COLOR_MAROON      0x7800
#define COLOR_PURPLE      0x780F
#define COLOR_OLIVE       0x7BE0
#define COLOR_LIGHTGREY   0xC618
#define COLOR_DARKGREY    0x7BEF
#define COLOR_BLUE        0x001F
#define COLOR_GREEN       0x07E0
#define COLOR_CYAN        0x07FF
#define COLOR_RED         0xF800
#define COLOR_MAGENTA     0xF81F
#define COLOR_YELLOW      0xFFE0
#define COLOR_WHITE       0xFFFF
#define COLOR_ORANGE      0xFD20
#define COLOR_GREENYELLOW 0xAFE5
#define COLOR_PINK        0xF81F
#define COLOR_SILVER      0xC618
#define COLOR_GRAY        0x8410
#define COLOR_LIME        0x07E0
#define COLOR_TEAL        0x0410
#define COLOR_FUCHSIA     0xF81F
#define COLOR_ESP_BKGD    0xD185

#ifdef __cplusplus
extern "C" {
#endif

void lcd_ili9325_init(void);

void lcd_ili9325_set_rotation(int dir);

void lcd_ili9325_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

void lcd_ili9325_write_data(uint16_t *pdata, uint16_t len);

void lcd_ili9325_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

void lcd_ili9325_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bitmap);

void lcd_ili9325_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color); 


#ifdef __cplusplus
}
#endif

#endif
