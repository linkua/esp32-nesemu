// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/periph_ctrl.h"
#include "spi_lcd.h"
#include "ili9325.h"

#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

extern uint16_t myPalette[];

void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[])
{
    uint16_t lcd_buf[320];
    
    if(data == NULL)
    {
        lcd_ili9325_fillRect(0, 0, 320, 240, COLOR_BLACK); 
    }    
      
    for (int y=0; y<height; y++) {
        lcd_ili9325_set_window(xs, ys+y, xs+width-1, ys+height-1);
        
        for (int x = 0; x < width; x++) {
            lcd_buf[x] = myPalette[(unsigned char)(data[y][x])];
        }
        lcd_ili9325_write_data(lcd_buf, width);        
    }
}

void ili9341_init()
{
    vTaskDelay(100 / portTICK_RATE_MS);
    lcd_ili9325_init();
    lcd_ili9325_set_rotation(1);
    lcd_ili9325_fillRect(0, 0, 320, 240, COLOR_BLACK); 
}

void ili9341_test()
{
    lcd_ili9325_set_rotation(0);
    lcd_ili9325_fillRect(0, 0, 240, 320, COLOR_BLACK);
    vTaskDelay(100 / portTICK_RATE_MS);    
    lcd_ili9325_fillRect(0, 0, 240, 320, COLOR_WHITE);
    vTaskDelay(100 / portTICK_RATE_MS);
    lcd_ili9325_fillRect(0, 0, 240, 320, COLOR_RED);
    vTaskDelay(100 / portTICK_RATE_MS);    
    lcd_ili9325_fillRect(0, 0, 240, 320, COLOR_GREEN);
    vTaskDelay(100 / portTICK_RATE_MS);          
    lcd_ili9325_fillRect(0, 0, 240, 320, COLOR_BLUE);
    vTaskDelay(100 / portTICK_RATE_MS);         
}




