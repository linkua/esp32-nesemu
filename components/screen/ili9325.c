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
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"

#include "ili9325.h"


#ifdef CONFIG_HW_I2S_ENA
#include "i2s_lcd_driver.h"

#define ILI9325_I2S_NUM    I2S_NUM_0

i2s_lcd_handle_t _i2s_lcd_handle;
#endif

#define LCD_D0_PIN	(1)  
#define LCD_D1_PIN	(3)  
#define LCD_D2_PIN	(23)
#define LCD_D3_PIN	(22)
#define LCD_D4_PIN	(21)
#define LCD_D5_PIN	(19)
#define LCD_D6_PIN	(18)  
#define LCD_D7_PIN	(5)  
#define LCD_D8_PIN	(16)   
#define LCD_D9_PIN	(17)  
#define LCD_D10_PIN	(0)
#define LCD_D11_PIN	(4)
#define LCD_D12_PIN	(15)
#define LCD_D13_PIN	(2)
#define LCD_D14_PIN	(13)
#define LCD_D15_PIN	(12)

#define LCD_RD_PIN	(-1)  
#define LCD_WR_PIN	(14)  
#define LCD_RS_PIN	(27)
#define LCD_CS_PIN	(-1)
#define LCD_REST_PIN    (-1)

#define ILI9325_IOlWrite(a, b) \
	do { \
		if (b) { \
		    GPIO.out_w1ts = (1 << a); \
		} else { \
		    GPIO.out_w1tc = (1 << a); \
		} \
	} while (0)
	
#define LCD_RS_PIN_HIGH()    GPIO.out_w1ts = (1 << LCD_RS_PIN)
#define LCD_RS_PIN_LOW()     GPIO.out_w1tc = (1 << LCD_RS_PIN)
#define LCD_WR_PIN_HIGH()    GPIO.out_w1ts = (1 << LCD_WR_PIN)
#define LCD_WR_PIN_LOW()     GPIO.out_w1tc = (1 << LCD_WR_PIN)
	
static int8_t rotation = 0;
	
static int8_t _data_num[16] = {
            LCD_D0_PIN,  LCD_D1_PIN,  LCD_D2_PIN,  LCD_D3_PIN,
            LCD_D4_PIN,  LCD_D5_PIN,  LCD_D6_PIN,  LCD_D7_PIN,
            LCD_D8_PIN,  LCD_D9_PIN,  LCD_D10_PIN, LCD_D11_PIN,
            LCD_D12_PIN, LCD_D13_PIN, LCD_D14_PIN, LCD_D15_PIN
        };

void _writeData16b(uint16_t data) 
{
#ifdef CONFIG_HW_I2S_ENA
    i2s_lcd_write(_i2s_lcd_handle, (const uint8_t *)&data, 2);
#else	
    //cs
    // rs ->1
    LCD_RS_PIN_HIGH();   
	
	for(int i=0;i<16;i++) {
		ILI9325_IOlWrite(_data_num[i], data & (1ul << i));
	}

    // wr 0--> 1
    LCD_WR_PIN_LOW();   
    LCD_WR_PIN_HIGH();
#endif
}

void _writeCmd16b(uint16_t cmd) 
{ 
#ifdef CONFIG_HW_I2S_ENA
    i2s_lcd_write_command(_i2s_lcd_handle, (const uint8_t *)&cmd, 2);
#else
    // cs
    // rs ->0
    LCD_RS_PIN_LOW();
    
	for(int i=0;i<16;i++) {
		ILI9325_IOlWrite(_data_num[i], cmd & (1ul << i));
	}	
	
    // wr 0--> 1
    LCD_WR_PIN_LOW();   
    LCD_WR_PIN_HIGH();
#endif
}

void _writeReg16b(uint16_t cmd, uint16_t data) 
{
    _writeCmd16b(cmd);
    _writeData16b(data);
}

void _writeData(uint16_t *pData, uint16_t len) 
{
#ifdef CONFIG_HW_I2S_ENA
    i2s_lcd_acquire(_i2s_lcd_handle);
    i2s_lcd_write(_i2s_lcd_handle, (const uint8_t *)pData, 2*len);
    i2s_lcd_release(_i2s_lcd_handle);
#else
    for(int i=0; i<len; i++)
    {
        _writeData16b(pData[i]);
    }
#endif
}

void _initReg(void) {    
    // SM=0 SS=1 GS=1 
    //_writeReg16b(0x00E5, 0x78F0);   // set SRAM internal timing
    _writeReg16b(0x00E3, 0x3008);     // Set internal timing
    _writeReg16b(0x00E7, 0x0012);     // Set internal timing 
    _writeReg16b(0x00EF, 0x1231);     // Set internal timing 
    _writeReg16b(0x0001, 0x0100);     // set Driver Output Control SM=0 SS=1
    _writeReg16b(0x0002, 0x0700);     // set 1 line inversion
    _writeReg16b(0x0003, 0x1030);     // set GRAM _write direction and BGR=1.
    _writeReg16b(0x0004, 0x0000);     // Resize register
    _writeReg16b(0x0005, 0x0000);     // .kbv 16bits Data Format Selection
    _writeReg16b(0x0008, 0x0207);     // set the back porch and front porch
    _writeReg16b(0x0009, 0x0000);     // set non-display area refresh cycle ISC[3:0]
    _writeReg16b(0x000A, 0x0000);     // FMARK function
    _writeReg16b(0x000C, 0x0000);     // RGB interface setting
    _writeReg16b(0x000D, 0x0000);     // Frame marker Position
    _writeReg16b(0x000F, 0x0000);     // RGB interface polarity
    // ----------- Power On sequence ----------- //
    _writeReg16b(0x0010, 0x0000);     // SAP, BT[3:0], AP, DSTB, SLP, STB
    _writeReg16b(0x0011, 0x0007);     // DC1[2:0], DC0[2:0], VC[2:0]
    _writeReg16b(0x0012, 0x0000);     // VREG1OUT voltage
    _writeReg16b(0x0013, 0x0000);     // VDV[4:0] for VCOM amplitude
    vTaskDelay(200 / portTICK_RATE_MS);                      // Dis-charge capacitor power voltage
    _writeReg16b(0x0010, 0x1690);     // SAP=1, BT=6, APE=1, AP=1, DSTB=0, SLP=0, STB=0
    _writeReg16b(0x0011, 0x0227);     // DC1=2, DC0=2, VC=7
    vTaskDelay(50 / portTICK_RATE_MS);                       // wait_ms 50ms
    _writeReg16b(0x0012, 0x000D);     // VCIRE=1, PON=0, VRH=5
    vTaskDelay(50 / portTICK_RATE_MS);                       // wait_ms 50ms
    _writeReg16b(0x0013, 0x1200);     // VDV=28 for VCOM amplitude
    _writeReg16b(0x0029, 0x000A);     // VCM=10 for VCOMH
    _writeReg16b(0x002B, 0x000D);     // Set Frame Rate
    vTaskDelay(50 / portTICK_RATE_MS);                       // wait_ms 50ms
    _writeReg16b(0x0020, 0x0000);     // GRAM horizontal Address
    _writeReg16b(0x0021, 0x0000);     // GRAM Vertical Address
    
    // ----------- Adjust the Gamma Curve ----------//
    _writeReg16b(0x0030, 0x0000);
    _writeReg16b(0x0031, 0x0404);
    _writeReg16b(0x0032, 0x0003);
    _writeReg16b(0x0035, 0x0405);
    _writeReg16b(0x0036, 0x0808);
    _writeReg16b(0x0037, 0x0407);
    _writeReg16b(0x0038, 0x0303);
    _writeReg16b(0x0039, 0x0707);
    _writeReg16b(0x003C, 0x0504);
    _writeReg16b(0x003D, 0x0808);

    //------------------ Set GRAM area ---------------//
    _writeReg16b(0x0050, 0x0000);     // Horizontal GRAM Start Address 
    _writeReg16b(0x0051, 0x00EF);     // Horizontal GRAM End Address
    _writeReg16b(0x0052, 0x0000);     // Vertical GRAM Start Address
    _writeReg16b(0x0053, 0x013F);     // Vertical GRAM Start Address 
    _writeReg16b(0x0060, 0xA700);     // Gate Scan Line 
    _writeReg16b(0x0061, 0x0001);     // NDL,VLE, REV 
    _writeReg16b(0x006A, 0x0000);     // set scrolling line 
    
    //-------------- Partial Display Control ---------//
    _writeReg16b(0x0080, 0x0000);
    _writeReg16b(0x0081, 0x0000);
    _writeReg16b(0x0082, 0x0000);
    _writeReg16b(0x0083, 0x0000);
    _writeReg16b(0x0084, 0x0000);
    _writeReg16b(0x0085, 0x0000);
    
    //-------------- Panel Control -------------------//
    _writeReg16b(0x0090,0x0010); 
    _writeReg16b(0x0092,0x0600); 
    _writeReg16b(0x0007,0x0133);   // 262K color and display ON
}


void lcd_ili9325_init(void)
{
#ifdef CONFIG_HW_I2S_ENA
    i2s_lcd_config_t _i2s_lcd_cfg = {
        .data_width  = 16,
        .pin_data_num = {
            _data_num[0],
            _data_num[1],
            _data_num[2],
            _data_num[3],
            _data_num[4],
            _data_num[5],
            _data_num[6],
            _data_num[7],
            _data_num[8],
            _data_num[9],
            _data_num[10],
            _data_num[11],
            _data_num[12],
            _data_num[13],
            _data_num[14],
            _data_num[15],
        },
        .pin_num_cs = -1,  //cs
        .pin_num_wr = LCD_WR_PIN,
        .pin_num_rs = LCD_RS_PIN,
		
        .clk_freq = 10000000,
        .i2s_port = ILI9325_I2S_NUM,
        .swap_data = false,
        .buffer_size = 32000,
    };
  
    _i2s_lcd_handle = i2s_lcd_driver_init(&_i2s_lcd_cfg);
#else
    //init gpio
    gpio_config_t gpioconf= {
			.pin_bit_mask=(1<<LCD_D0_PIN)|(1<<LCD_D1_PIN)|(1<<LCD_D2_PIN)|(1<<LCD_D3_PIN)|      \
			              (1<<LCD_D4_PIN)|(1<<LCD_D5_PIN)|(1<<LCD_D6_PIN)|(1<<LCD_D7_PIN)|      \
			              (1<<LCD_D8_PIN)|(1<<LCD_D9_PIN)|(1<<LCD_D10_PIN)|(1<<LCD_D11_PIN)|    \
			              (1<<LCD_D12_PIN)|(1<<LCD_D13_PIN)|(1<<LCD_D14_PIN)|(1<<LCD_D15_PIN)|  \
			              (1<<LCD_WR_PIN)|(1<<LCD_RS_PIN), 
			.mode=GPIO_MODE_OUTPUT, 
			.pull_up_en=GPIO_PULLUP_DISABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		};                               
    
    gpio_config(&gpioconf);
    
#endif	
  _initReg();
}

void lcd_ili9325_set_rotation(int dir)
{
    rotation = dir % 4; 
    
    switch (rotation) {
        case 0:
            _writeReg16b(0x0003,(1<<12)|(1<<5)|(1<<4)|(0<<3));
            break;
        case 1:  //90
            _writeReg16b(0x0003,(1<<12)|(0<<5)|(1<<4)|(1<<3));
            break;
        case 2:  //180
            _writeReg16b(0x0003,(1<<12)|(0<<5)|(0<<4)|(0<<3));
            break;
        case 3:  //270
            _writeReg16b(0x0003,(1<<12)|(1<<5)|(0<<4)|(1<<3));                   
            break;
    }	
}

void _setCursor(uint16_t x, uint16_t y) 
{
    switch (rotation) {
        case 0:
            _writeReg16b(0x0020, x);
            _writeReg16b(0x0021, y);
            break;
        case 1:  //90
            _writeReg16b(0x0020, y);
            _writeReg16b(0x0021, ILI9325_TFTHEIGHT - 1 - x); 
            break;
        case 2:  //180
            _writeReg16b(0x0020, ILI9325_TFTWIDTH - 1 -x);
            _writeReg16b(0x0021, ILI9325_TFTHEIGHT - 1 -y);
            break;
        case 3:  //270
            _writeReg16b(0x0020, ILI9325_TFTWIDTH - 1 - y);
            _writeReg16b(0x0021, x);               
            break;
    }  
}

void lcd_ili9325_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    switch (rotation) {
        case 0:
            _writeReg16b(0x0050, x0);
            _writeReg16b(0x0051, x1); 
            _writeReg16b(0x0052, y0);
            _writeReg16b(0x0053, y1);
            
            break;
        case 1:  //90
            _writeReg16b(0x0050, y0);
            _writeReg16b(0x0051, y1); 
            _writeReg16b(0x0052, ILI9325_TFTHEIGHT - 1 - x1);
            _writeReg16b(0x0053, ILI9325_TFTHEIGHT - 1 - x0);
            break;
        case 2:  //180
            _writeReg16b(0x0050, ILI9325_TFTWIDTH - 1 -x1);
            _writeReg16b(0x0051, ILI9325_TFTWIDTH -1 - x0); 
            _writeReg16b(0x0052, ILI9325_TFTHEIGHT - 1 -y1);
            _writeReg16b(0x0053, ILI9325_TFTHEIGHT - 1 -y0);
            break;
        case 3:  //270
            _writeReg16b(0x0050, ILI9325_TFTWIDTH - 1 - y1);
            _writeReg16b(0x0051, ILI9325_TFTWIDTH - 1 - y0); 
            _writeReg16b(0x0052, x0);             
            _writeReg16b(0x0053, x1);            
            break;
    }

    _setCursor(x0, y0);
    _writeCmd16b(0x0022);     // Write Data to GRAM  
}

void lcd_ili9325_write_data(uint16_t *pdata, uint16_t len)
{
    _writeData(pdata, len);
}

void lcd_ili9325_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{	
    _setCursor(x, y);
    _writeReg16b(0x0022, color); // Write Data to GRAM
}

void lcd_ili9325_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
    uint32_t len;
	
    len = w * h;

    lcd_ili9325_set_window(x, y, x + w - 1, y + h - 1);

    _writeData((uint8_t *)bitmap, len);
}

void lcd_ili9325_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
  lcd_ili9325_set_window(x, y, x+w-1, y+h-1);

#ifdef CONFIG_HW_I2S_ENA
  uint16_t _buf[320];
  int bufSize = w>h?w:h;
  for (int i=0;i<bufSize;i++) _buf[i]=color;
  
  if (w > h)
  {
    for (y=h; y>0; y--) {
      _writeData(_buf, w); 
    }
  }
  else
  {
    for (x=w; x>0; x--) {
      _writeData(_buf, h);
    }
  }
#else
   LCD_RS_PIN_HIGH();

  for(int i=0;i<16;i++) {
    ILI9325_IOlWrite(_data_num[i], (color & (1ul << i))? 1:0);  
  }

  for (y=h; y>0; y--) {
    for (x=w; x>0; x--) {
      // _writeData16b(color);
      LCD_WR_PIN_LOW();
      LCD_WR_PIN_HIGH();
    }
  }
  #endif
}
