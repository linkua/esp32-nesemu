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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "psxcontroller.h"
#include "sdkconfig.h"


#if CONFIG_HW_PSX_ENA

#define PSX_CLK CONFIG_HW_PSX_CLK
#define PSX_DAT CONFIG_HW_PSX_DAT
#define PSX_ATT CONFIG_HW_PSX_ATT
#define PSX_CMD CONFIG_HW_PSX_CMD

#define DELAY() asm("nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;nop; nop; nop; nop;")

/* Sends and receives a byte from/to the PSX controller using SPI */
static int psxSendRecv(int send) {
	int x;
	int ret=0;
	volatile int delay;
	
#if 0
	while(1) {
		GPIO.out_w1ts=(1<<PSX_CMD);
		GPIO.out_w1ts=(1<<PSX_CLK);
		GPIO.out_w1tc=(1<<PSX_CMD);
		GPIO.out_w1tc=(1<<PSX_CLK);
	}
#endif

	GPIO.out_w1tc=(1<<PSX_ATT);
	for (delay=0; delay<100; delay++);
	for (x=0; x<8; x++) {
		if (send&1) {
			GPIO.out_w1ts=(1<<PSX_CMD);
		} else {
			GPIO.out_w1tc=(1<<PSX_CMD);
		}
		DELAY();
		for (delay=0; delay<100; delay++);
		GPIO.out_w1tc=(1<<PSX_CLK);
		for (delay=0; delay<100; delay++);
		GPIO.out_w1ts=(1<<PSX_CLK);
		ret>>=1;
		send>>=1;
		if (GPIO.in&(1<<PSX_DAT)) ret|=128;
	}
	return ret;
}

static void psxDone() {
	DELAY();
	GPIO_REG_WRITE(GPIO_OUT_W1TS_REG, (1<<PSX_ATT));
}


int psxReadInput() {
	int b1, b2;

	psxSendRecv(0x01); //wake up
	psxSendRecv(0x42); //get data
	psxSendRecv(0xff); //should return 0x5a
	b1=psxSendRecv(0xff); //buttons byte 1
	b2=psxSendRecv(0xff); //buttons byte 2
	psxDone();
	return (b2<<8)|b1;

}


void psxcontrollerInit() {
	volatile int delay;
	int t;
	gpio_config_t gpioconf[2]={
		{
			.pin_bit_mask=(1<<PSX_CLK)|(1<<PSX_CMD)|(1<<PSX_ATT), 
			.mode=GPIO_MODE_OUTPUT, 
			.pull_up_en=GPIO_PULLUP_DISABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		},{
			.pin_bit_mask=(1<<PSX_DAT), 
			.mode=GPIO_MODE_INPUT, 
			.pull_up_en=GPIO_PULLUP_ENABLE, 
			.pull_down_en=GPIO_PULLDOWN_DISABLE, 
			.intr_type=GPIO_PIN_INTR_DISABLE
		}
	};
	gpio_config(&gpioconf[0]);
	gpio_config(&gpioconf[1]);
	
	//Send a few dummy bytes to clean the pipes.
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++) DELAY();
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++) DELAY();
	//Try and detect the type of controller, so we can give the user some diagnostics.
	psxSendRecv(0x01);
	t=psxSendRecv(0x00);
	psxDone();
	if (t==0 || t==0xff) {
		printf("No PSX/PS2 controller detected (0x%X). You will not be able to control the game.\n", t);
	} else {
		printf("PSX controller type 0x%X\n", t);
	}
}


#else

#include "gamepad.h"

int psxReadInput() {
    input_gamepad_state state;
    gamepad_read(&state);

    int result = 0;

    // A
    if (!state.values[GAMEPAD_INPUT_A])
    {
        result |= (1 << 13);
    }

    // B
    if (!state.values[GAMEPAD_INPUT_B])
    {
        result |= (1 << 14);
    }

    // select
    if (!state.values[GAMEPAD_INPUT_SELECT])
        result |= (1 << 0);

    // start
    if (!state.values[GAMEPAD_INPUT_START])
        result |= (1 << 3);

    // right
    if (!state.values[GAMEPAD_INPUT_RIGHT])
        result |= (1 << 5);

    // left
    if (!state.values[GAMEPAD_INPUT_LEFT])
        result |= (1 << 7);

    // up
    if (!state.values[GAMEPAD_INPUT_UP])
        result |= (1 << 4);

    // down
    if (!state.values[GAMEPAD_INPUT_DOWN])
        result |= (1 << 6);

    return result;
}


void psxcontrollerInit() {
	printf("PSX controller disabled in menuconfig; no input enabled.\n");
	gamepad_init();     
}

#endif
