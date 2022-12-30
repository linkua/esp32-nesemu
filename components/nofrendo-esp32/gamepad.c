#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "gamepad.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"

#define PSX_S_M     ADC1_CHANNEL_7  //35
#define PSX_A_X     ADC1_CHANNEL_4  //32
#define PSX_B_Y     ADC1_CHANNEL_6  //34
#define PSX_L_R     ADC1_CHANNEL_0  //36
#define PSX_U_D     ADC1_CHANNEL_3  //39

static volatile bool input_task_is_running = false;
static volatile input_gamepad_state gamepad_state;
static input_gamepad_state previous_gamepad_state;
static uint8_t debounce[GAMEPAD_INPUT_MAX];
static volatile bool input_gamepad_initialized = false;
static SemaphoreHandle_t xSemaphore;

input_gamepad_state gamepad_input_read_raw()
{
    input_gamepad_state state = {0};

    int joySM = adc1_get_raw(PSX_S_M);
    int joyX = adc1_get_raw(PSX_L_R);
    int joyY = adc1_get_raw(PSX_U_D);
    int joyA = adc1_get_raw(PSX_A_X);
    int joyB = adc1_get_raw(PSX_B_Y);    
    
    // slect menu
    if (joySM > 2048 + 1024)
    {
        state.values[GAMEPAD_INPUT_SELECT] = 0;
        state.values[GAMEPAD_INPUT_START] = 0;
    }
    else if ( joySM > 1024)
    {
        state.values[GAMEPAD_INPUT_SELECT] = 1;
        state.values[GAMEPAD_INPUT_START] = 0;
    }
    else 
    {
        state.values[GAMEPAD_INPUT_SELECT] = 0;
        state.values[GAMEPAD_INPUT_START] = 1;
    }

    // left right
    if (joyX > 2048 + 1024)
    {
        state.values[GAMEPAD_INPUT_LEFT] = 0;
        state.values[GAMEPAD_INPUT_RIGHT] = 0;
    }
    else if (joyX > 1024)
    {
        state.values[GAMEPAD_INPUT_LEFT] = 0;
        state.values[GAMEPAD_INPUT_RIGHT] = 1;
    }
    else
    {
        state.values[GAMEPAD_INPUT_LEFT] = 1;
        state.values[GAMEPAD_INPUT_RIGHT] = 0;
    }

    // up down
    if (joyY > 2048 + 1024)
    {
        state.values[GAMEPAD_INPUT_UP] = 0;
        state.values[GAMEPAD_INPUT_DOWN] = 0;
    }
    else if (joyY > 1024)
    {
        state.values[GAMEPAD_INPUT_UP] = 1;
        state.values[GAMEPAD_INPUT_DOWN] = 0;
    }
    else
    {
        state.values[GAMEPAD_INPUT_UP] = 0;
        state.values[GAMEPAD_INPUT_DOWN] = 1;
    }

    // A
    if (joyA > 2048 + 1024)
    {
        state.values[GAMEPAD_INPUT_A] = 0;
    }
    else if (joyA > 1024)
    {
        state.values[GAMEPAD_INPUT_A] = 1;
    }
    else
    {
        state.values[GAMEPAD_INPUT_A] = 1;
    }

    // B
    if (joyB > 2048 + 1024)
    {
        state.values[GAMEPAD_INPUT_B] = 0;
    }
    else if (joyB > 1024)
    {
        state.values[GAMEPAD_INPUT_B] = 1;
    }
    else
    {
        state.values[GAMEPAD_INPUT_B] = 1;
    }

    return state;
}

static void input_task(void *arg)
{
    input_task_is_running = true;

    // Initialize state
    for (int i = 0; i < GAMEPAD_INPUT_MAX; ++i)
    {
        debounce[i] = 0xff;
    }

    while (input_task_is_running)
    {
        // Shift current values
        for (int i = 0; i < GAMEPAD_INPUT_MAX; ++i)
        {
            debounce[i] <<= 1;
        }

        // Read hardware
        input_gamepad_state state = gamepad_input_read_raw();

        // Debounce
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        for (int i = 0; i < GAMEPAD_INPUT_MAX; ++i)
        {
            debounce[i] |= state.values[i] ? 1 : 0;
            uint8_t val = debounce[i] & 0x03; //0x0f;
            switch (val)
            {
            case 0x00:
                gamepad_state.values[i] = 0;
                break;

            case 0x03: //0x0f:
                gamepad_state.values[i] = 1;
                break;

            default:
                // ignore
                break;
            }
        }

        previous_gamepad_state = gamepad_state;

        xSemaphoreGive(xSemaphore);

        // delay
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    input_gamepad_initialized = false;

    vSemaphoreDelete(xSemaphore);

    // Remove the task from scheduler
    vTaskDelete(NULL);

    // Never return
    while (1)
    {
        vTaskDelay(1);
    }
}

void gamepad_read(input_gamepad_state *out_state)
{
    if (!input_gamepad_initialized)
        abort();

    xSemaphoreTake(xSemaphore, portMAX_DELAY);

    *out_state = gamepad_state;

    xSemaphoreGive(xSemaphore);
}

void gamepad_init()
{
    xSemaphore = xSemaphoreCreateMutex();

    if (xSemaphore == NULL)
    {
        printf("xSemaphoreCreateMutex failed.\n");
        abort();
    }

    adc1_config_width(ADC_WIDTH_12Bit);    
    adc1_config_channel_atten(PSX_L_R, ADC_ATTEN_11db);        
    adc1_config_channel_atten(PSX_U_D, ADC_ATTEN_11db);
    adc1_config_channel_atten(PSX_S_M, ADC_ATTEN_11db);         
    adc1_config_channel_atten(PSX_A_X, ADC_ATTEN_11db);
    adc1_config_channel_atten(PSX_B_Y, ADC_ATTEN_11db);

    input_gamepad_initialized = true;

    // Start background polling
    xTaskCreatePinnedToCore(&input_task, "input_task", 1024 * 2, NULL, 5, NULL, 1);

    printf("input_gamepad_init done.\n");
}

void input_gamepad_terminate()
{
    if (!input_gamepad_initialized)
        abort();

    input_task_is_running = false;
}
