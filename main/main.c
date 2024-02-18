/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "hd44780.h"

/**
 * Brief: Replacement controller for NordicTrack RW600 Rower
 * 
 * eventual functionality:
 * - Read the reed-switch to determine row speed
 *   - maybe display on a local screen
 *   - or serve up on an API for web access
 * - Read the magnetic-resistance-adjust potentiometer to determine position
 * - Drive the magnetic-resistance-adjust motor to adjust resistance
 *
 * GPIO status:
 * // Reed Sensor (can connect to GPIO21/U0TXD for test input)
 * GPIO8: input  - rotor reed sensor - pulled up, interrupt from rising edge.
 * 
 * // Motor control
 * GPIO2 / ADC1 channel 2: control potentiometer
 * GPIO3 / ADC1 channel 3: motor feedback potentiometer
 * GPIO0: output - motor forward
 * GPIO1: output - motor reverse
 * 
 * // LCD HD4478
 * GPIO18: output - LCD EN
 * GPIO19: output - LCD RS
 * GPIO04: output - LCD D4
 * GPIO05: output - LCD D5
 * GPIO06: output - LCD D6
 * GPIO07: output - LCD D7
 */

//*** Manual GPIO Config
//#define GPIO_INPUT_IO_0     CONFIG_GPIO_REED_INPUT
#define GPIO_INPUT_IO_0  GPIO_NUM_8
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)

#define ESP_INTR_FLAG_DEFAULT 0

// GPIO read logic
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    //uint32_t gpio_num = (uint32_t) arg;
    uint32_t trigger_time = esp_timer_get_time();
    xQueueSendFromISR(gpio_evt_queue, &trigger_time, NULL);
}

static uint32_t trigger_count = 0;
static uint32_t last_trigger = 0;
static void gpio_task(void* arg)
{
    uint32_t trigger_time;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &trigger_time, portMAX_DELAY)) {
            trigger_count++;
            last_trigger = trigger_time;
        }
    }
}

static int adc_raw[2][10];
void app_main(void)
{
    // initialize the reed input switch GPIO
    gpio_pullup_en(GPIO_INPUT_IO_0);
    gpio_set_direction(GPIO_INPUT_IO_0, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);
    gpio_intr_enable(GPIO_INPUT_IO_0);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    // ADC1 Init
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 Channel Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));

    hd44780_t lcd = {
        .write_cb = NULL,
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .e  = GPIO_NUM_18,
            .rs = GPIO_NUM_19,
            .d4 = GPIO_NUM_4,
            .d5 = GPIO_NUM_5,
            .d6 = GPIO_NUM_6,
            .d7 = GPIO_NUM_7,
            .bl = HD44780_NOT_USED
        }
    };
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_gotoxy(&lcd,0,0);
    // 2x16            "1234567890123456"
    hd44780_puts(&lcd, "sec  pwr    dist");

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    static char line2[16];
    int64_t last_time = esp_timer_get_time();
    //static unsigned char scaled_adc = 0;
    static uint32_t distance = 0;
    static uint32_t elapsed = 0;
    static unsigned char min = 0;
    static unsigned char sec = 0;
    static char elapsed_string[8];
    static char power_string[6];
    static char distance_string[11];
    trigger_count = 0;
    while (1) {
        if (esp_timer_get_time() - last_time > 500000) { // every half-second
            last_time = esp_timer_get_time();
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[0][0]));
            // ADC is 0 to 4095.  right-shift 4 takes it to 0-255
            //scaled_adc = (unsigned char) adc_raw[0][0]>>4;
            distance += trigger_count;
            elapsed = last_time / 1000000;
            min = elapsed / 60;
            sec = elapsed % 60;
            //printf("elapsed: %02d:%02d, triggers: %5d, distance %8lu\n", min, sec, (unsigned short) trigger_count, (unsigned long) distance);
            snprintf(elapsed_string, 8, "%02d:%02d", min, sec);
            snprintf(power_string, 6, "%3d", (unsigned short) trigger_count);
            snprintf(distance_string, 11, "%"PRIu32, distance);
            //snprintf(line2, 17, "%.5s%4d%7d", mmss, (unsigned short) trigger_count, (unsigned short) distance);
            snprintf(line2, 17, "%.5s%4.4s%7.7s", elapsed_string, power_string, distance_string);
            printf("time  pwr   dist\n");
            printf("%s\n", line2);
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, line2);
            trigger_count = 0;
        }

        // allow other tasks time to run
        vTaskDelay(10);
    }
}