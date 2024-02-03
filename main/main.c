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
 * GPIO5:  input, pulled up, interrupt from rising edge.
 * GPIO19: output (ESP32C2/ESP32H2 uses GPIO9 as the second output pin)
 * - can be connected to GPIO5 to simulate the reed switch
 * GPIO2 / ADC1 channel 2: potentiometer
 *
 */

/*
 * Let's say, GPIO_INPUT_IO_0=5, in binary representation,
 * 1ULL<<GPIO_INPUT_IO_1 is equal to 0000000000000000000000000000000000100000
 * */
#define GPIO_INPUT_IO_0     CONFIG_GPIO_INPUT_0
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)

#define GPIO_OUTPUT_IO_0    CONFIG_GPIO_OUTPUT_0
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

#define ESP_INTR_FLAG_DEFAULT 0

// GPIO read logic
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static uint32_t trigger_count = 0;
static void gpio_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == 5) {
                trigger_count++;
            }
        }
    }
}

void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    
    // configure test output pin
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    
    // configure reed input pin
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.intr_type = GPIO_INTR_POSEDGE; // interrupt on rising edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    int interval = 0;
    int last_time = esp_timer_get_time();
    while (1) {
        if (esp_timer_get_time() - last_time > 1000000) { // once / second
            printf("interval: %d, triggers: %"PRIu32"\n", interval++, trigger_count);
            trigger_count = 0;
            last_time = esp_timer_get_time();
        }

        gpio_set_level(GPIO_OUTPUT_IO_0, 0);
        gpio_set_level(GPIO_OUTPUT_IO_0, 1);
        vTaskDelay(5);
    }
}