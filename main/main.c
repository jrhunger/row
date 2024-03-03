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
 * GPIO9: input  - rotor reed sensor - pulled up, interrupt from rising edge.
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
#define GPIO_INPUT_IO_0  GPIO_NUM_9
#define ESP_INTR_FLAG_DEFAULT 0

// Savitzky-Golay linear/quadratic https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter
static int coefficient[9] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
// the trailing delay values
static int64_t delayvector[9] = {1024,1024,1024,1024,1024,1024,1024,1024,1024};
static int derivative = 0;
static int64_t last_pull = 0;
static uint32_t pull_duration = 1; // avoid divide-by-zero when calculating stroke rate
static void add_delay(int delay) {
    // do the last calculation first to initialize deriv
    int deriv = delay * coefficient[0];
    for (int count = 8; count > 0; count-- ) {
        delayvector[count] = delayvector[count-1];
        deriv += delayvector[count] * coefficient[count];
    }
    delayvector[0] = delay;
    // negative derivative means accelerating since we are working with the delay
    if (derivative >= 0 && deriv < 0) {
        pull_duration = esp_timer_get_time() - last_pull;
        last_pull = esp_timer_get_time();
    }
    // don't actually care about the real derivative, just the sign of it
    //derivative = deriv / normalization;
    derivative = deriv;
}

// GPIO read logic
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    //uint32_t gpio_num = (uint32_t) arg;
    uint32_t trigger_time = esp_timer_get_time();
    xQueueSendFromISR(gpio_evt_queue, &trigger_time, NULL);
}

static uint32_t trigger_count = 0;
static uint32_t last_trigger = 0;
static void gpio_task(void* arg) {
    uint32_t trigger_time;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &trigger_time, portMAX_DELAY)) {
            trigger_count++;
            add_delay(trigger_time - last_trigger);
            last_trigger = trigger_time;
        }
    }
}

//static int adc_raw[2][10];
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
    hd44780_puts(&lcd, "time  pwr   dist");

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    static char line1[16];
    static char line2[16];
    int64_t last_time = esp_timer_get_time();
    //static unsigned char scaled_adc = 0;
    static uint32_t distance = 0;
    static uint32_t elapsed = 0;
    static uint32_t rest = 0;
    static int64_t now = 0;
    static unsigned char min = 0;
    static unsigned char sec = 0;
    static unsigned char restmin = 0;
    static unsigned char restsec = 0;
    static char elapsed_string[8];
    static char resting_string[8];
    static char strokes_string[6];
    static char power_string[6];
    static char distance_string[11];
    static char pull_string[11];
    static bool active = false;
    static uint32_t interval_triggers = 0;
    static uint32_t strokes = 0;
    static unsigned short pull_rate = 0;
    trigger_count = 0;
    while (1) {
        now = esp_timer_get_time();
        if (now - last_time >= 500000) { // every half-second
            // grab and reset trigger count to not lose triggers during processing
            interval_triggers = trigger_count;
            trigger_count = 0;

            // first check if state needs to change
            if (active) {
                // check if not active
                if (interval_triggers == 0) {
                    printf("switching to rest\n");
                    pull_rate = 0;
                    active = false;
                }
            }
            else {
                // check if active
                if (interval_triggers > 0) {
                    printf("switching to active\n");
                    active = true;
                    rest = 0;
                }
            }

            // then process per state
            if (active) {
                pull_rate = 60000000/pull_duration;
                distance += interval_triggers;
                // show total elapsed training time
                elapsed += (now - last_time);
            }
            else {
                // show resting time
                rest += (now - last_time);
                pull_rate = 0;
            }

            min = elapsed / 60000000;
            sec = (elapsed / 1000000) % 60;
            restmin = rest / 60000000;
            restsec = (rest / 1000000) % 60;
                
            //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[0][0]));
            // ADC is 0 to 4095.  right-shift 4 takes it to 0-255
            //scaled_adc = (unsigned char) adc_raw[0][0]>>4;

            // create output line 1
            snprintf(resting_string, 8, "%02d:%02d", restmin, restsec);
            snprintf(strokes_string, 6, "%3d", pull_rate);
            snprintf(pull_string, 11, "%"PRIu32, strokes);
            snprintf(line1, 17, "%.5s%4.4s%7.7s", resting_string, strokes_string, pull_string);
            printf("%s\n", line1);

            // create output line 2
            snprintf(elapsed_string, 8, "%02d:%02d", min, sec);
            snprintf(power_string, 6, "%3d", (unsigned short) interval_triggers);
            snprintf(distance_string, 11, "%"PRIu32, distance);
            snprintf(line2, 17, "%.5s%4.4s%7.7s", elapsed_string, power_string, distance_string);

            // print to stdout
            //printf("%"PRIu32"\n", (uint32_t) (now - last_time));
            printf("%s\n", line2);
            // display on LCD
            hd44780_gotoxy(&lcd, 0, 0);
            hd44780_puts(&lcd, line1);
            hd44780_gotoxy(&lcd, 0, 1);
            hd44780_puts(&lcd, line2);
            last_time = now;
        }

        // allow other tasks time to run
        vTaskDelay(10);
    }
}