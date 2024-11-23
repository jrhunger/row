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
#include "custom_chars.h"

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
            last_trigger = trigger_time;
        }
    }
}

// https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter
// note - not skipping division by normalization because we only care about the sign
static int coeff[5] = {-2, -1, 0, 1, 2};
static int samples[5] = {0, 0, 0, 0, 0};
static int derivative(int sample) { 
    int deriv = sample * coeff[4];
    for (int i = 0; i<4; i++) {
        samples[i] = samples[i+1];
        deriv += samples[i] * coeff[i];
    }
    samples[4] = sample;
    //printf("--> sample %d, deriv %d\n", sample, deriv);
    return deriv;
}

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

static void init_display() {
    ESP_ERROR_CHECK(hd44780_init(&lcd));
    hd44780_upload_character(&lcd, 0, cchar0);
    hd44780_upload_character(&lcd, 1, cchar1);
    hd44780_upload_character(&lcd, 2, cchar2);
    hd44780_upload_character(&lcd, 3, cchar3);
    hd44780_upload_character(&lcd, 4, cchar4);
    hd44780_upload_character(&lcd, 5, cchar5);

    hd44780_gotoxy(&lcd,0,0);
    // 2x16            "1234567890123456"
    hd44780_puts(&lcd, "time  pwr   dist");
}

static uint32_t pacer_time;
static int32_t target_usec;
static void init_pacer() {
    pacer_time = 0;
    target_usec = 60 * 1000000 / 26;
}

static void update_pacer () {
    int strokeprogress;
    int bars;
    uint32_t now = (uint32_t) esp_timer_get_time();  
    if ((now - pacer_time) >= target_usec) {
        printf("%"PRIu32"\n", now - pacer_time);
        pacer_time = now;
        strokeprogress = 0;
        bars = 0;
    }
    else {
        strokeprogress = (now - pacer_time) * 50 / target_usec;
        if (strokeprogress > 25) {
            bars = 50 - strokeprogress;
        }
        else {
            bars = strokeprogress;
        }
    }

    for (int i = 1; i <= 5; i++) {
        hd44780_gotoxy(&lcd, i-1, 0);
        if(bars >= i * 5) {
            hd44780_putc(&lcd, 5);
        }
        else if (bars <= (i-1)*5) {
            hd44780_putc(&lcd, 0);
        }
        else {
            hd44780_putc(&lcd, bars % 5 + 1);
        }
    }
}

static void update_display(
                    //line 1
                    unsigned char restmin, 
                    unsigned char restsec, 
                    unsigned short pull_rate,
                    uint32_t strokes,
                    // line 2
                    unsigned char min, 
                    unsigned char sec,
                    uint32_t interval_triggers,
                    uint32_t distance
) {
    static char elapsed_string[8];
    static char resting_string[8];
    static char strokes_string[6];
    static char power_string[6];
    static char distance_string[11];
    static char pull_string[11];
    static char line[16];

    // create output line 0
    snprintf(strokes_string, 6, "%3d", pull_rate);
    snprintf(pull_string, 11, "%"PRIu32, strokes);
    if (restmin != 0 || restsec != 0) {
        snprintf(resting_string, 8, "%02d:%02d", restmin, restsec);
        snprintf(line, 17, "%.5s%4.4s%7.7s", resting_string, strokes_string, pull_string);
        hd44780_gotoxy(&lcd, 0, 0);
    }
    else {
        hd44780_gotoxy(&lcd, 5, 0);
        snprintf(line, 12, "%4.4s%7.7s", strokes_string, pull_string);
    }
    //printf("%s\n", line);
    // display on LCD line 0
    hd44780_puts(&lcd, line);

    // create output line 1
    snprintf(elapsed_string, 8, "%02d:%02d", min, sec);
    snprintf(power_string, 6, "%3d", (unsigned short) interval_triggers);
    snprintf(distance_string, 11, "%"PRIu32, distance);
    snprintf(line, 17, "%.5s%4.4s%7.7s", elapsed_string, power_string, distance_string);
    //printf("%s\n", line);
    // display on LCD line 1
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, line);
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

    init_display();
    init_pacer();

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    //static unsigned char scaled_adc = 0;
    static uint32_t distance = 0;
    static uint32_t elapsed = 0;
    static uint32_t rest = 0;
    // time variables
    static int64_t last_time = 0;
    static int64_t now = 0;
    static int64_t last_pull = 0;
    static int64_t last_display = 0;
    static unsigned char min = 0;
    static unsigned char sec = 0;
    static unsigned char restmin = 0;
    static unsigned char restsec = 0;
    static bool active = false;
    static uint32_t interval_triggers = 0;
    static uint32_t strokes = 0;
    static unsigned short pull_rate = 0;
    static bool pull = false;
    trigger_count = 0;
    last_time = esp_timer_get_time();
    while (1) {
        now = esp_timer_get_time();
        if (now - last_time >= 200000) { // every 1/5 second
            // grab and reset trigger count to not lose triggers during processing
            interval_triggers = trigger_count;
            trigger_count = 0;

            // first check if state needs to change
            if (active) {
                // check if not active
                if (interval_triggers == 0) {
                    printf("--> rest\n");
                    pull_rate = 0;
                    active = false;
                }
                update_pacer();
            }
            else {
                // check if active
                if (interval_triggers > 0) {
                    printf("--> active\n");
                    active = true;
                    rest = 0;
                }
            }

            // then process per state
            if (active) {
                distance += interval_triggers;
                // show total elapsed training time
                elapsed += (now - last_time);

                // check stroke phase (roughly)
                if (pull) {
                    if (derivative(interval_triggers) <= 0) {
                        pull = false;
                    }
                }
                else {
                    if (derivative(interval_triggers) > 0) {
                        // ignore as false positive if stroke rate seems too high
                        if (60000000/(now - last_pull) < 40) {
                            strokes += 1;
                            pull = true;
                            pull_rate = 60000000/(now - last_pull);
                            last_pull = now;
                            printf("-->       pull (%03d, %03d, %03d, %03d, %03d)\n", samples[0], samples[1], samples[2], samples[3], samples[4]);
                        } else {
                            printf("--> false-pull (%03d, %03d, %03d, %03d, %03d)\n", samples[0], samples[1], samples[2], samples[3], samples[4]);
                        }
                    }
                }
            }
            else {
                // add zero samples to derivative vector
                derivative(0);
                rest += (now - last_time);
                pull = false;
            }

            if (now - last_display >= 590000) {
                min = elapsed / 60000000;
                sec = (elapsed / 1000000) % 60;
                restmin = rest / 60000000;
                restsec = (rest / 1000000) % 60;
                    
                //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[0][0]));
                // ADC is 0 to 4095.  right-shift 4 takes it to 0-255
                //scaled_adc = (unsigned char) adc_raw[0][0]>>4;

                update_display(
                    //line 1
                    restmin, restsec, 
                    pull_rate,
                    strokes,
                    // line 2
                    min, sec,
                    interval_triggers,
                    distance
                );
                last_display = now;
            }
            last_time = now;
        }

        // allow other tasks time to run
        vTaskDelay(10);
    }
}
