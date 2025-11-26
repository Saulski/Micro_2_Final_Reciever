#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define BUZZER_GPIO    GPIO_NUM_27
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER   LEDC_TIMER_0

#define LED_GPIO       GPIO_NUM_26   // Red LED indicator

// Door/trunk buttons (external pull-downs)
#define DOOR1_GPIO GPIO_NUM_32
#define DOOR2_GPIO GPIO_NUM_33
#define TRUNK_GPIO GPIO_NUM_25

static bool alarm_active = false;
static bool armed_state = false;

// ESP-NOW receive callback
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    printf("Received: %.*s\n", len, data);

    // Read raw states (0 = closed, 1 = open with external pull-downs)
    int door1 = gpio_get_level(DOOR1_GPIO);
    int door2 = gpio_get_level(DOOR2_GPIO);
    int trunk = gpio_get_level(TRUNK_GPIO);

    printf("Door1=%d Door2=%d Trunk=%d\n", door1, door2, trunk);

    if (strncmp((char*)data, "PANIC", len) == 0) {
        alarm_active = true;
        gpio_set_level(LED_GPIO, 1); // LED ON during panic
        printf("Panic alarm triggered\n");

    } else if (strncmp((char*)data, "DISARM", len) == 0) {
        alarm_active = false;
        armed_state = false;
        gpio_set_level(LED_GPIO, 0); // LED OFF
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
        printf("System disarmed\n");

    } else if (strncmp((char*)data, "ARM", len) == 0) {
        if (door1 == 0 && door2 == 0 && trunk == 0) {
            armed_state = true;
            gpio_set_level(LED_GPIO, 1); // LED ON
            printf("System armed successfully\n");

            // One loud beep at max volume
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(300)); // beep duration
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);

        } else {
            armed_state = false;
            gpio_set_level(LED_GPIO, 0); // LED stays OFF
            printf("Arming failed — entry open\n");

            // 3 short, fast beeps at max volume
            for (int i = 0; i < 3; i++) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023); // max duty
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(150)); // short beep

                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0); // silence
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(150)); // short pause
            }
        }
    }
}

// Sawtooth alarm task
void alarm_task(void *pvParameter) {
    while (1) {
        if (alarm_active) {
            for (int duty = 0; duty <= 1023 && alarm_active; duty += 50) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(40));
            }
            if (alarm_active) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(600));
            }
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(400));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

void app_main(void) {
    // Init NVS + Wi-Fi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set custom MAC for receiver
    uint8_t custom_sta_mac[6] = {0x02, 0x00, 0x00, 0xAA, 0xBB, 0x02};
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, custom_sta_mac);
    if (err != ESP_OK) {
        printf("Failed to set MAC, error: %d\n", err);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Print back MAC
    uint8_t mac_sta[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac_sta));
    printf("Receiver STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac_sta[0], mac_sta[1], mac_sta[2], mac_sta[3], mac_sta[4], mac_sta[5]);

    // Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(recv_cb);

    // Configure buzzer PWM
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = BUZZER_TIMER,
        .freq_hz          = 2000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num       = BUZZER_GPIO,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = BUZZER_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = BUZZER_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // Configure red LED
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0); // start OFF

    // Configure door/trunk buttons (external pull-downs → floating in software)
    gpio_set_direction(DOOR1_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DOOR1_GPIO, GPIO_FLOATING);

    gpio_set_direction(DOOR2_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DOOR2_GPIO, GPIO_FLOATING);

    gpio_set_direction(TRUNK_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TRUNK_GPIO, GPIO_FLOATING);

    // Start alarm task
    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 5, NULL);
}