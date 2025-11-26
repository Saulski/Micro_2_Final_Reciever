#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define BUZZER_GPIO GPIO_NUM_27
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER   LEDC_TIMER_0

static bool alarm_active = false;

// ESP-NOW receive callback
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    printf("Received: %.*s\n", len, data);

    if (strncmp((char*)data, "PANIC", len) == 0) {
        alarm_active = true;
        printf("Alarm armed\n");
    } else if (strncmp((char*)data, "DISARM", len) == 0) {
        alarm_active = false;
        // silence immediately
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
        printf("System disarmed\n");
    }
}

// Sawtooth alarm task
void alarm_task(void *pvParameter) {
    while (1) {
        if (alarm_active) {
            // ramp up duty gradually
            for (int duty = 0; duty <= 1023 && alarm_active; duty += 50) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(40)); // controls ramp speed
            }

            // stay at max volume for a while
            if (alarm_active) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(600)); // hold max ~0.6s
            }

            // drop to silence before repeating
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(400)); // pause at silence
        } else {
            vTaskDelay(pdMS_TO_TICKS(200)); // idle
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
        .duty_resolution  = LEDC_TIMER_10_BIT, // 0–1023
        .timer_num        = BUZZER_TIMER,
        .freq_hz          = 2000,              // tone frequency
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

    // Start alarm task
    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 5, NULL);
}