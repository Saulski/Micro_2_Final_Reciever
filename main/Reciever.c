#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define BUZZER_GPIO GPIO_NUM_27

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    printf("Received: %.*s\n", len, data);

    if (strncmp((char*)data, "PANIC", len) == 0) {
        for (int i = 0; i < 5; i++) {
            gpio_set_level(BUZZER_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(BUZZER_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
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

    // Configure buzzer
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
}