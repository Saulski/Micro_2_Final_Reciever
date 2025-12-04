#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUZZER_GPIO    GPIO_NUM_27
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_TIMER   LEDC_TIMER_0

#define LED_GPIO       GPIO_NUM_26   // Armed indicator (ON when armed)
#define SERVO_GPIO     GPIO_NUM_18   // FS90R continuous servo
#define BREAKIN_LED    GPIO_NUM_19   // Break-in status indicator (latched until DISARM)

// Door/trunk buttons (external pull-downs)
#define DOOR1_GPIO GPIO_NUM_32
#define DOOR2_GPIO GPIO_NUM_33
#define TRUNK_GPIO GPIO_NUM_25

// ✅ Backseat pressure sensor (external pull-down)
#define PRESSURE_GPIO GPIO_NUM_34

// Tuned neutral pulse width to prevent creep
#define SERVO_NEUTRAL 1495

static bool alarm_active = false;
static bool armed_state = false;
static bool window_down = false; // false = up, true = down
static bool seat_occupied = false;

// --- Servo helpers (continuous rotation FS90R) ---
static inline int us_to_duty(int us) {
    return (us * 8191) / 20000; // 13-bit resolution, 20ms period
}

static inline void servo_set_us(int us) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, us_to_duty(us));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

// --- ESP-NOW receive callback ---
void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    printf("Received: %.*s\n", len, data);

    int door1 = gpio_get_level(DOOR1_GPIO);
    int door2 = gpio_get_level(DOOR2_GPIO);
    int trunk = gpio_get_level(TRUNK_GPIO);
    seat_occupied = gpio_get_level(PRESSURE_GPIO);   // ✅ read seat sensor

    if (strncmp((char*)data, "PANIC", len) == 0) {
        alarm_active = true;
        printf("Panic alarm triggered\n");

    } else if (strncmp((char*)data, "DISARM", len) == 0) {
        alarm_active = false;
        armed_state = false;
        gpio_set_level(LED_GPIO, 0);
        gpio_set_level(BREAKIN_LED, 0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
        printf("System disarmed\n");

    } else if (strncmp((char*)data, "ARM", len) == 0) {

        // ✅ Now includes seat_occupied check
        if (door1 == 0 && door2 == 0 && trunk == 0 && !window_down && !seat_occupied) {

            armed_state = true;
            gpio_set_level(LED_GPIO, 1);
            printf("System armed successfully\n");

            // One loud beep
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(300));
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);

        } else {
            armed_state = false;
            gpio_set_level(LED_GPIO, 0);

            // ✅ Correct failure reason
            if (seat_occupied) {
                printf("Arming failed — backseat occupied\n");
            } else if (window_down) {
                printf("Arming failed — window is down\n");
            } else {
                printf("Arming failed — entry open\n");
            }

            // 3 short beeps
            for (int i = 0; i < 3; i++) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(150));
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
                vTaskDelay(pdMS_TO_TICKS(150));
            }
        }

    } else if (strncmp((char*)data, "WINDOW", len) == 0) {
        if (!armed_state) {
            if (!window_down) {
                printf("Rolling window down...\n");
                servo_set_us(1400);
                vTaskDelay(pdMS_TO_TICKS(2000));
                servo_set_us(SERVO_NEUTRAL);
                window_down = true;
            } else {
                printf("Rolling window up...\n");
                servo_set_us(1600);
                vTaskDelay(pdMS_TO_TICKS(2000));
                servo_set_us(SERVO_NEUTRAL);
                window_down = false;
            }

            // short beep feedback
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 1023);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(200));
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, BUZZER_CHANNEL);

        } else {
            printf("Cannot roll windows while armed\n");
        }
    }
}

// --- Sawtooth alarm task ---
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

// --- Continuous monitoring task (unchanged) ---
void monitor_task(void *pvParameter) {
    const TickType_t poll_ms = pdMS_TO_TICKS(50);
    const int debounce_samples = 4;
    int door1_hist = 0, door2_hist = 0, trunk_hist = 0, window_hist = 0;

    while (1) {
        int d1 = gpio_get_level(DOOR1_GPIO);
        int d2 = gpio_get_level(DOOR2_GPIO);
        int tr = gpio_get_level(TRUNK_GPIO);
        int win = window_down ? 1 : 0;

        door1_hist = ((door1_hist << 1) | (d1 & 1)) & ((1 << debounce_samples) - 1);
        door2_hist = ((door2_hist << 1) | (d2 & 1)) & ((1 << debounce_samples) - 1);
        trunk_hist = ((trunk_hist << 1) | (tr & 1)) & ((1 << debounce_samples) - 1);
        window_hist = ((window_hist << 1) | (win & 1)) & ((1 << debounce_samples) - 1);

        if (armed_state && !alarm_active) {
            bool door1_open = (door1_hist == ((1 << debounce_samples) - 1));
            bool door2_open = (door2_hist == ((1 << debounce_samples) - 1));
            bool trunk_open = (trunk_hist == ((1 << debounce_samples) - 1));
            bool window_open = (window_hist == ((1 << debounce_samples) - 1));

            if (door1_open || door2_open || trunk_open || window_open) {
                alarm_active = true;
                gpio_set_level(BREAKIN_LED, 1);
                printf("Unauthorized opening detected — alarm engaged\n");
            }
        }

        vTaskDelay(poll_ms);
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

    uint8_t custom_sta_mac[6] = {0x02, 0x00, 0x00, 0xAA, 0xBB, 0x02};
    esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, custom_sta_mac);
    if (err != ESP_OK) printf("Failed to set MAC, error: %d\n", err);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

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

    // Indicators
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    gpio_set_direction(BREAKIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(BREAKIN_LED, 0);

    // Door/trunk inputs
    gpio_set_direction(DOOR1_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DOOR1_GPIO, GPIO_FLOATING);
    gpio_set_direction(DOOR2_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DOOR2_GPIO, GPIO_FLOATING);
    gpio_set_direction(TRUNK_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(TRUNK_GPIO, GPIO_FLOATING);

    // ✅ Backseat pressure sensor
    gpio_set_direction(PRESSURE_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PRESSURE_GPIO, GPIO_FLOATING);

    // Servo PWM
    ledc_timer_config_t servo_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

    ledc_channel_config_t servo_channel = {
        .gpio_num       = SERVO_GPIO,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_1,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));

    servo_set_us(SERVO_NEUTRAL);

    xTaskCreate(alarm_task, "alarm_task", 2048, NULL, 5, NULL);
    xTaskCreate(monitor_task, "monitor_task", 2048, NULL, 6, NULL);

    printf("Receiver ready. Monitoring entry points and backseat sensor...\n");
}