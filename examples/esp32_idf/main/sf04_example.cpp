/**
 * @file sf04_example.cpp
 * @brief ESP32 ESP-IDF example for Sensirion SF04 flow sensor
 *
 * This example demonstrates how to use the SF04 driver on ESP32
 * using the native ESP-IDF framework
 *
 * Hardware connections:
 * - GPIO21 (SDA) -> SF04 SDA
 * - GPIO22 (SCL) -> SF04 SCL
 * - 3.3V         -> SF04 VDD
 * - GND          -> SF04 GND
 *
 * Build & Flash:
 * idf.py build
 * idf.py -p PORT flash monitor
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "sensirion_sf04_v2.h"
#include "hal/esp32/sf04_hal_esp32_idf.h"

// I2C configuration
#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

// Sensor configuration
#define SF04_ADDR                   0x40
#define LED_GPIO                    GPIO_NUM_2

static const char *TAG = "SF04_EXAMPLE";

// Create HAL instances
SF04HAL::esp32_idf::ESP32IDFI2C halI2C(
    I2C_MASTER_NUM,
    I2C_MASTER_SDA_IO,
    I2C_MASTER_SCL_IO,
    I2C_MASTER_FREQ_HZ
);

SF04HAL::esp32_idf::ESP32IDFTiming halTiming;
SF04HAL::esp32_idf::ESP32IDFDebug halDebug("SF04");

// Create SF04 sensor instance
SF04 sensor(halI2C, halTiming, SF04_ADDR, 0, 16, &halDebug);

/**
 * @brief Initialize LED
 */
static void init_led(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

/**
 * @brief Toggle LED
 */
static void toggle_led(void)
{
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level(LED_GPIO, led_state ? 1 : 0);
}

/**
 * @brief Sensor measurement task
 */
static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "=== Sensirion SF04 Flow Sensor Example ===");
    ESP_LOGI(TAG, "Platform: ESP32 ESP-IDF");
    ESP_LOGI(TAG, "Starting up...\n");

    if (sensor.ready) {
        ESP_LOGI(TAG, "Sensor initialized successfully!");
        ESP_LOGI(TAG, "Serial Number: %lu", sensor.serialNumber.u32);
        ESP_LOGI(TAG, "Scale Factor: %u", sensor.scaleFactor.u16);
        ESP_LOGI(TAG, "Flow Unit: %s\n", sensor.flowUnitStr);
    } else {
        ESP_LOGE(TAG, "Sensor initialization failed!");
        ESP_LOGE(TAG, "Check connections and I2C address");
        while(1) {
            toggle_led();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    ESP_LOGI(TAG, "Starting measurements...\n");

    while (1) {
        // Toggle LED to show activity
        toggle_led();

        // Perform measurements
        u8t errorFlow = sensor.Measure(FLOW);
        u8t errorTemp = sensor.Measure(TEMP);
        u8t errorVdd = sensor.Measure(VDD);

        if (errorFlow == 0 && errorTemp == 0 && errorVdd == 0) {
            // Print raw values
            ESP_LOGI(TAG, "Raw - Flow: %d, Temp: %d, Vdd: %u",
                     sensor.flow.i16, sensor.temperature.i16, sensor.vdd.u16);

            // Print converted values
            ESP_LOGI(TAG, "Flow: %.2f %s | Temp: %.1f°C | Vdd: %.2fV",
                     sensor.getFlow(),
                     sensor.flowUnitStr,
                     sensor.getTemperature(),
                     sensor.getVdd());

            // Print memory info
            ESP_LOGI(TAG, "Free heap: %lu bytes\n", esp_get_free_heap_size());
        } else {
            ESP_LOGE(TAG, "Measurement failed (Flow=0x%02X, Temp=0x%02X, Vdd=0x%02X)",
                     errorFlow, errorTemp, errorVdd);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
    }
}

extern "C" void app_main(void)
{
    // Initialize LED
    init_led();

    // Create sensor measurement task
    xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5,
        NULL
    );

    // Optional: Create additional tasks for WiFi, web server, etc.
}
