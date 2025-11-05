/**
 * @file sf04_hal_esp32_idf.h
 * @brief ESP32 ESP-IDF implementation of SF04 HAL
 *
 * Provides ESP32-specific implementations using native ESP-IDF framework
 * Compatible with: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6
 *
 * Requirements:
 * - ESP-IDF v4.0 or later
 * - I2C driver component
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#ifndef SF04_HAL_ESP32_IDF_H
#define SF04_HAL_ESP32_IDF_H

#include "../sf04_hal.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdarg>
#include <cstdio>

namespace SF04HAL {
namespace esp32_idf {

/**
 * @brief ESP32 I2C implementation using ESP-IDF I2C driver
 */
class ESP32IDFI2C : public II2C {
private:
    i2c_port_t port_;
    uint32_t frequency_;
    gpio_num_t sdaPin_;
    gpio_num_t sclPin_;
    bool begun_;
    uint32_t timeout_ms_;

    static constexpr char* TAG = "SF04_I2C";

public:
    /**
     * @brief Constructor
     * @param port I2C port number (I2C_NUM_0 or I2C_NUM_1)
     * @param sdaPin SDA GPIO pin number
     * @param sclPin SCL GPIO pin number
     * @param frequency I2C clock frequency in Hz (default: 100kHz)
     * @param timeout_ms I2C timeout in milliseconds (default: 1000ms)
     */
    ESP32IDFI2C(i2c_port_t port, gpio_num_t sdaPin, gpio_num_t sclPin,
                uint32_t frequency = 100000, uint32_t timeout_ms = 1000)
        : port_(port), frequency_(frequency),
          sdaPin_(sdaPin), sclPin_(sclPin),
          begun_(false), timeout_ms_(timeout_ms) {
    }

    bool begin() override {
        if (!begun_) {
            i2c_config_t conf = {};
            conf.mode = I2C_MODE_MASTER;
            conf.sda_io_num = sdaPin_;
            conf.scl_io_num = sclPin_;
            conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
            conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
            conf.master.clk_speed = frequency_;

            esp_err_t err = i2c_param_config(port_, &conf);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
                return false;
            }

            err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
                return false;
            }

            begun_ = true;
        }
        return true;
    }

    void end() override {
        if (begun_) {
            i2c_driver_delete(port_);
            begun_ = false;
        }
    }

    void setFrequency(uint32_t frequency) override {
        frequency_ = frequency;
        if (begun_) {
            // Reconfigure I2C frequency
            i2c_config_t conf = {};
            conf.mode = I2C_MODE_MASTER;
            conf.sda_io_num = sdaPin_;
            conf.scl_io_num = sclPin_;
            conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
            conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
            conf.master.clk_speed = frequency_;
            i2c_param_config(port_, &conf);
        }
    }

    I2CResult write(uint8_t address, const uint8_t* data,
                   size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        if (!begun_) {
            return I2CResult::BUS_ERROR;
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, (uint8_t*)data, length, true);

        if (sendStop) {
            i2c_master_stop(cmd);
        }

        esp_err_t err = i2c_master_cmd_begin(port_, cmd,
                                             pdMS_TO_TICKS(timeout_ms_));
        i2c_cmd_link_delete(cmd);

        return convertESPError(err);
    }

    I2CResult read(uint8_t address, uint8_t* data,
                  size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        if (!begun_) {
            return I2CResult::BUS_ERROR;
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);

        if (length > 1) {
            i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);

        if (sendStop) {
            i2c_master_stop(cmd);
        }

        esp_err_t err = i2c_master_cmd_begin(port_, cmd,
                                             pdMS_TO_TICKS(timeout_ms_));
        i2c_cmd_link_delete(cmd);

        return convertESPError(err);
    }

    I2CResult writeRead(uint8_t address,
                       const uint8_t* writeData, size_t writeLength,
                       uint8_t* readData, size_t readLength) override {
        if (writeData == nullptr || writeLength == 0 ||
            readData == nullptr || readLength == 0) {
            return I2CResult::INVALID_PARAM;
        }

        if (!begun_) {
            return I2CResult::BUS_ERROR;
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        // Write phase
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, (uint8_t*)writeData, writeLength, true);

        // Repeated start for read
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);

        if (readLength > 1) {
            i2c_master_read(cmd, readData, readLength - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, readData + readLength - 1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(port_, cmd,
                                             pdMS_TO_TICKS(timeout_ms_));
        i2c_cmd_link_delete(cmd);

        return convertESPError(err);
    }

private:
    /**
     * @brief Convert ESP-IDF error to I2C result
     */
    I2CResult convertESPError(esp_err_t err) {
        switch (err) {
            case ESP_OK:
                return I2CResult::SUCCESS;
            case ESP_ERR_TIMEOUT:
                return I2CResult::TIMEOUT;
            case ESP_FAIL:
                return I2CResult::NACK;
            default:
                return I2CResult::BUS_ERROR;
        }
    }
};

/**
 * @brief ESP32 timing implementation using ESP-IDF
 */
class ESP32IDFTiming : public ITiming {
public:
    void delayMicroseconds(uint32_t us) override {
        ets_delay_us(us);
    }

    void delayMilliseconds(uint32_t ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    uint32_t millis() override {
        return (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    uint32_t micros() override {
        return (uint32_t)esp_timer_get_time();
    }
};

/**
 * @brief ESP32 debug output implementation using ESP-IDF logging
 */
class ESP32IDFDebug : public IDebug {
private:
    const char* tag_;
    char buffer_[256];

public:
    /**
     * @brief Constructor
     * @param tag Log tag (default: "SF04")
     */
    ESP32IDFDebug(const char* tag = "SF04")
        : tag_(tag) {
    }

    void print(const char* message) override {
        // Use ESP_LOG without newline
        printf("%s", message);
    }

    void println(const char* message) override {
        ESP_LOGI(tag_, "%s", message);
    }

    void printf(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        vsnprintf(buffer_, sizeof(buffer_), format, args);
        va_end(args);
        ESP_LOGI(tag_, "%s", buffer_);
    }
};

} // namespace esp32_idf
} // namespace SF04HAL

#endif // SF04_HAL_ESP32_IDF_H
