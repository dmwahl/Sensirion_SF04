/**
 * @file sf04_hal_esp32_arduino.h
 * @brief ESP32/ESP8266 Arduino framework implementation of SF04 HAL
 *
 * Provides ESP32/ESP8266-specific implementations using Arduino framework
 * Compatible with:
 * - ESP32 (all variants: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6)
 * - ESP8266
 *
 * Requirements:
 * - Arduino framework for ESP32/ESP8266
 * - Wire library (included in Arduino core)
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#ifndef SF04_HAL_ESP32_ARDUINO_H
#define SF04_HAL_ESP32_ARDUINO_H

#include "../sf04_hal.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstdarg>

namespace SF04HAL {
namespace esp32 {

/**
 * @brief ESP32/ESP8266 I2C implementation using Arduino Wire library
 *
 * Supports custom SDA/SCL pins on ESP32 (not available on ESP8266)
 */
class ESP32I2C : public II2C {
private:
    TwoWire* wire_;
    uint32_t frequency_;
    int8_t sdaPin_;
    int8_t sclPin_;
    bool begun_;

public:
    /**
     * @brief Constructor with default Wire interface
     * @param wire Reference to Wire object (Wire, Wire1, etc.)
     * @param frequency I2C clock frequency in Hz (default: 100kHz)
     */
    ESP32I2C(TwoWire& wire = Wire, uint32_t frequency = 100000)
        : wire_(&wire), frequency_(frequency),
          sdaPin_(-1), sclPin_(-1), begun_(false) {
    }

    /**
     * @brief Constructor with custom pins (ESP32 only)
     * @param wire Reference to Wire object
     * @param sdaPin SDA pin number
     * @param sclPin SCL pin number
     * @param frequency I2C clock frequency in Hz (default: 100kHz)
     */
    ESP32I2C(TwoWire& wire, int8_t sdaPin, int8_t sclPin, uint32_t frequency = 100000)
        : wire_(&wire), frequency_(frequency),
          sdaPin_(sdaPin), sclPin_(sclPin), begun_(false) {
    }

    bool begin() override {
        if (!begun_) {
            #ifdef ESP32
                // ESP32 supports custom pins
                if (sdaPin_ >= 0 && sclPin_ >= 0) {
                    wire_->begin(sdaPin_, sclPin_);
                } else {
                    wire_->begin();
                }
                wire_->setClock(frequency_);
            #else
                // ESP8266 uses default pins (GPIO4=SDA, GPIO5=SCL)
                wire_->begin();
                wire_->setClock(frequency_);
            #endif
            begun_ = true;
        }
        return true;
    }

    void end() override {
        if (begun_) {
            wire_->end();
            begun_ = false;
        }
    }

    void setFrequency(uint32_t frequency) override {
        frequency_ = frequency;
        if (begun_) {
            wire_->setClock(frequency_);
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

        wire_->beginTransmission(address);

        for (size_t i = 0; i < length; i++) {
            wire_->write(data[i]);
        }

        uint8_t result = wire_->endTransmission(sendStop);

        // Wire error codes:
        // 0: success
        // 1: data too long
        // 2: NACK on address
        // 3: NACK on data
        // 4: other error
        // 5: timeout (ESP32 only)
        switch (result) {
            case 0:
                return I2CResult::SUCCESS;
            case 2:
            case 3:
                return I2CResult::NACK;
            case 5:
                return I2CResult::TIMEOUT;
            default:
                return I2CResult::BUS_ERROR;
        }
    }

    I2CResult read(uint8_t address, uint8_t* data,
                  size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        if (!begun_) {
            return I2CResult::BUS_ERROR;
        }

        #ifdef ESP32
            // ESP32 Wire supports timeout parameter
            uint8_t received = wire_->requestFrom(address, (uint8_t)length, sendStop);
        #else
            // ESP8266 doesn't have timeout parameter
            uint8_t received = wire_->requestFrom(address, (uint8_t)length);
            if (sendStop) {
                wire_->endTransmission();
            }
        #endif

        if (received != length) {
            return I2CResult::NACK;
        }

        for (size_t i = 0; i < length; i++) {
            if (wire_->available()) {
                data[i] = wire_->read();
            } else {
                return I2CResult::TIMEOUT;
            }
        }

        return I2CResult::SUCCESS;
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

        // Write phase (with repeated start - no stop)
        I2CResult writeResult = write(address, writeData, writeLength, false);
        if (writeResult != I2CResult::SUCCESS) {
            return writeResult;
        }

        // Read phase
        return read(address, readData, readLength, true);
    }
};

/**
 * @brief ESP32/ESP8266 timing implementation
 */
class ESP32Timing : public ITiming {
public:
    void delayMicroseconds(uint32_t us) override {
        ::delayMicroseconds(us);
    }

    void delayMilliseconds(uint32_t ms) override {
        ::delay(ms);
    }

    uint32_t millis() override {
        return ::millis();
    }

    uint32_t micros() override {
        return ::micros();
    }
};

/**
 * @brief ESP32/ESP8266 debug output implementation
 *
 * Supports Serial, Serial1, Serial2 (ESP32 only)
 */
class ESP32Debug : public IDebug {
private:
    HardwareSerial* serial_;

public:
    /**
     * @brief Constructor with Serial object
     * @param serial Reference to Serial object (Serial, Serial1, Serial2)
     */
    ESP32Debug(HardwareSerial& serial = Serial)
        : serial_(&serial) {
        if (!serial_) {
            serial_->begin(115200);
        }
    }

    void print(const char* message) override {
        if (serial_) {
            serial_->print(message);
        }
    }

    void println(const char* message) override {
        if (serial_) {
            serial_->println(message);
        }
    }

    void printf(const char* format, ...) override {
        if (serial_) {
            char buffer[256];
            va_list args;
            va_start(args, format);
            vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            serial_->print(buffer);
        }
    }
};

} // namespace esp32
} // namespace SF04HAL

#endif // SF04_HAL_ESP32_ARDUINO_H
