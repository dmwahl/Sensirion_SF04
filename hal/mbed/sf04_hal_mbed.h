/**
 * @file sf04_hal_mbed.h
 * @brief mbed OS implementation of SF04 HAL
 *
 * Provides mbed-specific implementations of I2C and timing interfaces
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#ifndef SF04_HAL_MBED_H
#define SF04_HAL_MBED_H

#include "../sf04_hal.h"
#include "mbed.h"
#include <cstdarg>

namespace SF04HAL {
namespace mbed {

/**
 * @brief mbed I2C implementation
 */
class MbedI2C : public II2C {
private:
    I2C& i2c_;
    uint32_t frequency_;

public:
    /**
     * @brief Constructor
     * @param i2c Reference to mbed I2C object
     * @param frequency I2C clock frequency in Hz (default: 100kHz)
     */
    MbedI2C(I2C& i2c, uint32_t frequency = 100000)
        : i2c_(i2c), frequency_(frequency) {
    }

    bool begin() override {
        i2c_.frequency(frequency_);
        return true;
    }

    void end() override {
        // mbed I2C doesn't require explicit cleanup
    }

    void setFrequency(uint32_t frequency) override {
        frequency_ = frequency;
        i2c_.frequency(frequency_);
    }

    I2CResult write(uint8_t address, const uint8_t* data,
                   size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        // mbed uses 8-bit address (7-bit << 1)
        int result = i2c_.write(address << 1, (const char*)data,
                               length, !sendStop);

        return (result == 0) ? I2CResult::SUCCESS : I2CResult::NACK;
    }

    I2CResult read(uint8_t address, uint8_t* data,
                  size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        // mbed uses 8-bit address (7-bit << 1 | 1)
        int result = i2c_.read(address << 1, (char*)data, length, !sendStop);

        return (result == 0) ? I2CResult::SUCCESS : I2CResult::NACK;
    }

    I2CResult writeRead(uint8_t address,
                       const uint8_t* writeData, size_t writeLength,
                       uint8_t* readData, size_t readLength) override {
        if (writeData == nullptr || writeLength == 0 ||
            readData == nullptr || readLength == 0) {
            return I2CResult::INVALID_PARAM;
        }

        // Write with repeated start (no stop)
        i2c_.start();
        int ack = i2c_.write(address << 1);
        if (ack != 1) {
            i2c_.stop();
            return I2CResult::NACK;
        }

        for (size_t i = 0; i < writeLength; i++) {
            ack = i2c_.write(writeData[i]);
            if (ack != 1) {
                i2c_.stop();
                return I2CResult::NACK;
            }
        }

        // Repeated start for read
        i2c_.start();
        ack = i2c_.write((address << 1) | 1);
        if (ack != 1) {
            i2c_.stop();
            return I2CResult::NACK;
        }

        for (size_t i = 0; i < readLength; i++) {
            readData[i] = i2c_.read(i < readLength - 1);
        }

        i2c_.stop();
        return I2CResult::SUCCESS;
    }
};

/**
 * @brief mbed timing implementation
 */
class MbedTiming : public ITiming {
private:
    Timer timer_;

public:
    MbedTiming() {
        timer_.start();
    }

    void delayMicroseconds(uint32_t us) override {
        wait_us(us);
    }

    void delayMilliseconds(uint32_t ms) override {
        wait_us(ms * 1000);
    }

    uint32_t millis() override {
        return timer_.read_ms();
    }

    uint32_t micros() override {
        return timer_.read_us();
    }
};

/**
 * @brief mbed debug output implementation
 */
class MbedDebug : public IDebug {
private:
    Serial* serial_;
    bool ownsSerial_;

public:
    /**
     * @brief Constructor with existing Serial object
     */
    MbedDebug(Serial& serial) : serial_(&serial), ownsSerial_(false) {
    }

    /**
     * @brief Constructor - creates Serial on default pins
     */
    MbedDebug(PinName tx = USBTX, PinName rx = USBRX, int baud = 115200)
        : ownsSerial_(true) {
        serial_ = new Serial(tx, rx, baud);
    }

    ~MbedDebug() {
        if (ownsSerial_ && serial_) {
            delete serial_;
        }
    }

    void print(const char* message) override {
        if (serial_) {
            serial_->printf("%s", message);
        }
    }

    void println(const char* message) override {
        if (serial_) {
            serial_->printf("%s\n", message);
        }
    }

    void printf(const char* format, ...) override {
        if (serial_) {
            va_list args;
            va_start(args, format);
            serial_->vprintf(format, args);
            va_end(args);
        }
    }
};

} // namespace mbed
} // namespace SF04HAL

#endif // SF04_HAL_MBED_H
