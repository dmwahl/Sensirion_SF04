/**
 * @file sf04_hal_particle.h
 * @brief Particle.io implementation of SF04 HAL
 *
 * Provides Particle-specific implementations of I2C and timing interfaces
 * Compatible with: Photon, Electron, Argon, Boron, Xenon, P2, Photon 2
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#ifndef SF04_HAL_PARTICLE_H
#define SF04_HAL_PARTICLE_H

#include "../sf04_hal.h"
#include "Particle.h"
#include <cstdarg>

namespace SF04HAL {
namespace particle {

/**
 * @brief Particle I2C implementation
 *
 * Uses Particle Wire library for I2C communication
 */
class ParticleI2C : public II2C {
private:
    TwoWire& wire_;
    uint32_t frequency_;
    bool begun_;

public:
    /**
     * @brief Constructor
     * @param wire Reference to Particle Wire object (Wire, Wire1, Wire3)
     * @param frequency I2C clock frequency in Hz (default: 100kHz)
     */
    ParticleI2C(TwoWire& wire = Wire, uint32_t frequency = 100000)
        : wire_(wire), frequency_(frequency), begun_(false) {
    }

    bool begin() override {
        if (!begun_) {
            wire_.begin();
            wire_.setSpeed(frequency_);
            begun_ = true;
        }
        return true;
    }

    void end() override {
        if (begun_) {
            wire_.end();
            begun_ = false;
        }
    }

    void setFrequency(uint32_t frequency) override {
        frequency_ = frequency;
        if (begun_) {
            wire_.setSpeed(frequency_);
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

        wire_.beginTransmission(address);

        for (size_t i = 0; i < length; i++) {
            wire_.write(data[i]);
        }

        uint8_t result = wire_.endTransmission(sendStop);

        // Particle Wire error codes:
        // 0: success
        // 1: data too long
        // 2: NACK on address
        // 3: NACK on data
        // 4: other error
        switch (result) {
            case 0:
                return I2CResult::SUCCESS;
            case 2:
            case 3:
                return I2CResult::NACK;
            case 4:
                return I2CResult::BUS_ERROR;
            default:
                return I2CResult::INVALID_PARAM;
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

        uint8_t received = wire_.requestFrom(address, (uint8_t)length, sendStop);

        if (received != length) {
            return I2CResult::NACK;
        }

        for (size_t i = 0; i < length; i++) {
            if (wire_.available()) {
                data[i] = wire_.read();
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
 * @brief Particle timing implementation
 */
class ParticleTiming : public ITiming {
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
 * @brief Particle debug output implementation
 *
 * Uses Particle Serial object (Serial, Serial1, etc.)
 * or can use Log.info() for cloud logging
 */
class ParticleDebug : public IDebug {
private:
    USARTSerial* serial_;
    bool useCloudLog_;

public:
    /**
     * @brief Constructor with Serial object
     * @param serial Reference to Particle Serial object
     */
    ParticleDebug(USARTSerial& serial)
        : serial_(&serial), useCloudLog_(false) {
        serial_->begin(115200);
    }

    /**
     * @brief Constructor for cloud logging
     * @param useCloudLog If true, uses Log.info() instead of Serial
     */
    ParticleDebug(bool useCloudLog = false)
        : serial_(nullptr), useCloudLog_(useCloudLog) {
        if (!useCloudLog_) {
            // Default to Serial
            serial_ = &Serial;
            serial_->begin(115200);
        }
    }

    void print(const char* message) override {
        if (useCloudLog_) {
            Log.print(message);
        } else if (serial_) {
            serial_->print(message);
        }
    }

    void println(const char* message) override {
        if (useCloudLog_) {
            Log.info(message);
        } else if (serial_) {
            serial_->println(message);
        }
    }

    void printf(const char* format, ...) override {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        if (useCloudLog_) {
            Log.info(buffer);
        } else if (serial_) {
            serial_->print(buffer);
        }
    }
};

} // namespace particle
} // namespace SF04HAL

#endif // SF04_HAL_PARTICLE_H
