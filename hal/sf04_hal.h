/**
 * @file sf04_hal.h
 * @brief Hardware Abstraction Layer for SF04 sensor driver
 *
 * This HAL provides platform-independent interfaces for:
 * - I2C communication
 * - Timing/delays
 * - Debug output (optional)
 *
 * Supported platforms:
 * - mbed OS
 * - Particle.io (Photon, Electron, Argon, Boron, Xenon, P2, etc.)
 * - Arduino (future)
 * - ESP32/ESP8266 (future)
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#ifndef SF04_HAL_H
#define SF04_HAL_H

#include <stdint.h>
#include <stdbool.h>

namespace SF04HAL {

/**
 * @brief I2C result codes
 */
enum class I2CResult {
    SUCCESS = 0,        ///< Operation successful
    NACK = 1,           ///< No acknowledge received
    TIMEOUT = 2,        ///< Operation timed out
    BUS_ERROR = 3,      ///< Bus error occurred
    INVALID_PARAM = 4   ///< Invalid parameter
};

/**
 * @brief Abstract I2C interface
 *
 * Platform implementations must provide concrete implementations
 * of this interface for their specific I2C hardware.
 */
class II2C {
public:
    virtual ~II2C() {}

    /**
     * @brief Initialize I2C peripheral
     * @return true if successful, false otherwise
     */
    virtual bool begin() = 0;

    /**
     * @brief Deinitialize I2C peripheral
     */
    virtual void end() = 0;

    /**
     * @brief Set I2C clock frequency
     * @param frequency Clock frequency in Hz (typically 100000 or 400000)
     */
    virtual void setFrequency(uint32_t frequency) = 0;

    /**
     * @brief Write data to I2C device
     * @param address 7-bit I2C address
     * @param data Pointer to data buffer
     * @param length Number of bytes to write
     * @param sendStop Whether to send STOP condition after write
     * @return I2C result code
     */
    virtual I2CResult write(uint8_t address, const uint8_t* data,
                           size_t length, bool sendStop = true) = 0;

    /**
     * @brief Read data from I2C device
     * @param address 7-bit I2C address
     * @param data Pointer to receive buffer
     * @param length Number of bytes to read
     * @param sendStop Whether to send STOP condition after read
     * @return I2C result code
     */
    virtual I2CResult read(uint8_t address, uint8_t* data,
                          size_t length, bool sendStop = true) = 0;

    /**
     * @brief Write then read from I2C device (repeated start)
     * @param address 7-bit I2C address
     * @param writeData Pointer to write buffer
     * @param writeLength Number of bytes to write
     * @param readData Pointer to read buffer
     * @param readLength Number of bytes to read
     * @return I2C result code
     */
    virtual I2CResult writeRead(uint8_t address,
                               const uint8_t* writeData, size_t writeLength,
                               uint8_t* readData, size_t readLength) = 0;
};

/**
 * @brief Abstract timing interface
 *
 * Provides platform-independent delay functions
 */
class ITiming {
public:
    virtual ~ITiming() {}

    /**
     * @brief Delay for specified microseconds
     * @param us Microseconds to delay
     */
    virtual void delayMicroseconds(uint32_t us) = 0;

    /**
     * @brief Delay for specified milliseconds
     * @param ms Milliseconds to delay
     */
    virtual void delayMilliseconds(uint32_t ms) = 0;

    /**
     * @brief Get current timestamp in milliseconds
     * @return Millisecond timestamp
     */
    virtual uint32_t millis() = 0;

    /**
     * @brief Get current timestamp in microseconds
     * @return Microsecond timestamp
     */
    virtual uint32_t micros() = 0;
};

/**
 * @brief Abstract debug output interface (optional)
 *
 * Provides platform-independent debug logging
 */
class IDebug {
public:
    virtual ~IDebug() {}

    /**
     * @brief Print debug message
     * @param message Message to print
     */
    virtual void print(const char* message) = 0;

    /**
     * @brief Print debug message with newline
     * @param message Message to print
     */
    virtual void println(const char* message) = 0;

    /**
     * @brief Printf-style formatted output
     * @param format Printf format string
     * @param ... Variable arguments
     */
    virtual void printf(const char* format, ...) = 0;
};

} // namespace SF04HAL

#endif // SF04_HAL_H
