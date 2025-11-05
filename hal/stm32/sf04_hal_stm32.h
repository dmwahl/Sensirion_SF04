/**
 * @file sf04_hal_stm32.h
 * @brief STM32 HAL implementation of SF04 HAL
 *
 * Provides STM32-specific implementations of I2C and timing interfaces
 * Compatible with STM32 HAL library (all STM32 families)
 *
 * Requirements:
 * - STM32 HAL library
 * - I2C peripheral initialized with STM32CubeMX or manually
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#ifndef SF04_HAL_STM32_H
#define SF04_HAL_STM32_H

#include "../sf04_hal.h"
#include "stm32_hal.h"  // Generic header - include your specific STM32 HAL
                         // e.g., stm32f4xx_hal.h, stm32l4xx_hal.h, etc.
#include <cstdarg>
#include <cstdio>

namespace SF04HAL {
namespace stm32 {

/**
 * @brief STM32 I2C implementation using HAL library
 *
 * Uses STM32 HAL_I2C functions for I2C communication
 */
class STM32I2C : public II2C {
private:
    I2C_HandleTypeDef* hi2c_;
    uint32_t timeout_;  // I2C timeout in milliseconds

public:
    /**
     * @brief Constructor
     * @param hi2c Pointer to STM32 HAL I2C handle
     * @param timeout I2C timeout in milliseconds (default: 100ms)
     */
    STM32I2C(I2C_HandleTypeDef* hi2c, uint32_t timeout = 100)
        : hi2c_(hi2c), timeout_(timeout) {
    }

    bool begin() override {
        // I2C should already be initialized via STM32CubeMX or HAL_I2C_Init()
        // Just verify the handle is valid
        if (hi2c_ == nullptr) {
            return false;
        }
        return (hi2c_->State == HAL_I2C_STATE_READY);
    }

    void end() override {
        // Optionally deinitialize I2C
        // HAL_I2C_DeInit(hi2c_);
    }

    void setFrequency(uint32_t frequency) override {
        // Frequency is set during initialization via STM32CubeMX
        // Changing it at runtime requires reconfiguring the I2C peripheral
        // For now, this is a no-op. Users should configure I2C before creating the HAL
    }

    I2CResult write(uint8_t address, const uint8_t* data,
                   size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        // STM32 HAL uses 8-bit address (7-bit << 1)
        uint16_t devAddr = address << 1;

        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
            hi2c_,
            devAddr,
            (uint8_t*)data,
            length,
            timeout_
        );

        return convertHALStatus(status);
    }

    I2CResult read(uint8_t address, uint8_t* data,
                  size_t length, bool sendStop = true) override {
        if (data == nullptr || length == 0) {
            return I2CResult::INVALID_PARAM;
        }

        // STM32 HAL uses 8-bit address (7-bit << 1)
        uint16_t devAddr = address << 1;

        HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
            hi2c_,
            devAddr,
            data,
            length,
            timeout_
        );

        return convertHALStatus(status);
    }

    I2CResult writeRead(uint8_t address,
                       const uint8_t* writeData, size_t writeLength,
                       uint8_t* readData, size_t readLength) override {
        if (writeData == nullptr || writeLength == 0 ||
            readData == nullptr || readLength == 0) {
            return I2CResult::INVALID_PARAM;
        }

        // STM32 HAL uses 8-bit address (7-bit << 1)
        uint16_t devAddr = address << 1;

        // Write with no stop (I2C_FIRST_FRAME)
        HAL_StatusTypeDef status = HAL_I2C_Master_Seq_Transmit_IT(
            hi2c_,
            devAddr,
            (uint8_t*)writeData,
            writeLength,
            I2C_FIRST_FRAME
        );

        if (status != HAL_OK) {
            return convertHALStatus(status);
        }

        // Wait for write to complete
        uint32_t startTime = HAL_GetTick();
        while (HAL_I2C_GetState(hi2c_) != HAL_I2C_STATE_READY) {
            if ((HAL_GetTick() - startTime) > timeout_) {
                return I2CResult::TIMEOUT;
            }
        }

        // Read with stop (I2C_LAST_FRAME)
        status = HAL_I2C_Master_Seq_Receive_IT(
            hi2c_,
            devAddr,
            readData,
            readLength,
            I2C_LAST_FRAME
        );

        if (status != HAL_OK) {
            return convertHALStatus(status);
        }

        // Wait for read to complete
        startTime = HAL_GetTick();
        while (HAL_I2C_GetState(hi2c_) != HAL_I2C_STATE_READY) {
            if ((HAL_GetTick() - startTime) > timeout_) {
                return I2CResult::TIMEOUT;
            }
        }

        return I2CResult::SUCCESS;
    }

private:
    /**
     * @brief Convert STM32 HAL status to I2C result
     */
    I2CResult convertHALStatus(HAL_StatusTypeDef status) {
        switch (status) {
            case HAL_OK:
                return I2CResult::SUCCESS;
            case HAL_TIMEOUT:
                return I2CResult::TIMEOUT;
            case HAL_ERROR:
                // Check specific I2C error
                if (hi2c_->ErrorCode & HAL_I2C_ERROR_AF) {
                    return I2CResult::NACK;
                }
                return I2CResult::BUS_ERROR;
            default:
                return I2CResult::BUS_ERROR;
        }
    }
};

/**
 * @brief STM32 timing implementation using HAL library
 */
class STM32Timing : public ITiming {
private:
    uint32_t startTick_;

public:
    STM32Timing() : startTick_(HAL_GetTick()) {
    }

    void delayMicroseconds(uint32_t us) override {
        // STM32 HAL doesn't provide microsecond delay by default
        // Use DWT (Data Watchpoint and Trace) cycle counter for accurate delays
        #ifdef DWT
            uint32_t startCycle = DWT->CYCCNT;
            uint32_t cyclesToWait = (SystemCoreClock / 1000000) * us;
            while ((DWT->CYCCNT - startCycle) < cyclesToWait);
        #else
            // Fallback: rough approximation using HAL_Delay
            // Not accurate for small delays
            if (us >= 1000) {
                HAL_Delay(us / 1000);
            } else {
                // Busy wait - very approximate
                volatile uint32_t count = us * (SystemCoreClock / 1000000 / 4);
                while (count--);
            }
        #endif
    }

    void delayMilliseconds(uint32_t ms) override {
        HAL_Delay(ms);
    }

    uint32_t millis() override {
        return HAL_GetTick() - startTick_;
    }

    uint32_t micros() override {
        // Approximate microseconds using HAL_GetTick()
        // For accurate timing, use DWT or a hardware timer
        return (HAL_GetTick() - startTick_) * 1000;
    }
};

/**
 * @brief STM32 debug output implementation
 *
 * Supports multiple output methods:
 * - UART (using HAL_UART_Transmit)
 * - ITM (Instrumentation Trace Macrocell) for SWO debugging
 * - Semihosting (for debugging with IDE)
 */
class STM32Debug : public IDebug {
private:
    UART_HandleTypeDef* huart_;
    bool useITM_;
    char buffer_[256];

public:
    /**
     * @brief Constructor with UART
     * @param huart Pointer to STM32 HAL UART handle
     */
    STM32Debug(UART_HandleTypeDef* huart)
        : huart_(huart), useITM_(false) {
    }

    /**
     * @brief Constructor for ITM (SWO) output
     * @param useITM Set to true to use ITM instead of UART
     */
    STM32Debug(bool useITM = false)
        : huart_(nullptr), useITM_(useITM) {
    }

    void print(const char* message) override {
        if (useITM_) {
            #ifdef ITM
                // Send to ITM port 0
                while (*message) {
                    ITM_SendChar(*message++);
                }
            #endif
        } else if (huart_) {
            HAL_UART_Transmit(huart_, (uint8_t*)message,
                            strlen(message), HAL_MAX_DELAY);
        }
    }

    void println(const char* message) override {
        print(message);
        print("\r\n");
    }

    void printf(const char* format, ...) override {
        va_list args;
        va_start(args, format);
        vsnprintf(buffer_, sizeof(buffer_), format, args);
        va_end(args);
        print(buffer_);
    }
};

/**
 * @brief Helper function to enable DWT for microsecond delays
 *
 * Call this once in your initialization code to enable accurate
 * microsecond timing using the DWT cycle counter.
 */
inline void enableDWT() {
    #ifdef DWT
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    #endif
}

} // namespace stm32
} // namespace SF04HAL

#endif // SF04_HAL_STM32_H
