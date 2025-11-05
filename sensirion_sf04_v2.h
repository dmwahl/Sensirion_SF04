/**
 * @file sensirion_sf04_v2.h
 * @brief Platform-independent Sensirion SF04 flow sensor driver (v2.0)
 *
 * This is a refactored version of the SF04 driver that uses a Hardware
 * Abstraction Layer (HAL) for platform independence.
 *
 * Supported platforms:
 * - mbed OS
 * - Particle.io (Photon, Electron, Argon, Boron, Xenon, P2, etc.)
 * - Arduino (future)
 * - ESP32/ESP8266 (future)
 *
 * @version 2.0.0
 * @date 2025-11-04
 * @author Sensirion AG / Refactored for HAL compatibility
 *
 * Hardware: Sensirion SF04/SFM7033 Digital Flow Sensor
 * - I2C Address: 0x40 (7-bit)
 * - Protocol: I2C at 100kHz (typical)
 * - Features: Flow, Temperature, Vdd measurement with CRC verification
 */

#ifndef SENSIRION_SF04_V2_H
#define SENSIRION_SF04_V2_H

#include "sensirion_sf04_typedefs.h"
#include "hal/sf04_hal.h"

//---------- Defines -----------------------------------------------------------
// CRC
#define SF04_POLYNOMIAL 0x131 //P(x)=x^8+x^5+x^4+1 = 100110001

// SF04 I2C address (7-bit)
#define SF04_DEFAULT_ADDRESS 0x40

// SF04 eeprom map
#define SF04_EE_ADR_SN_CHIP 0x02E4
#define SF04_EE_ADR_SN_PRODUCT 0x02F8
#define SF04_EE_ADR_SCALE_FACTOR 0x02B6
#define SF04_EE_ADR_FLOW_UNIT 0x02B7

// Timing constants
#define SF04_DELAY_BETWEEN_COMMANDS_US 200  // microseconds

// sensor command
typedef enum {
    SF04_USER_REG_W = 0xE2, // command writing user register
    SF04_USER_REG_R = 0xE3, // command reading user register
    SF04_ADV_USER_REG_W = 0xE4, // command writing advanced user register
    SF04_ADV_USER_REG_R = 0xE5, // command reading advanced user register
    SF04_READ_ONLY_REG1_R = 0xE7, // command reading read-only register 1
    SF04_READ_ONLY_REG2_R = 0xE9, // command reading read-only register 2
    SF04_TRIGGER_FLOW_MEASUREMENT = 0xF1, // command trig. a flow measurement
    SF04_TRIGGER_TEMP_MEASUREMENT = 0xF3, // command trig. a temperature measurement
    SF04_TRIGGER_VDD_MEASUREMENT = 0xF5, // command trig. a supply voltage measurement
    SF04_EEPROM_W = 0xFA, // command writing eeprom
    SF04_EEPROM_R = 0xFA, // command reading eeprom
    SF04_SOFT_RESET = 0xFE // command soft reset
} etCommand;

// sensor register
typedef enum {
    SF04_USER_REG = SF04_USER_REG_R,
    SF04_ADV_USER_REG = SF04_ADV_USER_REG_R,
    SF04_READ_ONLY_REG1 = SF04_READ_ONLY_REG1_R,
    SF04_READ_ONLY_REG2 = SF04_READ_ONLY_REG2_R
} etSF04Register;

// measurement signal selection
typedef enum {
    FLOW = SF04_TRIGGER_FLOW_MEASUREMENT,
    TEMP = SF04_TRIGGER_TEMP_MEASUREMENT,
    VDD = SF04_TRIGGER_VDD_MEASUREMENT,
} etSF04MeasureType;

// This enum lists all available flow resolution (Advanced User Register [11:9])
typedef enum {
    eSF04_RES_9BIT = ( 0<<9 ),
    eSF04_RES_10BIT = ( 1<<9 ),
    eSF04_RES_11BIT = ( 2<<9 ),
    eSF04_RES_12BIT = ( 3<<9 ),
    eSF04_RES_13BIT = ( 4<<9 ),
    eSF04_RES_14BIT = ( 5<<9 ),
    eSF04_RES_15BIT = ( 6<<9 ),
    eSF04_RES_16BIT = ( 7<<9 ),
    eSF04_RES_MASK = ( 7<<9 ) // (0x0E00)
} etSF04Resolution;

// Error codes
typedef enum {
    SF04_SUCCESS = 0x00,
    SF04_ACK_ERROR = 0x01,
    SF04_TIME_OUT_ERROR = 0x02,
    SF04_CHECKSUM_ERROR = 0x04,
    SF04_UNIT_ERROR = 0x08,
    SF04_I2C_ERROR = 0x10
} etError;

// Calibration fields for different gases
typedef enum {
    eSF04_CF0_AIR = 0x0A00,
    eSF04_CF1_O2 = 0x0A10,
    eSF04_CF2_N2O = 0x0A20,
    eSF04_CF3_O2 = 0x0A30, // Only for concentration measurement
    eSF04_CF4_N2O = 0x0A40, // Only for concentration measurement
    eSF04_CFRAW = 0x0880, // Only for concentration measurement
    eSF04_CF_MASK = 0x70
} etSF04Calibration;

// Macro that returns number of elements in an array
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

/**
 * @brief SF04 sensor driver class (v2.0 with HAL)
 *
 * This class provides a platform-independent interface to the Sensirion SF04
 * flow sensor using a Hardware Abstraction Layer.
 */
class SF04
{
protected:
    SF04HAL::II2C &i2c_;
    SF04HAL::ITiming &timing_;
    SF04HAL::IDebug *debug_;  // Optional debug interface
    u8t i2cAddress_; // 7-bit I2C address

    u8t checkCRC(u8t data[], u8t numBytes, u8t checksum);

    u8t ReadRegister(etSF04Register eSF04Register, nt16 *pRegisterValue);
    u8t WriteRegister(etSF04Register eSF04Register, nt16 *pRegisterValue);
    u8t ReadEeprom(u16t eepromStartAdr, u16t size, nt16 eepromData[]);
    u8t ReadSerialNumber(nt32 *serialNumber);
    u8t ReadScaleFactor(nt16 *scaleFactor);
    u8t ReadFlowUnit(char *flowUnitStr);
    u8t setTempVddCorrectionBit(u8t value);

    void delayBetweenCommands();

public:
    /**
     * @brief Constructor
     * @param i2c Reference to HAL I2C interface
     * @param timing Reference to HAL timing interface
     * @param address 7-bit I2C address (default: 0x40)
     * @param calField Calibration field (0-4)
     * @param resolution Measurement resolution (9-16 bits)
     * @param debug Optional debug interface (default: nullptr)
     */
    SF04(SF04HAL::II2C &i2c,
         SF04HAL::ITiming &timing,
         uint8_t address = SF04_DEFAULT_ADDRESS,
         int calField = 0,
         int resolution = 16,
         SF04HAL::IDebug *debug = nullptr);

    /**
     * @brief Reset sensor
     * @return true if slave sends ACK, false if not
     */
    bool softReset();

    /**
     * @brief Set measurement resolution from 9-16 bits
     * @param resolution Resolution in bits (9-16)
     * @return Error code (0 = success)
     */
    u8t setMeasResolution(u16t resolution);

    /**
     * @brief Set active calibration field
     * @param calfield Calibration field (0-4)
     * @return Error code (0 = success)
     */
    u8t setCalibrationField(u8t calfield);

    /**
     * @brief Perform measurement
     * @param measureType Type of measurement (FLOW, TEMP, or VDD)
     * @return Error code (0 = success)
     */
    u8t Measure(etSF04MeasureType measureType);

    /**
     * @brief Get flow in physical units
     * @return Flow value in units specified by flowUnitStr
     */
    float getFlow() const;

    /**
     * @brief Get temperature in degrees Celsius
     * @return Temperature in °C
     */
    float getTemperature() const;

    /**
     * @brief Get supply voltage in volts
     * @return Supply voltage in V
     */
    float getVdd() const;

    // Public data members
    nt32 serialNumber;
    nt16 scaleFactor;
    char flowUnitStr[15]; //string for the flow unit

    nt16 flow, temperature, vdd, status, checksum; // raw readings

    bool ready; // signal whether or not an operation is in progress
};

#endif // SENSIRION_SF04_V2_H
