# Sensirion SF04 Flow Sensor Driver

A platform-independent driver library for the **Sensirion SF04/SFM7033** digital flow sensor with Hardware Abstraction Layer (HAL) support.

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/yourusername/sensirion_sf04)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

## 📋 Table of Contents

- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Supported Platforms](#supported-platforms)
- [Architecture](#architecture)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
- [Examples](#examples)
- [Improvements in v2.0](#improvements-in-v20)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## ✨ Features

- **Platform Independent**: Works on mbed, Particle.io, and easily portable to other platforms
- **Hardware Abstraction Layer (HAL)**: Clean separation between sensor logic and platform-specific code
- **Multiple Measurements**: Flow rate, temperature, and supply voltage monitoring
- **Multi-Gas Calibrations**: Supports Air, O2, N2O, and concentration measurements
- **Configurable Resolution**: 9-16 bit measurement resolution
- **Data Integrity**: CRC-8 checksum verification on all communications
- **Debug Support**: Optional debug output interface
- **Well Documented**: Comprehensive API documentation and examples

## 🔧 Hardware Overview

### Sensirion SF04/SFM7033 Specifications

| Parameter | Value |
|-----------|-------|
| **Communication** | I2C (100 kHz typical) |
| **I2C Address** | 0x40 (7-bit) |
| **Supply Voltage** | 3.0 - 5.0V |
| **Measurement Type** | Thermal mass flow sensor |
| **Resolution** | 9-16 bits (configurable) |
| **Measurements** | Flow, Temperature, Supply Voltage |

### Pin Connections

| SF04 Pin | Function | Connect To |
|----------|----------|------------|
| VDD | Power Supply | 3.3V or 5V |
| GND | Ground | GND |
| SDA | I2C Data | I2C SDA pin |
| SCL | I2C Clock | I2C SCL pin |

## 🖥️ Supported Platforms

### Currently Supported

- ✅ **mbed OS** - ARM Cortex microcontrollers
- ✅ **Particle.io** - Photon, Electron, Argon, Boron, Xenon, P2, Photon 2

### Easily Portable To

- 🔜 Arduino (AVR, ARM, ESP32)
- 🔜 ESP-IDF (ESP32/ESP8266)
- 🔜 STM32 HAL
- 🔜 Zephyr RTOS

## 🏗️ Architecture

The library uses a Hardware Abstraction Layer (HAL) to separate platform-specific code from sensor logic:

```
┌─────────────────────────────────────┐
│    Application Code                 │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│    SF04 Sensor Driver (v2)          │
│    - Flow measurements              │
│    - Temperature monitoring         │
│    - Configuration management       │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│    HAL Interface (Abstract)         │
│    - II2C                           │
│    - ITiming                        │
│    - IDebug (optional)              │
└──────────────┬──────────────────────┘
               │
       ┌───────┴────────┐
       │                │
┌──────▼─────┐   ┌──────▼──────┐
│ mbed HAL   │   │ Particle HAL│
└────────────┘   └─────────────┘
```

### File Structure

```
Sensirion_SF04/
├── hal/
│   ├── sf04_hal.h                    # HAL interface definitions
│   ├── mbed/
│   │   └── sf04_hal_mbed.h          # mbed implementation
│   └── particle/
│       └── sf04_hal_particle.h      # Particle implementation
├── sensirion_sf04_typedefs.h        # Portable type definitions
├── sensirion_sf04_v2.h               # Main driver header (v2)
├── sensirion_sf04_v2.cpp             # Main driver implementation (v2)
├── sensirion_sf04.h                  # Legacy driver (v1 - mbed only)
├── sensirion_sf04.cpp                # Legacy implementation (v1)
├── examples/
│   ├── mbed/
│   │   └── main.cpp                 # mbed example
│   └── particle/
│       ├── sf04_example.ino         # Particle example
│       └── project.properties       # Particle project file
└── README.md                         # This file
```

## 📦 Installation

### mbed OS

1. Clone or download this repository into your mbed project
2. Include the necessary files in your project
3. Add `#include "sensirion_sf04_v2.h"` and `#include "hal/mbed/sf04_hal_mbed.h"`

### Particle.io

**Method 1: Particle Web IDE**
1. Create a new project
2. Copy the library files to your project
3. Copy the example code from `examples/particle/sf04_example.ino`

**Method 2: Particle Workbench (VS Code)**
1. Clone this repository
2. Open the `examples/particle` folder in VS Code
3. Build and flash to your device

**Method 3: Particle CLI**
```bash
particle project create my_sf04_project
cd my_sf04_project
# Copy library files here
particle compile <platform>
particle flash <device> <binary>
```

## 🚀 Quick Start

### mbed Example

```cpp
#include "mbed.h"
#include "sensirion_sf04_v2.h"
#include "hal/mbed/sf04_hal_mbed.h"

#define SF04_ADDR 0x40

// Create I2C and HAL instances
I2C i2c(I2C_SDA, I2C_SCL);
SF04HAL::mbed::MbedI2C halI2C(i2c, 100000);
SF04HAL::mbed::MbedTiming halTiming;

// Create sensor instance
SF04 sensor(halI2C, halTiming, SF04_ADDR, 0, 16);

int main() {
    while(1) {
        sensor.Measure(FLOW);
        sensor.Measure(TEMP);

        printf("Flow: %.2f %s, Temp: %.1f°C\n",
               sensor.getFlow(),
               sensor.flowUnitStr,
               sensor.getTemperature());

        wait_us(1000000);  // 1 second
    }
}
```

### Particle Example

```cpp
#include "Particle.h"
#include "sensirion_sf04_v2.h"
#include "hal/particle/sf04_hal_particle.h"

#define SF04_ADDR 0x40

// Create HAL instances
SF04HAL::particle::ParticleI2C halI2C(Wire, 100000);
SF04HAL::particle::ParticleTiming halTiming;

// Create sensor instance
SF04 sensor(halI2C, halTiming, SF04_ADDR, 0, 16);

void setup() {
    Serial.begin(115200);
}

void loop() {
    sensor.Measure(FLOW);
    sensor.Measure(TEMP);

    Serial.printf("Flow: %.2f %s, Temp: %.1f°C\n",
                  sensor.getFlow(),
                  sensor.flowUnitStr,
                  sensor.getTemperature());

    delay(1000);  // 1 second
}
```

## 📚 API Reference

### Constructor

```cpp
SF04(SF04HAL::II2C &i2c,
     SF04HAL::ITiming &timing,
     uint8_t address = SF04_DEFAULT_ADDRESS,
     int calField = 0,
     int resolution = 16,
     SF04HAL::IDebug *debug = nullptr);
```

**Parameters:**
- `i2c` - Reference to HAL I2C interface
- `timing` - Reference to HAL timing interface
- `address` - 7-bit I2C address (default: 0x40)
- `calField` - Calibration field 0-4 (0=Air, 1=O2, 2=N2O, 3=O2 concentration, 4=N2O concentration)
- `resolution` - Measurement resolution 9-16 bits
- `debug` - Optional debug output interface

### Public Methods

#### Measurements

```cpp
// Perform measurement (FLOW, TEMP, or VDD)
u8t Measure(etSF04MeasureType measureType);

// Get flow in physical units
float getFlow() const;

// Get temperature in degrees Celsius
float getTemperature() const;

// Get supply voltage in volts
float getVdd() const;
```

#### Configuration

```cpp
// Reset sensor
bool softReset();

// Set measurement resolution (9-16 bits)
u8t setMeasResolution(u16t resolution);

// Set active calibration field (0-4)
u8t setCalibrationField(u8t calfield);
```

### Public Data Members

```cpp
nt32 serialNumber;          // Sensor serial number
nt16 scaleFactor;           // Scale factor for flow conversion
char flowUnitStr[15];       // Flow unit string (e.g., "mln/min")
nt16 flow;                  // Raw flow measurement
nt16 temperature;           // Raw temperature (0.1°C units)
nt16 vdd;                   // Raw supply voltage (1mV units)
bool ready;                 // Initialization status
```

### Error Codes

```cpp
SF04_SUCCESS           = 0x00  // Operation successful
SF04_ACK_ERROR         = 0x01  // I2C NACK
SF04_TIME_OUT_ERROR    = 0x02  // Communication timeout
SF04_CHECKSUM_ERROR    = 0x04  // CRC verification failed
SF04_UNIT_ERROR        = 0x08  // Invalid flow unit
SF04_I2C_ERROR         = 0x10  // I2C communication error
```

## 📖 Examples

Complete examples are provided in the `examples/` directory:

- **mbed**: `examples/mbed/main.cpp`
- **Particle**: `examples/particle/sf04_example.ino`

Each example demonstrates:
- Sensor initialization
- Performing measurements
- Reading flow, temperature, and voltage
- Error handling
- Debug output

## 🆕 Improvements in v2.0

The v2.0 driver includes significant improvements over the original version:

### Architecture Improvements

| Feature | v1.0 (Original) | v2.0 (Improved) |
|---------|-----------------|-----------------|
| **Platform Support** | mbed only | mbed, Particle.io, easily portable |
| **Hardware Abstraction** | ❌ Tightly coupled to mbed | ✅ Clean HAL interface |
| **Code Organization** | Sample code in header | ✅ Separate examples directory |
| **Debug Support** | ❌ No debug interface | ✅ Optional debug HAL |
| **Error Handling** | Basic error codes | ✅ Enhanced I2C error handling |
| **Convenience Methods** | ❌ Manual conversion required | ✅ `getFlow()`, `getTemperature()`, `getVdd()` |
| **Documentation** | Minimal README | ✅ Comprehensive documentation |

### Code Quality Improvements

1. **Magic Numbers Eliminated**
   - `200` → `SF04_DELAY_BETWEEN_COMMANDS_US`
   - `0x40` → `SF04_DEFAULT_ADDRESS`
   - `0x131` → `SF04_POLYNOMIAL`

2. **Improved I2C Operations**
   - Uses HAL I2C with proper error checking
   - Better support for repeated start transactions
   - Cleaner EEPROM read implementation

3. **Enhanced Error Handling**
   - New `SF04_I2C_ERROR` code
   - Better error propagation
   - Debug output for troubleshooting

4. **Memory Safety**
   - Dynamic buffer allocation for EEPROM reads
   - Proper buffer bounds checking
   - No buffer overflows

### New Features

1. **Convenience Methods**
   ```cpp
   float flow = sensor.getFlow();           // Auto-scaled
   float temp = sensor.getTemperature();    // In °C
   float voltage = sensor.getVdd();         // In V
   ```

2. **Debug Interface**
   ```cpp
   SF04HAL::particle::ParticleDebug debug(Serial);
   SF04 sensor(halI2C, halTiming, 0x40, 0, 16, &debug);
   // Automatic debug output during initialization and errors
   ```

3. **Platform Flexibility**
   - Easy to add new platforms by implementing HAL interfaces
   - No platform-specific code in main driver
   - Clean abstraction boundaries

## 🔍 Troubleshooting

### Sensor Not Responding

**Symptoms:**
- `ready` flag is false after initialization
- I2C errors

**Solutions:**
1. Check wiring connections (SDA, SCL, VDD, GND)
2. Verify I2C address (use I2C scanner)
3. Check pull-up resistors on I2C lines (typically 4.7kΩ)
4. Verify power supply is 3.0-5.0V

### CRC Errors

**Symptoms:**
- `SF04_CHECKSUM_ERROR` returned from `Measure()`

**Solutions:**
1. Check I2C signal integrity with oscilloscope
2. Reduce I2C clock speed (try 50 kHz instead of 100 kHz)
3. Shorten I2C wires (keep < 30cm)
4. Add I2C pull-up resistors if missing

### Incorrect Measurements

**Symptoms:**
- Flow readings don't make sense
- Values are scaled incorrectly

**Solutions:**
1. Verify correct calibration field for your gas type
2. Check scale factor is read correctly
3. Ensure proper flow unit string
4. Try different resolution settings

### Particle.io Cloud Issues

**Symptoms:**
- Can't publish data to Particle cloud
- Cloud variables not updating

**Solutions:**
1. Check `SYSTEM_MODE` setting
2. Verify Particle.connect() is called
3. Check rate limits (max 1 publish/second, 4/second burst)
4. Ensure device is online: `Particle.connected()`

## 🔌 Adding Support for New Platforms

To add support for a new platform:

1. **Create HAL implementation files**
   ```
   hal/your_platform/sf04_hal_your_platform.h
   ```

2. **Implement HAL interfaces**
   ```cpp
   namespace SF04HAL {
   namespace your_platform {
       class YourI2C : public II2C { /* ... */ };
       class YourTiming : public ITiming { /* ... */ };
       class YourDebug : public IDebug { /* ... */ };
   }
   }
   ```

3. **Create example**
   ```
   examples/your_platform/main.cpp
   ```

4. **Test thoroughly**
   - Verify I2C communication
   - Check timing accuracy
   - Validate measurements

5. **Submit pull request** (optional)

## 📄 License

This library is released under the MIT License. See LICENSE file for details.

Original driver by Sensirion AG.
HAL refactoring and improvements by [Your Name/Organization].

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/yourusername/sensirion_sf04/issues)
- **Datasheet**: [SF04 Datasheet](https://www.sensirion.com/sf04)
- **Sensirion Support**: https://www.sensirion.com/support

## 🙏 Acknowledgments

- Sensirion AG for the original mbed driver
- mbed community
- Particle.io community

---

**Version:** 2.0.0
**Last Updated:** 2025-11-04
**Maintainer:** [Your Name/Email]