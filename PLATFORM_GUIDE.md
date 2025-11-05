# Platform-Specific Guide for SF04 Driver

This guide provides detailed information for using the SF04 driver on each supported platform.

## Table of Contents

- [STM32 HAL](#stm32-hal)
- [ESP32 Arduino](#esp32-arduino)
- [ESP8266 Arduino](#esp8266-arduino)
- [ESP32 ESP-IDF](#esp32-esp-idf)
- [Platform Comparison](#platform-comparison)

---

## STM32 HAL

### Supported STM32 Families

- ✅ STM32F0, STM32F1, STM32F2, STM32F3, STM32F4, STM32F7
- ✅ STM32L0, STM32L1, STM32L4, STM32L5
- ✅ STM32H7, STM32G0, STM32G4
- ✅ STM32WB, STM32WL
- ✅ All other STM32 families with HAL library support

### Prerequisites

1. **STM32CubeMX** or **STM32CubeIDE** for project generation
2. **STM32 HAL library** (included with STM32Cube)
3. Configured I2C peripheral
4. (Optional) UART for debug output

### Hardware Setup

**Typical STM32F4 Nucleo Board:**
```
SF04        STM32
----        -----
VDD   ---   3.3V
GND   ---   GND
SDA   ---   PB7  (I2C1_SDA)
SCL   ---   PB6  (I2C1_SCL)
```

**Notes:**
- Most STM32 boards have internal pull-ups that can be enabled
- External 4.7kΩ pull-ups recommended for reliable operation
- I2C pins vary by STM32 series - check datasheet

### STM32CubeMX Configuration

1. **Configure I2C Peripheral:**
   - Enable I2C1 (or I2C2, I2C3)
   - Mode: I2C Master
   - I2C Speed: Standard Mode (100 kHz)
   - Enable GPIO pull-ups on SDA/SCL

2. **Configure UART (for debug):**
   - Enable USART2 (or other)
   - Baud Rate: 115200
   - Word Length: 8 bits
   - Parity: None

3. **Enable SYS:**
   - Debug: Serial Wire

4. **Generate Code**

### Software Integration

```cpp
#include "main.h"
#include "sensirion_sf04_v2.h"
#include "hal/stm32/sf04_hal_stm32.h"

// Handles initialized by STM32CubeMX
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

// Create HAL instances
SF04HAL::stm32::STM32I2C halI2C(&hi2c1);
SF04HAL::stm32::STM32Timing halTiming;
SF04HAL::stm32::STM32Debug halDebug(&huart2);

// Create sensor
SF04 sensor(halI2C, halTiming, 0x40, 0, 16, &halDebug);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    // Enable DWT for accurate microsecond delays
    SF04HAL::stm32::enableDWT();

    while(1) {
        sensor.Measure(FLOW);
        halDebug.printf("Flow: %.2f %s\n",
                       sensor.getFlow(),
                       sensor.flowUnitStr);
        HAL_Delay(1000);
    }
}
```

### Debugging Options

**Option 1: UART Debug**
```cpp
SF04HAL::stm32::STM32Debug halDebug(&huart2);
```

**Option 2: ITM/SWO Debug**
```cpp
SF04HAL::stm32::STM32Debug halDebug(true);  // Use ITM
```

**Option 3: Semihosting**
- Configure in project settings
- Use `printf()` directly

### Common Issues

**Issue: Accurate microsecond delays not working**
- Solution: Call `SF04HAL::stm32::enableDWT()` after `HAL_Init()`

**Issue: I2C timeout errors**
- Check pull-up resistors (4.7kΩ recommended)
- Verify I2C clock speed (100 kHz)
- Check SDA/SCL pin configuration

**Issue: Debug output not working**
- Verify UART initialization
- Check baud rate (115200)
- Ensure TX pin is connected

---

## ESP32 Arduino

### Supported ESP32 Variants

- ✅ ESP32 (original)
- ✅ ESP32-S2
- ✅ ESP32-S3
- ✅ ESP32-C3
- ✅ ESP32-C6

### Prerequisites

1. **Arduino IDE** with ESP32 board support
   - Install via Board Manager: `https://dl.espressif.com/dl/package_esp32_index.json`
2. **ESP32 Arduino Core** v2.0.0 or later
3. **USB cable** for programming

### Hardware Setup

**Standard ESP32 DevKit:**
```
SF04        ESP32
----        -----
VDD   ---   3.3V
GND   ---   GND
SDA   ---   GPIO21 (default)
SCL   ---   GPIO22 (default)
```

**Custom Pin Configuration:**
```cpp
// Any GPIO pin can be used for I2C on ESP32
#define SDA_PIN 21
#define SCL_PIN 22
SF04HAL::esp32::ESP32I2C halI2C(Wire, SDA_PIN, SCL_PIN, 100000);
```

### Software Integration

```cpp
#include <Arduino.h>
#include "sensirion_sf04_v2.h"
#include "hal/esp32/sf04_hal_esp32_arduino.h"

// Default pins (21=SDA, 22=SCL)
SF04HAL::esp32::ESP32I2C halI2C(Wire, 100000);
SF04HAL::esp32::ESP32Timing halTiming;
SF04HAL::esp32::ESP32Debug halDebug(Serial);

SF04 sensor(halI2C, halTiming, 0x40, 0, 16, &halDebug);

void setup() {
    Serial.begin(115200);
}

void loop() {
    sensor.Measure(FLOW);
    Serial.printf("Flow: %.2f %s\n",
                  sensor.getFlow(),
                  sensor.flowUnitStr);
    delay(1000);
}
```

### WiFi Integration

```cpp
#include <WiFi.h>

const char* ssid = "your_ssid";
const char* password = "your_password";

void setup() {
    Serial.begin(115200);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
}

void loop() {
    sensor.Measure(FLOW);

    // Send data over WiFi
    if (WiFi.status() == WL_CONNECTED) {
        // HTTP POST, MQTT, etc.
    }

    delay(1000);
}
```

### Features

- ✅ Flexible GPIO pin assignment
- ✅ WiFi/Bluetooth support
- ✅ Web server capabilities
- ✅ OTA updates
- ✅ Deep sleep modes
- ✅ Multiple I2C buses (Wire, Wire1)

### Common Issues

**Issue: I2C communication fails**
- Check pin assignments (different boards have different defaults)
- Verify 3.3V power supply (ESP32 is NOT 5V tolerant)
- Add pull-up resistors (4.7kΩ)

**Issue: Watchdog timer resets**
- Add `delay(10)` in main loop
- Use `yield()` in blocking operations

**Issue: Brown-out detector triggered**
- Use adequate power supply (500mA minimum)
- Add decoupling capacitors (10µF + 100nF)

---

## ESP8266 Arduino

### Prerequisites

1. **Arduino IDE** with ESP8266 board support
   - Install via Board Manager: `http://arduino.esp8266.com/stable/package_esp8266com_index.json`
2. **ESP8266 Arduino Core** v3.0.0 or later

### Hardware Setup

**NodeMCU/Wemos D1 Mini:**
```
SF04        ESP8266
----        --------
VDD   ---   3.3V
GND   ---   GND
SDA   ---   GPIO4  (D2)
SCL   ---   GPIO5  (D1)
```

**Note:** ESP8266 I2C pins are fixed (GPIO4=SDA, GPIO5=SCL)

### Software Integration

```cpp
#include <Arduino.h>
#include "sensirion_sf04_v2.h"
#include "hal/esp32/sf04_hal_esp32_arduino.h"  // Same HAL as ESP32

// ESP8266 uses default pins only
SF04HAL::esp32::ESP32I2C halI2C(Wire, 100000);
SF04HAL::esp32::ESP32Timing halTiming;
SF04HAL::esp32::ESP32Debug halDebug(Serial);

SF04 sensor(halI2C, halTiming, 0x40, 0, 16, &halDebug);

void setup() {
    Serial.begin(115200);
}

void loop() {
    sensor.Measure(FLOW);
    Serial.printf("Flow: %.2f %s\n",
                  sensor.getFlow(),
                  sensor.flowUnitStr);
    delay(1000);
}
```

### Differences from ESP32

| Feature | ESP32 | ESP8266 |
|---------|-------|---------|
| Custom I2C pins | ✅ Yes | ❌ No (fixed GPIO4/5) |
| I2C buses | 2 | 1 |
| WiFi standard | 802.11 b/g/n | 802.11 b/g/n |
| Bluetooth | ✅ Yes | ❌ No |
| CPU speed | 240 MHz | 80/160 MHz |
| RAM | 520 KB | 80 KB |

### Common Issues

**Issue: Sketch too large**
- Use smaller partition scheme
- Remove unused libraries
- Optimize code size

**Issue: Memory issues**
- ESP8266 has limited RAM (80 KB)
- Minimize global variables
- Use `PROGMEM` for constants

---

## ESP32 ESP-IDF

### Prerequisites

1. **ESP-IDF v4.0** or later
2. **ESP-IDF tools** installed
3. **CMake** and build tools

### Hardware Setup

Same as ESP32 Arduino (flexible GPIO assignment)

### Project Structure

```
your_project/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   └── main.cpp
└── components/
    └── sf04/
        ├── sensirion_sf04_v2.h
        ├── sensirion_sf04_v2.cpp
        ├── sensirion_sf04_typedefs.h
        └── hal/
            └── esp32/
                └── sf04_hal_esp32_idf.h
```

### CMakeLists.txt Configuration

**Project CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sf04_example)
```

**main/CMakeLists.txt:**
```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver
)
```

### Software Integration

```cpp
#include "sensirion_sf04_v2.h"
#include "hal/esp32/sf04_hal_esp32_idf.h"

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

SF04HAL::esp32_idf::ESP32IDFI2C halI2C(
    I2C_NUM_0, I2C_SDA, I2C_SCL, 100000
);
SF04HAL::esp32_idf::ESP32IDFTiming halTiming;
SF04HAL::esp32_idf::ESP32IDFDebug halDebug("SF04");

SF04 sensor(halI2C, halTiming, 0x40, 0, 16, &halDebug);

extern "C" void app_main(void) {
    while(1) {
        sensor.Measure(FLOW);
        ESP_LOGI("MAIN", "Flow: %.2f %s",
                sensor.getFlow(),
                sensor.flowUnitStr);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### Build & Flash

```bash
# Configure
idf.py menuconfig

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py -p /dev/ttyUSB0 monitor
```

### FreeRTOS Integration

```cpp
void sensor_task(void *pvParameters) {
    while(1) {
        sensor.Measure(FLOW);
        ESP_LOGI("SENSOR", "Flow: %.2f", sensor.getFlow());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void) {
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
```

### Advantages over Arduino

- ✅ Lower-level control
- ✅ Better performance
- ✅ Professional development tools
- ✅ FreeRTOS integration
- ✅ Advanced power management
- ✅ Better documentation

---

## Platform Comparison

### Hardware Requirements

| Platform | Min RAM | Min Flash | Clock Speed | I2C Hardware |
|----------|---------|-----------|-------------|--------------|
| STM32 | 8 KB | 16 KB | 8 MHz+ | Hardware I2C |
| ESP32 | 520 KB | 4 MB | 240 MHz | Hardware I2C |
| ESP8266 | 80 KB | 4 MB | 80 MHz | Software I2C |
| mbed | 16 KB | 64 KB | 16 MHz+ | Hardware I2C |
| Particle | 128 KB | 1 MB | 120 MHz | Hardware I2C |

### Memory Usage (Approximate)

| Platform | Code Size | RAM Usage |
|----------|-----------|-----------|
| STM32 HAL | ~12 KB | ~2 KB |
| ESP32 Arduino | ~15 KB | ~3 KB |
| ESP8266 Arduino | ~14 KB | ~3 KB |
| ESP32 IDF | ~10 KB | ~2 KB |
| mbed | ~13 KB | ~2 KB |
| Particle | ~14 KB | ~2 KB |

### Feature Comparison

| Feature | STM32 | ESP32 | ESP8266 | mbed | Particle |
|---------|-------|-------|---------|------|----------|
| WiFi | ❌ | ✅ | ✅ | ❌ | ✅ |
| Bluetooth | ❌ | ✅ | ❌ | ❌ | ✅ |
| Custom I2C pins | ✅ | ✅ | ❌ | ✅ | ✅ |
| Multiple I2C buses | ✅ | ✅ | ❌ | ✅ | ✅ |
| Deep sleep | ✅ | ✅ | ✅ | ✅ | ✅ |
| OTA updates | ❌ | ✅ | ✅ | ❌ | ✅ |
| Cloud integration | ❌ | ✅ | ✅ | ❌ | ✅ |
| Professional IDE | ✅ | ✅ | ✅ | ✅ | ✅ |

### When to Use Each Platform

**STM32 HAL:**
- Industrial applications
- Battery-powered devices
- Cost-sensitive projects
- Real-time requirements
- Automotive/medical applications

**ESP32 Arduino:**
- IoT projects with WiFi
- Rapid prototyping
- Hobbyist projects
- Smart home devices
- Web-connected sensors

**ESP8266 Arduino:**
- Low-cost WiFi projects
- Simple IoT devices
- Limited I/O requirements
- Constrained budgets

**ESP32 ESP-IDF:**
- Professional IoT products
- Advanced power management
- Complex FreeRTOS applications
- Production deployments
- Custom hardware integration

**mbed:**
- ARM-based prototyping
- University projects
- Cross-platform development
- Quick proof-of-concepts

**Particle:**
- Cloud-first IoT
- Fleet management
- Over-the-air updates
- Rapid deployment
- Cellular connectivity

---

## Troubleshooting by Platform

### STM32

| Issue | Solution |
|-------|----------|
| I2C timeout | Check clock stretching, reduce speed |
| Microsecond delays inaccurate | Enable DWT counter |
| Debug output missing | Configure UART/ITM properly |
| Hard fault | Check stack size, enable FPU if needed |

### ESP32/ESP8266

| Issue | Solution |
|-------|----------|
| Watchdog reset | Add delays in loop, use yield() |
| Brown-out | Improve power supply, add capacitors |
| WiFi drops | Reduce TX power, check antenna |
| Memory issues | Optimize code, use static allocation |

### All Platforms

| Issue | Solution |
|-------|----------|
| CRC errors | Add pull-ups, shorten I2C wires |
| No sensor response | Check address (0x40), verify connections |
| Incorrect readings | Verify calibration field, scale factor |

---

**Document Version:** 1.0
**Last Updated:** 2025-11-05
**Platforms Covered:** STM32, ESP32, ESP8266
