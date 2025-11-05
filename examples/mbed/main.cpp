/**
 * @file main.cpp
 * @brief mbed example for Sensirion SF04 flow sensor
 *
 * This example demonstrates how to use the SF04 driver on mbed OS
 *
 * Hardware connections:
 * - PB_3  -> SF04 SDA
 * - PB_10 -> SF04 SCL
 * - 3.3V  -> SF04 VDD
 * - GND   -> SF04 GND
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#include "mbed.h"
#include "sensirion_sf04_v2.h"
#include "hal/mbed/sf04_hal_mbed.h"

#define SFM7033_ADDR 0x40

// Create I2C interface on PB_3 (SDA), PB_10 (SCL)
I2C i2c(PB_3, PB_10);

// Create HAL instances
SF04HAL::mbed::MbedI2C halI2C(i2c, 100000);  // 100kHz
SF04HAL::mbed::MbedTiming halTiming;
SF04HAL::mbed::MbedDebug halDebug(USBTX, USBRX, 115200);

// Create SF04 sensor instance
// Parameters: (i2c, timing, address, calibration field, resolution, debug)
SF04 sfm(halI2C, halTiming, SFM7033_ADDR, 0, 16, &halDebug);

DigitalOut myled(LED1);

int main()
{
    halDebug.println("\n=== Sensirion SF04 Flow Sensor Example ===");
    halDebug.println("Platform: mbed OS");
    halDebug.println("Starting up...\n");

    if (sfm.ready) {
        halDebug.printf("Sensor initialized successfully!\n");
        halDebug.printf("Serial Number: %lu\n", sfm.serialNumber.u32);
        halDebug.printf("Scale Factor: %u\n", sfm.scaleFactor.u16);
        halDebug.printf("Flow Unit: %s\n\n", sfm.flowUnitStr);
    } else {
        halDebug.println("ERROR: Sensor initialization failed!");
        halDebug.println("Check connections and I2C address");
        while(1) {
            myled = !myled;
            wait_us(100000);  // 100ms
        }
    }

    while (1) {
        myled = !myled;

        // Perform measurements
        u8t errorFlow = sfm.Measure(FLOW);
        u8t errorTemp = sfm.Measure(TEMP);
        u8t errorVdd = sfm.Measure(VDD);

        if (errorFlow == 0 && errorTemp == 0 && errorVdd == 0) {
            // Print raw values
            halDebug.printf("Raw - Flow: %d, Temp: %d, Vdd: %u\n",
                          sfm.flow.i16, sfm.temperature.i16, sfm.vdd.u16);

            // Print converted values
            halDebug.printf("Flow: %.2f %s | Temp: %.1f°C | Vdd: %.2fV\n\n",
                          sfm.getFlow(),
                          sfm.flowUnitStr,
                          sfm.getTemperature(),
                          sfm.getVdd());
        } else {
            halDebug.printf("ERROR: Measurement failed (Flow=0x%02X, Temp=0x%02X, Vdd=0x%02X)\n",
                          errorFlow, errorTemp, errorVdd);
        }

        wait_us(1000000);  // 1 second delay
    }
}
