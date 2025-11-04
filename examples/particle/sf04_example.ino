/**
 * @file sf04_example.ino
 * @brief Particle.io example for Sensirion SF04 flow sensor
 *
 * This example demonstrates how to use the SF04 driver on Particle devices
 * Compatible with: Photon, Electron, Argon, Boron, Xenon, P2, Photon 2
 *
 * Hardware connections:
 * - D0 (SDA) -> SF04 SDA
 * - D1 (SCL) -> SF04 SCL
 * - 3.3V     -> SF04 VDD
 * - GND      -> SF04 GND
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#include "Particle.h"
#include "sensirion_sf04_v2.h"
#include "hal/particle/sf04_hal_particle.h"

// System mode - uncomment the mode you want
// SYSTEM_MODE(AUTOMATIC);  // Connect to cloud automatically
SYSTEM_MODE(SEMI_AUTOMATIC); // Manual cloud connection
// SYSTEM_MODE(MANUAL);      // No cloud connection

#define SFM7033_ADDR 0x40

// Create HAL instances
SF04HAL::particle::ParticleI2C halI2C(Wire, 100000);  // Use Wire (D0/D1), 100kHz
SF04HAL::particle::ParticleTiming halTiming;
SF04HAL::particle::ParticleDebug halDebug(Serial);

// Create SF04 sensor instance
// Parameters: (i2c, timing, address, calibration field, resolution, debug)
SF04 sfm(halI2C, halTiming, SFM7033_ADDR, 0, 16, &halDebug);

// Cloud variables (optional - for Particle cloud monitoring)
double cloudFlow = 0.0;
double cloudTemp = 0.0;
double cloudVdd = 0.0;

void setup() {
    // Start serial
    Serial.begin(115200);
    delay(2000);  // Wait for serial to initialize

    halDebug.println("\n=== Sensirion SF04 Flow Sensor Example ===");
    halDebug.println("Platform: Particle.io");
    halDebug.println("Starting up...\n");

    // Set up LED
    pinMode(D7, OUTPUT);

    // Register cloud variables (optional)
    Particle.variable("flow", cloudFlow);
    Particle.variable("temperature", cloudTemp);
    Particle.variable("vdd", cloudVdd);

    // Connect to cloud (if in SEMI_AUTOMATIC mode)
    // Particle.connect();

    if (sfm.ready) {
        halDebug.printf("Sensor initialized successfully!\n");
        halDebug.printf("Serial Number: %lu\n", sfm.serialNumber.u32);
        halDebug.printf("Scale Factor: %u\n", sfm.scaleFactor.u16);
        halDebug.printf("Flow Unit: %s\n\n", sfm.flowUnitStr);

        // Publish initialization event to cloud (optional)
        // Particle.publish("sf04_status", "initialized", PRIVATE);
    } else {
        halDebug.println("ERROR: Sensor initialization failed!");
        halDebug.println("Check connections and I2C address");

        // Publish error to cloud (optional)
        // Particle.publish("sf04_status", "init_failed", PRIVATE);

        while(1) {
            digitalWrite(D7, !digitalRead(D7));
            delay(100);
        }
    }
}

void loop() {
    static unsigned long lastMeasurement = 0;
    static bool ledState = false;

    // Toggle LED
    ledState = !ledState;
    digitalWrite(D7, ledState);

    // Perform measurements
    u8t errorFlow = sfm.Measure(FLOW);
    u8t errorTemp = sfm.Measure(TEMP);
    u8t errorVdd = sfm.Measure(VDD);

    if (errorFlow == 0 && errorTemp == 0 && errorVdd == 0) {
        // Update cloud variables
        cloudFlow = sfm.getFlow();
        cloudTemp = sfm.getTemperature();
        cloudVdd = sfm.getVdd();

        // Print raw values
        halDebug.printf("Raw - Flow: %d, Temp: %d, Vdd: %u\n",
                      sfm.flow.i16, sfm.temperature.i16, sfm.vdd.u16);

        // Print converted values
        halDebug.printf("Flow: %.2f %s | Temp: %.1f°C | Vdd: %.2fV\n\n",
                      cloudFlow,
                      sfm.flowUnitStr,
                      cloudTemp,
                      cloudVdd);

        // Publish to cloud every 10 seconds (optional)
        if (millis() - lastMeasurement > 10000) {
            lastMeasurement = millis();

            // Create JSON data
            char jsonData[128];
            snprintf(jsonData, sizeof(jsonData),
                    "{\"flow\":%.2f,\"temp\":%.1f,\"vdd\":%.2f}",
                    cloudFlow, cloudTemp, cloudVdd);

            // Publish to cloud (uncomment if desired)
            // Particle.publish("sf04_data", jsonData, PRIVATE);
        }
    } else {
        halDebug.printf("ERROR: Measurement failed (Flow=0x%02X, Temp=0x%02X, Vdd=0x%02X)\n",
                      errorFlow, errorTemp, errorVdd);

        // Publish error to cloud (optional)
        // Particle.publish("sf04_status", "measurement_error", PRIVATE);
    }

    delay(1000);  // 1 second delay
}
