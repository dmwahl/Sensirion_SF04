/**
 * @file sf04_example.ino
 * @brief ESP32/ESP8266 Arduino example for Sensirion SF04 flow sensor
 *
 * This example demonstrates how to use the SF04 driver on ESP32/ESP8266
 * using the Arduino framework
 *
 * Hardware connections:
 * ESP32:
 * - GPIO21 (SDA) -> SF04 SDA
 * - GPIO22 (SCL) -> SF04 SCL
 * - 3.3V         -> SF04 VDD
 * - GND          -> SF04 GND
 *
 * ESP8266:
 * - GPIO4 (SDA)  -> SF04 SDA
 * - GPIO5 (SCL)  -> SF04 SCL
 * - 3.3V         -> SF04 VDD
 * - GND          -> SF04 GND
 *
 * Features:
 * - Flow, temperature, and voltage measurements
 * - Custom I2C pins (ESP32 only)
 * - WiFi connectivity (optional)
 * - Web server for remote monitoring (optional)
 *
 * @version 2.0.0
 * @date 2025-11-04
 */

#include <Arduino.h>
#include "sensirion_sf04_v2.h"
#include "hal/esp32/sf04_hal_esp32_arduino.h"

// Uncomment for WiFi support
// #include <WiFi.h>           // ESP32
// #include <ESP8266WiFi.h>    // ESP8266
// const char* ssid = "your_ssid";
// const char* password = "your_password";

#define SF04_ADDR 0x40

// Option 1: Use default I2C pins
SF04HAL::esp32::ESP32I2C halI2C(Wire, 100000);

// Option 2: Use custom pins (ESP32 only - uncomment to use)
// #define I2C_SDA 21
// #define I2C_SCL 22
// SF04HAL::esp32::ESP32I2C halI2C(Wire, I2C_SDA, I2C_SCL, 100000);

// Create HAL instances
SF04HAL::esp32::ESP32Timing halTiming;
SF04HAL::esp32::ESP32Debug halDebug(Serial);

// Create SF04 sensor instance
SF04 sensor(halI2C, halTiming, SF04_ADDR, 0, 16, &halDebug);

// Measurement interval
const unsigned long MEASUREMENT_INTERVAL = 1000;  // 1 second
unsigned long lastMeasurement = 0;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Sensirion SF04 Flow Sensor Example ===");

    #ifdef ESP32
        Serial.println("Platform: ESP32");
    #else
        Serial.println("Platform: ESP8266");
    #endif

    Serial.println("Starting up...\n");

    // Initialize LED
    #ifdef ESP32
        pinMode(LED_BUILTIN, OUTPUT);
    #else
        pinMode(LED_BUILTIN, OUTPUT);
    #endif

    // Optional: Connect to WiFi
    /*
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    */

    // Check sensor initialization
    if (sensor.ready) {
        Serial.println("Sensor initialized successfully!");
        Serial.printf("Serial Number: %lu\n", sensor.serialNumber.u32);
        Serial.printf("Scale Factor: %u\n", sensor.scaleFactor.u16);
        Serial.printf("Flow Unit: %s\n\n", sensor.flowUnitStr);
    } else {
        Serial.println("ERROR: Sensor initialization failed!");
        Serial.println("Check connections and I2C address");
        while(1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(100);
        }
    }

    Serial.println("Starting measurements...\n");
}

void loop() {
    unsigned long currentTime = millis();

    // Perform measurements at regular intervals
    if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
        lastMeasurement = currentTime;

        // Toggle LED to show activity
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // Perform measurements
        u8t errorFlow = sensor.Measure(FLOW);
        u8t errorTemp = sensor.Measure(TEMP);
        u8t errorVdd = sensor.Measure(VDD);

        if (errorFlow == 0 && errorTemp == 0 && errorVdd == 0) {
            // Print raw values
            Serial.printf("Raw - Flow: %d, Temp: %d, Vdd: %u\n",
                         sensor.flow.i16, sensor.temperature.i16, sensor.vdd.u16);

            // Print converted values
            Serial.printf("Flow: %.2f %s | Temp: %.1f°C | Vdd: %.2fV\n",
                         sensor.getFlow(),
                         sensor.flowUnitStr,
                         sensor.getTemperature(),
                         sensor.getVdd());

            // Print memory info (ESP32 only)
            #ifdef ESP32
                Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
            #endif

            Serial.println();

            // Optional: Send data over WiFi
            /*
            if (WiFi.status() == WL_CONNECTED) {
                // Send data to server or MQTT broker
                // sendDataToServer(sensor.getFlow(), sensor.getTemperature());
            }
            */
        } else {
            Serial.printf("ERROR: Measurement failed (Flow=0x%02X, Temp=0x%02X, Vdd=0x%02X)\n",
                         errorFlow, errorTemp, errorVdd);
        }
    }

    // Optional: Handle web server requests
    // server.handleClient();  // If using web server

    // Small delay to prevent watchdog issues
    delay(10);
}

// Optional: Web server example
/*
#include <WebServer.h>  // ESP32
// #include <ESP8266WebServer.h>  // ESP8266

WebServer server(80);

void handleRoot() {
    String html = "<html><body>";
    html += "<h1>SF04 Flow Sensor</h1>";
    html += "<p>Flow: " + String(sensor.getFlow()) + " " + String(sensor.flowUnitStr) + "</p>";
    html += "<p>Temperature: " + String(sensor.getTemperature()) + " °C</p>";
    html += "<p>Voltage: " + String(sensor.getVdd()) + " V</p>";
    html += "<p><a href='/'>Refresh</a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.begin();
    Serial.println("HTTP server started");
}
*/
