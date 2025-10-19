// ============================================================
// ESP32 Multi-Sensor Configuration
// ============================================================

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>

// ============================================================
// WiFi Access Point Settings
// ============================================================
#define WIFI_SSID "ESP32-AP"
#define WIFI_PASSWORD "esp32pass"
#define WIFI_CHANNEL 1
#define MAX_WIFI_CONNECTIONS 4

// ============================================================
// Network Configuration
// ============================================================
#define ACCESS_POINT_IP IPAddress(10, 10, 10, 10)
#define ACCESS_POINT_GATEWAY IPAddress(10, 10, 10, 10)
#define ACCESS_POINT_SUBNET IPAddress(255, 255, 255, 0)
#define HTTP_SERVER_PORT 80
#define WEBSOCKET_PORT 81

// ============================================================
// Pin Assignments - I2C Sensors
// ============================================================
#define BMI160_PIN_SDA 21
#define BMI160_PIN_SCL 22
#define BMI160_I2C_ADDRESS 0x68

#define SI7021_PIN_SDA 19
#define SI7021_PIN_SCL 23
#define SI7021_I2C_ADDRESS 0x40

// ============================================================
// Pin Assignments - Digital Sensors
// ============================================================
#define PIR_MOTION_PIN 27
#define ULTRASONIC_TRIGGER_PIN 5
#define ULTRASONIC_ECHO_PIN 18

// ============================================================
// Pin Assignments - UART Sensors
// ============================================================
#define RADAR_UART_RX_PIN 16
#define RADAR_UART_TX_PIN 17
#define RADAR_BAUD_RATE 256000

// ============================================================
// Pin Assignments - Outputs
// ============================================================
#define BUZZER_PIN 25
#define STATUS_LED_PIN 2

// ============================================================
// Sensor Update Intervals (milliseconds)
// ============================================================
#define BMI160_UPDATE_INTERVAL_MS 100
#define SI7021_UPDATE_INTERVAL_MS 1000
#define PIR_UPDATE_INTERVAL_MS 200
#define ULTRASONIC_UPDATE_INTERVAL_MS 200
#define RADAR_UPDATE_INTERVAL_MS 100
#define WEBSOCKET_BROADCAST_INTERVAL_MS 100

// ============================================================
// Sensor Enable/Disable Flags
// ============================================================
#define ENABLE_BMI160_ACCELEROMETER true
#define ENABLE_SI7021_TEMP_HUMIDITY true
#define ENABLE_PIR_MOTION true
#define ENABLE_ULTRASONIC_DISTANCE true
#define ENABLE_RADAR_LD2450 true
#define ENABLE_BUZZER true

// ============================================================
// Fake Data Generator (for testing without sensors)
// ============================================================
#define USE_FAKE_BMI160_DATA false
#define USE_FAKE_SI7021_DATA false
#define USE_FAKE_PIR_DATA false
#define USE_FAKE_ULTRASONIC_DATA false
#define USE_FAKE_RADAR_DATA false

// ============================================================
// BMI160 Accelerometer Settings
// ============================================================
#define BMI160_ACCEL_SENSITIVITY 16384.0  // LSB/g for ±2g range
#define BMI160_AUTO_CALIBRATE_ON_BOOT true

// ============================================================
// SI7021 Temperature/Humidity Settings
// ============================================================
#define SI7021_HUMIDITY_CMD 0xF5
#define SI7021_TEMPERATURE_CMD 0xF3

// ============================================================
// PIR Motion Sensor Settings
// ============================================================
#define PIR_DEBOUNCE_TIME_MS 10

// ============================================================
// HC-SR04 Ultrasonic Settings
// ============================================================
#define ULTRASONIC_TIMEOUT_US 30000
#define ULTRASONIC_AGGREGATE_SAMPLES 5
#define ULTRASONIC_SAMPLE_GAP_MS 10
#define ULTRASONIC_USE_MEDIAN false  // false = mean
#define ULTRASONIC_MIN_DISTANCE_CM 2.0
#define ULTRASONIC_MAX_DISTANCE_CM 400.0
#define ULTRASONIC_SPEED_OF_SOUND_MS 343.0

// ============================================================
// HLK-LD2450 Radar Settings
// ============================================================
#define RADAR_MAX_TARGETS 3
#define RADAR_FRAME_SIZE 30

// ============================================================
// Buzzer Settings
// ============================================================
#define BUZZER_DEFAULT_ENABLED false

// ============================================================
// Debug Settings
// ============================================================
#define SERIAL_BAUD_RATE 115200
#define DEBUG_PRINT_SENSOR_DATA false
#define DEBUG_PRINT_WEBSOCKET_MESSAGES false

#endif // CONFIG_H