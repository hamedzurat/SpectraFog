// ============================================================
// BMI160 Accelerometer & Gyroscope Sensor Implementation
// ============================================================

#include "sensor_bmi160.h"

BMI160Sensor::BMI160Sensor() {
  bmi160 = nullptr;
  i2c_address = BMI160_I2C_ADDRESS;
  currentData = {0, 0, 0, 0, 0, 0, false, 0};
  lastUpdate = 0;
  accel_lsb_per_g = 16384.0f;  // Default ±2g
  gyro_lsb_per_dps = 16.4f;    // Default ±2000°/s
}

bool BMI160Sensor::i2cRead8(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)i2c_address, 1) != 1) return false;
  value = Wire.read();
  return true;
}

bool BMI160Sensor::getAccelLsbPerG() {
  uint8_t r;
  if (!i2cRead8(0x41, r)) return false;  // ACC_RANGE register
  uint8_t code = r & 0x0F;
  switch (code) {
    case 0b0011: accel_lsb_per_g = 16384.0f; break; // ±2g
    case 0b0101: accel_lsb_per_g =  8192.0f; break; // ±4g
    case 0b1000: accel_lsb_per_g =  4096.0f; break; // ±8g
    case 0b1100: accel_lsb_per_g =  2048.0f; break; // ±16g
    default:     accel_lsb_per_g = 16384.0f; break;
  }
  return true;
}

bool BMI160Sensor::getGyroLsbPerDPS() {
  uint8_t r;
  if (!i2cRead8(0x43, r)) return false;  // GYR_RANGE register
  uint8_t code = r & 0x07;
  switch (code) {
    case 0b000: gyro_lsb_per_dps = 16.4f;   break; // ±2000°/s
    case 0b001: gyro_lsb_per_dps = 32.8f;   break; // ±1000°/s
    case 0b010: gyro_lsb_per_dps = 65.6f;   break; // ±500°/s
    case 0b011: gyro_lsb_per_dps = 131.2f;  break; // ±250°/s
    case 0b100: gyro_lsb_per_dps = 262.4f;  break; // ±125°/s
    default:    gyro_lsb_per_dps = 16.4f;   break;
  }
  return true;
}

bool BMI160Sensor::begin() {
  #if !USE_FAKE_BMI160_DATA
  Wire.begin(BMI160_PIN_SDA, BMI160_PIN_SCL);
  Wire.setClock(400000);
  
  bmi160 = new DFRobot_BMI160();
  
  // Try 0x68 first, then 0x69
  int8_t result = bmi160->I2cInit(BMI160_I2C_ADDRESS);
  if (result != 0) {
    Serial.println("BMI160: Not found at 0x68, trying 0x69...");
    i2c_address = 0x69;
    result = bmi160->I2cInit(0x69);
  }
  
  if (result != 0) {
    Serial.printf("BMI160: Failed to initialize (error code: %d)\n", result);
    delete bmi160;
    bmi160 = nullptr;
    return false;
  }
  
  // Read actual sensitivity settings from device
  getAccelLsbPerG();
  getGyroLsbPerDPS();
  
  Serial.println("BMI160: Initialized successfully");
  Serial.printf("BMI160: Accel range LSB/g = %.1f, Gyro range LSB/(°/s) = %.1f\n", 
                accel_lsb_per_g, gyro_lsb_per_dps);
  #else
  Serial.println("BMI160: Using FAKE DATA mode");
  #endif
  
  return true;
}

float BMI160Sensor::generateFakeAccelValue(float base, float range) {
  return base + (random(-1000, 1000) / 1000.0) * range;
}

float BMI160Sensor::generateFakeGyroValue(float range) {
  return (random(-1000, 1000) / 1000.0) * range;
}

void BMI160Sensor::update() {
  unsigned long now = millis();
  if (now - lastUpdate < BMI160_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = now;
  
  #if USE_FAKE_BMI160_DATA
  // Generate realistic fake data
  currentData.accel_x_mps2 = generateFakeAccelValue(0.2, 0.5);
  currentData.accel_y_mps2 = generateFakeAccelValue(-0.1, 0.5);
  currentData.accel_z_mps2 = generateFakeAccelValue(9.81, 0.3);
  currentData.gyro_x_radps = generateFakeGyroValue(0.1);
  currentData.gyro_y_radps = generateFakeGyroValue(0.1);
  currentData.gyro_z_radps = generateFakeGyroValue(0.1);
  currentData.valid = true;
  currentData.timestamp = now;
  #else
  if (!bmi160) {
    currentData.valid = false;
    return;
  }
  
  int16_t raw[6];  // ax, ay, az, gx, gy, gz
  if (bmi160->getAccelGyroData(raw) != 0) {
    currentData.valid = false;
    return;
  }
  
  // Convert to SI units
  const float G0 = 9.80665f;  // m/s² per g
  const float DEG2RAD = PI / 180.0f;
  
  currentData.accel_x_mps2 = (raw[0] / accel_lsb_per_g) * G0;
  currentData.accel_y_mps2 = (raw[1] / accel_lsb_per_g) * G0;
  currentData.accel_z_mps2 = (raw[2] / accel_lsb_per_g) * G0;
  
  currentData.gyro_x_radps = (raw[3] / gyro_lsb_per_dps) * DEG2RAD;
  currentData.gyro_y_radps = (raw[4] / gyro_lsb_per_dps) * DEG2RAD;
  currentData.gyro_z_radps = (raw[5] / gyro_lsb_per_dps) * DEG2RAD;
  
  currentData.valid = true;
  currentData.timestamp = now;
  #endif
  
  #if DEBUG_PRINT_SENSOR_DATA
  Serial.printf("BMI160: Accel[%.2f, %.2f, %.2f] m/s², Gyro[%.3f, %.3f, %.3f] rad/s\n", 
                currentData.accel_x_mps2, currentData.accel_y_mps2, currentData.accel_z_mps2,
                currentData.gyro_x_radps, currentData.gyro_y_radps, currentData.gyro_z_radps);
  #endif
}

BMI160Data BMI160Sensor::getData() {
  return currentData;
}

bool BMI160Sensor::isDataValid() {
  return currentData.valid;
}