// ESP32 + GY-BMI160 (I2C) — Print accel in m/s^2 and gyro in rad/s
// Requires: DFRobot_BMI160 (Arduino Library Manager)
//
// Notes:
// - Reads ACC_RANGE (0x41) to select the correct LSB/g, then converts raw to m/s^2.
// - Reads GYR_RANGE (0x43) to select LSB/(°/s), then converts raw to rad/s.
// - Default ESP32 I2C pins: SDA=21, SCL=22.
// - If your DFRobot library on Linux errors on <arduino.h>, change to <Arduino.h> in the .h file.

#include <Wire.h>
#include "DFRobot_BMI160.h"

static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const uint32_t I2C_HZ = 400000;

DFRobot_BMI160 bmi160;
uint8_t bmi_addr = 0x68; // will be auto-detected (0x68 or 0x69)

// --- Helpers to read BMI160 registers directly over I2C
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  value = Wire.read();
  return true;
}

// --- Determine accelerometer LSB/g from ACC_RANGE (0x41)
bool getAccelLsbPerG(float &lsb_per_g) {
  uint8_t r;
  if (!i2cRead8(bmi_addr, 0x41, r)) return false;
  uint8_t code = r & 0x0F; // acc_range<3:0>
  switch (code) {
    case 0b0011: lsb_per_g = 16384.0f; break; // ±2g
    case 0b0101: lsb_per_g =  8192.0f; break; // ±4g
    case 0b1000: lsb_per_g =  4096.0f; break; // ±8g
    case 0b1100: lsb_per_g =  2048.0f; break; // ±16g
    default:     lsb_per_g = 16384.0f; break; // "all other" -> ±2g
  }
  return true;
}

// --- Determine gyro LSB/(°/s) from GYR_RANGE (0x43)
bool getGyroLsbPerDPS(float &lsb_per_dps) {
  uint8_t r;
  if (!i2cRead8(bmi_addr, 0x43, r)) return false;
  uint8_t code = r & 0x07; // gyr_range<2:0>
  switch (code) {
    case 0b000: lsb_per_dps = 16.4f;   break; // ±2000 dps
    case 0b001: lsb_per_dps = 32.8f;   break; // ±1000 dps
    case 0b010: lsb_per_dps = 65.6f;   break; // ±500 dps
    case 0b011: lsb_per_dps = 131.2f;  break; // ±250 dps
    case 0b100: lsb_per_dps = 262.4f;  break; // ±125 dps
    default:    lsb_per_dps = 16.4f;   break; // default to ±2000 dps
  }
  return true;
}

// --- Optional: set a desired accel range via ACC_RANGE (keeps everything else)
// Pass one of: 0b0011 (±2g), 0b0101 (±4g), 0b1000 (±8g), 0b1100 (±16g)
bool setAccelRange(uint8_t acc_range_code) {
  // Read-modify-write to keep upper bits 7..4 as 0 per datasheet recommendation
  uint8_t r;
  if (!i2cRead8(bmi_addr, 0x41, r)) return false;
  r = (r & 0xF0) | (acc_range_code & 0x0F);
  if (!i2cWrite8(bmi_addr, 0x41, r)) return false;
  // Datasheet: read data registers after range change to clear stale DRDY
  uint8_t dummy;
  i2cRead8(bmi_addr, 0x04, dummy);
  return true;
}

void scanI2C() {
  Serial.println(F("I2C scan:"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  found 0x"));
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
    }
  }
}

bool initBMI160() {
  // Try 0x68 then 0x69
  int8_t ret = bmi160.I2cInit(0x68);
  if (ret == 0) { bmi_addr = 0x68; return true; }
  Serial.println(F("BMI160 not at 0x68, trying 0x69..."));
  ret = bmi160.I2cInit(0x69);
  if (ret == 0) { bmi_addr = 0x69; return true; }
  Serial.print(F("BMI160 init failed, code=")); Serial.println(ret);
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);

  Serial.println(F("\nESP32 + BMI160 (m/s^2, rad/s)"));
  scanI2C();
  if (!initBMI160()) {
    Serial.println(F("Check wiring, I2C address (SDO), and library include casing on Linux."));
    while (1) delay(1000);
  }

  // (Optional) force a known accel range; comment out to keep whatever the board ships with.
  // setAccelRange(0b0011); // ±2g
  // setAccelRange(0b0101); // ±4g
  // setAccelRange(0b1000); // ±8g
  // setAccelRange(0b1100); // ±16g

  Serial.println(F("time_ms, ax[m/s^2], ay[m/s^2], az[m/s^2], gx[rad/s], gy[rad/s], gz[rad/s]"));
}

void loop() {
  // Fetch raw accel+gyro
  int16_t raw[6]; // ax, ay, az, gx, gy, gz (raw signed 16-bit)
  if (bmi160.getAccelGyroData(raw) != 0) {
    Serial.println(F("Read error"));
    delay(100);
    return;
  }

  // Determine current sensitivities from device registers
  float lsb_per_g = 16384.0f; // default (±2g)
  float lsb_per_dps = 16.4f;  // default (±2000 dps)
  getAccelLsbPerG(lsb_per_g);
  getGyroLsbPerDPS(lsb_per_dps);

  // Convert to SI
  constexpr float G0 = 9.80665f;           // m/s^2 per g
  constexpr float DEG2RAD = 3.1415926535f / 180.0f;

  float ax = (raw[0] / lsb_per_g) * G0;
  float ay = (raw[1] / lsb_per_g) * G0;
  float az = (raw[2] / lsb_per_g) * G0;

  float gx = (raw[3] / lsb_per_dps) * DEG2RAD; // rad/s
  float gy = (raw[4] / lsb_per_dps) * DEG2RAD;
  float gz = (raw[5] / lsb_per_dps) * DEG2RAD;

  // Print
  uint32_t t = millis();
  Serial.print(t); Serial.print(", ");
  Serial.print(ax, 5); Serial.print(", ");
  Serial.print(ay, 5); Serial.print(", ");
  Serial.print(az, 5); Serial.print(", ");
  Serial.print(gx, 5); Serial.print(", ");
  Serial.print(gy, 5); Serial.print(", ");
  Serial.println(gz, 5);

  delay(500); // ~20 Hz
}
