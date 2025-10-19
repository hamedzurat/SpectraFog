// ============================================================
// SI7021 Temperature/Humidity Sensor Implementation
// ============================================================

#include "sensor_si7021.h"

SI7021Sensor::SI7021Sensor(TwoWire* i2cBus) {
  wire = i2cBus;
  currentData = {0, 0, 0, false, 0};
  lastUpdate = 0;
}

bool SI7021Sensor::begin() {
  #if !USE_FAKE_SI7021_DATA
  wire->begin(SI7021_PIN_SDA, SI7021_PIN_SCL);
  delay(100);
  
  if (!checkI2CConnection()) {
    Serial.println("SI7021: Failed to detect sensor");
    return false;
  }
  
  Serial.println("SI7021: Initialized successfully");
  #else
  Serial.println("SI7021: Using FAKE DATA mode");
  #endif
  
  return true;
}

bool SI7021Sensor::checkI2CConnection() {
  wire->beginTransmission(SI7021_I2C_ADDRESS);
  byte error = wire->endTransmission();
  return (error == 0);
}

float SI7021Sensor::readHumidity() {
  wire->beginTransmission(SI7021_I2C_ADDRESS);
  wire->write(SI7021_HUMIDITY_CMD);
  wire->endTransmission();
  delay(50);
  
  wire->requestFrom(SI7021_I2C_ADDRESS, 2);
  if (wire->available() == 2) {
    uint8_t msb = wire->read();
    uint8_t lsb = wire->read();
    float humidity = ((msb << 8) | lsb) * 125.0 / 65536.0 - 6.0;
    return humidity;
  }
  return -1.0;
}

float SI7021Sensor::readTemperatureCelsius() {
  wire->beginTransmission(SI7021_I2C_ADDRESS);
  wire->write(SI7021_TEMPERATURE_CMD);
  wire->endTransmission();
  delay(50);
  
  wire->requestFrom(SI7021_I2C_ADDRESS, 2);
  if (wire->available() == 2) {
    uint8_t msb = wire->read();
    uint8_t lsb = wire->read();
    float temperature = ((msb << 8) | lsb) * 175.72 / 65536.0 - 46.85;
    return temperature;
  }
  return -100.0;
}

float SI7021Sensor::celsiusToFahrenheit(float celsius) {
  return celsius * 1.8 + 32;
}

void SI7021Sensor::update() {
  unsigned long now = millis();
  if (now - lastUpdate < SI7021_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = now;
  
  #if USE_FAKE_SI7021_DATA
  // Generate realistic fake temp/humidity data
  currentData.temperature_celsius = 22.0 + (random(-20, 50) / 10.0);
  currentData.humidity_percent = 50.0 + (random(-100, 100) / 10.0);
  currentData.temperature_fahrenheit = celsiusToFahrenheit(currentData.temperature_celsius);
  currentData.valid = true;
  currentData.timestamp = now;
  #else
  float temp = readTemperatureCelsius();
  float hum = readHumidity();
  
  if (temp > -100 && hum >= 0) {
    currentData.temperature_celsius = temp;
    currentData.temperature_fahrenheit = celsiusToFahrenheit(temp);
    currentData.humidity_percent = hum;
    currentData.valid = true;
    currentData.timestamp = now;
  } else {
    currentData.valid = false;
  }
  #endif
  
  #if DEBUG_PRINT_SENSOR_DATA
  Serial.printf("SI7021: Temp=%.1f°C Hum=%.1f%%\n", 
                currentData.temperature_celsius, 
                currentData.humidity_percent);
  #endif
}

SI7021Data SI7021Sensor::getData() {
  return currentData;
}

bool SI7021Sensor::isDataValid() {
  return currentData.valid;
}