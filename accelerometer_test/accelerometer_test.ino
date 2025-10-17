#include <Wire.h>

#define BMI160_I2C_ADDRESS 0x68
#define ACCEL_SENSITIVITY 16384.0  // Sensitivity for ±2g in LSB/g (adjust based on your configuration)

void setup() {
  Serial.begin(115200);  // Initialize Serial communication
  Wire.begin();          // Initialize I2C communication

  // Initialize BMI160 accelerometer
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E);  // Command register
  Wire.write(0x11);  // Set accelerometer to normal mode
  Wire.endTransmission();
  delay(100);

  // Perform accelerometer auto-calibration
  autoCalibrateAccelerometer();

  Serial.println("BMI160 Initialized and Calibrated");
}

void loop() {
  int16_t ax, ay, az;

  // Read accelerometer data
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x12);  // Start register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_I2C_ADDRESS, 6);

  if (Wire.available() == 6) {
    ax = (Wire.read() | (Wire.read() << 8));
    ay = (Wire.read() | (Wire.read() << 8));
    az = (Wire.read() | (Wire.read() << 8));
  }

  // Convert raw accelerometer values to m/s^2
  float ax_mps2 = ax * (9.81 / ACCEL_SENSITIVITY);
  float ay_mps2 = ay * (9.81 / ACCEL_SENSITIVITY);
  float az_mps2 = az * (9.81 / ACCEL_SENSITIVITY);

  // Print accelerometer values in m/s^2
  Serial.print("Accel (m/s^2): ");
  Serial.print(ax_mps2, 2);
  Serial.print(", ");
  Serial.print(ay_mps2, 2);
  Serial.print(", ");
  Serial.println(az_mps2, 2);

  delay(100);
}

void autoCalibrateAccelerometer() {
  // Configure accelerometer for auto-calibration
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E);  // Command register
  Wire.write(0x37);  // Start accelerometer offset calibration
  Wire.endTransmission();
  delay(100);

  // Wait for calibration to complete
  delay(1000);
  Serial.println("Accelerometer Auto-Calibration Complete");
}