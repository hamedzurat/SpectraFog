# ESP32 Multi-Sensor WebSocket JSON Schema

This document describes the JSON data structure broadcast by the ESP32 via WebSocket.

## Root Structure

```json
{
  "bmi160": { ... },
  "si7021": { ... },
  "pir": { ... },
  "ultrasonic": { ... },
  "radar": { ... },
  "system": { ... }
}
```

---

## 1. BMI160 Accelerometer & Gyroscope (`bmi160`)

Provides 3-axis acceleration data in m/s² and 3-axis gyroscope data in rad/s.

```json
{
  "bmi160": {
    "ax": 0.15,             // X-axis acceleration (m/s²)
    "ay": -0.08,            // Y-axis acceleration (m/s²)
    "az": 9.78,             // Z-axis acceleration (m/s²)
    "gx": 0.025,            // X-axis rotation rate (rad/s)
    "gy": -0.012,           // Y-axis rotation rate (rad/s)
    "gz": 0.003,            // Z-axis rotation rate (rad/s)
    "valid": true,          // Data validity (boolean)
    "timestamp": 123456     // Milliseconds since boot
  }
}
```

**Fields:**
- `ax`, `ay`, `az` (float): Acceleration on each axis in meters per second squared
- `gx`, `gy`, `gz` (float): Angular velocity on each axis in radians per second
- `valid` (boolean): `true` if sensor reading is valid
- `timestamp` (unsigned long): System milliseconds when data was captured

**Notes:**
- Uses DFRobot_BMI160 library
- Automatically detects I2C address (0x68 or 0x69)
- Reads actual sensor range settings to calculate correct values
- Typical Z-axis acceleration ≈ 9.81 m/s² when device is flat (gravity)

---

## 2. SI7021 Temperature & Humidity (`si7021`)

Temperature and relative humidity measurements.

```json
{
  "si7021": {
    "temp_c": 22.5,         // Temperature in Celsius
    "temp_f": 72.5,         // Temperature in Fahrenheit
    "humidity": 45.2,       // Relative humidity (%)
    "valid": true,          // Data validity
    "timestamp": 123456     // Milliseconds since boot
  }
}
```

**Fields:**
- `temp_c` (float): Temperature in degrees Celsius
- `temp_f` (float): Temperature in degrees Fahrenheit
- `humidity` (float): Relative humidity percentage (0-100)
- `valid` (boolean): `true` if sensor reading is valid
- `timestamp` (unsigned long): System milliseconds when data was captured

---

## 3. PIR Motion Sensor (`pir`)

Passive infrared motion detection.

```json
{
  "pir": {
    "motion": true,         // Motion detected (boolean)
    "start_time": 120000,   // When motion started (ms)
    "duration": 3450,       // Duration of motion (ms)
    "timestamp": 123456     // Current timestamp (ms)
  }
}
```

**Fields:**
- `motion` (boolean): `true` if motion is currently detected
- `start_time` (unsigned long): Timestamp when current motion event started
- `duration` (unsigned long): How long motion has been detected (milliseconds)
- `timestamp` (unsigned long): System milliseconds when data was captured

**Logic:**
- When `motion` changes from `false` to `true`, `start_time` is set
- `duration` increments while motion is `true`
- When motion ends, `duration` holds the total motion time

---

## 4. HC-SR04 Ultrasonic Distance (`ultrasonic`)

Distance measurement using ultrasonic waves.

```json
{
  "ultrasonic": {
    "distance": 45.3,       // Distance in centimeters
    "raw_us": 2640,         // Raw echo time (microseconds)
    "valid": true,          // Measurement validity
    "timestamp": 123456     // Milliseconds since boot
  }
}
```

**Fields:**
- `distance` (float): Calculated distance in centimeters
- `raw_us` (unsigned long): Raw echo pulse duration in microseconds
- `valid` (boolean): `true` if measurement is valid (object detected)
- `timestamp` (unsigned long): System milliseconds when data was captured

**Range:**
- Valid distance: 2 cm to 400 cm (configurable)
- `valid` is `false` when no echo is received (timeout)

---

## 5. HLK-LD2450 Radar (`radar`)

Multi-target radar tracking with position and velocity.

```json
{
  "radar": {
    "frames_total": 1523,   // Total frames received
    "frames_valid": 1520,   // Valid frames parsed
    "timestamp": 123456,    // Milliseconds since boot
    "targets": [
      {
        "valid": true,      // Target detected
        "x": 1200,          // X position (millimeters)
        "y": 2500,          // Y position (millimeters)
        "v": -15,           // Velocity (cm/s)
        "age": 50           // Time since last update (ms)
      },
      {
        "valid": false,
        "x": 0,
        "y": 0,
        "v": 0,
        "age": 5000
      },
      {
        "valid": true,
        "x": -800,
        "y": 1800,
        "v": 22,
        "age": 100
      }
    ]
  }
}
```

**Fields:**
- `frames_total` (unsigned long): Total UART frames received
- `frames_valid` (unsigned long): Frames that passed checksum validation
- `timestamp` (unsigned long): System milliseconds when data was captured
- `targets` (array): Array of 3 target objects (always 3 elements)

**Target Object:**
- `valid` (boolean): `true` if target is currently detected
- `x` (int16): X-coordinate in millimeters (negative = left, positive = right)
- `y` (int16): Y-coordinate in millimeters (distance from sensor)
- `v` (int16): Velocity in centimeters per second (negative = approaching, positive = receding)
- `age` (unsigned long): Milliseconds since target was last updated

**Coordinate System:**
- X-axis: Horizontal (left/right from sensor's perspective)
- Y-axis: Depth (distance from sensor)
- Origin: Sensor location

---

## 6. System Information (`system`)

ESP32 system statistics.

```json
{
  "system": {
    "uptime": 123456,       // System uptime (ms)
    "free_heap": 245680,    // Free heap memory (bytes)
    "clients": 2            // Connected WebSocket clients
  }
}
```

**Fields:**
- `uptime` (unsigned long): Milliseconds since ESP32 booted
- `free_heap` (uint32): Available heap memory in bytes
- `clients` (int): Number of connected WebSocket clients

---

## Complete Example

```json
{
  "bmi160": {
    "ax": 0.15,
    "ay": -0.08,
    "az": 9.78,
    "gx": 0.025,
    "gy": -0.012,
    "gz": 0.003,
    "valid": true,
    "timestamp": 123456
  },
  "si7021": {
    "temp_c": 22.5,
    "temp_f": 72.5,
    "humidity": 45.2,
    "valid": true,
    "timestamp": 123456
  },
  "pir": {
    "motion": true,
    "start_time": 120000,
    "duration": 3456,
    "timestamp": 123456
  },
  "ultrasonic": {
    "distance": 45.3,
    "raw_us": 2640,
    "valid": true,
    "timestamp": 123456
  },
  "radar": {
    "frames_total": 1523,
    "frames_valid": 1520,
    "timestamp": 123456,
    "targets": [
      {
        "valid": true,
        "x": 1200,
        "y": 2500,
        "v": -15,
        "age": 50
      },
      {
        "valid": false,
        "x": 0,
        "y": 0,
        "v": 0,
        "age": 5000
      },
      {
        "valid": true,
        "x": -800,
        "y": 1800,
        "v": 22,
        "age": 100
      }
    ]
  },
  "system": {
    "uptime": 123456,
    "free_heap": 245680,
    "clients": 2
  }
}
```

---

## Usage Notes

### Update Rate
- Default broadcast interval: 100ms (configurable via `WEBSOCKET_BROADCAST_INTERVAL_MS`)
- Individual sensor update rates vary (see `config.h`)

### Data Validation
- Always check the `valid` field before using sensor data
- Invalid data may contain stale or zero values

### Timestamps
- All timestamps are in milliseconds since ESP32 boot
- Useful for calculating data age and synchronization

### WebSocket Connection
- Endpoint: `ws://<ESP32_IP>:81/`
- Default IP: `10.10.10.10`
- Automatic reconnection recommended on disconnect

### Building Custom UI
When creating a custom interface:
1. Parse the JSON on each WebSocket message
2. Check `valid` flags before displaying data
3. Use timestamps for age calculation and timeout detection
4. Handle missing fields gracefully (sensors may be disabled)
5. Implement reconnection logic for reliability

---

## Configuration

All sensor enable/disable flags and fake data generators are in `config.h`:

```cpp
#define ENABLE_BMI160_ACCELEROMETER true
#define ENABLE_SI7021_TEMP_HUMIDITY true
#define ENABLE_PIR_MOTION true
#define ENABLE_ULTRASONIC_DISTANCE true
#define ENABLE_RADAR_LD2450 true

#define USE_FAKE_BMI160_DATA false
#define USE_FAKE_SI7021_DATA false
#define USE_FAKE_PIR_DATA false
#define USE_FAKE_ULTRASONIC_DATA false
#define USE_FAKE_RADAR_DATA false
```

Set `USE_FAKE_*_DATA` to `true` to test UI without physical sensors connected.
