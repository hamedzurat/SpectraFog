# 🚀 Quick Start Guide

Get your ESP32 sensor system running in 5 minutes!

## ⚡ Super Quick Setup (No Sensors)

**Want to test the web interface first?** Use fake data mode!

### Step 1: Install Libraries (3 minutes)
Open Arduino IDE and install:
1. **ESP32 Board Support** (Boards Manager)
2. **WebSockets** by Markus Sattler
3. **ArduinoJson** by Benoit Blanchon (version 6.x)

See `LIBRARIES.txt` for detailed instructions.

### Step 2: Enable Fake Data (30 seconds)
Edit `config.h`:
```cpp
#define USE_FAKE_BMI160_DATA true
#define USE_FAKE_SI7021_DATA true
#define USE_FAKE_PIR_DATA true
#define USE_FAKE_ULTRASONIC_DATA true
#define USE_FAKE_RADAR_DATA true
```

### Step 3: Upload (1 minute)
1. Connect ESP32 via USB
2. Select your board: `Tools > Board > ESP32 Dev Module`
3. Select COM port: `Tools > Port > [Your Port]`
4. Click **Upload** ⬆️

### Step 4: Connect (30 seconds)
1. Open Serial Monitor (115200 baud)
2. Note the WiFi credentials
3. Connect to WiFi: **ESP32-AP** / **esp32pass**
4. Open browser: **http://10.10.10.10**

**🎉 Done! You should see fake sensor data updating live.**

---

## 🔧 Full Setup (With Physical Sensors)

### Hardware Checklist
- [ ] ESP32 board
- [ ] BMI160 accelerometer
- [ ] SI7021 temp/humidity sensor
- [ ] HC-SR04 ultrasonic sensor
- [ ] PIR motion sensor
- [ ] HLK-LD2450 radar
- [ ] Passive buzzer (optional)
- [ ] Jumper wires
- [ ] Breadboard

### Wiring Guide

**I2C Sensors** (Two separate buses):
```
BMI160:
  VCC → 3.3V
  GND → GND
  SDA → GPIO 21
  SCL → GPIO 22

SI7021:
  VCC → 3.3V
  GND → GND
  SDA → GPIO 19
  SCL → GPIO 23
```

**Digital Sensors**:
```
PIR:
  VCC → 5V
  GND → GND
  OUT → GPIO 27

HC-SR04:
  VCC → 5V
  GND → GND
  TRIG → GPIO 5
  ECHO → GPIO 18 (⚠️ use voltage divider if 5V)

Buzzer:
  + → GPIO 25
  - → GND
```

**UART Radar**:
```
HLK-LD2450:
  VCC → 5V
  GND → GND
  TX → GPIO 16 (ESP32 RX)
  RX → GPIO 17 (ESP32 TX)
```

### Software Setup
1. Install libraries (see above)
2. Wire all sensors
3. In `config.h`, set fake data to `false`:
   ```cpp
   #define USE_FAKE_BMI160_DATA false
   #define USE_FAKE_SI7021_DATA false
   // etc...
   ```
4. Upload code
5. Open Serial Monitor to verify sensors initialize
6. Connect to ESP32-AP and access dashboard

---

## 🐛 Troubleshooting

### Nothing Happens After Upload
- Open Serial Monitor (115200 baud)
- Press ESP32 RESET button
- Check for error messages

### Can't Upload Code
- Hold **BOOT** button while clicking Upload
- Try different USB cable/port
- Check board selection in Tools menu

### Web Page Won't Load
1. Verify you're connected to **ESP32-AP** WiFi
2. Try http://10.10.10.10 (not https)
3. Check Serial Monitor for actual IP address
4. Some phones require "Use network without internet" option

### Sensor Shows "Invalid" Data
1. Enable debug in config.h:
   ```cpp
   #define DEBUG_PRINT_SENSOR_DATA true
   ```
2. Check Serial Monitor for error messages
3. Verify wiring matches pinout
4. Test that specific sensor with fake data mode
5. Try I2C scanner sketch for I2C sensors

### Compilation Errors
- **"WiFi.h not found"** → Install ESP32 board support
- **"WebSocketsServer.h not found"** → Install WebSockets library
- **"ArduinoJson.h not found"** → Install ArduinoJson library
- **"no matching function"** → Check ArduinoJson version (use 6.x)

---

## 📱 Access Dashboard

### From Computer
1. Connect to WiFi: **ESP32-AP**
2. Password: **esp32pass**
3. Open: http://10.10.10.10

### From Phone/Tablet
1. Join WiFi network: **ESP32-AP**
2. Enter password: **esp32pass**
3. May need to tap "Use network without internet"
4. Open browser: http://10.10.10.10

### Can't Remember IP?
Check Serial Monitor - IP address is printed on startup.

---

## 🎯 What You Should See

### Serial Monitor Output
```
╔════════════════════════════════════════╗
║   ESP32 Multi-Sensor System v1.0      ║
╚════════════════════════════════════════╝

WiFi: Starting Access Point...
WiFi: AP started successfully
WiFi: SSID: ESP32-AP
WiFi: IP Address: 10.10.10.10
WiFi: Connect and open: http://10.10.10.10
WebSocket: Server started on port 80
HTTP Server: Started

--- Initializing Sensors ---
BMI160: Initialized successfully
SI7021: Initialized successfully
PIR: Initialized successfully
Ultrasonic: Initialized successfully
Radar: Initialized successfully
Buzzer: Initialized successfully

--- Initialization Complete ---
System ready!
```

### Web Dashboard
- Purple gradient background
- Live sensor cards updating in real-time
- Green "Connected" status indicator
- All sensor values changing dynamically

---

## ⚙️ Quick Configuration

All settings in **config.h**:

### Change WiFi Credentials
```cpp
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
```

### Disable Sensors You Don't Have
```cpp
#define ENABLE_BMI160_ACCELEROMETER false
#define ENABLE_RADAR_LD2450 false
// etc...
```

### Change Update Rates
```cpp
#define BMI160_UPDATE_INTERVAL_MS 100  // 10 Hz
#define WEBSOCKET_BROADCAST_INTERVAL_MS 200  // 5 Hz
```

### Change IP Address
```cpp
#define ACCESS_POINT_IP IPAddress(192, 168, 4, 1)
```

---

## 📊 Understanding the Data

### Accelerometer (BMI160)
- **X, Y, Z**: Acceleration in m/s²
- **Z ≈ 9.8**: Normal when flat (gravity)
- **Valid**: Sensor responding correctly

### Temperature (SI7021)
- **Temp**: Both Celsius and Fahrenheit
- **Humidity**: Percentage (0-100%)
- Takes ~2 seconds to stabilize

### Motion (PIR)
- **Red dot**: Motion detected
- **Duration**: How long motion active
- Needs 30-60s warm-up after power on

### Distance (Ultrasonic)
- **Range**: 2 cm to 400 cm
- **Invalid**: No object in range or timeout
- **Raw μs**: Echo pulse duration

### Radar (LD2450)
- **3 Targets**: Can track up to 3 objects
- **X/Y**: Position in millimeters
- **V**: Velocity in cm/s (negative = approaching)
- **Age**: Time since last update

---

## 🎨 Customizing the UI

See **JSON_Schema.md** for complete data format.

Quick example - add custom display:
```javascript
// In web_interface.h, inside updateUI(data):
if (data.bmi160) {
  let magnitude = Math.sqrt(
    data.bmi160.x ** 2 + 
    data.bmi160.y ** 2 + 
    data.bmi160.z ** 2
  );
  console.log('Acceleration magnitude:', magnitude);
}
```

---

## ✅ Next Steps

After getting it working:
1. Read **README.md** for full documentation
2. Check **JSON_Schema.md** to understand data format
3. Modify **web_interface.h** to customize UI
4. Adjust sensor settings in **config.h**
5. Add your own sensors following the existing pattern

---

## 💡 Tips

- **Start with fake data** to verify software works
- **Add sensors one at a time** for easier debugging
- **Watch Serial Monitor** for helpful error messages
- **Check wiring twice** before powering on
- **Use multimeter** to verify 3.3V/5V levels
- **Keep USB connected** for power and debugging

---

## 🆘 Still Need Help?

1. ✅ Check Serial Monitor at 115200 baud
2. ✅ Enable debug output in config.h
3. ✅ Test with fake data mode
4. ✅ Verify all wiring connections
5. ✅ Check library versions
6. ✅ Try the troubleshooting section above

Happy building! 🚀✨
