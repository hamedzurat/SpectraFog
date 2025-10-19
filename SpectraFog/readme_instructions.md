# ESP32 Multi-Sensor WebSocket Server

A complete ESP32-based sensor monitoring system with real-time WebSocket data streaming and web interface.

## 📋 Features

- **6 Sensor Types**: Accelerometer, Temperature/Humidity, Motion, Distance, Radar, and Buzzer
- **Real-time WebSocket Streaming**: Live sensor data broadcast at 100ms intervals
- **WiFi Access Point**: Creates its own network for standalone operation
- **Web Dashboard**: Beautiful responsive interface accessible from any device
- **Modular Design**: Separate files for faster compilation
- **Fake Data Generators**: Test UI without physical sensors
- **Comprehensive Configuration**: All settings in one place

## 🔌 Hardware Requirements

### Components
1. **ESP32 Development Board** (any variant with WiFi)
2. **BMI160** - 3-Axis Accelerometer (I2C)
3. **SI7021** - Temperature & Humidity Sensor (I2C)
4. **HC-SR04** - Ultrasonic Distance Sensor
5. **PIR Motion Sensor** (HC-SR501 or similar)
6. **HLK-LD2450** - Multi-Target Radar (UART)
7. **Passive Buzzer** (optional)

### Pin Connections

| Component | ESP32 Pin | Function | Notes |
|-----------|-----------|----------|-------|
| BMI160 SDA | GPIO 21 | I2C Data | I2C Bus 1 |
| BMI160 SCL | GPIO 22 | I2C Clock | I2C Bus 1 |
| SI7021 SDA | GPIO 19 | I2C Data | I2C Bus 2 |
| SI7021 SCL | GPIO 23 | I2C Clock | I2C Bus 2 |
| PIR Signal | GPIO 27 | Digital Input | Motion detection |
| HC-SR04 TRIG | GPIO 5 | Digital Output | Trigger pulse |
| HC-SR04 ECHO | GPIO 18 | Digital Input | Echo measurement |
| LD2450 TX | GPIO 16 | UART RX | Radar data receive |
| LD2450 RX | GPIO 17 | UART TX | Radar commands (optional) |
| Buzzer | GPIO 25 | PWM Output | Tone generation |
| Status LED | GPIO 2 | Digital Output | Built-in LED |

**Power:**
- All sensors: Connect VCC to 3.3V or 5V (per sensor spec)
- All sensors: Connect GND to ESP32 GND
- HC-SR04: Requires 5V, use voltage divider on ECHO if needed

## 📦 Required Arduino Libraries

Install these via Arduino IDE Library Manager:

1. **WiFi** (Built-in with ESP32 core)
2. **WebSocketsServer** by Markus Sattler
   - Search: "WebSockets by Markus Sattler"
3. **ArduinoJson** by Benoit Blanchon
   - Search: "ArduinoJson"
   - Version 6.x recommended
4. **DFRobot_BMI160** by DFRobot
   - Search: "DFRobot BMI160"
   - For BMI160 accelerometer/gyroscope sensor
5. **Wire** (Built-in)

### Installing ESP32 Board Support

1. Open Arduino IDE
2. Go to **File > Preferences**
3. Add to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools > Board > Boards Manager**
5. Search "ESP32" and install "ESP32 by Espressif Systems"

## 🚀 Installation

### 1. Download Files

Place all files in the same folder with this structure:

```
esp32_sensor_system/
├── main.ino
├── config.h
├── sensor_bmi160.h
├── sensor_bmi160.cpp
├── sensor_si7021.h
├── sensor_si7021.cpp
├── sensor_pir.h
├── sensor_pir.cpp
├── sensor_ultrasonic.h
├── sensor_ultrasonic.cpp
├── sensor_radar.h
├── sensor_radar.cpp
├── buzzer.h
├── buzzer.cpp
├── wifi_manager.h
├── wifi_manager.cpp
└── web_interface.h
```

### 2. Configure Settings

Edit `config.h` to match your setup:

```cpp
// WiFi Settings
#define WIFI_SSID "ESP32-AP"
#define WIFI_PASSWORD "esp32pass"

// Enable/Disable Sensors
#define ENABLE_BMI160_ACCELEROMETER true
#define ENABLE_SI7021_TEMP_HUMIDITY true
#define ENABLE_PIR_MOTION true
#define ENABLE_ULTRASONIC_DISTANCE true
#define ENABLE_RADAR_LD2450 true

// Use Fake Data for Testing (no physical sensors needed)
#define USE_FAKE_BMI160_DATA false
#define USE_FAKE_SI7021_DATA false
#define USE_FAKE_PIR_DATA false
#define USE_FAKE_ULTRASONIC_DATA false
#define USE_FAKE_RADAR_DATA false
```

### 3. Upload to ESP32

1. Connect ESP32 via USB
2. Select **Tools > Board > ESP32 Dev Module** (or your specific board)
3. Select **Tools > Port** (your ESP32's COM port)
4. Click **Upload** (or press Ctrl+U)

### 4. Monitor Serial Output

1. Open **Tools > Serial Monitor**
2. Set baud rate to **115200**
3. You should see initialization messages and WiFi credentials

## 🌐 Accessing the Dashboard

### Method 1: Direct Connection
1. Connect your device (phone/laptop) to WiFi network: **ESP32-AP**
2. Password: **esp32pass**
3. Open browser and go to: **http://10.10.10.10**

### Method 2: Find IP via Serial Monitor
1. Check Serial Monitor for the IP address
2. Connect to the ESP32-AP network
3. Navigate to the displayed IP address

## 🧪 Testing Without Sensors

To test the system without connecting physical sensors:

1. Open `config.h`
2. Enable fake data for sensors you want to test:
   ```cpp
   #define USE_FAKE_BMI160_DATA true
   #define USE_FAKE_SI7021_DATA true
   #define USE_FAKE_PIR_DATA true
   #define USE_FAKE_ULTRASONIC_DATA true
   #define USE_FAKE_RADAR_DATA true
   ```
3. Upload and test the web interface with simulated data

## ⚙️ Configuration Options

### Update Intervals (milliseconds)

```cpp
#define BMI160_UPDATE_INTERVAL_MS 100
#define SI7021_UPDATE_INTERVAL_MS 1000
#define PIR_UPDATE_INTERVAL_MS 200
#define ULTRASONIC_UPDATE_INTERVAL_MS 200
#define RADAR_UPDATE_INTERVAL_MS 100
#define WEBSOCKET_BROADCAST_INTERVAL_MS 100
```

### Network Settings

```cpp
#define WIFI_SSID "ESP32-AP"
#define WIFI_PASSWORD "esp32pass"
#define ACCESS_POINT_IP IPAddress(10, 10, 10, 10)
#define WEBSOCKET_PORT 80
```

### Debug Options

```cpp
#define DEBUG_PRINT_SENSOR_DATA false
#define DEBUG_PRINT_WEBSOCKET_MESSAGES false
```

Set to `true` for verbose serial output.

## 🎨 Customizing the UI

The web interface is defined in `web_interface.h`. To customize:

1. Modify the HTML/CSS/JavaScript in the `WEB_PAGE` constant
2. The UI receives JSON data via WebSocket - see `JSON_Schema.md` for structure
3. Update the `updateUI(data)` JavaScript function to change how data is displayed

For a completely new UI, you can:
- Keep the WebSocket data format the same
- Replace the entire `WEB_PAGE` content
- Reference the JSON schema for available data fields

## 📊 JSON Data Format

See **JSON_Schema.md** for complete documentation of the WebSocket data structure.

Quick example:
```json
{
  "bmi160": { "x": 0.15, "y": -0.08, "z": 9.78, "valid": true },
  "si7021": { "temp_c": 22.5, "humidity": 45.2, "valid": true },
  "pir": { "motion": true, "duration": 3456 },
  "ultrasonic": { "distance": 45.3, "valid": true },
  "radar": { "targets": [...] },
  "system": { "uptime": 123456, "free_heap": 245680 }
}
```

## 🔧 Troubleshooting

### Compilation Errors

**Error: "WiFi.h not found"**
- Install ESP32 board support (see Installation section)

**Error: "WebSocketsServer.h not found"**
- Install "WebSockets by Markus Sattler" library

**Error: "ArduinoJson.h not found"**
- Install "ArduinoJson" library (version 6.x)

### Upload Issues

**"Failed to connect to ESP32"**
- Hold BOOT button while uploading
- Try different USB cable/port
- Check correct COM port selected

### Runtime Issues

**Can't connect to WiFi**
- Check SSID/password in config.h
- Some devices don't show hidden networks - network is visible by default

**Web page doesn't load**
- Verify IP address in Serial Monitor
- Ensure connected to ESP32-AP network
- Try http:// explicitly (not https://)

**No sensor data**
- Check wiring matches pin configuration
- Verify sensors are powered (3.3V or 5V per spec)
- Enable debug output in config.h
- Try fake data mode to isolate issue

**WebSocket disconnects**
- Normal behavior if no activity - reconnects automatically
- Check `free_heap` in system data - may be low memory

### Sensor-Specific Issues

**BMI160 not detected**
- Verify I2C address (0x68 or 0x69 depending on SDO pin)
- Check SDA/SCL connections (GPIO 21/22)
- Test with I2C scanner sketch

**SI7021 not responding**
- Confirm I2C address (0x40)
- Verify using second I2C bus (GPIO 19/23)
- Check 3.3V power connection

**PIR always HIGH or always LOW**
- PIR needs 30-60 second warm-up time after power on
- Adjust sensitivity potentiometer on sensor
- Check if sensor has jumper for retrigger mode

**Ultrasonic returns invalid readings**
- Verify 5V power for HC-SR04
- Check ECHO pin voltage divider if using 5V sensor
- Ensure no obstacles within minimum range (2cm)
- Increase timeout value in config.h

**Radar shows no targets**
- Confirm 256000 baud rate (unusual, double-check sensor spec)
- Verify TX/RX are not swapped
- Check radar power supply (typically 5V)
- Enable debug output to see frame reception

## 📈 Performance Tips

### Compilation Speed
The code is split into multiple files for faster incremental compilation:
- Arduino IDE only recompiles changed files
- Modify individual sensor files without recompiling everything
- Config changes require full recompile

### Memory Usage
- Monitor `free_heap` in system data
- If heap is low (<30KB), disable unused sensors
- Reduce JSON buffer size if needed
- Consider reducing WebSocket broadcast rate

### Network Performance
- Default 100ms broadcast rate = 10 updates/second
- Increase interval for slower networks
- Multiple clients share same data (efficient)

## 🔐 Security Notes

**⚠️ This is for local/development use only**

- Default password is weak - change in production
- No encryption on WiFi or WebSocket
- Access point is visible to all nearby devices
- No authentication on web interface
- Don't expose to public networks

For production use, consider:
- Strong WiFi password
- WSS (WebSocket Secure)
- Authentication layer
- Hidden SSID
- MAC address filtering

## 📝 License

This project is provided as-is for educational and personal use.

## 🤝 Contributing

To modify or extend:

1. **Add new sensor**: Create `.h` and `.cpp` files following existing pattern
2. **Update config.h**: Add enable flag and pin definitions
3. **Update main.ino**: Initialize sensor and add to JSON broadcast
4. **Update web_interface.h**: Add UI elements for new sensor
5. **Update JSON schema**: Document new data fields

## 📚 Additional Resources

### Sensor Datasheets
- **BMI160**: Bosch accelerometer/gyroscope datasheet
- **SI7021**: Silicon Labs humidity/temp sensor datasheet
- **HC-SR04**: Ultrasonic ranging module datasheet
- **HLK-LD2450**: HiLink radar module protocol documentation
- **PIR HC-SR501**: Passive infrared motion sensor datasheet

### ESP32 Resources
- [ESP32 Arduino Core Documentation](https://docs.espressif.com/projects/arduino-esp32/)
- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [WebSockets Library](https://github.com/Links2004/arduinoWebSockets)
- [ArduinoJson Documentation](https://arduinojson.org/)

### Example Use Cases
- **Home Automation**: Motion detection, temperature monitoring
- **Robotics**: Obstacle avoidance, environment sensing
- **Security**: Multi-sensor intrusion detection
- **IoT Projects**: Remote sensor monitoring dashboard
- **Research**: Data logging and analysis

## 🆘 Support

If you encounter issues:

1. Check Serial Monitor output at 115200 baud
2. Enable debug flags in `config.h`
3. Test with fake data mode to isolate hardware issues
4. Verify all wiring connections
5. Check library versions are compatible
6. Review ESP32 board package version

## ✨ Features Coming Soon

Potential enhancements (not yet implemented):
- Data logging to SD card
- MQTT support for home automation
- OTA (Over-The-Air) firmware updates
- Historical data graphs
- Email/SMS alerts
- Mobile app interface
- REST API endpoints
- Multi-language support

## 📌 Version History

**v1.0** - Initial release
- 6 sensor types supported
- WebSocket streaming
- WiFi Access Point mode
- Responsive web dashboard
- Fake data generators
- Modular file structure

---

**Made with ❤️ for ESP32 and Arduino Community**

Happy sensing! 🚀