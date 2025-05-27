# Shutter Checker

Goal of this project is to make the cheapest, simplest way to check a film camera shutter for speed accuracy. A ESP32 microcontroller will be used with an LED shining from the front of the camera, and a photoresistor behind the shutter curtain. The ESP32 will output the time the shutter was open to an LCD screen.

## ESP-IDF Project Setup

This is a native ESP-IDF project (no PlatformIO or Arduino framework).

### Requirements

- ESP-IDF v4.4 or later
- ESP32 development board
- Python 3.6+
- Hardware components:
  - ESP32-S3 Feather board (Adafruit)
  - Bright white LED for light source
  - Photoresistor/LDR (Light Dependent Resistor)
  - 10kΩ resistor (for voltage divider)
  - 220Ω resistor (for LED current limiting)
  - GME12864-11 OLED display (128x64, I2C, SSD1306 controller)
  - Breadboard and jumper wires

### Environment Setup

1. Install ESP-IDF:
   ```bash
   git clone -b v4.4 --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   ./install.sh
   ```

2. Set up the environment:
   ```bash
   . $HOME/esp-idf/export.sh
   ```

### Building and Flashing

1. Configure the project:
   ```bash
   idf.py set-target esp32
   idf.py menuconfig
   ```

2. Build:
   ```bash
   idf.py build
   ```

3. Flash and monitor:
   ```bash
   idf.py -p PORT flash monitor
   ```

### Project Structure

```
shutter_checker/
├── CMakeLists.txt          # Main CMake configuration
├── main/
│   ├── CMakeLists.txt      # Component CMake configuration
│   └── main.c              # Application entry point
├── README.md               # This file
└── CLAUDE.md              # Development notes
```

### Hardware Connections (ESP32-S3 Feather)

```
Photoresistor Voltage Divider:
- 3.3V → 10kΩ resistor → GPIO5 (A1) → Photoresistor → GND

Light Source LED:
- GPIO12 (D12) → 220Ω resistor → LED anode → LED cathode → GND

Status Indicator:
- GPIO13 (Built-in LED on Feather)

OLED Display (GME12864-11):
- SDA → GPIO3 (I2C data)
- SCL → GPIO4 (I2C clock)
- VCC → 3.3V
- GND → GND
```

### How It Works

1. **Setup Phase**: The light source LED turns on, illuminating the front of the camera
2. **Detection**: The photoresistor behind the shutter detects when light passes through
3. **Timing**: High-precision timer measures the duration between shutter open and close
4. **Display**: Results shown on OLED display and serial output in standard shutter speed notation (e.g., 1/125s)

### Calibration

The ADC thresholds may need adjustment based on your setup:
- `LIGHT_THRESHOLD_HIGH`: ADC value when light is detected (default: 2500)
- `LIGHT_THRESHOLD_LOW`: ADC value when dark (default: 1000)
- Monitor the serial output to see actual ADC readings and adjust accordingly

### Development Notes

- Uses native ESP-IDF APIs only
- FreeRTOS tasks for concurrent operation
- ESP-LOG for debugging output
- ADC1 for photoresistor reading (12-bit, 0-4095 range)
- GPIO for LED control
- High-resolution timer (microsecond precision)
- Debouncing to prevent false triggers
- I2C driver for SSD1306 OLED display
- ESP LCD panel API for display control
