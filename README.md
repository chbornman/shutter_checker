# Shutter Checker

Goal of this project is to make the cheapest, simplest way to check a film camera shutter for speed accuracy. A ESP32 microcontroller will be used with an LED shining from the front of the camera, and a photoresistor behind the shutter curtain. The ESP32 will output the time the shutter was open to an LCD screen.

## ESP-IDF Project Setup

This is a native ESP-IDF project (no PlatformIO or Arduino framework).

### Requirements

- ESP-IDF v4.4 or later
- ESP32 development board
- Python 3.6+
- Hardware components:
  - LED light source
  - Photoresistor/LDR
  - LCD display (I2C/SPI)
  - Resistors for voltage divider

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

### Development Notes

- Uses native ESP-IDF APIs only
- FreeRTOS for task management
- ESP-LOG for debugging
- ADC for photoresistor reading
- GPIO for LED control
- I2C/SPI for LCD communication
