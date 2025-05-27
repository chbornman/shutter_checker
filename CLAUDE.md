# Claude Development Notes

## Project Type: ESP-IDF Native

**IMPORTANT**: This is a native ESP-IDF project. Do NOT use PlatformIO or Arduino framework.

### Key Development Guidelines

1. **Framework**: Always use ESP-IDF APIs directly
   - No Arduino.h
   - No PlatformIO configuration
   - Pure ESP-IDF CMake build system

2. **Build Commands**:
   ```bash
   idf.py build
   idf.py flash
   idf.py monitor
   idf.py menuconfig
   ```

3. **Common ESP-IDF APIs to Use**:
   - `esp_log.h` for logging (ESP_LOGI, ESP_LOGE, etc.)
   - `driver/gpio.h` for GPIO control
   - `driver/adc.h` for ADC readings
   - `driver/i2c.h` or `driver/spi_master.h` for LCD
   - `freertos/FreeRTOS.h` for RTOS features
   - `esp_timer.h` for high-resolution timing

4. **Project Structure**:
   - Main component in `main/` directory
   - Component CMakeLists.txt uses `idf_component_register()`
   - Root CMakeLists.txt includes ESP-IDF project.cmake

5. **Shutter Timing Implementation Notes**:
   - Use `esp_timer_get_time()` for microsecond precision
   - Configure ADC for photoresistor reading
   - Set up GPIO interrupt for edge detection
   - Consider using hardware timer for accurate measurements

6. **Hardware Pins for ESP32-S3 Feather**:
   - **Photoresistor ADC**: GPIO5 (A1 pin) - ADC1 Channel 4
   - **Light Source LED**: GPIO12 (D12 pin)
   - **Status LED**: GPIO13 (Built-in LED)
   - **Future LCD Display**:
     - I2C: SDA=GPIO3, SCL=GPIO4 (STEMMA QT connector)
     - Or SPI: MOSI=GPIO35, MISO=GPIO37, SCK=GPIO36

7. **Wiring Guide**:
   ```
   Photoresistor Circuit:
   3.3V --- [10kΩ resistor] --- GPIO5 --- [Photoresistor] --- GND
                                 |
                            ADC input
   
   Light Source LED:
   GPIO12 --- [220Ω resistor] --- LED --- GND
   ```

8. **ADC Configuration**:
   - Using ADC1 (ADC2 conflicts with WiFi on ESP32)
   - 11dB attenuation for 0-3.3V range
   - 12-bit resolution (0-4095 values)
   - Light threshold: >2500 = light detected, <1000 = dark

Remember: This is ESP-IDF, not Arduino!