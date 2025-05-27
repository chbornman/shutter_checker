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

6. **Hardware Pins** (to be configured):
   - ADC pin for photoresistor
   - GPIO pin for LED control
   - I2C/SPI pins for LCD display

Remember: This is ESP-IDF, not Arduino!