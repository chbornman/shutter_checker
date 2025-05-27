#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "font8x8_basic.h"

static const char *TAG = "SHUTTER_CHECKER";

// Pin definitions for ESP32-S3 Feather
#define LED_PIN         13      // Built-in LED on Feather
#define LIGHT_SOURCE_PIN 12     // External LED to shine through shutter
#define ADC_CHANNEL     ADC_CHANNEL_7  // GPIO8 on ESP32-S3 Feather (A5)
#define ADC_ATTEN       ADC_ATTEN_DB_12  // 0-3.3V range
#define ADC_GPIO_PIN    8       // For verification

// I2C Display pins
#define I2C_SDA_PIN     3
#define I2C_SCL_PIN     4
#define I2C_FREQ_HZ     400000
#define DISPLAY_ADDR    0x3C    // Common address for SSD1306
#define I2C_PORT_NUM    0

// Display dimensions
#define LCD_H_RES       128
#define LCD_V_RES       64
#define LCD_CMD_BITS    8
#define LCD_PARAM_BITS  8

// Threshold values - adjusted for photoresistor (inverted logic)
#define LIGHT_THRESHOLD_HIGH 2400   // ADC value when dark (photoresistor high resistance)
#define LIGHT_THRESHOLD_LOW  1800   // ADC value when light detected (photoresistor low resistance)
#define DEBOUNCE_US     50         // 50µs debounce time (allows up to 1/20000s detection)

// Shutter timing states
typedef enum {
    STATE_IDLE,
    STATE_WAITING_OPEN,
    STATE_TIMING,
    STATE_COMPLETE
} shutter_state_t;

// Global variables
static adc_oneshot_unit_handle_t adc_handle;
static shutter_state_t current_state = STATE_IDLE;
static int64_t shutter_open_time = 0;
static int64_t shutter_close_time = 0;
static QueueHandle_t measurement_queue;
static esp_lcd_panel_handle_t panel_handle = NULL;

// Display buffer for drawing
static uint8_t *display_buffer = NULL;

// Forward declarations
void display_set_pixel(int x, int y, bool color);

typedef struct {
    int64_t duration_us;
    float duration_ms;
    char shutter_speed[32];
} measurement_t;

// Convert microseconds to shutter speed notation
void calculate_shutter_speed(int64_t duration_us, char* output) {
    float duration_s = duration_us / 1000000.0f;
    
    if (duration_s < 0.001) {
        sprintf(output, "1/%d s", (int)(1.0f / duration_s));
    } else if (duration_s < 1.0) {
        int denominator = (int)(1.0f / duration_s + 0.5f);
        sprintf(output, "1/%d s", denominator);
    } else {
        sprintf(output, "%.1f s", duration_s);
    }
}

// Initialize ADC
void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));
    ESP_LOGI(TAG, "ADC initialized on channel %d", ADC_CHANNEL);
}

// Initialize GPIOs
void init_gpio(void) {
    // Configure built-in LED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << LIGHT_SOURCE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Turn on light source LED
    gpio_set_level(LIGHT_SOURCE_PIN, 1);
    ESP_LOGI(TAG, "GPIO initialized, light source ON");
}

// Initialize I2C and SSD1306 display
void init_display(void) {
    ESP_LOGI(TAG, "Initializing GME12864/SSD1306 display...");
    
    // Initialize I2C bus
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, i2c_conf.mode, 0, 0, 0));

    // Create I2C panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = DISPLAY_ADDR,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .flags.dc_low_on_data = 0,
        .flags.disable_control_phase = 0,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_PORT_NUM, &io_config, &io_handle));

    // Create SSD1306 panel
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    
    // Reset and initialize display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    // Rotate display 180 degrees
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    
    // Allocate framebuffer
    display_buffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES / 8, MALLOC_CAP_DMA);
    if (display_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return;
    }
    memset(display_buffer, 0x00, LCD_H_RES * LCD_V_RES / 8);
    
    ESP_LOGI(TAG, "Display initialized successfully");
}

// Draw a single character at specified position
void display_draw_char(char c, int x, int y, bool color) {
    if (c < 0 || c > 127) return;
    
    for (int i = 0; i < 8; i++) {
        uint8_t line = font8x8_basic[(int)c][i];
        for (int j = 0; j < 8; j++) {
            if (line & 0x1) {
                display_set_pixel(x + j, y + i, color);
            }
            line >>= 1;
        }
    }
}

// Draw text on display at specified position
void display_draw_text(const char *text, int x, int y, bool large) {
    int cursor_x = x;
    int cursor_y = y;
    int char_width = large ? 16 : 8;
    int char_height = large ? 16 : 8;
    
    while (*text) {
        if (*text == '\n') {
            cursor_x = x;
            cursor_y += char_height;
        } else {
            if (large) {
                // Draw character scaled 2x
                for (int i = 0; i < 8; i++) {
                    uint8_t line = font8x8_basic[(int)*text][i];
                    for (int j = 0; j < 8; j++) {
                        if (line & (1 << j)) {
                            // Draw 2x2 block for each pixel
                            display_set_pixel(cursor_x + j*2, cursor_y + i*2, true);
                            display_set_pixel(cursor_x + j*2 + 1, cursor_y + i*2, true);
                            display_set_pixel(cursor_x + j*2, cursor_y + i*2 + 1, true);
                            display_set_pixel(cursor_x + j*2 + 1, cursor_y + i*2 + 1, true);
                        }
                    }
                }
            } else {
                display_draw_char(*text, cursor_x, cursor_y, true);
            }
            cursor_x += char_width;
            
            // Wrap to next line if needed
            if (cursor_x + char_width > LCD_H_RES) {
                cursor_x = x;
                cursor_y += char_height;
            }
        }
        text++;
    }
}

// Update display with current content
void display_update(void) {
    if (panel_handle != NULL && display_buffer != NULL) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, display_buffer);
    }
}

// Clear display buffer
void display_clear(void) {
    if (display_buffer != NULL) {
        memset(display_buffer, 0x00, LCD_H_RES * LCD_V_RES / 8);
    }
}

// Task to monitor light sensor and measure shutter timing
void shutter_monitor_task(void *pvParameters) {
    int adc_reading;
    int64_t last_transition = 0;
    bool light_detected = false;
    measurement_t measurement;
    int64_t last_log_time = 0;
    
    // Log ADC configuration
    ESP_LOGI(TAG, "ADC Configuration: Channel %d, GPIO %d, Attenuation %d", 
             ADC_CHANNEL, ADC_GPIO_PIN, ADC_ATTEN);
    
    // Counter for periodic yielding
    int loop_counter = 0;
    
    while (1) {
        // Single fast ADC read - no averaging for maximum speed
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_reading));
        
        int64_t current_time = esp_timer_get_time();
        
        // Only yield every 1000 iterations to prevent watchdog but maintain speed
        if (++loop_counter >= 1000) {
            vTaskDelay(1);  // Yield to prevent watchdog
            loop_counter = 0;
        }
        
        // Debounce check
        if ((current_time - last_transition) < DEBOUNCE_US) {
            continue;
        }
        
        // Log ADC value every 2 seconds regardless of state
        if ((current_time - last_log_time) > 2000000) { // Every 2 seconds
            float voltage = (adc_reading / 4095.0) * 3.3;
            const char* state_name = "UNKNOWN";
            
            switch (current_state) {
                case STATE_IDLE: state_name = "IDLE"; break;
                case STATE_WAITING_OPEN: state_name = "WAITING"; break;
                case STATE_TIMING: 
                    state_name = "SHUTTER OPEN";
                    int64_t elapsed_us = current_time - shutter_open_time;
                    ESP_LOGI(TAG, "%s - ADC: %d (%.2fV), Time open: %.1f ms", 
                            state_name, adc_reading, voltage, elapsed_us / 1000.0);
                    last_log_time = current_time;
                    break;
                case STATE_COMPLETE: state_name = "COMPLETE"; break;
            }
            
            if (current_state != STATE_TIMING) {
                ESP_LOGI(TAG, "%s - ADC: %d (%.2fV)", state_name, adc_reading, voltage);
                last_log_time = current_time;
            }
        }
        
        switch (current_state) {
            case STATE_IDLE:
                ESP_LOGI(TAG, "Ready. ADC: %d (waiting for shutter)", adc_reading);
                current_state = STATE_WAITING_OPEN;
                // No countdown delay needed without display
                break;
                
            case STATE_WAITING_OPEN:
                // Note: Logic inverted for photoresistor - lower ADC = more light (shutter open)
                if (adc_reading < LIGHT_THRESHOLD_LOW && !light_detected) {
                    shutter_open_time = current_time;
                    light_detected = true;
                    current_state = STATE_TIMING;
                    last_transition = current_time;
                    gpio_set_level(LED_PIN, 1);  // Turn on indicator LED
                    ESP_LOGI(TAG, "Shutter OPENED at %lld us", shutter_open_time);
                }
                break;
                
            case STATE_TIMING:
                // Note: Logic inverted for photoresistor - higher ADC = less light (shutter closing)
                if (adc_reading > LIGHT_THRESHOLD_HIGH && light_detected) {
                    shutter_close_time = current_time;
                    light_detected = false;
                    current_state = STATE_COMPLETE;
                    last_transition = current_time;
                    gpio_set_level(LED_PIN, 0);  // Turn off indicator LED
                    ESP_LOGI(TAG, "Shutter CLOSED at %lld us", shutter_close_time);
                    
                    // Calculate and send measurement
                    measurement.duration_us = shutter_close_time - shutter_open_time;
                    measurement.duration_ms = measurement.duration_us / 1000.0f;
                    calculate_shutter_speed(measurement.duration_us, measurement.shutter_speed);
                    
                    // Print the shutter speed immediately
                    ESP_LOGI(TAG, "=================================");
                    ESP_LOGI(TAG, "SHUTTER SPEED: %s", measurement.shutter_speed);
                    ESP_LOGI(TAG, "Duration: %.3f ms", measurement.duration_ms);
                    ESP_LOGI(TAG, "=================================");
                    
                    xQueueSend(measurement_queue, &measurement, 0);
                }
                break;
                
            case STATE_COMPLETE:
                // Reset after a short delay
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_IDLE;
                break;
        }
    }
}

// Simple function to draw a pixel in the buffer
void display_set_pixel(int x, int y, bool color) {
    if (x >= 0 && x < LCD_H_RES && y >= 0 && y < LCD_V_RES) {
        int byte_index = x + (y / 8) * LCD_H_RES;
        int bit_index = y % 8;
        if (color) {
            display_buffer[byte_index] |= (1 << bit_index);
        } else {
            display_buffer[byte_index] &= ~(1 << bit_index);
        }
    }
}

// Draw a simple box
void display_draw_box(int x, int y, int w, int h) {
    for (int i = x; i < x + w; i++) {
        display_set_pixel(i, y, true);
        display_set_pixel(i, y + h - 1, true);
    }
    for (int i = y; i < y + h; i++) {
        display_set_pixel(x, i, true);
        display_set_pixel(x + w - 1, i, true);
    }
}

// Draw a filled rectangle
void display_fill_rect(int x, int y, int w, int h, bool color) {
    for (int i = x; i < x + w && i < LCD_H_RES; i++) {
        for (int j = y; j < y + h && j < LCD_V_RES; j++) {
            display_set_pixel(i, j, color);
        }
    }
}

// Task to display results
void display_task(void *pvParameters) {
    measurement_t measurement;
    char line1[64];
    char last_speed[32] = "None";
    
    // Wait a bit for display to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Show welcome screen with countdown
    for (int i = 3; i > 0; i--) {
        display_clear();
        display_draw_box(0, 0, LCD_H_RES-1, LCD_V_RES-1);
        display_draw_text("SHUTTER", 20, 8, true);
        display_draw_text("CHECKER", 20, 24, true);
        snprintf(line1, sizeof(line1), "Start in %d", i);
        display_draw_text(line1, 28, 48, false);
        display_update();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Show ready screen
    display_clear();
    display_draw_box(0, 0, LCD_H_RES-1, LCD_V_RES-1);
    display_draw_text("Last:", 8, 8, false);
    display_draw_text(last_speed, 16, 20, true);
    display_draw_text("Ready...", 32, 48, false);
    display_update();
    
    while (1) {
        // Check for new measurements with timeout
        if (xQueueReceive(measurement_queue, &measurement, pdMS_TO_TICKS(100))) {
            // Store the measurement
            strncpy(last_speed, measurement.shutter_speed, sizeof(last_speed) - 1);
            last_speed[sizeof(last_speed) - 1] = '\0';
            
            // Show measurement for 3 seconds
            display_clear();
            display_draw_box(0, 0, LCD_H_RES-1, LCD_V_RES-1);
            display_draw_text("MEASURED!", 24, 8, false);
            
            // Center the shutter speed
            int text_len = strlen(measurement.shutter_speed);
            int x_pos = (LCD_H_RES - (text_len * 16)) / 2;
            if (x_pos < 0) x_pos = 0;
            display_draw_text(measurement.shutter_speed, x_pos, 24, true);
            
            snprintf(line1, sizeof(line1), "%.1f ms", measurement.duration_ms);
            text_len = strlen(line1);
            x_pos = (LCD_H_RES - (text_len * 8)) / 2;
            display_draw_text(line1, x_pos, 48, false);
            display_update();
            
            vTaskDelay(pdMS_TO_TICKS(3000));
            
            // Return to ready screen
            display_clear();
            display_draw_box(0, 0, LCD_H_RES-1, LCD_V_RES-1);
            display_draw_text("Last:", 8, 8, false);
            display_draw_text(last_speed, 16, 20, true);
            display_draw_text("Ready...", 32, 48, false);
            display_update();
        }
        
        // Update display periodically even without new measurements
        if (current_state == STATE_TIMING) {
            int64_t elapsed_us = esp_timer_get_time() - shutter_open_time;
            display_clear();
            display_draw_box(0, 0, LCD_H_RES-1, LCD_V_RES-1);
            display_draw_text("TIMING...", 28, 16, false);
            snprintf(line1, sizeof(line1), "%.1f ms", elapsed_us / 1000.0);
            display_draw_text(line1, 32, 32, false);
            display_update();
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Shutter Checker for ESP32-S3 Starting");
    
    // Initialize hardware
    init_gpio();
    init_adc();
    init_display();
    
    // Create queue for measurements
    measurement_queue = xQueueCreate(10, sizeof(measurement_t));
    
    // Create monitoring task with high priority for fast response
    xTaskCreatePinnedToCore(shutter_monitor_task, "shutter_monitor", 8192, NULL, 
                            configMAX_PRIORITIES - 1, NULL, 1);  // Pin to core 1, max priority
    
    // Create display task
    xTaskCreate(display_task, "display", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "System initialized. Place camera in front of light source.");
}
