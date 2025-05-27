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

static const char *TAG = "SHUTTER_CHECKER";

// Pin definitions for ESP32-S3 Feather
#define LED_PIN         13      // Built-in LED on Feather
#define LIGHT_SOURCE_PIN 12     // External LED to shine through shutter
#define ADC_CHANNEL     ADC_CHANNEL_4  // GPIO5 on ESP32-S3 Feather (A1)
#define ADC_ATTEN       ADC_ATTEN_DB_11

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

// Threshold values
#define LIGHT_THRESHOLD_HIGH 2500  // ADC value when light detected
#define LIGHT_THRESHOLD_LOW  1000  // ADC value when dark
#define DEBOUNCE_US     1000       // 1ms debounce time

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
    ESP_LOGI(TAG, "Initializing SSD1306 display...");
    
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
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_PORT_NUM, &io_config, &io_handle));

    // Create SSD1306 panel
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    
    // Reset and initialize display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    
    // Allocate framebuffer
    display_buffer = heap_caps_malloc(LCD_H_RES * LCD_V_RES / 8, MALLOC_CAP_DMA);
    memset(display_buffer, 0x00, LCD_H_RES * LCD_V_RES / 8);
    
    ESP_LOGI(TAG, "Display initialized");
}

// Draw text on display at specified position
void display_draw_text(const char *text, int x, int y, bool large) {
    // For now, just log the text. In a real implementation, you'd use a graphics library
    ESP_LOGI(TAG, "Display text at (%d,%d): %s", x, y, text);
}

// Update display with current content
void display_update(void) {
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, display_buffer);
}

// Clear display buffer
void display_clear(void) {
    memset(display_buffer, 0x00, LCD_H_RES * LCD_V_RES / 8);
}

// Task to monitor light sensor and measure shutter timing
void shutter_monitor_task(void *pvParameters) {
    int adc_reading;
    int64_t last_transition = 0;
    bool light_detected = false;
    measurement_t measurement;
    
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_reading));
        
        int64_t current_time = esp_timer_get_time();
        
        // Debounce check
        if ((current_time - last_transition) < DEBOUNCE_US) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        
        switch (current_state) {
            case STATE_IDLE:
                ESP_LOGI(TAG, "Ready. ADC: %d (waiting for shutter)", adc_reading);
                current_state = STATE_WAITING_OPEN;
                break;
                
            case STATE_WAITING_OPEN:
                if (adc_reading > LIGHT_THRESHOLD_HIGH && !light_detected) {
                    shutter_open_time = current_time;
                    light_detected = true;
                    current_state = STATE_TIMING;
                    last_transition = current_time;
                    gpio_set_level(LED_PIN, 1);  // Turn on indicator LED
                    ESP_LOGI(TAG, "Shutter OPENED at %lld us", shutter_open_time);
                }
                break;
                
            case STATE_TIMING:
                if (adc_reading < LIGHT_THRESHOLD_LOW && light_detected) {
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
                    
                    xQueueSend(measurement_queue, &measurement, 0);
                }
                break;
                
            case STATE_COMPLETE:
                // Reset after a short delay
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_IDLE;
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms sampling rate
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

// Task to display results
void display_task(void *pvParameters) {
    measurement_t measurement;
    char line1[32], line2[32], line3[32];
    
    // Show initial screen
    display_clear();
    display_draw_box(0, 0, LCD_H_RES, LCD_V_RES);
    display_draw_text("SHUTTER CHECKER", 20, 20, true);
    display_draw_text("Ready...", 40, 40, false);
    display_update();
    
    while (1) {
        if (xQueueReceive(measurement_queue, &measurement, portMAX_DELAY)) {
            ESP_LOGI(TAG, "=================================");
            ESP_LOGI(TAG, "MEASUREMENT COMPLETE!");
            ESP_LOGI(TAG, "Duration: %lld microseconds", measurement.duration_us);
            ESP_LOGI(TAG, "Duration: %.3f milliseconds", measurement.duration_ms);
            ESP_LOGI(TAG, "Shutter Speed: %s", measurement.shutter_speed);
            ESP_LOGI(TAG, "=================================");
            
            // Update LCD display
            display_clear();
            display_draw_box(0, 0, LCD_H_RES, LCD_V_RES);
            
            // Format display lines
            snprintf(line1, sizeof(line1), "Speed: %s", measurement.shutter_speed);
            snprintf(line2, sizeof(line2), "%.1f ms", measurement.duration_ms);
            snprintf(line3, sizeof(line3), "%lld us", measurement.duration_us);
            
            // Draw on display (positions are approximate - would need proper font rendering)
            display_draw_text(line1, 10, 10, true);
            display_draw_text(line2, 10, 30, false);
            display_draw_text(line3, 10, 45, false);
            
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
    
    // Create monitoring task
    xTaskCreate(shutter_monitor_task, "shutter_monitor", 4096, NULL, 5, NULL);
    
    // Create display task
    xTaskCreate(display_task, "display", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "System initialized. Place camera in front of light source.");
}