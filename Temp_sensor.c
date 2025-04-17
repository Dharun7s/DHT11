#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define DHT11_PIN 0
#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// LCD control bits
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_RS_DATA 0x01
#define LCD_RS_CMD 0x00

// Function prototypes
void i2c_write_byte(uint8_t addr, uint8_t data);
void lcd_send_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);
void dht11_start(void);
bool dht11_check_response(void);
bool dht11_read_byte(uint8_t *byte);
bool dht11_read_data(uint8_t *temperature, uint8_t *humidity);

void i2c_write_byte(uint8_t addr, uint8_t data) {
    uint8_t buf[1] = {data};
    i2c_write_blocking(I2C_PORT, addr, buf, 1, false);
}

void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = nibble << 4;
    data |= LCD_BACKLIGHT | rs;
    i2c_write_byte(LCD_ADDR, data | LCD_ENABLE);
    sleep_us(1);
    i2c_write_byte(LCD_ADDR, data & ~LCD_ENABLE);
    sleep_us(100);
}

void lcd_send_cmd(uint8_t cmd) {
    uint8_t high_nibble = cmd >> 4;
    uint8_t low_nibble = cmd & 0x0F;
    lcd_send_nibble(high_nibble, LCD_RS_CMD);
    lcd_send_nibble(low_nibble, LCD_RS_CMD);
}

void lcd_send_data(uint8_t data) {
    uint8_t high_nibble = data >> 4;
    uint8_t low_nibble = data & 0x0F;
    lcd_send_nibble(high_nibble, LCD_RS_DATA);
    lcd_send_nibble(low_nibble, LCD_RS_DATA);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    sleep_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    if (row >= LCD_ROWS) {
        row = LCD_ROWS - 1;
    }
    lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_init(void) {
    sleep_ms(50);
    lcd_send_nibble(0x03, LCD_RS_CMD);
    sleep_ms(5);
    lcd_send_nibble(0x03, LCD_RS_CMD);
    sleep_us(100);
    lcd_send_nibble(0x03, LCD_RS_CMD);
    sleep_us(100);
    lcd_send_nibble(0x02, LCD_RS_CMD);
    sleep_us(100);
    
    lcd_send_cmd(0x28);  // 4-bit mode, 2 lines, 5x8 font
    lcd_send_cmd(0x0C);  // Display on, cursor off, blink off
    lcd_send_cmd(0x06);  // Increment cursor, no display shift
    lcd_clear();
}

void dht11_start(void) {
    gpio_set_dir(DHT11_PIN, GPIO_OUT);
    gpio_put(DHT11_PIN, 0);
    sleep_ms(18);  // Minimum 18ms low for DHT11
    gpio_put(DHT11_PIN, 1);
    sleep_us(20);  // 20-40us high
    gpio_set_dir(DHT11_PIN, GPIO_IN);
    gpio_pull_up(DHT11_PIN);
}

bool dht11_check_response(void) {
    // Wait for sensor to pull low (should happen within 20-40us)
    absolute_time_t timeout = make_timeout_time_ms(1);
    while (gpio_get(DHT11_PIN)) {
        if (time_reached(timeout)) return false;
        sleep_us(1);
    }
    
    // Should stay low for ~80us
    timeout = make_timeout_time_us(100);
    while (!gpio_get(DHT11_PIN)) {
        if (time_reached(timeout)) return false;
        sleep_us(1);
    }
    
    // Should stay high for ~80us
    timeout = make_timeout_time_us(100);
    while (gpio_get(DHT11_PIN)) {
        if (time_reached(timeout)) return false;
        sleep_us(1);
    }
    
    return true;
}

bool dht11_read_byte(uint8_t *byte) {
    *byte = 0;
    for (int i = 0; i < 8; i++) {
        // Wait for start of bit (low to high transition)
        absolute_time_t timeout = make_timeout_time_us(100);
        while (!gpio_get(DHT11_PIN)) {
            if (time_reached(timeout)) return false;
            sleep_us(1);
        }
        
        // Measure the width of the high pulse
        uint32_t start = time_us_32();
        timeout = make_timeout_time_us(100);
        while (gpio_get(DHT11_PIN)) {
            if (time_reached(timeout)) return false;
            sleep_us(1);
        }
        uint32_t duration = time_us_32() - start;
        
        *byte <<= 1;
        if (duration > 50) {  // If high for >50us, it's a 1
            *byte |= 1;
        }
    }
    return true;
}

bool dht11_read_data(uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0};
    
    dht11_start();
    if (!dht11_check_response()) {
        printf("No response from DHT11\n");
        return false;
    }
    
    for (int i = 0; i < 5; i++) {
        if (!dht11_read_byte(&data[i])) {
            printf("Timeout reading byte %d\n", i);
            return false;
        }
    }
    
    // Verify checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        printf("Checksum error: %d != %d\n", data[4], (data[0] + data[1] + data[2] + data[3]));
        return false;
    }
    
    *humidity = data[0];
    *temperature = data[2];
    
    // Validate reasonable ranges
    if (*humidity > 100) {
        printf("Invalid humidity: %d \n", *humidity);
        return false;
    }
    if (*temperature > 50) {  // DHT11 max is 50°C
        printf("Invalid temperature: %d \n", *temperature);
        return false;
    }
    
    return true;
}

int main() {
    stdio_init_all();
    gpio_init(DHT11_PIN);
    
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("DHT11 Sensor");
    
    while (1) {
        uint8_t temperature = 0, humidity = 0;
        if (dht11_read_data(&humidity, &temperature)) {
            printf("Temperature: %d°C, Humidity: %d%%\n", temperature, humidity);
            
            char buffer[LCD_COLS + 1];
            snprintf(buffer, sizeof(buffer), "Temp: %d C", temperature);
            lcd_set_cursor(0, 0);
            lcd_print(buffer);
            
            snprintf(buffer, sizeof(buffer), "Hum:  %d%% ", humidity);
            lcd_set_cursor(0, 1);
            lcd_print(buffer);
        } else {
            printf("Failed to read from DHT11\n");
            lcd_set_cursor(0, 0);
            lcd_print("DHT11 Error   ");
            lcd_set_cursor(0, 1);
            lcd_print("Check Sensor  ");
        }
        sleep_ms(2000);  // Wait at least 1s between readings
    }
    return 0;
}