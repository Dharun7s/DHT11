#include "dht11.h"
#include "pico/time.h"

static uint dht_pin;

void dht11_init(uint pin) {
    dht_pin = pin;
    gpio_init(dht_pin);
}

static void dht11_start() {
    gpio_set_dir(dht_pin, GPIO_OUT);
    gpio_put(dht_pin, 0);
    sleep_ms(18);
    gpio_put(dht_pin, 1);
    sleep_us(20);
    gpio_set_dir(dht_pin, GPIO_IN);
    gpio_pull_up(dht_pin);
}

static bool dht11_check_response() {
    absolute_time_t timeout = make_timeout_time_ms(1);
    while (gpio_get(dht_pin)) {
        if (time_reached(timeout)) return false;
        sleep_us(1);
    }
    
    timeout = make_timeout_time_us(100);
    while (!gpio_get(dht_pin)) {
        if (time_reached(timeout)) return false;
        sleep_us(1);
    }
    
    timeout = make_timeout_time_us(100);
    while (gpio_get(dht_pin)) {
        if (time_reached(timeout)) return false;
        sleep_us(1);
    }
    
    return true;
}

static bool dht11_read_byte(uint8_t *byte) {
    *byte = 0;
    for (int i = 0; i < 8; i++) {
        absolute_time_t timeout = make_timeout_time_us(100);
        while (!gpio_get(dht_pin)) {
            if (time_reached(timeout)) return false;
            sleep_us(1);
        }
        
        uint32_t start = time_us_32();
        timeout = make_timeout_time_us(100);
        while (gpio_get(dht_pin)) {
            if (time_reached(timeout)) return false;
            sleep_us(1);
        }
        
        *byte <<= 1;
        if (time_us_32() - start > 50) *byte |= 1;
    }
    return true;
}

bool dht11_read(uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0};
    
    dht11_start();
    if (!dht11_check_response()) return false;
    
    for (int i = 0; i < 5; i++) {
        if (!dht11_read_byte(&data[i])) return false;
    }
    
    // Verify checksum
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return false;
    
    *humidity = data[0];
    *temperature = data[2];
    
    // Validate ranges
    return (*humidity <= 100) && (*temperature <= 50);
}
