#ifndef DHT11_H
#define DHT11_H

#include "pico/stdlib.h"

// Initialize DHT11 on a specific GPIO pin
void dht11_init(uint pin);

// Read temperature (°C) and humidity (%)
bool dht11_read(uint8_t *temperature, uint8_t *humidity);

#endif
