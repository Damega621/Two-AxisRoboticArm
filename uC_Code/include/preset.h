#ifndef PRESET_H
#define PRESET_H

#include "driver/gpio.h"

//struct defines preset pin configuration
typedef struct {
    gpio_num_t preset_pin;
    uint8_t current_preset;
    uint8_t num_presets; //smallest possible memory footprint
    uint32_t debounce_ms; //freeRTOS and ESP APIs expect 32 bit values
} preset_select_args_t;

//initializes gpio 19 to poll for preset inputs.
void preset_init(gpio_num_t preset_pin);

//actuall polling function that handles the polling logic and cycles each press
void poll_preset(void *arg);

//modular function created to implement control logic
//it just prints messages for verification purposes
void apply_preset(uint8_t preset_number);

#endif