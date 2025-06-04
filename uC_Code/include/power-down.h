#ifndef POWER_DOWN_H
#define POWER_DOWN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"


typedef struct {
    gpio_num_t shutdown_pin;
    gpio_num_t motor1_pin;
    gpio_num_t motor2_pin;
    uint32_t delay_ms;
} power_down_task_args_t;


#ifdef __cplusplus
extern "C" {
#endif

//Initializes the power down gpio pin
void power_down_init(gpio_num_t power_down_pin);

//funtion to poll the shutdon gpio pin
void poll_power_down(void *arg);

#ifdef __cplusplus
}
#endif

#endif 