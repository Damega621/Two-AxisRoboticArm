
#include "power-down.h"


static const char *TAG = "POWER-DOWN";

extern void power_down_init(gpio_num_t power_down_pin){
    //gpio_config_t sets up the instance power_io_conf correctly for the gpio_config function
    gpio_config_t power_io_conf = {
        .pin_bit_mask = (1ULL << power_down_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    //now have to pass in our struct to config corretly
    gpio_config(&power_io_conf);
    ESP_LOGI(TAG, "POWER-DOWN Initialized on pin %d", power_down_pin);
}

extern void poll_power_down(void *arg){

    power_down_task_args_t *args = (power_down_task_args_t *)arg;

    while(1){
        if(gpio_get_level(args->shutdown_pin) == 0){
            ESP_LOGI(TAG, "SHUTDOWN REQUESTED. PERFORMING CLEANUP...");

            //TURN OFF MOTORS
            gpio_set_level(args->motor1_pin, 0);
            gpio_set_level(args->motor2_pin, 0);

            //wait to finish logs
            vTaskDelay(pdMS_TO_TICKS(args->delay_ms));
            ESP_LOGI(TAG, "Shutdown complete. Safe to power off.");
            
            //this puts the system in an infinie loop. doing nothing and makes it safe to remove external power
            while(1){
                vTaskDelay(pdMS_TO_TICKS(args->delay_ms));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}