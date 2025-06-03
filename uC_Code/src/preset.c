#include "preset.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "PRESET";

void preset_init(gpio_num_t preset_pin){
    gpio_config_t preset_io_conf = {
        .pin_bit_mask = (1ULL << preset_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&preset_io_conf);
    ESP_LOGI(TAG, "PIN enabled on gpio %d", preset_pin);
}

void poll_preset(void *arg){
    preset_select_args_t *args = (preset_select_args_t *)arg;
    bool current_state;
    bool previous_state = true;

    while (1){
        current_state = gpio_get_level(args->preset_pin);

        if (current_state == false && previous_state == true){
            args->current_preset++;
            if(args->current_preset >= args->num_presets){
                args->current_preset = 0;
            }
            ESP_LOGI("PRESET", "Current preset: %d", args->current_preset);
            apply_preset(args->current_preset);
            vTaskDelay(pdMS_TO_TICKS(args->debounce_ms));
        }
        previous_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(args->debounce_ms));
    }
}

// void apply_preset(uint8_t preset_number) {
//     // Log output to demonstrate functionality
//     // This is where we would implement control logic if implementing this code
//     ESP_LOGI("CONTROL", "Applying preset #%d - [Simulated action]", preset_number);
// }

void apply_preset(uint8_t preset_number) {
    const char* paths[] = {"/preset1.gcode", "/preset2.gcode", "/preset3.gcode"};
    if (preset_number < 3) {
        ESP_LOGI("CONTROL", "Applying preset #%d", preset_number);
        executeFile(paths[preset_number], false);
    }
}