#include "servo_tools.h"

esp_err_t servo_init(ServoConfig *config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = hw_servo_init(config->gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t servo_set_angle(ServoConfig *config, ServoAngle angle) {
    if (!config || angle < config->min_angle || angle > config->max_angle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t pulse_width = (angle * (2500 - 500)) / (config->max_angle - config->min_angle) + 500;

    return hw_servo_set_pulse_width(config->gpio_num, pulse_width);
}

esp_err_t servo_get_angle(const ServoConfig *config, ServoAngle *angle) {
    if (!config || !angle) {
        return ESP_ERR_INVALID_ARG;
    }
    *angle = 0;
    return ESP_OK;
}