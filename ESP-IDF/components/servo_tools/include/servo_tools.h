#ifndef SERVO_TOOLS_H
#define SERVO_TOOLS_H

#include "../../servo_hw/include/servo_hw.h"
#include "esp_err.h"

typedef uint16_t ServoAngle;

typedef struct {
    uint8_t gpio_num;
    uint32_t pwm_freq;
    ServoAngle min_angle;
    ServoAngle max_angle;

} ServoConfig;

esp_err_t servo_init(ServoConfig *config);

esp_err_t servo_set_angle(ServoConfig *config, ServoAngle angle);

esp_err_t servo_get_angle(const ServoConfig *config, ServoAngle *angle);

#endif