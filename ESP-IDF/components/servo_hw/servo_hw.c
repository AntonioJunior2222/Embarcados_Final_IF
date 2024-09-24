#include "servo_hw.h"
#include "driver/ledc.h"

#define MAX_SERVOS 10

typedef struct
{
    uint8_t gpio_num;
    ledc_channel_t channel;
} ServoChannelMap;

static ServoChannelMap servo_map[MAX_SERVOS] = {0};
static bool timer_initialized = false;

static ledc_channel_t find_available_channel()
{
    for (ledc_channel_t channel = LEDC_CHANNEL_0; channel < LEDC_CHANNEL_MAX; ++channel)
    {
        bool used = false;
        for (int i = 0; i < MAX_SERVOS; ++i)
        {
            if (servo_map[i].channel == channel)
            {
                used = true;
                break;
            }
        }
        if (!used)
        {
            return channel;
        }
    }
    return LEDC_CHANNEL_MAX;
}

esp_err_t hw_servo_init(uint8_t gpio_num)
{
    if (!timer_initialized)
    {
        ledc_timer_config_t timer_conf = {
            .duty_resolution = LEDC_TIMER_16_BIT,
            .freq_hz = 50,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0};
        esp_err_t ret = ledc_timer_config(&timer_conf);
        if (ret != ESP_OK)
        {
            return ESP_ERR_NOT_FOUND;
        }
        timer_initialized = true;
    }

    ledc_channel_t channel = find_available_channel();
    if (channel == LEDC_CHANNEL_MAX)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ledc_channel_config_t channel_conf = {
        .channel = channel,
        .duty = 0,
        .gpio_num = gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0};
    esp_err_t ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }

    for (int i = 0; i < MAX_SERVOS; ++i)
    {
        if (servo_map[i].gpio_num == 0)
        {
            servo_map[i].gpio_num = gpio_num;
            servo_map[i].channel = channel;
            break;
        }
    }

    ret = hw_servo_set_pulse_width(gpio_num, 500);
    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    ret = hw_servo_set_pulse_width(gpio_num, 2500);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ESP_OK;
}

esp_err_t hw_servo_set_pulse_width(uint8_t gpio_num, uint32_t pulse_width_us)
{
    ledc_channel_t channel = LEDC_CHANNEL_MAX;
    for (int i = 0; i < MAX_SERVOS; ++i)
    {
        if (servo_map[i].gpio_num == gpio_num)
        {
            channel = servo_map[i].channel;
            break;
        }
    }

    if (channel == LEDC_CHANNEL_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = (pulse_width_us * 65536) / 20000;
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    return ret == ESP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t hw_servo_deinit(uint8_t gpio_num)
{
    ledc_channel_t channel = LEDC_CHANNEL_MAX;
    for (int i = 0; i < MAX_SERVOS; ++i)
    {
        if (servo_map[i].gpio_num == gpio_num)
        {
            channel = servo_map[i].channel;
            servo_map[i].gpio_num = 0;
            servo_map[i].channel = LEDC_CHANNEL_MAX;
            break;
        }
    }

    if (channel == LEDC_CHANNEL_MAX)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
    return ESP_OK;
}