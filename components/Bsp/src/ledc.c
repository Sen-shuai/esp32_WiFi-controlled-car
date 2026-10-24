#include "ledc.h"

//计算占空比对应的值
uint32_t ledc_duty_pow(ledc_config_t *config,uint32_t duty)
{
    uint32_t result = 1;
    uint8_t n = (uint8_t)config->duty_resolution;   // 获取占空比分辨率的位数
    if(duty > 100) {
        duty = 100; // 限制占空比在 0-100% 范围内
    }
    while(n > 0) {
        result *= 2;
        n--;
    }
    return (result * duty) / 100;
}


void ledc_init(ledc_config_t *config)
{
    // 配置 LEDC 定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = config->speed_mode,
        .timer_num        = config->timer_num,
        .duty_resolution  = config->duty_resolution,
        .freq_hz          = config->freq_hz,  // 设置 PWM 频率为 5 kHz
        .clk_cfg          = config->clk_cfg     // 设置 LEDC 时钟源
    };
    ledc_timer_config(&ledc_timer);

    // 配置 LEDC 通道
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = config->speed_mode,
        .channel        = config->channel,
        .timer_sel      = config->timer_num,    // 选择使用的定时器
        .intr_type      = LEDC_INTR_DISABLE,    // 禁用中断
        .gpio_num       = config->gpio_num,  // 输出 GPIO 引脚
        .duty           = config->duty_value, // 设置初始占空比
        .hpoint         = 0                 // 初始偏移为 0
    };
    ledc_channel_config(&ledc_channel);
}

void ledc_pwm_set_duty(ledc_config_t *config,uint32_t duty)
{
    config->duty = duty;
    config->duty_value = ledc_duty_pow(config,config->duty);
    // 设置占空比
    ledc_set_duty(config->speed_mode, config->channel, config->duty_value);
    // 更新占空比
    ledc_update_duty(config->speed_mode, config->channel);
}


