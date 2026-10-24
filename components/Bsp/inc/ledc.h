#ifndef _LEDC_H_
#define _LEDC_H_

#include "driver/gpio.h"
#include "driver/ledc.h"


typedef struct {

    ledc_clk_cfg_t clk_cfg;        // LEDC 时钟选择
    uint8_t duty;             // PWM 占空比 (0-100)
    uint32_t duty_value;        // PWM 占空比对应的值 (0-8191)
    ledc_timer_bit_t duty_resolution; // PWM 占空比分辨率
    ledc_mode_t speed_mode;    // LEDC 模式 (高速度或低速度)
    uint32_t freq_hz;          // PWM 频率 (Hz)
    ledc_channel_t channel;     // LEDC 通道
    ledc_timer_t timer_num;     // LEDC 定时器编号
    gpio_num_t gpio_num;       // 控制 LED 的 GPIO 引脚

} ledc_config_t;


void ledc_init(ledc_config_t *config);
uint32_t ledc_duty_pow(ledc_config_t *config,uint32_t duty);
void ledc_pwm_set_duty(ledc_config_t *config,uint32_t duty);

#endif

