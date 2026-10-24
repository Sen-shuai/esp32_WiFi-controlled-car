#ifndef _EXIT_H
#define _EXIT_H

#include "driver/gpio.h"
#include "esp_system.h"

extern volatile int32_t L_count; // 左轮计数（ISR 中修改，必须 volatile）
extern volatile int32_t R_count; // 右轮计数（ISR 中修改，必须 volatile）



#define LA_EXIT_PIN GPIO_NUM_10 // 左编码器 A 相
#define LB_EXIT_PIN GPIO_NUM_11 // 左编码器 B 相
#define RA_EXIT_PIN GPIO_NUM_12 // 右编码器 A 相
#define RB_EXIT_PIN GPIO_NUM_13 // 右编码器 B 相

void exit_init(gpio_num_t exit_pin,uint8_t exit_id);

#endif

