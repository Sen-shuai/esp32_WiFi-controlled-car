#ifndef _GP_TIM_H
#define _GP_TIM_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_clk_tree.h"
#include "exit.h"


/* PID 任务句柄：由 main 创建任务时填入，定时器报警回调据此唤醒 pid_task。
 * 为 NULL 时回调不发通知，避免任务还没建就被唤醒。 */
extern TaskHandle_t pid_task_handle;

typedef struct {
    gptimer_clock_source_t clk_src;     // 定时器时钟源
    uint32_t tim_clk_hz;                // 定时器时钟频率
    gptimer_count_direction_t direction; // 定时器计数方向
    uint64_t tim_set;                   // 定时器设置值
    uint64_t alarm_value;               // 定时器报警值
    uint64_t counter_value;             // 定时器计数值
    bool auto_reload;                   // 自动重载模式

}tim_t;


void GP_TIM_Init(tim_t *tim);


#endif
