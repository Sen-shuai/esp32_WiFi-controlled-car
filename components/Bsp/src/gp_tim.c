#include "gp_tim.h"


TaskHandle_t pid_task_handle = NULL; // PID 任务句柄，由 main 创建任务时填入

/* 定时器报警回调（ISR 上下文）：只唤醒 pid_task，不做任何计算。
 * 这样 PID 周期完全由硬件定时器决定，任务阻塞多久都不影响下一拍。 */
bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_woken = pdFALSE;

    if (pid_task_handle != NULL) {
        vTaskNotifyGiveFromISR(pid_task_handle, &high_task_woken);
        portYIELD_FROM_ISR(high_task_woken);
    }

    return true;
}

void GP_TIM_Init(tim_t *tim)
{
    gptimer_handle_t gptimer = NULL;


    gptimer_config_t config = {
        .direction = tim->direction,        // 设置计数方向为向上计数
        .resolution_hz = tim->tim_clk_hz,          // 设置计数器分辨率为时钟频率
        .intr_priority = 1,                    // 设置中断优先级为1
        .clk_src = tim->clk_src,            // 设置时钟源
    };

    gptimer_new_timer(&config, &gptimer); // 创建新的GPTimer定时器


    gptimer_alarm_config_t alarm_config = {
        .alarm_count = tim->alarm_value,      // 设置报警值
        .reload_count = tim->tim_set,         // 设置重载值
        .flags.auto_reload_on_alarm = tim->auto_reload, // 设置自动重载模式
    };

    gptimer_event_callbacks_t cbs = {
        .on_alarm = gptimer_callback, // 设置报警回调函数
    };

    gptimer_set_alarm_action(gptimer, &alarm_config); // 设置报警动作为增加计数器值

    gptimer_register_event_callbacks(gptimer, &cbs, NULL); // 注册事件回调函数

    gptimer_enable(gptimer); // 启用定时器
    gptimer_start(gptimer);  // 启动计数，否则不会触发报警

}


