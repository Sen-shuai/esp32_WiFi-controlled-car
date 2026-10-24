#include <stdbool.h>
#include "exit.h"

volatile int32_t L_count = 0; // 左轮计数
volatile int32_t R_count = 0; // 右轮计数

/* 一个引脚一份上下文：pin 是本中断对应的引脚，other 是同一编码器的另一相 */
typedef struct {
    gpio_num_t        pin;
    gpio_num_t        other;
    volatile int32_t *count;
    bool              is_a;       // true = 本引脚是 A 相
} enc_ctx_t;

/* 按 exit_init 传入的 exit_id 索引：0=左A 1=左B 2=右A 3=右B */
static enc_ctx_t s_enc_ctx[] = {
    {LA_EXIT_PIN, LB_EXIT_PIN, &L_count, true},   // 左 A
    {LB_EXIT_PIN, LA_EXIT_PIN, &L_count, false},  // 左 B
    {RA_EXIT_PIN, RB_EXIT_PIN, &R_count, true},   // 右 A
    {RB_EXIT_PIN, RA_EXIT_PIN, &R_count, false},  // 右 B
};

/**
 * @brief 编码器中断：正交解码判向并计数
 * @note  正转时 A 相超前 B 相：A 相跳变后两相等电平为正转，B 相跳变后两相不等电平为正转。
 *        两相必须用不同的判据；若两相用同一条规则，每个周期的加和减会互相抵消，净计数恒为 0。
 */
void IRAM_ATTR enc_isr_handler(void *arg)
{
    enc_ctx_t *enc = (enc_ctx_t *)arg;

    int level = gpio_get_level(enc->pin);        // 触发本次中断的那一相
    int other = gpio_get_level(enc->other);      // 另一相

    bool forward = enc->is_a ? (level == other) : (level != other);

    if (forward) {
        (*enc->count)++;
    } else {
        (*enc->count)--;
    }
}

void exit_init(gpio_num_t exit_pin, uint8_t exit_id)
{
    // 安装 GPIO 中断服务，确保只安装一次
    static bool isr_handler_installed = false;
    if (!isr_handler_installed) {
        gpio_install_isr_service(0); // 安装 GPIO 中断服务，参数 0 表示默认中断优先级
        isr_handler_installed = true;
    }

    // 配置 GPIO 引脚为输入模式，并启用上拉电阻
    gpio_config_t exit_config = {
        .mode = GPIO_MODE_INPUT,                 // 设置为输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE,        // 使能上拉电阻（编码器多为 OC 输出）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // 禁用下拉电阻
        .intr_type = GPIO_INTR_ANYEDGE,          // 双边沿：实现四倍频
        .pin_bit_mask = (1ULL << exit_pin)       // 设置引脚掩码
    };
    gpio_config(&exit_config);

    if (exit_id >= sizeof(s_enc_ctx) / sizeof(s_enc_ctx[0])) {
        exit_id = 0;                             // 越界兜底，避免取到不存在的上下文
    }
    gpio_isr_handler_add(exit_pin, enc_isr_handler, &s_enc_ctx[exit_id]);
}
