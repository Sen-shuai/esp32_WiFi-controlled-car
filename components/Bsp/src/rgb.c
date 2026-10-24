#include "rgb.h"


#define RGB_PIN 48          // RGB 灯珠的数据引脚（GPIO48）
#define RGB_LED_NUM 1       // 灯珠数量（板载 1 个）

// 灯带句柄，初始化后用于后续所有操作
led_strip_handle_t rgb_handle = NULL;


/**
 * @brief 初始化 RGB 灯珠（基于 RMT 外设驱动 WS2812 类灯珠）
 */
void rgb_init(void)
{
    // 灯带基本配置：引脚与灯珠数量
    led_strip_config_t rgb_config = {
        .strip_gpio_num = RGB_PIN,
        .max_leds = RGB_LED_NUM,
    };

    // RMT 发送通道配置
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 40000000,   // 40MHz  
        .mem_block_symbols = 64,          // 每个 RMT 通道的内存块大小，单位为符号数
        .flags.with_dma = 0,              // 不使用 DMA
    };

    // 创建灯带设备，成功后 rgb_handle 生效
    led_strip_new_rmt_device(&rgb_config, &rmt_config, &rgb_handle);
    led_strip_clear(rgb_handle);        // 熄灭所有灯珠（上电默认关灯）
}

/**
 * @brief 设置灯珠颜色并立即刷新
 * @param red   红色分量 0~255
 * @param green 绿色分量 0~255
 * @param blue  蓝色分量 0~255
 */
void rgb_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    led_strip_set_pixel(rgb_handle, 0, red, green, blue);   // 只写缓存，第 0 颗灯珠
    led_strip_refresh(rgb_handle);                          // 把缓存发送到硬件，灯珠才会亮
}
