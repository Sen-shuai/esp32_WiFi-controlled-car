#ifndef _WIFI_H_
#define _WIFI_H_

#include "esp_err.h"
#include "esp_wifi.h"          // WiFi 驱动接口
#include "esp_netif.h"         // 网络接口层
#include "esp_event.h"         // 默认事件循环

/**
 * @brief 初始化 AP+STA 共存模式
 * @note  ESP32 既连接外部路由器，又创建自己的热点供手机访问。
 *        AP 与 STA 共用一个射频信道；STA 连上路由器后，AP 会自动切换到路由器信道。
 *        内部会先初始化 NVS，调用前不要重复初始化 NVS。
 */
void wifi_init_apsta(void);

#endif
