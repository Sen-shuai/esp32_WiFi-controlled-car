#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_wifi.h"                // WiFi 驱动接口
#include "esp_netif.h"               // 网络接口层（TCP/IP 协议栈与 WiFi 之间的桥梁）
#include "esp_event.h"               // ESP-IDF 默认事件循环
#include "esp_log.h"
#include "nvs_flash.h"               // NVS：WiFi 驱动会把配置信息存到 Flash，必须先初始化
#include "esp_mac.h"
#include "wifi.h"

/* ========== 需要你修改的配置 ========== */
/* 下面两个是占位符：clone 下来要改成你自己的路由器名称和密码，否则 STA 连不上（AP 和网页不受影响） */
#define WIFI_STA_SSID       "YOUR_ROUTER_SSID"       // ESP32 要连接的路由器名称
#define WIFI_STA_PASSWORD   "YOUR_ROUTER_PASSWORD"   // 路由器密码

#define WIFI_AP_SSID        "ESP32-RGB"            // 手机等设备看到的 ESP32 热点名称
#define WIFI_AP_PASSWORD    "12345678"             // SoftAP 密码，WPA2 要求至少 8 位
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONNECTIONS 4
#define ESP_MAXIMUM_RETRY   5

static const char *TAG = "wifi_apsta";
static int s_retry_num = 0;                    // STA 连接路由器的已重试次数


/**
 * @brief 同时处理 STA（连接路由器）与 AP（设备接入 ESP32）事件
 * @param event_base 事件来源：WIFI_EVENT 或 IP_EVENT，用来区分是哪类事件
 * @param event_id   具体事件编号：同一个来源下的不同事件
 * @param event_data 事件附带数据的指针；事件不同，要强转成对应的结构体再使用
 * @note  事件判断顺序：先判断 event_base，再判断 event_id。
 *        AP 与 STA 相互独立；即使 STA 连接失败，AP 和网页仍可继续使用。
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* STA 接口已启动：此时才允许发起连接，调用 esp_wifi_connect 开始连路由器 */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA 已启动，正在连接路由器: %s", WIFI_STA_SSID);
        esp_wifi_connect();
    }
    /* STA 与路由器断开：常见原因是密码错误、信号弱或路由器踢设备 */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            s_retry_num++;
            ESP_LOGW(TAG, "STA 连接失败，第 %d 次重试", s_retry_num);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "STA 无法连接 %s；SoftAP 和网页不受影响", WIFI_STA_SSID);
        }
    }
    /* STA 通过 DHCP 拿到 IP：到这一步才说明真正可以上网 */
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /* ip_event_got_ip_t：存放拿到的 IP 信息，常用字段 ip_info.ip 是本机的 IP 地址 */
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "STA 已获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;                       // 连上了就把重试计数清零
    }
    /* SoftAP 启动完成：之后手机就能搜索到这个热点 */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "SoftAP 已启动，SSID: %s", WIFI_AP_SSID);
    }
    /* 有设备连上 ESP32 的 AP：event_data 里有设备的 MAC 和分配的 AID */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        /* wifi_event_ap_staconnected_t：存放接入设备信息，mac 是设备地址，aid 是关联编号 */
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "设备 " MACSTR " 已连接 AP，AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    /* 设备断开 AP：手机掉线时会触发，可在此做状态清理 */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "设备 " MACSTR " 已断开 AP，AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}


/**
 * @brief 初始化 AP+STA 共存模式
 * @note  ESP32 既连接外部路由器，又创建自己的热点供手机访问。
 *        AP 与 STA 共用一个射频信道；STA 连上路由器后，AP 会自动切换到路由器信道。
 */
void wifi_init_apsta(void)
{
    // 初始化 NVS：WiFi 驱动会把配置信息存到 Flash，必须先初始化
    esp_err_t ret = nvs_flash_init();
    // 如果 Flash 空间不足或版本不匹配，则擦除 Flash 并重新初始化
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //初始化TCP/IP (LwIP) 协议栈和网络接口层（esp_netif）以及默认事件循环
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* AP 和 STA 必须各自创建一个网络接口。SoftAP 默认地址是 192.168.4.1。 */
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    /* wifi_init_config_t：WiFi 驱动底层参数（任务优先级、缓冲区数量等），用默认值即可 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册句柄仅用于接口输出；本例运行期间不会注销这些事件回调。 */
    esp_event_handler_instance_t wifi_event_instance;
    esp_event_handler_instance_t ip_event_instance;
    // 注册 WiFi 事件回调：WIFI_EVENT、IP_EVENT 都会触发 wifi_event_handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &wifi_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &ip_event_instance));

    /* wifi_config_t：STA 和 AP 的配置共用一个联合体，用 .sta 或 .ap 选择填哪一侧 */
    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,             // 要连接的路由器名称
            .password = WIFI_STA_PASSWORD,     // 路由器密码
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // 只接受不低于该加密方式的热点
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,  // 兼容 WPA3 的协商方式
        },
    };

    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,                        // 手机搜索到的热点名称
            .ssid_len = strlen(WIFI_AP_SSID),            // SSID 长度，可填 0 让系统自动识别
            .channel = WIFI_AP_CHANNEL,                  // 起始信道，STA 连上后会跟随路由器信道
            .password = WIFI_AP_PASSWORD,                // 热点密码，WPA2 至少 8 位
            .max_connection = WIFI_AP_MAX_CONNECTIONS,   // 最多允许多少个设备同时连接
            .authmode = WIFI_AUTH_WPA2_PSK,              // 加密方式：WPA2-PSK
            .pmf_cfg = {
                .required = false,                       // 不强制要求管理帧保护，兼容老手机
            },
        },
    };

    /* 密码为空时允许创建开放热点；使用 WPA2 时密码必须至少 8 位。 */
    if (strlen(WIFI_AP_PASSWORD) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP+STA 初始化完成；连接 %s 后访问 http://192.168.4.1",
             WIFI_AP_SSID);
}
