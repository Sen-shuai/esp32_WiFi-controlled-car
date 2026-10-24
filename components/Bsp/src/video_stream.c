/* ==========================================================================
 * 视频流模块（端口 81，独立 httpd 实例，与小车控制的 80 端口互不干扰）
 *
 * 数据流向：
 *   K230 --UART(AA 55 | len | JPEG)-- uart_rx_task(Bsp, 帧同步)
 *        -- video_rx_task -- 最新一帧缓冲(s_jpeg)
 *        -- MJPEG over HTTP -- 浏览器 <img src="http://IP:81/stream">
 *
 * 流程：有人请求 /stream 时才向 K230 发 K230_START_CMD，K230 开始推流后才创建本地收帧任务，
 *       避免上电就白白占用串口带宽；最后一个客户端断开时延时 K230_STOP_DELAY_MS 再发 K230_STOP_CMD。
 *
 * 多客户端：esp_http_server 是单任务的，在 URI 处理器里直接推流会让第二个客户端永远轮不到处理，
 *           所以 /stream 走异步 —— 处理器立刻返回，每条连接交给一条 stream_client_task 推流。
 *           K230 只推一路，多个客户端共用同一份最新帧。
 * ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"   // 用事件组通知"有新帧到达"
#include "esp_log.h"
#include "esp_heap_caps.h"           // heap_caps_malloc / MALLOC_CAP_SPIRAM
#include "esp_http_server.h"         // ESP-IDF 轻量 HTTP 服务器
#include "uart.h"                    // 本组件的 UART 收帧接口（接 K230）
#include "video_stream.h"

static const char *TAG = "video_stream";


/* ============================ 模块参数 ============================ */
#define VIDEO_JPEG_MAX      UART_FRAME_MAX_SIZE // 单帧上限，与 UART 帧缓冲一致
#define VIDEO_TASK_STACK    (6 * 1024)          // 收帧任务栈
#define VIDEO_TASK_PRIO     7                   // 低于 UART 收帧任务，避免抢占搬运时间
#define VIDEO_MAX_CLIENTS   4                   // 最多同时几个客户端拉流（每个一份帧缓冲 + 一条推流任务）
#define VIDEO_CLIENT_STACK  (3 * 1024)          // 推流任务栈（单位是 Word，实际约 12KB）
#define VIDEO_CLIENT_PRIO   5                   // 与 httpd 任务同级：不抢 PID(5)、收帧(7/10)
#define VIDEO_PART_BOUNDARY "esp32frame"        // multipart 分隔符
#define VIDEO_FRAME_WAIT_MS 2000                // 等新帧的上限，超时只是让出 CPU
#define VIDEO_NEW_FRAME_BIT BIT0                // 新帧到达事件

/* 有人打开视频页面时发给 K230 的启动指令：K230 收到后才开始往串口推 JPEG 帧。
 * 最后一个看视频的人断开后，延时 K230_STOP_DELAY_MS 再发停止指令让 K230 停下。
 * 具体字符按你 K230 那边的协议改。 */
#define K230_START_CMD      "START\r\n"
#define K230_STOP_CMD       "STOP\r\n"
#define K230_STOP_DELAY_MS  2000            // 无人拉流后延时多久才发停止指令（刷新页面时才不会停了又马上开）


/*
 * 约定：K230 通过 UART 每帧发一张完整 JPEG(0xAA 0x55 | len | payload)。
 * 浏览器只认 JPEG，所以走 MJPEG over HTTP：
 *   Content-Type: multipart/x-mixed-replace;boundary=esp32frame
 * 一个响应里不断追加 "帧头 + JPEG + 分隔符"，<img> 指向 /stream 就能持续出画面。
 */
static uint8_t *s_jpeg = NULL;              /* 最新一帧 JPEG(所有客户端共用，后到的帧覆盖旧的) */
static size_t   s_jpeg_len = 0;             /* 最新一帧字节数 */
static uint32_t s_frame_seq = 0;            /* 帧序号：客户端用它判断有没有新帧 */
static SemaphoreHandle_t s_jpeg_lock = NULL;/* 保护上面三个变量 */
static EventGroupHandle_t s_frame_evt = NULL;
static bool     s_rx_task_created = false;  /* 收帧任务是否已创建（第一次拉流时创建，之后一直留着） */
static bool     s_stream_running = false;   /* K230 当前是否在推流 */
static int      s_stream_clients = 0;       /* 当前正在拉流的客户端数 */
static TimerHandle_t s_stop_timer = NULL;   /* 无人拉流后延时发停止指令的定时器 */
static SemaphoreHandle_t s_stream_lock = NULL; /* 保护上面三个变量：异步推流后客户端会从不同任务进出 */

/**
 * @brief 视频接收任务：从 UART 取帧 -> 更新"最新一帧" -> 广播新帧事件
 * @note  只保留最新帧：网络慢时旧帧直接被覆盖，符合实时预览"宁可丢帧不要延迟"的要求。
 */
static void video_rx_task(void *arg)
{
    uint8_t *frame = NULL;
    size_t len = 0;
    uint32_t bad_cnt = 0;

    while (1) {
        if (uart_receive(&frame, &len, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        /* JPEG 文件头是 FF D8，简单判一下，避免把非图片数据推给浏览器 */
        if (len > 4 && frame[0] == 0xFF && frame[1] == 0xD8) {
            if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
                memcpy(s_jpeg, frame, len);
                s_jpeg_len = len;
                s_frame_seq++;
                xSemaphoreGive(s_jpeg_lock);
                /* 广播：多个客户端都在等这一位，都会被唤醒 */
                xEventGroupSetBits(s_frame_evt, VIDEO_NEW_FRAME_BIT);
            }
        } else if (bad_cnt++ % 50 == 0) {
            ESP_LOGW(TAG, "载荷不是 JPEG(首字节 %02X %02X)，客户端将无法显示",
                     frame[0], frame[1]);
        }

        uart_release(frame);                /* 必须归还，否则 UART 没有空闲缓冲会丢帧 */
        frame = NULL;
    }
}

/**
 * @brief 延时到了之后才真的通知 K230 停止推流
 * @note  运行在定时器服务任务里：刷新网页是“先断开再重连”，延时的这段时间里
 *        若又有人拉流，video_client_add() 会停掉这个定时器，不会停了马上又开。
 */
static void k230_stop_timer_cb(TimerHandle_t timer)
{
    (void)timer;

    xSemaphoreTake(s_stream_lock, portMAX_DELAY);
    if (s_stream_clients > 0) {
        xSemaphoreGive(s_stream_lock);
        return;
    }
    s_stream_running = false;
    xSemaphoreGive(s_stream_lock);

    ESP_LOGI(TAG, "已无客户端拉流，向 K230 发送停止指令");
    uart_send_str(K230_STOP_CMD);
}

/**
 * @brief 准备视频资源（缓冲 / 锁 / 事件组），此时还不让 K230 推流
 */
void video_stream_init(void)
{
    s_jpeg_lock = xSemaphoreCreateMutex();
    s_frame_evt = xEventGroupCreate();
    s_stream_lock = xSemaphoreCreateMutex();

    /* 一次性定时器：无人拉流后延时 K230_STOP_DELAY_MS 触发，期间有人拉流就停掉它 */
    s_stop_timer = xTimerCreate("k230_stop", pdMS_TO_TICKS(K230_STOP_DELAY_MS),
                                pdFALSE, NULL, k230_stop_timer_cb);

    /* 帧缓冲优先放 PSRAM：内部 RAM 要留给 WiFi 和 UART 驱动 */
    s_jpeg = heap_caps_malloc(VIDEO_JPEG_MAX, MALLOC_CAP_SPIRAM);
    if (s_jpeg == NULL) {
        s_jpeg = heap_caps_malloc(VIDEO_JPEG_MAX, MALLOC_CAP_INTERNAL);
    }
    if (s_jpeg == NULL) {
        ESP_LOGE(TAG, "视频帧缓冲分配失败");
    }
}

/**
 * @brief 有人开始拉流：作废待发的停止指令，第一个客户端还要让 K230 开始推流
 * @note  收帧任务只创建一次并一直留着：没人看时它阻塞在取帧上，不占 CPU。
 * @return true 表示接受了这个客户端，false 表示已达 VIDEO_MAX_CLIENTS 上限
 */
static bool video_client_add(void)
{
    xSemaphoreTake(s_stream_lock, portMAX_DELAY);

    if (s_stream_clients >= VIDEO_MAX_CLIENTS) {
        xSemaphoreGive(s_stream_lock);
        return false;
    }

    xTimerStop(s_stop_timer, 0);        /* 别把刚连上的流又停掉 */
    s_stream_clients++;

    if (!s_stream_running) {
        s_stream_running = true;

        ESP_LOGI(TAG, "检测到用户拉取视频，向 K230 发送启动指令");
        uart_send_str(K230_START_CMD);

        if (!s_rx_task_created) {
            s_rx_task_created = true;
            xTaskCreate(video_rx_task, "video_rx", VIDEO_TASK_STACK, NULL, VIDEO_TASK_PRIO, NULL);
        }
    }

    xSemaphoreGive(s_stream_lock);
    return true;
}

/**
 * @brief 有人停止拉流：最后一个断开时才延时向 K230 发停止指令
 */
static void video_client_remove(void)
{
    xSemaphoreTake(s_stream_lock, portMAX_DELAY);

    s_stream_clients--;
    ESP_LOGI(TAG, "客户端断开拉流，剩余 %d 个", s_stream_clients);

    if (s_stream_clients <= 0 && s_stream_running) {
        ESP_LOGI(TAG, "无人拉流，%d ms 后向 K230 发送停止指令", K230_STOP_DELAY_MS);
        xTimerStart(s_stop_timer, 0);
    }

    xSemaphoreGive(s_stream_lock);
}

/* 一个客户端一份上下文：异步推流后每个客户端跑在自己的任务里，
 * req 副本与帧拷贝都不共享。 */
typedef struct {
    httpd_req_t *req;          /* httpd_req_async_handler_begin() 给的副本 */
    uint8_t     *jpeg;         /* 本连接独立的帧拷贝 */
} stream_client_t;

/**
 * @brief 一条客户端连接一条推流任务：等新帧 -> 组 multipart 段 -> 发 chunk
 * @note  跑在独立任务里，httpd 任务已经在 stream_get_handler() 返回了，能继续接待下一个客户端，
 *        所以多个设备可以同时看同一路画面（K230 只推一路，多客户端共用同一份最新帧）。
 *        收尾必须调 httpd_req_async_handler_complete()，否则那个 socket 不会被 httpd 回收，
 *        久了 httpd 会报 "error in accept (23)" 并且接不了新连接。
 */
static void stream_client_task(void *arg)
{
    stream_client_t *client = (stream_client_t *)arg;
    httpd_req_t *req = client->req;
    uint8_t *jpeg = client->jpeg;

    /* multipart/x-mixed-replace：服务器可以无限往同一个响应里追加内容，浏览器逐段刷新 */
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" VIDEO_PART_BOUNDARY);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    uint32_t sent_seq = 0;
    uint32_t sent_frames = 0;

    /* 起始分隔符：multipart 正文必须以 --boundary 开头 */
    static const char boundary_head[] = "--" VIDEO_PART_BOUNDARY "\r\n";
    static const char boundary_tail[] = "\r\n--" VIDEO_PART_BOUNDARY "\r\n";

    if (httpd_resp_send_chunk(req, boundary_head, strlen(boundary_head)) == ESP_OK) {
        while (1) {
            /* 等新帧；超时返回时下面会判断有没有未发送过的帧，没有就继续等 */
            xEventGroupWaitBits(s_frame_evt, VIDEO_NEW_FRAME_BIT, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(VIDEO_FRAME_WAIT_MS));

            size_t len = 0;
            if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
                continue;
            }
            if (s_jpeg_len == 0 || s_frame_seq == sent_seq) {   /* 没有新帧 */
                xSemaphoreGive(s_jpeg_lock);
                continue;
            }
            len = s_jpeg_len;
            memcpy(jpeg, s_jpeg, len);
            sent_seq = s_frame_seq;
            xSemaphoreGive(s_jpeg_lock);

            /* 分块发送：帧头 -> JPEG 数据 -> 分隔符。任一步失败都认为客户端已断开 */
            char part[96];
            int part_len = snprintf(part, sizeof(part),
                                    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                    (unsigned int)len);
            if (httpd_resp_send_chunk(req, part, part_len) != ESP_OK) {
                break;
            }
            if (httpd_resp_send_chunk(req, (const char *)jpeg, len) != ESP_OK) {
                break;
            }
            if (httpd_resp_send_chunk(req, boundary_tail, strlen(boundary_tail)) != ESP_OK) {
                break;
            }

            if (++sent_frames % 30 == 0) {
                ESP_LOGI(TAG, "已推送 %u 帧，本帧 %u 字节", (unsigned int)sent_frames, (unsigned int)len);
            }
        }
    }

    /* 收尾：归还 req 副本 -> 释放本连接的缓冲 -> 更新拉流计数（可能触发延时停止） */
    httpd_req_async_handler_complete(req);
    free(jpeg);
    free(client);
    video_client_remove();
    vTaskDelete(NULL);
}

/**
 * @brief GET /stream  MJPEG 实时流（异步：一条连接交给一条独立任务推流）
 * @note  esp_http_server 是单任务的，如果在本处理器里直接推流，第二个客户端的请求永远轮不到处理。
 *        所以这里先 httpd_req_async_handler_begin() 拿一份 req 副本，交给 stream_client_task，
 *        本处理器立刻返回，httpd 任务回去接待下一个客户端。
 *        并发上限 VIDEO_MAX_CLIENTS：每个客户端一份 25KB 帧缓冲 + 一条任务，超了直接回 403。
 */
static esp_err_t stream_get_handler(httpd_req_t *req)
{
    /* 先把可能失败的事做完：这一步之前失败还能用原始 req 直接回错误码 */
    if (!video_client_add()) {
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "拉流客户端数已达上限");
    }

    uint8_t *jpeg = heap_caps_malloc(VIDEO_JPEG_MAX, MALLOC_CAP_SPIRAM);
    if (jpeg == NULL) {
        jpeg = heap_caps_malloc(VIDEO_JPEG_MAX, MALLOC_CAP_INTERNAL);
    }
    if (jpeg == NULL) {
        video_client_remove();
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "视频缓冲分配失败");
    }

    /* 异步化：拿到副本后原始 req 不能再用了，后面一律用 async_req */
    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        free(jpeg);
        video_client_remove();
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "异步请求创建失败");
    }

    stream_client_t *client = malloc(sizeof(stream_client_t));
    if (client == NULL) {
        httpd_req_async_handler_complete(async_req);
        free(jpeg);
        video_client_remove();
        return ESP_FAIL;
    }
    client->req = async_req;
    client->jpeg = jpeg;

    ESP_LOGI(TAG, "客户端开始拉流");

    if (xTaskCreate(stream_client_task, "stream_cli", VIDEO_CLIENT_STACK, client,
                    VIDEO_CLIENT_PRIO, NULL) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(jpeg);
        free(client);
        video_client_remove();
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief 视频流单独开一个 HTTP 服务（端口 81）
 * @note  esp_http_server 是单任务的，所有 URI 处理器在同一个 httpd 任务里串行执行；
 *        /stream 已改成异步（处理器立刻返回），所以这个单任务仍然能连续接待多个客户端，
 *        只要不超过 VIDEO_MAX_CLIENTS 和实例自身的 max_open_sockets。
 *        ctrl_port 必须和 80 端口那个实例的 ctrl 端口错开。
 */
void video_stream_start_server(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = VIDEO_STREAM_PORT;    // 81
    config.ctrl_port = 32769;                  // 每个实例的 ctrl 端口必须互不相同
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "视频流服务器启动失败");
        return;
    }

    const httpd_uri_t stream_uri = {
        .uri = "/stream",                      // 访问路径：http://<IP>:81/stream
        .method = HTTP_GET,
        .handler = stream_get_handler,         // MJPEG 持续推流（异步）
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &stream_uri));
    ESP_LOGI(TAG, "视频流服务已启动: 端口 %d", VIDEO_STREAM_PORT);
}
