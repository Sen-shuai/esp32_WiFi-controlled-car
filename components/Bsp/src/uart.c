#include "uart.h"
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"


static const char *TAG = "uart";


/* 帧缓冲: UART 没有 DMA, 用普通内部 RAM 即可(MALLOC_CAP_INTERNAL 保证不被放到 PSRAM) */
static uint8_t *s_frame_buf[UART_FRAME_QUEUE_NUM] = {NULL};

/* 驱动事件队列: UART_DATA / UART_FIFO_OVF / UART_BUFFER_FULL 等 */
static QueueHandle_t s_evt_queue = NULL;
/* 空闲缓冲索引队列: 帧归还后回到这里等下一帧 */
static QueueHandle_t s_free_queue = NULL;
/* 已收满的帧队列: 供上层 uart_receive() 取 */
static QueueHandle_t s_frame_queue = NULL;

/* 帧同步状态机: 找帧头 -> 取长度 -> 收载荷 */
typedef enum
{
    ST_HEAD0,       /* 等 0xAA */
    ST_HEAD1,       /* 等 0x55 */
    ST_LEN,         /* 收 4 字节长度(小端) */
    ST_PAYLOAD      /* 收 payload */
} sync_state_t;

static sync_state_t s_state = ST_HEAD0;
static uint8_t s_len_bytes[4] = {0};        /* 长度字段的 4 个字节 */
static uint32_t s_len_idx = 0;              /* 当前收到第几个长度字节 */
static uint32_t s_frame_len = 0;            /* 本帧期望长度 */
static uint8_t *s_cur = NULL;               /* 当前正在填充的帧缓冲 */
static uint32_t s_cur_len = 0;              /* 已填充字节数 */
static uint32_t s_drop_cnt = 0;             /* 因上层未及时归还而丢弃的帧数 */

/* 已完成的帧描述 */
typedef struct
{
    uint8_t *buf;
    size_t len;
} frame_item_t;


/* 由缓冲区首地址反查索引, 找不到返回 -1 */
static int buf_index(uint8_t *buf)
{
    for (int i = 0; i < UART_FRAME_QUEUE_NUM; i++)
    {
        if (s_frame_buf[i] == buf)
        {
            return i;
        }
    }
    return -1;
}

/* 归还当前正在填充的缓冲(出错/丢帧时用) */
static void release_buf(uint8_t *buf)
{
    if (buf == NULL)
    {
        return;
    }

    int idx = buf_index(buf);
    if (idx >= 0)
    {
        uint32_t idx_u32 = (uint32_t)idx;
        xQueueSend(s_free_queue, &idx_u32, 0);
    }
}

/* 状态机复位, 重新去找帧头 */
static void frame_reset(void)
{
    s_state = ST_HEAD0;
    s_len_idx = 0;
    s_frame_len = 0;
    s_cur = NULL;
    s_cur_len = 0;
}

/* 逐字节喂给状态机(字节流 -> 一帧帧数据) */
static void feed_byte(uint8_t byte)
{
    switch (s_state)
    {
    case ST_HEAD0:
        s_state = (byte == UART_FRAME_HEAD0) ? ST_HEAD1 : ST_HEAD0;
        break;

    case ST_HEAD1:
        if (byte == UART_FRAME_HEAD1)
        {
            s_state = ST_LEN;
            s_len_idx = 0;
        }
        else
        {
            s_state = ST_HEAD0;     /* 帧头没对上, 重新同步 */
        }
        break;

    case ST_LEN:
        s_len_bytes[s_len_idx++] = byte;
        if (s_len_idx < 4)
        {
            break;
        }

        s_frame_len = (uint32_t)s_len_bytes[0] |
                      ((uint32_t)s_len_bytes[1] << 8) |
                      ((uint32_t)s_len_bytes[2] << 16) |
                      ((uint32_t)s_len_bytes[3] << 24);

        if (s_frame_len == 0 || s_frame_len > UART_FRAME_MAX_SIZE)
        {
            ESP_LOGW(TAG, "bad frame len %u, resync", (unsigned int)s_frame_len);
            frame_reset();          /* 长度异常, 认为同步丢失 */
            break;
        }

        {
            uint32_t idx = 0;
            if (xQueueReceive(s_free_queue, &idx, 0) != pdTRUE)
            {
                s_drop_cnt++;       /* 上层没还缓冲, 没地方放这一帧, 只能丢 */
                ESP_LOGW(TAG, "no free buffer, drop frame (%u)", (unsigned int)s_drop_cnt);
                frame_reset();
                break;
            }
            s_cur = s_frame_buf[idx];
            s_cur_len = 0;
            s_state = ST_PAYLOAD;
        }
        break;

    case ST_PAYLOAD:
        s_cur[s_cur_len++] = byte;
        if (s_cur_len >= s_frame_len)
        {
            frame_item_t item = {.buf = s_cur, .len = s_cur_len};
            if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE)
            {
                s_drop_cnt++;       /* 帧队列满: 上层处理太慢, 丢这一帧 */
                ESP_LOGW(TAG, "frame queue full, drop frame (%u)", (unsigned int)s_drop_cnt);
                release_buf(s_cur);
            }
            frame_reset();
        }
        break;
    }
}

/* 接收任务: 事件驱动地把环形缓冲里的数据搬进帧缓冲 */
static void uart_rx_task(void *arg)
{
    uart_event_t evt;
    uint8_t tmp[256];               /* 中转缓冲, 避免一次读太多占着中断数据 */

    while (1)
    {
        if (xQueueReceive(s_evt_queue, &evt, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        switch (evt.type)
        {
        case UART_DATA:
        {
            // 事件里已告知可读字节数, 这里用 0 超时读, 不阻塞
            size_t remain = evt.size;
            while (remain > 0)
            {
                size_t want = (remain > sizeof(tmp)) ? sizeof(tmp) : remain;
                int rd = uart_read_bytes(UART_PORT_USE, tmp, want, 0);
                if (rd <= 0)
                {
                    break;
                }
                for (int i = 0; i < rd; i++)
                {
                    feed_byte(tmp[i]);
                }
                remain -= (size_t)rd;
            }
            break;
        }

        case UART_FIFO_OVF:     /* 硬件 FIFO 溢出: 硬件流控没接好或阈值太高 */
        case UART_BUFFER_FULL:  /* 环形缓冲满: 任务没及时搬走 */
            ESP_LOGW(TAG, "rx overflow (%s), drop current frame",
                     evt.type == UART_FIFO_OVF ? "fifo" : "ringbuf");
            uart_flush_input(UART_PORT_USE);
            release_buf(s_cur);
            frame_reset();
            break;

        case UART_FRAME_ERR:    /* 帧错误/校验错误: 波特率不匹配或线路质量差 */
        case UART_PARITY_ERR:
            ESP_LOGW(TAG, "line error, drop current frame");
            release_buf(s_cur);
            frame_reset();
            break;

        default:
            break;
        }
    }
}

/* 串口初始化: 配置参数 -> 绑定引脚 -> 安装驱动 -> 建队列/缓冲 -> 启动接收任务 */
void uart_init(void)
{
    // 通信参数
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD_RATE,                    // 波特率, 收发双方必须一致
        .data_bits = UART_DATA_8_BITS,                  // 数据位 8
        .parity = UART_PARITY_DISABLE,                  // 无校验
        .stop_bits = UART_STOP_BITS_1,                  // 1 位停止位(8N1, 每字节 10 bit)
        .flow_ctrl = UART_HW_FLOWCTRL_EN ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_FLOWCTRL_THRESH,    // FIFO 中字节数达此值时拉高 RTS 让对端停发
        .source_clk = UART_SCLK_APB,                    // 80MHz APB, 高波特率下分频更准
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_USE, &uart_cfg));

    // 绑定引脚: 不用硬件流控时 RTS/CTS 传 -1
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_USE,
                                 UART_PIN_TXD,
                                 UART_PIN_RXD,
                                 UART_HW_FLOWCTRL_EN ? UART_PIN_RTS : -1,
                                 UART_HW_FLOWCTRL_EN ? UART_PIN_CTS : -1));

    /*
     * 安装驱动:
     *   rx_buffer_size - 接收环形缓冲, 中断收到的数据先落这里, 任务再搬走
     *   tx_buffer_size - 发送环形缓冲, 写函数先落这里, 中断再逐字节发出
     *   queue_size     - 事件队列长度, 用来收 UART_DATA/FIFO_OVF 等事件
     *   uart_queue     - 事件队列句柄
     */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_USE,
                                        UART_RX_BUF_SIZE,
                                        UART_TX_BUF_SIZE,
                                        32,
                                        &s_evt_queue,
                                        0));

    // 降低高波特率下的中断频率: FIFO 攒多一点再报中断; 超时阈值按字节时间计
    uart_set_rx_full_threshold(UART_PORT_USE, UART_RX_FULL_THRESH);
    uart_set_rx_timeout(UART_PORT_USE, UART_RX_TOUT_THRESH);

    // 波特率是否可达(分频做不出来会失败)
    ESP_ERROR_CHECK(uart_set_baudrate(UART_PORT_USE, UART_BAUD_RATE));

    // 帧缓冲 + 两个队列
    s_free_queue = xQueueCreate(UART_FRAME_QUEUE_NUM, sizeof(uint32_t));
    s_frame_queue = xQueueCreate(UART_FRAME_QUEUE_NUM, sizeof(frame_item_t));

    for (int i = 0; i < UART_FRAME_QUEUE_NUM; i++)
    {
        //  先从 PSRAM 分配
        s_frame_buf[i] = heap_caps_malloc(UART_FRAME_MAX_SIZE, MALLOC_CAP_INTERNAL);
        if (s_frame_buf[i] == NULL)
        {
            ESP_LOGE(TAG, "malloc frame buffer %d failed", i);
            return;
        }
        uint32_t idx_u32 = (uint32_t)i;
        xQueueSend(s_free_queue, &idx_u32, 0);
    }

    // 接收任务: 优先级高于应用任务, 保证环形缓冲被及时搬空
    xTaskCreate(uart_rx_task, "uart_rx", UART_TASK_STACK, NULL, UART_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "uart stream init ok (port:%d baud:%d tx:%d rx:%d rts:%d cts:%d buf:%d frame:%d*%d)",
             UART_PORT_USE, UART_BAUD_RATE, UART_PIN_TXD, UART_PIN_RXD,
             (UART_HW_FLOWCTRL_EN ? UART_PIN_RTS : -1), (UART_HW_FLOWCTRL_EN ? UART_PIN_CTS : -1),
             UART_RX_BUF_SIZE, UART_FRAME_QUEUE_NUM, UART_FRAME_MAX_SIZE);
}

/* 取一帧已接收完成的数据 */
esp_err_t uart_receive(uint8_t **frame, size_t *len, TickType_t ticks_to_wait)
{
    if (frame == NULL || len == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    frame_item_t item = {0};
    if (xQueueReceive(s_frame_queue, &item, ticks_to_wait) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    *frame = item.buf;
    *len = item.len;

    return ESP_OK;
}

/* 归还缓冲区, 让它重新参与接收 */
esp_err_t uart_release(uint8_t *frame)
{
    if (frame == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int idx = buf_index(frame);
    if (idx < 0)
    {
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t idx_u32 = (uint32_t)idx;
    return (xQueueSend(s_free_queue, &idx_u32, 0) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

/* 发送数据 */
esp_err_t uart_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int written = uart_write_bytes(UART_PORT_USE, (const char *)data, len);
    if (written < 0)
    {
        return ESP_FAIL;
    }

    return (written == (int)len) ? ESP_OK : ESP_ERR_NO_MEM;     // TX 环形缓冲不够, 只写入了一部分
}

/* 发送字符串 */
esp_err_t uart_send_str(const char *str)
{
    if (str == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return uart_send((const uint8_t *)str, strlen(str));
}


