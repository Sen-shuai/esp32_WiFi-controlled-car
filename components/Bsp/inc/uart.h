#ifndef _UART_H
#define _UART_H


#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"


/* 串口外设: UART0 常被 USB/下载口占用, 这里用 UART1 */
#define UART_PORT_USE           UART_NUM_1

/* 串口引脚(按实际接线改)
 * 注意: 小车工程里 GPIO4/5 已被左轮 ENA / IN1 占用, 所以不能用参考工程的 RTS=4 / CTS=5,
 *       这里改成不用硬件流控, 只接 TX(17) / RX(18) 两根线。 */
#define UART_PIN_TXD            17              // 接 K230 的 RX (绿色)
#define UART_PIN_RXD            18              // 接 K230 的 TX (黄色)
#define UART_PIN_RTS            (-1)            /* 硬件流控: 本端告诉对端"别发了"(不用则填 -1) */
#define UART_PIN_CTS            (-1)            /* 硬件流控: 对端告诉本端"可以发"(不用则填 -1) */
#define UART_HW_FLOWCTRL_EN     0               /* 1:开硬件流控(需要接 RTS/CTS 两根线)  0:关 */

/* 通信参数: 8N1, 波特率上限见 SOC_UART_BITRATE_MAX(S3 = 5Mbps) */
#define UART_BAUD_RATE          3000000         /* 波特率: 8N1 下理论约 300KB/s */
#define UART_RX_BUF_SIZE        (32 * 1024)     /* 驱动接收环形缓冲区(内部 RAM, 抗抖动) */
#define UART_TX_BUF_SIZE        (2 * 1024)      /* 驱动发送环形缓冲区 */
#define UART_RX_FULL_THRESH     100             /* RX FIFO(128B)满中断阈值: 越大中断越少, 溢出风险越高 */
#define UART_RX_TOUT_THRESH     32              /* RX 超时阈值, 单位=发一个字节的时间 */
#define UART_FLOWCTRL_THRESH    64              /* 硬件流控阈值: FIFO 内字节数达此值时拉高 RTS 让对端停发 */

/* 帧格式(UART 是字节流, 没有 CS 那种帧边界, 必须自己定协议) */
/* 0xAA 0x55 | len(4字节, 小端) | payload(len 字节) */
#define UART_FRAME_HEAD0        0xAA
#define UART_FRAME_HEAD1        0x55
#define UART_FRAME_HEAD_LEN     6               /* 2 字节帧头 + 4 字节长度 */
#define UART_FRAME_MAX_SIZE     (25 * 1024)     /* 单帧最大字节数 */
#define UART_FRAME_QUEUE_NUM    2               /* 双缓冲: 处理一帧时仍能接收下一帧 */

/* 接收任务: 专门把环形缓冲搬进帧缓冲并做帧同步 */
#define UART_TASK_STACK         (4 * 1024)
#define UART_TASK_PRIO          10              /* 高于普通应用任务, 保证及时搬走数据 */


/* 初始化: 配置参数 -> 绑定引脚 -> 安装驱动(大环形缓冲+事件队列) -> 启动接收任务 */
void uart_init(void);

/*
 * 取一帧已接收完成的数据(阻塞)
 * frame    - 输出: 帧缓冲区首地址
 * len      - 输出: 本帧字节数
 * ticks    - 阻塞超时(portMAX_DELAY 表示一直等)
 * 取出的缓冲区由调用者占用, 处理完必须调用 uart_release() 归还
 */
esp_err_t uart_receive(uint8_t **frame, size_t *len, TickType_t ticks_to_wait);

/* 归还缓冲区, 让它重新参与接收 */
esp_err_t uart_release(uint8_t *frame);

/* 发送数据: 拷进 TX 环形缓冲即返回, 实际发送由中断完成 */
esp_err_t uart_send(const uint8_t *data, size_t len);

/* 发送字符串(不发送结尾的 '\0') */
esp_err_t uart_send_str(const char *str);


#endif


