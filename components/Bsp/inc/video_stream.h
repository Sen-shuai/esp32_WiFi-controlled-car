#ifndef _VIDEO_STREAM_H
#define _VIDEO_STREAM_H


/* 视频流端口：与控制网页的 80 分开。
 * 必须单独一个 httpd 实例：把视频的长连接和小车控制的短请求隔开，互不干扰。 */
#define VIDEO_STREAM_PORT   81


/*
 * 准备视频资源：最新帧缓冲 / 锁 / 事件组 / 无人拉流后的停止定时器。
 * 此时还不会让 K230 推流，第一个客户端拉流时才发 K230_START_CMD。
 * 依赖 Bsp 的 UART 收帧接口，所以要在 uart_init() 之后调用。
 */
void video_stream_init(void);

/*
 * 启动视频流 HTTP 服务（端口 VIDEO_STREAM_PORT），注册 GET /stream。
 * 多客户端同时观看：每条连接一条独立推流任务，上限见 video_stream.c 的 VIDEO_MAX_CLIENTS。
 */
void video_stream_start_server(void);


#endif
