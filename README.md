# esp32_WiFi-controlled-car

ESP32-S3 无线小车 + K230 图传。浏览器开网页按钮控车，同时能实时看摄像头画面。

- **控制**：ESP32-S3 同时开 AP + STA，手机/电脑浏览器打开控制页，点按钮走 HTTP 请求；两轮各一套编码器 PID 闭环调速
- **图传**：K230 通过串口把 JPEG 帧发给 ESP32，ESP32 在 81 端口以 MJPEG over HTTP 推出去，网页 `<img>` 直连出画面
- **多客户端**：异步推流，最多 4 个设备同时看；**无人拉流 2 秒后自动向 K230 发 STOP** 停止推流，有人再看时重新发 START

## 硬件

主控：ESP32-S3（16MB Flash + Octal PSRAM），ESP-IDF v6.1

| 功能 | 引脚 | 备注 |
|---|---|---|
| 左电机 ENA（PWM） | GPIO4 | LEDC 5kHz / 13bit |
| 左电机 IN1 / IN2 | GPIO5 / GPIO6 | 一高一低为一个方向 |
| 右电机 ENB（PWM） | GPIO7 | 与左轮共用 LEDC 定时器 |
| 右电机 IN3 / IN4 | GPIO15 / GPIO16 | |
| 左编码器 A / B | GPIO10 / GPIO11 | 双边沿中断，四倍频 |
| 右编码器 A / B | GPIO12 / GPIO13 | |
| UART1 TX / RX | GPIO17 / GPIO18 | 接 K230 的 RX / TX，3Mbps 8N1，无硬件流控 |
| 板载 LED | GPIO2 | |
| RGB 灯珠 | GPIO48 | RMT 驱动，1 颗 |

> 电机转向装反了不用改线：改 `main.c` 里的 `MOTOR_L_INVERT` / `MOTOR_R_INVERT` 即可。

## 编译与烧录

```bash
idf.py set-target esp32s3    # 只在第一次需要
idf.py build
idf.py -p COMx flash monitor # COMx 换成你的串口
```

关键配置已经在 `sdkconfig` 里配好（PSRAM 打开、Flash 16MB、使用 `partitions-16Mib.csv`），换机器 clone 下来直接 build 即可。

## 怎么访问

**方式一：直连小车的热点**

- 热点名 `ESP32-RGB`，密码 `12345678`（改 `components/Bsp/src/wifi.c` 顶部的 `WIFI_AP_SSID` / `WIFI_AP_PASSWORD`）
- 控制页：`http://192.168.4.1`
- 视频流：`http://192.168.4.1:81/stream`

**方式二：和电脑在同一个路由器下**

- 小车会以 STA 身份连上路由器，SSID 和密码在 `components/Bsp/src/wifi.c` 顶部改（`WIFI_STA_SSID` / `WIFI_STA_PASSWORD`）
- 连上后串口会打印 `STA 已获取 IP: 192.168.x.x`，用**那个 IP**，不是 192.168.4.1
- 控制页：`http://<STA_IP>`；视频流：`http://<STA_IP>:81/stream`

> 端口别漏：80 是控制页，81 才是视频流。只开控制页的话画面会自动加载，不用手输地址。

## K230 串口协议

数据帧（K230 → ESP32）：

```
0xAA 0x55 | len(4 字节，小端) | payload(len 字节 JPEG)
```

指令（ESP32 → K230），具体字符在 `components/Bsp/src/video_stream.c` 的宏里改：

| 指令 | 时机 |
|---|---|
| `START\r\n` | 第一个客户端开始拉流时 |
| `STOP\r\n` | 最后一个客户端断开后延时 2 秒 |

## 目录结构

```
main/
  main.c              控制网页 + HTTP 控制接口（端口 80）+ 电机 PID 闭环
components/Bsp/
  src/led.c rgb.c     板载 LED / RGB 灯珠
  src/ledc.c pid.c gp_tim.c exit.c    PWM、PID、节拍定时器、编码器输入捕获
  src/uart.c wifi.c   串口收帧、AP+STA 网络
  src/video_stream.c  视频流：收帧、MJPEG 推流、多客户端管理、K230 启停
partitions-16Mib.csv  16MB Flash 分区表
```

## 速度参数

都在 `main.c` 顶部，实测电机满速 11000 脉冲/秒：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `MOTOR_SPEED_MAX_PPS` | 11000 | 速度上限 |
| `MOTOR_SPEED_DEFAULT` | 5000 | 前进/后退速度 |
| `MOTOR_SPEED_STEP` | 500 | 每次加/减速的步长 |
| `MOTOR_TURN_BIAS` | 500 | 每点一次转向键的偏置步长 |
| `PID_PERIOD_MS` | 20 | PID 周期（也是测速窗口） |

## 已知限制

- 视频流并发上限 4 个客户端（`VIDEO_MAX_CLIENTS`），每个约 25KB 缓冲
- `LWIP_MAX_SOCKETS=10`，要更多观众需要同时调大它和 TCP 发送窗口
- AP+STA 共用射频，AP 信道会自动跟随 STA 所在路由器信道（默认起在信道 1）
- STA 连不上路由器时重试 5 次后不再重连，AP 和网页不受影响
