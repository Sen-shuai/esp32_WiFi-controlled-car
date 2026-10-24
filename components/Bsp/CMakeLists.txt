set(src_dirs
            src)
set(include_dirs
            inc)
set(requires
            esp_driver_gpio             # led.c 使用 GPIO 输出控制普通 LED
            esp_driver_rmt              # rgb.c 使用 RMT 外设驱动 WS2812 灯珠
            led_strip
            esp_driver_ledc
            esp_driver_gptimer          # gp_tim.c 使用 GPTimer 做 PID 节拍
            esp_driver_uart             # uart.c 使用 UART 收 K230 发来的 JPEG 帧
            esp_wifi                    # wifi.c 使用 WiFi 驱动（AP+STA）
            esp_netif                   # wifi.c 创建 AP / STA 网络接口
            esp_event                   # wifi.c 注册 WiFi / IP 事件回调
            esp_http_server              # video_stream.c 开 81 端口的视频流服务
            nvs_flash)                  # wifi.c 初始化 NVS（WiFi 驱动依赖）

# 向 ESP-IDF 构建系统注册本组件：
#   SRCS_DIRS    - 源文件目录（src），目录下的 .c/.cpp 会被自动收集编译
#   INCLUDE_DIRS - 对外暴露的头文件目录（inc），依赖本组件的其它组件可直接 #include
#   REQUIRES     - 依赖的其它组件（esp_driver_gpio），构建时会自动链接其头文件与库
idf_component_register(SRC_DIRS "${src_dirs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES ${requires})

# 仅对本组件生效的编译选项：
#   -ffast-math            - 放宽浮点运算精度规则以换取速度（可能有精度误差）
#   -O3                    - 最高级别优化
#   -Wno-error=format      - 格式化字符串告警不升级为错误
#   -Wno-format            - 关闭格式化字符串检查告警
component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)