#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_http_server.h"       // ESP-IDF 轻量 HTTP 服务器
#include "video_stream.h"           // 视频流模块（端口 81）
#include "led.h"
#include "rgb.h"
#include "exit.h"
#include "ledc.h"
#include "pid.h"
#include "gp_tim.h"
#include "wifi.h"
#include "uart.h"                   // Bsp 的 UART 收帧接口：app_main 里初始化，帧由 video_stream.c 取

/* ================= 小车参数：电机 / 编码器接线与 PID ================= */
#define MOTOR_L_PWM_GPIO    GPIO_NUM_4      // 左电机 ENA（PWM 调速）
#define MOTOR_R_PWM_GPIO    GPIO_NUM_7      // 右电机 ENB（PWM 调速）

/* 驱动模块的四个方向输入：一高一低为一个方向，对调即反向 */
#define MOTOR_L_IN1_GPIO    GPIO_NUM_5      // 左电机 IN1
#define MOTOR_L_IN2_GPIO    GPIO_NUM_6      // 左电机 IN2
#define MOTOR_R_IN1_GPIO    GPIO_NUM_15     // 右电机 IN3
#define MOTOR_R_IN2_GPIO    GPIO_NUM_16     // 右电机 IN4

#define MOTOR_PWM_FREQ_HZ   5000            // 电机 PWM 频率
#define MOTOR_PWM_RES       LEDC_TIMER_13_BIT // PWM 分辨率，13 位对应 0~8191

#define PID_PERIOD_MS       20              // PID 计算周期（也是测速窗口）

/* 速度统一用“脉冲 / 秒”描述，实测：电机满速 11000 脉冲每秒。
 * 测速窗口是 PID_PERIOD_MS，所以进 PID 前要用 SPEED_TO_CNT 换算成“每窗口计数”。 */
#define SPEED_TO_CNT(pps)   ((pps) * PID_PERIOD_MS / 1000)

#define PID_OUT_MAX          100           // PID 输出限幅：占空比上限（13 位原值 0~8191）
#define MOTOR_SPEED_MAX_PPS  11000          // 速度上限：11000 脉冲/秒 -> 220 计数/20ms
#define MOTOR_SPEED_DEFAULT  5000           // 前进 / 后退速度：6000 脉冲/秒 -> 120 计数/20ms
#define MOTOR_SPEED_STEP     500           // 每次加速 / 减速的步长（脉冲/秒）
#define MOTOR_TURN_BIAS      500          // 每按一次转向键的偏置步长（脉冲/秒）
#define MOTOR_TURN_BIAS_MAX  4000           // 转向偏置累计上限（脉冲/秒）
#define MOTOR_SYNC_GAIN      0            // 差速同步系数（实际系数 = 该值 / 10）：把两轮速差拉回目标速差
#define MOTOR_CNT_FULL       SPEED_TO_CNT(MOTOR_SPEED_MAX_PPS)  // 满速时每窗口计数 = 220

/* 编码器计数方向（06.motor_test 实测，前进电平 IN1=1/IN2=0、90% 占空比：
 * 左 -11360、右 -11010，两轮同为负），所以两路都取反，让“前进”时反馈为正，
 * 与 PID 的正目标值同号，否则会变成正反馈、输出顶满。 */
#define ENC_L_SIGN          -1
#define ENC_R_SIGN          -1

/* 两电机的安装方向：车抬起来发 forward，若某一轮转向相反（两轮互相顶住），
 * 把对应宏改成 true，软件里把这一路 IN1/IN2 对调，不用改线。 */
#define MOTOR_L_INVERT      false
#define MOTOR_R_INVERT      false

static const char *TAG = "car_main";    // 日志 TAG，用于区分不同模块的日志输出


/* 小车指令与状态：HTTP 任务负责写入，PID 任务负责读取。
 * 都是单个 int，读写天然原子，不需要加锁。 */
typedef enum {
    CAR_STOP   = 0,                     // 停止
    CAR_FORWARD,                        // 前进：两轮同向正转
    CAR_BACK,                           // 后退：两轮同向反转
    CAR_LEFT,                           // 左转：左轮 -1000 脉冲/秒、右轮 +1000（差速）
    CAR_RIGHT,                          // 右转：左轮 +1000 脉冲/秒、右轮 -1000（差速）
    CAR_FASTER,                         // 加速
    CAR_SLOWER,                         // 减速
} car_cmd_t;

static volatile int s_speed_target = 0;     // 目标速度（脉冲/秒），上电为 0，收到前进/后退后才给速度
static volatile int s_dir_l = 1;        // 左轮方向：1 正转，-1 反转
static volatile int s_dir_r = 1;        // 右轮方向：1 正转，-1 反转
static volatile int s_bias_l = 0;       // 左轮速度偏置（脉冲/秒），正值加大、负值减小
static volatile int s_bias_r = 0;       // 右轮速度偏置（脉冲/秒）
static volatile int s_pid_reset = 0;    // 置 1 后由 PID 任务清掉积分与上一次误差

/* ========== 电机驱动结构体：PWM 输出 / PID / 节拍定时器 ==========
 * 三者都是“配置 + 运行状态”合一的全局变量，统一放在使用它们的函数前面，
 * 下面的函数可以直接引用，不用再往前翻。
 */

/* 两路 PWM：共用一个 LEDC 定时器（频率必须相同），只有通道和引脚不同。
 * 初始占空比 0%，上电先让电机停转。 */
static ledc_config_t s_motor_l = {
    .clk_cfg         = LEDC_AUTO_CLK,
    .duty            = 0,
    .duty_value      = 0,
    .duty_resolution = MOTOR_PWM_RES,
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .freq_hz         = MOTOR_PWM_FREQ_HZ,
    .channel         = LEDC_CHANNEL_0,
    .timer_num       = LEDC_TIMER_0,
    .gpio_num        = MOTOR_L_PWM_GPIO,
};

static ledc_config_t s_motor_r = {
    .clk_cfg         = LEDC_AUTO_CLK,
    .duty            = 0,
    .duty_value      = 0,
    .duty_resolution = MOTOR_PWM_RES,
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .freq_hz         = MOTOR_PWM_FREQ_HZ,
    .channel         = LEDC_CHANNEL_1,
    .timer_num       = LEDC_TIMER_0,
    .gpio_num        = MOTOR_R_PWM_GPIO,
};

/* 两路 PID：左右轮各一套，先用保守参数，之后按实际响应整定。
 * integral / last_error 是运行量；setpoint 每拍由 pid_task 写成 方向 × 目标速度。
 * begin_integral：误差超过该范围就不再积分，避免积分饱和。 */
static pid_config_t s_pid_l = {
    .Kp              = 0.10f,
    .Ki              = 0.01f,
    .Kd              = 0.0f,
    .setpoint        = SPEED_TO_CNT(MOTOR_SPEED_DEFAULT),
    .integral        = 0,
    .error           = 0,
    .last_error      = 0,
    .integral_max  = PID_OUT_MAX,
    .output          = 0,
    .output_max      = PID_OUT_MAX,      // PID 输出限幅 = 13 位占空比上限
};

static pid_config_t s_pid_r = {
    .Kp              = 0.10f,
    .Ki              = 0.01f,
    .Kd              = 0.0f,
    .setpoint        = SPEED_TO_CNT(MOTOR_SPEED_DEFAULT),
    .integral        = 0,
    .error           = 0,
    .last_error      = 0,
    .integral_max    = PID_OUT_MAX,
    .output          = 0,
    .output_max      = PID_OUT_MAX,
};

/* PID 节拍定时器：1MHz 计数（1us 一步），每 PID_PERIOD_MS 报警一次并自动重装。
 * 报警回调给 pid_task 发任务通知，所以 PID 周期就等于这里的报警周期。 */
static tim_t s_pid_tim = {
    .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
    .tim_clk_hz    = 1000000,               // 1MHz
    .direction     = GPTIMER_COUNT_UP,
    .tim_set       = 0,                     // 自动重装到 0
    .alarm_value   = PID_PERIOD_MS * 1000,  // 20ms -> 20000us
    .counter_value = 0,
    .auto_reload   = true,
};


static void car_apply_cmd(car_cmd_t cmd);   // 实现在下面的电机 / PID 部分


/* ==========================================================================
 * 第一部分：控制网页 + HTTP 控制接口（端口 80）
 *   浏览器访问 80 端口拿到这个页面 -> 点按钮请求 /car?cmd=xxx -> car_apply_cmd()
 *   这里只改目标状态变量，真正把指令变成 PWM 输出由第三部分的 PID 任务完成。
 * ========================================================================== */

/* 网页直接存放在 Flash 中，不需要 SPIFFS 文件系统。 */
static const char CAR_CONTROL_PAGE[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 小车控制</title><style>"
    "body{font-family:sans-serif;text-align:center;background:#141414;color:#eee;}"
    "button{width:96px;height:64px;margin:6px;font-size:18px;border:0;border-radius:10px;"
    "background:#2e7cf6;color:#fff;}"
    "button:active{background:#1856b8;}"
    "#st{margin-top:16px;font-size:16px;color:#8f8;}"
    "</style></head><body>"
    "<h2>ESP32 小车控制</h2>"
    "<div><button data-cmd=forward onclick=go(this)>前进</button></div>"
    "<div><button data-cmd=left onclick=go(this)>左转</button>"
    "<button data-cmd=stop onclick=go(this)>停止</button>"
    "<button data-cmd=right onclick=go(this)>右转</button></div>"
    "<div><button data-cmd=back onclick=go(this)>后退</button></div>"
    "<div><button data-cmd=slower onclick=go(this)>减速</button>"
    "<button data-cmd=faster onclick=go(this)>加速</button></div>"
    "<div><img id='video' alt='视频流' style='width:100%;max-width:360px;border-radius:8px;background:#000'></div>"\
    "<div id='st'>等待操作</div>"
    "<script>"
    "document.getElementById('video').src='http://'+location.hostname+':81/stream';"
    "function go(el){var c=el.dataset.cmd;fetch('/car?cmd='+c)"
    ".then(function(r){return r.json();})"
    ".then(function(j){document.getElementById('st').innerText='速度 '+j.speed+' / '+j.max;});}"
    "</script></body></html>";

/**
 * @brief GET / 返回小车控制网页
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, CAR_CONTROL_PAGE, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief GET /car?cmd=forward|back|left|right|stop|faster|slower 控制小车
 * @note  这里只改状态（目标速度 / 方向），真正输出到电机由 PID 任务完成。
 */
static esp_err_t car_get_handler(httpd_req_t *req)
{
    char query[32];
    char cmd_text[12];

    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0 || query_len >= sizeof(query) ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "cmd", cmd_text, sizeof(cmd_text)) != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "参数格式应为 /car?cmd=forward|back|left|right|stop|faster|slower");
    }

    car_cmd_t cmd;
    if (strcmp(cmd_text, "forward") == 0) {
        cmd = CAR_FORWARD;
    } else if (strcmp(cmd_text, "back") == 0) {
        cmd = CAR_BACK;
    } else if (strcmp(cmd_text, "left") == 0) {
        cmd = CAR_LEFT;
    } else if (strcmp(cmd_text, "right") == 0) {
        cmd = CAR_RIGHT;
    } else if (strcmp(cmd_text, "stop") == 0) {
        cmd = CAR_STOP;
    } else if (strcmp(cmd_text, "faster") == 0) {
        cmd = CAR_FASTER;
    } else if (strcmp(cmd_text, "slower") == 0) {
        cmd = CAR_SLOWER;
    } else {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "未知指令");
    }

    car_apply_cmd(cmd);
    ESP_LOGI(TAG, "收到小车指令: %s, 目标速度=%d", cmd_text, (int)s_speed_target);

    char response[64];
    snprintf(response, sizeof(response),
             "{\"cmd\":\"%s\",\"speed\":%d,\"max\":%d}",
             cmd_text, (int)s_speed_target, MOTOR_SPEED_MAX_PPS);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief 启动 HTTP 服务并注册页面与小车控制接口
 */
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    /* httpd_config_t：服务器参数（端口、栈大小等），HTTPD_DEFAULT_CONFIG 默认监听 80 端口 */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;            // 连接不够用时自动关闭最久未使用的连接

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败");
        return NULL;
    }

    /* httpd_uri_t：一条“网址 -> 处理函数”的绑定关系，注册后才能被访问 */
    const httpd_uri_t root_uri = {
        .uri = "/",                            // 访问路径：http://192.168.4.1/
        .method = HTTP_GET,                    // 只接受 GET 请求
        .handler = root_get_handler,           // 匹配后调用的回调函数
        .user_ctx = NULL,                      // 传给回调的用户数据，这里不需要
    };
    // 注册 /car 接口，访问示例：http://192.168.4.1/car?cmd=forward
    const httpd_uri_t car_uri = {
        .uri = "/car",                         // 访问路径：http://192.168.4.1/car?cmd=forward
        .method = HTTP_GET,
        .handler = car_get_handler,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &car_uri));
    ESP_LOGI(TAG, "HTTP 服务已启动: http://192.168.4.1");
    return server;
}


/* ==========================================================================
 * 第二部分：视频流（端口 81）
 *   已经拆到 video_stream.c / video_stream.h，这里只在 app_main 里调用：
 *     video_stream_init()          准备帧缓冲 / 锁 / 事件组（要放在 uart_init 之后）
 *     video_stream_start_server()  启动 81 端口的 httpd 并注册 GET /stream
 *   多客户端异步推流、无人拉流时延时向 K230 发停止指令都在 video_stream.c 里。
 * ========================================================================== */


/* ==========================================================================
 * 第三部分：小车电机控制与 PID 闭环
 *   car_apply_cmd()   网页指令 -> 改目标速度 / 方向 / 转向偏置（只改状态）
 *   motor_apply()     PID 输出 -> IN1/IN2 方向电平 + PWM 占空比
 *   pid_task()        GPTimer 每 20ms 唤醒：读编码器计数 -> 两轮同步修正
 *                     -> 各自 PID -> 输出占空比
 *   速度单位：脉冲/秒 -> 每窗口计数用 SPEED_TO_CNT 换算。
 * ========================================================================== */

/**
 * @brief 执行一次小车指令，只更新状态，具体输出由 PID 任务负责
 */
static void car_apply_cmd(car_cmd_t cmd)
{
    switch (cmd) {
    /* 前进 / 后退：两轮同向，同时清掉转向偏置 */
    case CAR_FORWARD: s_dir_l = 1;  s_dir_r = 1;  s_bias_l = 0; s_bias_r = 0; break;
    case CAR_BACK:    s_dir_l = -1; s_dir_r = -1; s_bias_l = 0; s_bias_r = 0; break;

    /* 转向：把内侧占空比调小、外侧调大，靠差速转弯。
     * 可以连续点，每点一次偏置再累加 MOTOR_TURN_BIAS。 */
    case CAR_LEFT:    s_dir_l = 1;  s_dir_r = 1; s_bias_l -= MOTOR_TURN_BIAS; s_bias_r += MOTOR_TURN_BIAS; break;
    case CAR_RIGHT:   s_dir_l = 1;  s_dir_r = 1; s_bias_l += MOTOR_TURN_BIAS; s_bias_r -= MOTOR_TURN_BIAS; break;

    case CAR_FASTER:  s_speed_target += MOTOR_SPEED_STEP; break;
    case CAR_SLOWER:  s_speed_target -= MOTOR_SPEED_STEP; break;
    case CAR_STOP:    s_speed_target = 0; break;
    }

    /* 转向偏置限幅，避免一直点转向键把一侧压到 0、另一侧拉满 */
    if (s_bias_l > MOTOR_TURN_BIAS_MAX)  { s_bias_l = MOTOR_TURN_BIAS_MAX;  }
    if (s_bias_l < -MOTOR_TURN_BIAS_MAX) { s_bias_l = -MOTOR_TURN_BIAS_MAX; }
    if (s_bias_r > MOTOR_TURN_BIAS_MAX)  { s_bias_r = MOTOR_TURN_BIAS_MAX;  }
    if (s_bias_r < -MOTOR_TURN_BIAS_MAX) { s_bias_r = -MOTOR_TURN_BIAS_MAX; }

    /* 停止后再给方向指令时，速度还是 0，这里恢复一个默认值才动得起来 */
    if (cmd != CAR_STOP && cmd != CAR_FASTER && cmd != CAR_SLOWER &&
        s_speed_target == 0) {
        s_speed_target = MOTOR_SPEED_DEFAULT;
    }

    if (s_speed_target > MOTOR_SPEED_MAX_PPS) {
        s_speed_target = MOTOR_SPEED_MAX_PPS;   // 限速：11000 脉冲/秒
    } else if (s_speed_target < 0) {
        s_speed_target = 0;
    }

    if (cmd == CAR_STOP) {
        s_bias_l = s_bias_r = 0;        // 停车时转向偏置也一起清掉
        s_pid_reset = 1;                // 停车时清积分，下次起步不会带着旧积分冲出去
    }
}


/**
 * @brief 把 PID 输出作用到一路电机：符号决定方向，绝对值决定占空比
 * @param pwm    该轮对应的 PWM 配置（EN 脚）
 * @param in1    该轮方向输入 1
 * @param in2    该轮方向输入 2
 * @param output PID 输出，正 = 正转，负 = 反转
 * @param invert true 时把这一路的 IN1/IN2 对调，用于纠正反向安装的电机
 * @note  转向偏置加在速度目标上（见 pid_task），这里只管把 PID 输出变成方向和占空比。
 */
static void motor_apply(ledc_config_t *pwm, gpio_num_t in1, gpio_num_t in2, int output, bool invert)
{
    int duty = (output < 0) ? -output : output; // 先取大小，方向由 IN1/IN2 决定

    if (duty > PID_OUT_MAX) {
        duty = PID_OUT_MAX;                  // 兜底限幅，PID 内部已经限过一次
    }

    bool forward = (output >= 0) != invert;     // 需要取反的轮子把方向电平互换

    if (duty == 0) {                            // 占空比为 0：IN1/IN2 同时拉低，电机停转
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
    } else if (forward) {                       // 正转：IN1 高、IN2 低
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
    } else {                                    // 反转：IN1 低、IN2 高
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
    }

    ledc_pwm_set_duty(pwm, duty);
}


/**
 * @brief 初始化两路电机 PWM 与四个编码器输入引脚
 */
static void motor_and_encoder_init(void)
{
    /* 四个方向输入引脚配置为推挽输出 */
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_L_IN1_GPIO) | (1ULL << MOTOR_L_IN2_GPIO) |
                        (1ULL << MOTOR_R_IN1_GPIO) | (1ULL << MOTOR_R_IN2_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_cfg);
    /* 四个方向脚全部拉低，上电时电机不转 */
    gpio_set_level(MOTOR_L_IN1_GPIO, 0);
    gpio_set_level(MOTOR_L_IN2_GPIO, 0);
    gpio_set_level(MOTOR_R_IN1_GPIO, 0);
    gpio_set_level(MOTOR_R_IN2_GPIO, 0);

    ledc_init(&s_motor_l);
    ledc_init(&s_motor_r);
    ledc_pwm_set_duty(&s_motor_l, 0);       // 上电先输出 0 占空比，避免电机突然转动
    ledc_pwm_set_duty(&s_motor_r, 0);

    exit_init(LA_EXIT_PIN, 0);          // 左轮 A 相
    exit_init(LB_EXIT_PIN, 1);          // 左轮 B 相
    exit_init(RA_EXIT_PIN, 2);          // 右轮 A 相
    exit_init(RB_EXIT_PIN, 3);          // 右轮 B 相
}


/**
 * @brief PID 调速任务：测速 -> 计算 PID -> 更新两路 PWM 占空比（固定在 core1）
 * @param arg 创建任务时传入的参数，这里未使用
 * @note  测速方式：读取编码器在 PID_PERIOD_MS 内累计的计数值增量当作速度。
 *        L_count / R_count 由编码器中断在 ISR 中修改，任务里只做读取。
 *        不用 vTaskDelay 轮询：任务阻塞在任务通知上，由 GPTimer 报警回调唤醒，
 *        周期由硬件定时器决定，任务执行时间不会累积成周期漂移。
 */
static void pid_task(void *arg)
{
    int last_l = L_count;               // 上一周期的左轮累计计数
    int last_r = R_count;               // 上一周期的右轮累计计数

    while (1) {
        /* 等定时器报警；超时用 portMAX_DELAY，避免超时与通知双重触发导致一拍算两次。
         * pdTRUE：进入前把通知值清 0，只按“有没有新的一拍”处理。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 乘上方向符号：得到单次时间间隔内的脉冲差都为正 */
        int speed_l = ENC_L_SIGN * (L_count - last_l);
        int speed_r = ENC_R_SIGN * (R_count - last_r);
        last_l = L_count;
        last_r = R_count;

        if (s_pid_reset) {
            s_pid_l.integral = s_pid_r.integral = 0;
            s_pid_l.last_error = s_pid_r.last_error = 0;
            s_pid_reset = 0;
        }

        /* 目标速度与转向偏置都是“脉冲/秒”，先换算成每个测速窗口的计数，
         * 再加上方向符号：负号表示反转，PID 自己就能算出该往哪边加力。 */
        int base_cnt  = SPEED_TO_CNT(s_speed_target);
        int max_cnt   = MOTOR_CNT_FULL;

        // 计算左右两边目标值，bias已在car_apply_cmd更改
        int target_l  = base_cnt + SPEED_TO_CNT(s_bias_l);
        int target_r  = base_cnt + SPEED_TO_CNT(s_bias_r);

        //目标值都取正，后面乘方向dir
        if (target_l < 0) 
        { 
            target_l = 0; 
        } 
        else if (target_l > max_cnt) 
        { 
            target_l = max_cnt; 
        }

        if (target_r < 0) 
        { 
            target_r = 0; 
        } 
        else if (target_r > max_cnt) 
        { 
            target_r = max_cnt; 
        }

        int sp_l = s_dir_l * target_l;
        int sp_r = s_dir_r * target_r;
        s_pid_l.setpoint = sp_l;
        s_pid_r.setpoint = sp_r;

        int out_l = 0;
        int out_r = 0;
        PID_get_solut(&s_pid_l, speed_l, &out_l);
        PID_get_solut(&s_pid_r, speed_r, &out_r);

        /* 两轮同步修正：让“实际速差”跟上“目标速差”。
         * 直线时目标速差为 0，谁快就压谁、给慢的那边补一点，车才走得直；
         * 转弯时目标速差非 0，修正会把速差拉到转向要求的差值上，转弯更利落。 */
        int diff_err = (speed_l - speed_r) - (sp_l - sp_r);
        int sync = diff_err * MOTOR_SYNC_GAIN / 10;
        out_l -= sync;
        out_r += sync;

        if (out_l >  PID_OUT_MAX) { out_l =  PID_OUT_MAX; }
        else if (out_l < -PID_OUT_MAX) { out_l = -PID_OUT_MAX; }
        if (out_r >  PID_OUT_MAX) { out_r =  PID_OUT_MAX; }
        else if (out_r < -PID_OUT_MAX) { out_r = -PID_OUT_MAX; }

        //把设置同步到电机
        motor_apply(&s_motor_l, MOTOR_L_IN1_GPIO, MOTOR_L_IN2_GPIO, out_l, MOTOR_L_INVERT);
        motor_apply(&s_motor_r, MOTOR_R_IN1_GPIO, MOTOR_R_IN2_GPIO, out_r, MOTOR_R_INVERT);

        static int time_add = 0;
        time_add ++;
        if(time_add >= 50)
        {
            time_add = 0;
            /* 打印实测编码数（每个测速窗口内的计数增量），不打印目标值，方便看实际转速 */
            ESP_LOGI(TAG, "目标速度 %d ,实测编码数 L=%d R=%d, PID 输出 L=%d%% R=%d&&",s_pid_l.setpoint,
                speed_l, speed_r, out_l, out_r);
        }
    }
}


void app_main(void)
{
    led_init();
    rgb_init();
    rgb_set_color(0, 0, 0);             // 上电时先关闭 RGB 灯

    uart_init();                        // 视频数据从这个串口进来(0xAA 0x55 | len | JPEG)
    video_stream_init();            // 建最新帧缓冲；收帧任务等有人拉流时再启动

    /* HTTP 服务不等待 STA 联网，保证路由器不可用时 AP 控制仍然可用。 */
    wifi_init_apsta();
    (void)start_webserver();            // 控制网页（端口 80）
    video_stream_start_server();    // 视频流（端口 81，独立 httpd 实例，不阻塞小车控制）

    motor_and_encoder_init();

    /* 固定到 core1，core0 留给 WiFi / 协议栈，避免网络繁忙时调速周期被拉长。
     * 参数依次是：任务函数, 任务名(调试用), 栈大小(单位 Word，4096≈16KB),
     *             传给任务的参数, 优先级(越大越先执行), 任务句柄, 绑定的核(0/1)
     * 句柄存进 pid_task_handle，定时器回调靠它唤醒 pid_task。
     */
    xTaskCreatePinnedToCore(pid_task, "pid_task", 2048, NULL, 5, &pid_task_handle, 1);

    /* 最后启动节拍定时器：早于上面的句柄赋值会导致回调无处可发。 */
    GP_TIM_Init(&s_pid_tim);

    /* 板载普通 LED 每秒翻转一次，表示主任务仍在运行。 */
    while (1) {
        LED_toggle();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
