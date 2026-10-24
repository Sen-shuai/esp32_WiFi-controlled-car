#ifndef _PID_H_
#define _PID_H_


#include "stdlib.h"


typedef struct {

    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    int setpoint;    // 目标值
    int integral;    // 积分累计值
    int error;       // 当前误差值
    int last_error;  // 上一次误差值
    int integral_max; // 开始积分值
    int output;      // PID 输出值
    int output_max;  // 输出最大值

} pid_config_t;

void PID_get_solut(pid_config_t *pid, int current_value, int *output);


#endif
