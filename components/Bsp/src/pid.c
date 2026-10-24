#include "pid.h"



void PID_get_solut(pid_config_t *pid, int current_value, int *output)
{

    int result = 0;
    int error = pid->setpoint - current_value;  // 计算误差

    pid->integral += error;  
    // 抗积分饱和
    if(pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if(pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    int derivative = error - pid->last_error;   // 计算微分
    result = (int)(pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative);  // PID 输出

    if(result > pid->output_max)
    {
        result = pid->output_max;  // 限制输出范围
    }
    else if(result < -(pid->output_max))
    {
        result = -(pid->output_max); // 限制输出范围
    }
    *output = result;
    pid->last_error = error;                    // 更新上一次误差值

}

