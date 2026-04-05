#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float integral_limit, float output_limit){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->output = 0;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

float PID_Update(PID_t *pid, float error) {
    // 比例项
    float p_out = pid->Kp * error;
    
    // 积分项（带限幅）
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_out = pid->Ki * pid->integral;
    
    // 微分项（采用测量值微分，避免微分冲击）
    float derivative = (error - pid->last_error) / pid->dt;
    float d_out = pid->Kd * derivative;
    pid->last_error = error;
    
    // 总输出并限幅
    float output = p_out + i_out + d_out;
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}
