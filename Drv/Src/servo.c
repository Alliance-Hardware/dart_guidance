#include "servo.h"
#include "tim.h"
#include "stm32g4xx_hal_tim.h"

static uint32_t servo_ccr_min = SERVO_PULSE_MIN_US;  // 计数最小值 (0.5ms)
static uint32_t servo_ccr_max = SERVO_PULSE_MAX_US;  // 计数最大值 (2.5ms)

// 辅助函数：根据角度计算比较值（CCR）
static uint32_t AngleToCCR(uint8_t angle)
{
    // 线性映射 angle [0,180] -> CCR [min, max]
    return servo_ccr_min + (uint32_t)((float)angle * (servo_ccr_max - servo_ccr_min) / SERVO_ANGLE_MAX);
}

void Servo_Init(void)
{
    //初始化定时器和PWM通道
    MX_TIM4_Init();
    MX_TIM8_Init();

    // 启动定时器PWM输出
    HAL_TIM_PWM_Start(&SERVO1_TIM, SERVO1_CHANNEL);  // 舵机1
    HAL_TIM_PWM_Start(&SERVO2_TIM, SERVO2_CHANNEL);  // 舵机2
    HAL_TIM_PWM_Start(&SERVO3_TIM, SERVO3_CHANNEL);  // 舵机3
    HAL_TIM_PWM_Start(&SERVO4_TIM, SERVO4_CHANNEL);  // 舵机4

    // 初始化所有舵机到中位（90度）
    Servo_SetAngle(1, 90);
    Servo_SetAngle(2, 90);
    Servo_SetAngle(3, 90);
    Servo_SetAngle(4, 90);
}   

void Servo_SetAngle(uint8_t id, uint8_t angle){
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;    
    uint32_t ccr = AngleToCCR(angle);
    switch (id) {
        case 1:__HAL_TIM_SET_COMPARE(&SERVO1_TIM, SERVO1_CHANNEL, ccr);break;
        case 2:__HAL_TIM_SET_COMPARE(&SERVO2_TIM, SERVO2_CHANNEL, ccr);break;
        case 3:__HAL_TIM_SET_COMPARE(&SERVO3_TIM, SERVO3_CHANNEL, ccr);break;
        case 4:__HAL_TIM_SET_COMPARE(&SERVO4_TIM, SERVO4_CHANNEL, ccr);break;
    }
}