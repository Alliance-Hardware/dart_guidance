#ifndef __SERVO_H
#define __SERVO_H

#include "stm32g4xx_hal.h"

// 舵机参数：周期20ms，脉宽0.5ms~2.5ms对应0~180度
#define SERVO_PWM_PERIOD_MS   20.0f
#define SERVO_PULSE_MIN_US    500   // 0.5ms
#define SERVO_PULSE_MAX_US    2500  // 2.5ms
#define SERVO_ANGLE_MIN       0
#define SERVO_ANGLE_MAX       180

// 舵机数量和对应的定时器通道（根据实际连接修改）
#define SERVO1                 1
#define SERVO1_TIM             htim4
#define SERVO1_CHANNEL         TIM_CHANNEL_1
#define SERVO2                 2
#define SERVO2_TIM             htim8
#define SERVO2_CHANNEL         TIM_CHANNEL_1
#define SERVO3                 3
#define SERVO3_TIM             htim8
#define SERVO3_CHANNEL         TIM_CHANNEL_2
#define SERVO4                 4
#define SERVO4_TIM             htim8
#define SERVO4_CHANNEL         TIM_CHANNEL_3

/**
 * @brief 初始化舵机控制
 */
void Servo_Init(void);

/**
 * @brief 设置舵机PWM值
 * @param id     舵机ID（1~4）
 * @param angle  角度值（0~180）
 */
void Servo_SetAngle(uint8_t id, uint8_t angle);

#endif
