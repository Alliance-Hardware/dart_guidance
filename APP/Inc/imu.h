#ifndef __IMU_H
#define __IMU_H

#include "stm32g4xx_hal.h"
#include "mahony.h"

/* IMU 状态结构体 */
typedef struct {
    mahony_t mahony;            // Mahony滤波器实例
    SPI_HandleTypeDef *hspi;    // SPI句柄（用于BMI088通信）
    uint32_t last_tick;         // 上一次更新的系统滴答计数（毫秒）
    float dt;                   // 实际时间间隔（秒）
    float roll;                 // 横滚角（度）
    float pitch;                // 俯仰角（度）
    float yaw;                  // 航向角（度）
} imu_t;

/**
 * @brief 初始化IMU模块
 * @param imu  IMU对象指针
 * @param hspi SPI句柄（已配置好时钟、极性相位等）
 * @param Kp   Mahony滤波器比例系数
 * @param Ki   Mahony滤波器积分系数
 * @retval HAL_OK 成功，否则错误码
 */
HAL_StatusTypeDef imu_init(imu_t *imu, SPI_HandleTypeDef *hspi, float Kp, float Ki);

/**
 * @brief 更新IMU姿态（应周期性调用）
 * @param imu IMU对象指针
 * @retval HAL_OK 成功，否则错误码
 */
HAL_StatusTypeDef imu_update(imu_t *imu);

/**
 * @brief 获取当前欧拉角（度）
 * @param imu   IMU对象指针
 * @param roll  横滚角输出指针
 * @param pitch 俯仰角输出指针
 * @param yaw   航向角输出指针
 */
void imu_get_euler(imu_t *imu, float *roll, float *pitch, float *yaw);

#endif /* __IMU_H */