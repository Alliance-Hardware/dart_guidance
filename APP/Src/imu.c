#include "imu.h"
#include "bmi088.h"
#include <math.h>

/* 将毫秒转换为秒 */
#define MS_TO_SEC(x)   ((float)(x) / 1000.0f)

HAL_StatusTypeDef imu_init(imu_t *imu, SPI_HandleTypeDef *hspi, float Kp, float Ki)
{
    HAL_StatusTypeDef status;

    if (imu == NULL || hspi == NULL) return HAL_ERROR;

    /* 保存SPI句柄 */
    imu->hspi = hspi;
    /* 初始化BMI088传感器 */
    status = bmi088_init(hspi);
    if (status != HAL_OK) return status;

    /* 初始化Mahony滤波器 */
    mahony_init(&imu->mahony, Kp, Ki);

    /* 记录初始化时刻的系统滴答（用于后续dt计算） */
    imu->last_tick = HAL_GetTick();
    imu->dt = 0.01f;  // 初始估计值，避免第一次dt为0

    /* 初始化欧拉角 */
    imu->roll = 0.0f;
    imu->pitch = 0.0f;
    imu->yaw = 0.0f;

    return HAL_OK;
}

HAL_StatusTypeDef imu_update(imu_t *imu)
{
    HAL_StatusTypeDef status;
    uint32_t now_tick;
    float dt;
    float accel_mg[3];   // 加速度原始值（mg）
    float gyro_dps[3];   // 陀螺仪原始值（°/s）
    float ax_g, ay_g, az_g;   // 加速度（g）
    float gx_rad, gy_rad, gz_rad; // 角速度（rad/s）

    if (imu == NULL) return HAL_ERROR;

    /* 计算时间间隔 */
    now_tick = HAL_GetTick();
    dt = MS_TO_SEC(now_tick - imu->last_tick);
    /* 限制dt范围，避免过大或过小 */
    if (dt > 0.1f) dt = 0.1f;
    if (dt < 0.0001f) dt = 0.0001f;
    imu->dt = dt;
    imu->last_tick = now_tick;

    /* 读取加速度计（mg） */
    status = bmi088_read_accel(imu->hspi, accel_mg);
    if (status != HAL_OK) return status;

    /* 读取陀螺仪（°/s） */
    status = bmi088_read_gyro(imu->hspi, gyro_dps);
    if (status != HAL_OK) return status;

    /* 单位转换 */
    // 加速度：mg -> g （1g = 1000mg）
    ax_g = accel_mg[0] / 1000.0f;
    ay_g = accel_mg[1] / 1000.0f;
    az_g = accel_mg[2] / 1000.0f;

    // 陀螺仪：°/s -> rad/s
    gx_rad = gyro_dps[0] * (float)M_PI / 180.0f;
    gy_rad = gyro_dps[1] * (float)M_PI / 180.0f;
    gz_rad = gyro_dps[2] * (float)M_PI / 180.0f;

    /* 更新Mahony滤波器 */
    mahony_update(&imu->mahony, gx_rad, gy_rad, gz_rad,
                  ax_g, ay_g, az_g, dt);

    /* 从四元数计算欧拉角（度） */
    mahony_compute_euler(&imu->mahony);

    /* 保存当前欧拉角到IMU结构体 */
    imu->roll  = imu->mahony.roll;
    imu->pitch = imu->mahony.pitch;
    imu->yaw   = imu->mahony.yaw;

    return HAL_OK;
}

void imu_get_euler(imu_t *imu, float *roll, float *pitch, float *yaw)
{
    if (imu == NULL) return;
    *roll  = imu->roll;
    *pitch = imu->pitch;
    *yaw   = imu->yaw;
}