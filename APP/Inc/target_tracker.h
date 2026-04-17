#ifndef __TARGET_TRACKER_H
#define __TARGET_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#define TARGET_TRACKER_NO_TARGET 0xFFFFU

typedef struct {
    uint16_t image_width;
    uint16_t image_height;
    float fov_horizontal;
    float fov_vertical;

    float pixel_to_angle_x;
    float pixel_to_angle_y;

    float servo_center_angle;

    float max_pitch_angle;
    float max_roll_angle;
} TargetTracker_Config_t;

typedef struct {
    TargetTracker_Config_t config;

    bool target_detected;
    int16_t pixel_error_x;
    int16_t pixel_error_y;
    float angle_pitch;
    float angle_roll;
} TargetTracker_t;

void TargetTracker_Init(TargetTracker_t *tracker, const TargetTracker_Config_t *config);
void TargetTracker_Update(TargetTracker_t *tracker, uint16_t target_x, uint16_t target_y);
void TargetTracker_GetServoAngles(const TargetTracker_t *tracker,
                                  float *servo1,
                                  float *servo2,
                                  float *servo3,
                                  float *servo4);

#endif
