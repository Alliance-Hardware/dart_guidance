#include "target_tracker.h"

static float TargetTracker_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float TargetTracker_LimitSymmetric(float value, float max_abs_value)
{
    if (max_abs_value < 0.0f) {
        max_abs_value = -max_abs_value;
    }

    return TargetTracker_Clamp(value, -max_abs_value, max_abs_value);
}

static float TargetTracker_Absolute(float value)
{
    if (value < 0.0f) {
        return -value;
    }

    return value;
}

void TargetTracker_Init(TargetTracker_t *tracker, const TargetTracker_Config_t *config)
{
    if ((tracker == 0) || (config == 0)) {
        return;
    }

    tracker->config = *config;
    tracker->config.pixel_to_angle_x = 0.0f;
    tracker->config.pixel_to_angle_y = 0.0f;

    if (tracker->config.image_width > 0U) {
        tracker->config.pixel_to_angle_x = tracker->config.fov_horizontal / (float)tracker->config.image_width;
    }

    if (tracker->config.image_height > 0U) {
        tracker->config.pixel_to_angle_y = tracker->config.fov_vertical / (float)tracker->config.image_height;
    }

    tracker->config.servo_center_angle = TargetTracker_Clamp(tracker->config.servo_center_angle, 0.0f, 180.0f);
    tracker->config.max_pitch_angle = TargetTracker_Absolute(tracker->config.max_pitch_angle);
    tracker->config.max_roll_angle = TargetTracker_Absolute(tracker->config.max_roll_angle);

    tracker->target_detected = false;
    tracker->pixel_error_x = 0;
    tracker->pixel_error_y = 0;
    tracker->angle_pitch = 0.0f;
    tracker->angle_roll = 0.0f;
}

void TargetTracker_Update(TargetTracker_t *tracker, uint16_t target_x, uint16_t target_y)
{
    int32_t center_x;
    int32_t center_y;
    int32_t error_x;
    int32_t error_y;

    if (tracker == 0) {
        return;
    }

    if ((target_x == TARGET_TRACKER_NO_TARGET) && (target_y == TARGET_TRACKER_NO_TARGET)) {
        tracker->target_detected = false;
        tracker->pixel_error_x = 0;
        tracker->pixel_error_y = 0;
        tracker->angle_pitch = 0.0f;
        tracker->angle_roll = 0.0f;
        return;
    }

    center_x = (int32_t)tracker->config.image_width / 2;
    center_y = (int32_t)tracker->config.image_height / 2;

    error_x = (int32_t)target_x - center_x;
    error_y = center_y - (int32_t)target_y;

    tracker->pixel_error_x = (int16_t)error_x;
    tracker->pixel_error_y = (int16_t)error_y;
    tracker->angle_roll = (float)tracker->pixel_error_x * tracker->config.pixel_to_angle_x;
    tracker->angle_pitch = (float)tracker->pixel_error_y * tracker->config.pixel_to_angle_y;

    tracker->angle_roll = TargetTracker_LimitSymmetric(tracker->angle_roll, tracker->config.max_roll_angle);
    tracker->angle_pitch = TargetTracker_LimitSymmetric(tracker->angle_pitch, tracker->config.max_pitch_angle);
    tracker->target_detected = true;
}

void TargetTracker_GetServoAngles(const TargetTracker_t *tracker,
                                  float *servo1,
                                  float *servo2,
                                  float *servo3,
                                  float *servo4)
{
    float center_angle;
    float servo_angle_1;
    float servo_angle_2;
    float servo_angle_3;
    float servo_angle_4;

    if (tracker == 0) {
        return;
    }

    center_angle = tracker->config.servo_center_angle;

    servo_angle_1 = center_angle + tracker->angle_pitch + tracker->angle_roll;
    servo_angle_2 = center_angle + tracker->angle_pitch - tracker->angle_roll;
    servo_angle_3 = center_angle - tracker->angle_pitch - tracker->angle_roll;
    servo_angle_4 = center_angle - tracker->angle_pitch + tracker->angle_roll;

    servo_angle_1 = TargetTracker_Clamp(servo_angle_1, 0.0f, 180.0f);
    servo_angle_2 = TargetTracker_Clamp(servo_angle_2, 0.0f, 180.0f);
    servo_angle_3 = TargetTracker_Clamp(servo_angle_3, 0.0f, 180.0f);
    servo_angle_4 = TargetTracker_Clamp(servo_angle_4, 0.0f, 180.0f);

    if (servo1 != 0) {
        *servo1 = servo_angle_1;
    }
    if (servo2 != 0) {
        *servo2 = servo_angle_2;
    }
    if (servo3 != 0) {
        *servo3 = servo_angle_3;
    }
    if (servo4 != 0) {
        *servo4 = servo_angle_4;
    }
}
