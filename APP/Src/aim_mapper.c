#include "aim_mapper.h"

#include <stddef.h>

static float AimMapper_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float AimMapper_Absolute(float value)
{
    if (value < 0.0f) {
        return -value;
    }
    return value;
}

static float AimMapper_LimitSymmetric(float value, float max_abs_value)
{
    if (max_abs_value < 0.0f) {
        max_abs_value = -max_abs_value;
    }

    return AimMapper_Clamp(value, -max_abs_value, max_abs_value);
}

void AimMapper_LoadDefaultConfig(AimMapper_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->image_width = 640U;
    config->image_height = 480U;
    config->fov_horizontal = 60.0f;
    config->fov_vertical = 45.0f;
    config->pixel_to_angle_x = 0.0f;
    config->pixel_to_angle_y = 0.0f;
    config->max_pitch_angle = 30.0f;
    config->max_roll_angle = 30.0f;
}

void AimMapper_Init(AimMapper_t *mapper, const AimMapper_Config_t *config)
{
    if ((mapper == NULL) || (config == NULL)) {
        return;
    }

    mapper->config = *config;
    mapper->config.pixel_to_angle_x = 0.0f;
    mapper->config.pixel_to_angle_y = 0.0f;

    if (mapper->config.image_width > 0U) {
        mapper->config.pixel_to_angle_x = mapper->config.fov_horizontal / (float)mapper->config.image_width;
    }

    if (mapper->config.image_height > 0U) {
        mapper->config.pixel_to_angle_y = mapper->config.fov_vertical / (float)mapper->config.image_height;
    }

    mapper->config.max_pitch_angle = AimMapper_Absolute(mapper->config.max_pitch_angle);
    mapper->config.max_roll_angle = AimMapper_Absolute(mapper->config.max_roll_angle);

    mapper->aim_command.target_detected = false;
    mapper->aim_command.pixel_error.x = 0;
    mapper->aim_command.pixel_error.y = 0;
    mapper->aim_command.pitch_deg = 0.0f;
    mapper->aim_command.roll_deg = 0.0f;
}

void AimMapper_Update(AimMapper_t *mapper, const GuidanceMeasurement_t *measurement)
{
    int32_t center_x;
    int32_t center_y;
    int32_t error_x;
    int32_t error_y;

    if ((mapper == NULL) || (measurement == NULL)) {
        return;
    }

    if ((measurement->x == GUIDANCE_NO_TARGET_COORDINATE) &&
        (measurement->y == GUIDANCE_NO_TARGET_COORDINATE)) {
        mapper->aim_command.target_detected = false;
        mapper->aim_command.pixel_error.x = 0;
        mapper->aim_command.pixel_error.y = 0;
        mapper->aim_command.pitch_deg = 0.0f;
        mapper->aim_command.roll_deg = 0.0f;
        return;
    }

    center_x = (int32_t)mapper->config.image_width / 2;
    center_y = (int32_t)mapper->config.image_height / 2;
    error_x = (int32_t)measurement->x - center_x;
    error_y = center_y - (int32_t)measurement->y;

    mapper->aim_command.pixel_error.x = (int16_t)error_x;
    mapper->aim_command.pixel_error.y = (int16_t)error_y;
    mapper->aim_command.roll_deg = (float)mapper->aim_command.pixel_error.x * mapper->config.pixel_to_angle_x;
    mapper->aim_command.pitch_deg = (float)mapper->aim_command.pixel_error.y * mapper->config.pixel_to_angle_y;
    mapper->aim_command.roll_deg = AimMapper_LimitSymmetric(mapper->aim_command.roll_deg,
                                                            mapper->config.max_roll_angle);
    mapper->aim_command.pitch_deg = AimMapper_LimitSymmetric(mapper->aim_command.pitch_deg,
                                                             mapper->config.max_pitch_angle);
    mapper->aim_command.target_detected = true;
}
