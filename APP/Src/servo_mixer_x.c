#include "servo_mixer_x.h"

#include <stddef.h>

static float ServoMixerX_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void ServoMixerX_LoadDefaultConfig(ServoMixerX_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->servo_center_angle = 90.0f;
    config->servo_min_angle = 0.0f;
    config->servo_max_angle = 180.0f;
}

void ServoMixerX_FillCenter(const ServoMixerX_Config_t *config, GuidanceServoAngles_t *servo_angles)
{
    uint8_t index;
    float center_angle;

    if ((config == NULL) || (servo_angles == NULL)) {
        return;
    }

    center_angle = ServoMixerX_Clamp(config->servo_center_angle,
                                     config->servo_min_angle,
                                     config->servo_max_angle);

    for (index = 0U; index < GUIDANCE_SERVO_COUNT; ++index) {
        servo_angles->values[index] = center_angle;
    }
}

void ServoMixerX_Mix(const ServoMixerX_Config_t *config,
                     const GuidanceAimCommand_t *aim_command,
                     GuidanceServoAngles_t *servo_angles)
{
    float center_angle;

    if ((config == NULL) || (aim_command == NULL) || (servo_angles == NULL)) {
        return;
    }

    center_angle = ServoMixerX_Clamp(config->servo_center_angle,
                                     config->servo_min_angle,
                                     config->servo_max_angle);

    servo_angles->values[0] = ServoMixerX_Clamp(center_angle + aim_command->pitch_deg + aim_command->roll_deg,
                                                config->servo_min_angle,
                                                config->servo_max_angle);
    servo_angles->values[1] = ServoMixerX_Clamp(center_angle + aim_command->pitch_deg - aim_command->roll_deg,
                                                config->servo_min_angle,
                                                config->servo_max_angle);
    servo_angles->values[2] = ServoMixerX_Clamp(center_angle - aim_command->pitch_deg - aim_command->roll_deg,
                                                config->servo_min_angle,
                                                config->servo_max_angle);
    servo_angles->values[3] = ServoMixerX_Clamp(center_angle - aim_command->pitch_deg + aim_command->roll_deg,
                                                config->servo_min_angle,
                                                config->servo_max_angle);
}
