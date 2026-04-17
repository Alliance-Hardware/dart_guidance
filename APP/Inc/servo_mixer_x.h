#ifndef __SERVO_MIXER_X_H
#define __SERVO_MIXER_X_H

#include "guidance_types.h"

typedef struct
{
    float servo_center_angle;
    float servo_min_angle;
    float servo_max_angle;
} ServoMixerX_Config_t;

void ServoMixerX_LoadDefaultConfig(ServoMixerX_Config_t *config);
void ServoMixerX_FillCenter(const ServoMixerX_Config_t *config, GuidanceServoAngles_t *servo_angles);
void ServoMixerX_Mix(const ServoMixerX_Config_t *config,
                     const GuidanceAimCommand_t *aim_command,
                     GuidanceServoAngles_t *servo_angles);

#endif
