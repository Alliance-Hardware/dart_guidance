#ifndef __AIM_MAPPER_H
#define __AIM_MAPPER_H

#include "guidance_types.h"
#include <stdint.h>

typedef struct
{
    uint16_t image_width;
    uint16_t image_height;
    float fov_horizontal;
    float fov_vertical;
    float pixel_to_angle_x;
    float pixel_to_angle_y;
    float max_pitch_angle;
    float max_roll_angle;
} AimMapper_Config_t;

typedef struct
{
    AimMapper_Config_t config;
    GuidanceAimCommand_t aim_command;
} AimMapper_t;

void AimMapper_LoadDefaultConfig(AimMapper_Config_t *config);
void AimMapper_Init(AimMapper_t *mapper, const AimMapper_Config_t *config);
void AimMapper_Update(AimMapper_t *mapper, const GuidanceMeasurement_t *measurement);

#endif
