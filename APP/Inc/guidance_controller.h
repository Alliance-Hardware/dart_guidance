#ifndef __GUIDANCE_CONTROLLER_H
#define __GUIDANCE_CONTROLLER_H

#include "aim_mapper.h"
#include "servo_mixer_x.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define GUIDANCE_CONTROLLER_LOOP_PERIOD_MS 10U

typedef struct
{
    AimMapper_Config_t aim_mapper_config;
    ServoMixerX_Config_t servo_mixer_config;
} GuidanceController_Config_t;

typedef struct
{
    AimMapper_t aim_mapper;
    ServoMixerX_Config_t servo_mixer_config;
    GuidanceMeasurement_t measurement;
    GuidanceServoAngles_t servo_angles;
} GuidanceController_t;

void GuidanceController_LoadDefaultConfig(GuidanceController_Config_t *config);
void GuidanceController_Init(GuidanceController_t *controller,
                             UART_HandleTypeDef *receiver_uart,
                             const GuidanceController_Config_t *config);
bool GuidanceController_FetchMeasurement(GuidanceController_t *controller);
void GuidanceController_Solve(GuidanceController_t *controller);
void GuidanceController_ApplyOutputs(const GuidanceController_t *controller);
void GuidanceController_LogState(const GuidanceController_t *controller);
void GuidanceController_RunOnce(GuidanceController_t *controller);
uint8_t GuidanceController_ToServoCommand(float angle);

#endif
