#include "guidance_controller.h"

#include "interface.h"
#include "servo.h"
#include <stddef.h>
#include <stdio.h>

void GuidanceController_LoadDefaultConfig(GuidanceController_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    AimMapper_LoadDefaultConfig(&config->aim_mapper_config);
    ServoMixerX_LoadDefaultConfig(&config->servo_mixer_config);
}

void GuidanceController_Init(GuidanceController_t *controller,
                             UART_HandleTypeDef *receiver_uart,
                             const GuidanceController_Config_t *config)
{
    if ((controller == NULL) || (receiver_uart == NULL) || (config == NULL)) {
        return;
    }

    controller->measurement.x = GUIDANCE_NO_TARGET_COORDINATE;
    controller->measurement.y = GUIDANCE_NO_TARGET_COORDINATE;
    controller->measurement.area = 0U;
    controller->servo_mixer_config = config->servo_mixer_config;

    AimMapper_Init(&controller->aim_mapper, &config->aim_mapper_config);
    ServoMixerX_FillCenter(&controller->servo_mixer_config, &controller->servo_angles);

    Servo_Init();
    uart_receiver_init(receiver_uart);
    uart_receiver_start();

    printf("Target tracking system initialized\r\n");
}

bool GuidanceController_FetchMeasurement(GuidanceController_t *controller)
{
    if (controller == NULL) {
        return false;
    }

    return uart_receiver_get_data(&controller->measurement.x,
                                  &controller->measurement.y,
                                  &controller->measurement.area);
}

void GuidanceController_Solve(GuidanceController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    AimMapper_Update(&controller->aim_mapper, &controller->measurement);

    if (controller->aim_mapper.aim_command.target_detected) {
        ServoMixerX_Mix(&controller->servo_mixer_config,
                        &controller->aim_mapper.aim_command,
                        &controller->servo_angles);
    } else {
        ServoMixerX_FillCenter(&controller->servo_mixer_config, &controller->servo_angles);
    }
}

void GuidanceController_ApplyOutputs(const GuidanceController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    Servo_SetAngle(SERVO1, GuidanceController_ToServoCommand(controller->servo_angles.values[0]));
    Servo_SetAngle(SERVO2, GuidanceController_ToServoCommand(controller->servo_angles.values[1]));
    Servo_SetAngle(SERVO3, GuidanceController_ToServoCommand(controller->servo_angles.values[2]));
    Servo_SetAngle(SERVO4, GuidanceController_ToServoCommand(controller->servo_angles.values[3]));
}

void GuidanceController_LogState(const GuidanceController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    if (controller->aim_mapper.aim_command.target_detected) {
        printf("Target:(%u,%u) Area:%u Pitch:%.1f Roll:%.1f S:[%.1f %.1f %.1f %.1f]\r\n",
               controller->measurement.x,
               controller->measurement.y,
               controller->measurement.area,
               controller->aim_mapper.aim_command.pitch_deg,
               controller->aim_mapper.aim_command.roll_deg,
               controller->servo_angles.values[0],
               controller->servo_angles.values[1],
               controller->servo_angles.values[2],
               controller->servo_angles.values[3]);
    } else {
        printf("Target lost, servos centered\r\n");
    }
}

void GuidanceController_RunOnce(GuidanceController_t *controller)
{
    if (!GuidanceController_FetchMeasurement(controller)) {
        return;
    }

    GuidanceController_Solve(controller);
    GuidanceController_ApplyOutputs(controller);
    GuidanceController_LogState(controller);
}

uint8_t GuidanceController_ToServoCommand(float angle)
{
    if (angle < 0.0f) {
        angle = 0.0f;
    }
    if (angle > 180.0f) {
        angle = 180.0f;
    }

    return (uint8_t)(angle + 0.5f);
}
