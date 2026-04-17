#ifndef __GUIDANCE_TYPES_H
#define __GUIDANCE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define GUIDANCE_NO_TARGET_COORDINATE 0xFFFFU
#define GUIDANCE_SERVO_COUNT 4U

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t area;
} GuidanceMeasurement_t;

typedef struct
{
    int16_t x;
    int16_t y;
} GuidancePixelError_t;

typedef struct
{
    bool target_detected;
    GuidancePixelError_t pixel_error;
    float pitch_deg;
    float roll_deg;
} GuidanceAimCommand_t;

typedef struct
{
    float values[GUIDANCE_SERVO_COUNT];
} GuidanceServoAngles_t;

#endif
