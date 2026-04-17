#ifndef DART_GUIDANCE_PIXEL_SERVO_MAPPER_HPP
#define DART_GUIDANCE_PIXEL_SERVO_MAPPER_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace dart_guidance::guidance {

constexpr std::size_t kServoCount = 4;
constexpr std::uint16_t kNoTargetCoordinate = 0xFFFFu;

enum class LostTargetMode : std::uint8_t {
    HoldLast,
    ReturnCenter,
};

struct ImageSize {
    std::uint16_t width{640};
    std::uint16_t height{480};

    constexpr bool Valid() const
    {
        return (width > 0U) && (height > 0U);
    }
};

struct CameraModel {
    ImageSize image{};
    float horizontal_fov_deg{60.0f};
    float vertical_fov_deg{45.0f};

    constexpr float PixelToAngleX() const
    {
        return (image.width > 0U) ? (horizontal_fov_deg / static_cast<float>(image.width)) : 0.0f;
    }

    constexpr float PixelToAngleY() const
    {
        return (image.height > 0U) ? (vertical_fov_deg / static_cast<float>(image.height)) : 0.0f;
    }
};

struct AxisLimit {
    float pitch_max_deg{30.0f};
    float roll_max_deg{30.0f};
};

struct ServoLimit {
    float center_deg{90.0f};
    float min_deg{0.0f};
    float max_deg{180.0f};
};

struct MapperConfig {
    CameraModel camera{};
    AxisLimit axis_limit{};
    ServoLimit servo_limit{};
    LostTargetMode lost_target_mode{LostTargetMode::ReturnCenter};
};

struct TargetMeasurement {
    std::uint16_t x{kNoTargetCoordinate};
    std::uint16_t y{kNoTargetCoordinate};
    std::uint16_t area{0};
    std::uint32_t timestamp_ms{0};

    constexpr bool IsDetected() const
    {
        return (x != kNoTargetCoordinate) && (y != kNoTargetCoordinate);
    }

    static constexpr TargetMeasurement NoTarget()
    {
        return {};
    }
};

struct PixelError {
    std::int16_t x{0};
    std::int16_t y{0};
};

struct AimCommand {
    float pitch_deg{0.0f};
    float roll_deg{0.0f};
};

struct ServoAngles {
    std::array<float, kServoCount> values{0.0f, 0.0f, 0.0f, 0.0f};
};

struct MappingSnapshot {
    bool target_detected{false};
    TargetMeasurement measurement{};
    PixelError pixel_error{};
    AimCommand aim{};
    ServoAngles servos{};
};

class PixelServoMapper {
public:
    explicit PixelServoMapper(const MapperConfig &config = MapperConfig())
        : config_(NormalizeConfig(config))
    {
        Reset();
    }

    void Configure(const MapperConfig &config)
    {
        config_ = NormalizeConfig(config);
        Reset();
    }

    const MapperConfig &config() const
    {
        return config_;
    }

    void Reset()
    {
        snapshot_ = {};
        snapshot_.servos = BuildCenterAngles();
    }

    PixelError ComputePixelError(const TargetMeasurement &measurement) const
    {
        if (!measurement.IsDetected() || !config_.camera.image.Valid()) {
            return {};
        }

        const auto center_x = static_cast<std::int32_t>(config_.camera.image.width / 2U);
        const auto center_y = static_cast<std::int32_t>(config_.camera.image.height / 2U);

        PixelError error{};
        error.x = static_cast<std::int16_t>(static_cast<std::int32_t>(measurement.x) - center_x);
        error.y = static_cast<std::int16_t>(center_y - static_cast<std::int32_t>(measurement.y));
        return error;
    }

    AimCommand MapPixelError(const PixelError &error) const
    {
        AimCommand aim{};
        aim.roll_deg = Clamp(static_cast<float>(error.x) * config_.camera.PixelToAngleX(),
                             -config_.axis_limit.roll_max_deg,
                             config_.axis_limit.roll_max_deg);
        aim.pitch_deg = Clamp(static_cast<float>(error.y) * config_.camera.PixelToAngleY(),
                              -config_.axis_limit.pitch_max_deg,
                              config_.axis_limit.pitch_max_deg);
        return aim;
    }

    ServoAngles MixX(const AimCommand &aim) const
    {
        ServoAngles servos{};
        servos.values[0] = Clamp(config_.servo_limit.center_deg + aim.pitch_deg + aim.roll_deg,
                                 config_.servo_limit.min_deg,
                                 config_.servo_limit.max_deg);
        servos.values[1] = Clamp(config_.servo_limit.center_deg + aim.pitch_deg - aim.roll_deg,
                                 config_.servo_limit.min_deg,
                                 config_.servo_limit.max_deg);
        servos.values[2] = Clamp(config_.servo_limit.center_deg - aim.pitch_deg - aim.roll_deg,
                                 config_.servo_limit.min_deg,
                                 config_.servo_limit.max_deg);
        servos.values[3] = Clamp(config_.servo_limit.center_deg - aim.pitch_deg + aim.roll_deg,
                                 config_.servo_limit.min_deg,
                                 config_.servo_limit.max_deg);
        return servos;
    }

    MappingSnapshot Update(const TargetMeasurement &measurement)
    {
        snapshot_.measurement = measurement;
        snapshot_.target_detected = measurement.IsDetected();

        if (!snapshot_.target_detected) {
            snapshot_.pixel_error = {};
            snapshot_.aim = {};
            if (config_.lost_target_mode == LostTargetMode::ReturnCenter) {
                snapshot_.servos = BuildCenterAngles();
            }
            return snapshot_;
        }

        snapshot_.pixel_error = ComputePixelError(measurement);
        snapshot_.aim = MapPixelError(snapshot_.pixel_error);
        snapshot_.servos = MixX(snapshot_.aim);
        return snapshot_;
    }

    const MappingSnapshot &snapshot() const
    {
        return snapshot_;
    }

private:
    static constexpr float Clamp(float value, float min_value, float max_value)
    {
        return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
    }

    static constexpr float Absolute(float value)
    {
        return (value < 0.0f) ? -value : value;
    }

    static MapperConfig NormalizeConfig(MapperConfig config)
    {
        if (config.servo_limit.min_deg > config.servo_limit.max_deg) {
            const float temp = config.servo_limit.min_deg;
            config.servo_limit.min_deg = config.servo_limit.max_deg;
            config.servo_limit.max_deg = temp;
        }

        config.servo_limit.center_deg = Clamp(config.servo_limit.center_deg,
                                              config.servo_limit.min_deg,
                                              config.servo_limit.max_deg);
        config.axis_limit.pitch_max_deg = Absolute(config.axis_limit.pitch_max_deg);
        config.axis_limit.roll_max_deg = Absolute(config.axis_limit.roll_max_deg);
        return config;
    }

    ServoAngles BuildCenterAngles() const
    {
        ServoAngles servos{};
        servos.values.fill(config_.servo_limit.center_deg);
        return servos;
    }

    MapperConfig config_{};
    MappingSnapshot snapshot_{};
};

}  // namespace dart_guidance::guidance

#endif
