#pragma once
#include <cstdint>

namespace sbw {

struct AxleConfig {
    float gearRatio;                 // motor rad per kingpin rad
    std::int32_t motorCountsPerRev;  // encoder counts per motor revolution
    std::uint16_t as5600ZeroCount;   // raw count at kingpin angle 0 (0 in simulation)
    float playHalfWidthRad;          // the ECU's estimate of the play
    float torqueConstantNmPerA;
    float motorInertiaKgM2;
    float motorViscousNmSecPerRad;
    float steerLimitRad;          // 30 degrees
    float hardwareCurrentLimitA;  // the DRV8874 chops at about 2.9 A, so the software limit sits below it
};

struct ControllerGains {
    float proportionalAPerRad;
    float integralAPerRadSec;
    float derivativeASecPerRad;
    float integratorDeadbandRad;          // 2 AS5600 steps: below this error the integrator holds
    float motorMovingThresholdRadPerSec;  // at the kingpin
};

struct JamObserverConfig {
    float observerGain;  // per tick
    float jamTorqueThresholdNm;
    float stallSpeedRadPerSec;  // at the kingpin
    std::uint32_t confirmTicks;
};

struct MonitorConfig {
    float disagreementLimitRad;
    std::uint32_t disagreementConfirmTicks;
    std::uint32_t commandTimeoutTicks;
    std::uint32_t frameFaultLatchCount;
    // A stale AS5600 reading for this many ticks latches a fault: the sensor failed, not one bad read.
    std::uint32_t staleKingpinLatchTicks;
};

struct EcuConfig {
    AxleConfig axle;
    ControllerGains gains;
    JamObserverConfig observer;
    MonitorConfig monitors;
};

// The 10 kHz current loop's own configuration (used by CurrentLoop, not by SteeringEcu).
struct CurrentLoopConfig {
    float proportionalVoltPerA;  // L * 2*pi*300 Hz at L = 1.5 mH
    float integralVoltPerASec;   // R * 2*pi*300 Hz: the PI zero sits at R/L = 2000 rad/s
    float maxDuty;               // below 1 so the brake phase stays long enough to sample
};

// Estimates for simulation, to be replaced with values identified from measurements on the real axle.
// Designated initializers name every field next to its value, so a reordered struct fails to compile
// instead of silently swapping gains.
inline constexpr EcuConfig kNominalEcuConfig{
    .axle = {.gearRatio = 135.0f,
             .motorCountsPerRev = 44,
             .as5600ZeroCount = 0,
             .playHalfWidthRad = 0.0133333f,
             .torqueConstantNmPerA = 0.0115f,
             .motorInertiaKgM2 = 1.0e-6f,
             .motorViscousNmSecPerRad = 6.7e-7f,
             .steerLimitRad = 0.5236f,
             .hardwareCurrentLimitA = 2.5f},
    .gains = {.proportionalAPerRad = 19.0f,
              .integralAPerRadSec = 150.0f,
              .derivativeASecPerRad = 0.76f,
              .integratorDeadbandRad = 0.003068f,
              .motorMovingThresholdRadPerSec = 0.05f},
    .observer = {.observerGain = 0.3f,
                 .jamTorqueThresholdNm = 0.012f,
                 .stallSpeedRadPerSec = 0.3f,
                 .confirmTicks = 5u},
    .monitors = {.disagreementLimitRad = 0.05f,
                 .disagreementConfirmTicks = 10u,
                 .commandTimeoutTicks = 50u,
                 .frameFaultLatchCount = 3u,
                 .staleKingpinLatchTicks = 3u}};

inline constexpr CurrentLoopConfig kNominalCurrentLoopConfig{
    .proportionalVoltPerA = 2.83f, .integralVoltPerASec = 5655.0f, .maxDuty = 0.94f};

inline EcuConfig nominalEcuConfigWithPlay(float playHalfWidthRad) {
    EcuConfig config = kNominalEcuConfig;
    config.axle.playHalfWidthRad = playHalfWidthRad;
    return config;
}

}  // namespace sbw
