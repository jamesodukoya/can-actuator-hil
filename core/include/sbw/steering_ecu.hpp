#pragma once
#include <cstdint>

#include "sbw/alive_counter.hpp"
#include "sbw/axle_config.hpp"
#include "sbw/can_protocol.hpp"
#include "sbw/disagreement_monitor.hpp"
#include "sbw/jam_observer.hpp"
#include "sbw/rate_estimator.hpp"
#include "sbw/steering_controller.hpp"
#include "sbw/steering_io.hpp"
#include "sbw/types.hpp"

namespace sbw {

inline constexpr std::uint32_t kStatusPeriodTicks = 5;      // 200 Hz
inline constexpr std::uint32_t kHeartbeatPeriodTicks = 10;  // 100 Hz

// The steering node's application layer, one call per 1 ms tick. It never calls a driver, a clock or a
// socket: sensor readings and received frames arrive as data in SteeringInputs, and the current command and
// frames to send leave in SteeringOutputs. The firmware and the host simulation call the same step().
class SteeringEcu {
 public:
    explicit SteeringEcu(const EcuConfig& config);

    SteeringOutputs step(const SteeringInputs& inputs);
    ControlMode activeMode() const;
    std::uint8_t latchedFaults() const;
    bool isBridgeEnabled() const;
    float estimatedLoadTorqueNm() const;

 private:
    struct MeasuredState {
        float motorAngleAtKingpinRad;
        float kingpinAngleRad;
        float motorSpeedRadPerSec;  // motor shaft
        float motorSpeedAtKingpinRadPerSec;
        float kingpinSpeedRadPerSec;
        float motorCurrentA;
        float busVoltageV;
    };

    void receiveFrames(const SteeringInputs& inputs);
    void receiveCommandFrame(const CanFrame& frame);
    MeasuredState measure(const SensorReadings& readings);
    void latchModeIndependentFaults(bool hasScheduleOverrun);
    std::uint16_t kingpinRawToUse(const SensorReadings& readings);
    void applyNewCommand();
    void clearFaultsIfAllowed();
    void runMonitors(bool isJamDetected, bool isDisagreementConfirmed);
    void latch(std::uint8_t faultBit);
    void enterMode(ControlMode mode);
    ControllerInput controllerInputFrom(const MeasuredState& measured) const;
    void appendFrames(SteeringOutputs& outputs, const MeasuredState& measured, bool hasNewFault) const;

    EcuConfig config_;
    SteeringController controller_;
    JamObserver observer_;
    DisagreementMonitor disagreement_;
    AliveCounterChecker counterChecker_;
    WindowedRateEstimator motorRate_;
    WindowedRateEstimator kingpinRate_;
    ControlMode activeMode_ = ControlMode::Off;
    std::uint8_t latchedFaults_ = 0;
    CommandMessage latestCommand_{};
    bool hasNewCommand_ = false;
    std::uint32_t tickCount_ = 0;  // wraps after 49.7 days; unsigned differences stay correct
    std::uint32_t lastAcceptedTick_ = 0;
    std::uint32_t crcFailureStreak_ = 0;
    std::uint32_t counterFailureStreak_ = 0;
    std::uint16_t lastFreshKingpinRaw_ = 0;
    // Consecutive ticks with a failed kingpin read; stops at the latch limit.
    std::uint32_t staleKingpinStreak_ = 0;
    bool isMotorZeroed_ = false;
    float motorZeroOffsetRad_ = 0.0f;  // motor encoder angle at the first tick, aligned to the kingpin sensor
};

}  // namespace sbw
