#include "sbw/steering_ecu.hpp"

#include <algorithm>
#include <cstddef>

#include "sbw/sensor_conversion.hpp"

namespace sbw {

namespace {
constexpr float kMilliPerUnit = 1000.0f;
constexpr float kDeciPerUnit = 10.0f;
}  // namespace

SteeringEcu::SteeringEcu(const EcuConfig& config)
    : config_(config),
      controller_(config.gains, config.axle),
      observer_(config.observer, config.axle),
      disagreement_(config.monitors) {}

SteeringOutputs SteeringEcu::step(const SteeringInputs& inputs) {
    receiveFrames(inputs);
    const MeasuredState measured = measure(inputs.sensors);
    const ObserverInput observerInput = {.motorCurrentA = measured.motorCurrentA,
                                         .motorSpeedRadPerSec = measured.motorSpeedRadPerSec,
                                         .kingpinSpeedRadPerSec = measured.kingpinSpeedRadPerSec};
    const bool isJamDetected = observer_.update(observerInput);
    const bool isDisagreementConfirmed =
        disagreement_.update(measured.motorAngleAtKingpinRad, measured.kingpinAngleRad);

    const std::uint8_t faultsBefore = latchedFaults_;
    latchModeIndependentFaults(inputs.hasScheduleOverrun);
    applyNewCommand();
    runMonitors(isJamDetected, isDisagreementConfirmed);

    SteeringOutputs outputs;
    outputs.isBridgeEnabled = isBridgeEnabled();
    if (outputs.isBridgeEnabled) {
        outputs.currentCommandA = controller_.update(controllerInputFrom(measured));
    }
    const std::uint8_t newFaultBits =
        static_cast<std::uint8_t>(latchedFaults_ & static_cast<std::uint8_t>(~faultsBefore));
    appendFrames(outputs, measured, newFaultBits != 0u);
    ++tickCount_;
    return outputs;
}

ControlMode SteeringEcu::activeMode() const {
    return activeMode_;
}

std::uint8_t SteeringEcu::latchedFaults() const {
    return latchedFaults_;
}

bool SteeringEcu::isBridgeEnabled() const {
    return activeMode_ != ControlMode::Off && latchedFaults_ == 0;
}

float SteeringEcu::estimatedLoadTorqueNm() const {
    return observer_.estimatedLoadTorqueNm();
}

// Frames are only recorded here. Changing the mode between the frames of one tick would make the result
// depend on the order they arrived in; applyNewCommand changes the mode once, at a fixed point in the tick.
void SteeringEcu::receiveFrames(const SteeringInputs& inputs) {
    // A larger count is a runtime bug; ignoring the extra frames is safer than reading past the array.
    const std::size_t frameCount =
        std::min<std::size_t>(inputs.receivedFrameCount, kMaxReceivedFramesPerTick);
    for (std::size_t index = 0; index < frameCount; ++index) {
        receiveCommandFrame(inputs.receivedFrames[index]);
    }
}

void SteeringEcu::receiveCommandFrame(const CanFrame& frame) {
    if (frame.id != kCommandId) {
        return;
    }
    CommandMessage message;
    const CommandDecodeResult result = unpackCommand(frame, message);
    if (result == CommandDecodeResult::WrongLength || result == CommandDecodeResult::CrcMismatch) {
        ++crcFailureStreak_;
        return;
    }
    if (result != CommandDecodeResult::Ok) {
        return;  // InvalidMode: discarded, and it does not refresh the timeout
    }
    if (counterChecker_.check(message.aliveCounter) != CounterVerdict::Accepted) {
        ++counterFailureStreak_;
        return;
    }
    crcFailureStreak_ = 0;
    counterFailureStreak_ = 0;
    latestCommand_ = message;
    hasNewCommand_ = true;
    lastAcceptedTick_ = tickCount_;
}

SteeringEcu::MeasuredState SteeringEcu::measure(const SensorReadings& readings) {
    const float shaftAngleRad = motorAngleRad(readings.motorEncoderCounts, config_.axle.motorCountsPerRev);
    const float kingpinRad = kingpinAngleRad(kingpinRawToUse(readings), config_.axle.as5600ZeroCount);
    const float gearRatio = config_.axle.gearRatio;
    if (!isMotorZeroed_) {
        // The motor encoder is incremental, so it starts at an arbitrary angle; align it to the absolute
        // kingpin sensor once. In simulation both start at 0.
        motorZeroOffsetRad_ = shaftAngleRad - gearRatio * kingpinRad;
        isMotorZeroed_ = true;
    }
    const float relativeShaftAngleRad = shaftAngleRad - motorZeroOffsetRad_;
    const float motorSpeedRadPerSec = motorRate_.update(relativeShaftAngleRad);
    return {.motorAngleAtKingpinRad = relativeShaftAngleRad / gearRatio,
            .kingpinAngleRad = kingpinRad,
            .motorSpeedRadPerSec = motorSpeedRadPerSec,
            .motorSpeedAtKingpinRadPerSec = motorSpeedRadPerSec / gearRatio,
            .kingpinSpeedRadPerSec = kingpinRate_.update(kingpinRad),
            .motorCurrentA = readings.motorCurrentA,
            .busVoltageV = readings.busVoltageV};
}

// A failed read keeps the last good angle: the raw count of a failed read is old and must not be trusted. The
// streak is counted here, every tick and in every mode, so the clear guard sees it even while no mode is
// active. It stops at the latch limit so it cannot wrap around during a long sensor failure.
std::uint16_t SteeringEcu::kingpinRawToUse(const SensorReadings& readings) {
    if (readings.isKingpinReadingFresh) {
        lastFreshKingpinRaw_ = readings.as5600RawCount;
        staleKingpinStreak_ = 0;
    } else if (staleKingpinStreak_ < config_.monitors.staleKingpinLatchTicks) {
        ++staleKingpinStreak_;
    }
    return lastFreshKingpinRaw_;
}

// Active in every mode. Corrupted or repeated command frames mean the link is broken whatever the mode, and a
// schedule overrun means the ECU cannot keep its own timing whatever it is doing.
void SteeringEcu::latchModeIndependentFaults(bool hasScheduleOverrun) {
    if (crcFailureStreak_ >= config_.monitors.frameFaultLatchCount) {
        latch(fault_bits::kCrc);
    }
    if (counterFailureStreak_ >= config_.monitors.frameFaultLatchCount) {
        latch(fault_bits::kCounter);
    }
    if (hasScheduleOverrun) {
        latch(fault_bits::kInternal);
    }
}

void SteeringEcu::applyNewCommand() {
    if (!hasNewCommand_) {
        return;
    }
    hasNewCommand_ = false;
    // Safe cast: unpackCommand rejected every value above ClearFault.
    const ControlMode requested = static_cast<ControlMode>(latestCommand_.controlMode);
    if (requested == ControlMode::ClearFault) {
        clearFaultsIfAllowed();
        enterMode(ControlMode::Off);
        return;
    }
    if (latchedFaults_ != 0u) {
        return;  // stays Off until the faults are cleared
    }
    if (requested != activeMode_) {
        enterMode(requested);
    }
}

// A fault whose cause is still present must not clear, or a gateway that keeps sending ClearFault would turn
// torque back on with a broken sensor.
void SteeringEcu::clearFaultsIfAllowed() {
    const bool isConditionStillPresent = disagreement_.isConditionActive() || staleKingpinStreak_ > 0;
    if (!isConditionStillPresent) {
        latchedFaults_ = 0;
    }
}

// Silence is normal while Off (the gateway may not be running yet), and the observer, the monitor and the
// stale-read count run every tick, so none of them may latch a fault while Off.
void SteeringEcu::runMonitors(bool isJamDetected, bool isDisagreementConfirmed) {
    if (activeMode_ == ControlMode::Off) {
        return;
    }
    const std::uint32_t ticksSinceCommand = tickCount_ - lastAcceptedTick_;
    if (ticksSinceCommand > config_.monitors.commandTimeoutTicks) {
        latch(fault_bits::kTimeout);
    }
    if (staleKingpinStreak_ >= config_.monitors.staleKingpinLatchTicks) {
        latch(fault_bits::kInternal);
    }
    if (isDisagreementConfirmed) {
        latch(fault_bits::kDisagree);
    }
    if (isJamDetected) {
        latch(fault_bits::kJam);
    }
}

void SteeringEcu::latch(std::uint8_t faultBit) {
    latchedFaults_ = static_cast<std::uint8_t>(latchedFaults_ | faultBit);
    enterMode(ControlMode::Off);
}

// A new mode starts clean: an integrator or half-confirmed fault from the old mode must not carry over.
void SteeringEcu::enterMode(ControlMode mode) {
    activeMode_ = mode;
    controller_.reset();
    observer_.resetConfirmation();
    disagreement_.resetConfirmation();
}

ControllerInput SteeringEcu::controllerInputFrom(const MeasuredState& measured) const {
    return {.mode = activeMode_,
            .targetKingpinAngleRad = latestCommand_.targetKingpinAngleMrad / kMilliPerUnit,
            .motorAngleAtKingpinRad = measured.motorAngleAtKingpinRad,
            .kingpinAngleRad = measured.kingpinAngleRad,
            .motorSpeedAtKingpinRadPerSec = measured.motorSpeedAtKingpinRadPerSec,
            .kingpinSpeedRadPerSec = measured.kingpinSpeedRadPerSec,
            .currentLimitA = latestCommand_.currentLimitMilliAmp / kMilliPerUnit};
}

void SteeringEcu::appendFrames(SteeringOutputs& outputs, const MeasuredState& measured,
                               bool hasNewFault) const {
    const bool isStatusDue = tickCount_ % kStatusPeriodTicks == 0 || hasNewFault;
    const bool isHeartbeatDue = tickCount_ % kHeartbeatPeriodTicks == 0;
    if (isStatusDue) {
        const std::uint8_t modeBits = static_cast<std::uint8_t>(activeMode_);
        const StatusMessage status = {
            .kingpinAngleMrad = saturateToInt16(measured.kingpinAngleRad * kMilliPerUnit),
            .motorSpeedDeciRadPerSec = saturateToInt16(measured.motorSpeedRadPerSec * kDeciPerUnit),
            .motorCurrentMilliAmp = saturateToInt16(measured.motorCurrentA * kMilliPerUnit),
            .estLoadTorqueMilliNm = saturateToInt8(observer_.estimatedLoadTorqueNm() * kMilliPerUnit),
            .stateAndFaults = static_cast<std::uint8_t>(modeBits | latchedFaults_)};
        outputs.frames[outputs.frameCount] = packStatus(status);
        ++outputs.frameCount;
    }
    if (isHeartbeatDue) {
        const bool hasFault = latchedFaults_ != 0u;
        const HeartbeatMessage heartbeat = {
            .nodeState = static_cast<std::uint8_t>(hasFault ? NodeState::Fault : NodeState::Running),
            .uptimeMs = tickCount_,  // one tick is 1 ms
            .busVoltageMilliVolt = saturateToUint16(measured.busVoltageV * kMilliPerUnit)};
        outputs.frames[outputs.frameCount] = packHeartbeat(heartbeat);
        ++outputs.frameCount;
    }
}

}  // namespace sbw
