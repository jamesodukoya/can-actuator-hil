#include "sbw/steering_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sbw {

SteeringController::SteeringController(const ControllerGains& gains, const AxleConfig& axle)
    : gains_(gains), axle_(axle) {}

float SteeringController::update(const ControllerInput& input) {
    const bool isActiveMode = input.mode == ControlMode::MotorOnly ||
                              input.mode == ControlMode::KingpinOnly || input.mode == ControlMode::DualLoop;
    if (!isActiveMode) {
        return 0.0f;
    }
    const LoopSignals signals = selectSignals(input);
    // A negative or huge command limit can never widen the bound beyond what the hardware allows.
    const float limitA = std::min(std::max(input.currentLimitA, 0.0f), axle_.hardwareCurrentLimitA);
    const float unsaturatedA = gains_.proportionalAPerRad * signals.errorRad + integratorA_ -
                               gains_.derivativeASecPerRad * signals.rateRadPerSec;
    const float commandA = std::clamp(unsaturatedA, -limitA, limitA);
    // The integrator is updated after the output is computed, so the new term acts from the next tick.
    if (!shouldHoldIntegrator(input, signals.errorRad, {unsaturatedA, limitA})) {
        integratorA_ += gains_.integralAPerRadSec * signals.errorRad * kControlPeriodSec;
    }
    return commandA;
}

void SteeringController::reset() {
    integratorA_ = 0.0f;
}

float SteeringController::integratorA() const {
    return integratorA_;
}

SteeringController::LoopSignals SteeringController::selectSignals(const ControllerInput& input) const {
    const float steerLimitRad = axle_.steerLimitRad;
    const float targetRad = std::clamp(input.targetKingpinAngleRad, -steerLimitRad, steerLimitRad);
    switch (input.mode) {
        case ControlMode::MotorOnly:
            return {targetRad - input.motorAngleAtKingpinRad, input.motorSpeedAtKingpinRadPerSec};
        case ControlMode::KingpinOnly:
            return {targetRad - input.kingpinAngleRad, input.kingpinSpeedRadPerSec};
        case ControlMode::DualLoop:
            return {targetRad - input.kingpinAngleRad, input.motorSpeedAtKingpinRadPerSec};
        default:
            return {0.0f, 0.0f};
    }
}

// Anti-windup. Three cases where integrating the error only builds up a debt that overshoots later.
bool SteeringController::shouldHoldIntegrator(const ControllerInput& input, float errorRad,
                                              const SaturationState& saturation) const {
    // The output is at its limit and the error pushes it further into the limit.
    const bool isOutputBeyondLimit = std::fabs(saturation.unsaturatedA) > saturation.limitA;
    const bool isErrorPushingFurther = (saturation.unsaturatedA > 0.0f) == (errorRad > 0.0f);
    const bool isSaturatedFurther = isOutputBeyondLimit && isErrorPushingFurther;

    // The sensor cannot resolve an error this small, so integrating it only builds stick-slip.
    const bool isInsideDeadband = std::fabs(errorRad) <= gains_.integratorDeadbandRad;

    // While the motor crosses the play the kingpin error cannot change, so integrating it is the same trap.
    const float motorToKingpinGapRad = std::fabs(input.motorAngleAtKingpinRad - input.kingpinAngleRad);
    const bool isMotorInsidePlay = motorToKingpinGapRad <= axle_.playHalfWidthRad;
    const float motorSpeedRadPerSec = std::fabs(input.motorSpeedAtKingpinRadPerSec);
    const bool isMotorMoving = motorSpeedRadPerSec > gains_.motorMovingThresholdRadPerSec;
    const bool isCrossingPlay = input.mode == ControlMode::DualLoop && isMotorInsidePlay && isMotorMoving;

    return isSaturatedFurther || isInsideDeadband || isCrossingPlay;
}

}  // namespace sbw
