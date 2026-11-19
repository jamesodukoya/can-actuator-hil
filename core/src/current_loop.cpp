#include "sbw/current_loop.hpp"

#include <algorithm>
#include <cmath>

#include "sbw/types.hpp"

namespace sbw {

CurrentLoop::CurrentLoop(const CurrentLoopConfig& config) : config_(config) {}

// The runtime applies the returned duty one current tick later; this function does not model that delay.
CurrentLoopOutputs CurrentLoop::step(const CurrentLoopInputs& inputs) {
    const bool canDrive = inputs.isBridgeEnabled && inputs.busVoltageV >= kMinBusVoltageV;
    if (!canDrive) {
        // Also keeps a stale integrator from kicking the motor when the bridge comes back, and avoids
        // dividing by a zero bus voltage below.
        reset();
        return {.duty = 0.0f};
    }

    const float errorA = inputs.setpointA - inputs.measuredA;
    const float unsaturatedV = config_.proportionalVoltPerA * errorA + integratorV_;
    // The cap is below the full bus voltage so every PWM period keeps a brake phase for the current sense.
    const float maxCommandV = config_.maxDuty * inputs.busVoltageV;
    const float commandV = std::clamp(unsaturatedV, -maxCommandV, maxCommandV);

    // Stop integrating only while the error pushes the output further into saturation; integrating the other
    // way is what lets the loop leave saturation.
    const bool isBeyondLimit = std::fabs(unsaturatedV) > maxCommandV;
    const bool isErrorPushingFurther = (unsaturatedV > 0.0f) == (errorA > 0.0f);
    const bool isSaturatedFurther = isBeyondLimit && isErrorPushingFurther;
    if (!isSaturatedFurther) {
        integratorV_ += config_.integralVoltPerASec * errorA * kCurrentPeriodSec;
    }
    return {.duty = commandV / inputs.busVoltageV};
}

void CurrentLoop::reset() {
    integratorV_ = 0.0f;
}

float CurrentLoop::integratorV() const {
    return integratorV_;
}

}  // namespace sbw
