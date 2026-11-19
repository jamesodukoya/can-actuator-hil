#include "sbw/jam_observer.hpp"

#include <cmath>

#include "sbw/types.hpp"

namespace sbw {

JamObserver::JamObserver(const JamObserverConfig& config, const AxleConfig& axle)
    : config_(config), axle_(axle) {}

bool JamObserver::update(const ObserverInput& input) {
    // The current of the previous tick is what accelerated the motor into this tick's speed, so the law pairs
    // it with the speed change. The first call after a reset has no previous sample and only stores one.
    if (hasPreviousSample_) {
        const float speedChangeRadPerSec = input.motorSpeedRadPerSec - previousSpeedRadPerSec_;
        const float accelerationTorqueNm = axle_.motorInertiaKgM2 * speedChangeRadPerSec / kControlPeriodSec;
        const float driveTorqueNm = axle_.torqueConstantNmPerA * previousCurrentA_;
        const float frictionTorqueNm = axle_.motorViscousNmSecPerRad * previousSpeedRadPerSec_;
        const float measuredLoadNm = driveTorqueNm - frictionTorqueNm - accelerationTorqueNm;
        estimatedLoadTorqueNm_ += config_.observerGain * (measuredLoadNm - estimatedLoadTorqueNm_);
    }
    hasPreviousSample_ = true;
    previousCurrentA_ = input.motorCurrentA;
    previousSpeedRadPerSec_ = input.motorSpeedRadPerSec;

    // A jam can push the load either way (steering left or right), so the sign is ignored.
    const bool isLoadHigh = std::fabs(estimatedLoadTorqueNm_) > config_.jamTorqueThresholdNm;
    const bool isKingpinStalled = std::fabs(input.kingpinSpeedRadPerSec) < config_.stallSpeedRadPerSec;
    if (isLoadHigh && isKingpinStalled) {
        ++confirmCount_;
    } else {
        confirmCount_ = 0;
    }
    // The count keeps rising past the threshold; the ECU latches on the first true.
    return confirmCount_ >= config_.confirmTicks;
}

void JamObserver::resetConfirmation() {
    confirmCount_ = 0;
}

void JamObserver::reset() {
    hasPreviousSample_ = false;
    previousCurrentA_ = 0.0f;
    previousSpeedRadPerSec_ = 0.0f;
    estimatedLoadTorqueNm_ = 0.0f;
    confirmCount_ = 0;
}

float JamObserver::estimatedLoadTorqueNm() const {
    return estimatedLoadTorqueNm_;
}

}  // namespace sbw
