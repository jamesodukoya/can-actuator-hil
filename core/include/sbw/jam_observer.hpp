#pragma once
#include <cstdint>

#include "sbw/axle_config.hpp"

namespace sbw {

struct ObserverInput {
    float motorCurrentA;
    float motorSpeedRadPerSec;  // motor shaft, from WindowedRateEstimator
    float kingpinSpeedRadPerSec;
};

// Disturbance observer: estimates the torque the wheel pushes back on the motor from the measured current and
// speed, then declares a jam when that torque is high while the kingpin has stopped.
class JamObserver {
 public:
    JamObserver(const JamObserverConfig& config, const AxleConfig& axle);

    bool update(const ObserverInput& input);  // true once the jam condition held for confirmTicks
    void resetConfirmation();                 // on every mode entry
    void reset();                             // everything, including the estimate
    float estimatedLoadTorqueNm() const;

 private:
    JamObserverConfig config_;
    AxleConfig axle_;
    bool hasPreviousSample_ = false;
    float previousCurrentA_ = 0.0f;
    float previousSpeedRadPerSec_ = 0.0f;
    float estimatedLoadTorqueNm_ = 0.0f;
    std::uint32_t confirmCount_ = 0;
};

}  // namespace sbw
