#pragma once
#include <cstdint>

#include "sbw/axle_config.hpp"

namespace sbw {

// Debounced check that the motor encoder and the kingpin sensor still tell the same story. It only reports;
// the ECU decides whether a confirmed disagreement latches a fault, because that is only allowed while
// torque is enabled.
class DisagreementMonitor {
 public:
    explicit DisagreementMonitor(const MonitorConfig& config);

    bool update(float motorAngleAtKingpinRad, float kingpinAngleRad);  // true once confirmed
    bool isConditionActive() const;  // the difference exceeds the limit in the latest sample
    void resetConfirmation();

 private:
    MonitorConfig config_;
    bool isConditionActive_ = false;
    std::uint32_t confirmCount_ = 0;
};

}  // namespace sbw
