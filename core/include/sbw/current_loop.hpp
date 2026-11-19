#pragma once
#include "sbw/axle_config.hpp"

namespace sbw {

inline constexpr float kMinBusVoltageV = 4.5f;  // DRV8874 minimum supply (datasheet): below it, no drive

struct CurrentLoopInputs {
    float setpointA = 0.0f;
    float measuredA = 0.0f;  // decoded current-sense value
    float busVoltageV = 0.0f;
    bool isBridgeEnabled = false;
};

struct CurrentLoopOutputs {
    float duty = 0.0f;  // signed; |duty| <= maxDuty; the sign selects PH
};

// PI controller that turns a current setpoint into a bridge duty.
class CurrentLoop {
 public:
    explicit CurrentLoop(const CurrentLoopConfig& config);

    CurrentLoopOutputs step(const CurrentLoopInputs& inputs);  // once per 100 us current tick
    void reset();
    float integratorV() const;

 private:
    CurrentLoopConfig config_;
    float integratorV_ = 0.0f;
};

}  // namespace sbw
