#pragma once
#include "sbw/axle_config.hpp"
#include "sbw/types.hpp"

namespace sbw {

struct ControllerInput {
    ControlMode mode;
    float targetKingpinAngleRad;
    float motorAngleAtKingpinRad;        // theta_m / N
    float kingpinAngleRad;               // theta_k
    float motorSpeedAtKingpinRadPerSec;  // omega_m / N
    float kingpinSpeedRadPerSec;         // omega_k
    float currentLimitA;                 // from SBW_Command
};

// Steering law, all quantities at the kingpin and the current in amps:
//   command = clamp(Kp * error + integral - Kd * rate, -limit, +limit)
// MotorOnly measures error and rate on the motor shaft, KingpinOnly on the kingpin, and DualLoop takes the
// error from the kingpin and the damping rate from the motor shaft. The motor shaft still moves inside the
// linkage play, where the kingpin is still, so the damping keeps working while the wheel angle is the goal.
class SteeringController {
 public:
    SteeringController(const ControllerGains& gains, const AxleConfig& axle);
    float update(const ControllerInput& input);  // amps; 0 for Off and ClearFault
    void reset();                                // integrator to 0
    float integratorA() const;

 private:
    struct LoopSignals {
        float errorRad;
        float rateRadPerSec;
    };
    struct SaturationState {
        float unsaturatedA;
        float limitA;
    };

    LoopSignals selectSignals(const ControllerInput& input) const;
    bool shouldHoldIntegrator(const ControllerInput& input, float errorRad,
                              const SaturationState& saturation) const;

    ControllerGains gains_;
    AxleConfig axle_;
    float integratorA_ = 0.0f;
};

}  // namespace sbw
