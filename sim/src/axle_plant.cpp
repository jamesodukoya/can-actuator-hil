#include "sbw/axle_plant.hpp"

#include <algorithm>
#include <cmath>

namespace sbw {
namespace {

// Estimates for simulation only; to be replaced with measured values when the real axle is identified.
constexpr double kMotorResistanceOhm = 3.0;         // 12 V / 4 A stall current assumed for a 520-size motor
constexpr double kMotorInductanceHenry = 1.5e-3;    // typical for a 520-size brushed motor
constexpr double kTorqueConstantNmPerA = 0.0115;    // no-load 1037 rad/s at 12 V gives k_e of about 0.0116
constexpr double kBackEmfVoltSecPerRad = 0.0115;    // equal to the torque constant in SI units
constexpr double kMotorInertiaKgM2 = 1.0e-6;        // low end for a 35 mm can rotor
constexpr double kMotorViscousNmSecPerRad = 6.7e-7;  // 0.7 of the 1.7 mN.m no-load friction at 1047 rad/s
constexpr double kMotorCoulombNm = 1.0e-3;          // the other 1.0 mN.m of the no-load friction
constexpr double kGearRatio = 135.0;                // gearbox 90 times linkage 45 mm / 30 mm (1.5)
constexpr double kKingpinInertiaKgM2 = 2.0e-4;      // two knuckles, 0.08 kg at 35 mm: 2 * 0.08 * 0.035^2
constexpr double kKingpinViscousNmSecPerRad = 1.0e-3;
constexpr double kLinkStiffnessNmPerRad = 200.0;    // full torque at 1.5 A (2.33 N.m) deflects 11.6 mrad
constexpr double kLinkDampingNmSecPerRad = 0.08;    // damping ratio 0.2: 2 * 0.2 * sqrt(200 * 2e-4)
constexpr double kBearingFrictionNm = 0.005;        // bearings and printed surfaces
constexpr double kNoScrubNm = 0.0;                  // on the stands the wheels do not touch the ground
constexpr double kFloorScrubStaticNm = 0.65;        // 2 wheels * 0.8 * 17 N * 0.024 m, about 0.33 N.m each
constexpr double kFloorScrubCoulombNm = 0.50;       // about 0.77 times the static level
constexpr double kStribeckSpeedRadPerSec = 0.05;
constexpr double kBusVoltageV = 12.0;               // 3S pack, nominal rounded

}  // namespace

double playHalfWidthRadFromSlotClearanceMm(double slotClearanceMm) {
    return 0.5 * slotClearanceMm / kSteeringArmLengthMm;
}

AxlePlantParams standsPlantParams(double slotClearanceMm) {
    return {.motorResistanceOhm = kMotorResistanceOhm,
            .motorInductanceHenry = kMotorInductanceHenry,
            .torqueConstantNmPerA = kTorqueConstantNmPerA,
            .backEmfVoltSecPerRad = kBackEmfVoltSecPerRad,
            .motorInertiaKgM2 = kMotorInertiaKgM2,
            .motorViscousNmSecPerRad = kMotorViscousNmSecPerRad,
            .motorCoulombNm = kMotorCoulombNm,
            .gearRatio = kGearRatio,
            .kingpinInertiaKgM2 = kKingpinInertiaKgM2,
            .kingpinViscousNmSecPerRad = kKingpinViscousNmSecPerRad,
            .linkStiffnessNmPerRad = kLinkStiffnessNmPerRad,
            .linkDampingNmSecPerRad = kLinkDampingNmSecPerRad,
            .playHalfWidthRad = playHalfWidthRadFromSlotClearanceMm(slotClearanceMm),
            .bearingFrictionNm = kBearingFrictionNm,
            .scrubStaticNm = kNoScrubNm,
            .scrubCoulombNm = kNoScrubNm,
            .stribeckSpeedRadPerSec = kStribeckSpeedRadPerSec,
            .busVoltageV = kBusVoltageV};
}

AxlePlantParams floorPlantParams(double slotClearanceMm) {
    AxlePlantParams params = standsPlantParams(slotClearanceMm);
    params.scrubStaticNm = kFloorScrubStaticNm;
    params.scrubCoulombNm = kFloorScrubCoulombNm;
    return params;
}

double stepSpeedWithCoulombFriction(const FrictionStepInput& input) {
    const double speedChange = input.accelerationWithoutFrictionRadPerSec2 * kPlantStepSec;
    const double trialSpeed = input.speedRadPerSec + speedChange;
    const double maxStaticImpulse = input.staticLevelNm * kPlantStepSec;
    if (std::fabs(input.inertiaKgM2 * trialSpeed) <= maxStaticImpulse) {
        return 0.0;  // friction can stop it within this step: stick
    }
    const double frictionSpeedLoss = input.kineticLevelNm * kPlantStepSec / input.inertiaKgM2;
    return trialSpeed - std::copysign(frictionSpeedLoss, trialSpeed);
}

double linkTorqueNm(double deflectionRad, double deflectionRateRadPerSec, const AxlePlantParams& params) {
    // No torque passes while the crank pin is inside the slot clearance. Beyond it the printed parts act as
    // a spring and damper on the part of the deflection past the clearance.
    const double dampingTorqueNm = params.linkDampingNmSecPerRad * deflectionRateRadPerSec;
    if (deflectionRad > params.playHalfWidthRad) {
        return params.linkStiffnessNmPerRad * (deflectionRad - params.playHalfWidthRad) + dampingTorqueNm;
    }
    if (deflectionRad < -params.playHalfWidthRad) {
        return params.linkStiffnessNmPerRad * (deflectionRad + params.playHalfWidthRad) + dampingTorqueNm;
    }
    return 0.0;
}

double kingpinStaticFrictionNm(const AxlePlantParams& params) {
    return params.bearingFrictionNm + params.scrubStaticNm;
}

double kingpinKineticFrictionNm(double kingpinSpeedRadPerSec, const AxlePlantParams& params) {
    // Stribeck curve: friction is highest at rest and falls to the Coulomb level as the speed grows past
    // the Stribeck speed.
    const double speedRatio = kingpinSpeedRadPerSec / params.stribeckSpeedRadPerSec;
    const double stribeckExcessNm =
        (params.scrubStaticNm - params.scrubCoulombNm) * std::exp(-speedRatio * speedRatio);
    return params.bearingFrictionNm + params.scrubCoulombNm + stribeckExcessNm;
}

AxlePlant::AxlePlant(const AxlePlantParams& params) : params_(params) {}

void AxlePlant::step(double duty, bool isBridgeEnabled) {
    stepCurrent(duty, isBridgeEnabled);
    const double deflectionRad = motorAngleRad_ / params_.gearRatio - kingpinAngleRad_;
    const double deflectionRate = motorSpeedRadPerSec_ / params_.gearRatio - kingpinSpeedRadPerSec_;
    const double linkTorque = linkTorqueNm(deflectionRad, deflectionRate, params_);
    stepMotor(linkTorque);
    stepKingpin(linkTorque);
    // Semi-implicit Euler: positions advance with the new speeds.
    motorAngleRad_ += motorSpeedRadPerSec_ * kPlantStepSec;
    kingpinAngleRad_ += kingpinSpeedRadPerSec_ * kPlantStepSec;
}

void AxlePlant::stepCurrent(double duty, bool isBridgeEnabled) {
    if (!isBridgeEnabled) {
        // Both half-bridges are high impedance, so the motor coasts. The real current decays through the
        // body diodes in about L*|i|/V_bus (at most 0.36 ms at 2.9 A); the model sets it to zero at once.
        motorCurrentA_ = 0.0;
        return;
    }
    const double appliedVoltageV = std::clamp(duty, -1.0, 1.0) * params_.busVoltageV;
    const double backEmfV = params_.backEmfVoltSecPerRad * motorSpeedRadPerSec_;
    const double stepOverInductance = kPlantStepSec / params_.motorInductanceHenry;
    // Backward Euler on L di/dt = v - R i - k_e w: stable for any step, exact at steady state.
    motorCurrentA_ = (motorCurrentA_ + stepOverInductance * (appliedVoltageV - backEmfV)) /
                     (1.0 + stepOverInductance * params_.motorResistanceOhm);
}

void AxlePlant::stepMotor(double linkTorque) {
    const double driveTorqueNm = params_.torqueConstantNmPerA * motorCurrentA_ -
                                 params_.motorViscousNmSecPerRad * motorSpeedRadPerSec_ -
                                 linkTorque / params_.gearRatio;
    const double accelerationRadPerSec2 = driveTorqueNm / params_.motorInertiaKgM2;
    motorSpeedRadPerSec_ = stepSpeedWithCoulombFriction({
        .speedRadPerSec = motorSpeedRadPerSec_,
        .accelerationWithoutFrictionRadPerSec2 = accelerationRadPerSec2,
        .inertiaKgM2 = params_.motorInertiaKgM2,
        .staticLevelNm = params_.motorCoulombNm,
        .kineticLevelNm = params_.motorCoulombNm,
    });
}

void AxlePlant::stepKingpin(double linkTorque) {
    if (isJammed_) {
        kingpinSpeedRadPerSec_ = 0.0;  // an obstacle against the tyre holds the kingpin rigidly
        return;
    }
    const double driveTorqueNm = linkTorque - params_.kingpinViscousNmSecPerRad * kingpinSpeedRadPerSec_;
    // The kinetic level uses the speed before this step's update.
    kingpinSpeedRadPerSec_ = stepSpeedWithCoulombFriction({
        .speedRadPerSec = kingpinSpeedRadPerSec_,
        .accelerationWithoutFrictionRadPerSec2 = driveTorqueNm / params_.kingpinInertiaKgM2,
        .inertiaKgM2 = params_.kingpinInertiaKgM2,
        .staticLevelNm = kingpinStaticFrictionNm(params_),
        .kineticLevelNm = kingpinKineticFrictionNm(kingpinSpeedRadPerSec_, params_),
    });
}

void AxlePlant::setJammed(bool isJammed) { isJammed_ = isJammed; }

void AxlePlant::resetPose(double motorAngleRad, double kingpinAngleRad) {
    motorAngleRad_ = motorAngleRad;
    kingpinAngleRad_ = kingpinAngleRad;
    motorSpeedRadPerSec_ = 0.0;
    kingpinSpeedRadPerSec_ = 0.0;
    motorCurrentA_ = 0.0;
}

double AxlePlant::motorAngleRad() const { return motorAngleRad_; }
double AxlePlant::motorSpeedRadPerSec() const { return motorSpeedRadPerSec_; }
double AxlePlant::kingpinAngleRad() const { return kingpinAngleRad_; }
double AxlePlant::kingpinSpeedRadPerSec() const { return kingpinSpeedRadPerSec_; }
double AxlePlant::motorCurrentA() const { return motorCurrentA_; }
bool AxlePlant::isJammed() const { return isJammed_; }
const AxlePlantParams& AxlePlant::params() const { return params_; }

}  // namespace sbw
