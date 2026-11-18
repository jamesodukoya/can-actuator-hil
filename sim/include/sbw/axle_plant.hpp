#pragma once

namespace sbw {

// Physical constants of the steering axle. SI units; the motor angle is at the motor shaft and the
// kingpin angle is at the left kingpin.
struct AxlePlantParams {
    double motorResistanceOhm;
    double motorInductanceHenry;
    double torqueConstantNmPerA;
    double backEmfVoltSecPerRad;
    double motorInertiaKgM2;
    double motorViscousNmSecPerRad;
    double motorCoulombNm;
    double gearRatio;  // motor radians per kingpin radian
    double kingpinInertiaKgM2;
    double kingpinViscousNmSecPerRad;
    double linkStiffnessNmPerRad;
    double linkDampingNmSecPerRad;
    double playHalfWidthRad;
    double bearingFrictionNm;
    double scrubStaticNm;
    double scrubCoulombNm;
    double stribeckSpeedRadPerSec;
    double busVoltageV;
};

// 10 us: the winding's time constant L/R = 0.5 ms is 50 steps, and the fastest mechanical mode (the
// kingpin on the contact spring, about 1000 rad/s) advances 0.01 rad per step, far below the stability limit.
inline constexpr double kPlantStepSec = 1.0e-5;
inline constexpr int kPlantStepsPerCurrentTick = 10;  // one 100 us current-loop tick
inline constexpr double kSteeringArmLengthMm = 45.0;  // hardware/cad/rover_cad/params.py, SteeringLinkage

double playHalfWidthRadFromSlotClearanceMm(double slotClearanceMm);  // half the clearance, divided by the arm
AxlePlantParams standsPlantParams(double slotClearanceMm);
AxlePlantParams floorPlantParams(double slotClearanceMm);  // the stands parameters plus tyre scrub

struct FrictionStepInput {
    double speedRadPerSec;
    double accelerationWithoutFrictionRadPerSec2;
    double inertiaKgM2;
    double staticLevelNm;
    double kineticLevelNm;
};
// Speed after one plant step with Coulomb friction. Friction that can absorb the whole step's momentum
// stops the body at exactly zero instead of letting it reverse.
double stepSpeedWithCoulombFriction(const FrictionStepInput& input);
double linkTorqueNm(double deflectionRad, double deflectionRateRadPerSec, const AxlePlantParams& params);
double kingpinStaticFrictionNm(const AxlePlantParams& params);
double kingpinKineticFrictionNm(double kingpinSpeedRadPerSec, const AxlePlantParams& params);

class AxlePlant {
 public:
    explicit AxlePlant(const AxlePlantParams& params);
    void step(double duty, bool isBridgeEnabled);  // advances kPlantStepSec
    void setJammed(bool isJammed);
    void resetPose(double motorAngleRad, double kingpinAngleRad);  // speeds and current to 0
    double motorAngleRad() const;
    double motorSpeedRadPerSec() const;
    double kingpinAngleRad() const;
    double kingpinSpeedRadPerSec() const;
    double motorCurrentA() const;
    bool isJammed() const;
    const AxlePlantParams& params() const;

 private:
    void stepCurrent(double duty, bool isBridgeEnabled);
    void stepMotor(double linkTorqueNm);
    void stepKingpin(double linkTorqueNm);

    AxlePlantParams params_;
    double motorAngleRad_ = 0.0;
    double motorSpeedRadPerSec_ = 0.0;
    double kingpinAngleRad_ = 0.0;
    double kingpinSpeedRadPerSec_ = 0.0;
    double motorCurrentA_ = 0.0;
    bool isJammed_ = false;
};

}  // namespace sbw
