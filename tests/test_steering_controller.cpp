#include <gtest/gtest.h>

#include "sbw/steering_controller.hpp"

namespace {

using sbw::ControlMode;
using sbw::ControllerInput;
using sbw::SteeringController;

constexpr float kCommandLimitA = 1.5f;
constexpr float kToleranceA = 1e-5f;

struct Pose {
    float targetRad;
    float kingpinRad;
    float motorAtKingpinRad;
    float motorSpeedAtKingpinRadPerSec;
};

SteeringController nominalController() {
    return SteeringController(sbw::kNominalEcuConfig.gains, sbw::kNominalEcuConfig.axle);
}

// The kingpin speed is 0 unless a test sets it afterwards.
ControllerInput inputFor(ControlMode mode, const Pose& pose, float currentLimitA = kCommandLimitA) {
    return {.mode = mode,
            .targetKingpinAngleRad = pose.targetRad,
            .motorAngleAtKingpinRad = pose.motorAtKingpinRad,
            .kingpinAngleRad = pose.kingpinRad,
            .motorSpeedAtKingpinRadPerSec = pose.motorSpeedAtKingpinRadPerSec,
            .kingpinSpeedRadPerSec = 0.0f,
            .currentLimitA = currentLimitA};
}

TEST(SteeringController, OffAndClearFaultReturnZero) {
    SteeringController controller = nominalController();
    const Pose largeError = {1.0f, 0.0f, 0.0f, 0.0f};

    EXPECT_EQ(controller.update(inputFor(ControlMode::Off, largeError)), 0.0f);
    EXPECT_EQ(controller.update(inputFor(ControlMode::ClearFault, largeError)), 0.0f);
}

TEST(SteeringController, ProportionalOnFirstTick) {
    SteeringController controller = nominalController();
    // The motor is 0.02 rad from the kingpin, outside the 0.0133 rad play, so the integrator runs.
    const ControllerInput input = inputFor(ControlMode::DualLoop, {0.11f, 0.10f, 0.12f, 0.0f});

    EXPECT_NEAR(controller.update(input), 0.19f, kToleranceA);  // 19 * 0.01
    // The integrator then holds 150 * 0.01 * 0.001 = 0.0015 A.
    EXPECT_NEAR(controller.update(input), 0.1915f, kToleranceA);
}

TEST(SteeringController, DampingOpposesMotorSpeed) {
    SteeringController controller = nominalController();

    const float commandA = controller.update(inputFor(ControlMode::DualLoop, {0.1f, 0.1f, 0.1f, 0.1f}));

    EXPECT_NEAR(commandA, -0.076f, kToleranceA);  // 0.76 * 0.1
}

TEST(SteeringController, SaturatesAtCommandLimit) {
    SteeringController controller = nominalController();
    const ControllerInput input = inputFor(ControlMode::DualLoop, {0.2f, 0.0f, 0.0f, 0.0f});

    for (int tick = 0; tick < 5; ++tick) {
        EXPECT_EQ(controller.update(input), kCommandLimitA);
    }
    EXPECT_EQ(controller.integratorA(), 0.0f);
}

TEST(SteeringController, HardwareLimitCapsCommandLimit) {
    RecordProperty("requirements", "REQ-SAF-001");
    SteeringController controller = nominalController();
    constexpr float kAboveHardwareA = 3.0f;
    const ControllerInput input = inputFor(ControlMode::DualLoop, {0.2f, 0.0f, 0.0f, 0.0f}, kAboveHardwareA);

    const float commandA = controller.update(input);

    EXPECT_EQ(commandA, sbw::kNominalEcuConfig.axle.hardwareCurrentLimitA);
}

TEST(SteeringController, NegativeLimitMeansZero) {
    RecordProperty("requirements", "REQ-SAF-001");
    SteeringController controller = nominalController();
    constexpr float kNegativeLimitA = -1.0f;
    const ControllerInput input = inputFor(ControlMode::DualLoop, {0.2f, 0.0f, 0.0f, 0.0f}, kNegativeLimitA);

    const float commandA = controller.update(input);

    EXPECT_EQ(commandA, 0.0f);
}

TEST(SteeringController, TargetIsClampedToSteeringLimit) {
    RecordProperty("requirements", "REQ-SAF-004");
    SteeringController controller = nominalController();

    const float commandA = controller.update(inputFor(ControlMode::KingpinOnly, {1.0f, 0.5f, 0.5f, 0.0f}));

    EXPECT_NEAR(commandA, 0.4484f, 1e-4f);  // 19 * (0.5236 - 0.5)
}

TEST(SteeringController, DeadbandHoldsIntegrator) {
    SteeringController controller = nominalController();
    // 0.003 rad is inside the deadband of 0.003068 rad (2 AS5600 steps).
    const ControllerInput input = inputFor(ControlMode::DualLoop, {0.003f, 0.0f, 0.0f, 0.0f});

    for (int tick = 0; tick < 10; ++tick) {
        controller.update(input);
    }
    EXPECT_EQ(controller.integratorA(), 0.0f);
}

TEST(SteeringController, CrossingPlayHoldsIntegratorInDualLoop) {
    constexpr float kMovingRadPerSec = 0.1f;  // above the 0.05 rad/s threshold
    SteeringController crossingController = nominalController();
    SteeringController standingController = nominalController();

    // The motor sits at the kingpin angle (inside the play) and moves: the kingpin error cannot change yet.
    crossingController.update(inputFor(ControlMode::DualLoop, {0.01f, 0.0f, 0.0f, kMovingRadPerSec}));
    EXPECT_EQ(crossingController.integratorA(), 0.0f);

    // The same pose with the motor standing still integrates: 150 * 0.01 * 0.001.
    standingController.update(inputFor(ControlMode::DualLoop, {0.01f, 0.0f, 0.0f, 0.0f}));
    EXPECT_NEAR(standingController.integratorA(), 0.0015f, kToleranceA);
}

TEST(SteeringController, MotorOnlyUsesMotorAngle) {
    SteeringController controller = nominalController();

    // The error comes from the motor angle (0.1 - 0.09), not from the kingpin angle (0.1 - 0.0).
    const float commandA = controller.update(inputFor(ControlMode::MotorOnly, {0.1f, 0.0f, 0.09f, 0.0f}));

    EXPECT_NEAR(commandA, 0.19f, kToleranceA);
}

TEST(SteeringController, KingpinOnlyDampsOnKingpinSpeed) {
    SteeringController controller = nominalController();
    ControllerInput input = inputFor(ControlMode::KingpinOnly, {0.1f, 0.1f, 0.1f, 5.0f});
    input.kingpinSpeedRadPerSec = 0.1f;

    // The motor speed of 5 rad/s is ignored in this mode.
    EXPECT_NEAR(controller.update(input), -0.076f, kToleranceA);
}

TEST(SteeringController, ResetClearsIntegrator) {
    SteeringController controller = nominalController();
    const ControllerInput input = inputFor(ControlMode::DualLoop, {0.11f, 0.10f, 0.12f, 0.0f});

    controller.update(input);
    controller.update(input);
    EXPECT_GT(controller.integratorA(), 0.0f);

    controller.reset();
    EXPECT_EQ(controller.integratorA(), 0.0f);
}

}  // namespace
