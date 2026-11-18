#include <gtest/gtest.h>

#include "sbw/axle_plant.hpp"

namespace {

using sbw::AxlePlant;
using sbw::AxlePlantParams;

constexpr int kSpinUpSteps = 50000;  // 0.5 s, about 22 mechanical time constants

// The link never engages, so only the electrical and motor equations act.
AxlePlantParams motorOnlyParams() {
    AxlePlantParams params = sbw::standsPlantParams(1.2);
    params.playHalfWidthRad = 1000.0;
    return params;
}

void runSteps(AxlePlant& plant, int stepCount, double duty, bool isBridgeEnabled) {
    for (int step = 0; step < stepCount; ++step) {
        plant.step(duty, isBridgeEnabled);
    }
}

TEST(AxlePlantMotor, NoLoadSpeedMatchesAnalytic) {
    AxlePlant plant(motorOnlyParams());
    runSteps(plant, kSpinUpSteps, 1.0, true);
    // Steady state: (k_t V / R - tau_c) / (b + k_t k_e / R) = (0.046 - 0.001) / (6.7e-7 + 4.40833e-5).
    EXPECT_NEAR(plant.motorSpeedRadPerSec(), 1005.5117, 0.01);
}

TEST(AxlePlantMotor, SpeedRiseMatchesFirstOrderWithinOnePercent) {
    constexpr int kRiseSteps = 2230;  // 0.0223 s, one mechanical time constant
    constexpr double kFirstOrderSpeedRadPerSec = 634.86;  // 1005.5117 * (1 - exp(-0.0223 / 0.0223447))
    AxlePlant plant(motorOnlyParams());
    runSteps(plant, kRiseSteps, 1.0, true);
    EXPECT_NEAR(plant.motorSpeedRadPerSec(), kFirstOrderSpeedRadPerSec, 0.01 * kFirstOrderSpeedRadPerSec);
}

TEST(AxlePlantMotor, CurrentRisesWithElectricalTimeConstant) {
    constexpr int kElectricalTimeConstantSteps = 50;  // L / R = 0.5 ms
    AxlePlantParams params = motorOnlyParams();
    params.motorInertiaKgM2 = 1.0e3;  // rotor effectively locked
    AxlePlant plant(params);
    runSteps(plant, kElectricalTimeConstantSteps, 1.0, true);
    // Backward Euler gives 4 * (1 - 1.02^-50); the continuous solution would be 4 * (1 - e^-1) = 2.5285.
    EXPECT_NEAR(plant.motorCurrentA(), 2.5138885, 1e-6);
}

TEST(AxlePlantMotor, BridgeOffGivesZeroCurrentAndCoastsToRest) {
    constexpr int kCoastSteps = 100000;  // the model stops after 76,871 steps
    AxlePlant plant(motorOnlyParams());
    runSteps(plant, kSpinUpSteps, 1.0, true);
    runSteps(plant, kCoastSteps, 1.0, false);
    EXPECT_EQ(plant.motorCurrentA(), 0.0);
    EXPECT_EQ(plant.motorSpeedRadPerSec(), 0.0);
}

TEST(AxlePlantMotor, BrakeStopsFasterThanCoast) {
    constexpr int kSlowDownSteps = 5000;  // 0.05 s
    AxlePlant brakedPlant(motorOnlyParams());
    AxlePlant coastingPlant(motorOnlyParams());
    runSteps(brakedPlant, kSpinUpSteps, 1.0, true);
    runSteps(coastingPlant, kSpinUpSteps, 1.0, true);
    runSteps(brakedPlant, kSlowDownSteps, 0.0, true);  // bridge on with zero duty shorts the motor
    runSteps(coastingPlant, kSlowDownSteps, 0.0, false);
    EXPECT_LT(brakedPlant.motorSpeedRadPerSec(), 0.2 * coastingPlant.motorSpeedRadPerSec());
}

TEST(AxlePlantMotor, DutyIsClamped) {
    AxlePlant clampedPlant(motorOnlyParams());
    AxlePlant fullDutyPlant(motorOnlyParams());
    runSteps(clampedPlant, kSpinUpSteps, 2.0, true);
    runSteps(fullDutyPlant, kSpinUpSteps, 1.0, true);
    EXPECT_NEAR(clampedPlant.motorSpeedRadPerSec(), fullDutyPlant.motorSpeedRadPerSec(), 1e-9);
}

TEST(AxlePlantMotor, SlotClearanceToPlay) {
    EXPECT_NEAR(sbw::playHalfWidthRadFromSlotClearanceMm(1.2), 0.6 / 45.0, 1e-9);
    EXPECT_NEAR(sbw::playHalfWidthRadFromSlotClearanceMm(0.4), 0.2 / 45.0, 1e-9);
}

}  // namespace
