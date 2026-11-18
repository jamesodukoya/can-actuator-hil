#include <gtest/gtest.h>

#include "sbw/axle_plant.hpp"

namespace {

using sbw::AxlePlant;
using sbw::AxlePlantParams;

// The 1.2 mm slot gives a half width of 0.6 / 45 = 0.013333 rad at the kingpin.
const AxlePlantParams kStandsParams = sbw::standsPlantParams(1.2);
const AxlePlantParams kFloorParams = sbw::floorPlantParams(1.2);

constexpr int kJamTestSteps = 100000;  // 1 s of plant time
constexpr double kStribeckSpeedRadPerSec = 0.05;

void runSteps(AxlePlant& plant, int stepCount, double duty) {
    for (int step = 0; step < stepCount; ++step) {
        plant.step(duty, true);
    }
}

TEST(AxlePlantLinkage, NoTorqueInsidePlay) {
    // 0.01 rad is inside the 0.013333 rad half width. Even a fast deflection rate must pass no torque.
    EXPECT_EQ(sbw::linkTorqueNm(0.01, 5.0, kStandsParams), 0.0);
}

TEST(AxlePlantLinkage, TorqueBeyondContact) {
    // 200 * (0.02 - 0.013333) + 0.08 * 0.1 = 1.341333 N*m, and the mirror image on the negative side.
    EXPECT_NEAR(sbw::linkTorqueNm(0.02, 0.1, kStandsParams), 1.341333, 1e-6);
    EXPECT_NEAR(sbw::linkTorqueNm(-0.02, -0.1, kStandsParams), -1.341333, 1e-6);
}

TEST(AxlePlantLinkage, FrictionLevels) {
    // Floor: static = 0.005 bearing + 0.65 scrub; kinetic falls from 0.655 to 0.505 along the Stribeck curve.
    EXPECT_NEAR(sbw::kingpinStaticFrictionNm(kFloorParams), 0.655, 1e-6);
    EXPECT_NEAR(sbw::kingpinKineticFrictionNm(0.0, kFloorParams), 0.655, 1e-6);
    EXPECT_NEAR(sbw::kingpinKineticFrictionNm(kStribeckSpeedRadPerSec, kFloorParams), 0.560182, 1e-6);
    EXPECT_NEAR(sbw::kingpinKineticFrictionNm(1.0, kFloorParams), 0.505, 1e-6);
    // Stands: only the bearing friction remains, at every speed.
    EXPECT_NEAR(sbw::kingpinStaticFrictionNm(kStandsParams), 0.005, 1e-6);
    EXPECT_NEAR(sbw::kingpinKineticFrictionNm(kStribeckSpeedRadPerSec, kStandsParams), 0.005, 1e-6);
}

TEST(AxlePlantLinkage, CoulombStepSticksAndSlides) {
    // A 0.4 N*m push is below the 0.5 N*m static level: the body stays at rest.
    EXPECT_EQ(sbw::stepSpeedWithCoulombFriction({0.0, 0.4, 1.0, 0.5, 0.5}), 0.0);
    // 1 N*m of kinetic friction on 1 kg*m^2 removes 1 * 1e-5 rad/s in one step.
    EXPECT_NEAR(sbw::stepSpeedWithCoulombFriction({1.0, 0.0, 1.0, 1.0, 1.0}), 0.99999, 1e-12);
    // Friction must stop a nearly stopped body, never reverse it.
    EXPECT_EQ(sbw::stepSpeedWithCoulombFriction({1.0e-6, 0.0, 1.0, 1.0, 1.0}), 0.0);
}

TEST(AxlePlantLinkage, KingpinStaysPutInsidePlay) {
    constexpr int kMaxSteps = 1000000;
    constexpr double kPlayFractionToUse = 0.9;
    constexpr double kLowDuty = 0.05;
    AxlePlant plant(kStandsParams);
    const double motorAngleLimitRad =
        kPlayFractionToUse * kStandsParams.playHalfWidthRad * kStandsParams.gearRatio;
    int stepCount = 0;
    while (plant.motorAngleRad() < motorAngleLimitRad && stepCount < kMaxSteps) {
        plant.step(kLowDuty, true);
        ++stepCount;
    }
    ASSERT_LT(stepCount, kMaxSteps) << "the motor never used up 90 % of the play";
    EXPECT_EQ(plant.kingpinAngleRad(), 0.0);
    EXPECT_EQ(plant.kingpinSpeedRadPerSec(), 0.0);
}

TEST(AxlePlantLinkage, JammedKingpinNeverMoves) {
    constexpr double kHighDuty = 0.3;
    AxlePlant plant(kStandsParams);
    plant.setJammed(true);
    runSteps(plant, kJamTestSteps, kHighDuty);
    EXPECT_EQ(plant.kingpinAngleRad(), 0.0);
}

TEST(AxlePlantLinkage, JammedLinkSettlesInsideStaticBand) {
    constexpr double kDuty = 0.1;
    // Stall current is 0.1 * 12 / 3 = 0.4 A, so the motor torque is 4.6 mN*m. The motor sticks wherever
    // the link torque is within gear ratio * (4.6 +- 1) mN*m = [0.486, 0.756] N*m, which is a deflection
    // beyond the play of 2.43 to 3.78 mrad.
    constexpr double kMinExcessRad = 0.0024;
    constexpr double kMaxExcessRad = 0.0038;
    AxlePlant plant(kStandsParams);
    plant.setJammed(true);
    runSteps(plant, kJamTestSteps, kDuty);
    const double deflectionRad = plant.motorAngleRad() / kStandsParams.gearRatio;
    const double excessBeyondPlayRad = deflectionRad - kStandsParams.playHalfWidthRad;
    EXPECT_GE(excessBeyondPlayRad, kMinExcessRad);
    EXPECT_LE(excessBeyondPlayRad, kMaxExcessRad);
    EXPECT_EQ(plant.motorSpeedRadPerSec(), 0.0);
}

}  // namespace
