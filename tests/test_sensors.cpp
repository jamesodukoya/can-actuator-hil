#include <gtest/gtest.h>

#include "sbw/axle_plant.hpp"
#include "sbw/sensor_conversion.hpp"
#include "sbw/sensor_model.hpp"

namespace {

using sbw::AxlePlant;
using sbw::SensorModel;
using sbw::SensorModelConfig;

constexpr double kPi = 3.141592653589793;
constexpr double kAs5600LsbRad = 0.0015339808;  // 2 pi / 4096
constexpr double kAngleToleranceRad = 1e-5;
constexpr SensorModelConfig kZeroOffsetConfig = {44, 0};

struct Pose {
    double motorAngleRad;
    double kingpinAngleRad;
};

// Puts the plant at `pose` and lets the AS5600 filter reach it, as the runtime does at start-up.
void settleAt(AxlePlant& plant, SensorModel& sensors, const Pose& pose) {
    plant.resetPose(pose.motorAngleRad, pose.kingpinAngleRad);
    sensors.settleKingpinFilter(plant);
}

TEST(SensorConversion, MotorAngleFromCounts) {
    EXPECT_NEAR(sbw::motorAngleRad(44, 44), kPi * 2.0, kAngleToleranceRad);
    EXPECT_NEAR(sbw::motorAngleRad(-22, 44), -kPi, kAngleToleranceRad);
}

TEST(SensorConversion, KingpinWrapsAroundZero) {
    // One count below zero reads 4095 and one above reads 0 when the zero count is 4095: a small move
    // must stay small instead of looking like almost a full turn.
    EXPECT_NEAR(sbw::kingpinAngleRad(4095, 0), -kAs5600LsbRad, kAngleToleranceRad);
    EXPECT_NEAR(sbw::kingpinAngleRad(0, 4095), kAs5600LsbRad, kAngleToleranceRad);
    EXPECT_NEAR(sbw::kingpinAngleRad(2048, 0), -kPi, kAngleToleranceRad);
    EXPECT_NEAR(sbw::kingpinAngleRad(2047, 0), 2047 * kAs5600LsbRad, kAngleToleranceRad);
}

TEST(SensorModel, QuantizesWithFloor) {
    AxlePlant plant(sbw::standsPlantParams(1.2));
    SensorModel sensors(kZeroOffsetConfig);

    // 0.2 / 0.1428 = 1.40 -> 1 and 0.1 / 0.0015340 = 65.19 -> 65.
    settleAt(plant, sensors, {0.2, 0.1});
    EXPECT_EQ(sensors.readMotorCounts(plant), 1);
    EXPECT_EQ(sensors.readKingpinRaw(), 65);

    // -1.40 rounds down to -2, not up to -1; -65.19 rounds down to -66, which wraps to 4096 - 66.
    settleAt(plant, sensors, {-0.2, -0.1});
    EXPECT_EQ(sensors.readMotorCounts(plant), -2);
    EXPECT_EQ(sensors.readKingpinRaw(), 4030);
}

TEST(SensorModel, ConversionRoundTripIsWithinOneStep) {
    AxlePlant plant(sbw::standsPlantParams(1.2));
    SensorModel sensors(kZeroOffsetConfig);
    constexpr double kTrueAngleRad = 0.1;

    settleAt(plant, sensors, {0.0, kTrueAngleRad});
    const double convertedRad = static_cast<double>(sbw::kingpinAngleRad(sensors.readKingpinRaw(), 0));

    EXPECT_NEAR(convertedRad, 0.0997088, 1e-6);
    EXPECT_LE(convertedRad, kTrueAngleRad);
    EXPECT_GT(convertedRad, kTrueAngleRad - kAs5600LsbRad);
}

TEST(SensorModel, FreezeHoldsCounts) {
    AxlePlant plant(sbw::standsPlantParams(1.2));
    SensorModel sensors(kZeroOffsetConfig);

    plant.resetPose(0.2, 0.0);
    sensors.freezeMotorEncoder(plant);
    plant.resetPose(1.0, 0.0);
    EXPECT_EQ(sensors.readMotorCounts(plant), 1);

    sensors.clearFaults();
    EXPECT_EQ(sensors.readMotorCounts(plant), 7);  // 1.0 / 0.1428 = 7.003
}

TEST(SensorModel, KingpinOffsetShiftsRaw) {
    AxlePlant plant(sbw::standsPlantParams(1.2));
    SensorModel sensors(kZeroOffsetConfig);

    settleAt(plant, sensors, {0.0, 0.0});
    sensors.setKingpinOffsetMrad(100);
    EXPECT_EQ(sensors.readKingpinRaw(), 65);
}

TEST(SensorModel, ZeroCountShiftsRaw) {
    AxlePlant plant(sbw::standsPlantParams(1.2));
    SensorModel sensors({44, 100});

    settleAt(plant, sensors, {0.0, 0.0});
    EXPECT_EQ(sensors.readKingpinRaw(), 100);
    EXPECT_EQ(sbw::kingpinAngleRad(100, 100), 0.0f);
}

TEST(SensorModel, KingpinFilterFollowsTimeConstant) {
    AxlePlant plant(sbw::standsPlantParams(1.2));
    SensorModel sensors(kZeroOffsetConfig);
    constexpr int kStepsAfterFirstCheck = 71;

    settleAt(plant, sensors, {0.0, 0.0});
    plant.resetPose(0.0, 0.1);
    for (int step = 0; step < 29; ++step) {
        sensors.advance(plant);
    }
    // 0.1 * (1 - (1 - 0.0337838)^29): about 63 % of the 0.1 rad step after one 0.286 ms time constant.
    EXPECT_NEAR(sensors.kingpinFilteredRad(), 0.0630890, 1e-6);
    EXPECT_EQ(sensors.readKingpinRaw(), 41);

    for (int step = 0; step < kStepsAfterFirstCheck; ++step) {
        sensors.advance(plant);
    }
    EXPECT_NEAR(sensors.kingpinFilteredRad(), 0.0967831, 1e-6);
    EXPECT_EQ(sensors.readKingpinRaw(), 63);
}

TEST(SensorModel, FailingReadsAreReported) {
    SensorModel sensors(kZeroOffsetConfig);

    sensors.setKingpinReadsFailing(true);
    EXPECT_TRUE(sensors.isKingpinReadFailing());

    sensors.clearFaults();
    EXPECT_FALSE(sensors.isKingpinReadFailing());
}

}  // namespace
