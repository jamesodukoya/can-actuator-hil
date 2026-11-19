#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "sbw/axle_plant.hpp"
#include "sbw/current_loop.hpp"
#include "sbw/sensor_conversion.hpp"
#include "sbw/sensor_model.hpp"

namespace {

using sbw::CurrentLoop;
using sbw::CurrentLoopInputs;

constexpr float kToleranceDuty = 1e-6f;
constexpr float kBusVoltageV = 12.0f;
constexpr float kSaturatingSetpointA = 10.0f;

// Step-response checks against the plant.
constexpr double kStepTargetA = 1.0;
constexpr double kSettleBandA = 0.02;
constexpr double kNominalInductanceHenry = 1.5e-3;
constexpr double kLowInductanceHenry = 0.5e-3;
constexpr double kHighInductanceHenry = 4.0e-3;
constexpr double kNominalSettleLimitSec = 2.0e-3;
constexpr double kNominalOvershootLimit = 0.05;
constexpr double kRangeSettleLimitSec = 10.0e-3;
constexpr double kRangeOvershootLimit = 0.25;
constexpr double kSlotClearanceMm = 1.2;
constexpr int kTraceTicks = 100;  // 10 ms of 100 us current ticks
// Huge play keeps the link from ever engaging, and huge inertia keeps the motor from turning, so the
// current loop faces only the winding's resistance and inductance.
constexpr double kNeverEngagedPlayRad = 1000.0;
constexpr double kLockedMotorInertiaKgM2 = 1.0e3;

CurrentLoop nominalLoop() {
    return CurrentLoop(sbw::kNominalCurrentLoopConfig);
}

CurrentLoopInputs drivingInputs(float setpointA) {
    return {.setpointA = setpointA, .measuredA = 0.0f, .busVoltageV = kBusVoltageV, .isBridgeEnabled = true};
}

// Plant current after every plant step. Same order as the runtime: sample with the duty in force, step the
// loop, and the plant runs on the old duty until the next tick.
std::vector<double> currentStepTrace(double inductanceHenry) {
    sbw::AxlePlantParams params = sbw::standsPlantParams(kSlotClearanceMm);
    params.playHalfWidthRad = kNeverEngagedPlayRad;
    params.motorInertiaKgM2 = kLockedMotorInertiaKgM2;
    params.motorInductanceHenry = inductanceHenry;
    sbw::AxlePlant plant(params);
    CurrentLoop loop = nominalLoop();
    sbw::CurrentSenseDecoder decoder;
    double appliedDuty = 0.0;
    std::vector<double> trace;
    for (int tick = 0; tick < kTraceTicks; ++tick) {
        const sbw::CurrentSenseSample sample = sbw::currentSenseSampleFor(plant.motorCurrentA(), appliedDuty);
        const float measuredA = decoder.decode(sample, static_cast<float>(appliedDuty));
        const CurrentLoopInputs inputs = {.setpointA = static_cast<float>(kStepTargetA),
                                          .measuredA = measuredA,
                                          .busVoltageV = kBusVoltageV,
                                          .isBridgeEnabled = true};
        const float duty = loop.step(inputs).duty;
        for (int step = 0; step < sbw::kPlantStepsPerCurrentTick; ++step) {
            plant.step(appliedDuty, true);
            trace.push_back(plant.motorCurrentA());
        }
        appliedDuty = static_cast<double>(duty);
    }
    return trace;
}

// Time of the last sample outside the band. Infinite if the current is still outside it at the end of the
// trace, so a loop that never settles cannot pass by running out of samples.
double settleTimeSec(const std::vector<double>& traceA) {
    if (std::fabs(traceA.back() - kStepTargetA) > kSettleBandA) {
        return std::numeric_limits<double>::infinity();
    }
    std::size_t stepsUntilSettled = 0;
    for (std::size_t index = 0; index < traceA.size(); ++index) {
        if (std::fabs(traceA[index] - kStepTargetA) > kSettleBandA) {
            stepsUntilSettled = index + 1;  // traceA[index] is the current after plant step index + 1
        }
    }
    return static_cast<double>(stepsUntilSettled) * sbw::kPlantStepSec;
}

double overshootFraction(const std::vector<double>& traceA) {
    const double peakA = *std::max_element(traceA.begin(), traceA.end());
    return (peakA - kStepTargetA) / kStepTargetA;
}

TEST(CurrentLoop, FirstStepIsProportional) {
    CurrentLoop loop = nominalLoop();
    const CurrentLoopInputs inputs = drivingInputs(1.0f);

    EXPECT_NEAR(loop.step(inputs).duty, 0.2358333f, kToleranceDuty);  // 2.83 V/A * 1 A / 12 V
    EXPECT_NEAR(loop.integratorV(), 0.5655f, kToleranceDuty);         // 5655 * 1 A * 100 us
    EXPECT_NEAR(loop.step(inputs).duty, 0.2829583f, kToleranceDuty);  // (2.83 + 0.5655) V / 12 V
}

TEST(CurrentLoop, SaturatesAtMaxDutyAndHoldsIntegrator) {
    CurrentLoop loop = nominalLoop();
    const CurrentLoopInputs inputs = drivingInputs(kSaturatingSetpointA);

    for (int call = 0; call < 5; ++call) {
        EXPECT_NEAR(loop.step(inputs).duty, sbw::kNominalCurrentLoopConfig.maxDuty, kToleranceDuty);
    }
    EXPECT_EQ(loop.integratorV(), 0.0f);
}

TEST(CurrentLoop, NegativeSetpointGivesNegativeDuty) {
    CurrentLoop loop = nominalLoop();

    EXPECT_NEAR(loop.step(drivingInputs(-1.0f)).duty, -0.2358333f, kToleranceDuty);
}

TEST(CurrentLoop, DisabledBridgeGivesZeroAndResets) {
    RecordProperty("requirements", "REQ-SAF-001");
    CurrentLoop loop = nominalLoop();
    loop.step(drivingInputs(1.0f));
    loop.step(drivingInputs(1.0f));

    CurrentLoopInputs disabled = drivingInputs(1.0f);
    disabled.isBridgeEnabled = false;

    EXPECT_EQ(loop.step(disabled).duty, 0.0f);
    EXPECT_EQ(loop.integratorV(), 0.0f);
}

TEST(CurrentLoop, LowBusVoltageGivesZero) {
    CurrentLoop loop = nominalLoop();
    CurrentLoopInputs lowBus = drivingInputs(1.0f);
    lowBus.busVoltageV = 3.0f;  // below the 4.5 V minimum supply

    EXPECT_EQ(loop.step(lowBus).duty, 0.0f);
}

TEST(CurrentLoop, StepResponseAtNominalInductance) {
    RecordProperty("requirements", "REQ-CUR-001");
    const std::vector<double> trace = currentStepTrace(kNominalInductanceHenry);

    EXPECT_LE(settleTimeSec(trace), kNominalSettleLimitSec);
    EXPECT_LE(overshootFraction(trace), kNominalOvershootLimit);
}

TEST(CurrentLoop, StaysStableAcrossInductanceRange) {
    RecordProperty("requirements", "REQ-CUR-001");

    for (const double inductanceHenry : {kLowInductanceHenry, kHighInductanceHenry}) {
        const std::vector<double> trace = currentStepTrace(inductanceHenry);

        EXPECT_LE(settleTimeSec(trace), kRangeSettleLimitSec) << "inductance " << inductanceHenry << " H";
        EXPECT_LE(overshootFraction(trace), kRangeOvershootLimit) << "inductance " << inductanceHenry << " H";
    }
}

// reset() is called inside step() for a disabled bridge, but no other test calls it directly.
TEST(CurrentLoop, ResetClearsIntegrator) {
    CurrentLoop loop = nominalLoop();
    loop.step(drivingInputs(1.0f));
    ASSERT_NE(loop.integratorV(), 0.0f);

    loop.reset();

    EXPECT_EQ(loop.integratorV(), 0.0f);
}

}  // namespace
