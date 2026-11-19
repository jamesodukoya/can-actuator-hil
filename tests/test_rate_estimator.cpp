#include <gtest/gtest.h>

#include "sbw/rate_estimator.hpp"

namespace {

using sbw::WindowedRateEstimator;

constexpr int kRampSamples = 21;
constexpr float kRampStepRad = 0.002f;  // per 1 ms tick: 2 rad/s
constexpr float kRampRateRadPerSec = 2.0f;
constexpr float kRampToleranceRadPerSec = 1e-3f;
constexpr int kFullWindowSamples = 9;  // 9 positions make 8 intervals
constexpr float kStepRad = 0.008f;     // over an 8 ms window: 1 rad/s
constexpr float kStepRateRadPerSec = 1.0f;
constexpr float kStepToleranceRadPerSec = 1e-4f;
constexpr int kTicksStepStaysInWindow = 8;

TEST(RateEstimator, FirstSampleGivesZero) {
    WindowedRateEstimator estimator;

    EXPECT_EQ(estimator.update(1.0f), 0.0f);
}

TEST(RateEstimator, RampGivesSlope) {
    WindowedRateEstimator estimator;

    estimator.update(0.0f);
    for (int sampleIndex = 1; sampleIndex < kRampSamples; ++sampleIndex) {
        const float rateRadPerSec = estimator.update(kRampStepRad * static_cast<float>(sampleIndex));
        EXPECT_NEAR(rateRadPerSec, kRampRateRadPerSec, kRampToleranceRadPerSec) << "sample " << sampleIndex;
    }
}

TEST(RateEstimator, ConstantGivesZero) {
    WindowedRateEstimator estimator;
    float lastRateRadPerSec = 1.0f;

    for (int sampleIndex = 0; sampleIndex < 20; ++sampleIndex) {
        lastRateRadPerSec = estimator.update(0.5f);
    }
    EXPECT_EQ(lastRateRadPerSec, 0.0f);
}

TEST(RateEstimator, StepIsSeenForExactlyEightTicks) {
    WindowedRateEstimator estimator;
    for (int sampleIndex = 0; sampleIndex < kFullWindowSamples; ++sampleIndex) {
        estimator.update(0.0f);
    }

    // The step enters the window and stays in it for 8 ticks, until the old zero has left.
    for (int tick = 0; tick < kTicksStepStaysInWindow; ++tick) {
        const float rateRadPerSec = estimator.update(kStepRad);
        EXPECT_NEAR(rateRadPerSec, kStepRateRadPerSec, kStepToleranceRadPerSec) << "tick " << tick;
    }
    EXPECT_NEAR(estimator.update(kStepRad), 0.0f, kStepToleranceRadPerSec);
}

TEST(RateEstimator, ResetForgetsHistory) {
    WindowedRateEstimator estimator;
    for (int sampleIndex = 0; sampleIndex < kRampSamples; ++sampleIndex) {
        estimator.update(kRampStepRad * static_cast<float>(sampleIndex));
    }

    estimator.reset();
    EXPECT_EQ(estimator.update(5.0f), 0.0f);
}

}  // namespace
