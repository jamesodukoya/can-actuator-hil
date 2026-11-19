#include <gtest/gtest.h>

#include "sbw/jam_observer.hpp"

namespace {

using sbw::JamObserver;
using sbw::ObserverInput;

constexpr float kToleranceNm = 1e-6f;
constexpr float kCruiseSpeedRadPerSec = 100.0f;
// With the nominal motor constants, this current at 100 rad/s leaves a 0.010 N·m load after friction.
constexpr float kCruiseCurrentA = 0.8753913f;
// k_t * i = 0.020 N·m: well above the 0.012 N·m jam threshold once the estimate has settled.
constexpr float kJamCurrentA = 1.7391304f;
constexpr float kMovingKingpinSpeedRadPerSec = 0.5f;
constexpr int kCallsBeforeConfirmation = 7;
constexpr int kMovingCalls = 50;

JamObserver nominalObserver() {
    return JamObserver(sbw::kNominalEcuConfig.observer, sbw::kNominalEcuConfig.axle);
}

// Motor and kingpin both stopped while the current pushes.
ObserverInput stalledInput(float currentA, float kingpinSpeedRadPerSec = 0.0f) {
    return {.motorCurrentA = currentA,
            .motorSpeedRadPerSec = 0.0f,
            .kingpinSpeedRadPerSec = kingpinSpeedRadPerSec};
}

TEST(JamObserver, ConvergesGeometricallyToConstantLoad) {
    JamObserver observer = nominalObserver();
    const ObserverInput input = {.motorCurrentA = kCruiseCurrentA,
                                 .motorSpeedRadPerSec = kCruiseSpeedRadPerSec,
                                 .kingpinSpeedRadPerSec = 0.0f};

    observer.update(input);  // only stores the sample
    for (int call = 0; call < 10; ++call) {
        observer.update(input);
    }

    // Each update closes 30 % of the gap to 0.010 N·m: 0.010 * (1 - 0.7^10).
    EXPECT_NEAR(observer.estimatedLoadTorqueNm(), 0.0097175f, kToleranceNm);
}

TEST(JamObserver, AccelerationIsNotLoad) {
    JamObserver observer = nominalObserver();

    observer.update({.motorCurrentA = 0.0f, .motorSpeedRadPerSec = 0.0f, .kingpinSpeedRadPerSec = 0.0f});
    observer.update({.motorCurrentA = 0.0f, .motorSpeedRadPerSec = 1.0f, .kingpinSpeedRadPerSec = 0.0f});

    // Speeding up by 1 rad/s in 1 ms needs J * 1000 = 0.001 N·m, so the measured load is -0.001 N·m,
    // and the estimate moves 30 % of the way there.
    EXPECT_NEAR(observer.estimatedLoadTorqueNm(), -0.0003f, kToleranceNm);
}

TEST(JamObserver, ConfirmsJamOnEighthCall) {
    RecordProperty("requirements", "REQ-JAM-001");
    JamObserver observer = nominalObserver();
    const ObserverInput input = stalledInput(kJamCurrentA);

    // Call 1 only stores. The estimate passes 0.012 N·m on call 4, and 5 consecutive calls confirm.
    for (int call = 1; call <= kCallsBeforeConfirmation; ++call) {
        EXPECT_FALSE(observer.update(input)) << "call " << call;
    }
    EXPECT_TRUE(observer.update(input));
}

TEST(JamObserver, MovingKingpinNeverConfirms) {
    RecordProperty("requirements", "REQ-JAM-002");
    JamObserver observer = nominalObserver();
    const ObserverInput input = stalledInput(kJamCurrentA, kMovingKingpinSpeedRadPerSec);

    for (int call = 1; call <= kMovingCalls; ++call) {
        EXPECT_FALSE(observer.update(input)) << "call " << call;
    }
}

TEST(JamObserver, ResetConfirmationRestartsCount) {
    JamObserver observer = nominalObserver();
    const ObserverInput input = stalledInput(kJamCurrentA);

    for (int call = 1; call <= kCallsBeforeConfirmation; ++call) {
        observer.update(input);  // the count is 4 after call 7
    }
    observer.resetConfirmation();

    // The estimate is kept, so counting starts again at 1 on call 8 and reaches 5 on call 12.
    for (int call = 8; call <= 11; ++call) {
        EXPECT_FALSE(observer.update(input)) << "call " << call;
    }
    EXPECT_TRUE(observer.update(input));
}

TEST(JamObserver, NegativeLoadAlsoConfirms) {
    JamObserver observer = nominalObserver();
    const ObserverInput input = stalledInput(-kJamCurrentA);

    for (int call = 1; call <= kCallsBeforeConfirmation; ++call) {
        EXPECT_FALSE(observer.update(input)) << "call " << call;
    }
    EXPECT_TRUE(observer.update(input));
}

// reset() is a public function that no other test calls.
TEST(JamObserver, ResetForgetsEstimateAndSamples) {
    JamObserver observer = nominalObserver();
    const ObserverInput input = stalledInput(kJamCurrentA);
    for (int call = 1; call <= kCallsBeforeConfirmation + 1; ++call) {
        observer.update(input);
    }

    observer.reset();

    EXPECT_EQ(observer.estimatedLoadTorqueNm(), 0.0f);
    // Same timing as a new observer: the first call only stores, and the jam confirms on the eighth.
    for (int call = 1; call <= kCallsBeforeConfirmation; ++call) {
        EXPECT_FALSE(observer.update(input)) << "call " << call;
    }
    EXPECT_TRUE(observer.update(input));
}

}  // namespace
