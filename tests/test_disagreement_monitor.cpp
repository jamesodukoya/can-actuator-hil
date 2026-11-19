#include <gtest/gtest.h>

#include "sbw/disagreement_monitor.hpp"

namespace {

using sbw::DisagreementMonitor;

// The motor is held at 0 rad, so the kingpin angle is also the difference. The nominal limit is 0.05 rad and
// 10 consecutive samples confirm.
constexpr float kBeyondLimitRad = 0.06f;
constexpr float kWithinLimitRad = 0.01f;
constexpr float kAtLimitRad = 0.05f;
constexpr int kSamplesBeforeConfirmation = 9;
constexpr int kLongRun = 20;

DisagreementMonitor nominalMonitor() {
    return DisagreementMonitor(sbw::kNominalEcuConfig.monitors);
}

// Returns the result of the last call. The count only reaches the threshold on the last sample of a run
// that starts from zero, so "false" here means no earlier call returned true either.
bool updateRepeatedly(DisagreementMonitor& monitor, float kingpinAngleRad, int sampleCount) {
    bool isConfirmed = false;
    for (int sample = 0; sample < sampleCount; ++sample) {
        isConfirmed = monitor.update(0.0f, kingpinAngleRad);
    }
    return isConfirmed;
}

TEST(DisagreementMonitor, ConfirmsOnTenthConsecutiveSample) {
    RecordProperty("requirements", "REQ-MON-002");
    DisagreementMonitor monitor = nominalMonitor();

    EXPECT_FALSE(updateRepeatedly(monitor, kBeyondLimitRad, kSamplesBeforeConfirmation));
    EXPECT_TRUE(monitor.update(0.0f, kBeyondLimitRad));
}

TEST(DisagreementMonitor, InRangeSampleRestartsCount) {
    RecordProperty("requirements", "REQ-MON-002");
    DisagreementMonitor monitor = nominalMonitor();

    updateRepeatedly(monitor, kBeyondLimitRad, kSamplesBeforeConfirmation);
    EXPECT_FALSE(monitor.update(0.0f, kWithinLimitRad));

    EXPECT_FALSE(updateRepeatedly(monitor, kBeyondLimitRad, kSamplesBeforeConfirmation));
    EXPECT_TRUE(monitor.update(0.0f, kBeyondLimitRad));
}

TEST(DisagreementMonitor, ExactlyAtLimitIsNotAFault) {
    RecordProperty("requirements", "REQ-MON-002");
    DisagreementMonitor monitor = nominalMonitor();

    // The limit and this angle are the same float literal, so the difference equals the limit exactly.
    EXPECT_FALSE(updateRepeatedly(monitor, kAtLimitRad, kLongRun));
}

TEST(DisagreementMonitor, ConditionTracksLatestSample) {
    RecordProperty("requirements", "REQ-MON-002");
    DisagreementMonitor monitor = nominalMonitor();

    monitor.update(0.0f, kBeyondLimitRad);
    EXPECT_TRUE(monitor.isConditionActive());

    monitor.update(0.0f, kWithinLimitRad);
    EXPECT_FALSE(monitor.isConditionActive());
}

TEST(DisagreementMonitor, ResetConfirmationRestartsCount) {
    RecordProperty("requirements", "REQ-MON-002");
    DisagreementMonitor monitor = nominalMonitor();

    updateRepeatedly(monitor, kBeyondLimitRad, kSamplesBeforeConfirmation);
    monitor.resetConfirmation();

    EXPECT_FALSE(updateRepeatedly(monitor, kBeyondLimitRad, kSamplesBeforeConfirmation));
    EXPECT_TRUE(monitor.update(0.0f, kBeyondLimitRad));
}

}  // namespace
