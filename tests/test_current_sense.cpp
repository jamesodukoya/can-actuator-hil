#include <gtest/gtest.h>

#include "sbw/sensor_conversion.hpp"
#include "sbw/sensor_model.hpp"

namespace {

using sbw::CurrentSenseDecoder;
using sbw::CurrentSenseSample;

constexpr float kToleranceA = 1e-6f;
// 1365 counts * 7.32421875e-4 A per count: what a 1 A current decodes to.
constexpr float kOneAmpDecodedA = 0.9997559f;
constexpr std::uint16_t kOneAmpCounts = 1365;  // 1 / 7.32421875e-4 = 1365.3, rounded down
constexpr float kForwardDuty = 0.5f;

TEST(CurrentSense, SamplesFollowIpropiRules) {
    RecordProperty("requirements", "REQ-CUR-002");

    // Current with PH: both samples see it. Against PH: the drive sample reads 0.
    const CurrentSenseSample withPh = sbw::currentSenseSampleFor(1.0, 0.5);
    EXPECT_EQ(withPh.driveCounts, kOneAmpCounts);
    EXPECT_EQ(withPh.brakeCounts, kOneAmpCounts);

    const CurrentSenseSample againstPh = sbw::currentSenseSampleFor(-1.0, 0.5);
    EXPECT_EQ(againstPh.driveCounts, 0);
    EXPECT_EQ(againstPh.brakeCounts, kOneAmpCounts);

    // PH reversed: the same rules with the signs swapped.
    const CurrentSenseSample reversedWithPh = sbw::currentSenseSampleFor(-1.0, -0.5);
    EXPECT_EQ(reversedWithPh.driveCounts, kOneAmpCounts);
    EXPECT_EQ(reversedWithPh.brakeCounts, kOneAmpCounts);

    const CurrentSenseSample reversedAgainstPh = sbw::currentSenseSampleFor(1.0, -0.5);
    EXPECT_EQ(reversedAgainstPh.driveCounts, 0);
    EXPECT_EQ(reversedAgainstPh.brakeCounts, kOneAmpCounts);

    // 5 A is beyond the 12-bit range (about 2.7 A): both samples clamp at 4095.
    const CurrentSenseSample overRange = sbw::currentSenseSampleFor(5.0, 0.5);
    EXPECT_EQ(overRange.driveCounts, 4095);
    EXPECT_EQ(overRange.brakeCounts, 4095);
}

TEST(CurrentSense, DecodesForwardCurrent) {
    RecordProperty("requirements", "REQ-CUR-002");
    CurrentSenseDecoder decoder;

    EXPECT_NEAR(decoder.decode({kOneAmpCounts, kOneAmpCounts}, kForwardDuty), kOneAmpDecodedA, kToleranceA);
}

TEST(CurrentSense, DecodesCurrentAgainstPh) {
    RecordProperty("requirements", "REQ-CUR-002");
    // The braking case: the current opposes PH, so the drive sample is 0 and the sign must come out negative.
    CurrentSenseDecoder brakingForward;
    EXPECT_NEAR(brakingForward.decode({0, kOneAmpCounts}, kForwardDuty), -kOneAmpDecodedA, kToleranceA);

    CurrentSenseDecoder brakingReversed;
    EXPECT_NEAR(brakingReversed.decode({0, kOneAmpCounts}, -kForwardDuty), kOneAmpDecodedA, kToleranceA);

    CurrentSenseDecoder drivingReversed;
    EXPECT_NEAR(drivingReversed.decode({kOneAmpCounts, kOneAmpCounts}, -kForwardDuty), -kOneAmpDecodedA,
                kToleranceA);
}

TEST(CurrentSense, KeepsDirectionBelowMinimumDuty) {
    RecordProperty("requirements", "REQ-CUR-002");
    CurrentSenseDecoder decoder;
    constexpr float kTooShortDuty = 0.03f;

    EXPECT_NEAR(decoder.decode({0, kOneAmpCounts}, kForwardDuty), -kOneAmpDecodedA, kToleranceA);
    // The drive phase is too short to sample, so its reading is ignored and the last direction stays.
    EXPECT_NEAR(decoder.decode({kOneAmpCounts, kOneAmpCounts}, kTooShortDuty), -kOneAmpDecodedA, kToleranceA);
    // Exactly the minimum duty counts as long enough.
    EXPECT_NEAR(decoder.decode({kOneAmpCounts, kOneAmpCounts}, sbw::kMinSenseDuty), kOneAmpDecodedA,
                kToleranceA);
}

TEST(CurrentSense, HalfThresholdDecidesDirection) {
    RecordProperty("requirements", "REQ-CUR-002");
    constexpr std::uint16_t kJustEnoughDriveCounts = 683;  // 2 * 683 = 1366 >= 1365
    constexpr std::uint16_t kJustTooLowDriveCounts = 682;  // 2 * 682 = 1364 < 1365

    CurrentSenseDecoder atThreshold;
    EXPECT_NEAR(atThreshold.decode({kJustEnoughDriveCounts, kOneAmpCounts}, kForwardDuty), kOneAmpDecodedA,
                kToleranceA);

    CurrentSenseDecoder belowThreshold;
    EXPECT_NEAR(belowThreshold.decode({kJustTooLowDriveCounts, kOneAmpCounts}, kForwardDuty),
                -kOneAmpDecodedA, kToleranceA);
}

}  // namespace
