#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "sbw/can_protocol.hpp"
#include "sbw/steering_ecu.hpp"

namespace {

using sbw::CanFrame;
using sbw::ControlMode;
using sbw::SensorReadings;
using sbw::SteeringEcu;
using sbw::SteeringInputs;
using sbw::SteeringOutputs;

constexpr float kToleranceA = 1e-5f;
constexpr std::uint16_t kDefaultLimitMilliAmp = 1500;
constexpr std::size_t kCrcByteIndex = 7;
constexpr int kTicksInStatusTest = 20;
constexpr int kLastStepBeforeTimeout = 51;
constexpr int kIdleSteps = 200;
constexpr float kLoadTestCurrentA = 1.7391304f;  // k_t * i = 0.020 N·m with the nominal motor constant
constexpr int kKeepAlivePeriodTicks = 5;         // a DualLoop command this often keeps the timeout away
constexpr int kSettleSteps = 20;
constexpr int kDisagreementConfirmTicks = 10;      // nominal configuration
constexpr int kStaleLatchTicks = 3;                // nominal configuration
constexpr std::uint16_t kMisreadRawCount = 65;     // the kingpin reads 99.7 mrad while the motor stays at 0
constexpr float kJamTestCurrentA = 1.8f;           // k_t * i = 20.7 mN·m with the nominal motor constant
constexpr int kJamTestSteps = 100;
constexpr int kStepsToJamLatch = 8;
constexpr std::int16_t kExpectedHeldAngleMrad = 100;  // raw 65 is 99.7 mrad, rounded

// Grouped so the helper takes one parameter; fields left out take the defaults.
struct CommandSpec {
    ControlMode mode;
    std::int16_t targetMrad = 0;
    std::uint16_t limitMilliAmp = kDefaultLimitMilliAmp;
    std::uint8_t counter = 0;
};

SteeringEcu nominalEcu() {
    return SteeringEcu(sbw::kNominalEcuConfig);
}

CanFrame commandFrame(const CommandSpec& spec) {
    return sbw::packCommand({.targetKingpinAngleMrad = spec.targetMrad,
                             .currentLimitMilliAmp = spec.limitMilliAmp,
                             .controlMode = static_cast<std::uint8_t>(spec.mode),
                             .aliveCounter = spec.counter});
}

CanFrame withCorruptedCrc(CanFrame frame) {
    frame.data[kCrcByteIndex] = static_cast<std::uint8_t>(frame.data[kCrcByteIndex] ^ 0x01u);
    return frame;
}

SensorReadings restingReadings() {
    return {.motorEncoderCounts = 0,
            .as5600RawCount = 0,
            .isKingpinReadingFresh = true,
            .motorCurrentA = 0.0f,
            .busVoltageV = 12.0f};
}

// At most 3 frames fit; .at() fails loudly if a test passes more.
SteeringInputs inputsWith(std::initializer_list<CanFrame> frames,
                          const SensorReadings& readings = restingReadings()) {
    SteeringInputs inputs;
    inputs.sensors = readings;
    for (const CanFrame& frame : frames) {
        inputs.receivedFrames.at(inputs.receivedFrameCount) = frame;
        ++inputs.receivedFrameCount;
    }
    return inputs;
}

SteeringInputs noInputs() {
    return inputsWith({});
}

// Enters DualLoop, runs one idle tick, then delivers three frames with a broken CRC. Returns the outputs of
// the tick in which the fault latches.
SteeringOutputs latchCrcFault(SteeringEcu& ecu) {
    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop, .counter = 0})}));
    ecu.step(noInputs());
    const CanFrame broken = withCorruptedCrc(commandFrame({.mode = ControlMode::DualLoop, .counter = 1}));
    return ecu.step(inputsWith({broken, broken, broken}));
}

SensorReadings readingsWithRaw(std::uint16_t as5600RawCount) {
    SensorReadings readings = restingReadings();
    readings.as5600RawCount = as5600RawCount;
    return readings;
}

SensorReadings staleReadings() {
    SensorReadings readings = restingReadings();
    readings.isKingpinReadingFresh = false;
    return readings;
}

SensorReadings pushingReadings() {
    SensorReadings readings = restingReadings();
    readings.motorCurrentA = kJamTestCurrentA;
    return readings;
}

// Runs an ECU with a DualLoop command every 5th tick so the timeout never fires, and keeps the alive counter
// running across calls.
struct KeepAliveDriver {
    SteeringEcu ecu = nominalEcu();
    int tick = 0;
    std::uint8_t counter = 0;

    SteeringOutputs step(const SensorReadings& readings) {
        if (tick % kKeepAlivePeriodTicks == 0) {
            return stepWith(ControlMode::DualLoop, readings);
        }
        return advance(inputsWith({}, readings));
    }

    SteeringOutputs stepWith(ControlMode mode, const SensorReadings& readings) {
        const CanFrame frame = commandFrame({.mode = mode, .counter = counter});
        counter = static_cast<std::uint8_t>((counter + 1) & sbw::kAliveCounterMask);
        return advance(inputsWith({frame}, readings));
    }

 private:
    SteeringOutputs advance(const SteeringInputs& inputs) {
        ++tick;
        return ecu.step(inputs);
    }
};

// Settles in DualLoop, then misreads the kingpin until the disagreement latches.
void runUntilDisagreementLatched(KeepAliveDriver& driver) {
    for (int step = 0; step < kSettleSteps; ++step) {
        driver.step(restingReadings());
    }
    for (int step = 0; step < kDisagreementConfirmTicks; ++step) {
        driver.step(readingsWithRaw(kMisreadRawCount));
    }
}

// Enters DualLoop, then fails the kingpin read until the stale-reading fault latches.
void runUntilStaleLatched(KeepAliveDriver& driver) {
    driver.step(restingReadings());
    driver.step(restingReadings());
    for (int step = 0; step < kStaleLatchTicks; ++step) {
        driver.step(staleReadings());
    }
}

TEST(SteeringEcu, StartsOffWithBridgeDisabled) {
    SteeringEcu ecu = nominalEcu();

    const SteeringOutputs outputs = ecu.step(noInputs());

    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
    EXPECT_FALSE(ecu.isBridgeEnabled());
    EXPECT_FALSE(outputs.isBridgeEnabled);
    EXPECT_EQ(outputs.currentCommandA, 0.0f);
}

TEST(SteeringEcu, ValidCommandEntersMode) {
    SteeringEcu ecu = nominalEcu();

    const SteeringOutputs outputs = ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop})}));

    EXPECT_EQ(ecu.activeMode(), ControlMode::DualLoop);
    EXPECT_TRUE(ecu.isBridgeEnabled());
    EXPECT_TRUE(outputs.isBridgeEnabled);
}

TEST(SteeringEcu, CurrentFollowsControllerAndLimit) {
    RecordProperty("requirements", "REQ-SAF-001");
    SteeringEcu ecu = nominalEcu();
    // Both commands use the default 1500 mA limit.
    const CanFrame farTarget = commandFrame({.mode = ControlMode::DualLoop, .targetMrad = 100, .counter = 0});
    const CanFrame nearTarget = commandFrame({.mode = ControlMode::DualLoop, .targetMrad = 10, .counter = 1});

    // 19 A/rad * 0.1 rad = 1.9 A is above the 1.5 A limit, so the output saturates and the integrator holds.
    const SteeringOutputs saturated = ecu.step(inputsWith({farTarget}));
    EXPECT_NEAR(saturated.currentCommandA, 1.5f, kToleranceA);

    // 19 A/rad * 0.01 rad = 0.19 A, with the integrator still at 0.
    const SteeringOutputs following = ecu.step(inputsWith({nearTarget}));
    EXPECT_NEAR(following.currentCommandA, 0.19f, kToleranceA);
}

TEST(SteeringEcu, StatusEveryFiveTicksHeartbeatEveryTen) {
    RecordProperty("requirements", "REQ-CAN-004");
    SteeringEcu ecu = nominalEcu();
    int statusCount = 0;
    int heartbeatCount = 0;

    for (int tick = 0; tick < kTicksInStatusTest; ++tick) {
        const SteeringOutputs outputs = ecu.step(noInputs());
        if (tick == 0) {
            EXPECT_EQ(outputs.frameCount, std::uint8_t{2});
            EXPECT_EQ(outputs.frames[0].id, sbw::kStatusId);
            EXPECT_EQ(outputs.frames[1].id, sbw::kHeartbeatId);
        }
        for (std::size_t index = 0; index < outputs.frameCount; ++index) {
            if (outputs.frames[index].id == sbw::kStatusId) {
                ++statusCount;
            }
            if (outputs.frames[index].id == sbw::kHeartbeatId) {
                ++heartbeatCount;
            }
        }
    }

    EXPECT_EQ(statusCount, 4);     // ticks 0, 5, 10, 15
    EXPECT_EQ(heartbeatCount, 2);  // ticks 0, 10
}

TEST(SteeringEcu, StatusReportsModeAndAngle) {
    SteeringEcu ecu = nominalEcu();

    // The mode is applied before the frames are built, so the status of the same tick already shows it.
    const SteeringOutputs outputs = ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop})}));

    const sbw::StatusMessage status = sbw::unpackStatus(outputs.frames[0]);
    EXPECT_EQ(status.stateAndFaults & 0x03, 3);  // bits 0-1 hold the mode
    EXPECT_EQ(status.kingpinAngleMrad, 0);
}

TEST(SteeringEcu, HeartbeatCarriesUptimeAndVoltage) {
    SteeringEcu ecu = nominalEcu();
    SteeringOutputs outputs;
    for (int tick = 0; tick <= 10; ++tick) {
        outputs = ecu.step(noInputs());
    }

    // Tick 10 has a status frame first and the heartbeat second.
    const sbw::HeartbeatMessage heartbeat = sbw::unpackHeartbeat(outputs.frames[1]);
    EXPECT_EQ(heartbeat.uptimeMs, 10u);
    EXPECT_EQ(heartbeat.busVoltageMilliVolt, 12000);
    EXPECT_EQ(heartbeat.nodeState, static_cast<std::uint8_t>(sbw::NodeState::Running));
}

TEST(SteeringEcu, ThreeBadCrcsLatchCrcFault) {
    RecordProperty("requirements", "REQ-CAN-001,REQ-SAF-003,REQ-CAN-004");
    SteeringEcu ecu = nominalEcu();

    // This is tick 2, which is neither a status nor a heartbeat tick, so the only frame is the extra
    // status that reports the new fault.
    const SteeringOutputs outputs = latchCrcFault(ecu);

    EXPECT_EQ(ecu.latchedFaults(), sbw::fault_bits::kCrc);
    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
    EXPECT_FALSE(ecu.isBridgeEnabled());
    EXPECT_FALSE(outputs.isBridgeEnabled);
    EXPECT_EQ(outputs.currentCommandA, 0.0f);
    ASSERT_EQ(outputs.frameCount, std::uint8_t{1});
    EXPECT_EQ(outputs.frames[0].id, sbw::kStatusId);
    EXPECT_NE(sbw::unpackStatus(outputs.frames[0]).stateAndFaults & sbw::fault_bits::kCrc, 0);
}

TEST(SteeringEcu, TwoBadCrcsThenGoodAreTolerated) {
    RecordProperty("requirements", "REQ-CAN-001");
    SteeringEcu ecu = nominalEcu();
    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop, .counter = 0})}));

    const CanFrame brokenOne = withCorruptedCrc(commandFrame({.mode = ControlMode::DualLoop, .counter = 1}));
    const CanFrame brokenTwo = withCorruptedCrc(commandFrame({.mode = ControlMode::DualLoop, .counter = 2}));
    const CanFrame good = commandFrame({.mode = ControlMode::DualLoop, .counter = 3});
    ecu.step(inputsWith({brokenOne, brokenTwo, good}));

    EXPECT_EQ(ecu.latchedFaults(), 0u);
    EXPECT_EQ(ecu.activeMode(), ControlMode::DualLoop);
}

TEST(SteeringEcu, ThreeRepeatedCountersLatchCounterFault) {
    RecordProperty("requirements", "REQ-CAN-002");
    SteeringEcu ecu = nominalEcu();
    const CanFrame repeated = commandFrame({.mode = ControlMode::DualLoop, .counter = 0});
    ecu.step(inputsWith({repeated}));

    ecu.step(inputsWith({repeated, repeated, repeated}));

    EXPECT_EQ(ecu.latchedFaults(), sbw::fault_bits::kCounter);
    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
}

TEST(SteeringEcu, TimeoutAfterMoreThanFiftyTicks) {
    RecordProperty("requirements", "REQ-MON-001");
    SteeringEcu ecu = nominalEcu();
    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop})}));  // step 1, tick 0

    for (int stepNumber = 2; stepNumber <= kLastStepBeforeTimeout; ++stepNumber) {
        ecu.step(noInputs());
        EXPECT_EQ(ecu.activeMode(), ControlMode::DualLoop) << "step " << stepNumber;
    }
    EXPECT_EQ(ecu.latchedFaults(), 0u);

    // Step 52 is tick 51: 51 ticks since the command is more than the 50-tick limit.
    const SteeringOutputs outputs = ecu.step(noInputs());

    EXPECT_EQ(ecu.latchedFaults(), sbw::fault_bits::kTimeout);
    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
    EXPECT_FALSE(outputs.isBridgeEnabled);
}

TEST(SteeringEcu, NoTimeoutWhileOff) {
    SteeringEcu ecu = nominalEcu();

    for (int step = 0; step < kIdleSteps; ++step) {
        ecu.step(noInputs());
    }

    EXPECT_EQ(ecu.latchedFaults(), 0u);
}

TEST(SteeringEcu, LatchedFaultIgnoresModeCommands) {
    RecordProperty("requirements", "REQ-SAF-002");
    SteeringEcu ecu = nominalEcu();
    latchCrcFault(ecu);

    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop, .counter = 1})}));

    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
    EXPECT_NE(ecu.latchedFaults(), 0u);
}

TEST(SteeringEcu, ClearFaultClearsAndStaysOff) {
    RecordProperty("requirements", "REQ-SAF-002");
    SteeringEcu ecu = nominalEcu();
    latchCrcFault(ecu);

    ecu.step(inputsWith({commandFrame({.mode = ControlMode::ClearFault, .counter = 1})}));
    EXPECT_EQ(ecu.latchedFaults(), 0u);
    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);

    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop, .counter = 2})}));
    EXPECT_EQ(ecu.activeMode(), ControlMode::DualLoop);
}

TEST(SteeringEcu, ClearFaultWhileHealthyTurnsOff) {
    SteeringEcu ecu = nominalEcu();
    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop, .counter = 0})}));

    ecu.step(inputsWith({commandFrame({.mode = ControlMode::ClearFault, .counter = 1})}));

    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
    EXPECT_EQ(ecu.latchedFaults(), 0u);
}

TEST(SteeringEcu, InvalidModeIsIgnored) {
    SteeringEcu ecu = nominalEcu();
    ecu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop, .counter = 0})}));

    ecu.step(inputsWith({commandFrame({.mode = static_cast<ControlMode>(5), .counter = 1})}));

    EXPECT_EQ(ecu.activeMode(), ControlMode::DualLoop);
    EXPECT_EQ(ecu.latchedFaults(), 0u);
}

// estimatedLoadTorqueNm() is a public function that no other test calls.
TEST(SteeringEcu, ReportsEstimatedLoadTorque) {
    SteeringEcu ecu = nominalEcu();
    SensorReadings pushing = restingReadings();
    pushing.motorCurrentA = kLoadTestCurrentA;

    // The observer's first sample only stores; the second moves the estimate 30 % of the way to 0.020 N·m.
    ecu.step(inputsWith({}, pushing));
    ecu.step(inputsWith({}, pushing));

    EXPECT_NEAR(ecu.estimatedLoadTorqueNm(), 0.006f, kToleranceA);
}

TEST(SteeringEcu, DisagreementLatchesOnTenthTick) {
    RecordProperty("requirements", "REQ-MON-002,REQ-SAF-003");
    KeepAliveDriver driver;
    for (int step = 0; step < kSettleSteps; ++step) {
        driver.step(restingReadings());
    }
    ASSERT_EQ(driver.ecu.activeMode(), ControlMode::DualLoop);

    for (int stepAfterSwitch = 1; stepAfterSwitch < kDisagreementConfirmTicks; ++stepAfterSwitch) {
        driver.step(readingsWithRaw(kMisreadRawCount));
        EXPECT_EQ(driver.ecu.activeMode(), ControlMode::DualLoop) << "step " << stepAfterSwitch;
    }
    const SteeringOutputs outputs = driver.step(readingsWithRaw(kMisreadRawCount));

    EXPECT_EQ(driver.ecu.latchedFaults(), sbw::fault_bits::kDisagree);
    EXPECT_EQ(driver.ecu.activeMode(), ControlMode::Off);
    EXPECT_FALSE(outputs.isBridgeEnabled);
}

TEST(SteeringEcu, DisagreementBlocksClearUntilGone) {
    RecordProperty("requirements", "REQ-SAF-002");
    KeepAliveDriver driver;
    runUntilDisagreementLatched(driver);

    driver.stepWith(ControlMode::ClearFault, readingsWithRaw(kMisreadRawCount));
    EXPECT_EQ(driver.ecu.latchedFaults(), sbw::fault_bits::kDisagree);

    driver.step(restingReadings());  // the kingpin reads correctly again, so the condition ends this tick
    driver.stepWith(ControlMode::ClearFault, restingReadings());
    EXPECT_EQ(driver.ecu.latchedFaults(), 0u);
    EXPECT_EQ(driver.ecu.activeMode(), ControlMode::Off);
}

TEST(SteeringEcu, JamLatchesOnEighthTick) {
    RecordProperty("requirements", "REQ-JAM-001,REQ-SAF-003");
    KeepAliveDriver driver;

    // Step 1 only stores a sample, the estimate passes 12 mN·m on step 4, and 5 confirming steps follow.
    for (int step = 1; step < kStepsToJamLatch; ++step) {
        driver.step(pushingReadings());
        EXPECT_EQ(driver.ecu.latchedFaults(), 0u) << "step " << step;
    }
    ASSERT_EQ(driver.ecu.activeMode(), ControlMode::DualLoop);
    const SteeringOutputs outputs = driver.step(pushingReadings());

    EXPECT_EQ(driver.ecu.latchedFaults(), sbw::fault_bits::kJam);
    EXPECT_EQ(driver.ecu.activeMode(), ControlMode::Off);
    EXPECT_FALSE(outputs.isBridgeEnabled);
    ASSERT_GE(outputs.frameCount, std::uint8_t{1});
    EXPECT_NE(sbw::unpackStatus(outputs.frames[0]).stateAndFaults & sbw::fault_bits::kJam, 0);
}

TEST(SteeringEcu, JamIgnoredWhileOff) {
    SteeringEcu ecu = nominalEcu();

    for (int step = 0; step < kJamTestSteps; ++step) {
        ecu.step(inputsWith({}, pushingReadings()));
    }

    EXPECT_EQ(ecu.latchedFaults(), 0u);
    EXPECT_EQ(ecu.activeMode(), ControlMode::Off);
}

TEST(SteeringEcu, StaleKingpinLatchesOnThirdTick) {
    RecordProperty("requirements", "REQ-MON-003,REQ-SAF-003");
    KeepAliveDriver driver;
    driver.step(restingReadings());
    driver.step(restingReadings());

    for (int staleStep = 1; staleStep < kStaleLatchTicks; ++staleStep) {
        driver.step(staleReadings());
        EXPECT_EQ(driver.ecu.latchedFaults(), 0u) << "stale step " << staleStep;
    }
    const SteeringOutputs outputs = driver.step(staleReadings());

    EXPECT_EQ(driver.ecu.latchedFaults(), sbw::fault_bits::kInternal);
    EXPECT_FALSE(outputs.isBridgeEnabled);
    EXPECT_EQ(outputs.currentCommandA, 0.0f);
}

TEST(SteeringEcu, StaleKingpinHoldsLastFreshAngle) {
    RecordProperty("requirements", "REQ-MON-003");
    SteeringEcu ecu = nominalEcu();
    ecu.step(inputsWith({}, readingsWithRaw(kMisreadRawCount)));  // tick 0: a fresh read

    SteeringOutputs outputs;
    for (int tick = 1; tick <= 5; ++tick) {
        outputs = ecu.step(inputsWith({}, staleReadings()));  // the raw count of a failed read is 0
    }

    // Tick 5 is a status tick, so the status frame is first.
    EXPECT_EQ(sbw::unpackStatus(outputs.frames[0]).kingpinAngleMrad, kExpectedHeldAngleMrad);
}

TEST(SteeringEcu, StaleReadingBlocksClear) {
    RecordProperty("requirements", "REQ-MON-003,REQ-SAF-002");
    KeepAliveDriver driver;
    runUntilStaleLatched(driver);

    driver.stepWith(ControlMode::ClearFault, staleReadings());
    EXPECT_EQ(driver.ecu.latchedFaults(), sbw::fault_bits::kInternal);

    driver.step(restingReadings());  // one fresh read
    driver.stepWith(ControlMode::ClearFault, restingReadings());
    EXPECT_EQ(driver.ecu.latchedFaults(), 0u);
    EXPECT_EQ(driver.ecu.activeMode(), ControlMode::Off);
}

TEST(SteeringEcu, ScheduleOverrunLatchesInternalFaultInAnyMode) {
    RecordProperty("requirements", "REQ-TIM-001,REQ-SAF-003");
    SteeringInputs overrun = noInputs();
    overrun.hasScheduleOverrun = true;

    // From Off: ticks 0 and 1 are normal. Tick 2 is not a status tick, so the only frame is the event status.
    SteeringEcu idleEcu = nominalEcu();
    idleEcu.step(noInputs());
    idleEcu.step(noInputs());
    const SteeringOutputs idleOutputs = idleEcu.step(overrun);
    EXPECT_EQ(idleEcu.latchedFaults(), sbw::fault_bits::kInternal);
    ASSERT_EQ(idleOutputs.frameCount, std::uint8_t{1});
    EXPECT_EQ(idleOutputs.frames[0].id, sbw::kStatusId);

    // From DualLoop: the bridge is off in the same tick.
    SteeringEcu drivingEcu = nominalEcu();
    drivingEcu.step(inputsWith({commandFrame({.mode = ControlMode::DualLoop})}));
    drivingEcu.step(noInputs());
    const SteeringOutputs drivingOutputs = drivingEcu.step(overrun);
    EXPECT_EQ(drivingEcu.latchedFaults(), sbw::fault_bits::kInternal);
    EXPECT_EQ(drivingEcu.activeMode(), ControlMode::Off);
    EXPECT_FALSE(drivingOutputs.isBridgeEnabled);
}

}  // namespace
