#include <gtest/gtest.h>

#include "sbw/alive_counter.hpp"

TEST(AliveCounter, FirstFrameIsAccepted) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;

    EXPECT_EQ(checker.check(7), sbw::CounterVerdict::Accepted);
}

TEST(AliveCounter, IncrementIsAccepted) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(7);

    EXPECT_EQ(checker.check(8), sbw::CounterVerdict::Accepted);
}

TEST(AliveCounter, RepeatIsRejected) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(7);
    checker.check(8);

    EXPECT_EQ(checker.check(8), sbw::CounterVerdict::Repeated);
    EXPECT_EQ(checker.check(9), sbw::CounterVerdict::Accepted);  // the reference stayed at 8
}

TEST(AliveCounter, JumpOfThreeIsAccepted) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(8);

    EXPECT_EQ(checker.check(11), sbw::CounterVerdict::Accepted);
}

TEST(AliveCounter, JumpOfFourIsRejectedAndResyncs) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(11);

    EXPECT_EQ(checker.check(15), sbw::CounterVerdict::Jumped);
    EXPECT_EQ(checker.check(0), sbw::CounterVerdict::Accepted);  // delta 1 from 15
}

TEST(AliveCounter, RolloverIsAccepted) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(15);

    EXPECT_EQ(checker.check(0), sbw::CounterVerdict::Accepted);
}

TEST(AliveCounter, UpperNibbleIsIgnored) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(7);

    EXPECT_EQ(checker.check(0x18), sbw::CounterVerdict::Accepted);  // 0x18 & 0x0F = 8
}

TEST(AliveCounter, ResetForgetsReference) {
    RecordProperty("requirements", "REQ-CAN-002");
    sbw::AliveCounterChecker checker;
    checker.check(7);
    checker.reset();

    EXPECT_EQ(checker.check(7), sbw::CounterVerdict::Accepted);  // would be Repeated without reset
}
