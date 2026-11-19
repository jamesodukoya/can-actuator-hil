#pragma once
#include <cstdint>

#include "sbw/axle_plant.hpp"
#include "sbw/sensor_conversion.hpp"

namespace sbw {

struct SensorModelConfig {
    std::int32_t motorCountsPerRev;  // 44: 11 pulses x 4 edges (assumed, not yet measured)
    std::uint16_t as5600ZeroCount;   // raw count at kingpin angle 0
};

// What the ECU's two angle sensors report for the plant's true angles: counts rounded down to whole
// steps, the AS5600's output filter, and the sensor faults the tests inject.
class SensorModel {
 public:
    explicit SensorModel(const SensorModelConfig& config);
    void advance(const AxlePlant& plant);              // after every plant step: the AS5600 filter
    void settleKingpinFilter(const AxlePlant& plant);  // filter output = true angle (start-up, tests)
    std::int32_t readMotorCounts(const AxlePlant& plant) const;
    std::uint16_t readKingpinRaw() const;  // filtered angle + offset, quantized
    double kingpinFilteredRad() const;
    void freezeMotorEncoder(const AxlePlant& plant);     // counts stay at the current value
    void setKingpinOffsetMrad(std::int16_t offsetMrad);  // adds to the filtered angle before quantization
    void setKingpinReadsFailing(bool isFailing);         // the runtime then marks readings stale
    bool isKingpinReadFailing() const;
    CurrentSenseSample sampleCurrent(const AxlePlant& plant, double appliedDuty) const;
    void clearFaults();

 private:
    std::int32_t motorCountsFor(double angleRad) const;
    std::uint16_t as5600CountFor(double angleRad) const;

    SensorModelConfig config_;
    double kingpinFilteredRad_ = 0.0;
    bool isMotorEncoderFrozen_ = false;
    std::int32_t frozenMotorCounts_ = 0;
    double kingpinOffsetRad_ = 0.0;
    bool isKingpinReadFailing_ = false;
};

// What the two ADC samples read for a motor current under the IPROPI rules: the drive sample sees the
// current only when it flows with the bridge's PH direction, the brake sample sees it either way. The
// drive sample is the ideal one even at a duty too small to sample; the decoder decides whether to use it.
CurrentSenseSample currentSenseSampleFor(double motorCurrentA, double appliedDuty);

}  // namespace sbw
