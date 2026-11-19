#include "sbw/sensor_model.hpp"

#include <cmath>

#include "sbw/sensor_conversion.hpp"

namespace sbw {

namespace {

constexpr double kTwoPiRad = 6.283185307179586;
constexpr double kMilliradPerRad = 1000.0;

// AS5600 slow filter (SF=11): 0.286 ms step response (datasheet), used as a first-order time constant.
// This overstates the lag if the datasheet figure is a settling time rather than a time constant.
constexpr double kAs5600FilterTimeConstantSec = 0.286e-3;
// Backward Euler: gain dt / (tau + dt) = 0.0337838 per plant step. Only + - * / are used so the result
// is the same on every machine.
constexpr double kAs5600FilterGain = kPlantStepSec / (kAs5600FilterTimeConstantSec + kPlantStepSec);

}  // namespace

SensorModel::SensorModel(const SensorModelConfig& config) : config_(config) {}

void SensorModel::advance(const AxlePlant& plant) {
    kingpinFilteredRad_ += (plant.kingpinAngleRad() - kingpinFilteredRad_) * kAs5600FilterGain;
}

void SensorModel::settleKingpinFilter(const AxlePlant& plant) {
    kingpinFilteredRad_ = plant.kingpinAngleRad();
}

std::int32_t SensorModel::readMotorCounts(const AxlePlant& plant) const {
    if (isMotorEncoderFrozen_) {
        return frozenMotorCounts_;
    }
    return motorCountsFor(plant.motorAngleRad());
}

std::uint16_t SensorModel::readKingpinRaw() const {
    return as5600CountFor(kingpinFilteredRad_ + kingpinOffsetRad_);
}

double SensorModel::kingpinFilteredRad() const {
    return kingpinFilteredRad_;
}

void SensorModel::freezeMotorEncoder(const AxlePlant& plant) {
    frozenMotorCounts_ = motorCountsFor(plant.motorAngleRad());
    isMotorEncoderFrozen_ = true;
}

void SensorModel::setKingpinOffsetMrad(std::int16_t offsetMrad) {
    kingpinOffsetRad_ = offsetMrad / kMilliradPerRad;
}

void SensorModel::setKingpinReadsFailing(bool isFailing) {
    isKingpinReadFailing_ = isFailing;
}

bool SensorModel::isKingpinReadFailing() const {
    return isKingpinReadFailing_;
}

void SensorModel::clearFaults() {
    isMotorEncoderFrozen_ = false;
    kingpinOffsetRad_ = 0.0;
    isKingpinReadFailing_ = false;
}

// An encoder reports which step the shaft is in, so the angle is rounded down, not to the nearest step.
std::int32_t SensorModel::motorCountsFor(double angleRad) const {
    const double lsbRad = kTwoPiRad / config_.motorCountsPerRev;
    return static_cast<std::int32_t>(std::floor(angleRad / lsbRad));
}

std::uint16_t SensorModel::as5600CountFor(double angleRad) const {
    const double lsbRad = kTwoPiRad / kAs5600CountsPerRev;
    const auto steps = static_cast<std::int64_t>(std::floor(angleRad / lsbRad));
    const std::int64_t countsPerRev = kAs5600CountsPerRev;
    const std::int64_t stepsFromMechanicalZero = steps + config_.as5600ZeroCount;
    // C++ keeps the sign of the dividend in %, so a negative step count needs the extra + countsPerRev.
    const std::int64_t wrapped = ((stepsFromMechanicalZero % countsPerRev) + countsPerRev) % countsPerRev;
    return static_cast<std::uint16_t>(wrapped);
}

}  // namespace sbw
