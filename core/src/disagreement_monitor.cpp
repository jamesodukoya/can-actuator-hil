#include "sbw/disagreement_monitor.hpp"

#include <cmath>

namespace sbw {

DisagreementMonitor::DisagreementMonitor(const MonitorConfig& config) : config_(config) {}

bool DisagreementMonitor::update(float motorAngleAtKingpinRad, float kingpinAngleRad) {
    const float differenceRad = std::fabs(motorAngleAtKingpinRad - kingpinAngleRad);
    // Strictly greater: the requirement says "more than" the limit.
    isConditionActive_ = differenceRad > config_.disagreementLimitRad;

    if (!isConditionActive_) {
        confirmCount_ = 0;
    } else if (confirmCount_ < config_.disagreementConfirmTicks) {
        // The count stops at the threshold so it cannot wrap around while a disagreement lasts.
        ++confirmCount_;
    }
    return confirmCount_ >= config_.disagreementConfirmTicks;
}

bool DisagreementMonitor::isConditionActive() const {
    return isConditionActive_;
}

void DisagreementMonitor::resetConfirmation() {
    confirmCount_ = 0;
}

}  // namespace sbw
