#include "sbw/rate_estimator.hpp"

#include "sbw/types.hpp"

namespace sbw {

float WindowedRateEstimator::update(float angleRad) {
    newestIndex_ = (newestIndex_ + 1) % history_.size();
    history_[newestIndex_] = angleRad;
    if (sampleCount_ < history_.size()) {
        ++sampleCount_;
    }
    // While the window fills, the rate uses however many intervals exist (1 to 8).
    const std::size_t intervals = sampleCount_ - 1;
    if (intervals == 0) {
        return 0.0f;
    }
    const std::size_t oldestIndex = (newestIndex_ + history_.size() - intervals) % history_.size();
    const float elapsedSec = static_cast<float>(intervals) * kControlPeriodSec;
    return (angleRad - history_[oldestIndex]) / elapsedSec;
}

void WindowedRateEstimator::reset() {
    sampleCount_ = 0;
}

}  // namespace sbw
