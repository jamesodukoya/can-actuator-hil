#pragma once
#include <array>
#include <cstddef>

namespace sbw {

// Speed from the difference between the newest angle and the one 8 ticks earlier. Differencing over 8 ms
// gives 8 times finer speed resolution than one tick, and delays the estimate by 4 ms.
class WindowedRateEstimator {
 public:
    static constexpr std::size_t kWindowSamples = 8;

    float update(float angleRad);  // returns rad/s; call once per tick
    void reset();

 private:
    // One more position than the window length: 9 positions make 8 intervals.
    std::array<float, kWindowSamples + 1> history_{};
    std::size_t newestIndex_ = 0;
    std::size_t sampleCount_ = 0;
};

}  // namespace sbw
