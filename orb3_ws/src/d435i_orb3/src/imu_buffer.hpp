#pragma once
#include <array>
#include <cmath>
#include <deque>
#include <vector>

struct ImuSample {
  double t;
  std::array<float, 3> a, w;
};

// Call under the owner's mutex. Keep the last sample before each image as
// the interpolation anchor for the next image; never integrate a sample twice.
class ImuBuffer {
 public:
  bool push(const ImuSample &s) {
    if (!std::isfinite(s.t) || s.t <= latest_) return false;
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(s.a[i]) || !std::isfinite(s.w[i])) return false;
    latest_ = s.t;
    data_.push_back(s);
    if (data_.size() > 4000) data_.pop_front();
    return true;
  }
  bool ready(double t) const {
    return data_.size() >= 2 && data_.front().t <= t && data_.back().t >= t;
  }
  bool stale(double t) const { return !data_.empty() && data_.front().t > t; }
  std::vector<ImuSample> take(double t, double previous) {
    std::vector<ImuSample> result;
    if (!ready(t) || t <= previous) return result;
    for (const auto &s : data_) {
      if (s.t > t) break;
      if (s.t > previous) result.push_back(s);
    }
    while (data_.size() > 1 && data_[1].t <= t) data_.pop_front();
    if (data_.front().t < t) {
      const auto &lo = data_[0];
      const auto &hi = data_[1];
      const double f = (t - lo.t) / (hi.t - lo.t);
      ImuSample end{t, {}, {}};
      for (int k = 0; k < 3; ++k) {
        end.a[k] = lo.a[k] + f * (hi.a[k] - lo.a[k]);
        end.w[k] = lo.w[k] + f * (hi.w[k] - lo.w[k]);
      }
      result.push_back(end);
    }
    return result;
  }
 private:
  double latest_ = -1.0;
  std::deque<ImuSample> data_;
};
