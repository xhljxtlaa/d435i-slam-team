#pragma once

// A map reset or inertial world alignment starts a new coordinate history.
// Recently-lost tracking alone may relocalize in the same map and is retained.
class TrajectoryContinuity {
 public:
  bool update(int state, bool imu_initialized, bool inertial) {
    const bool bad = state <= 1 || state == 4;
    const bool was_bad = previous_state_ <= 1 || previous_state_ == 4;
    bool split = seen_ && bad && !was_bad;
    if (bad) {
      previous_imu_initialized_ = false;
    } else if (state == 2) {
      if (seen_ && inertial && imu_initialized != previous_imu_initialized_) split = true;
      previous_imu_initialized_ = imu_initialized;
    }
    previous_state_ = state;
    seen_ = true;
    return split;
  }
 private:
  bool seen_ = false;
  int previous_state_ = 0;
  bool previous_imu_initialized_ = false;
};
