#include "../src/d435i_orb3/src/imu_buffer.hpp"
#include <cassert>
#include <iostream>
#include <limits>

ImuSample sample(double t) {
  return {t, {{float(t), float(2*t), 9.81f}}, {{0.0f, 0.0f, float(3*t)}}};
}

int main() {
  ImuBuffer b;
  assert(!b.ready(1.0));
  assert(b.push(sample(1.0)));
  assert(b.push(sample(2.0)));
  assert(b.push(sample(3.0)));
  assert(!b.push(sample(3.0)));  // duplicate timestamps
  assert(!b.push(sample(2.5)));  // out-of-order timestamps
  auto invalid = sample(4.0);
  invalid.a[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!b.push(invalid));
  assert(!b.ready(3.5));  // future sample required
  assert(b.stale(0.5));
  const auto first = b.take(1.5, 0.9);
  assert(first.size() == 2 && first[0].t == 1.0 && first[1].t == 1.5);
  assert(std::abs(first.back().a[0] - 1.5f) < 1e-6);
  assert(std::abs(first.back().w[2] - 4.5f) < 1e-6);
  const auto second = b.take(2.5, 1.5);
  assert(second.size() == 2 && second[0].t == 2.0 && second[1].t == 2.5);
  const auto third = b.take(3.0, 2.5);
  assert(third.size() == 1 && third[0].t == 3.0);
  assert(b.take(3.0, 3.0).empty());
  // A stalled image consumer cannot cause unbounded IMU memory growth.
  for (int i = 4; i < 5000; ++i) assert(b.push(sample(i)));
  assert(b.stale(100.0));
  assert(b.ready(4998.5));
  std::cout << "IMU interpolation, frame boundaries, invalid samples and bounded queue: PASS\n";
}
