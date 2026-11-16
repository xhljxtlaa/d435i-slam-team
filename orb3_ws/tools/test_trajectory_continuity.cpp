#include "../src/d435i_orb3/src/trajectory_continuity.hpp"
#include <cassert>
#include <iostream>

int main() {
  TrajectoryContinuity rgbd;
  assert(!rgbd.update(1, false, false));
  assert(!rgbd.update(2, false, false));
  assert(!rgbd.update(2, false, false));
  assert(!rgbd.update(3, false, false));
  assert(!rgbd.update(2, false, false));
  assert(rgbd.update(1, false, false));  // map reset
  assert(!rgbd.update(1, false, false)); // one split per reset
  assert(!rgbd.update(2, false, false));
  assert(rgbd.update(4, false, false));  // lost/new-map boundary
  assert(!rgbd.update(1, false, false));
  assert(!rgbd.update(2, false, false));
  TrajectoryContinuity vi;
  assert(!vi.update(1, false, true));
  assert(!vi.update(2, false, true));
  assert(vi.update(2, true, true));      // inertial world alignment
  assert(!vi.update(2, true, true));
  assert(!vi.update(3, false, true));    // no status during short tracking loss
  assert(!vi.update(2, true, true));
  assert(vi.update(1, false, true));
  assert(!vi.update(1, false, true));
  assert(!vi.update(2, false, true));
  assert(vi.update(2, true, true));
  std::cout << "Map-reset and inertial-alignment trajectory segmentation: PASS\n";
}
