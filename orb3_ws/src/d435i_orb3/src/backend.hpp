#pragma once
#include <array>
#include <string>
#include <vector>
#include <opencv2/core/mat.hpp>
#include "imu_buffer.hpp"
#include "cloud_geometry.hpp"

struct MappingOptions {
  bool enabled = true;
  bool dense_enabled = false;
  float voxel_size = 0.05f;
  float min_depth = 0.18f;
  float max_depth = 6.0f;
  int pixel_stride = 8;
  size_t max_points = 100000;
  size_t max_keyframes = 200;
  unsigned min_observations = 1;
  double update_interval = 1.0;
};

// Keep Eigen/Sophus and their aligned allocators out of the ROS translation unit.
// ROS Humble uses C++17; the upstream ORB library and g2o use older standards.
struct BackendResult {
  int state = 0;
  bool imu_initialized = false;
  bool finite = false;
  std::array<float, 3> position{};
  std::array<float, 4> quaternion{};  // x y z w
  bool cloud_updated = false;
  bool cloud_reset = false;
  int64_t map_id = -1;
  size_t map_keyframes = 0;
  size_t cached_depth_keyframes = 0;
  std::vector<CloudPoint> surface_cloud;
  std::vector<CloudPoint> sparse_cloud;
  std::vector<CloudPoint> depth_cloud;
};

class Backend {
 public:
  Backend(const std::string &vocabulary, const std::string &settings, bool inertial, bool viewer,
          const MappingOptions &mapping = MappingOptions());
  ~Backend();
  Backend(const Backend &) = delete;
  Backend &operator=(const Backend &) = delete;
  BackendResult track(const cv::Mat &image, const cv::Mat &depth,
                      double time, const std::vector<ImuSample> &samples);
  void shutdown(const std::string &directory, bool save);
  void clearMapping();
 private:
  class Impl;
  Impl *impl_;
};
