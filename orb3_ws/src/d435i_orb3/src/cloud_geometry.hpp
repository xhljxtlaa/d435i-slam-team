#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

struct CloudPoint { float x, y, z, intensity; };
static_assert(sizeof(CloudPoint) == 16, "PointCloud2 requires four packed floats");
using CloudTransform = std::array<float, 12>; // row-major [R|t]

inline CloudPoint transformPoint(const CloudPoint &p, const CloudTransform &T) {
  return {T[0]*p.x+T[1]*p.y+T[2]*p.z+T[3],
          T[4]*p.x+T[5]*p.y+T[6]*p.z+T[7],
          T[8]*p.x+T[9]*p.y+T[10]*p.z+T[11], p.intensity};
}

class DepthProjector {
 public:
  DepthProjector(const cv::Mat &K, const cv::Mat &D, int width, int height,
                 int stride, float minimum, float maximum, float depth_factor)
      : width_(width), height_(height), minimum_(minimum), maximum_(maximum),
        scale_(1.0f/depth_factor) {
    if (stride < 1 || width < 3 || height < 3 || minimum <= 0 || maximum <= minimum || depth_factor <= 0)
      throw std::invalid_argument("Invalid depth projection parameters");
    std::vector<cv::Point2f> pixels;
    for (int y = 1; y < height-1; y += stride)
      for (int x = 1; x < width-1; x += stride) {
        pixels.emplace_back(float(x), float(y));
        coordinates_.push_back({x, y});
      }
    cv::undistortPoints(pixels, rays_, K, D);
  }
  std::vector<CloudPoint> project(const cv::Mat &depth, const cv::Mat &gray) const {
    if (depth.cols != width_ || depth.rows != height_ || gray.size() != depth.size() || gray.type() != CV_8UC1 ||
        (depth.type() != CV_16UC1 && depth.type() != CV_32FC1))
      throw std::invalid_argument("Depth/image format differs from calibrated projection");
    auto read = [&](int y, int x) {
      return (depth.type() == CV_16UC1 ? float(depth.at<uint16_t>(y, x)) : depth.at<float>(y, x))*scale_;
    };
    std::vector<CloudPoint> points;
    points.reserve(rays_.size());
    for (size_t i = 0; i < rays_.size(); ++i) {
      const int x = coordinates_[i][0], y = coordinates_[i][1];
      const float z = read(y, x);
      if (!std::isfinite(z) || z < minimum_ || z > maximum_) continue;
      // Reject pixels bordering holes or depth discontinuities instead of
      // bridging foreground/background surfaces with interpolation.
      bool edge = false;
      for (const auto &offset : neighbors_) {
        const float other = read(y+offset[1], x+offset[0]);
        if (!std::isfinite(other) || other <= 0 || std::abs(other-z) > 0.03f+0.02f*z) { edge = true; break; }
      }
      if (edge) continue;
      points.push_back({rays_[i].x*z, rays_[i].y*z, z, float(gray.at<uint8_t>(y, x))});
    }
    return points;
  }
 private:
  int width_, height_;
  float minimum_, maximum_, scale_;
  std::vector<std::array<int, 2>> coordinates_;
  std::vector<cv::Point2f> rays_;
  const std::array<std::array<int, 2>, 4> neighbors_{{{{-1,0}},{{1,0}},{{0,-1}},{{0,1}}}};
};

struct VoxelKey {
  int64_t x, y, z;
  bool operator==(const VoxelKey &b) const { return x==b.x && y==b.y && z==b.z; }
};
struct VoxelHash {
  size_t operator()(const VoxelKey &p) const {
    return std::hash<int64_t>{}(p.x) ^ (std::hash<int64_t>{}(p.y)<<1) ^ (std::hash<int64_t>{}(p.z)<<2);
  }
};

// Rebuild from keyframe-local samples and the latest optimized poses. A frame
// contributes at most one observation per world voxel, regardless of pixel count.
class VoxelFusion {
 public:
  VoxelFusion(float voxel, size_t limit, unsigned minimum_observations)
      : voxel_(voxel), limit_(limit), minimum_(minimum_observations) {
    if (voxel <= 0 || limit == 0 || minimum_observations == 0)
      throw std::invalid_argument("Invalid voxel map limits");
  }
  void addFrame(uint64_t id, const std::vector<CloudPoint> &points, const CloudTransform &Twc) {
    if (!frames_.insert(id).second) return;
    std::unordered_map<VoxelKey, Sum, VoxelHash> frame;
    for (const auto &camera : points) {
      const auto p = transformPoint(camera, Twc);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity)) continue;
      const double qx = std::floor(p.x/voxel_), qy = std::floor(p.y/voxel_), qz = std::floor(p.z/voxel_);
      if (std::abs(qx)>1e12 || std::abs(qy)>1e12 || std::abs(qz)>1e12) continue;
      const VoxelKey key{int64_t(qx), int64_t(qy), int64_t(qz)};
      if (frame.size() >= limit_ && !frame.count(key)) continue;
      auto &s = frame[key];
      s.x += p.x; s.y += p.y; s.z += p.z; s.i += p.intensity; ++s.n;
    }
    for (const auto &entry : frame) {
      if (world_.size() >= limit_ && !world_.count(entry.first)) continue;
      const auto &p = entry.second;
      auto &s = world_[entry.first];
      s.x += p.x/p.n; s.y += p.y/p.n; s.z += p.z/p.n; s.i += p.i/p.n; ++s.n;
    }
  }
  std::vector<CloudPoint> cloud() const {
    std::vector<CloudPoint> output;
    output.reserve(world_.size());
    for (const auto &entry : world_) {
      const auto &s = entry.second;
      if (s.n >= minimum_)
        output.push_back({float(s.x/s.n),float(s.y/s.n),float(s.z/s.n),float(s.i/s.n)});
    }
    return output;
  }
 private:
  struct Sum { double x=0,y=0,z=0,i=0; unsigned n=0; };
  float voxel_;
  size_t limit_;
  unsigned minimum_;
  std::unordered_set<uint64_t> frames_;
  std::unordered_map<VoxelKey, Sum, VoxelHash> world_;
};

inline void writeCloudPly(const std::string &path, const std::vector<CloudPoint> &cloud) {
  const uint16_t one = 1;
  if (*reinterpret_cast<const uint8_t*>(&one) != 1)
    throw std::runtime_error("PLY writer requires little-endian host");
  std::ofstream out(path+".tmp", std::ios::binary);
  if (!out) throw std::runtime_error("Cannot open PLY output");
  out << "ply\nformat binary_little_endian 1.0\nelement vertex " << cloud.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n";
  out.write(reinterpret_cast<const char*>(cloud.data()), cloud.size()*sizeof(CloudPoint));
  out.close();
  if (!out || std::rename((path+".tmp").c_str(),path.c_str()) != 0)
    throw std::runtime_error("Failed to save PLY output atomically");
}
