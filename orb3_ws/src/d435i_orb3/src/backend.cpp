#include "backend.hpp"
#include <System.h>
#include <memory>
#include <Map.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <map>
#include <deque>
#include <mutex>
#include <limits>

class Backend::Impl {
 public:
  Impl(const std::string &vocabulary, const std::string &settings, bool use_imu, bool viewer,
       const MappingOptions &options) : inertial(use_imu), mapping(options) {
    if (mapping.enabled) {
      if (mapping.max_keyframes < 1 || mapping.max_points < 1 || mapping.update_interval <= 0 || mapping.min_observations < 1)
        throw std::invalid_argument("Invalid mapping limits");
    }
    if (mapping.enabled && mapping.dense_enabled) {
      cv::FileStorage fs(settings, cv::FileStorage::READ);
      cv::Mat K = cv::Mat::eye(3,3,CV_64F), D = cv::Mat::zeros(1,5,CV_64F);
      K.at<double>(0,0) = double(fs["Camera1.fx"]); K.at<double>(1,1) = double(fs["Camera1.fy"]);
      K.at<double>(0,2) = double(fs["Camera1.cx"]); K.at<double>(1,2) = double(fs["Camera1.cy"]);
      const char *names[] = {"Camera1.k1","Camera1.k2","Camera1.p1","Camera1.p2","Camera1.k3"};
      for (int i=0;i<5;++i) if (!fs[names[i]].empty()) D.at<double>(i)=double(fs[names[i]]);
      projector.reset(new DepthProjector(K,D,int(fs["Camera.width"]),int(fs["Camera.height"]),
        mapping.pixel_stride,mapping.min_depth,mapping.max_depth,float(fs["RGBD.DepthMapFactor"])));
    }
    system.reset(new ORB_SLAM3::System(vocabulary, settings,
      use_imu ? ORB_SLAM3::System::IMU_RGBD : ORB_SLAM3::System::RGBD, viewer));
  }
  using Points = std::shared_ptr<const std::vector<CloudPoint>>;
  struct DepthFrame { double time; Points cloud; };
  struct DepthKeyframe { ORB_SLAM3::KeyFrame *frame; double time; Points cloud; };
  static CloudTransform transform(const Sophus::SE3f &pose) {
    const auto matrix=pose.matrix();
    CloudTransform out{};
    for(int row=0;row<3;++row) for(int col=0;col<4;++col) out[row*4+col]=matrix(row,col);
    return out;
  }
  void resetMapping() {
    recent.clear(); cached.clear(); map_id=-1;
    last_publish=-std::numeric_limits<double>::infinity();
  }
  void updateMapping(const cv::Mat &image, const cv::Mat &depth, double time,
                     const Sophus::SE3f &Twc, BackendResult &result) {
    if (!mapping.enabled) return;
    if (result.state<=1 || result.state==4) {
      result.cloud_reset=map_id>=0 || !cached.empty();
      resetMapping();
      return;
    }
    if (result.state!=2 || !result.finite) return;
    ORB_SLAM3::Map *active=nullptr;
    for (auto *point:system->GetTrackedMapPoints()) {
      if (point && !point->isBad()) { active=point->GetMap(); if(active) break; }
    }
    if (!active) return;
    const int64_t current_id=static_cast<int64_t>(active->GetId());
    if (map_id!=current_id) {
      result.cloud_reset=map_id>=0;
      resetMapping(); map_id=current_id;
    }
    result.map_id=map_id;
    Points points;
    if(mapping.dense_enabled) {
      points=std::make_shared<const std::vector<CloudPoint>>(projector->project(depth,image));
      recent.push_back({time,points});
      while(recent.size()>120) recent.pop_front();
    }
    if (time-last_publish<mapping.update_interval) return;
    last_publish=time;
    std::vector<std::pair<Points,CloudTransform>> views;
    {
      // Copy a consistent pose/landmark snapshot, then do voxel fusion outside
      // the SLAM map lock so rendering work does not block bundle adjustment.
      std::unique_lock<std::mutex> lock(active->mMutexMapUpdate);
      const auto keyframes=active->GetAllKeyFrames();
      result.map_keyframes=keyframes.size();
      for(auto *keyframe:keyframes) {
        if(!keyframe || keyframe->isBad() || cached.count(keyframe->mnId)) continue;
        for(auto it=recent.rbegin();it!=recent.rend();++it) {
          if(std::abs(it->time-keyframe->mTimeStamp)<1e-6) {
            cached.emplace(keyframe->mnId,DepthKeyframe{keyframe,keyframe->mTimeStamp,it->cloud});
            break;
          }
        }
      }
      while(cached.size()>mapping.max_keyframes) cached.erase(cached.begin());
      result.cached_depth_keyframes=cached.size();
      // Newest keyframes take priority if the global voxel capacity is reached.
      for(auto it=cached.rbegin();it!=cached.rend();++it) {
        auto *frame=it->second.frame;
        Sophus::SE3f relative;
        unsigned parents=0;
        while(frame && frame->isBad() && parents++<100) {
          relative=relative*frame->mTcp;
          frame=frame->GetParent();
        }
        if(!frame || frame->isBad() || frame->GetMap()!=active) continue;
        views.push_back({it->second.cloud,transform((relative*frame->GetPose()).inverse())});
      }
      for(auto *point:active->GetAllMapPoints()) {
        if(!point || point->isBad()) continue;
        const auto p=point->GetWorldPos();
        if(p.allFinite()) result.sparse_cloud.push_back({p.x(),p.y(),p.z(),255.f});
        if(result.sparse_cloud.size()>=mapping.max_points) break;
      }
    }
    if(mapping.dense_enabled) {
      VoxelFusion fusion(mapping.voxel_size,mapping.max_points,mapping.min_observations);
      uint64_t view=0;
      for(const auto &entry:views) fusion.addFrame(++view,*entry.first,entry.second);
      result.surface_cloud=fusion.cloud();
      const auto current_transform=transform(Twc);
      result.depth_cloud.reserve(points->size());
      for(const auto &point:*points) result.depth_cloud.push_back(transformPoint(point,current_transform));
    }
    result.cloud_updated=true;
  }
  bool inertial;
  bool stopped = false;
  std::unique_ptr<ORB_SLAM3::System> system;
  MappingOptions mapping;
  std::unique_ptr<DepthProjector> projector;
  std::deque<DepthFrame> recent;
  std::map<uint64_t,DepthKeyframe> cached;
  int64_t map_id=-1;
  double last_publish=-std::numeric_limits<double>::infinity();
};

Backend::Backend(const std::string &vocabulary, const std::string &settings, bool inertial, bool viewer,
                 const MappingOptions &mapping)
    : impl_(new Impl(vocabulary, settings, inertial, viewer, mapping)) {}

Backend::~Backend() {
  if (!impl_->stopped) impl_->system->Shutdown();
  delete impl_;
}

BackendResult Backend::track(const cv::Mat &image, const cv::Mat &depth,
                            double time, const std::vector<ImuSample> &samples) {
  std::vector<ORB_SLAM3::IMU::Point> points;
  for (const auto &s : samples)
    points.emplace_back(s.a[0], s.a[1], s.a[2], s.w[0], s.w[1], s.w[2], s.t);
  const auto Tcw = impl_->system->TrackRGBD(image, depth, time, points);
  BackendResult result;
  result.state = impl_->system->GetTrackingState();
  result.imu_initialized = impl_->inertial && result.state == 2 &&
      impl_->system->GetTimeFromIMUInit() > 0.0;
  const auto Twc = Tcw.inverse();
  if (result.state == 2) {
    const Eigen::Vector3f p = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();
    result.finite = p.allFinite() && q.coeffs().allFinite();
    result.position = {{p.x(), p.y(), p.z()}};
    result.quaternion = {{q.x(), q.y(), q.z(), q.w()}};
  }
  impl_->updateMapping(image,depth,time,Twc,result);
  return result;
}

void Backend::clearMapping() { impl_->resetMapping(); }

void Backend::shutdown(const std::string &directory, bool save) {
  if (impl_->stopped) return;
  impl_->system->Shutdown();
  impl_->stopped = true;
  // Upstream SaveTrajectoryTUM indexes the first keyframe unconditionally.
  if (save && impl_->system->GetTrackingState() == 2) {
    impl_->system->SaveTrajectoryTUM(directory + "/optimized_trajectory.tum");
    impl_->system->SaveKeyFrameTrajectoryTUM(directory + "/keyframes.tum");
  }
}
