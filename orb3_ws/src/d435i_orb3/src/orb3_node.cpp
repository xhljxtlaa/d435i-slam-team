#include "backend.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <cstring>
#include <sstream>
#include "imu_buffer.hpp"
#include "trajectory_continuity.hpp"

using Image = sensor_msgs::msg::Image;
using PairPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

class OrbNode : public rclcpp::Node {
 public:
  OrbNode() : Node("d435i_orb3") {
    const auto vocabulary = declare_parameter<std::string>("vocabulary", "");
    const auto settings = declare_parameter<std::string>("settings", "");
    const auto mode = declare_parameter<std::string>("mode", "rgbd_imu");
    if (mode != "rgbd" && mode != "rgbd_imu")
      throw std::runtime_error("mode must be rgbd or rgbd_imu");
    inertial_ = mode == "rgbd_imu";
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/camera/infra1/image_rect_raw");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
    frame_ = declare_parameter<std::string>("map_frame", "orb3_map");
    output_ = declare_parameter<std::string>("output_dir", ".");
    max_sync_ = declare_parameter<double>("max_sync_seconds", 0.005);
    if (max_sync_ <= 0.0) throw std::runtime_error("max_sync_seconds must be positive");
    if (!std::ifstream(vocabulary) || !std::ifstream(settings))
      throw std::runtime_error("Vocabulary/settings missing. Run tools/deploy.sh and start.sh");
    cv::FileStorage config(settings, cv::FileStorage::READ);
    const double factor = static_cast<double>(config["RGBD.DepthMapFactor"]);
    if (std::abs(factor - 1000.0) < 1e-6) expected_depth_ = "16UC1";
    else if (std::abs(factor - 1.0) < 1e-6) expected_depth_ = "32FC1";
    else throw std::runtime_error("Expected millimetre 16UC1 or metre 32FC1 depth settings");
    live_.open(output_ + "/camera_pose.tum");
    if (!live_) throw std::runtime_error("Cannot write output_dir");
    const bool viewer = declare_parameter<bool>("viewer", false);
    MappingOptions mapping;
    mapping.enabled=declare_parameter<bool>("mapping",true);
    mapping.dense_enabled=declare_parameter<bool>("dense_mapping",false);
    sparse_only_=!mapping.dense_enabled;
    mapping.voxel_size=float(declare_parameter<double>("map_voxel_size",0.05));
    mapping.min_depth=float(declare_parameter<double>("map_min_depth",0.18));
    mapping.max_depth=float(declare_parameter<double>("map_max_depth",6.0));
    mapping.pixel_stride=declare_parameter<int>("map_pixel_stride",8);
    const int max_points=declare_parameter<int>("map_max_points",100000);
    const int max_keyframes=declare_parameter<int>("map_max_keyframes",200);
    const int observations=declare_parameter<int>("map_min_observations",1);
    const double map_hz=declare_parameter<double>("map_publish_hz",1.0);
    if(max_points<1 || max_keyframes<1 || observations<1 || map_hz<=0)
      throw std::invalid_argument("Invalid point-cloud map limits");
    mapping.max_points=size_t(max_points); mapping.max_keyframes=size_t(max_keyframes);
    mapping.min_observations=unsigned(observations); mapping.update_interval=1.0/map_hz;
    map_voxel_=mapping.voxel_size;
    backend_.reset(new Backend(vocabulary, settings, inertial_, viewer,mapping));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orb3/pose", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/orb3/odometry", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/orb3/path", 1);
    state_pub_ = create_publisher<std_msgs::msg::Int32>("/orb3/tracking_state", 1);
    imu_init_pub_ = create_publisher<std_msgs::msg::Bool>("/orb3/imu_initialized", 1);
    segment_pub_ = create_publisher<std_msgs::msg::Int32>("/orb3/trajectory_segment", 1);
    auto map_qos=rclcpp::QoS(1).reliable().transient_local();
    surface_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/orb3/map_cloud",map_qos);
    sparse_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/orb3/map_points",map_qos);
    depth_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/orb3/depth_points",map_qos);
    save_service_=create_service<std_srvs::srv::Trigger>("~/save_map",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        try {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          const auto &points=sparse_only_?sparse_:surface_;
          if(points.empty()) throw std::runtime_error("No map points available yet");
          saveClouds("map"); response->success=true;
          response->message="Saved "+std::to_string(points.size())+" map points to "+output_+"/map.ply";
        } catch(const std::exception &e) { response->success=false; response->message=e.what(); }
      });
    clear_service_=create_service<std_srvs::srv::Trigger>("~/clear_map",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        if(sparse_only_) {
          response->success=false;
          response->message="Feature map is managed by SLAM; restart localization to reset it";
          return;
        }
        clear_map_requested_=true; response->success=true;
        response->message="Clear queued for next image; new keyframes will rebuild the surface map";
      });
    stats_service_=create_service<std_srvs::srv::Trigger>("~/map_stats",
      [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        std::ostringstream text;
        text<<"{\"representation\":\""<<(sparse_only_?"orb_features":"depth_surface")
            <<"\",\"surface_points\":"<<surface_.size()<<",\"sparse_points\":"<<sparse_.size()
            <<",\"map_id\":"<<cloud_map_id_<<",\"map_keyframes\":"<<map_keyframes_
            <<",\"cached_depth_keyframes\":"<<depth_keyframes_<<",\"map_epoch\":"<<cloud_epoch_
            <<",\"voxel_size_m\":"<<map_voxel_<<",\"output_dir\":\""<<output_<<"\"}";
        response->success=true; response->message=text.str();
      });
    path_.header.frame_id = frame_;
    if (inertial_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          ImuSample s{rclcpp::Time(m->header.stamp).seconds(),
            {{float(m->linear_acceleration.x), float(m->linear_acceleration.y), float(m->linear_acceleration.z)}},
            {{float(m->angular_velocity.x), float(m->angular_velocity.y), float(m->angular_velocity.z)}}};
          std::lock_guard<std::mutex> lock(mutex_);
          if (!imu_.push(s)) ++bad_imu_;
          changed_.notify_all();
        });
    }
    auto image_qos = rclcpp::SensorDataQoS();
    if (declare_parameter<bool>("reliable_images", true)) image_qos.reliable();
    image_sub_.subscribe(this, image_topic, image_qos.get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic, image_qos.get_rmw_qos_profile());
    sync_.reset(new message_filters::Synchronizer<PairPolicy>(PairPolicy(10), image_sub_, depth_sub_));
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_));
    sync_->registerCallback(std::bind(&OrbNode::onImages, this,
      std::placeholders::_1, std::placeholders::_2));
    running_ = true;
    worker_ = std::thread([this] { process(); });
    RCLCPP_INFO(get_logger(), "mode=%s; image=%s; depth=%s; IMU=%s", mode.c_str(), image_topic.c_str(), depth_topic.c_str(), imu_topic.c_str());
  }
  ~OrbNode() override {
    running_ = false;
    changed_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (backend_) backend_->shutdown(output_, tracked_ > 0);
    try {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if(!surface_.empty() || !sparse_.empty()) saveClouds("map");
    } catch(const std::exception &e) { RCLCPP_ERROR(get_logger(),"Map save failed: %s",e.what()); }
    RCLCPP_INFO(get_logger(), "Tracked=%lu, queue drops=%lu, rejected IMU=%lu", tracked_, dropped_, bad_imu_);
  }
 private:
  sensor_msgs::msg::PointCloud2 cloudMessage(const std::vector<CloudPoint> &points,
                                            const std_msgs::msg::Header &header) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header=header; msg.header.frame_id=frame_;
    msg.height=1; msg.width=uint32_t(points.size()); msg.point_step=16; msg.row_step=msg.width*16;
    msg.is_dense=true; msg.is_bigendian=false;
    const char *names[]={"x","y","z","intensity"};
    for(unsigned i=0;i<4;++i) {
      sensor_msgs::msg::PointField field; field.name=names[i]; field.offset=i*4;
      field.datatype=sensor_msgs::msg::PointField::FLOAT32; field.count=1; msg.fields.push_back(field);
    }
    msg.data.resize(points.size()*sizeof(CloudPoint));
    if(!points.empty()) std::memcpy(msg.data.data(),points.data(),msg.data.size());
    return msg;
  }
  // Caller holds cloud_mutex_. Save the latest consistent published snapshot.
  void saveClouds(const std::string &name) {
    writeCloudPly(output_+"/"+name+".ply",sparse_only_?sparse_:surface_);
    writeCloudPly(output_+"/"+name+"_sparse.ply",sparse_);
    std::ofstream info(output_+"/"+name+".json");
    info<<"{\"representation\":\""<<(sparse_only_?"orb_features":"depth_surface")
        <<"\",\"frame\":\""<<frame_<<"\",\"map_id\":"<<cloud_map_id_
        <<",\"surface_points\":"<<surface_.size()<<",\"sparse_points\":"<<sparse_.size()
        <<",\"cached_depth_keyframes\":"<<depth_keyframes_<<",\"voxel_size_m\":"<<map_voxel_<<"}\n";
  }
  void receiveClouds(const BackendResult &result, const std_msgs::msg::Header &header, bool manual_clear) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if(result.cloud_reset || manual_clear) {
      if(!surface_.empty() || !sparse_.empty()) {
        try { saveClouds("map_epoch_"+std::to_string(cloud_epoch_)); }
        catch(const std::exception &e) { RCLCPP_ERROR(get_logger(),"Old map save failed: %s",e.what()); }
      }
      ++cloud_epoch_; surface_.clear(); sparse_.clear();
      cloud_map_id_=-1; map_keyframes_=0; depth_keyframes_=0;
      surface_pub_->publish(cloudMessage({},header));
      sparse_pub_->publish(cloudMessage({},header));
      depth_pub_->publish(cloudMessage({},header));
    }
    if(!result.cloud_updated) return;
    surface_=result.surface_cloud; sparse_=result.sparse_cloud;
    cloud_map_id_=result.map_id; map_keyframes_=result.map_keyframes; depth_keyframes_=result.cached_depth_keyframes;
    surface_pub_->publish(cloudMessage(surface_,header));
    sparse_pub_->publish(cloudMessage(sparse_,header));
    depth_pub_->publish(cloudMessage(result.depth_cloud,header));
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
      "Map=%ld; depth keyframes=%zu; surface points=%zu; SLAM landmarks=%zu",
      cloud_map_id_,depth_keyframes_,surface_.size(),sparse_.size());
  }
  void onImages(Image::ConstSharedPtr a, Image::ConstSharedPtr b) {
    if (a->encoding != "mono8" || b->encoding != expected_depth_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Image encoding differs from captured settings; restart to capture configuration");
      return;
    }
    const double dt = std::abs((rclcpp::Time(a->header.stamp) - rclcpp::Time(b->header.stamp)).seconds());
    if (dt > max_sync_ || a->width != b->width || a->height != b->height) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (images_.size() >= 3) { images_.pop_front(); ++dropped_; }
    images_.push_back({a, b});
    changed_.notify_all();
  }
  void process() {
    double previous = -1.0;
    while (running_) {
      Image::ConstSharedPtr image, depth;
      std::vector<ImuSample> samples;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [this] { return !running_ || !images_.empty(); });
        if (!running_) break;
        image = images_.front().first;
        depth = images_.front().second;
        images_.pop_front();
        const double t = rclcpp::Time(image->header.stamp).seconds();
        if (t <= previous) continue;
        if (inertial_) {
          const bool received = changed_.wait_for(lock, std::chrono::milliseconds(500),
            [this, t] { return !running_ || imu_.ready(t) || imu_.stale(t); });
          if (!running_) break;
          if (!received || !imu_.ready(t)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for IMU coverage: check topic timestamps/frequency");
            continue;
          }
          samples = imu_.take(t, previous < 0 ? t - 0.2 : previous);
        }
      }
      try {
        if (depth->encoding != "16UC1" && depth->encoding != "32FC1")
          throw std::runtime_error("Depth must be 16UC1 or 32FC1");
        const auto gray = cv_bridge::toCvShare(image, "mono8");
        const auto z = cv_bridge::toCvShare(depth);
        const double t = rclcpp::Time(image->header.stamp).seconds();
        const bool manual_clear=clear_map_requested_.exchange(false);
        if(manual_clear) backend_->clearMapping();
        const auto result = backend_->track(gray->image, z->image, t, samples);
        receiveClouds(result,image->header,manual_clear);
        previous = t;
        const int state = result.state;
        std_msgs::msg::Int32 state_msg;
        state_msg.data = state;
        state_pub_->publish(state_msg);
        std_msgs::msg::Bool initialized;
        initialized.data = result.imu_initialized;
        imu_init_pub_->publish(initialized);
        if (continuity_.update(state, initialized.data, inertial_) || result.cloud_reset) {
          ++segment_;
          path_.poses.clear();
          path_.header.stamp = image->header.stamp;
          last_path_ = -1.0;
          path_pub_->publish(path_);
          live_ << "# segment " << segment_ << " at " << std::fixed << std::setprecision(9)
            << t << " (tracking reset or inertial world alignment)\n";
          live_.flush();
          RCLCPP_WARN(get_logger(), "Starting trajectory segment %d after tracking reset/inertial alignment", segment_);
        }
        std_msgs::msg::Int32 segment_msg;
        segment_msg.data = segment_;
        segment_pub_->publish(segment_msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Tracking state=%d (2=OK); IMU initialized=%d; output poses=%lu", state, initialized.data, tracked_);
        if (state != 2) continue;
        if (!result.finite) continue;
        const auto &p = result.position;
        const auto &q = result.quaternion;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = image->header;
        pose.header.frame_id = frame_;
        pose.pose.position.x = p[0]; pose.pose.position.y = p[1]; pose.pose.position.z = p[2];
        pose.pose.orientation.x = q[0]; pose.pose.orientation.y = q[1];
        pose.pose.orientation.z = q[2]; pose.pose.orientation.w = q[3];
        pose_pub_->publish(pose);
        nav_msgs::msg::Odometry odom;
        odom.header = pose.header;
        odom.child_frame_id = "orb3_camera_optical";
        odom.pose.pose = pose.pose;
        // Pose-only odometry: twist/covariance are not estimated by this adapter.
        odom_pub_->publish(odom);
        live_ << std::fixed << std::setprecision(9) << t << " " << p[0] << " " << p[1] << " " << p[2]
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "\n";
        if (++tracked_ % 10 == 0) live_.flush();
        if (t - last_path_ >= 0.2) {
          last_path_ = t;
          path_.header.stamp = pose.header.stamp;
          path_.poses.push_back(pose);
          if (path_.poses.size() > 3000) path_.poses.erase(path_.poses.begin());
          path_pub_->publish(path_);
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Frame failed: %s", e.what());
      }
    }
  }
  bool inertial_ = true;
  std::atomic<bool> running_{false};
  std::atomic<bool> clear_map_requested_{false};
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable changed_;
  ImuBuffer imu_;
  std::deque<std::pair<Image::ConstSharedPtr, Image::ConstSharedPtr>> images_;
  std::unique_ptr<Backend> backend_;
  message_filters::Subscriber<Image> image_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<PairPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_init_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr segment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_pub_, sparse_pub_, depth_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_,clear_service_,stats_service_;
  std::mutex cloud_mutex_;
  std::vector<CloudPoint> surface_, sparse_;
  int64_t cloud_map_id_=-1;
  size_t map_keyframes_=0,depth_keyframes_=0;
  unsigned cloud_epoch_=0;
  float map_voxel_=0.01f;
  bool sparse_only_=true;
  TrajectoryContinuity continuity_;
  int segment_ = 0;
  nav_msgs::msg::Path path_;
  std::string frame_, output_, expected_depth_;
  std::ofstream live_;
  double max_sync_ = 0.005, last_path_ = -1.0;
  uint64_t tracked_ = 0, dropped_ = 0, bad_imu_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OrbNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception &e) {
    std::cerr << "ORB-SLAM3 startup failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
