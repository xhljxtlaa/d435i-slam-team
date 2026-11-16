#include "../src/d435i_orb3/src/cloud_geometry.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <iterator>

int main() {
  const cv::Mat K=(cv::Mat_<double>(3,3)<<2,0,2,0,2,2,0,0,1);
  const cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
  cv::Mat depth(6,6,CV_16UC1,cv::Scalar(1000)), gray(6,6,CV_8UC1,cv::Scalar(128));
  DepthProjector projector(K,D,6,6,1,0.18f,6.0f,1000.f);
  auto cloud=projector.project(depth,gray);
  assert(cloud.size()==16);
  bool found=false;
  for(const auto &p:cloud) if(std::abs(p.x-.5f)<1e-6 && std::abs(p.y)<1e-6) {
    assert(std::abs(p.z-1.f)<1e-6 && p.intensity==128); found=true;
  }
  assert(found);
  depth.at<uint16_t>(2,3)=0;
  assert(projector.project(depth,gray).size()==11); // hole and four edge neighbors
  cv::Mat metres(6,6,CV_32FC1,cv::Scalar(1.f));
  DepthProjector float_projector(K,D,6,6,1,0.18f,6.0f,1.f);
  assert(float_projector.project(metres,gray).size()==16);
  metres.at<float>(2,3)=std::numeric_limits<float>::quiet_NaN();
  assert(float_projector.project(metres,gray).size()==11);
  metres.setTo(7.f); assert(float_projector.project(metres,gray).empty());

  // Residual distortion must be removed from rays, while depth remains sampled
  // at the original pixel. Reprojection should recover that original grid.
  cv::Mat distortion=(cv::Mat_<double>(1,5)<<0.01,-0.001,0.001,0.002,0.0);
  metres.setTo(1.f);
  DepthProjector distorted(K,distortion,6,6,1,0.18f,6.0f,1.f);
  auto corrected=distorted.project(metres,gray);
  std::vector<cv::Point3f> points;
  for(const auto &p:corrected) points.emplace_back(p.x,p.y,p.z);
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,distortion,pixels);
  assert(pixels.size()==16);
  for(size_t i=0;i<pixels.size();++i) {
    assert(std::abs(pixels[i].x-float(1+i%4))<1e-4);
    assert(std::abs(pixels[i].y-float(1+i/4))<1e-4);
  }

  const CloudTransform identity{{1,0,0,0,0,1,0,0,0,0,1,0}};
  const CloudTransform rotation{{0,-1,0,1,1,0,0,0,0,0,1,0}};
  const auto world=transformPoint({.5f,0,1,128},rotation);
  assert(world.x==1 && world.y==.5f && world.z==1);
  const std::vector<CloudPoint> repeated{{.01f,.01f,1.f,50},{.02f,.02f,1.f,100}};
  VoxelFusion confirmed(.1f,100,3);
  confirmed.addFrame(1,repeated,identity); confirmed.addFrame(1,repeated,identity);
  assert(confirmed.cloud().empty());
  confirmed.addFrame(2,repeated,identity); assert(confirmed.cloud().empty());
  confirmed.addFrame(3,repeated,identity); assert(confirmed.cloud().size()==1);
  assert(std::abs(confirmed.cloud()[0].intensity-75.f)<1e-6);

  // Corrected keyframe poses rebuild the map, rather than leaving a ghost copy
  // at the old world position.
  VoxelFusion before(.1f,100,1), after(.1f,100,1);
  CloudTransform moved=identity; moved[3]=1.f;
  before.addFrame(1,repeated,identity); after.addFrame(1,repeated,moved);
  assert(before.cloud().size()==1 && after.cloud().size()==1);
  assert(std::abs(after.cloud()[0].x-before.cloud()[0].x-1.f)<1e-6);
  VoxelFusion bounded(.1f,5,1);
  std::vector<CloudPoint> many;
  for(int i=0;i<100;++i) many.push_back({float(i),0,1,100});
  many.push_back({std::numeric_limits<float>::infinity(),0,1,100});
  bounded.addFrame(1,many,identity); assert(bounded.cloud().size()==5);
  const std::string path="/tmp/orb3_geometry_test.ply";
  writeCloudPly(path,after.cloud());
  std::ifstream input(path,std::ios::binary);
  const std::string content((std::istreambuf_iterator<char>(input)),std::istreambuf_iterator<char>());
  const auto header=content.find("end_header\n")+11;
  assert(content.find("element vertex 1\n")!=std::string::npos);
  assert(content.size()-header==16);
  std::remove(path.c_str());
  std::cout<<"Depth units/rays, distortion, transforms, per-keyframe confirmations, pose correction, bounds and PLY: PASS\n";
}
