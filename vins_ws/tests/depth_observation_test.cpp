#include "utility/depth_observation.h"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdexcept>
void require(bool value){if(!value)throw std::runtime_error("depth observation test failed");}
int main(){
 cv::Mat depth(7,7,CV_32FC1,cv::Scalar(2.));
 require(std::abs(sampleMetricDepth(depth,3,3,.25,6.)-2.)<1e-8);
 depth.at<float>(3,3)=0;require(sampleMetricDepth(depth,3,3,.25,6.)<0);
 depth.at<float>(3,3)=2.;depth.at<float>(3,4)=3.;require(sampleMetricDepth(depth,3,3,.25,6.)<0);
 require(sampleMetricDepth(depth,0,0,.25,6.)<0);
 require(sampleMetricDepth(depth,3,3,.25,1.)<0);
 cv::Mat wrong(7,7,CV_16UC1,cv::Scalar(2000));require(sampleMetricDepth(wrong,3,3,.25,6.)<0);
 Eigen::Matrix3d R0=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitY()).toRotationMatrix();
 Eigen::Matrix3d R1=R0*Eigen::AngleAxisd(.003,Eigen::Vector3d::UnitX()).toRotationMatrix();
 Eigen::Vector3d t0(-.00552,.0051,.01174),t1(.04439,.0052,.0119),ray(.12,-.08,1.);
 for(double z:{.3,1.,2.,5.5}){
  auto r=depthObservationInRight(ray,z,R0,t0,R1,t1);
  Eigen::Matrix<double,3,4> P0,P1;
  P0.leftCols<3>()=R0.transpose();P0.rightCols<1>()=-R0.transpose()*t0;
  P1.leftCols<3>()=R1.transpose();P1.rightCols<1>()=-R1.transpose()*t1;
  Eigen::Matrix4d A;
  A.row(0)=ray.x()*P0.row(2)-P0.row(0);A.row(1)=ray.y()*P0.row(2)-P0.row(1);
  A.row(2)=r.x()/r.z()*P1.row(2)-P1.row(0);A.row(3)=r.y()/r.z()*P1.row(2)-P1.row(1);
  Eigen::Vector4d h=A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
  Eigen::Vector3d recovered=R0.transpose()*(h.head<3>()/h.w()-t0);
  require(std::abs(recovered.z()-z)<1e-8);
 }
 std::cout<<"PASS: metric sampling, holes, edge rejection, units/types, bounds and measured-depth geometry round-trip\n";
}
