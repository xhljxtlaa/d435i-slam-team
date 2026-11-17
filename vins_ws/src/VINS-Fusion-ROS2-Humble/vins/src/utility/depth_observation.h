#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

// Input is a metric Z-depth image registered to the left infrared pixel grid.
// Reject holes and discontinuities rather than filling foreground/background edges.
inline double sampleMetricDepth(const cv::Mat &depth, double u, double v,
                                double near_limit, double far_limit)
{
    if (depth.type() != CV_32FC1 || !std::isfinite(u) || !std::isfinite(v)) return -1;
    int x = cvRound(u), y = cvRound(v);
    if (x < 1 || y < 1 || x >= depth.cols-1 || y >= depth.rows-1) return -1;
    const double center = depth.at<float>(y,x);
    if (!std::isfinite(center) || center < near_limit || center > far_limit) return -1;
    std::vector<double> values;
    for (int j=-1;j<=1;++j) for(int i=-1;i<=1;++i) {
        double z=depth.at<float>(y+j,x+i);
        if(std::isfinite(z) && z>=near_limit && z<=far_limit) values.push_back(z);
    }
    if(values.size()<5) return -1;
    std::sort(values.begin(),values.end());
    double median=values[values.size()/2], tolerance=std::max(0.03,0.03*median);
    if(values.back()-values.front()>tolerance || std::abs(center-median)>tolerance) return -1;
    return median;
}

// Encode a sensor depth as an equivalent stereo observation. This supplies the
// existing stereo factors (including marginalization) with measured metric depth,
// without an independently matched right-image feature or temporal triangulation.
inline Eigen::Vector3d depthObservationInRight(const Eigen::Vector3d &left_ray,
        double depth, const Eigen::Matrix3d &R0, const Eigen::Vector3d &t0,
        const Eigen::Matrix3d &R1, const Eigen::Vector3d &t1)
{
    return R1.transpose()*(R0*(left_ray/left_ray.z()*depth)+t0-t1);
}
