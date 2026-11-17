#pragma once
#include <Eigen/Dense>

inline bool trackingStateFailed(const Eigen::Vector3d &position,
        const Eigen::Vector3d &previous_position, const Eigen::Matrix3d &rotation,
        const Eigen::Vector3d &velocity, const Eigen::Vector3d &acc_bias,
        const Eigen::Vector3d &gyro_bias, int tracks, bool use_imu, int &weak_frames)
{
    weak_frames = tracks < 6 ? weak_frames+1 : 0;
    return !position.allFinite() || !previous_position.allFinite() || !rotation.allFinite()
        || !velocity.allFinite() || !acc_bias.allFinite() || !gyro_bias.allFinite()
        || weak_frames >= 3 || (position-previous_position).norm() > 2.0
        || (use_imu && (acc_bias.norm() > 2.5 || gyro_bias.norm() > 1.0));
}
