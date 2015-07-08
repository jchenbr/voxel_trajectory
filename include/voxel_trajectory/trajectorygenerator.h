
#ifndef _VOXEL_TRAJECTORY_TRAJECTORY_GENERATOR_H_
#define _VOXEL_TRAJECTORY_TRAJECTORY_GENERATOR_H_

#include <eigen3/Eigen/Dense>

#include "voxel_trajectory/voxelmacro.h"

namespace VoxelTrajectory
{
    class TrajectoryGenerator
    {
private:
public:
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> genPolyCoeffTime(
            const Eigen::MatrixXd &PBE,
            const Eigen::MatrixXd &inflated_path,
            const Eigen::MatrixXd &vel,
            const Eigen::MatrixXd &acc,
            const double maxVel,
            const double maxAcc,
            double & coeff_t);
   };
}

#endif
