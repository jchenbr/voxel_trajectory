
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
            const Eigen::MatrixXd &vel = Eigen::MatrixXd::Zero(_TOT_DIM, 2),
            const Eigen::MatrixXd &acc = Eigen::MatrixXd::Zero(_TOT_DIM, 2),
            const double maxVel = 1.0,
            const double maxAcc = 1.0);
   };
}

#endif
