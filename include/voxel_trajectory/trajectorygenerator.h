
#ifndef _VOXEL_TRAJECTORY_TRAJECTORY_GENERATOR_H_
#define _VOXEL_TRAJECTORY_TRAJECTORY_GENERATOR_H_

#include "voxel_trajectory/voxelmacro.h"

#include <eigen3/Eigen/Dense>
#include <vector>

namespace VoxelTrajectory
{
    class TrajectoryGenerator
    {
private:
public:
	std::vector<double> qp_cost;
	
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> genPolyCoeffTime(
            const Eigen::MatrixXd &PBE,
            const Eigen::MatrixXd &inflated_path,
            const Eigen::MatrixXd &vel,
            const Eigen::MatrixXd &acc,
            const double maxVel,
            const double maxAcc,
            const double fVel,
            const double fAcc,
            double & coeff_t);
            
        std::pair<Eigen::MatrixXd,Eigen::VectorXd> genPolyCoeffTime(
            const Eigen::MatrixXd &PBE,
            const Eigen::MatrixXd &inflated_path,
            const Eigen::MatrixXd &vel,
            const Eigen::MatrixXd &acc,
            const double maxVel,
            const double maxAcc,
            const double fVel,
            const double fAcc,
            std::vector<double> & arr_time,
            double & coeff_t);
   };
}

#endif
