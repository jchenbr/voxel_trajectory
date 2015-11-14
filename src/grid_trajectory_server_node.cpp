#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "voxel_trajectory/voxelmacro.h"
#include <eigen3/Eigen/Dense>

using namespace std;

class TrajectoryServer
{
private:

    // Subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _traj_sub;

    // publishers
    ros::Publisher _cmd_pub;
    
    // configuration for trajectory
    int _n_order = 6;
    int _n_segment = 0;
    int _traj_id = 0;
    uint32_t _traj_flag = 0;
    Eigen::VectorXd _time;
    Eigen::MatrixXd _coef[_TOT_DIM];
    ros::Time _final_time = ros::TIME_MIN;
    ros::Time _start_time = ros::TIME_MAX;
    double _start_yaw = 0.0, _final_yaw = 0.0;

    // state of the server
    enum ServerState{INIT, TRAJ, HOVER} state = INIT;
    nav_msgs::Odometry _odom;
    quadrotor_msgs::PositionCommand _cmd;

public:
    TrajectoryServer(ros::NodeHandle & handle)
    {
        _odom_sub = 
            handle.subscribe("odometry", 50, &TrajectoryServer::rcvOdometryCallback, this);

        _traj_sub =
            handle.subscribe("trajectory", 2, &TrajectoryServer::rcvTrajectoryCallabck, this);

        _cmd_pub = 
            handle.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);

        double pos_gain[_TOT_DIM] = {3.7, 3.7, 5.2};
        double vel_gain[_TOT_DIM] = {2.4, 2.4, 3.0};
        setGains(pos_gain, vel_gain);
    }

    void setGains(double pos_gain[_TOT_DIM], double vel_gain[_TOT_DIM])
    {
        _cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        _cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        _cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        _cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        _cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        _cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    void rcvOdometryCallback(const nav_msgs::Odometry & odom)
    {
        if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return ;
        // #1. store the odometry
        _odom = odom;

        // #2. try to publish command
        pubPositionCommand();

        // #3. try to calculate the new state
        if (state == TRAJ && odom.header.stamp > _final_time)
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
    }

    void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory & traj)
    {
        ROS_WARN("[SERVER] Recevied The Trajectory with %.3lf.", _start_time.toSec());
        // #1. try to execuse the action
        if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
        {
            ROS_WARN("[SERVER] Loading the trajectory.");
            if ((int)traj.trajectory_id < _traj_id) return ;

            state = TRAJ;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

           _traj_id = traj.trajectory_id;

            _n_order = traj.num_order;
            _n_segment = traj.num_segment;

            _coef[_DIM_x].resize(_n_order, _n_segment);
            _coef[_DIM_y].resize(_n_order, _n_segment);
            _coef[_DIM_z].resize(_n_order, _n_segment);
            _time.resize(_n_segment);

            _final_time = _start_time = traj.header.stamp;
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                _final_time += ros::Duration(traj.time[idx]);
                _time(idx) = traj.time[idx];

                for (int j = 0; j < _n_order; ++j)
                {
                    _coef[_DIM_x](j, idx) = traj.coef_x[idx * _n_order + j];
                    _coef[_DIM_y](j, idx) = traj.coef_y[idx * _n_order + j];
                    _coef[_DIM_z](j, idx) = traj.coef_z[idx * _n_order + j];
                }
            }

            _start_yaw = traj.start_yaw;
            _final_yaw = traj.final_yaw;

            //ROS_WARN("[SERVER] Finished the loading.");
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) 
        {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        }
        // #2. try to store the trajectory if the case
    }

    void pubPositionCommand()
    {
        // #1. check if it is right state
        if (state == INIT) return;
        if (state == HOVER)
        {
            if (_cmd.header.frame_id != "/map")
            {
                _cmd.position = _odom.pose.pose.position;
            }

            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "/map";
            _cmd.trajectory_flag = _traj_flag;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;
            
            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;
        }
        // #2. locate the trajectory segment
        if (state == TRAJ)
        {
            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "/map";
            _cmd.trajectory_flag = _traj_flag;
            _cmd.trajectory_id = _traj_id;

            double t = max(0.0, (_odom.header.stamp - _start_time).toSec());
            if (_odom.header.stamp > _final_time) t = (_final_time - _start_time).toSec();

            _cmd.yaw_dot = 0.0;
            _cmd.yaw = _start_yaw + (_final_yaw - _start_yaw) * t 
                / ((_final_time - _start_time).toSec() + 1e-9);

            // #3. calculate the desired states
            //ROS_WARN("[SERVER] the time : %.3lf\n, n = %d, m = %d", t, _n_order, _n_segment);
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                if (t > _time[idx] && idx + 1 < _n_segment)
                {
                    t -= _time[idx];
                }
                else
                {
                    //ROS_WARN("[SERVER] the remained time : %.3lf\n", t);
                    Eigen::VectorXd _T(_n_order), _vec;
                    _T(0) = 1.0;
                    for (int i = 1; i < _n_order; ++i) _T(i) = _T(i - 1) * t;

                    // position
                    _vec = _T;
                    _cmd.position.x = _vec.dot(_coef[_DIM_x].col(idx));
                    _cmd.position.y = _vec.dot(_coef[_DIM_y].col(idx));
                    _cmd.position.z = _vec.dot(_coef[_DIM_z].col(idx));
                    
                    // velocity
                    for (int i = 0; i < _n_order; ++i)
                    {
                        if (i > 0) _vec(i) = _T(i - 1) * i;
                        else _vec(i) = 0;
                    }

                    _cmd.velocity.x = _vec.dot(_coef[_DIM_x].col(idx));
                    _cmd.velocity.y = _vec.dot(_coef[_DIM_y].col(idx));
                    _cmd.velocity.z = _vec.dot(_coef[_DIM_z].col(idx));

                    // acceleration
                    for (int i = 0; i < _n_order; ++i)
                    {
                        if (i > 1)
                        {
                            _vec(i) = _T(i - 2) * i * (i - 1);
                        }
                        else
                        {
                            _vec(i) = 0;
                        }
                    }

                    _cmd.acceleration.x = _vec.dot(_coef[_DIM_x].col(idx));
                    _cmd.acceleration.y = _vec.dot(_coef[_DIM_y].col(idx));
                    _cmd.acceleration.z = _vec.dot(_coef[_DIM_z].col(idx));
                    break;
                } 
            }
        }
        // #4. just publish
        _cmd_pub.publish(_cmd);
    }

};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "grid_trajectory_server_node");
    ros::NodeHandle handle("~");

    TrajectoryServer server(handle);

    ros::spin();

    return 0;
}
