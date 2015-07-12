
#ifndef _VOXEL_TRAJECTORY_VOXEL_SERVER_H_
#define _VOXEL_TRAJECTORY_VOXEL_SERVER_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <assert.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "voxel_trajectory/voxelmacro.h"
#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/trajectorygenerator.h"

const double _DB_INF = 1e100;
const double _eps = 1e-9;
const double _eps_vel = 1e-3;

namespace VoxelTrajectory
{
    using namespace std;
    using namespace Eigen;
    class VoxelServer
    {
private:
        bool hasTraj;
        int  nTraj, nPoly;

        MatrixXd P, path, inflated_path;
        VectorXd T;

        double max_acc;
        double max_vel;
        double resolution;
        double margin;
        double init_time, final_time;
        double bdy[_TOT_BDY];
        double coeff_t;

        VoxelTrajectory::OctoMap * voxel_map;
        ros::Publisher * pt_grid_pub = NULL;
public:

        // # -1. func for visualization
        // set up grid visualization publisher.
        void setGridPublisher(ros::Publisher & pub)
        {
            pt_grid_pub = & pub;
        }

        // publish grid for visualization, brutal way
        void publishGrid(double grid[_TOT_BDY], int id = 0)
        {
            if (pt_grid_pub == NULL) return ;

            visualization_msgs::Marker marker;

            marker.header.frame_id  = "/map";
            marker.header.stamp     = ros::Time::now();
            
            marker.ns   = "voxel_inflated_path";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.color.a = 0.2;
            marker.color.b = 4.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;

            marker.id = id;

            marker.pose.position.x = (grid[_BDY_x] + grid[_BDY_X]) * 0.5;
            marker.pose.position.y = (grid[_BDY_y] + grid[_BDY_Y]) * 0.5;
            marker.pose.position.z = (grid[_BDY_z] + grid[_BDY_Z]) * 0.5;

            marker.scale.x = grid[_BDY_X] - grid[_BDY_x];
            marker.scale.y = grid[_BDY_Y] - grid[_BDY_y];
            marker.scale.z = grid[_BDY_Z] - grid[_BDY_z];


            pt_grid_pub->publish(marker);
        }

        /* 0. Init our server. */
        VoxelServer()
        {
            hasTraj = false;

            nTraj   = 0;
            nPoly   = 6;

            max_acc = 1.0;
            max_vel = 1.0;
            resolution  = 0.2;
            margin  = 0.02;
            voxel_map = NULL; 
            bdy[0] = bdy[2] = bdy[4] = +_DB_INF;
            bdy[1] = bdy[3] = bdy[5] = -_DB_INF;
        }

        ~VoxelServer()
        {
            delete voxel_map;
        }

        /*  1.  Receive point cloud 
        */

        void addMapBlock(const vector<double> &blk)
        {
            assert(blk.size() % _TOT_BDY == 0);
            if (voxel_map == NULL) voxel_map = new VoxelTrajectory::OctoMap(bdy, resolution); 

            vector<double> to_add(blk);
            for (size_t idx = 0; idx < to_add.size(); idx += 2) to_add[idx] += _eps;

            voxel_map->insertBlocks(to_add);
        }

        /* set up the map by 3-D points.
         */
        void setPointCloud(const vector<double> &pt)
        {
            assert(pt.size() % _TOT_DIM == 0);
            if (voxel_map == NULL) voxel_map = new VoxelTrajectory::OctoMap(bdy, resolution);

            vector<double> to_add(pt);
            for (size_t idx = 0; idx < to_add.size(); idx += 2) to_add[idx] += _eps;

            voxel_map->insertPoints(to_add);
        }

        //> Get simplified point cloud
        vector<double> getPointCloud()
        {
            return voxel_map->getPointCloud();
        }

        //> Add more points.
        void addPointCloud(const vector<double> &pt)
        {
            this->setPointCloud(pt);
        }

        /*  2.  Set up init/final states, 
        *       generate trajectory.
        */
        const static int _TRAJ_NULL = 0;
        const static int _TRAJ_HALF = 1;
        const static int _TRAJ_SUCC = 2;
        const double _CHECK_DELTA_T = 0.1;

        bool checkHalfWayObstacle_BrutalForce()
        {
            vector<double> state;
            for (double t = init_time; t < final_time; t += _CHECK_DELTA_T)
            {
                getDesiredState(t);
                if (voxel_map->testObstacle(state.data())) 
                {
                    return true;
                }
            }
            return false;
        }

        MatrixXd getInflatedPath(const MatrixXd & path)
        {
                int m = path.rows() >> 1;
                MatrixXd inflated_path = path.block(1, 0, m, path.cols());
#if 1
                auto within = [&](double pt[_TOT_DIM], double bdy[_TOT_BDY])
                {
                    //for (int i = 0; i < _TOT_DIM; i++) clog << pt[i] << " "; clog << endl;
                    //for (int i = 0; i < _TOT_BDY; i++) clog << bdy[i] << " "; clog << endl;
                    for (int dim = 0; dim < _TOT_BDY; ++dim)
                    {
                        if (abs(pt[dim >> 1] - bdy[dim]) < _eps) return dim;
                    }
                    assert(false);
                    //clog << "!" << endl;
                };

                for (int iRow = 0; iRow < m; ++iRow)
                {
                    int direction[_TOT_BDY] = {-1, 1, -1, 1, -1, 1};
                    int neighbor[_TOT_BDY] = {0, 0, 0, 0, 0, 0};

                    // original grid
                    double bdy[_TOT_BDY] = 
                    {
                        inflated_path(iRow, _BDY_x), inflated_path(iRow, _BDY_X),
                        inflated_path(iRow, _BDY_y), inflated_path(iRow, _BDY_Y),
                        inflated_path(iRow, _BDY_z), inflated_path(iRow, _BDY_Z)
                    };


                    //clog << "iRow = " << iRow << endl; 
                    if (iRow == 0)
                    {
                        double pt[_TOT_DIM + _TOT_DIM] =  
                        {
                            (path(m + iRow + 1, _BDY_x) + path(m + iRow + 1, _BDY_X)) * 0.5,
                            (path(m + iRow + 1, _BDY_y) + path(m + iRow + 1, _BDY_Y)) * 0.5,
                            (path(m + iRow + 1, _BDY_z) + path(m + iRow + 1, _BDY_Z)) * 0.5
                        };
                        //clog << " ? " << endl;
                        neighbor[within(pt, bdy)] = 1;
                    }
                    else if (iRow + 1== m)
                    {
                        double pt[_TOT_DIM + _TOT_DIM] = 
                        {
                            (path(m + iRow, _BDY_x) + path(m + iRow, _BDY_X)) * 0.5,
                            (path(m + iRow, _BDY_y) + path(m + iRow, _BDY_Y)) * 0.5,
                            (path(m + iRow, _BDY_z) + path(m + iRow, _BDY_Z)) * 0.5
                        };
                        neighbor[within(pt, bdy)] = 1;
                    }
                    else
                    {
                        double pt[_TOT_DIM + _TOT_DIM] = 
                        {
                            (path(m + iRow, _BDY_x) + path(m + iRow, _BDY_X)) * 0.5,
                            (path(m + iRow, _BDY_y) + path(m + iRow, _BDY_Y)) * 0.5,
                            (path(m + iRow, _BDY_z) + path(m + iRow, _BDY_Z)) * 0.5,
                            (path(m + iRow + 1, _BDY_x) + path(m + iRow + 1, _BDY_X)) * 0.5,
                            (path(m + iRow + 1, _BDY_y) + path(m + iRow + 1, _BDY_Y)) * 0.5,
                            (path(m + iRow + 1, _BDY_z) + path(m + iRow + 1, _BDY_Z)) * 0.5
                        };
                        neighbor[within(pt, bdy)] = 1;
                        neighbor[within(pt + _TOT_DIM, bdy)] = 1;
                    }

                   // #1. inflate towards all direction inflation;
                    voxel_map->inflateBdy(bdy, direction);

                    // #2. inflate towards labours;
                    
                    for (int dim = 0; dim < _TOT_BDY; ++dim)
                    {
                        neighbor[dim] *= direction[dim];
                        //clog << neighbor[dim] << " ";
                    }
                    //clog << endl;

                    voxel_map->inflateBdy(bdy, neighbor);

                    for (int dim = 0; dim < _TOT_BDY; ++dim)
                        if (neighbor[dim] != 0)
                        {
                            neighbor[dim] = 0;
                            voxel_map->inflateBdy(bdy, neighbor, 8);
                            neighbor[dim] = (dim & 1) ? 1 : -1;
                        }
                    

                    // #3. inflate towards one direction each time;
                    for (int drc = 0; drc < _TOT_BDY; ++drc)
                    {
                        if (neighbor[drc] != 0) continue;
                       // clog << "[!!] drc = " << drc << endl;
                        memset(direction, 0, sizeof(direction));
                        direction[drc] = (drc & 1) ? 1 : -1;
                        voxel_map->inflateBdy(bdy, direction, 8);
                    }

                    //clog << "----------------------------------------" << endl;
                    // store in the matrix
                    inflated_path.row(iRow) << 
                        bdy[_BDY_x], bdy[_BDY_X],
                        bdy[_BDY_y], bdy[_BDY_Y],
                        bdy[_BDY_z], bdy[_BDY_Z];

                    // visualize it.
                    //publishGrid(bdy, iRow);
                }
                clog << "[ inflated Path ]: \n" << inflated_path << endl;
#endif
                return inflated_path;
        }

        int setWayPoints(
                const vector<double> & p_s,
                const vector<double> wp,
                double init_T)
        {
            assert(p_s.size() == _TOT_DIM * 3);
            assert(wp.size() % _TOT_DIM == 0);
            
            int ret = _TRAJ_SUCC, n = wp.size() / _TOT_DIM, m = 0;
            
            vector<MatrixXd> vec_edge; 
            vector<MatrixXd> vec_node;

            double p[2][_TOT_DIM] = {p_s[0], p_s[1], p_s[2], wp[0], wp[1], wp[2]};

            for (int idx = 0; idx < n; ++idx)
            {
                p[~idx & 1][_DIM_x] = wp[idx * _TOT_DIM + _DIM_x];
                p[~idx & 1][_DIM_y] = wp[idx * _TOT_DIM + _DIM_y];
                p[~idx & 1][_DIM_z] = wp[idx * _TOT_DIM + _DIM_z];
                auto edge_node = voxel_map->getPath(p[idx & 1], p[~idx & 1]);
                if (edge_node.first.rows() == 0) 
                {
                    ROS_WARN("[TRAJ] Can't find a path.");
                    return -1;
                }
                vec_edge.push_back(edge_node.first);
                vec_node.push_back(edge_node.second);
                m += edge_node.second.cols();
            }
            //clog << " the array of node and edge : " << vec_node.size()<< ", " << vec_edge.size() << endl;

            MatrixXd edge(m + 1, _TOT_BDY), node(m, _TOT_BDY);

            m = 0;
            for (int idx = 0; idx < n; ++idx)
            {
                int c = vec_node[idx].cols();
                edge.block(m, 0, c + 1, _TOT_BDY) << vec_edge[idx].transpose();
                node.block(m, 0, c, _TOT_BDY) << vec_node[idx].transpose();
                m += c;
            }
            path.resize(m << 1, _TOT_BDY);
            // set up the start and the final position
            path.row(0) << p_s[_DIM_x], p_s[_DIM_y], p_s[_DIM_z], 
                wp[(n - 1) * _TOT_DIM + _DIM_x], 
                wp[(n - 1) * _TOT_DIM + _DIM_y], 
                wp[(n - 1) * _TOT_DIM + _DIM_z]; 
            // set up the box
            path.block(1, 0, m, _TOT_BDY) << node;
            // set up the window
            path.block(m + 1, 0, m - 1, _TOT_BDY) << edge.block(1, 0, m - 1, _TOT_BDY);
            clog << "[ PATH ]: \n" << path << endl;

            inflated_path = getInflatedPath(path);

            MatrixXd Vel(_TOT_DIM, 2);
            MatrixXd Acc(_TOT_DIM, 2);

            Vel.col(0) << p_s[3 + 0], p_s[3 + 1], p_s[3 + 2];
            Vel.col(1) << 0.0, 0.0, 0.0;

            Acc.col(0) << p_s[6 + 0], p_s[6 + 1], p_s[6 + 2];
            Acc.col(1) << 0.0, 0.0, 0.0;

            // generate the trajectory
            VoxelTrajectory::TrajectoryGenerator traj_gen;
            pair<MatrixXd, VectorXd> traj = 
               traj_gen.genPolyCoeffTime(path, inflated_path, 
                       Vel, Acc, max_vel, max_acc, coeff_t);


            P   = traj.first;
            T   = traj.second;

            if (T(0) < 0) 
            {
                clog << "[WARN] No Suitable Trajectory !";
                return _TRAJ_NULL;
            }

            hasTraj = true;
            
            nTraj   = T.rows();

            init_time = init_T;

             // inflation rate
            coeff_t = 1.0 / coeff_t;
            vector<double> state = getDesiredState(init_time);
            if ((abs(state[3])+ abs(state[4]) + abs(state[5])) > _eps_vel) coeff_t = 1.0; 

            clog << "coeff t = " << coeff_t << endl;

           final_time = init_T + T.sum();

            return ret;
        }

        int setPoints(
            const vector<double> & p_s,
            const vector<double> & p_t,
            double init_T)
        {
            assert(p_s.size() == _TOT_DIM * 3);
            assert(p_t.size() == _TOT_DIM * 3);
            int ret = _TRAJ_SUCC;

            double p[2][3] = {p_s[0], p_s[1], p_s[2], p_t[0], p_t[1], p_t[2]};

            
            VoxelTrajectory::TrajectoryGenerator traj_gen;
            MatrixXd Vel (_TOT_DIM, 2);
            MatrixXd Acc (_TOT_DIM, 2); 

            do{
                // Here's some ugly construction due to the elder version.
                auto edge_node = voxel_map->getPath(p[0], p[1]);
                auto & edge = edge_node.first;
                auto & node = edge_node.second;
                if (edge.rows() == 0)
                {
                    ROS_WARN("[TRAJ] Can't find a path.");
                    return -1;
                }
                
                int m = node.cols();

                if (m < 1)
                {
                    clog << "[WRONG] ILLEGAL POINTS!" << endl;
                    return _TRAJ_NULL;
                }


                path.resize(m << 1, _TOT_BDY);
                // set up the start and the final position
                path.row(0) << p_s[0], p_s[1], p_s[2], p_t[0], p_t[1], p_t[2];
                // set up the box
                path.block(1, 0, m, _TOT_BDY) << node.transpose();
                // set up the window
                path.block(m + 1, 0, m - 1, _TOT_BDY) << edge.transpose().block(1, 0, m - 1, _TOT_BDY);

                clog << "[ PATH ]: \n" << path << endl;

                // the matrix for inflation
                inflated_path = getInflatedPath(path);
                

                Vel.col(0) << p_s[3 + 0], p_s[3 + 1], p_s[3 + 2];
                Vel.col(1) << p_t[3 + 0], p_t[3 + 1], p_t[3 + 2];

                Acc.col(0) << p_s[6 + 0], p_s[6 + 1], p_s[6 + 2];
                Acc.col(1) << p_t[6 + 0], p_t[6 + 1], p_t[6 + 2];

                // generate the trajectory
                pair<MatrixXd, VectorXd> traj = 
                   traj_gen.genPolyCoeffTime(path, inflated_path, 
                           Vel, Acc, max_vel, max_acc, coeff_t);

                P   = traj.first;
                T   = traj.second;

                if (T(0) < 0) 
                {
                    clog << "[WARN] No Suitable Trajectory !";
                    return _TRAJ_NULL;
                }
            } while (T(0) < 0);

            hasTraj = true;
            
            nTraj   = T.rows();

            init_time = init_T;
            final_time = init_T + T.sum();

            // inflation rate
            coeff_t = 1.0 / coeff_t;
            vector<double> state = getDesiredState(init_time);
            if ((abs(state[3])+ abs(state[4]) + abs(state[5])) > _eps_vel) coeff_t = 1.0; 

            return ret;
        }

        /* 3. Receive time stamp(in odometry)m, 
        *   return desired states.
        *   > if not small than time_s, return the init states.
        *   > if more than time_t, return the final states.
        *   > otherwise, calculate on the traj.
        */

        vector<double> getDesiredState(double t_now)
        {
            //clog<<"[t_now] = "<< t_now << ", [init_time] = " << init_time << ", "<< t_now - init_time<<endl;
            //clog<<"[ T ]" << T.transpose() << endl;
            t_now -= init_time;
            vector<double > ret(3 * 3, 0);
            int iSeg = 0;

            while (iSeg + 1 < nTraj && t_now > T[iSeg])
            {
                t_now -= T[iSeg];
                iSeg += 1;
            }

            t_now = max(0.0, t_now);
            t_now = min(t_now, T[iSeg]);

                    for (int dim = 0; dim < _TOT_DIM; ++dim)
                    {
                        VectorXd coeff = P.col(dim).segment(iSeg * nPoly, nPoly);
                        //clog<<"[seg] = " << iSeg << ", [dim] = " << dim << ", " << coeff.transpose() <<endl; 
                        
                        VectorXd t;

                        double scale_ratio = 1.0;
                        for (int i = 0; i < _TOT_DIM; ++i)
                        {
                            t = VectorXd::Zero(nPoly);
                            t(i) = 1.0;
                            
                            for (int j = i + 1; j < nPoly; ++j) t(j) = t(j-1) * t_now;
                                
                            ret[dim + i * 3] = scale_ratio * coeff.dot(t);
                            scale_ratio *= coeff_t;

                            for (int j = 0; j < nPoly; ++j) coeff(j) *= (j-i);
                        }
                    }
            return ret;
        }

        /* 4. some helpful function
        */
        //> set max accleration
        void setMaxAcceleration(double acc) 
        {this->max_acc  = acc;}

        //> set max veloctiy
        void setMaxVelocity(double vel) 
        {this->max_vel   = vel;}

        //> set margin on voxel
        void setMargin(double margin) 
        {this->margin = margin;}

        //> set resolution on the map
        void setResolution(double resolution)
        {this->resolution   = resolution;}
        
        //> set mapping boundary (usually useless) 
        void setMapBoundary(double bdy[_TOT_BDY])
        {memcpy(this->bdy, bdy, sizeof(double) * _TOT_BDY);}

        const double * getBdy()
        {return bdy;}

        double getBeginTime()
        {return init_time;}

        double getFinalTime()
        {return final_time;} 

        MatrixXd getPolyCoeff()
        {return P;}

        VectorXd getTimeAllocation()
        {return T;}

        MatrixXd getVoxelPath()
        {return path;}

        const MatrixXd & getPathConstRef()
        {return path;}
        
        const MatrixXd & getInflatedPathConstRef()
        {return inflated_path;}

        vector<double> getCheckPoint()
        {
            vector<double> ret, state;
            double timeNow = init_time;
            for (int i = 0; i < nTraj; i++)
            {
                state = getDesiredState(timeNow);
                ret.push_back(state[_DIM_x]);
                ret.push_back(state[_DIM_y]);
                ret.push_back(state[_DIM_z]);
                timeNow += T(i);
            }

            return ret;
        }

        bool isTraj()
        {return this->hasTraj;}
    };
}

#endif
