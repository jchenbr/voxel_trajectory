#ifndef _VOXEL_TRAJECTORY_VOXEL_SERVER_H_
#define _VOXEL_TRAJECTORY_VOXEL_SERVER_H_

#include "voxel_trajectory/voxelmacro.h"
#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/trajectorygenerator.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <assert.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <voxel_map/voxel_map.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>

#define _use_voxel_map_ 1

const double _DB_INF = 1e100;
const double _eps = 1e-9;
const double _eps_vel = 1e-3;
const double _eps_pos = 1e-5;

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

        double max_acc, max_vel;
        double flight_vel, flight_acc;
        double resolution;
        double margin;
        double init_time, final_time;
        double bdy[_TOT_BDY];
        double coeff_t;
        double ob_radius;
        double heu_ratio = 5.0;
        double log_odd[5] = {-2.0, 4.5, 0.0, 0.85, -0.4};

#if _use_voxel_map_
        double dislike_margin;
        double path_resolution;
        voxel_map::VoxelMap * _map;

        voxel_map::VoxelFilter _path_voxel_filter;
#else
        VoxelTrajectory::OctoMap * voxel_map;
#endif
public:
        vector<double> qp_cost;

        /* 0. Init our server. */
        VoxelServer()
        {
            hasTraj = false;

            nTraj   = 0;
            nPoly   = 6;

            max_acc = 1.0;
            max_vel = 1.0;
            flight_vel = 1.0;
            flight_acc = 1.0;
            resolution  = 0.2;
            margin  = 0.02;
#if _use_voxel_map_
            _map = NULL;
#else
            voxel_map = NULL; 
#endif

            init_time = final_time = 0.0;

            bdy[0] = bdy[2] = bdy[4] = +_DB_INF;
            bdy[1] = bdy[3] = bdy[5] = -_DB_INF;

            _path_voxel_filter = [](const voxel_map::Box & box)
            {
                auto ctr = box.getCenter();
                return ctr(2) >= 0.0;
            };
        }

        ~VoxelServer()
        {
#if _use_voxel_map_
            delete _map;
#else
            delete voxel_map;
#endif
        }

        void setPathResolution(double res)
        {
            path_resolution = res;
        }

        void setDislikeMargin(double margin)
        {
            dislike_margin = margin;
        }
        
        void initMap()
        {
#if _use_voxel_map_
            //ROS_WARN("[VoxelServer] starting initialization.");
            if (_map != NULL) delete _map;

            voxel_map::VoxelMapConfiguration map_config(resolution, log_odd[0], 
                    log_odd[0], log_odd[1], log_odd[2], log_odd[3], log_odd[4],
                    voxel_map::Box{bdy}, ob_radius);

            voxel_map::VoxelPathConfiguration path_config(path_resolution,
                    margin, dislike_margin, 10, [](const voxel_map::Box & box){return box.lower(2) >= 0.0;}, heu_ratio);
            
            _map = new voxel_map::VoxelMap(map_config, path_config);
            //ROS_WARN("[VoxelServer] finished initialization.");
#else
            delete voxel_map;
            voxel_map = new VoxelTrajectory::OctoMap(bdy, resolution);
#endif
        }

        /*  1.  Receive point cloud 
        */

        void addMapBlock(const vector<double> &blk)
        {
            assert(blk.size() % _TOT_BDY == 0);
#if _use_voxel_map_
            //ROS_WARN("[VoxelServer] A");
            if (_map == NULL) initMap();
            vector<voxel_map::Box> boxes;
            boxes.reserve(blk.size() / _TOT_BDY);

            for (size_t idx = 0; idx < blk.size(); idx += _TOT_BDY)
                boxes.emplace_back(blk.data() + idx);

            _map->obBoxes(boxes);
            //ROS_WARN("[VoxelServer] a");
#else
            if (voxel_map == NULL) voxel_map = new VoxelTrajectory::OctoMap(bdy, resolution); 

            vector<double> to_add(blk);
            for (size_t idx = 0; idx < to_add.size(); idx += 2) to_add[idx] += _eps;

            voxel_map->insertBlocks(to_add);
#endif
        }

#if _use_voxel_map_
        void addRays(const vector<voxel_map::Ray> rays)
        {
            if (_map == NULL) initMap();
            _map->obRays(rays);
        }
        
        void addBoxes(const vector<voxel_map::Box> boxes)
        {
            if (_map == NULL) initMap();
            _map->obBoxes(boxes);
        }
        
        void addPoints(const vector<voxel_map::Point> pts)
        {
            if (_map == NULL) initMap();
            _map->obPoints(pts);
        }
#endif

        /* set up the map by 3-D points.
         */
        void setPointCloud(const vector<double> &pt)
        {
            assert(pt.size() % _TOT_DIM == 0);
#if _use_voxel_map_
            //ROS_WARN("[VoxelServer] C");
            if (_map == NULL) initMap();
            vector<voxel_map::Point> pts;
            pts.reserve(pts.size() / _TOT_DIM);
            for (size_t idx = 0; idx < pt.size(); idx += _TOT_DIM)
            {
                voxel_map::Point p(pt[idx + _DIM_x], pt[idx + _DIM_y], pt[idx + _DIM_z]);
                pts.push_back(p);
            }
            _map->obPoints(pts);
            //ROS_WARN("[VoxelServer] c");
#else
            if (voxel_map == NULL) voxel_map = new VoxelTrajectory::OctoMap(bdy, resolution);

            vector<double> to_add(pt);
            for (size_t idx = 0; idx < to_add.size(); idx += 2) to_add[idx] += _eps;

            voxel_map->insertPoints(to_add);
#endif
        }

        //> Get simplified point cloud
        vector<double> getPointCloud()
        {
#if _use_voxel_map_
            // ROS_WARN("[VoxelServer] loading the map points");
            if (_map == NULL) initMap();
            auto boxes = _map->getOccupiedGrids();
            vector<double> pts;
            pts.reserve(boxes.size() * _TOT_DIM);
            for (auto &box : boxes) 
            {
                auto pt = box.getCenter();
                pts.push_back(pt(_DIM_x));
                pts.push_back(pt(_DIM_y));
                pts.push_back(pt(_DIM_z));
            }
            // ROS_WARN_STREAM("[VoxelServer] Total points in the map = " << boxes.size());
            return pts;
#else
            if (voxel_map == NULL) return vector<double>(0);
            return voxel_map->getPointCloud();
#endif
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
        const double _CHECK_DELTA_T = 0.03;

        bool checkHalfWayObstacle_BrutalForce(double beg_time = 0.0)
        {
            //ROS_INFO("[VOXEL_SERVER] In the core alreday.");
            vector<double> state;
            for (double t = init_time; t < final_time; t += _CHECK_DELTA_T)
            {
                state = getDesiredState(t);
                //ROS_INFO("[VXOEL_SERVER] !");
#if _use_voxel_map_
                
            //ROS_WARN("[VoxelServer] E");
                if (_map->isPointUnsafe(voxel_map::Point(state[_DIM_x], state[_DIM_y], state[_DIM_z])))
                {
                    return true;
                }
            //ROS_WARN("[VoxelServer] e");
#else
                if (voxel_map->testObstacle(state.data())) 
                {
                    return true;
                }
#endif
            }
            return false;
        }

	    bool isValid3DPoint(double x, double y, double z)
        {
#if _use_voxel_map_
            //ROS_WARN("[VoxelServer] F");
            if (_map == NULL) initMap();
            return !_map->isPointUnsafe(voxel_map::Point(x, y, z));
            //ROS_WARN("[VoxelServer] f");
#else
            if (voxel_map == NULL) return true;
            double pt[_TOT_DIM]  {x, y, z};
            return !voxel_map->testObstacle(pt);
#endif
        }

#if _use_voxel_map_
        bool isValidBox(const voxel_map::Box & box)
        {
            if (_map == NULL) initMap();
            return !_map->isBoxUnsafe(box);
        }
#endif

        int setWayPointsRec(
                const vector<double> & p_s,
                const vector<double> wp,
                double init_T,
                vector<double> & arr_time,
                vector<double> & cost_time)
        {
            assert(p_s.size() == _TOT_DIM * 3);
            assert(wp.size() % _TOT_DIM == 0);
            addMapBlock(vector<double>(0));
            
            int ret = _TRAJ_SUCC, n = wp.size() / _TOT_DIM, m = 0;
            
            ros::Time prv_time = ros::Time::now();

            ROS_WARN("[VoxelServer] H");
            MatrixXd edge, node;
            vector<int> seg_id;
            {
                prv_time = ros::Time::now();
                vector<voxel_map::Box> voxels, windows;
                { // search the voxel path
                    _map->setPathVoxelFilter(_path_voxel_filter);

                    for (size_t idx = 0; idx < wp.size(); idx += _TOT_DIM)
                    {
                        // set up
                        auto src_pt = (idx) ? wp.begin() + (idx - _TOT_DIM) : p_s.begin();
                        auto dest_pt = wp.begin() + idx;
                        voxel_map::Point src(src_pt[_DIM_x], src_pt[_DIM_y], src_pt[_DIM_z]);
                        voxel_map::Point dest(dest_pt[_DIM_x], dest_pt[_DIM_y], dest_pt[_DIM_z]);
                        //ROS_WARN_STREAM("[VOXEL_SERVER] segment_id = " << idx / _TOT_DIM);
                        //ROS_WARN_STREAM("[VOXEL_SERVER] src_pt = " << src.transpose());
                        //ROS_WARN_STREAM("[VOXEL_SERVER] dest_pt = " << dest.transpose());

                        // serach it
                        voxel_map::Path::VoxelPath path;
                        if (!_map->retPathBySrcDest(src, dest, path))
                        {
                            ROS_INFO_STREAM("[VOXEL_SERVER] No feasible path.");
                            return ret = _TRAJ_NULL;
                        }

                        // collect result
                        voxels.insert(voxels.end(), path.begin(), path.end());

                        seg_id.push_back(voxels.size() - 1);

                        if (windows.empty()) windows.emplace_back(src, src);
                        for (size_t ind = 0; ind + 1 < path.size(); ++ind) 
                            windows.push_back(path[ind].intersect(path[ind + 1]));
                        windows.emplace_back(dest, dest);
                    }

                    m = voxels.size();
                    node.resize(m, _TOT_BDY);
                    edge.resize(m + 1, _TOT_BDY);

                    for (size_t ind = 0; ind < voxels.size(); ++ind)
                    {
                        node.row(ind) << 
                            voxels[ind].lower(_DIM_x), voxels[ind].upper(_DIM_x),
                            voxels[ind].lower(_DIM_y), voxels[ind].upper(_DIM_y),
                            voxels[ind].lower(_DIM_z), voxels[ind].upper(_DIM_z);
                    }                    

                    for (size_t ind = 0; ind < windows.size(); ++ind)
                    {
                        edge.row(ind) << 
                            windows[ind].lower(_DIM_x), windows[ind].upper(_DIM_x),
                            windows[ind].lower(_DIM_y), windows[ind].upper(_DIM_y),
                            windows[ind].lower(_DIM_z), windows[ind].upper(_DIM_z);
                    }
                }
                ROS_WARN_STREAM("[VOXEL_SERVER] time for path search = " 
                        << (ros::Time::now() - prv_time).toSec());
                cost_time.push_back((ros::Time::now() - prv_time).toSec());

                prv_time = ros::Time::now();
                { // inflate the voxels of the path
                    voxel_map::Path::VoxelPath _inflated_path;
                    _map->retInflatedPathAsLike(voxels, _inflated_path, 4.0);
                    inflated_path.resize(m, _TOT_BDY);
                    for (int idx = 0; idx < m; ++idx)
                    {
                        auto & box = _inflated_path[idx];
                        inflated_path.row(idx) <<
                            box.lower(_DIM_x), box.upper(_DIM_x),
                            box.lower(_DIM_y), box.upper(_DIM_y),
                            box.lower(_DIM_z), box.upper(_DIM_z);
                    }
                }
                ROS_WARN_STREAM("[VOXEL_SERVER] time for inflation = " 
                        << (ros::Time::now() - prv_time).toSec());
                cost_time.push_back((ros::Time::now() - prv_time).toSec());
            }

            { // record and formating
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
            }

            prv_time = ros::Time::now();

            // #2. trajectory generation
            { // call the api
                MatrixXd Vel(_TOT_DIM, 2);
                MatrixXd Acc(_TOT_DIM, 2);

                Vel.col(0) << p_s[3 + 0], p_s[3 + 1], p_s[3 + 2];
                Vel.col(1) << 0.0, 0.0, 0.0;

                Acc.col(0) << p_s[6 + 0], p_s[6 + 1], p_s[6 + 2];
                Acc.col(1) << 0.0, 0.0, 0.0;

                VoxelTrajectory::TrajectoryGenerator traj_gen;
                pair<MatrixXd, VectorXd> traj = 
                   traj_gen.genPolyCoeffTime(path, inflated_path, 
                           Vel, Acc, max_vel, max_acc,
                           flight_vel, flight_acc, coeff_t);
                qp_cost = traj_gen.qp_cost;
                P   = traj.first;
                T   = traj.second;
            }
            ROS_WARN_STREAM("[VOXEL_SERVER] time for traj generation = " 
                    << (ros::Time::now() - prv_time).toSec());
            cost_time.push_back((ros::Time::now() - prv_time).toSec());



            if (T(0) < 0) 
            {
                ROS_WARN_STREAM("[WARN] No Suitable Trajectory !");
                return _TRAJ_NULL;
            }

            hasTraj = true;
            
            // record the important time
            {
                nTraj   = T.rows();

                init_time = init_T;
                final_time = init_T + T.sum();

                arr_time.clear();
                double tmp_time = init_time;
                int sid = 0;
                for (int iRow = m + 1; iRow < m + m; ++iRow)
                {
                    tmp_time += T(iRow - m - 1);
                    if (iRow - m - 1 == seg_id[sid])
                    {
                        arr_time.push_back(tmp_time);
                        sid += 1;
                    }
                }
                arr_time.push_back(final_time);
            }

            // #3. scaling rate
#if 0
            coeff_t = 1.0 / coeff_t;
            vector<double> state = getDesiredState(init_time);
            if ((abs(state[3])+ abs(state[4]) + abs(state[5])) > _eps_vel) coeff_t = 1.0; 
            coeff_t = 1.0;
#endif

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
            if (T.cols() == 0 || !hasTraj) return vector<double>(0); 

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

        quadrotor_msgs::PolynomialTrajectory getTraj()
        {
            quadrotor_msgs::PolynomialTrajectory traj;
            traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
            traj.num_order = nPoly;
            traj.num_segment = nTraj;


            traj.coef_x.resize(P.rows());
            traj.coef_y.resize(P.rows());
            traj.coef_z.resize(P.rows());

            for (int idx = 0; idx < P.rows(); ++idx)
            {
                traj.coef_x[idx] = P(idx, _DIM_x);
                traj.coef_y[idx] = P(idx, _DIM_y);
                traj.coef_z[idx] = P(idx, _DIM_z);
            }

            traj.header.frame_id = "/map";
            traj.header.stamp = ros::Time(init_time); 
            
            traj.time.resize(T.rows());
            for (int idx = 0; idx < T.rows(); ++idx)
            {
                traj.time[idx] = T[idx];
            }
            return traj;
        }

        /* 4. some helpful function
        */
        //> set max accleration
        void setMaxAcceleration(double acc) 
        {this->max_acc  = acc;}

        //> set max veloctiy
        void setMaxVelocity(double vel) 
        {this->max_vel   = vel;}

        void setFlightVelocity(double vel)
        {this->flight_vel = vel;}

        void setFlightAcceleration(double acc)
        {this->flight_acc = acc;}

        //> set margin on voxel
        void setMargin(double margin) 
        {this->margin = margin;}
        
        void setHeuristicRatio(double ratio)
        {assert(ratio >= 1.0); this->heu_ratio = ratio;}

        //> set resolution on the map
        void setResolution(double resolution)
        {this->resolution   = resolution;}
        
        //> set mapping boundary (usually useless) 
        void setMapBoundary(const double bdy[_TOT_BDY])
        {memcpy(this->bdy, bdy, sizeof(double) * _TOT_BDY);}

        void setLogOdd(const double lo[5])
        {memcpy(this->log_odd, lo, sizeof(double) * 5);}

        void setObservationRadius(double radius)
        {ob_radius = radius;}

        void setPathVoxelFilter(const voxel_map::VoxelFilter & filter)
        {_path_voxel_filter = filter;}

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
