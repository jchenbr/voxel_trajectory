
#ifndef _VOXEL_TRAJECTORY_VOXEL_SERVER_H_
#define _VOXEL_TRAJECTORY_VOXEL_SERVER_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <assert.h>
//#include <ros/console.h>

#include "voxel_trajectory/voxelmacro.h"
#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/voxelgraph.h"
#include "voxel_trajectory/trajectorygenerator.h"

const double _DB_INF = 1e100;
const double _eps = 1e-9;

namespace VoxelTrajectory
{
    using namespace std;
    using namespace Eigen;
    class VoxelServer
    {
private:
        bool hasMap;
        bool hasTraj;
        int  nTraj, nPoly;

        MatrixXd P, path;
        VectorXd T;

        double max_acc;
        double max_vel;
        double resolution;
        double margin;
        double init_time;
        double bdy[_TOT_BDY];

        VoxelTrajectory::OctoMap * voxel_map;
        VoxelTrajectory::VoxelGraph * voxel_graph; 
public:
        /* 0. Init our server. */
        VoxelServer()
        {
            hasMap  = false;
            hasTraj = false;

            nTraj   = 0;
            nPoly   = 6;

            max_acc = 1.0;
            max_vel = 1.0;
            resolution  = 0.2;
            margin  = 0.02;
            voxel_map = NULL; 
            voxel_graph = NULL;
            bdy[0] = bdy[2] = bdy[4] = +_DB_INF;
            bdy[1] = bdy[3] = bdy[5] = -_DB_INF;
        }

        ~VoxelServer()
        {
            if (hasMap) 
            {
                delete voxel_map;
                delete voxel_graph;
            }
        }

        /*  1.  Receive point cloud 
        */

        void addMapBlock(const vector<double> &blk)
        {
            assert(blk.size() % _TOT_BDY == 0);

            size_t n_blk = blk.size() / _TOT_BDY;

            if (!hasMap)
            {
                voxel_map   = new VoxelTrajectory::OctoMap(bdy, resolution);
            }

            for (int i = 0; i < n_blk; i++)
            {
                double to_add[_TOT_BDY] = 
                {   blk[i * _TOT_BDY + _BDY_x], blk[i * _TOT_BDY + _BDY_X] + _eps,
                    blk[i * _TOT_BDY + _BDY_y], blk[i * _TOT_BDY + _BDY_Y] + _eps,
                    blk[i * _TOT_BDY + _BDY_z], blk[i * _TOT_BDY + _BDY_Z] + _eps};
                voxel_map->insertBlock(to_add);
            }

            if (!hasMap)
            {
                hasMap  = true;
                voxel_graph = new VoxelTrajectory::VoxelGraph(voxel_map);
            }
        }

        void setPointCloud(const vector<double> &pt)
        {
            assert(pt.size() % _TOT_DIM == 0);

            size_t n_pt = pt.size() / _TOT_DIM;

            if (!hasMap)
            {   
#if  1
                voxel_map = new VoxelTrajectory::OctoMap(bdy, resolution);
#else
                double l = min(bdy[_BDY_x], min(bdy[_BDY_y], bdy[_BDY_z]));
                double r = max(bdy[_BDY_X], max(bdy[_BDY_Y], bdy[_BDY_Z]]);
                for (int i = 0; i < pt.size(); i++)
                {
                    l   = min(l, pt[i] - margin);
                    r   = max(r, pt[i] + margin); 
                }

                double tmp[_TOT_BDY] = {l, r, l, r, l, r};

                voxel_map   = new VoxelTrajectory::OctoMap(tmp, resolution);

                double x_lower_wall[_TOT_BDY], x_upper_wall[_TOT_BDY]; 
                double y_lower_wall[_TOT_BDY], y_upper_wall[_TOT_BDY]; 
                double z_lower_wall[_TOT_BDY], z_upper_wall[_TOT_BDY]; 

                memcpy(x_lower_wall, bdy, _TOT_BDY * size(double));
                memcpy(x_upper_wall, bdy, _TOT_BDY * size(double));
                memcpy(y_lower_wall, bdy, _TOT_BDY * size(double));
                memcpy(y_upper_wall, bdy, _TOT_BDY * size(double));
                memcpy(z_lower_wall, bdy, _TOT_BDY * size(double));
                memcpy(z_upper_wall, bdy, _TOT_BDY * size(double));

                x_lower_wall[_BDY_X] = bdy[_BDY_x];
                x_upper_wall[_BDY_x] = bdy[_BDY_X];
                y_lower_wall[_BDY_Y] = bdy[_BDY_y];
                y_upper_wall[_BDY_y] = bdy[_BDY_Y];
                z_lower_wall[_BDY_Z] = bdy[_BDY_z];
                z_upper_wall[_BDY_z] = bdy[_BDY_Y];

                voxel_map -> insertBlock(x_lower_wall);
                voxel_map -> insertBlock(x_upper_wall);
                voxel_map -> insertBlock(y_lower_wall);
                voxel_map -> insertBlock(y_upper_wall);
                voxel_map -> insertBlock(z_lower_wall);
                voxel_map -> insertBlock(z_upper_wall);
#endif
            }


            for (int i = 0; i < n_pt; i++)
            {
                double p [_TOT_DIM] =
                {   pt[i * _TOT_DIM + _DIM_x],
                    pt[i * _TOT_DIM + _DIM_y],
                    pt[i * _TOT_DIM + _DIM_z]};

                voxel_map->insert(p);
            }

            if (!hasMap)
            {
                hasMap  = true;
                voxel_graph = new VoxelTrajectory::VoxelGraph(voxel_map);
            }
        }

        //> Get simplified point cloud
        vector<double> getPointCloud()
        {
            if (hasMap)
                return voxel_map->getPointCloud();
            else
                return vector<double>();
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

        int setPoints(
            const vector<double> & p_s,
            const vector<double> & p_t,
            double init_T)
        {
            assert(p_s.size()==_TOT_DIM*3 && p_t.size() == _TOT_DIM*3);
            int ret = _TRAJ_SUCC;

            if (!hasMap) {
                clog << "WRONG! CAN'T SET UP! NO MAP NOW!"<<endl;
                return _TRAJ_NULL;
            }

            double p[2][3]= {
                p_s[0], p_s[1], p_s[2],
                p_t[0], p_t[1], p_t[2]};

            
            VoxelTrajectory::TrajectoryGenerator traj_gen;
            MatrixXd Vel (_TOT_DIM, 2);
            MatrixXd Acc (_TOT_DIM, 2); 

            do{
                VoxelTrajectory::VoxelGraph * graph = voxel_graph;
                graph->SetUp(p[0], p[1], voxel_map);

                path = graph->getPathMatrix();
                clog<<"[ PATH ]: \n"<<path<<endl;

                if (path.rows()<1)
                {
                    clog << "[WRONG] ILLEGAL POINTS!"<<endl;
                    return _TRAJ_NULL;
                }

                for (int i=0; i<_TOT_DIM; i++) Vel(i,0) = p_s[1 * 3 + i];
                for (int i=0; i<_TOT_DIM; i++) Vel(i,1) = p_t[1 * 3 + i];

                for (int i=0; i<_TOT_DIM; i++) Acc(i,0) = p_s[2 * 3 + i];
                for (int i=0; i<_TOT_DIM; i++) Acc(i,1) = p_t[2 * 3 + i];


                pair<MatrixXd, VectorXd> traj   = 
                   traj_gen.genPolyCoeffTime(path, Vel, Acc, max_vel, max_acc);

                P   = traj.first;
                T   = traj.second;

                if (T(0) < 0) 
                {
                    clog << "[WARN] No Suitable Trajectory !";
                    ret = _TRAJ_HALF;
                    
                    if (path.rows() <= 2)
                    {
                        return _TRAJ_NULL;
                    }else
                    {
                        int M = path.rows() >> 1;
                        RowVectorXd midWin=
                            path.row( (M+1)>>1 );
                        p[1][_DIM_x] = ( midWin(_BDY_x) + midWin(_BDY_X) ) * 0.5;
                        p[1][_DIM_y] = ( midWin(_BDY_y) + midWin(_BDY_Y) ) * 0.5;
                        p[1][_DIM_z] = ( midWin(_BDY_z) + midWin(_BDY_Z) ) * 0.5;
                    	clog << "_TRAJ_ROWS = " << path.rows() << "ch = "<< ( (M+1)>>1) <<endl;
                    }
                    clog << "_TRAJ_MID_PT = " << p[1][0] <<" "<< p[1][1] <<" "<< p[1][2] <<" " <<endl;

                }
            } while (T(0) < 0);

            hasTraj = true;
            
            nTraj   = T.rows();

            init_time = init_T;

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
            t_now   -= init_time;
            vector<double > ret(3*3, 0);
            int iSeg=0;

            while (iSeg+1<nTraj && t_now>T[iSeg])
            {
                t_now -= T[iSeg];
                iSeg += 1;
            }

            t_now = max(0.0, t_now);
            t_now = min(t_now, T[iSeg]);

                    for (int dim = 0; dim<_TOT_DIM; dim++)
                    {
                        VectorXd coeff = P.col(dim).segment(iSeg * nPoly, nPoly);
                        //clog<<"[seg] = " << iSeg << ", [dim] = " << dim << ", " << coeff.transpose() <<endl; 
                        
                        VectorXd t;

                        for (int i = 0; i < _TOT_DIM; i++)
                        {
                            t   = VectorXd::Zero(nPoly);
                            t(i)    = 1.0;
                            
                            for (int j = i+1; j<nPoly; j++) 
                                t(j) = t(j-1)*t_now;
                                
                            ret[dim + i*3] = coeff.dot(t);

                            for (int j = 0; j<nPoly; j++)
                                coeff(j) *= (j-i);
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

        double getBeginTime()
        {return init_time;}

        double getFinalTime()
        {return init_time + T.sum();}

        MatrixXd getPolyCoeff()
        {return P;}

        VectorXd getTimeAllocation()
        {return T;}

        MatrixXd getVoxelPath()
        {return path;}
    };
}

#endif
