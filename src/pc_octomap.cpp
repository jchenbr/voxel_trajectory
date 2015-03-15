#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/voxelgraph.h"
#include "voxel_trajectory/trajectorygenerator.h"

const int _BUFF_SIZE    = 30000000;
const static double _EPS   = 1e-11;
static double _MARGIN = 1;
static double _MAP_RES   = 1*1*1;
static float data[_BUFF_SIZE];
static double p_s[_TOT_DIM],p_t[_TOT_DIM];

using namespace std;

int main(int argc, char ** argv)
{

    clog<<"argc"<<argc<<endl;
    if (argc > 2)
    {
    	sscanf(argv[2], "%lf", &_MARGIN);
        _MAP_RES = _MARGIN*_MARGIN*_MARGIN;

        sscanf(argv[3], "%lf,%lf,%lf", p_s+0, p_s+1, p_s+2);
        sscanf(argv[4], "%lf,%lf,%lf", p_t+0, p_t+1, p_t+2);
    }
    clog<<"ps="<<p_s[0]<<","<<p_s[1]<<","<<p_s[2]<<endl;
    clog<<"pt="<<p_t[0]<<","<<p_t[1]<<","<<p_t[2]<<endl;
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
        (new pcl::PointCloud<pcl::PointXYZ>);

    //load point cloud 
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        clog<<"Can't find point cloud file."<<endl;
        return -1;
    }

    double bdy[_TOT_BDY], l, r;

    for (size_t i = 0; i < cloud -> points.size(); ++i)
    {
#if 0
        cout << "(" << cloud->points[i].x << " "
                    << cloud->points[i].y << " "
                    << cloud->points[i].z << ")\n";
#endif
        l = min(l, (double)cloud->points[i].x);
        r = max(r, (double)cloud->points[i].x);
        l = min(l, (double)cloud->points[i].y);
        r = max(r, (double)cloud->points[i].y);
        l = min(l, (double)cloud->points[i].z);
        r = max(r, (double)cloud->points[i].z);
    }

    bdy[_BDY_x] = l-_MARGIN;
    bdy[_BDY_X] = r+_MARGIN;
    bdy[_BDY_y] = l-_MARGIN;
    bdy[_BDY_Y] = r+_MARGIN;
    bdy[_BDY_z] = l-_MARGIN;
    bdy[_BDY_Z] = r+_MARGIN;

    clog<< "(\t"<< bdy[_BDY_x] << "," << bdy[_BDY_X] << "\n"
        << "\t" << bdy[_BDY_y] << "," << bdy[_BDY_Y] << "\n"
        << "\t" << bdy[_BDY_z] << "," << bdy[_BDY_Z] << ")\n";
    clog<<"Get the point cloud."<<endl;

/*===========================================================*/

    VoxelTrajectory::OctoMap octomap(bdy, _MAP_RES);
    double pt[3];
    for (size_t i = 0; i < cloud -> points.size(); ++i)
    {
        pt[0]   = cloud->points[i].x;
        pt[1]   = cloud->points[i].y;
        pt[2]   = cloud->points[i].z;
        octomap.insert(pt);
    }

    Eigen::MatrixXd pc  = octomap.getPointCloud();


    clog<<"Get the octomap."<<endl;

#if 0
    ros::Rate loop_rate(1);
    int count =0;
    while (ros::ok())
    {
        if (pc_pub.getNumSubscribers()>0)
        {
            pc_pub.publish(pc_msg);
            ROS_WARN("Msg has been published!");
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
#endif

    //cout<<"PointCloud:\n"<< pc << endl;

/*==================================================*/

    VoxelTrajectory::VoxelGraph graph(&octomap, p_s, p_t);
    VoxelTrajectory::TrajectoryGenerator traj_gen;

    Eigen::MatrixXd res = graph.getPath(&octomap);

    cout<<"Path:\n"<<res<<endl;
    
    return 0;
}
