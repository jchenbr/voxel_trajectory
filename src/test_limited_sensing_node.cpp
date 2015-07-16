
#include "voxel_trajectory/octomap.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <cmath>
#include <cstring>

using namespace std;

class LimitedSensringTester
{
private:
    ros::Subscriber _odom_sub;
    ros::Subscriber _blks_sub;
    ros::Subscriber _pts_sub;
    ros::Publisher _pts_pub;

    bool _has_odom = false;

    VoxelTrajectory::OctoMap * _map;
    nav_msgs::Odometry _odom;

    double _sensing_radius = 3.0;           // meter
    double _sensing_resolution = 0.1;       // arc
    int _sensing_rate = 100;                // Hz
    vector<float> _sensed_pts;

    const double _PI = acos(-1.0);          // arc
    const double _EPS = 1e-7;
public:

    LimitedSensringTester(ros::NodeHandle & handle, ros::NodeHandle & map_handle)
    {
        double bdy[_TOT_BDY], resolution;
        map_handle.param("map/boundary/lower_x", bdy[_BDY_x], -100.0);
        map_handle.param("map/boundary/upper_x", bdy[_BDY_X], 100.0);
        map_handle.param("map/boundary/lower_y", bdy[_BDY_y], -100.0);
        map_handle.param("map/boundary/upper_y", bdy[_BDY_Y], 100.0);
        map_handle.param("map/boundary/lower_z", bdy[_BDY_z], 0.0);
        map_handle.param("map/boundary/upper_z", bdy[_BDY_Z], 200.0);
        map_handle.param("map/resolution", resolution, 0.4);
        _map = new VoxelTrajectory::OctoMap(bdy, resolution);

        ROS_INFO("[LIMITED_SENSING] map size : %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",
                bdy[0], bdy[1], bdy[2], bdy[3], bdy[4], bdy[5]);

        handle.param("limited_sensing/sensing_radius", _sensing_radius, 3.0);
        handle.param("limited_sensing/sensing_resolution", _sensing_resolution, 0.1);
        handle.param("limited_sensing/sensing_rate", _sensing_rate, 100);

        string _file;
        handle.getParam("limited_sensing/obstacle_file", _file);
        this->loadFromFile(_file);

        _odom_sub = 
            handle.subscribe("odometry", 50, &LimitedSensringTester::setOdometry, this);
        _blks_sub = 
            handle.subscribe("obstacle_blocks", 2, &LimitedSensringTester::setObstacleBlocks, this);
        _pts_sub = 
            handle.subscribe("obstacle_points", 2, &LimitedSensringTester::setObstaclePoints, this);
        _pts_pub = 
            handle.advertise<sensor_msgs::PointCloud2>("sensed_points", 20);
    }

    int getRate()
    {
        return this->_sensing_rate;
    }

    void setOdometry(const nav_msgs::Odometry & odom)
    {
        _odom = odom;
        _has_odom = true;

    }

    void loadFromFile(const string file)
    {
        ifstream fin(file);
        if (!fin.is_open()) return;

        int n;
        fin >> n;
        vector<double> blk(n * _TOT_BDY);

        for (int idx = 0; idx < n * _TOT_BDY; ++idx)
        {
            fin >> blk[idx];
            if (idx & 1) blk[idx] += 0.0 + _EPS;
            else blk[idx] -= 0.0;
        }
        _map->insertBlocks(blk);
        ROS_INFO("[LIMITED_SENSING] %lu blocks were added!", blk.size() / _TOT_BDY);
        auto pts = _map->getPointCloud();
        ROS_INFO("[LIMITED_SENSING] %lu to be added, %d", pts.size(), _map->testObstacle(pts.data()));
    }

    void setObstaclePoints(const sensor_msgs::PointCloud2 & pts_cloud)
    {
        const float * data = reinterpret_cast<const float *>(pts_cloud.data.data());
        vector<double> pt;
        pt.reserve(pts_cloud.width * _TOT_BDY);
        for (size_t idx = 0; idx < pts_cloud.width; ++idx)
        {
            pt.push_back(data[idx * _TOT_DIM + _DIM_x] - _EPS);
            pt.push_back(data[idx * _TOT_DIM + _DIM_x] + _EPS);
            pt.push_back(data[idx * _TOT_DIM + _DIM_y] - _EPS);
            pt.push_back(data[idx * _TOT_DIM + _DIM_y] + _EPS);
            pt.push_back(data[idx * _TOT_DIM + _DIM_z] - _EPS);
            pt.push_back(data[idx * _TOT_DIM + _DIM_z] + _EPS);
        }
        _map->insertBlocks(pt);
    }

    void setObstacleBlocks(const sensor_msgs::PointCloud2 & blks_cloud)
    {
        const float * data = reinterpret_cast<const float *>(blks_cloud.data.data());
        vector<double> blk;
        blk.reserve(blks_cloud.width * _TOT_BDY);
        for (size_t idx = 0; idx < blks_cloud.width; ++idx)
        {
            blk.push_back(data[idx * _TOT_BDY + _BDY_x] - _EPS);
            blk.push_back(data[idx * _TOT_BDY + _BDY_X] + _EPS);
            blk.push_back(data[idx * _TOT_BDY + _BDY_y] - _EPS);
            blk.push_back(data[idx * _TOT_BDY + _BDY_Y] + _EPS);
            blk.push_back(data[idx * _TOT_BDY + _BDY_z] - _EPS);
            blk.push_back(data[idx * _TOT_BDY + _BDY_Z] + _EPS);
        }
        _map->insertBlocks(blk);
    }

    void pubSensedPoints()
    {
        _sensed_pts.reserve(
                _sensed_pts.size() + 
                static_cast<int>(
                    _PI * 2.0 * _PI / (_sensing_resolution * _sensing_resolution)
                    )
                );

        for (double ltd = -0.5 * _PI; ltd <= 0.5 * _PI; ltd += _sensing_resolution)
        {
            for (double atd = 0.0; atd  <= 2.0 * _PI; atd += _sensing_resolution)
            {
                double pt[_TOT_DIM] =
                {
                    _odom.pose.pose.position.x + _sensing_radius * cos(ltd) * sin(atd),
                    _odom.pose.pose.position.y + _sensing_radius * cos(ltd) * cos(atd),
                    _odom.pose.pose.position.z + _sensing_radius * sin(ltd)
                };
                if (!_map->testObstacle(pt)) continue;
                _sensed_pts.push_back(pt[_DIM_x]);
                _sensed_pts.push_back(pt[_DIM_y]);
                _sensed_pts.push_back(pt[_DIM_z]);
            }
        }
        ROS_INFO("[LIMITED_SENSING] Odometry received, %lu points to be added.", _sensed_pts.size());

        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = "/map";
        cloud.header.stamp = _odom.header.stamp;

        cloud.height = 1;
        cloud.width = _sensed_pts.size() / 3;
        cloud.is_bigendian = false;
        cloud.is_dense = true;
        cloud.point_step = 4 * _TOT_DIM;
        cloud.row_step = cloud.point_step * cloud.width;

        sensor_msgs::PointField field;
        cloud.fields.resize(_TOT_DIM);
        string f_name[_TOT_DIM] = {"x", "y", "z"};
        for (size_t idx = 0; idx < _TOT_DIM; ++idx)
        {
            field.name = f_name[idx];
            field.offset = idx * 4;
            field.datatype = 7;
            field.count = 1;
            cloud.fields[idx] =field;
        }

        cloud.data.clear();
        cloud.data.reserve(cloud.row_step);
        uint8_t * pt_int = reinterpret_cast<uint8_t *>(_sensed_pts.data());
        for (size_t idx = 0; idx < cloud.row_step; ++idx)
        {
            cloud.data.push_back(pt_int[idx]);
        }
        _pts_pub.publish(cloud);
        _sensed_pts.clear();
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "limited_sensing_tester");
    ros::NodeHandle handle("~"), map_handle("/trajectory_generator/");

    LimitedSensringTester tester(handle, map_handle);


    ros::Rate loop_rate(tester.getRate());
    while (ros::ok())
    {
        tester.pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

