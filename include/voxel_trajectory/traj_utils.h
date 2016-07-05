#ifndef _VOXEL_TRAJECTORY_TRAJ_UTILS_H_
#define _VOXEL_TRAJECTORY_TRAJ_UTILS_H_

#include "voxel_trajectory/voxelmacro.h"

#include <voxel_map/basics.h>

#include <eigen3/Eigen/Dense>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace voxel_trajectory
{
    using namespace std;

    inline bool retPointsFromCloud2Msg(const sensor_msgs::PointCloud2 & cloud, 
            vector<voxel_map::Point> & pts)
    {
        const float * raw_data = 
            reinterpret_cast<const float *>(cloud.data.data());
        vector<voxel_map::Point> ret;
        ret.reserve(cloud.width);
        for (size_t idx = 0; idx < cloud.width; ++idx)
            ret.emplace_back(
                    raw_data[getDimIndexByPointIndex<X>(idx)],
                    raw_data[getDimIndexByPointIndex<Y>(idx)],
                    raw_data[getDimIndexByPointIndex<Z>(idx)]);
        pts = move(ret);
        return true;
    }

    inline bool retBoxesFromCloud2Msg(const sensor_msgs::PointCloud2 & cloud,
            vector<voxel_map::Box> & boxes)
    {
        const float * raw_data =
            reinterpret_cast<const float *>(cloud.data.data());
        vector<voxel_map::Box> ret;
        ret.reserve(cloud.width);
        for (size_t idx = 0; idx < cloud.width; ++idx)
        {
            voxel_map::Point lower(
                    raw_data[getDimIndexByBoundaryIndex<LX>(idx)],
                    raw_data[getDimIndexByBoundaryIndex<LY>(idx)],
                    raw_data[getDimIndexByBoundaryIndex<LZ>(idx)]);
            voxel_map::Point upper(
                    raw_data[getDimIndexByBoundaryIndex<RX>(idx)],
                    raw_data[getDimIndexByBoundaryIndex<RY>(idx)],
                    raw_data[getDimIndexByBoundaryIndex<RZ>(idx)]);
            //clog << lower.transpose() << ", "<< upper.transpose();
            ret.emplace_back(lower, upper);
        }
        boxes = move(ret);
        return true;
    }

    inline bool retBoxesFromCloud1Msg(const sensor_msgs::PointCloud & cloud,
            vector<voxel_map::Box> & boxes)
    {
        vector<voxel_map::Box> ret;
        ret.reserve(cloud.points.size());
        for (size_t idx = 0; idx < cloud.points.size(); idx += 2)
        {
            voxel_map::Point lower(
                    cloud.points[idx].x,
                    cloud.points[idx].y,
                    cloud.points[idx].z);
            voxel_map::Point upper(
                    cloud.points[idx + 1].x,
                    cloud.points[idx + 1].y,
                    cloud.points[idx + 1].z);
            ret.emplace_back(lower, upper);
        }
        boxes = move(ret);
        return true;
    }

    inline bool retRaysFromScanOdom(const sensor_msgs::LaserScan & scan,
            const nav_msgs::Odometry & odom, 
            vector<voxel_map::Ray> & rays,
            double height_thld = -1e30,
            double scan_res = EPS,
            int count_thld = 1)
    {

        sensor_msgs::PointCloud cloud;
        laser_geometry::LaserProjection().projectLaser(scan, cloud);

        using namespace Eigen;
        Matrix3d rot = Quaterniond(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z).toRotationMatrix();
        Vector3d trans (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z); 
        vector<voxel_map::Ray> ret;
        ret.reserve(cloud.points.size());

        if (cloud.points.empty()) 
        {
            rays.clear();
            return true;
        }

        bool is_first = true;
        int pt_count = 0;
        Vector3d last_pt(cloud.points.front().x, 
                cloud.points.front().y, 
                cloud.points.front().z);
        for (auto & lp : cloud.points)
        {
            auto pt = rot * Vector3d(lp.x, lp.y, lp.z) + trans;

            if (pt(Z) < height_thld) continue;

            if ((pt - last_pt).norm() > scan_res)
            {
                if (pt_count >= count_thld) ret.emplace_back(trans, last_pt - trans);
                last_pt = pt;
                pt_count = 1;
            }
            else
                pt_count += 1;
        }
        if (pt_count >= count_thld) ret.emplace_back(trans, last_pt - trans);
        rays = move(ret);
        return true;
    }

    bool retRaysFromCloud2Odom(const sensor_msgs::PointCloud2 & cloud2,
            const nav_msgs::Odometry & odom, 
            vector<voxel_map::Ray> & rays,
            double height_thld = -1e30,
            double scan_res = EPS,
            int count_thld = 1)
    {

        pcl::PointCloud<pcl::PointXYZ> pts;
        pcl::fromROSMsg(cloud2, pts);
        using namespace Eigen;
        Matrix3d rot = Quaterniond(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z).toRotationMatrix();
        Vector3d trans (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z); 
        vector<voxel_map::Ray> ret;
        ret.reserve(pts.size());
        if (pts.empty()) 
        {
            rays.clear();
            return true;
        }

        bool is_first = true;
        int pt_count = 0;
        Vector3d last_pt(pts.front().x, 
                pts.front().y, 
                pts.front().z);
        for (auto & lp : pts)
        {
            auto pt = rot * Vector3d(lp.x, lp.y, lp.z) + trans;

            if (pt(Z) < height_thld) continue;

            if ((pt - last_pt).norm() > scan_res)
            {
                if (pt_count >= count_thld) ret.emplace_back(trans, last_pt - trans);
                last_pt = pt;
                pt_count = 1;
            }
            else
                pt_count += 1;
        }
        if (pt_count >= count_thld) ret.emplace_back(trans, last_pt - trans);
        rays = move(ret);
        return true;
    }
}

#endif
