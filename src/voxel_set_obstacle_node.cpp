#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

const int _TOT_BDY = 6;
const int _DIM_BDY_x = 0;
const int _DIM_BDY_X = 1;
const int _DIM_BDY_y = 2;
const int _DIM_BDY_Y = 3;
const int _DIM_BDY_z = 4;
const int _DIM_BDY_Z = 5;

ros::Publisher pubObs;
sensor_msgs::PointCloud2 msgPubObs;

using namespace std;

int main(int argc, char ** argv)
{
    // init the node
    ros::init(argc, argv, "voxel_set_obstale_node");
    ros::NodeHandle handle("~");

    // init the pub
    pubObs = handle.advertise<sensor_msgs::PointCloud2>("obstacle_points", 2);

    // init the file
    string filepathObs;
    handle.getParam("obstacle_file", filepathObs);
    fstream fin(filepathObs);
    
    // input the obs
    int nObs;
    fin >> nObs;
    vector<vector<double> > vecObs;
    vecObs.resize(nObs);
    for (size_t idx = 0; idx < nObs; ++idx)
    {
        vecObs[idx].resize(_TOT_BDY);
        fin >> vecObs[idx][_DIM_BDY_x] >> vecObs[idx][_DIM_BDY_X]
            >> vecObs[idx][_DIM_BDY_y] >> vecObs[idx][_DIM_BDY_Y]
            >> vecObs[idx][_DIM_BDY_z] >> vecObs[idx][_DIM_BDY_Z];
        ROS_WARN("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf"
            , vecObs[idx][_DIM_BDY_x] , vecObs[idx][_DIM_BDY_X]
            , vecObs[idx][_DIM_BDY_y] , vecObs[idx][_DIM_BDY_Y]
            , vecObs[idx][_DIM_BDY_z] , vecObs[idx][_DIM_BDY_Z]);
    }

    // init the msg
    msgPubObs.height    = 1;
    msgPubObs.width     = nObs;
    msgPubObs.is_dense  = true;
    msgPubObs.is_bigendian  = false;
    msgPubObs.point_step    = 6 * 4;
    msgPubObs.row_step      = msgPubObs.width * msgPubObs.point_step;

    // init the fileds
    sensor_msgs::PointField f;
    msgPubObs.fields.resize(_TOT_BDY);
    string f_name[] = {string("x"), string("X"), 
        string("y"), string("Y"), string("z"), string("Z")};
    for (size_t idx = 0; idx < _TOT_BDY; ++idx)
    {
        f.name      = f_name[idx];
        f.offset    = idx << 2;
        f.datatype  = 7;
        f.count     = 1;
        msgPubObs.fields[idx] = f;
    }

    float * data = new float[nObs * _TOT_BDY];
    for (size_t idx = 0; idx < (size_t)nObs; ++idx)
        for (size_t dim = 0; dim < _TOT_BDY; ++dim)
            data [idx * _TOT_BDY + dim] = vecObs[idx][dim];

    msgPubObs.data.resize(msgPubObs.row_step);
    uint8_t * dataRaw = reinterpret_cast<uint8_t *>(data);

    for (size_t idx = 0; idx < msgPubObs.row_step; ++idx)
        msgPubObs.data[idx] = dataRaw[idx];
    delete data;

    ros::Rate wait_rate(2);
    while (ros::ok() && pubObs.getNumSubscribers() ==0)
        wait_rate.sleep();
    
    pubObs.publish(msgPubObs);
    ros::spinOnce();

    return 0;
}
