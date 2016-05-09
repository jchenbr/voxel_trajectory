#ifndef _VOXEL_TRAJECTORY_MACRO_
#define _VOXEL_TRAJECTORY_MACRO_

// old conversions, 
namespace VoxelTrajectory
{
    const int  _BDY_x = 0;
    const int  _BDY_X = 1;
    const int  _BDY_y = 2;
    const int  _BDY_Y = 3;
    const int  _BDY_z = 4;
    const int  _BDY_Z = 5;

    const int  _DIM_x = 0;
    const int  _DIM_y = 1;
    const int  _DIM_z = 2;

    const int  _TOT_BDY = 6;
    const int  _TOT_DIM = 3;
    const int  _TOT_CHD = 8;
}

// new conversion since April, 2016
namespace voxel_trajectory
{
    constexpr int X = 0;
    constexpr int Y = 1;
    constexpr int Z = 2;
    
    constexpr int LX = 0;
    constexpr int RX = 1;
    constexpr int LY = 2;
    constexpr int RY = 3;
    constexpr int LZ = 4;
    constexpr int RZ = 5;
    
    constexpr int NUM_DIM = 3;
    constexpr int NUM_BDY = 6;
    constexpr double EPS = 1e-9;
    
    template <int dim> 
    int getDimIndexByPointIndex(int pt_idx)
    {
        return pt_idx * NUM_DIM + dim;
    }
    
    template <int dim> 
    int getDimIndexByBoundaryIndex(int bdy_idx)
    {
        return bdy_idx * NUM_BDY + dim;
    }
}

#endif
