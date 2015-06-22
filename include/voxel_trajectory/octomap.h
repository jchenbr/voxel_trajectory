
#ifndef _VOXEL_TRAJECTORY_OCTO_MAP_H_
#define _VOXEL_TRAJECTORY_OCTO_MAP_H_

#include "voxel_trajectory/voxelgraph.h"
#include "voxel_trajectory/voxelmacro.h"

#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

namespace VoxelTrajectory
{

    class OctoMap
    {
private:
        double resolution;
        std::vector<double> atom;

        struct Node
        {
            int id;
            int tag;
            double bdy[_TOT_BDY];
            int son[_TOT_CHD];
            Node();
            Node(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int id, int tag);
        };

        std::vector<Node> node;

        inline bool isLeaf(const Node & node);

        void addNode(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int tag);
        void update(int rt);
        void splitNode(int rt);
        void insert(const double pt[_TOT_DIM], int rt);
        void insertBlock(const double bdy[_TOT_DIM], int rt);
        bool testEmpty(const double bdy[_TOT_DIM], int rt);
public:
        // construction function
        OctoMap(const double bdy[_TOT_BDY], double resolution);
        OctoMap(std::string filename);

        // insert a point 
        void insert(const double pt[_TOT_DIM]);
        void insertBlock(const double bdy[_TOT_BDY]);

        // query about some volume
        void query(int rt, int from, const double bdy[_TOT_BDY], VoxelGraph * graph);

        void saveAsFile(std::string  filename);
        void loadFromFile(std::string filename);

        //
        void retBox(int id,double bdy[_TOT_BDY]);
        void retGraph(VoxelGraph * graph);
        std::vector<double> getPointCloud();

        // for grid inflation
        bool testEmpty(const double bdy[_TOT_BDY]);
        bool inflateBdy(double bdy[_TOT_BDY], int direction[_TOT_BDY], 
                int inflation_lim = -1);
    };
}
#endif
