
#ifndef _VOXEL_TRAJECTORY_OCTO_MAP_H_
#define _VOXEL_TRAJECTORY_OCTO_MAP_H_

#include "voxel_trajectory/voxel_graph.h"
#include "voxel_trajectory/voxelmacro.h"

#include <vector>
#include <list>
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
        std::vector<VoxelGraph::NodePtr> graph_node_ptr;

        //for gragh
        std::vector<int> log;

        inline bool isAtom(const Node & node);

        void addNode(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int tag);
        void update(int rt);
        void splitNode(int rt);
        //void insert(const double pt[_TOT_DIM], int rt);
        //void insertBlock(const double bdy[_TOT_DIM], int rt);
        void insertPoint(const double pt[_TOT_DIM], int rt);
        void insertBlock(const double bdy[_TOT_BDY], int rt);
        bool testEmpty(const double bdy[_TOT_DIM], int rt);

        void connectTo(int rt, int src, const double bdy[_TOT_BDY]);
        void dealWithLog();
public:
        // construction function
        OctoMap(const double bdy[_TOT_BDY], double resolution);
        OctoMap(std::string filename);
        ~OctoMap();

        // insert a point 
        //void insert(const double pt[_TOT_DIM]);
        void insertPoints(const std::vector<double> & pt);
        void insertBlocks(const std::vector<double> & blk);

        // query about some volume
        //void query(int rt, int from, const double bdy[_TOT_BDY], VoxelGraph * graph);
        int queryPoint(int rt, const double pt[_TOT_DIM]);

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
            getPath(const double src[_TOT_DIM], const double dest[_TOT_DIM]);

        void saveAsFile(std::string  filename);
        void loadFromFile(std::string filename);

        //
        void retBox(int id,double bdy[_TOT_BDY]);
        //void retGraph(VoxelGraph * graph);
        std::vector<double> getPointCloud();

        bool testObstacle(const double pt[_TOT_DIM]);
        // for grid inflation
        bool testEmpty(const double bdy[_TOT_BDY]);
        bool inflateBdy(double bdy[_TOT_BDY], int direction[_TOT_BDY], 
                int inflation_lim = -1);
    };
}
#endif
