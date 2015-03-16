
#ifndef _VOXEL_TRAJECTORY_VOXEL_GRAPH_H_
#define _VOXEL_TRAJECTORY_VOXEL_GRAPH_H_

#include "voxel_trajectory/voxelmacro.h"

#include <vector>
#include <utility>
#include <eigen3/Eigen/Dense>

namespace VoxelTrajectory
{
    class OctoMap;
    class VoxelGraph
    {
    private:

        struct Node
        {
            int beg;
            double bdy[_TOT_BDY];
            Node(){};
            Node(int beg,double bdy[_TOT_BDY]);
        };

        struct Edge
        {
            int nxt; // the nxt edge;
            int bid; // in wich box;
            int to;  // directed edge; 
            double cost;
            Edge(){};
            Edge(int to,double cost,int bid,int nxt);
        };
     

        bool hasGotPath, isReady;
        int N,E;
        // save the graph
        std::vector<Node> node;
        std::vector<Edge> edge;
        // for connection
        std::vector<std::pair<int,int> > bid_nid;

        // for the path
        std::vector<int> path_nid;
        std::vector<int> path_bid;

        // funcitonal methods
        void addNode(double bdy[_TOT_BDY]);
        void addEdge(int from_u,int to_v,double cost,int bid);

        // procedural methods 
        void connectNodesWithSameBid(int beg);
        void calBestPath();

        Eigen::MatrixXd _path_;

        Eigen::MatrixXd getPath(OctoMap *octomap);

    public:
        // accessory methods
        VoxelGraph(OctoMap *octomap);

        void SetUp(double pt_s[_TOT_DIM], double pt_t[_TOT_DIM], OctoMap * octomap);

        Eigen::MatrixXd getPathMatrix()
        { if (hasGotPath) return _path_; else return Eigen::MatrixXd::Zero(0,_TOT_BDY);}

        // communicational metods
        void add_bdy_id_id(double bdy[_TOT_BDY],int bid_a,int bid_b);
    };
}
#endif
