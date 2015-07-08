#ifndef _VOXEL_TRAJECTORY_VOXEL_GRAPH_H_
#define _VOXEL_TRAJECTORY_VOXEL_GRAPH_H_

#include "voxel_trajectory/voxelmacro.h"
#include "voxel_trajectory/voxel_graph.h"

#include <map>
#include <eigen3/Eigen/Dense>

namespace VoxelTrajectory
{
    namespace VoxelGraph
    {
        /* #0. Basic Types */
        struct Node;
        struct Edge;
        typedef Node * NodePtr;
        typedef Edge * EdgePtr;
        typedef const Node * ConstNodePtr;
        typedef const Edge * ConstEdgePtr;

        struct Node
        {
            std::map<int, EdgePtr> nxt;
            int id;
            double bdy[_TOT_BDY];
            Node(){};
            Node(int nid, const double bdy[_TOT_BDY]);
        };

        struct Edge
        {
            NodePtr p_left, p_right;
            double bdy[_TOT_BDY];
            int id;
            Edge(){};
            Edge(NodePtr lhs, NodePtr rhs, const double bdy[_TOT_BDY]);
            NodePtr next(ConstNodePtr p_node);
        };


        /* #1. manipulate node and edge */
        NodePtr addNode(int id, const double bdy[_TOT_BDY]);
        EdgePtr addEdge(NodePtr lhs, NodePtr rhs, const double bdy[_TOT_BDY]);
        void delNode(NodePtr p_node);
        void delEdge(EdgePtr p_edge);

        /* #2. graph algorithms */
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> 
            getPathAStar(EdgePtr p_start_edge, EdgePtr p_target_edge);
    }
}

#endif
