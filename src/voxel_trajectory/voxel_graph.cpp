
#include "voxel_trajectory/voxel_graph.h"

#include <memory>
#include <cstring>
#include <cmath>
#include <iostream>

using namespace VoxelTrajectory;
using namespace VoxelTrajectory::VoxelGraph;
using namespace std;

const int _EDGE_ID_START = -1;
const int _EDGE_ID_TARGET = -2;
const int _EDGE_ID_UNKNOWN = -3;
const int _RESERVE_SIZE = 10000;

VoxelGraph::Node::Node(int nid, const double bdy[_TOT_BDY])
{
    this->id = nid;
    memcpy(this->bdy, bdy, sizeof(double) * _TOT_BDY);
}

VoxelGraph::Edge::Edge(NodePtr lhs, NodePtr rhs, const double bdy[_TOT_BDY])
{
    p_left = lhs;
    p_right = rhs;
    id = _EDGE_ID_UNKNOWN;
    memcpy(this->bdy, bdy, sizeof(double) * _TOT_BDY);
}

NodePtr Edge::next(ConstNodePtr p_node)
{
    return p_node == p_left ? p_right : p_left;
}

NodePtr VoxelGraph::addNode(int id, const double bdy[_TOT_BDY])
{
    return new Node(id, bdy);
}

EdgePtr VoxelGraph::addEdge(NodePtr lhs, NodePtr rhs, const double bdy[_TOT_BDY])
{
    assert(lhs != NULL && rhs !=NULL);

    if (lhs->nxt.count(rhs->id) > 0) return NULL;

    EdgePtr ret = new Edge(lhs, rhs, bdy);
    lhs->nxt[rhs->id] = ret;
    rhs->nxt[lhs->id] = ret;

    return ret;
}

void VoxelGraph::delEdge(EdgePtr p_edge)
{
    if (p_edge->p_left != NULL && p_edge->p_right != NULL)
    {
        p_edge->p_left->nxt.erase(p_edge->p_right->id);
        p_edge->p_right->nxt.erase(p_edge->p_left->id);
    }

    delete p_edge;
}

void VoxelGraph::delNode(NodePtr p_node)
{
    for (auto & pr : p_node->nxt)
    {
        auto p_nxt = pr.second->next(p_node);
        if (p_nxt != NULL) p_nxt->nxt.erase(p_node->id);
        delete pr.second;
    }

    delete p_node;
}

pair<Eigen::MatrixXd, Eigen::MatrixXd> VoxelGraph::getPathAStar(EdgePtr p_start_edge, EdgePtr p_target_edge)
{
    multimap<double, EdgePtr> weight_pedge;
    vector<EdgePtr> id_edge;
    vector<double> opt_val;
    vector<double> heu_val;
    vector<NodePtr> lst_node;
    vector<EdgePtr> lst_edge;

    id_edge.reserve(_RESERVE_SIZE);
    opt_val.reserve(_RESERVE_SIZE);
    heu_val.reserve(_RESERVE_SIZE);
    lst_edge.reserve(_RESERVE_SIZE);
    lst_node.reserve(_RESERVE_SIZE);

    // the cost is distance between the centers of two edge.
    const static auto getCost = [](ConstEdgePtr u, ConstEdgePtr v)
    {
        const static auto sq = [] (double && val)
        {
            return val * val;
        };

        return sqrt(
                sq(u->bdy[_BDY_x] + u->bdy[_BDY_X] - 
                    v->bdy[_BDY_x] - v->bdy[_BDY_X]) +

                sq(u->bdy[_BDY_y] + u->bdy[_BDY_Y] - 
                    v->bdy[_BDY_y] - v->bdy[_BDY_Y]) +

                sq(u->bdy[_BDY_z] + u->bdy[_BDY_Z] -
                    v->bdy[_BDY_z] - v->bdy[_BDY_Z])
                ) * 0.5;
    };

    EdgePtr u = p_start_edge;
    NodePtr p_node;

    // init for the very fast edge
    u->id = (int)id_edge.size();
    id_edge.push_back(u);
    heu_val.push_back(getCost(u, p_target_edge));
    opt_val.push_back(0.0);
    lst_edge.push_back(NULL);
    lst_node.push_back(NULL);

    // update the current edge p_edge using last edge p_pre
    auto update = [&](EdgePtr p_edge, EdgePtr p_pre, NodePtr p_node) -> void
    {
        if (p_edge == p_pre) return ;
        double cost = getCost(p_edge, p_pre);

        if (p_edge->id == _EDGE_ID_UNKNOWN)
        {
            p_edge->id = (int)id_edge.size();
            id_edge.push_back(p_edge);
            heu_val.push_back(getCost(p_edge, p_target_edge));
            
            opt_val.push_back(cost + opt_val[p_pre->id]);
            lst_edge.push_back(p_pre);
            lst_node.push_back(p_node);
            weight_pedge.insert(make_pair(opt_val[p_edge->id] + heu_val[p_edge->id], p_edge));
        }
        else
        {
            if (cost + opt_val[p_pre->id] < opt_val[p_edge->id])
            {
                opt_val[p_edge->id] = cost + opt_val[p_pre->id];
                lst_edge[p_edge->id] = p_pre;
                lst_node[p_edge->id] = p_node;
                weight_pedge.insert(make_pair(opt_val[p_edge->id] + heu_val[p_edge->id], p_edge));
            }
        }
        //clog << "update " << p_edge->id << ", " << p_pre->id << endl;
    };

    // A * begin here
    while (u != p_target_edge)
    {
        double pt [_TOT_DIM] 
        {
            0.5 * (u->bdy[0] + u->bdy[1]), 
            0.5 * (u->bdy[2] + u->bdy[3]),
            0.5 * (u->bdy[4] + u->bdy[5])
        };

        //clog << "now = " << u->id << ", last address = " << lst_node[u->id] << endl;

        //if (lst_node[u->id]) clog << ", last = " <<  lst_node[u->id]->id << endl;

        if ((p_node = u->p_left) != NULL)
        {
            //clog << "left = " << p_node->id << ", nxt to :" << p_node->nxt.size() << endl;
            for (auto & pr: p_node->nxt) update(pr.second, u, p_node);
        }

        if ((p_node = u->p_right) != NULL)
        {
            //clog << "left = " << p_node -> id << ", nxt to :" << p_node->nxt.size() << endl;
            for (auto & pr: p_node->nxt) update(pr.second, u, p_node);
        }

        if (weight_pedge.empty()) break;

        u = weight_pedge.begin() -> second;

        weight_pedge.erase(weight_pedge.begin());
    }

    if (u != p_target_edge) return make_pair(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0));

    // calculate the matrix output 
    int M = 0;

    for (u = p_target_edge; u != p_start_edge; u = lst_edge[u->id]) M += 1;

    Eigen::MatrixXd node(_TOT_BDY, M), edge(_TOT_BDY, M + 1);

    int m = 1;
    for (u = p_target_edge; u != p_start_edge; u = lst_edge[u->id])
    {
        edge.col(M + 1 - m) << 
            u->bdy[_BDY_x], u->bdy[_BDY_X],
            u->bdy[_BDY_y], u->bdy[_BDY_Y],
            u->bdy[_BDY_z], u->bdy[_BDY_Z];

        p_node = lst_node[u->id];
        node.col(M - m) << 
            p_node->bdy[_BDY_x], p_node->bdy[_BDY_X], 
            p_node->bdy[_BDY_y], p_node->bdy[_BDY_Y], 
            p_node->bdy[_BDY_z], p_node->bdy[_BDY_Z]; 
        m += 1;
    }

    edge.col(0) << 
            u->bdy[_BDY_x], u->bdy[_BDY_X],
            u->bdy[_BDY_y], u->bdy[_BDY_Y],
            u->bdy[_BDY_z], u->bdy[_BDY_Z];

    // recovery
    for (auto v: id_edge) v->id = _EDGE_ID_UNKNOWN;
    return make_pair(edge, node);
}
