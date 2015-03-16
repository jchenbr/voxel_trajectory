#include "voxel_trajectory/voxelgraph.h"
#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/voxelmacro.h"

#include <cmath>
#include <string.h>
#include <algorithm>
#include <map>
#include <iostream>
const static int _EDGE_NULL    = -1;
const static int _NODE_NULL    = -1;
static int _NODE_S   = 0;
static int  _NODE_T  = 1;
const static double _TURN_PENALTY  = 1.6;
const static double _EPS = 1e-9;
const static double _INF = 1e100;

namespace VoxelTrajectory
{
    using namespace std;
    VoxelGraph::Node::Node(int beg,double bdy[_TOT_BDY])
    {
        this->beg   = beg;
        memcpy(this->bdy,bdy,sizeof(this->bdy));
    }

    VoxelGraph::Edge::Edge(int nxt,double cost,int bid,int to)
    {
        this->nxt   = nxt;
        this->cost  = cost;
        this->bid   = bid;
        this->to    = to;
    }

    void VoxelGraph::addNode(double bdy[_TOT_BDY])
    {
        node.push_back(Node(_EDGE_NULL,bdy));
        N   += 1;
    }

    void VoxelGraph::addEdge(int from_u,int to_v,double cost, int bid)
    {
        edge.push_back(Edge(node[from_u].beg,cost,bid,to_v));
        node[from_u].beg    = E;
        E   +=1;
    }

    static inline int getRevEdgeID(int eid)
    {
        return eid^1;
    }

    void VoxelGraph::SetUp(double pt_s[_TOT_DIM], double pt_t[_TOT_DIM], OctoMap * octomap)
    {
        double bdy_s[_TOT_BDY]=
        {
            pt_s[_DIM_x],pt_s[_DIM_x] + _EPS,
            pt_s[_DIM_y],pt_s[_DIM_y] + _EPS,
            pt_s[_DIM_z],pt_s[_DIM_z] + _EPS
        };

        double bdy_t[_TOT_BDY]=
        {
            pt_t[_DIM_x],pt_t[_DIM_x] + _EPS,
            pt_t[_DIM_y],pt_t[_DIM_y] + _EPS,
            pt_t[_DIM_z],pt_t[_DIM_z] + _EPS
        };

#if _USE_DEBUG_PRINT_
        for (int j = 0; j < 6; j++) clog<<bdy_s[j]<<",";
        clog<<"\n";
        clog<<"[bdy_t] = ";
        for (int j = 0; j < 6; j++) clog<<bdy_t[j]<<",";
        clog<<"\n";
#endif
        int old_E = E, old_N = N, old_pr = bid_nid.size();

        octomap->query(1, -1, bdy_s, this);
        octomap->query(1, -2, bdy_t, this);

#if 0
        clog
            << bdy_s[0] <<" |"
            << bdy_s[1] <<" |"
            << bdy_s[2] <<" |"
            << bdy_s[3] <<" |"
            << bdy_s[4] <<" |"
            << bdy_s[5] <<"\n";

        clog
            << bdy_t[0] << " |"
            << bdy_t[1] << " |"
            << bdy_t[2] << " |"
            << bdy_t[3] << " |"
            << bdy_t[4] << " |"
            << bdy_t[5] << "\n";
#endif

        connectNodesWithSameBid(old_pr);

        //clog<<"graph = " << bid_nid.size() <<" [ "<< E << "," << old_E <<"],["<< N << "," << old_N << " ]\n";

        // For path
        hasGotPath = false;
        _path_ = getPath(octomap);


        // For recovery
        E   = old_E;
        N   = old_N;

        for (int u = 0; u < N; ++ u)
            while (node[u].beg > E)
                node[u].beg = edge[node[u].beg].nxt;

        node.resize(N);
        edge.resize(E);
        bid_nid.resize(old_pr);
    }

    VoxelGraph::VoxelGraph(OctoMap * octomap)
    {
        N    = 0;
        E    = 0; 

        hasGotPath   = false;

        octomap->retGraph(this);
        connectNodesWithSameBid(0);
    }

    static inline double getSurface(double bdy[_TOT_BDY])
    {
        double x    = bdy[_BDY_X]-bdy[_BDY_x];
        double y    = bdy[_BDY_Y]-bdy[_BDY_y];
        double z    = bdy[_BDY_Z]-bdy[_BDY_z];
        double surface  = (x*y+y*z+z*x)*2;
        return surface;
    }

    static inline bool isAllowed(double bdy[_TOT_BDY])
    {
        return 
            bdy[_BDY_x] < bdy[_BDY_X] &&
            bdy[_BDY_y] < bdy[_BDY_Y] &&
            bdy[_BDY_z] < bdy[_BDY_Z];
    }

    void VoxelGraph::add_bdy_id_id(double bdy[_TOT_BDY],int bid_a,int bid_b)
    {
        //if (isAllowed(bdy)) return ;

        bid_nid.push_back(std::pair<int,int>(bid_a,N));
        bid_nid.push_back(std::pair<int,int>(bid_b,N));
#if 0
        {
            std::cout<<"surface="<<getSurface(bdy)<<","<<(getSurface(bdy)>_EPS)<<std::endl;
            std::cout<<"\t"<<bdy[0]<<","<<bdy[1]<<std::endl;
            std::cout<<"\t"<<bdy[2]<<","<<bdy[3]<<std::endl;
            std::cout<<"\t"<<bdy[4]<<","<<bdy[5]<<std::endl;
        }
#endif
        addNode(bdy);
    }

    static inline double sqr(double x) {return x*x;}

    static inline double norm(double pt_a[_TOT_DIM],double pt_b[_TOT_DIM])
    {
        return std::sqrt( 
            sqr(pt_a[_DIM_x]-pt_b[_DIM_x])+
            sqr(pt_a[_DIM_y]-pt_b[_DIM_y])+
            sqr(pt_a[_DIM_z]-pt_b[_DIM_z]));
    }

    static inline double getDistance(double bdy_a[_TOT_BDY],double bdy_b[_TOT_BDY])
    {
        double a[_TOT_DIM]=
        {
            (bdy_a[_BDY_x]+bdy_a[_BDY_X])*0.5,
            (bdy_a[_BDY_y]+bdy_a[_BDY_Y])*0.5,
            (bdy_a[_BDY_z]+bdy_a[_BDY_Z])*0.5
        };

        double b[_TOT_DIM]=
        {
            (bdy_b[_BDY_x]+bdy_b[_BDY_X])*0.5,
            (bdy_b[_BDY_y]+bdy_b[_BDY_Y])*0.5,
            (bdy_b[_BDY_z]+bdy_b[_BDY_Z])*0.5
        };

        return norm(a,b);
    }

    static inline double max(double a, double b){ return (a>b)?a:b;}
    static inline double min(double a, double b){ return (a<b)?a:b;}

    static inline double getTurnCost(double bdy_a[_TOT_BDY],double bdy_b[_TOT_BDY])
    {
        double bdy[_TOT_BDY]    = 
        {
            max(bdy_a[_BDY_x],bdy_b[_BDY_x]),min(bdy_a[_BDY_X],bdy_b[_BDY_X]),
            max(bdy_a[_BDY_y],bdy_b[_BDY_y]),min(bdy_a[_BDY_Y],bdy_b[_BDY_Y]),
            max(bdy_a[_BDY_z],bdy_b[_BDY_z]),min(bdy_a[_BDY_Z],bdy_b[_BDY_Z]),
        };
        if ( 
            (bdy[_BDY_X]-bdy[_BDY_x])*
            (bdy[_BDY_Y]-bdy[_BDY_y])*
            (bdy[_BDY_Z]-bdy[_BDY_z])>
            _EPS)
            return _TURN_PENALTY;
        else
            return 1.0;
    }


    void VoxelGraph::connectNodesWithSameBid(int beg)
    {
        if (beg == 0)
            sort(bid_nid.begin(), bid_nid.end());

        for (int i = beg; i < bid_nid.size(); i++)
        {
            int bid = bid_nid[i].first;
            int u   = bid_nid[i].second;
            int v;

            for (int j = i-1; j >= 0; --j)
            {
                if (bid_nid[j].first != bid) 
                    if (beg == 0)
                        break;
                    else
                        continue;

                v   = bid_nid[j].second;

                double cost = getDistance(node[u].bdy,node[v].bdy);

                if (beg==0) cost *= getTurnCost(node[u].bdy,node[v].bdy); 

                addEdge(u,v,cost,bid);
                addEdge(v,u,cost,bid);
            }
        }
    }



    void VoxelGraph::calBestPath()
    {
        if (hasGotPath || node.size() < 2) return ;

        _NODE_S = N - 2;
        _NODE_T = N - 1;

        std::multimap<double,int> weight2nid;
        std::vector<double> best(N,_INF);
        std::vector<int>    last(N,_NODE_NULL);
        std::vector<double> heu(N);



        //std::cout<<"OK1,n="<<N<<std::endl;
        for (int nid=0; nid<N; nid++)
            heu[nid]    = getDistance(node[nid].bdy,node[_NODE_T].bdy);

        //std::cout<<"OK2,E="<<E<<std::endl;
        int u   = _NODE_S,v,eid;
        best[u]   = 0;
        //std::cout<<"OK3,n="<<node.size()<<std::endl;

        while (u!=_NODE_T)
        {
            //std::cout<<"ok4,u="<<u<<","<<std::endl;
            //std::cout<<"ok4,eid="<<eid<<","<<std::endl;
            //std::cout<<"ok4,u_eid="<<node[u].beg<<","<<_EDGE_NULL<<std::endl;
            for (eid = node[u].beg;eid!=_EDGE_NULL; eid=edge[eid].nxt)
            {
                //std::cout<<"bool="<<(eid!=_EDGE_NULL)<<std::endl;
                v   = edge[eid].to;
                //std::cout<<"v="<<v<<std::endl;
                if (best[u] + edge[eid].cost <best[v])
                {
                    best[v] = best[u] + edge[eid].cost;
                    last[v] = eid;
                    weight2nid.insert(std::pair<double,int>(best[v]+heu[v],v));
                }
            }
            
            //std::cout<<"ok5,"<<weight2nid.size()<<","<<std::endl;

            std::multimap<double,int>::iterator it = weight2nid.begin();
            if (it == weight2nid.end()) break ;

            u   = it->second;
            weight2nid.erase(it);
        }
        if (last[_NODE_T] == _NODE_NULL) return ;

        u   = _NODE_T;
        path_nid    = vector<int> (0);
        path_bid    = vector<int> (0);
        while (u!=_NODE_S)
        {
            eid = getRevEdgeID(last[u]);
            u   = edge[eid].to;

            path_nid.push_back(u);
            path_bid.push_back(edge[eid].bid);
        }
        hasGotPath  = true;
    }

    Eigen::MatrixXd VoxelGraph::getPath(OctoMap *octomap)
    {
        //std::cout<<"OK,"<<std::endl;
        calBestPath();
        //clog << "HasGotPath" << hasGotPath <<std::endl;

        if (!hasGotPath) return Eigen::MatrixXd(0,0);

        int M=path_bid.size(), mid=0;
        
        //std::cout<<"M="<<M<<std::endl;

        Eigen::MatrixXd ret(2*M,_TOT_BDY);

#if 1
        ret.row(mid++)<<
            node[_NODE_S].bdy[_BDY_x],
            node[_NODE_S].bdy[_BDY_y],
            node[_NODE_S].bdy[_BDY_z],
            node[_NODE_T].bdy[_BDY_x],
            node[_NODE_T].bdy[_BDY_y],
            node[_NODE_T].bdy[_BDY_z];
        
        //std::cout<<"Mid="<<mid<<std::endl;

        double box[_TOT_BDY];
        for (int pid=M-1; pid>=0; pid--)
        {
            octomap->retBox(path_bid[pid],box);
            //std::cout<<"box_bdy,bid="<<path_bid[pid]<<std::endl;
            //std::cout<<"\t"<<box[0]<<","<<box[1]<<std::endl;
            //std::cout<<"\t"<<box[2]<<","<<box[3]<<std::endl;
            //std::cout<<"\t"<<box[4]<<","<<box[5]<<std::endl;
                
            ret.row(mid++)<<
                box[_BDY_x],box[_BDY_X],
                box[_BDY_y],box[_BDY_Y],
                box[_BDY_z],box[_BDY_Z];
        }

        //std::cout<<"Mid="<<mid<<std::endl;
        double *win;
        for (int pid=M-2; pid>=0; pid--)
        {
            win = node[path_nid[pid]].bdy;
            //std::cout<<"windows_bdy"<<std::endl;
            //std::cout<<"\t"<<win[0]<<","<<win[1]<<std::endl;
            //std::cout<<"\t"<<win[2]<<","<<win[3]<<std::endl;
            //std::cout<<"\t"<<win[4]<<","<<win[5]<<std::endl;

            ret.row(mid++)<<
                win[_BDY_x],win[_BDY_X],
                win[_BDY_y],win[_BDY_Y],
                win[_BDY_z],win[_BDY_Z];
        }
#endif

        return ret;
    }
}
