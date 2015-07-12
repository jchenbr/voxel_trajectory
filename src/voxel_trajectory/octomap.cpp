#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/voxelmacro.h"

#include <string.h>
#include <fstream>
#include <list>
#include <vector>
#include <iostream>
const static int _TAG_NUL   = 0;
const static int _TAG_EMP   = 1;
const static int _TAG_OBS   = 2;
const static int _TAG_MIX   = 3;
const static int _TAG_TAL   = 4;

const static int _NODE_NULL = 0;
const static int _NODE_ROOT = 1;

namespace VoxelTrajectory
{
    using namespace std;
    const double eps = 1e-9;

    OctoMap::Node::Node()
    {
    }

    OctoMap::Node::Node(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int id, int tag)
    {
        assert(_TAG_NUL <= tag  && tag < _TAG_TAL);

        this->id    = id;

        if (bdy == NULL)
        {
            memset(this->bdy, 0, sizeof(this->bdy));
        }
        else
        {
            memcpy(this->bdy, bdy, sizeof(this->bdy));
        }

        if (son == NULL)
        {
            memset(this->son, 0, sizeof(this->bdy));
        }
        else
        {
            memcpy(this->son, son, sizeof(this->son));
        }

        this->tag   = tag;
    }

    void OctoMap::addNode(const double bdy[_TOT_BDY], const int son[_TOT_CHD], int tag)
    {
        node.push_back(OctoMap::Node(bdy, son, (int)node.size(), tag));
    }

    OctoMap::OctoMap(std::string filename)
    {
        this->loadFromFile(filename);
    }

    OctoMap::OctoMap(const double bdy[_TOT_BDY], double resolution)
    {
        // it must be a legal 3-D space;
        assert
            (
                (bdy[_BDY_x] < bdy[_BDY_X]) && 
                (bdy[_BDY_y] < bdy[_BDY_Y]) &&
                (bdy[_BDY_z] < bdy[_BDY_Z])
            );
        //clog << "?????????????????????????????????????????????????????????" << endl;

        // this is the allowed minimal grid volume
        this->resolution    = resolution;

        // NULL Node
        addNode(NULL, NULL, _TAG_NUL);
        node[_NODE_NULL].bdy[_BDY_X] -= eps;
        node[_NODE_NULL].bdy[_BDY_Y] -= eps;
        node[_NODE_NULL].bdy[_BDY_Z] -= eps;

        log.reserve(10000);

        // The big picture, set as tree root
        addNode(bdy, NULL, _TAG_EMP);

        atom.resize(_TOT_DIM);
        atom[_DIM_x] = bdy[_BDY_X] - bdy[_BDY_x];
        atom[_DIM_y] = bdy[_BDY_Y] - bdy[_BDY_y];
        atom[_DIM_z] = bdy[_BDY_Z] - bdy[_BDY_z];

        while (atom[_DIM_x] * atom[_DIM_y] * atom[_DIM_z] > resolution)
        {
            atom[_DIM_x] *= 0.5;
            atom[_DIM_y] *= 0.5;
            atom[_DIM_z] *= 0.5;
        }

    }

    inline bool OctoMap::isAtom(const Node & node)
    {
        return (node.bdy[_BDY_X] - node.bdy[_BDY_x] < this->atom[_DIM_x] + eps);
    }

    OctoMap::~OctoMap()
    {
        for (size_t idx = 0; idx < node.size(); ++idx)
        {
            if (isAtom(node[idx])) VoxelGraph::delNode(graph_node_ptr[idx]);
        }
    }

    static inline bool within(
        const double pt[_TOT_DIM],
        const double bdy[_TOT_BDY])
    {
        return 
            bdy[_BDY_x] < pt[_DIM_x] + eps && pt[_DIM_x] < bdy[_BDY_X] &&
            bdy[_BDY_y] < pt[_DIM_y] + eps && pt[_DIM_y] < bdy[_BDY_Y] &&
            bdy[_BDY_z] < pt[_DIM_z] + eps && pt[_DIM_z] < bdy[_BDY_Z];
    }

    static inline double getVolume(const double bdy[_TOT_BDY])
    {
        return 
            (bdy[_BDY_X] - bdy[_BDY_x]) *
            (bdy[_BDY_Y] - bdy[_BDY_y]) *

            (bdy[_BDY_Z] - bdy[_BDY_z]);
    }

#if 0
    void OctoMap::insert(const double pt[_TOT_DIM])
    {
        insert(pt, _NODE_ROOT);
    }

    void OctoMap::insert(const double pt[_TOT_DIM], int rt)
    {
        //check if the point is within the voxel
        if (rt == _NODE_NULL) return ;
        if (!within(pt,node[rt].bdy)) return;
        
        // set tag
        node[rt].tag    |= _TAG_OBS;

        //check resolution
        if (isAtom(node[rt]))
        {
            node[rt].tag    = _TAG_OBS;
            return ;
        }
        
        splitNode(rt);

        //insert son
        for (int chd = 0; chd < _TOT_CHD; chd++)
            insert(pt, node[rt].son[chd]);

        //Update
        update(rt);
    }
#endif
    static inline double max(double a,double b)
    {return (a>b)?a:b;}

    static inline double min(double a,double b)
    {return (a<b)?a:b;}

    static inline void retSharedArea(
        const double bdy_a[_TOT_BDY],
        const double bdy_b[_TOT_BDY],
        double bdy[_TOT_BDY])
    {
        bdy[_BDY_x] = max(bdy_a[_BDY_x], bdy_b[_BDY_x]);
        bdy[_BDY_X] = min(bdy_a[_BDY_X], bdy_b[_BDY_X]);

        bdy[_BDY_y] = max(bdy_a[_BDY_y], bdy_b[_BDY_y]);
        bdy[_BDY_Y] = min(bdy_a[_BDY_Y], bdy_b[_BDY_Y]);

        bdy[_BDY_z] = max(bdy_a[_BDY_z], bdy_b[_BDY_z]);
        bdy[_BDY_Z] = min(bdy_a[_BDY_Z], bdy_b[_BDY_Z]);
    }

    static inline bool isHot(const double bdy[_TOT_BDY])
    {
        return (
            bdy[_BDY_x] < bdy[_BDY_X]  &&
            bdy[_BDY_y] < bdy[_BDY_Y]  &&
            bdy[_BDY_z] < bdy[_BDY_Z] );
    }

    static bool isIntersected(
        const double box[_TOT_BDY],
        const double bdy[_TOT_BDY])
    {
        double tmp[_TOT_BDY];

        retSharedArea(box, bdy, tmp);

        return isHot(tmp);
    }

    static bool testEnclose(
        const double box[_TOT_BDY],
        const double bdy[_TOT_BDY])
    {
        return box[_BDY_x] < bdy[_BDY_x] + eps && bdy[_BDY_X] < box[_BDY_X] + eps &&
               box[_BDY_y] < bdy[_BDY_y] + eps && bdy[_BDY_Y] < box[_BDY_Y] + eps &&
               box[_BDY_z] < bdy[_BDY_z] + eps && bdy[_BDY_Z] < box[_BDY_Z] + eps;
    }

   void OctoMap::insertPoint(const double pt[_TOT_DIM], int rt)
    {
        if (rt == _NODE_NULL) return ;
        if (!within(pt, node[rt].bdy)) return ;

        node[rt].tag |= _TAG_OBS;

        if (isAtom(node[rt]))
        {
            node[rt].tag = _TAG_OBS;
            return ;
        }

        splitNode(rt);
        
        for (size_t chd = 0; chd < _TOT_CHD; ++chd)
            insertPoint(pt, node[rt].son[chd]);

        update(rt);
    }

    void OctoMap::insertPoints(const vector<double> & pt)
    {
        assert(pt % _TOT_DIM == 0);
        
        log.clear();

        // insert
        for (size_t idx = 0; idx < pt.size(); idx += _TOT_DIM)
        {
            double pt[_TOT_DIM] 
            {
                pt[idx + _DIM_x], 
                pt[idx + _DIM_y], 
                pt[idx + _DIM_z]
            };
            insertPoint(pt, _NODE_ROOT);
        }

        // delete graph node
        
        // add graph node
        dealWithLog();
    }

    void printBlock(const double bdy[_TOT_BDY])
    {
        clog << bdy[0] << ", " << bdy[1] << ", " << bdy[2] << ", ";
        clog << bdy[3] << ", " << bdy[4] << ", " << bdy[5] << endl;
    }

    void OctoMap::insertBlock(const double bdy[_TOT_BDY], int rt)
    {
        if (rt == _NODE_NULL) return;
        //clog << "inserting : " ; printBlock(bdy);
        //clog << " to :" ; printBlock(node[rt].bdy);
        if (!isIntersected(bdy, node[rt].bdy)) return ;

        if (testEnclose(bdy, node[rt].bdy) || isAtom(node[rt]))
        {
            node[rt].tag    = _TAG_OBS;
            return ;
        }

        splitNode(rt);

        for (size_t chd = 0; chd < _TOT_CHD; ++chd)
            insertBlock(bdy, node[rt].son[chd]);

        update(rt);
    }

    void OctoMap::insertBlocks(const vector<double> & blk)
    {
        assert(blk % _TOT_BDY == 0);

        log.clear();

        // insert graph node
        for (size_t idx = 0; idx < blk.size(); idx += _TOT_BDY)
        {
            double bdy[_TOT_BDY]
            {
                blk[idx + _BDY_x], blk[idx + _BDY_X],
                blk[idx + _BDY_y], blk[idx + _BDY_Y],
                blk[idx + _BDY_z], blk[idx + _BDY_Z]
            };
            //clog << "Inserting Block ..." << endl;
            insertBlock(bdy, _NODE_ROOT);
        }

        //clog << "going to deal with log." << endl;

        // delete graph node
        
        // add graph node
        
        dealWithLog();

        //clog << "Dealed log." << endl;
    }

#if 0
    void OctoMap::insertBlock(const double bdy[_TOT_BDY])
    {
        assert(
                (bdy[_BDY_x] < bdy[_BDY_X]) && 
                (bdy[_BDY_y] < bdy[_BDY_Y]) && 
                (bdy[_BDY_z] < bdy[_BDY_Z])
              );

        insertBlock(bdy, _NODE_ROOT);
    }

    void OctoMap::insertBlock(const double bdy[_TOT_BDY], int rt)
    {
#if 0
        clog<<"<"<< bdy[0] << ", " << bdy[1] << ", " << bdy[2] 
            << ", " << bdy[3] << ", " << bdy[4] << ", " << bdy[5]<<">"<<endl;

        clog<<"<"<< node[rt].bdy[0] << ", " << node[rt].bdy[1] << ", " << node[rt].bdy[2] 
            << ", " << node[rt].bdy[3] << ", " << node[rt].bdy[4] << ", " << node[rt].bdy[5]<<">"<<endl;
        clog<< "{" << isIntersected(bdy, node[rt].bdy) << ", " << Contained(bdy, node[rt].bdy)
            << ", " << (getVolume(node[rt].bdy) < this->resolution + eps)<<" }"<<endl;
#endif
        if (rt == _NODE_NULL) return;
        if (!isIntersected(bdy, node[rt].bdy)) return ;

        splitNode(rt);

        node[rt].tag |= _TAG_OBS;

        if (testEnclose(bdy, node[rt].bdy) || isAtom(node[rt]))
        {
            node[rt].tag    = _TAG_OBS;
            return ;
        }

        for (int chd = 0; chd < _TOT_CHD; chd++)
            insertBlock(bdy, node[rt].son[chd]);

        update(rt);
    }
#endif

#if 0
    static bool PointWithinSphere(
        const double p[_TOT_DIM],
        const double pt[_TOT_DIM], const double r)
    {
        double tmp  =   (p[_DIM_x] - pt[_DIM_x]) * (p[_DIM_x] - pt[_DIM_x]) + 
                        (p[_DIM_y] - pt[_DIM_y]) * (p[_DIM_y] - pt[_DIM_y]) +   
                        (p[_DIM_z] - pt[_DIM_z]) * (p[_DIM_z] - pt[_DIM_z]);
        return sqrt(tmp) < r + eps;
    }

    static bool isSphereContained(
        const double pt[_TOT_DIM], const double r, 
        const double bdy[_TOT_BDY])
    {
        return true;
    }

    static bool SphereContained(
        const double pt[_TOT_DIM], const double r,
        const double bdy[_TOT_DIM])
    {
        bool ret = true;

        for (double mask = 0; msk < (1<<3); msk++)
        {
            double p[] = {bdy[ 0 | (msk&1)], 
                          bdy[ 2 | ((msk>>1) & 1)] ,
                          bdy[ 4 | ((msg>>2) & 1)]};
            ret = ret && PointWithinSphere(p, pt, r);
        }

        return ret;
    }

    void OctoMap::insertSphere(const double pt[_TOT_DIM], const double r, int rt)
    {
        if (rt == _NODE_NULL) return ;
        if (!isSphereIntersected(pt, r, node[rt].bdy)) return ;

        node[rt].tag    |= _TAG_OBS;

        if (SphereContained(pt, r, node[rt].bdy) ||
            getVolume(node[rt].bdy) + eps > this->resolution)
        {
            node[rt].tag    = _TAG_OBS;
        }

        splitNode(rt);

        for (int chd = 0; chd < _TOT_CHD; chd++)
            insertSphere(pt, r, node[rt].son[chd]);

        update(rt);
    }
#endif

    // split the node into 8 subnode;
    void OctoMap::splitNode(int rt)
    {
        if (node[rt].son[0] || isAtom(node[rt])) return;

        log.push_back( -rt );

        double * bdy = node[rt].bdy;

        double mid[_TOT_DIM] = 
        {
            (bdy[_BDY_x] + bdy[_BDY_X]) * 0.5,
            (bdy[_BDY_y] + bdy[_BDY_Y]) * 0.5,
            (bdy[_BDY_z] + bdy[_BDY_Z]) * 0.5
        };

        double BDY[_TOT_BDY][2] = 
        {
            bdy[_BDY_x], mid[_DIM_x], mid[_DIM_x], bdy[_BDY_X],
            bdy[_BDY_y], mid[_DIM_y], mid[_DIM_y], bdy[_BDY_Y],
            bdy[_BDY_z], mid[_DIM_z], mid[_DIM_z], bdy[_BDY_Z]
        };

        for (int chd = 0; chd < _TOT_CHD; ++chd)
        {
            double new_bdy[_TOT_BDY] =
            {   
                BDY[_BDY_x][(chd >> _DIM_x) & 1], BDY[_BDY_X][(chd >> _DIM_x) & 1],
                BDY[_BDY_y][(chd >> _DIM_y) & 1], BDY[_BDY_Y][(chd >> _DIM_y) & 1],
                BDY[_BDY_z][(chd >> _DIM_z) & 1], BDY[_BDY_Z][(chd >> _DIM_z) & 1],
            };
            node[rt].son[chd] = (int)node.size();
            log.push_back((int) node.size());
            
            addNode(new_bdy, NULL, node[rt].tag);
        }
    }

    void OctoMap::connectTo(int rt, int src, const double bdy[_TOT_BDY])
    {
        /*
        if (src == 10)
        {
            clog << "src bdy : "; printBlock(bdy);
            clog << "to  bdy : "; printBlock(node[rt].bdy);
            clog << " tag = " << node[rt].tag << endl;
        }
        */

        if (!isIntersected(node[rt].bdy, bdy)) return ;

        if (node[rt].tag == _TAG_MIX)
        {
            for (size_t chd = 0; chd < _TOT_CHD; ++chd)
            {
                //if (src == 10) clog << "rt = " << rt << ", chd = " << chd << endl;
                connectTo(node[rt].son[chd], src, bdy);
            }
        }
        else if (node[rt].tag == _TAG_EMP)
        {
            double bdy_i[_TOT_BDY];
            retSharedArea(node[rt].bdy, bdy, bdy_i);
            //if (src == 10){clog << "src = " << src << ", to = " << rt << endl;}
            VoxelGraph::addEdge(graph_node_ptr[rt], graph_node_ptr[src], bdy_i);
        }
    }

    void OctoMap::dealWithLog()
    {
        int old_id = (int) graph_node_ptr.size();
        graph_node_ptr.resize(node.size());

        /*
        clog << "id list : \n";
        for (auto id : log) clog << id << ", ";
        clog << endl;
        */

        for (auto id : log)
        {
            if (id < -1 && (-id) < old_id)
            { 
                VoxelGraph::delNode(graph_node_ptr[-id]);
                graph_node_ptr[-id] = NULL;
            }

            if (id > 1 && node[id].tag == _TAG_EMP) 
            {
                graph_node_ptr[id] = VoxelGraph::addNode(id, node[id].bdy);
            }
        }

        for (auto id : log)
        {
            if (id > 1 && node[id].tag == _TAG_EMP) 
            {
                double bdy[_TOT_BDY];
                for (int dim = 0; dim < _TOT_BDY; dim += 2)
                {
                    memcpy(bdy, node[id].bdy, sizeof(double) * _TOT_BDY);

                    //if (id == 10) {clog << "\nsource : "; printBlock(node[id].bdy);}

                    //bdy[dim] = node[id].bdy[dim] - atom[dim >> 1] * 0.5 - eps;
                    //bdy[dim | 1] = node[id].bdy[dim] - atom[dim >> 1] * 0.5 + eps; 
                    bdy[dim] = bdy[dim | 1] = node[id].bdy[dim];
                    bdy[dim] -= eps;
                    connectTo(_NODE_ROOT, id, bdy);

                    //bdy[dim] = node[id].bdy[dim | 1] + atom[dim >> 1] * 0.5 - eps;
                    //bdy[dim | 1] = node[id].bdy[dim | 1] + atom[dim >> 1] * 0.5 + eps;
                    bdy[dim] = bdy[dim | 1] = node[id].bdy[dim | 1];
                    bdy[dim] += eps;
                    connectTo(_NODE_ROOT, id, bdy);
                }

                /*
                if (id == 10)
                {
                    clog << "id = " << 10 << ", neighbor to " << graph_node_ptr[id]->nxt.size() << endl;
                }
                */
            }
        }

        log.clear();
    }

    // update the tag of the node #rt;
    void OctoMap::update(int rt)
    {
#if 0
        for (int chd: node[rt].son)
        {
            node[rt].tag |= node[chd].tag;
        }
#else       
        int * son = node[rt].son;
        node[rt].tag |= 
            node[son[0]].tag | node[son[1]].tag | 
            node[son[2]].tag | node[son[3]].tag |
            node[son[4]].tag | node[son[5]].tag |
            node[son[6]].tag | node[son[7]].tag;
#endif
    }


#if 0
    void OctoMap::query(int rt,int from, const double bdy[_TOT_BDY],
        VoxelGraph * graph)
    {
#if 0
            if (from < 0){
            clog<<"[bdy_rt] = ";
            for (int j = 0; j < 6; j++) clog<<node[rt].bdy[j]<<",";
            clog<<"\n";
            clog<<"[bdy_query] = ";
            for (int j = 0; j < 6; j++) clog<<bdy[j]<<",";
            clog<<"\n";
           }
#endif

        // check if they are relavent
        if (!isIntersected(bdy, node[rt].bdy))return;

        // check tag
        if (node[rt].tag == _TAG_MIX)
        {
            for (int chd = 0; chd < _TOT_CHD; chd++)
                query(node[rt].son[chd], from, bdy, graph);
        }
        else if (node[rt].tag == _TAG_EMP)
        {
            double bdy_i[_TOT_BDY];
            retSharedArea(bdy, node[rt].bdy, bdy_i);
#if 0    
           if (from < 0)
           {
            clog<<"[bdy_inter] = ";
            for (int j = 0; j < 6; j++) clog<<bdy_i[j]<<",";
            clog<<"\n";
           }
#endif
            graph->add_bdy_id_id(bdy_i, rt, from);
        }
    }
#endif

    int OctoMap::queryPoint(int rt, const double pt[_TOT_DIM])
    {
        if (node[rt].son[0] == _NODE_NULL) return rt;
#if 0
        clog << "Pt  : "; clog << pt[0] << ", " << pt[1] << ", " << pt[2] << endl;
        clog << "Box : "; printBlock(node[rt].bdy);
        clog << "son : "; for (auto chd : node[rt].son) clog << chd << ", "; clog << endl;
        clog << "node size : " << node.size() << ", " << graph_node_ptr.size() << endl;
        clog << endl;
#endif

        for (size_t chd = 0; chd < _TOT_CHD; ++chd) 
        {
            if (within(pt, node[node[rt].son[chd]].bdy)) 
                return queryPoint(node[rt].son[chd], pt);
        }
        return rt;
    }

    bool OctoMap::testObstacle(const double pt[_TOT_DIM])
    {
        return node[queryPoint(_NODE_ROOT, pt)].tag != _TAG_EMP;
    }


    pair<Eigen::MatrixXd, Eigen::MatrixXd>
        OctoMap::getPath(const double src[_TOT_DIM], const double dest[_TOT_DIM])
    {
        if (!within(src, node[_NODE_ROOT].bdy) || !within(dest, node[_NODE_ROOT].bdy))
            return make_pair(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0));

        //clog << "src = " << src[0] << ", " << src[1] << ", " << src[2] << endl;
        //clog << "dest = " << dest[0] << ", " << dest[1] << ", " << dest[2] << endl;

        auto src_id = queryPoint(_NODE_ROOT, src);
        auto dest_id = queryPoint(_NODE_ROOT, dest);
        if (node[src_id].tag != _TAG_EMP || node[dest_id].tag != _TAG_EMP) 
            return make_pair(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0));

        //clog << "id " << src_id << ", " << dest_id << endl;

        auto p_src_node = graph_node_ptr[src_id];
        auto p_dest_node = graph_node_ptr[dest_id];

        //clog << "address : " << p_src_node << ", " << p_dest_node << endl;

        //clog << "src : " << p_src_node->id << ", dest : " << p_dest_node->id << endl;

        double bdy[_TOT_BDY];

        for (int dim = 0; dim < _TOT_BDY; ++dim) bdy[dim] = src[dim >> 1]; 
        auto p_src_edge = new VoxelGraph::Edge(NULL, p_src_node, bdy);
        p_src_node->nxt[-1] = p_src_edge;
        //clog << "src pos : "; printBlock(bdy);

        for (int dim = 0; dim < _TOT_BDY; ++dim) bdy[dim] = dest[dim >> 1];
        auto p_dest_edge = new VoxelGraph::Edge(p_dest_node, NULL, bdy);
        p_dest_node->nxt[-2] = p_dest_edge;
        //clog << "dest pos : "; printBlock(bdy);

    
        auto ret = VoxelGraph::getPathAStar(p_src_edge, p_dest_edge);
        //clog << "edge :\n" << ret.first << endl << "node:\n" << ret.second << endl;

        p_src_node->nxt.erase(-1);
        delete p_src_edge;

        p_dest_node->nxt.erase(-2);
        delete p_dest_edge;

        return ret;
    }

    void OctoMap::saveAsFile(std::string filename)
    {
        std::ofstream fout(filename.c_str());

        fout << node.size() << "\t" << resolution << std::endl;

        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it != node.end(); ++it)
        {
            fout << it->id << "\t" << it->tag;

            for (int i = 0; i < _TOT_BDY; i++)
                fout << "\t" << it->bdy[i];

            for (int chd = 0; chd < _TOT_CHD; chd++)
                fout << "\t" << it->son[chd];

            fout << std::endl;
        }
    }

    void OctoMap::loadFromFile(std::string filename)
    {
        std::ifstream fin(filename.c_str());

        int N;
        fin >> N >> resolution;

        node = std::vector<OctoMap::Node>(N);

        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it!=node.end();it++)
        {
            fin >> it->id >> it->tag;
            
            for (int i = 0; i< _TOT_BDY ; i++)
                fin >> it->bdy[i];

            for (int chd = 0; chd < _TOT_CHD; chd++)
                fin >> it->son[chd];
        }
    }
    
    void OctoMap::retBox(int id,double bdy[_TOT_BDY])
    {
        memcpy(bdy, node[id].bdy, sizeof(node[id].bdy));
    }

#if 0
    void OctoMap::retGraph(VoxelGraph * graph)
    {
        for (auto it = node.begin(); it!=node.end(); ++it)
        {
            if (it->tag != _TAG_EMP) continue;

            double bdy[_TOT_BDY];

            for (int dim = 0; dim < _TOT_DIM; dim++)
            {
                memcpy(bdy, it->bdy, sizeof(bdy));
                bdy[dim << 1] = bdy[dim << 1 | 1];
                bdy[dim << 1 | 1] += eps;

                query(_NODE_ROOT, it->id, bdy, graph);
            }
        }
    }
#endif

    std::vector<double> OctoMap::getPointCloud()
    {
        std::vector<double> pt;

        for (auto it = node.begin(); it != node.end(); ++it)
        {
            if (it->son[0] == _NODE_NULL && it->tag == _TAG_OBS)
            {
                pt.push_back((it->bdy[_BDY_x] + it->bdy[_BDY_X]) * 0.5);
                pt.push_back((it->bdy[_BDY_y] + it->bdy[_BDY_Y]) * 0.5);
                pt.push_back((it->bdy[_BDY_z] + it->bdy[_BDY_Z]) * 0.5);
            }
        }

        clog << "[TRAJ] Map points already in octomap module." << endl;
        return pt;
    }

    bool OctoMap::testEmpty(const double bdy[_TOT_BDY])
    {
        assert(
                (bdy[_BDY_x] < bdy[_BDY_X]) &&
                (bdy[_BDY_y] < bdy[_BDY_Y]) &&
                (bdy[_BDY_z] < bdy[_BDY_Z])
              );
        return testEmpty(bdy, _NODE_ROOT);
    }

    bool OctoMap::testEmpty(const double bdy[_TOT_BDY], int rt)
    {
        if (rt == _NODE_NULL) return true;
        if (!isIntersected(bdy, node[rt].bdy)) return true;

        if (node[rt].tag == _TAG_EMP) return true;

        splitNode(rt);

        if (testEnclose(bdy, node[rt].bdy) || isAtom(node[rt]))
            return node[rt].tag == _TAG_EMP;

        bool ret = true;
        for (int chd = 0; chd < _TOT_CHD; chd++)
            ret = ret && testEmpty(bdy, node[rt].son[chd]);

        update(rt);
        return ret;
    }

    bool OctoMap::inflateBdy(double bdy[_TOT_BDY], int direction[_TOT_BDY], int inflate_lim)
    {
        int L = 0, R = (node[_NODE_ROOT].bdy[_BDY_X] - node[_NODE_NULL].bdy[_BDY_x] + eps) 
            / atom[_DIM_x];

        if (inflate_lim >= 0) R = min(R, inflate_lim);

        for (int dim = 0; dim < _TOT_BDY; ++dim)
            if (direction[dim])
                R = min(R, abs(node[_NODE_ROOT].bdy[dim] - bdy[dim] + eps) / atom[dim >> 1]);

        int mid;
       
        while (L < R)
        {
            //clog << "L : " << L << ", Mid : " << mid << ", R : " << R << endl;
            mid = (L + R + 1) >> 1;
            double sub_bdy[_TOT_BDY] = 
            {
                bdy[_BDY_x] + mid * direction[_BDY_x] * atom[_DIM_x],
                bdy[_BDY_X] + mid * direction[_BDY_X] * atom[_DIM_x],
                bdy[_BDY_y] + mid * direction[_BDY_y] * atom[_DIM_y],
                bdy[_BDY_Y] + mid * direction[_BDY_Y] * atom[_DIM_y],
                bdy[_BDY_z] + mid * direction[_BDY_z] * atom[_DIM_z],
                bdy[_BDY_Z] + mid * direction[_BDY_Z] * atom[_DIM_z],
            };
            if (testEmpty(sub_bdy))
                L = mid;
            else
                R = mid - 1;
        }
        //clog << "[Finally] L : " << L << ", R : " << R << endl;

        for (int dim = 0; dim < _TOT_BDY; ++dim)
            bdy[dim] += L * direction[dim] * atom[dim >> 1];

        return testEmpty(bdy);
    }
}
