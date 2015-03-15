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
const static int _NODE_NULL = 0;

namespace VoxelTrajectory
{
    using namespace std;
    const double eps = 1e-9;

    OctoMap::Node::Node()
    {
    }

    OctoMap::Node::Node(const double bdy[_TOT_BDY],int son[_TOT_CHD],int id,int tag)
    {
        this->id    = id;

        if (bdy==NULL)
            memset(this->bdy, 0, sizeof(this->bdy));
        else
            memcpy(this->bdy, bdy, sizeof(this->bdy));

        if (son==NULL)
            memset(this->son, _NODE_NULL ,sizeof(this->son));
        else
            memcpy(this->son, son ,sizeof(this->son));

        this->tag   = tag;
    }

    void OctoMap::addNode(const double bdy[_TOT_BDY],int son[_TOT_CHD],int tag)
    {
        node.push_back(OctoMap::Node(bdy,son,N,tag));
        N+=1;
    }

    OctoMap::OctoMap(std::string filename)
    {
        this->loadFromFile(filename);
    }

    OctoMap::OctoMap(const double bdy[_TOT_BDY],double resolution)
    {
        this->N   = 0;
        this->resolution    = resolution;
        // NULL Node
        addNode(NULL,NULL,_TAG_NUL);
        node[0].bdy[_BDY_X] -= eps;
        node[0].bdy[_BDY_Y] -= eps;
        node[0].bdy[_BDY_Z] -= eps;
        // The big picture
        addNode(bdy,NULL,_TAG_EMP);
    }

    static inline bool within(
        const double pt[_TOT_DIM],
        const double bdy[_TOT_BDY])
    {
        return 
            bdy[_BDY_x] < pt[_DIM_x]+eps && pt[_DIM_x] < bdy[_BDY_X] &&
            bdy[_BDY_y] < pt[_DIM_y]+eps && pt[_DIM_y] < bdy[_BDY_Y] &&
            bdy[_BDY_z] < pt[_DIM_z]+eps && pt[_DIM_z] < bdy[_BDY_Z];
    }

    static inline double getVolume(const double bdy[_TOT_BDY])
    {
        return 
            (bdy[_BDY_X] - bdy[_BDY_x])*
            (bdy[_BDY_Y] - bdy[_BDY_y])*
            (bdy[_BDY_Z] - bdy[_BDY_z]);
    }

    void OctoMap::insert(const double pt[_TOT_DIM], int rt)
    {
        //check if the point is within the voxel
        if (rt == _NODE_NULL) return ;
        if (!within(pt,node[rt].bdy)) return;
        
        // set tag
        node[rt].tag    |= _TAG_OBS;

        //check resolution
        if (getVolume(node[rt].bdy) < this->resolution + eps)
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

    static bool Contained(
        const double box[_TOT_BDY],
        const double bdy[_TOT_BDY])
    {
        return box[_BDY_x] < bdy[_BDY_x] + eps && bdy[_BDY_X] < box[_BDY_X] + eps &&
               box[_BDY_y] < bdy[_BDY_y] + eps && bdy[_BDY_Y] < box[_BDY_Y] + eps &&
               box[_BDY_z] < bdy[_BDY_z] + eps && bdy[_BDY_Z] < box[_BDY_Z] + eps;
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

        node[rt].tag    |= _TAG_OBS;

        if (Contained(bdy, node[rt].bdy) || 
            getVolume(node[rt].bdy) < this->resolution + eps)
        {
            node[rt].tag    = _TAG_OBS;
            return ;
        }

        splitNode(rt);

        for (int chd = 0; chd < _TOT_CHD; chd++)
            insertBlock(bdy, node[rt].son[chd]);

        update(rt);
    }

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
        if (node[rt].son[0]) return;

        double * bdy = node[rt].bdy;

        double mid[_TOT_DIM] = 
        {
            (bdy[_BDY_x]+bdy[_BDY_X])*0.5,
            (bdy[_BDY_y]+bdy[_BDY_Y])*0.5,
            (bdy[_BDY_z]+bdy[_BDY_Z])*0.5
        };

        double BDY[_TOT_BDY][2] = 
        {
            bdy[_BDY_x],mid[_DIM_x],mid[_DIM_x],bdy[_BDY_X],
            bdy[_BDY_y],mid[_DIM_y],mid[_DIM_y],bdy[_BDY_Y],
            bdy[_BDY_z],mid[_DIM_z],mid[_DIM_z],bdy[_BDY_Z]
        };

        for (int chd = 0; chd<_TOT_CHD; ++chd)
        {
            double new_bdy[_TOT_BDY]=
            {   
                BDY[_BDY_x][(chd>>_DIM_x)&1],BDY[_BDY_X][(chd>>_DIM_x)&1],
                BDY[_BDY_y][(chd>>_DIM_y)&1],BDY[_BDY_Y][(chd>>_DIM_y)&1],
                BDY[_BDY_z][(chd>>_DIM_z)&1],BDY[_BDY_Z][(chd>>_DIM_z)&1],
            };
            node[rt].son[chd]   = N;
            addNode(new_bdy,NULL,_TAG_EMP);
        }
    }

    // update the tag of the node #rt;
    void OctoMap::update(int rt)
    {
#if 0
        for (int chd = 0; chd<_TOT_CHD; ++chd)
        {
            node[rt].tag    |= node[node[rt].son[chd]].tag;
        }
#else       
        int * son = node[rt].son;
        node[rt].tag    |= node[son[0]].tag | node[son[1]].tag;
        node[rt].tag    |= node[son[2]].tag | node[son[3]].tag;
        node[rt].tag    |= node[son[4]].tag | node[son[5]].tag;
        node[rt].tag    |= node[son[6]].tag | node[son[7]].tag;
#endif
    }


    void OctoMap::query(int rt,int from, const double bdy[_TOT_BDY],
        VoxelGraph * graph)
    {
        // check if they are relavent
        if (!isIntersected(bdy, node[rt].bdy))return;

        // check tag
        if (node[rt].tag==_TAG_MIX)
        {
            for (int chd = 0; chd < _TOT_CHD; chd++)
                query(node[rt].son[chd],from,bdy,graph);
        }else if (node[rt].tag==_TAG_EMP)
        {
            double bdy_i[_TOT_BDY];
            retSharedArea(bdy, node[rt].bdy, bdy_i);
#if 0
            clog<<"[bdy_rt] = ";
            for (int j = 0; j < 6; j++) clog<<node[rt].bdy[j]<<",";
            clog<<"\n";
            clog<<"[bdy_query] = ";
            for (int j = 0; j < 6; j++) clog<<node[rt].bdy[j]<<",";
            clog<<"\n";
            clog<<"[bdy_i] = ";
            for (int j = 0; j < 6; j++) clog<<bdy_i[j]<<",";
            clog<<"\n\n";
#endif
            graph->add_bdy_id_id(bdy_i,rt,from);
        }
    }

    void OctoMap::saveAsFile(std::string filename)
    {
        std::ofstream fout(filename.c_str());

        fout<<N<<"\t"<<resolution<<std::endl;

        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it!=node.end();it++)
        {
            fout<<it->id<<"\t"<<it->tag;

            for (int i = 0; i < _TOT_BDY; i++)
                fout<<"\t"<<it->bdy[i];

            for (int chd = 0; chd < _TOT_CHD; chd++)
                fout<<"\t"<<it->son[chd];

            fout<<std::endl;
        }
    }

    void OctoMap::loadFromFile(std::string filename)
    {
        std::ifstream fin(filename.c_str());

        fin>>N>>resolution;

        node = std::vector<OctoMap::Node>(N);

        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it!=node.end();it++)
        {
            fin>>it->id>>it->tag;
            
            for (int i = 0; i< _TOT_BDY ; i++)
                fin>>it->bdy[i];

            for (int chd = 0; chd < _TOT_CHD; chd++)
                fin>>it->son[chd];
        }
    }
    
    void OctoMap::retBox(int id,double bdy[_TOT_BDY])
    {
        memcpy(bdy,node[id].bdy,sizeof(node[id].bdy));
    }

    void OctoMap::retGraph(VoxelGraph * graph)
    {
        for (std::vector<OctoMap::Node>::iterator it = node.begin();
            it!=node.end();it++)
        {
            if (it->tag!=_TAG_EMP) continue;

            for (int dim=0; dim<_TOT_DIM; dim++)
            {
                double bdy[_TOT_BDY];
                memcpy(bdy,it->bdy,sizeof(bdy));

                bdy[dim*2]  = bdy[dim*2+1];
                bdy[dim*2+1]    += eps;

                query(1, it->id, bdy, graph);
            }
        }
    }

    std::vector<double> OctoMap::getPointCloud()
    {
        std::vector<double> pt;

        for (std::vector<Node>::iterator it = node.begin();
             it != node.end(); ++it)
        {
            if (it->son[0]==_NODE_NULL && it->tag==_TAG_OBS)
            {
                pt.push_back((it->bdy[_BDY_x] + it->bdy[_BDY_X])*0.5);
                pt.push_back((it->bdy[_BDY_y] + it->bdy[_BDY_Y])*0.5);
                pt.push_back((it->bdy[_BDY_z] + it->bdy[_BDY_Z])*0.5);
            }
        }

        return pt;
    }
}
