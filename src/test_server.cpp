#include <iostream>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "voxel_trajectory/voxelserver.h"

using namespace std;

int main(int argc, char ** argv)
{
    int mod;
    istringstream sin(argv[1]);
    sin>>mod;

    if (mod == 0)
    {
        VoxelTrajectory::VoxelServer server;
        double bdy[6];

        ifstream fin(argv[2]);

        fin>>bdy[0] >> bdy[1] >> bdy[2] >> bdy[3] >> bdy[4] >> bdy[5];
        server.setMapBoundary(bdy);
        clog<<"DEBUG: set up boundary fine."<<endl;

        int n;
        fin >> n;
        clog<<" TOT N = "<<n<<endl;
        vector<double> pt(n*3);
        for (int i=0; i<n*3; i++)
            fin >> pt[i];
        server.setPointCloud(pt);
        clog<<"DEBUG: set up boundary fine."<<endl;

        vector<double > s(9),t(9);
        for (int i = 0; i< 3 * 3;i++)
            if (i<3) 
                fin>>s[i];
            else    
                s[i]=0;

        for (int i = 0; i < 3 * 3; i++)
            if (i<3)
                fin>>t[i];
            else
                t[i]=0;
        clog<<"DEBUG: Fine Before set points"<<endl;
        server.setPoints(s, t, 0.0);

        double t_ [] = {server.getBeginTime(), server.getFinalTime()};
        
        cout<<"Point Cloud:"<<endl;
        pt = server.getPointCloud();
        for (int i=0; i<pt.size(); i++)
        {
            cout<<pt[i]<<" ";
            if (i%3 == 2) cout<<endl;
        }
        cout<<"P:\n"<< server.getPolyCoeff()<<endl;
        cout<<"T:\n"<< server.getTimeAllocation()<<endl;

    }
    return 0;
}
