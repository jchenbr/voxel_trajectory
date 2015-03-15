#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <eigen3/Eigen/Dense>


#include "voxel_trajectory/octomap.h"
#include "voxel_trajectory/voxelgraph.h"
#include "voxel_trajectory/trajectorygenerator.h"

#include <fstream>

using namespace std;

class test_graph
{
public:
    void add_bdy_id_id(double bdy[6],int x,int y)
    {
        for (int i=0;i<3;i++)
        {
            cout<<bdy[i*2]<<"\t"<<bdy[i*2+1]<<endl;
        }
        cout<<"("<<x<<","<<y<<")"<<endl;
    }
};

int main(int argc,char ** argv)
{

    char mod = argv[1][0];
    // only generate the map
    if (mod=='1')
    {
        ifstream fin(argv[3]);
        double bdy[6];

        for (int i=0;i<6;i++)
            fin >> bdy[i];

        VoxelTrajectory::OctoMap octomap(bdy,5*5*5);

        int N;
        fin >> N;
        for (int i=0;i<N;i++)
        {
            double pt[3];
            for (int j=0;j<3;j++)
                fin >> pt[j];
            octomap.insert(pt);
        } 
        octomap.saveAsFile(string(argv[2]));
    } 

    
    // wasted
    if (mod=='2')
    {
        string dumpfile(argv[2]);
        VoxelTrajectory::OctoMap octomap(dumpfile);

        double bdy[6]={40,60,40,60,40,60};
    }
    // only display the path
    if (mod=='3')
    {
        string dumpfile(argv[2]);
        VoxelTrajectory::OctoMap octomap(dumpfile);
        
        double p_s[3]={1,1,1};
        double p_t[3]={99,99,99};
        VoxelTrajectory::VoxelGraph graph(&octomap,p_s,p_t);


        Eigen::MatrixXd res = graph.getPath(&octomap);

#if 1
        for (int i=0;i<res.rows();i++)
        {
            cout<<res.row(i)<<endl;
        }
#endif
    }

    if (mod=='4')
    {
        string dumpfile(argv[2]);
        VoxelTrajectory::OctoMap octomap(dumpfile);
        
        double p_s[3]={1,1,1};
        double p_t[3]={99,99,99};
        VoxelTrajectory::VoxelGraph graph(&octomap,p_s,p_t);


        Eigen::MatrixXd res = graph.getPath(&octomap);

#if 1
        cout<<"Voxel path:"<<endl;
        cout<<res<<endl;
        cout<<endl;
#endif
        VoxelTrajectory::TrajectoryGenerator traj_gen;
        pair<Eigen::MatrixXd,Eigen::MatrixXd> ans = 
            traj_gen.genPolyCoeffTime(res);

        cout<<"Poly coeffecients:"<<endl;
        cout<<ans.first<<endl;
        cout<<"Time allocation:"<<endl;
        cout<<ans.second<<endl;
    }

    if (mod == '5')
    {
        ifstream fin(argv[2]);
        cout<<"the in path file:\n"<<argv[2]<<endl;
        int M;
        fin>>M;
        Eigen::MatrixXd data(M*2,6);
        for (int i=0;i<M*2;i++)
            for (int j=0;j<6;j++)fin>>data(i,j);

        cout<<"the data:\n"<<data<<endl;

        VoxelTrajectory::TrajectoryGenerator traj_gen;
        pair<Eigen::MatrixXd,Eigen::MatrixXd> ans =
            traj_gen.genPolyCoeffTime(data);

        cout<<"Poly coeffecients:"<<endl;
        cout<<ans.first<<endl;
        cout<<"Time allocation:"<<endl;
        cout<<ans.second<<endl;
    }

    if (mod == '6')
    {
        ifstream fin(argv[2]);
        double bdy[6];

        for (int i=0;i<6;i++)
            fin >> bdy[i];

        VoxelTrajectory::OctoMap octomap(bdy,5*5*5);

        int N;
        fin >> N;
        for (int i=0;i<N;i++)
        {
            double pt[3];
            for (int j=0;j<3;j++)
                fin >> pt[j];
            octomap.insert(pt);
        } 
        double p_s[3];
        double p_t[3];
        fin>>p_s[0] >> p_s[1] >> p_s[2];
        fin>>p_t[0] >> p_t[1] >> p_t[2];
        VoxelTrajectory::VoxelGraph graph(&octomap, p_s, p_t);
        Eigen::MatrixXd res = graph.getPath(&octomap);

        VoxelTrajectory::TrajectoryGenerator traj_gen;

        pair<Eigen::MatrixXd,Eigen::MatrixXd> ans =
            traj_gen.genPolyCoeffTime(res);

	cout<<"data:"<<endl;
	cout<<res<<endl;
        cout<<"Poly coeffecients:"<<endl;
        cout<<ans.first<<endl;
        cout<<"Time allocation:"<<endl;
        cout<<ans.second<<endl;

    }
    if (mod == '7')
    {
        ifstream fin(argv[2]);
        double bdy[6];

        for (int i=0;i<6;i++)
            fin >> bdy[i];

        VoxelTrajectory::OctoMap octomap(bdy,5*5*5);

        int N;
        fin >> N;
        for (int i=0;i<N;i++)
        {
            double pt[3];
            for (int j=0;j<3;j++)
                fin >> pt[j];
            octomap.insert(pt);
        } 
        double p_s[3];
        double p_t[3];
        fin>>p_s[0] >> p_s[1] >> p_s[2];
        fin>>p_t[0] >> p_t[1] >> p_t[2];
        VoxelTrajectory::VoxelGraph graph(&octomap, p_s, p_t);

        Eigen::MatrixXd res = graph.getPath(&octomap);

        VoxelTrajectory::TrajectoryGenerator traj_gen;

        pair<Eigen::MatrixXd,Eigen::MatrixXd> ans =
            traj_gen.genPolyCoeffTime(res);

	cout<<"data:"<<endl;
	cout<<res<<endl;
        cout<<"Poly coeffecients:"<<endl;
        cout<<ans.first<<endl;
        cout<<"Time allocation:"<<endl;
        cout<<ans.second<<endl;

        ifstream fin_(argv[3]);
        cout<<"the in path file:\n"<<argv[3]<<endl;
        int M;
        fin_>>M;
        Eigen::MatrixXd data(M*2,6);
        for (int i=0;i<M*2;i++)
            for (int j=0;j<6;j++)fin_>>data(i,j);
        //data = res;
        cout<<"the data:\n"<<data<<endl;

        VoxelTrajectory::TrajectoryGenerator traj_gen_2;
        pair<Eigen::MatrixXd,Eigen::MatrixXd> ans_2 =
            traj_gen_2.genPolyCoeffTime(data);

        cout<<"Poly coeffecients:"<<endl;
        cout<<ans_2.first<<endl;
        cout<<"Time allocation:"<<endl;
        cout<<ans_2.second<<endl;
    }

    return 0;
}
