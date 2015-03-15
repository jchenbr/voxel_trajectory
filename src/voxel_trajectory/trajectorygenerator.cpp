
#include "voxel_trajectory/trajectorygenerator.h"
#include "voxel_trajectory/voxelmacro.h"

#include <cmath>
#include <utility>
#include <string.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <list>
#include <assert.h>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Sparse>

#define _USE_SPARSE_

#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>


#undef _USE_DEBUG_PRINT_

#ifdef _USE_MATLABQP_
#include "engine.h"
#endif

typedef Eigen::SparseMatrix<double> SMatrixXd;

const static int N = _TOT_BDY;
const static int R = _TOT_DIM;
const static double _EPS  = 1e-11;
const static int _N_LOOP   = 10;
const static int _DER_MIN  =   3;
const static bool _CHECK_EX   = true;
const static double _MARGIN_EX = 0.02;
static int M;

const static int _BUFF_SIZE = 256;
static char buffer[_BUFF_SIZE]="\0";

static double max_vel = 1.0, max_acc = 1.0;
    
namespace VoxelTrajectory
{

    using namespace Eigen;
    using namespace std;

    inline static VectorXd getCenter(const RowVectorXd &row)
    {
        return Vector3d(
            (row(_BDY_x) + row(_BDY_X)) * 0.5,
            (row(_BDY_y) + row(_BDY_Y)) * 0.5,
            (row(_BDY_z) + row(_BDY_Z)) * 0.5);
    }

    static double getTransTime(
        const VectorXd &p0,
        const VectorXd &p1,
        const VectorXd &vel0,
        const VectorXd &vel1)
    {
        VectorXd direct = p1 - p0;
        double distance = direct.norm();
        direct  /= distance;

        double Vel_s    = direct.dot(vel0);
        double Vel_t    = direct.dot(vel1);
        double aVel_s   = abs(Vel_s);
        double aVel_t   = abs(Vel_t);

        double t0, t1, t2, t3;

        t0 = t1 = t2 = t3 = 0.0;

        /* Case 1: Slow Down */
        if (aVel_s * aVel_s > 2.0 * max_acc * distance)
        {
            t0  = (Vel_s < 0) * (2.0 * aVel_s/ max_acc);
            t3  = (aVel_s / max_vel);
        } else /*Case 2: speed up and slow down.*/
        if (2.0 * max_vel * max_vel - aVel_s * aVel_s > 2.0 * max_acc * distance)
        {
            t0  = (Vel_s < 0) * (2.0 * aVel_s/ max_acc);
            t1  = (-aVel_s + sqrt( 0.5 * aVel_s * aVel_s + max_acc * distance )) / max_acc;
            t3  = t1 + aVel_s / max_acc;
        }else 
        {
            t0  = (Vel_s < 0) * (2.0 * aVel_s / max_acc);
            t1  = (max_vel - aVel_s) / max_acc;
            t3  = max_vel / max_acc;
            t2  = (distance - 0.5*(max_vel + aVel_s)*t1 - 0.5 * max_vel * t3)/max_vel;
        }

        return t0 + t1 + t2 + t3;
    }

    static VectorXd getTime_smart(
        const VectorXd &p_s,
        const VectorXd &p_t,
        const MatrixXd &E,
        const MatrixXd &vel)
    {
        VectorXd T = VectorXd::Zero(M);

        if (E.rows()  == 0)
        {
            T(0) = getTransTime( p_s, p_t, vel.col(0), VectorXd::Zero(_TOT_DIM));
            return T;
        }

        T(0)    = getTransTime(p_s, getCenter(E.row(0)), 
                                vel.col(0), VectorXd::Zero(_TOT_DIM));
        T(M-1)  = getTransTime(p_t, getCenter(E.row(M-2)), 
                                vel.col(1), VectorXd::Zero(_TOT_DIM));
        
        for (int i = 1; i < M-1; i++)
        {
            T(i) = getTransTime(getCenter(E.row(i-1)), getCenter(E.row(i)),
                            VectorXd::Zero(_TOT_DIM), VectorXd::Zero(_TOT_DIM));
        }

        return T;
    }

    static double cookTime(const double VEL, const double distance)
    {
        double t0, t1, t2, vel = abs(VEL), acc = max_acc;
        t0 = t1 = t2 = 0.0;

        t0  = (VEL < 0) * vel / acc * 2.0;
        
        if ( (max_vel - vel) * (max_vel - vel) < 2.0 * acc * distance)
        {
            t1  = sqrt(2 * distance / acc + vel * vel / (acc * acc) ) - vel / acc;
        }else 
        {
            t1  = (max_vel - vel) / acc;
            t2  = (distance - (max_vel + vel) * t1 * 0.5) / max_vel;
        }
        //clog<<"distance="<<distance<<",vel="<<vel<<" , acc="<<acc<<", t=["<<t0<<","<<t1<<","<<t2<<"]"<<endl;
        //clog<<"under sqrt:"<< (2 * distance / acc + vel * vel / (acc * acc) ) <<" ,"<< (vel / acc)<<endl;

        return t0 + t1 + t2;
    }

    static VectorXd getTime_stupid(
        const VectorXd &p_s,
        const VectorXd &p_t,
        const MatrixXd &E,
        const MatrixXd &vel)
    {
        VectorXd T = VectorXd(M);

        if (E.rows() == 0)
        {
            VectorXd direct = p_t - p_s;

            direct  /= direct.norm();

            double vel0 = direct.dot(vel.col(0));

            double to_add = (vel0 < 0) * abs(vel0) / max_acc;

            T(0)    = ( p_t  - p_s ).norm() / max_vel * 2.0 + to_add;
            return T;
        }
        // for the init segment
        {
            VectorXd direct = getCenter(E.row(0)) - p_s;
            direct  /= direct.norm();
            double vel0 = direct.dot(vel.col(0));
            double to_add = (vel0 < 0) * abs(vel0) / max_acc;

            //clog<<"vel_0 = " <<vel0  <<", to_add="<<to_add<<endl;

            T(0)    = (getCenter(E.row(0)) - p_s).norm() / (max_vel + abs(vel0)) * 2.0 + to_add;
        }
        // for the final segment
        {
            double distance = (p_t - getCenter(E.row(M-2)) ).norm();
            
            if (distance< (0.5 * max_vel * max_vel / max_acc) )
                T(M-1)  = distance/ max_vel * 2.0;
            else 
                T(M-1)  = max_vel/max_vel + (distance - (0.5 * max_vel * max_vel / max_acc))/max_vel ;
        }
        for (int i = 1; i < M-1; i++)
        {
            T(i)    = (getCenter(E.row(i)) - getCenter(E.row(i-1))).norm() / max_vel;
        }

        return T;
    }

    static void retInit(
        const MatrixXd  &PBE,
        VectorXd &p_s,
        VectorXd &p_t,
        MatrixXd    &B,
        MatrixXd    &E)
    {
        M   = PBE.rows()/2;

        p_s = PBE.block<1,_TOT_DIM>(0,0).transpose();
        p_t = PBE.block<1,_TOT_DIM>(0,3).transpose();

        B   = PBE.block(1,0,M,_TOT_BDY);

        E   = PBE.block(M+1,0,M-1,_TOT_BDY);
    }

    static void insertBlock(
        SMatrixXd &m, 
        int lr, int lc, 
        const SMatrixXd &n)
    {
        for (int k = 0; k < n.outerSize(); k++)
            for (SMatrixXd::InnerIterator it(n,k); it; ++it)
                m.insert(lr + it.row(), lc + it.col()) = it.value();
    }

    static pair<SMatrixXd, VectorXd> combineColsPr(
            const pair<SMatrixXd, VectorXd> &x, 
            const pair<SMatrixXd, VectorXd> &y)
    {
        size_t rows_x = x.first.rows(), rows_y = y.first.rows();
        size_t cols = x.first.cols();

        assert( y.first.cols() == cols );

        SMatrixXd m(rows_x + rows_y, cols);
        m.reserve(x.first.nonZeros() + y.first.nonZeros());

        VectorXd b = VectorXd::Zero( rows_x + rows_y);
        
        insertBlock(m, 0, 0, x.first);
        insertBlock(m, rows_x, 0, y.first);

        b << x.second , y.second;
        
        return make_pair(m,b);
    }

    static void printSM(SMatrixXd &x)
    {
        MatrixXd m(x.rows(), x.cols());
        for (int i=0; i<x.rows(); i++)
            for (int j=0; j<x.cols(); j++)
                m(i, j) = x.coeffRef(i, j);
        clog<<m<<endl;
    }

    static MatrixXd getDM(SMatrixXd &x)
    {
        MatrixXd m = MatrixXd::Zero(x.rows(), x.cols());

        for (int k =0; k<x.outerSize(); k++)
            for (SMatrixXd::InnerIterator it(x,k); it; ++it)
                m(it.row(), it.col()) = it.value();
        return m;
    }

    static void printPr(pair<SMatrixXd, VectorXd> &pr)
    {
        MatrixXd m(pr.first.rows(), pr.first.cols() +1);
        for (int i=0; i<pr.first.rows(); i++)
        {
            for (int j=0; j<pr.first.cols(); j++)
                m(i, j) = pr.first.coeffRef(i, j);
            m(i, pr.first.cols()) = pr.second(i);
        }
        clog<<m<<endl;
    }

    static pair<SMatrixXd, VectorXd> combineColsPr(
            const pair<SMatrixXd, VectorXd> &x, 
            const pair<SMatrixXd, VectorXd> &y,
            const pair<SMatrixXd, VectorXd> &z)
    {
        return combineColsPr(x, combineColsPr(y, z));
    }


    static SMatrixXd getHessianMatrix(const double t)
    {
        SMatrixXd H(N,N);
        H.reserve(N*N);

        double t_[N+N];

        t_[0] = 1.0;

        for (int i=1; i<N+N; i++)
            t_[i] = t_[i-1]*t;

        for (int i=_DER_MIN; i<N ;i++)
        {
            for (int j=_DER_MIN; j<N; j++)
            {
                double coeff = 1.0;
                for (int k=0;k<_DER_MIN;k++)
                    coeff   *= (i-k)*(j-k);

                int now  = (i-_DER_MIN) +(j-_DER_MIN)+1;
                coeff   /=now;
                
                H.insert(i,j)  = coeff*t_[now];
            }
        }
        return H;
    }

    static SMatrixXd getCostMatrix(
        const VectorXd &T)
    {
        SMatrixXd H (M*N,M*N);
        H.reserve(M*N*N);
        for (int i=0; i<M; i++)
            insertBlock(H, i*N, i*N, getHessianMatrix(T(i)));

        return H;
    }

    static SMatrixXd getMappingMatrix(
        const VectorXd &T)
    {
        SMatrixXd A(M*N, M*N);
        A.reserve(M*N*N);

        for (int i = 0; i<M; i++)
        {

            RowVectorXd coeff = RowVectorXd::Ones(N),t;

            MatrixXd P2D = MatrixXd::Zero(2*R, N);
            
            for (int j = 0; j<R; j++)
            {
                t   = RowVectorXd::Zero(N);
                t(j)= 1.0;
                for (int  k = j+1; k<N; k++)
                    t(k)    = t(k-1)*T(i);


                for (int k = j; k<=j; k++)
                {
                    //A(i*N + 0*R + j, i*N + k) = coeff(k)*t(k);
                    P2D(0*R + j, k) = coeff(k)*t(k);
                }


                for (int k = 0; k<N; k++)
                {
                    //A(i*N + 1*R + j, i*N + k) = coeff(k)*t(k);
                    P2D(1*R + j, k) = coeff(k)*t(k);
                }

                for (int k = 0; k<N; k++)
                    coeff(k)    *= k-j;
            }
            P2D = P2D.inverse();
            for (int r=0; r<N; r++)
                for (int c=0; c<N; c++)
                    if (abs(P2D(r,c)) >_EPS)
                    A.insert(i*N +r, i*N +c) = P2D(r,c);
        }
        return A;
    }

    static pair<SMatrixXd, VectorXd> getConstrainsEndpoints(
        const VectorXd &T,
        const double p_s,
        const double p_t,
        const RowVectorXd & vel,
        const RowVectorXd & acc)
    {
        //clog<<"CE-E-fine.-3"<<endl;
        //MatrixXd CE = MatrixXd::Zero(R+R,M*N + 1);
        SMatrixXd CE(R + R, M*N);
        CE.reserve(N*2);
        VectorXd b = VectorXd::Zero(R + R);

        b   << p_s, vel(0), acc(0),
               p_t, vel(1), acc(1);

        RowVectorXd coeff,t;
        
        coeff   = RowVectorXd::Ones(N);
        for (int j  = 0; j<R; j++)
        {
            t   = RowVectorXd::Zero(N);
            t(j)    = 1.0;

            for (int k = j+1; k<N; k++)
                t(k)    = t(k-1)*T(M-1);

            for (int k = j; k<=j; k++)
                CE.insert(0*R + j, 0*N + k) = coeff(k)*t(k);

            for (int k = 0; k<N; k++)
                CE.insert(1*R + j, (M-1)*N + k) = coeff(k)*t(k);

            for (int k = 0; k<N; k++)
                coeff(k)    *= k-j;
        }

        return make_pair(CE, b);
    }

    static pair< pair<SMatrixXd, VectorXd>, pair<SMatrixXd,VectorXd> > 
    getConstrainsCorridors(
        const VectorXd &T,
        const MatrixXd &E)
    {
        int N_CE = 0,N_CI = 0;
        for (int i = 0; i<M-1; i++)
        {
            int val = (E(i,1)-E(i,0))>_EPS;
            N_CI += val;
            N_CE += 1-val;
        }

        SMatrixXd CE (N_CE   ,M*N ), CI (N_CI*2 ,M*N );
        CE.reserve(N_CE*N);
        CI.reserve(N_CI*2*N);
        VectorXd CE_b = VectorXd::Zero(N_CE);
        VectorXd CI_b = VectorXd::Zero(N_CI*2);
        int i_ce = 0,i_ci = 0;

        for (int i = 0; i < M-1; i++)
        {
            if ((E(i,1)-E(i,0))>_EPS)
            {
                CI.insert(i_ci,(i+1)*N + 0) = -1.0;
                CI_b(i_ci)    = -E(i,0);
                i_ci    += 1;

                CI.insert(i_ci,(i+1)*N + 0) = 1.0;
                CI_b(i_ci)    = E(i,1);
                i_ci    += 1;
            }else
            {
                CE.insert(i_ce,(i+1)*N + 0)  = 1.0;
                CE_b(i_ce)  = E(i,0);
                i_ce    += 1;
            }
        }

        return make_pair( make_pair(CE, CE_b),
                          make_pair(CI, CI_b));
    }

    static pair<SMatrixXd, VectorXd> getConstrainsContinuity(
        const VectorXd &T)
    {
        SMatrixXd CE((M-1)*R, M*N);
        CE.reserve(M*N*2);
        VectorXd b = VectorXd::Zero((M-1)*R);

        for (int i = 0; i<M-1; i++)
        {
            RowVectorXd coeff   = RowVectorXd::Ones(N),t;

            double _c_next_seg  = 1.0;
            for (int j = 0; j<R; j++)
            {
                t   = RowVectorXd::Zero(N);
                t(j)= 1.0;
                for (int  k = j+1; k<N; k++)
                    t(k)    = t(k-1)*T(i);

                for (int k = 0; k<N; k++)
                    CE.insert(i*R + j,i*N + k) = coeff(k)*t(k);

                CE.insert(i*R + j, (i+1)*N + j)   = -_c_next_seg;
                _c_next_seg *=  j+1;

                for (int k = 0; k<N; k++)
                    coeff(k)    *= k-j;
            }
        }

        return make_pair(CE, b);
    }

    static inline VectorXd getPolyDerRoots(
        const VectorXd &P)
    {
        //clog<<"Der poly:\n"<<P<<endl;
        VectorXd p = P;
        //der
        for (int i = 0; i<N; i++) 
            p(i) = (i+1<N)? p(i+1)*(i+1) : 0.0;
        //clog<<"Der poly matrix 1:\n"<<p<<endl;

        int N_nz = N-1;
        while (N_nz>0 && abs(p(N_nz-1))<_EPS) 
            N_nz -=1;
        //clog<<"Der poly matrix 2: N_nz ="<<N_nz<<endl<<p.segment(0, N_nz).reverse()<<endl;
    
        if (N_nz < 2) return -VectorXd::Ones(4);
        RowVectorXd rp = p.segment(0, N_nz).reverse();


        //clog<<"Der poly matrix 3:\n"<< rp<<endl;

        MatrixXd tmp = MatrixXd::Zero(N_nz-1, N_nz-1);

        tmp.diagonal(-1) << VectorXd::Ones(N_nz-2);
        //clog<<"Der poly matrix:\n"<<tmp<<endl;
        //clog<<"Der poly to_add:\n"<<-p.segment(1,N_nz-1).transpose()/p(0)<<endl;
        tmp.row(0) << -rp.segment(1, N_nz-1)/rp(0);
        //clog<<"Der poly matrix 4:\n"<<tmp<<endl;
        
        VectorXcd eig   = tmp.eigenvalues();
        VectorXd rts    = -VectorXd::Ones(4);

        for (int i = 0; i<N_nz-1; i++)
        {
            //clog<<"eig:"<<eig(i).real()<<","<<eig(i).imag()<<endl;

            if (abs(eig(i).imag())<_EPS)
                rts(i)  = eig(i).real();
        }
        //clog<<"rts:\n"<<rts<<endl;
        //clog<< "coef:\n" << p <<"\nroots:\n" << eig<<"\n"<<rts<<"\n\n";
        return rts;
    }

    static MatrixXd getExtremums(
        const VectorXd &P)
    {
        MatrixXd ex = MatrixXd::Zero((N-2),M);
        for (int i=0;i<M;i++)
            ex.block<N-2,1>(0,i) = getPolyDerRoots(P.segment(i*N,N));

        return ex;
    }

    static bool checkExtremums(
        const VectorXd & T,
        const MatrixXd & B,
        const VectorXd & P,
        const MatrixXd & ex,
        pair<SMatrixXd, VectorXd> & CI)
    {
        bool ret    = true;
        VectorXd t = VectorXd::Zero(N);
        list<pair<int,double> > l,r;
        //clog<<"ex:\n"<<ex<<endl;


        for (int i = 0; i<M; i++)
            for (int j = 0; j<N-2; j++)
            {
                if (ex(j,i)<0 || ex(j,i)>T(i)) continue;

                t(0)    = 1.0;
                for (int k = 1; k<N; k++)
                    t(k)    = t(k-1)*ex(j,i);

                double now = P.segment(i*N,N).dot(t);
                if (now<B(i,0)||now>B(i,1))
                {
                    ret = false;
                    if (now<B(i,0))
                        l.push_back(pair<int,double>(i,ex(j,i)) );
                    else
                        r.push_back(pair<int,double>(i,ex(j,i)) );
                }
            }

        SMatrixXd to_add(l.size() + r.size(), M*N);
        to_add.reserve(N*to_add.rows());
        VectorXd b = VectorXd::Zero(l.size() + r.size());

        int i_ci = 0;
        RowVectorXd coeff = RowVectorXd::Zero(N);

        for (list<pair<int,double> >::iterator 
            it = l.begin(); it!=l.end(); it++)
        {
            coeff(0)    = 1.0;
            for (int k = 1; k<N; k++) 
                coeff(k)    = coeff(k-1)*it->second;

            //to_add.row(i_ci).segment(it->first*N,N)<<-coeff;
            for (int k = 0; k<N; k++)
                to_add.insert(i_ci, it->first*N + k) = -coeff(k);

            b(i_ci++)    = -(B(it->first, 0) + _MARGIN_EX); 
        }

        for (list<pair<int,double> >::iterator
            it = r.begin(); it!=r.end(); it++)
        {
            coeff(0)    = 1.0;
            for (int k = 1; k<N; k++)
                coeff(k)    = coeff(k-1)*it->second;
            
            //to_add.row(i_ci).segment(it->first*N,N)<<coeff;
            for (int k = 0; k<N; k++)
                to_add.insert(i_ci, it->first*N + k) = coeff(k);

            b(i_ci++)    = B(it->first, 1) - _MARGIN_EX;
        }

        CI = combineColsPr(CI, make_pair(to_add, b));
        return ret;
    }

    static MatrixXd getConstrainsExtremums(
        const VectorXd &T,
        const MatrixXd &B,
        const VectorXd &P,
        const MatrixXd &ex)
    {
        if (ex.cols()==0) return MatrixXd::Zero(0,M*N+1);
        int N_CI    = 0;

        for (int i = 0; i<M; i++)
            for (int j = 0; j<N-2; j++)
                N_CI    += !(ex(j,i)<0 || ex(j,i)>T(i));

        MatrixXd CI = MatrixXd::Zero(N_CI,M*N+2);
        int i_ci    = 0;

        VectorXd t = VectorXd::Zero(N);
        for (int i = 0; i<M; i++)
            for (int j = 0; j<N-2; j++)
            {
                if (ex(j,i)<0||ex(j,i)>T(i)) continue;

                t(0)    = 1.0;
                for (int k = 1; k<N; k++)
                    t(k)    = t(k-1)*ex(j,i);

                CI.block<1,N>(i_ci,i*N) = t;    
                CI(i_ci,M*N)    = B(i,0);
                CI(i_ci,M*N+1)  = B(i,1);
                i_ci    += 1;
            }

        return CI;
    }

#ifdef _USE_SPARSE_

static int _error_code = 0;

/* Some unknown bugs here. */
    static MatrixXd getPolyDer(
         SMatrixXd & Q,
         pair<SMatrixXd, VectorXd> & CE,
         pair<SMatrixXd, VectorXd> & CI)
    {
        // number of variable
        const int N_X  = M*N;

        // linear cost 
        double c[N_X] ;
        for (int i=0; i<N_X; i++) c[i] = 0.0;

        // upper bound 
        double xupp[N_X];
        char ixupp[N_X];
        for (int i=0; i<N_X; i++) xupp[i] = 0.0;
        memset(ixupp, 0, sizeof(ixupp));

        // lower bound
        double xlow[N_X];    
        char ixlow[N_X];
        for (int i=0; i<N_X; i++) xlow[i] = 0.0;
        memset(ixlow, 0, sizeof(ixlow));


        //clog<<"QP: Fine"<<endl;
        vector<pair<pair<int,int>, double> > tmp;
        // quadratic cost matrix
        int N_Q = 0;

        for (int k = 0; k<Q.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(Q,k); it; ++it)
            if (it.col() <= it.row()) //lower trianglur matrix
            {
                N_Q+= abs(it.value())>_EPS;
            }

        int iQ[N_Q], jQ[N_Q], i_q=0;
        double dQ[N_Q];

        tmp.resize(N_Q);
        for (int k = 0; k<Q.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(Q,k); it; ++it)
                if (it.col()<=it.row() && abs(it.value()) > _EPS)
                {
                    tmp[i_q++]=make_pair(make_pair(it.row(),it.col()),it.value());
                }
        sort(tmp.begin(), tmp.end());

        for (int i=0; i<tmp.size();i++)
        {
            iQ[i] = tmp[i].first.first;
            jQ[i] = tmp[i].first.second;
            dQ[i] = tmp[i].second;
        }


        // equality constrains
        int N_CE = 0, N_CE_ROW = CE.first.rows();
        for (int k = 0; k<CE.first.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(CE.first,k); it; ++it)
            {
                N_CE += abs(it.value()) > _EPS;
            }
        int iCE[N_CE], jCE[N_CE];
        double dCE[N_CE], b[N_CE_ROW];

        int i_ce =0;
        tmp.resize(N_CE);
        for (int k = 0; k<CE.first.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(CE.first,k); it; ++it)
            if (abs(it.value())>_EPS)
            {
                tmp[i_ce++]=make_pair(make_pair(it.row(),it.col()),it.value());
            }

        sort(tmp.begin(),tmp.end());

        for (int i=0; i<tmp.size();i++)
        {
            iCE[i] = tmp[i].first.first;
            jCE[i] = tmp[i].first.second;
            dCE[i] = tmp[i].second;
        }

        for (int i = 0; i < N_CE_ROW; i++)
            b[i]    = CE.second(i);


        // inequality constrains
        int N_CI = 0, N_CI_ROW = CI.first.rows();

        for (int k = 0; k<CI.first.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(CI.first,k); it; ++it)
            {
                N_CI += abs(it.value()) > _EPS;
            }

        int iCI[N_CI],jCI[N_CI];
        char ilb[N_CI_ROW],iub[N_CI_ROW];
        double dCI[N_CI],lb[N_CI_ROW],ub[N_CI_ROW];

        int i_ci =0;

        tmp.resize(N_CI);
        for (int k = 0; k<CI.first.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(CI.first,k); it; ++it)
            if (abs(it.value())>_EPS)
            {
                tmp[i_ci++]=make_pair(make_pair(it.row(),it.col()),it.value());
            }

        sort(tmp.begin(),tmp.end());

        for (int i=0; i<tmp.size();i++)
        {
            iCI[i] = tmp[i].first.first;
            jCI[i] = tmp[i].first.second;
            dCI[i] = tmp[i].second;
        }

        for (int i=0; i<N_CI_ROW; i++)
        {
            lb[i]   = 0;
            ub[i]   = CI.second(i);
            ilb[i]  = 0;
            iub[i]  = 1;
        }

#ifdef _USE_DEBUG_PRINT_1

        clog<<"N_Q="<<N_Q<<", N_CE="<<N_CE<<", N_CI="<<N_CI<<endl<<endl;

        MatrixXd tmp;
        clog<<"QP Q:"<<endl;
        tmp = MatrixXd::Zero(N_X, N_X);
        for (int i=0; i<N_Q; i++) tmp(iQ[i], jQ[i]) = dQ[i];
        for (int i=0; i<tmp.rows(); i++)
            for (int j=0; j<tmp.cols(); j++)
            if (abs(tmp(i,j))>_EPS) clog<<"("<<i<<","<<j<<")="<<tmp(i,j)<<endl;
        clog<<tmp<<endl<<endl;

        clog<<"QP CE:"<<endl;
        tmp = MatrixXd::Zero(N_CE_ROW, N_X);
        for (int i=0; i<N_CE; i++) tmp(iCE[i], jCE[i]) = dCE[i];
        for (int i=0; i<tmp.rows(); i++)
            for (int j=0; j<tmp.cols(); j++)
            if (abs(tmp(i,j))>_EPS) clog<<"("<<i<<","<<j<<")="<<tmp(i,j)<<endl;
        for (int i=0; i<N_CE_ROW; i++)
            clog<<b[i]<<endl;
        clog<<tmp<<endl<<endl;

        clog<<"QP CI:"<<endl;
        tmp = MatrixXd::Zero(N_CI_ROW, N_X);
        for (int i=0; i<N_CI; i++) tmp(iCI[i], jCI[i]) = dCI[i];
        for (int i=0; i<tmp.rows(); i++)
            for (int j=0; j<tmp.cols(); j++)
            if (abs(tmp(i,j))>_EPS) clog<<"("<<i<<","<<j<<")="<<tmp(i,j)<<endl;
        for (int i=0; i<N_CI_ROW; i++)
            clog<<lb[i]<<"("<<(short)ilb[i]<<"), "<<ub[i]<<"("<<(short)iub[i]<<")"<<endl;
        clog<<tmp<<endl<<endl;
#endif
        // qp solver
        QpGenSparseMa27 * qp  =
            new QpGenSparseMa27(N_X, N_CE_ROW, N_CI_ROW, N_Q, N_CE, N_CI);

        QpGenData   * prob  = (QpGenData *) qp->copyDataFromSparseTriple(
            c,      
            iQ,     N_Q,    jQ,     dQ,
            xlow,   ixlow,  xupp,   ixupp,  
            iCE,    N_CE,   jCE,    dCE,    b,
            iCI,    N_CI,   jCI,    dCI,    
            lb,     ilb,    ub,     iub);

        QpGenVars   * vars  = 
            (QpGenVars *) qp->makeVariables(prob);

        QpGenResiduals * resid  = 
            (QpGenResiduals *) qp->makeResiduals(prob);

        GondzioSolver   * s = new GondzioSolver(qp, prob);

        //clog<<"Just Fine log - 0"<<endl;

#ifdef _USE_DEBUG_PRINT_1
        s->monitorSelf();
#endif
        int ierr    = s->solve(prob,vars,resid);
        _error_code = ierr;
        //clog<<"Just Fine log - 1"<<endl;
        delete s;
        delete qp;

        if (ierr==0)
        {
            clog<<"Successfully found the solution."<<endl;
        }else{
            clog<<"Something Wrong with the QP. ERR_CODE="<<ierr<<endl;
        }

        //clog<<"Just Fine log - 2"<<endl;
        VectorXd D = VectorXd::Zero(M*N);
        double d[M*N];
        vars->x->copyIntoArray(d);
        //clog<<"Just Fine log - 3"<<endl;
        for (int i=0;i<M*N;i++)
            D(i)    = d[i];

        //clog<<"Just Fine log - 4"<<endl;
        return D;
    }
#endif

    static VectorXd getCoeff(
        const double p_s,
        const double p_t,
        const MatrixXd & B,
        const MatrixXd & E,
        const VectorXd & T,
        const RowVectorXd & vel,
        const RowVectorXd & acc)
    {
        //> Hessian Matrix 
        SMatrixXd H  = getCostMatrix(T);

        //clog << "0. H Fine." << endl;
        //> Mapping Matrix
        SMatrixXd IP2D    = getMappingMatrix(T);
        //clog << "1. IP2D." << endl;
        
        //IP2D   = MatrixXd::Identity(N*M,N*M);
       

        //> Endpoints
        pair<SMatrixXd, VectorXd> CE_0   = getConstrainsEndpoints(
            T, p_s, p_t, vel, acc);
        //clog << "2. Endpoints." << endl;
        // Corridors
        pair<pair<SMatrixXd, VectorXd>, pair<SMatrixXd, VectorXd> > CE_CI_1 = 
            getConstrainsCorridors(T,E);
        //clog << "3. Corriors." << endl;
        // Continuity
        pair<SMatrixXd, VectorXd> CE_2   = getConstrainsContinuity(T);
        //clog << "4. Continuity." << endl;
        // Extremums
        MatrixXd ex = MatrixXd(N-2,0);

        VectorXd P,D;

        SMatrixXd Q;
        Q.reserve(M*N*N);
        pair<SMatrixXd, VectorXd> CE, CI;

        // the cost matrix
        Q   = IP2D.transpose() * H * IP2D;
        //clog << "5. Q." << endl;

        // the eqality constrains
        CE  = combineColsPr(CE_0, CE_CI_1.first, CE_2);
        CE.first = CE.first * IP2D;
        //clog << "6. CE."<<endl;

        pair<SMatrixXd, VectorXd> CI_3 = make_pair(
            SMatrixXd(0, M*N), VectorXd::Zero(0));

        for (int loop_v=0 ; loop_v<_N_LOOP; loop_v++)
        {

            // the inequality constrains

            CI  = combineColsPr(CE_CI_1.second , CI_3);
            CI.first  = CI.first * IP2D;

            //clog << "7. CI." << endl;

#ifdef _USE_DEBUG_PRINT_
            clog<< "h:" <<endl; printSM(H);
            clog<< "Q:" <<endl; printSM(Q);
            clog<< "IP2D:" <<endl; printSM(IP2D);
            clog<< "endpoints_CE:" <<endl; printPr(CE_0);
            clog<< "Corridors_CE:" <<endl; printPr(CE_CI_1.first);
            clog<< "Corridors_CI:" <<endl; printPr(CE_CI_1.second);
            clog<< "Continuity_CE:" <<endl; printPr(CE_2);
            clog<< "Extremums_CI" <<endl; printPr(CI_3);
            clog<< "CE:"<<endl; printPr(CE);
            clog<< "CI:"<<endl; printPr(CI);
#endif

#ifdef _USE_SPARSE_
            D   = getPolyDer(Q, CE, CI);
#endif
            //clog << "8.1 derivatives." << endl;
            P   = IP2D * D;
            //clog<<"P:\n"<<P<<endl;
            //clog<<"D:\n"<<D<<endl;
            
            ex  = getExtremums(P);

            //clog << "8. ex." << endl;
            if ((!_CHECK_EX)||checkExtremums(T,B,P,ex,CI_3))
                break;
            //clog << "9. check." << endl;
        }

        return P;
    }

    static MatrixXd getTrajCoeff(
        const VectorXd &p_s,
        const VectorXd &p_t,
        const MatrixXd &B,
        const MatrixXd &E,
        const VectorXd &T,
        const MatrixXd &vel,
        const MatrixXd &acc)
    {
        MatrixXd P = MatrixXd::Zero(M*N, _TOT_DIM);

        int _error_sum = 0;
        for (int dim = 0; dim< _TOT_DIM; dim++)
        {
            P.block(0,dim,N*M,1) = getCoeff(
                            p_s(dim),
                            p_t(dim),
                            B.block(0,dim<<1,M,2),
                            E.block(0,dim<<1,M-1,2),
                            T,
                            vel.row(dim),
                            acc.row(dim));
            _error_sum += _error_code;
            clog << "[ERROR] code = " << _error_code <<endl;
        }
        _error_code = _error_sum;

        return P;
    }

    pair<MatrixXd,VectorXd> TrajectoryGenerator::genPolyCoeffTime(
        const MatrixXd &PBE,
        const MatrixXd &vel,
        const MatrixXd &acc,
        const double maxVel,
        const double maxAcc) 
    {
        VectorXd p_s,p_t;
        MatrixXd B,E,P;
        VectorXd T;
        // init()
        max_vel = maxVel;
        max_acc = maxAcc;
        retInit(PBE, p_s, p_t, B, E);

        // allocate time for each segment
        T   = getTime_stupid(p_s, p_t, E, vel);
        T(0) /=1.0;

        // generate the coeff for polynomial traj
        P   = getTrajCoeff(p_s, p_t, B, E, T, vel, acc); 
#ifdef _DEBUG_SCREEN_PRINT_
        clog<<"[ !data! ]\n" << PBE <<endl;
        clog<<"[ !vel! ]\n" << vel <<endl;
        clog<<"[ !acc! ]\n" << acc <<endl;
        clog<<"[ !max_vel! ]\n" << maxVel <<endl;
        clog<<"[ !max_acc! ]\n" << maxAcc <<endl;
        clog<<"[ !T! ]\n" << T <<endl;
        clog<<"[ !P! ]\n" << P <<endl;
#endif

        if (_error_code > 0) T(0) = -1;
        return pair<MatrixXd,VectorXd>(P,T);
    }
}
