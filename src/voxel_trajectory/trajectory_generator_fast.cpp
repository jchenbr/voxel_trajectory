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

#define _TRAJECTORY_GENERATOR_FLAG_USE_SPARSE_
//#define _TRAJECTORY_GENERATOR_FLAG_NO_INFLATION__

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
const static double _EPS  = 1e-7;
const static int _N_LOOP   = 10;
const static int _DER_MIN  =   3;
const static bool _CHECK_EX   = true;
const static double _MARGIN_EX = 0.02;
const static double _SAFE_RATE  = 0.05;
const static double _PLAN_RATE  = 1.0;
const static double _LIM_RATE   = 1.0;
const static double _BEG_FIN_RELAX_RATE = 1.5;
static int M;

const static int _BUFF_SIZE = 256;
static char buffer[_BUFF_SIZE] = "\0";

static double max_vel = 1.0, max_acc = 1.0;
static double f_vel = 1.0, f_acc = 1.0;
vector<double> _qp_cost;
    
namespace VoxelTrajectory
{

    using namespace Eigen;
    using namespace std;

    inline static double chooseKInN(int n, int k)
    {
        double a = 1.0, b = 1.0;
        for (int i = 0; i < k; ++i)
            a *= k - i, b *= n - i;  
        return b / a;
    }

    inline static double pow(double a, int n)
    {
        double ret = 1.0;
        while (n > 0)
        {
            if (n & 1) ret *= a;
            a *= a;
            n >>= 1;
        }
        return ret;
    }


    inline static int genMappingMatrices(
            const VectorXd &T,
            const vector<bool> &is_fixed, 
            const int M,
            const int N,
            const int R,
            SMatrixXd &D2D,
            SMatrixXd &RDF,
            SMatrixXd &RDD)
    {

        { // mapping the equalled derivatives into the same one
            D2D.resize( (M + 1) * R, M * N);
            D2D.reserve( (M + 1) * R);
            int i = 0;
            for (int j = 0; j < M + M; ++j)
            {
                i = i + (j & 1);
                for (int k = 0; k < R; ++k)
                {
                    D2D.insert(i * R + k, j * R + k) = 1;
                }
            }
        }

        { // mapping which divide the derivatives into fixed ones and dynamic ones
            int n_f = 0;
            for (auto item: is_fixed) n_f += item;
            int n_d = (M + 1) * R - n_f;
            RDF.resize(n_f, (M + 1) * R);
            RDF.reserve(n_f);
            RDD.resize(n_d, (M + 1) * R);
            RDD.reserve(n_d);
            int i_f = 0, i_d = 0;
            for (int j = 0; j < M + 1; ++j)
            {
                for (int k = 0; k < R; ++k)
                {
                    int idx = j * R + k;
                    //clog << "idx = " << idx << ", j = " << j << ", k = " << k << endl;
                    if (is_fixed[idx])
                        RDF.insert(i_f++, idx) = 1.0;
                    else
                        RDD.insert(i_d++, idx) = 1.0;
                }
            }
        }

        return 0;
    }

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
        //clog << "p_0:\n" << p0 << endl;
        //clog << "p_1:\n" << p1 << endl;
        VectorXd direct = p1 - p0;
        double distance = direct.norm();
        direct  /= distance;

        double Vel_s    = direct.dot(vel0);
        double Vel_t    = direct.dot(vel1);
        double aVel_s   = abs(Vel_s);
        double aVel_t   = abs(Vel_t);
        double maxVel   = f_vel * _PLAN_RATE;
        double maxAcc   = f_acc * _PLAN_RATE;

        double t0, t1, t2, t3;

        t0 = t1 = t2 = t3 = 0.0;

        /* Case 1: Slow Down */
        if (aVel_s * aVel_s > 2.0 * maxAcc * distance)
        {
            t0  = (Vel_s < 0) * (2.0 * aVel_s/ maxAcc);
            t3  = (aVel_s / maxVel);
        } else /*Case 2: speed up and slow down.*/
        if (2.0 * maxVel * maxVel - aVel_s * aVel_s > 2.0 * maxAcc * distance)
        {
            t0  = (Vel_s < 0) * (2.0 * aVel_s/ maxAcc);
            t1  = (-aVel_s + sqrt( 0.5 * aVel_s * aVel_s + maxAcc * distance )) / maxAcc;
            t3  = t1 + aVel_s / maxAcc;
        }else 
        {
            t0  = (Vel_s < 0) * (2.0 * aVel_s / maxAcc);
            t1  = (maxVel - aVel_s) / maxAcc;
            t3  = maxVel / maxAcc;
            t2  = (distance - 0.5 * (maxVel + aVel_s) * t1 - 0.5 * maxVel * t3) / maxVel;
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
            T(0) = getTransTime(p_s, p_t, vel.col(0), VectorXd::Zero(_TOT_DIM)) * _BEG_FIN_RELAX_RATE;
            return T;
        }

        T(0)    = getTransTime(p_s, getCenter(E.row(0)), 
                                vel.col(0), VectorXd::Zero(_TOT_DIM)) * _BEG_FIN_RELAX_RATE;
        T(M-1)  = getTransTime(p_t, getCenter(E.row(M-2)), 
                                vel.col(1), VectorXd::Zero(_TOT_DIM)) * _BEG_FIN_RELAX_RATE;
        
        for (int i = 1; i < M - 1; i++)
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

        double maxAcc   = max_acc * _PLAN_RATE;
        double maxVel   = max_vel * _PLAN_RATE;

        if (E.rows() == 0)
        {
            VectorXd direct = p_t - p_s;

            direct  /= direct.norm();

            double vel0 = direct.dot(vel.col(0));

            double to_add = (vel0 < 0) * abs(vel0) / maxAcc;

            T(0)    = ( p_t  - p_s ).norm() / maxVel * 2.0 + to_add;
            return T;
        }
        // for the init segment
        {
            VectorXd direct = getCenter(E.row(0)) - p_s;
            direct  /= direct.norm();
            double vel0 = direct.dot(vel.col(0));
            double to_add = (vel0 < 0) * abs(vel0) / maxAcc * 2.0;

            //clog<<"vel_0 = " <<vel0  <<", to_add="<<to_add<<endl;

            T(0)    = (getCenter(E.row(0)) - p_s).norm() / (maxVel + abs(vel0)*0.5) * 2.0 + to_add;
        }
        // for the final segment
        {
            double distance = (p_t - getCenter(E.row(M-2)) ).norm();
            
            //if (distance< (0.5 * maxVel * maxVel / maxAcc) )
                T(M-1)  = distance/ maxVel * 2.0;
            //else 
            //    T(M-1)  = maxVel/maxVel + (distance - (0.5 * maxVel * maxVel / maxAcc))/maxVel ;
        }
        for (int i = 1; i < M-1; i++)
        {
            T(i)    = (getCenter(E.row(i)) - getCenter(E.row(i-1))).norm() / maxVel;
        }

        return T;
    }

    static void retInit(
        const MatrixXd  &PBE,
        const MatrixXd  &inflated_path, 
        VectorXd &p_s,
        VectorXd &p_t,
        MatrixXd    &B,
        MatrixXd    &B_,
        MatrixXd    &E)
    {
        M   = PBE.rows() >> 1;

        p_s = PBE.block<1, _TOT_DIM>(0, 0).transpose();
        p_t = PBE.block<1, _TOT_DIM>(0, 3).transpose();

        B   = PBE.block(1, 0, M, _TOT_BDY);
        B_  = inflated_path;

        E   = PBE.block(M + 1, 0, M - 1, _TOT_BDY);
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

    static pair<SMatrixXd, VectorXd> combineRowsPr(
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

    static SMatrixXd combineRowsSM(const SMatrixXd &a, const SMatrixXd &b)
    {
        size_t cols = a.cols(), rows_a = a.rows(), rows_b = b.rows();
        assert(b.cols() == cols);

        SMatrixXd m(rows_a + rows_b, cols);
        m.reserve(a.nonZeros() + b.nonZeros());
        insertBlock(m, 0, 0, a);
        insertBlock(m, rows_a, 0, b);

        return m;
    }

    static SMatrixXd combineRowsSM(const SMatrixXd &a, const SMatrixXd &b, const SMatrixXd &c)
    {
        return combineRowsSM(combineRowsSM(a, b), c);
    }

    static void printSM(SMatrixXd &x)
    {
        MatrixXd m(x.rows(), x.cols());
        for (int i = 0; i < x.rows(); i++)
            for (int j = 0; j < x.cols(); j++)
                m(i, j) = x.coeffRef(i, j);
        clog << m <<endl;
    }

    static MatrixXd getDM(SMatrixXd &x)
    {
        MatrixXd m = MatrixXd::Zero(x.rows(), x.cols());

        for (int k = 0; k < x.outerSize(); k++)
            for (SMatrixXd::InnerIterator it(x,k); it; ++it)
                m(it.row(), it.col()) = it.value();
        return m;
    }

    static void printPr(pair<SMatrixXd, VectorXd> &pr)
    {
        MatrixXd m(pr.first.rows(), pr.first.cols() +1);
        for (int i = 0; i < pr.first.rows(); i++)
        {
            for (int j = 0; j < pr.first.cols(); j++)
                m(i, j) = pr.first.coeffRef(i, j);
            m(i, pr.first.cols()) = pr.second(i);
        }
        clog<<m<<endl;
    }

    static pair<SMatrixXd, VectorXd> combineRowsPr(
            const pair<SMatrixXd, VectorXd> &x, 
            const pair<SMatrixXd, VectorXd> &y,
            const pair<SMatrixXd, VectorXd> &z)
    {
        return combineRowsPr(x, combineRowsPr(y, z));
    }


    static SMatrixXd getHessianMatrix(const double t)
    {
        SMatrixXd H(N,N);
        H.reserve(N * N);

        double t_[N + N];

        t_[0] = 1.0;

        for (int i = 1; i < N + N; i++)
            t_[i] = t_[i - 1] * t;

        for (int i = _DER_MIN; i < N ;i++)
        {
            for (int j = _DER_MIN; j < N; j++)
            {
                int now  = (i - _DER_MIN) + (j - _DER_MIN) + 1;
                double coeff = pow(t, now) / now;

                for (int k = 0; k < _DER_MIN; k++)
                    coeff   *= (i - k) * ( j - k);
                
                H.insert(i, j)  = coeff;
            }
        }
        return H;
    }

    static SMatrixXd getCostMatrix(
        const VectorXd &T)
    {
        SMatrixXd H (M * N, M * N);
        H.reserve(M * N * N);
        for (int i = 0; i < M; i++)
            insertBlock(H, i * N, i * N, getHessianMatrix(T(i)));

        return H;
    }

    static SMatrixXd getMappingMatrix(
        const VectorXd &T)
    {
        SMatrixXd A(M * N, M * N);
        A.reserve(M * N * N);

        for (int i = 0; i < M; i++)
        {

            RowVectorXd coeff = RowVectorXd::Ones(N),t;

            MatrixXd P2D = MatrixXd::Zero(2 * R, N);
            
            for (int j = 0; j < R; j++)
            {
                t = RowVectorXd::Zero(N);
                t(j) = 1.0;
                for (int  k = j + 1; k < N; k++)
                    t(k)    = t(k - 1) * T(i);


                for (int k = j; k <= j; k++)
                {
                    //A(i*N + 0*R + j, i*N + k) = coeff(k)*t(k);
                    P2D(0 * R + j, k) = coeff(k) * t(k);
                }


                for (int k = 0; k < N; k++)
                {
                    //A(i*N + 1*R + j, i*N + k) = coeff(k)*t(k);
                    P2D(1 * R + j, k) = coeff(k) * t(k);
                }

                for (int k = 0; k < N; k++)
                    coeff(k)    *= k-j;
            }
            P2D = P2D.inverse();
            for (int r = 0; r < N; r++)
                for (int c = 0; c < N; c++)
                    A.insert(i * N + r, i * N + c) = P2D(r,c);
        }
        return A;
    }

    static pair<SMatrixXd, VectorXd> getConstrainsEndpoints_D(
        const VectorXd &T,
        const MatrixXd &B,
        const MatrixXd &E,
        const double p_s,
        const double p_t,
        const RowVectorXd & vel,
        const RowVectorXd & acc)
    {
        //clog<<"CE-E-fine.-3"<<endl;
        //MatrixXd CE = MatrixXd::Zero(R+R,M*N + 1);
        SMatrixXd CE(R + R, (M + 1) * R);
        CE.reserve(R + R);
        VectorXd b = VectorXd::Zero(R + R);

        b   << p_s, vel(0), acc(0),
               p_t, vel(1), acc(1);

        for (int j  = 0; j < R; j++)
        {
            CE.insert(j, 0 * R + j) = 1.0;
            CE.insert(R + j, M * R + j) = 1.0;
        }

        return make_pair(CE, b);
    }

    static pair<SMatrixXd, VectorXd> getConstrainsEndpoints(
        const VectorXd &T,
        const MatrixXd &B,
        const MatrixXd &E,
        const double p_s,
        const double p_t,
        const RowVectorXd & vel,
        const RowVectorXd & acc)
    {
        //clog<<"CE-E-fine.-3"<<endl;
        //MatrixXd CE = MatrixXd::Zero(R+R,M*N + 1);
        SMatrixXd CE(R + R, M * N);
        CE.reserve(N * 2);
        VectorXd b = VectorXd::Zero(R + R);

        b   << p_s, vel(0), acc(0),
               p_t, vel(1), acc(1);

        RowVectorXd coeff,t;
        
        coeff   = RowVectorXd::Ones(N);
        for (int j  = 0; j < R; j++)
        {
            t   = RowVectorXd::Zero(N);
            t(j)    = 1.0;

            for (int k = j + 1; k < N; k++)
                t(k) = t(k - 1) * T(M - 1);

            for (int k = j; k <= j; k++)
                CE.insert(0 * R + j, 0 * N + k) = coeff(k) * t(k);

            for (int k = 0; k < N; k++)
                CE.insert(1 * R + j, (M - 1) * N + k) = coeff(k) * t(k);

            for (int k = 0; k < N; k++)
                coeff(k) *= k - j;
        }

        int c_wp = 0, i_wp = 0;
        for (int i = 0; i + 1 < M; ++i)
        {
            c_wp += E(i, 0) + _EPS > E(i, 1) && (B.row(i) - B.row(i + 1)).norm() < _EPS;
        }

        //clog << "! ! c_wp = " << c_wp << endl;
        SMatrixXd CE_wp(c_wp, M * N);
        CE_wp.reserve(N * c_wp);
        VectorXd b_wp(c_wp);

        for (int i = 0; i + 1 < M; ++i)
        {
            if (E(i, 0) + _EPS > E(i, 1) && (B.row(i) - B.row(i + 1)).norm() < _EPS)
            {
                //clog << "Edge : \n" << E.row(i) << endl;
                //clog << "Node : \n" << B.row(i) << endl << B.row(i + 1) << endl; 
                
                t(0) = 1.0;
                for (int k = 1; k < N; ++k) 
                    t(k) = t(k - 1) * T(i);

                for (int k = 0; k < N; ++k)
                    CE_wp.insert(i_wp, i * N + k) = t(k);

                b_wp(i_wp++) = E(i, 0);
            }
        }

        return combineRowsPr(make_pair(CE, b), make_pair(CE_wp, b_wp));
    }

    static pair< pair<SMatrixXd, VectorXd>, pair<SMatrixXd,VectorXd> > 
    getConstrainsCorridors(
        const VectorXd &T,
        const MatrixXd &B,
        const MatrixXd &E)
    {
        int last = 0;
        list<pair<int, double > > vecCE;
        list<pair<int, pair<double, double> > > vecCI;

        for (int i = 0; i < M - 1; i++)
        {
            if ( (B(i, 1) - B(i, 0)) + _EPS < (B(i + 1, 1) - B(i + 1, 0)) )
            {
                last = (last == -1) ? 0 : 1;
            }else
            {
                last = -1;
            }
            //last = 0;

            if ((E(i, 1) - E(i, 0)) > _EPS)
            {
                vecCI.push_back(make_pair(i, 
                        make_pair(E(i, 0), E(i, 1))));
            }else
            {
                if (last == 0)
                {
                    vecCE.push_back(make_pair(i,E(i,0)));
                }else
                {
                    int j = (last == 1) ? i + 1 : i;
                    double l = min(B(j, 0), E(i, 0));
                    double r = max(B(j, 1), E(i, 1));
                    vecCI.push_back(make_pair(i, 
                                make_pair(l, r)));
                }
            }
        }

        SMatrixXd CE (vecCE.size() , M * N );
        SMatrixXd CI (vecCI.size() * 2 , M * N );

        CE.reserve(vecCE.size() * N);
        CI.reserve(vecCI.size() * 2 * N);

        VectorXd CE_b = VectorXd::Zero(vecCE.size());
        VectorXd CI_b = VectorXd::Zero(vecCI.size() * 2);

        int i_ce = 0,i_ci = 0;

        for (auto & pr: vecCE)
        {
            int i = pr.first;
            CE.insert(i_ce, (i + 1) * N + 0) = 1.0;
            CE_b(i_ce) = pr.second;
            i_ce += 1;
        }

        for (auto & pr: vecCI)
        {
            int i = pr.first;
            CI.insert(i_ci, (i + 1) * N + 0) = -1.0;
            CI_b(i_ci) = -pr.second.first;
            i_ci += 1;

            CI.insert(i_ci, (i + 1) * N + 0) = 1.0;
            CI_b(i_ci) = pr.second.second;
            i_ci += 1;        
        }
#if 0
        for (int i = 0; i < M-1; i++)
        {
            if ((E(i, 1) - E(i, 0)) > _EPS)
            {
                CI.insert(i_ci, (i + 1) * N + 0) = -1.0;
                CI_b(i_ci)    = -E(i,0);
                i_ci    += 1;

                CI.insert(i_ci, (i + 1) * N + 0) = 1.0;
                CI_b(i_ci)    = E(i,1);
                i_ci    += 1;
            }else
            {
                CI.insert(i_ci, (i + 1) * N + 0) = -1.0;
                CI_b(i_ci)  = -min(B(i, 0), B(i + 1, 0));
                i_ci    += 1;

                CI.insert(i_ci, (i + 1) * N + 0) = 1.0;
                CI_b(i_ci)  = max(B(i, 1), B(i + 1, 1));
                i_ci    += 1;
                //clog << "i = " << i << ", " << B.row(i) << ", " << B.row(i+1)<<endl;
            }
        }
#endif

        //clog << "Corridors_CI: \n" << CI << endl;
        //clog << "Corridors_CI_b: \n" << CI_b << endl;
        //clog << "Corridors_CE: \n" << CE << endl;

        return make_pair( make_pair(CE, CE_b),
                          make_pair(CI, CI_b));
    }

    static pair< pair<SMatrixXd, VectorXd>, pair<SMatrixXd,VectorXd> > 
    getConstrainsOverlappedCorridors_D(
        const VectorXd &T,
        const MatrixXd &B,
        const MatrixXd &E)
    {
        //int last = 0;
        vector<pair<int, double > > vecCE;
        vector<pair<int, pair<double, double> > > vecCI;
        vecCE.reserve(M);
        vecCI.reserve(M);

        for (int i = 0; i < M - 1; i++)
        {
            double l = max(B(i, 0), B(i + 1, 0));
            double r = min(B(i, 1), B(i + 1, 1));
            if (abs(r - l) < _EPS)
                vecCE.push_back(make_pair(i, l));
            else
                vecCI.push_back(make_pair(i, make_pair(l, r)));
        }


        SMatrixXd CE (vecCE.size() , (M + 1) * R);
        SMatrixXd CI (vecCI.size() * 2 , (M + 1) * R);

        CE.reserve(vecCE.size());
        CI.reserve(vecCI.size() * 2);

        VectorXd CE_b = VectorXd::Zero(vecCE.size());
        VectorXd CI_b = VectorXd::Zero(vecCI.size() * 2);

        int i_ce = 0, i_ci = 0;

        for (auto & pr: vecCE)
        {
            CE.insert(i_ce, (pr.first + 1) * R + 0) = 1.0;
            CE_b(i_ce++) = pr.second;
        }

        for (auto & pr: vecCI)
        {
            CI.insert(i_ci, (pr.first + 1) * R + 0) = -1.0;
            CI_b(i_ci++) = -pr.second.first;

            CI.insert(i_ci, (pr.first + 1) * R + 0) = 1.0;
            CI_b(i_ci++) = pr.second.second;
        }

        return make_pair( make_pair(CE, CE_b),
                          make_pair(CI, CI_b));
    }


    static pair<SMatrixXd, VectorXd> getConstrainsContinuity(
        const VectorXd &T)
    {
        SMatrixXd CE((M - 1) * R, M * N);
        CE.reserve(M * N * 2);
        VectorXd b = VectorXd::Zero((M - 1) * R);

        for (int i = 0; i < M - 1; i++)
        {
            RowVectorXd coeff   = RowVectorXd::Ones(N),t;

            double _c_next_seg  = 1.0;
            for (int j = 0; j < R; j++)
            {
                t   = RowVectorXd::Zero(N);
                t(j) = 1.0;
                for (int  k = j + 1; k < N; k++)
                    t(k)    = t(k - 1) * T(i);

                for (int k = 0; k<N; k++)
                    CE.insert(i * R + j,i * N + k) = coeff(k) * t(k);

                CE.insert(i * R + j, (i + 1) * N + j)   = -_c_next_seg;
                _c_next_seg *=  j+1;

                for (int k = 0; k < N; k++)
                    coeff(k)    *= k-j;
            }
        }

        return make_pair(CE, b);
    }

    static inline VectorXd getPolyDerRoots(
        const VectorXd &P,
        const int degree)
    {
        VectorXd p = P;

        // get the derivatives
        for (int k = 0; k < degree; ++k)
        {
            for (int i = 0; i < N; ++i) 
                p(i) = (i + 1 < N) ? p(i + 1) * (i + 1) : 0.0;
        }

        // get the number of non-zero element
        int N_nz = N - 1;

        while (N_nz > 0 && abs(p(N_nz - 1)) < _EPS) N_nz -=1;
    
        if (N_nz < 2) return -VectorXd::Ones(N - degree - 1);

        // get the poly coeff
        RowVectorXd rp = p.segment(0, N_nz).reverse();

        // to solve the eigenvalue to get the roots
        MatrixXd tmp = MatrixXd::Zero(N_nz - 1, N_nz - 1);

        tmp.diagonal(-1) << VectorXd::Ones(N_nz - 2);

        tmp.row(0) << -rp.segment(1, N_nz - 1) / rp(0);
        
        // the complex roots
        VectorXcd eig   = tmp.eigenvalues();

        // to return the real roots, -1 is non-sense.
        VectorXd rts    = -VectorXd::Ones(N - degree - 1);

        for (int i = 0; i < N_nz - 1; i++)
        {
            // check if it's real
            if (abs(eig(i).imag()) < _EPS)
                rts(i)  = eig(i).real();
        }

        return rts;
    }

    static MatrixXd getExtremums(
        const VectorXd &P, 
        const int degree)
    {
        MatrixXd ex = MatrixXd::Zero((N - degree -1), M);

        for (int i = 0; i < M; ++i)
            ex.block(0, i, N - degree - 1, 1) = 
                getPolyDerRoots( P.segment(i * N, N), degree);

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


        for (int i = 0; i < M; i++)
            for (int j = 0; j < N - 2; j++)
            {
                if (ex(j, i) < 0 || ex(j, i) > T(i)) continue;

                t(0)    = 1.0;
                for (int k = 1; k< N; k++)
                    t(k)    = t(k - 1) * ex(j, i);

                double now = P.segment(i * N, N).dot(t);
                if (now < B(i, 0) || now > B(i, 1))
                {
                    ret = false;
                    if (now < B(i, 0))
                        l.push_back(pair<int, double>(i, ex(j, i)));
                    else
                        r.push_back(pair<int, double>(i, ex(j, i)));
                }
            }

        SMatrixXd to_add(l.size() + r.size(), M * N);
        to_add.reserve(N * to_add.rows());
        VectorXd b = VectorXd::Zero(l.size() + r.size());

        int i_ci = 0;
        RowVectorXd coeff = RowVectorXd::Zero(N);

        for (list<pair<int, double> >::iterator 
            it = l.begin(); it != l.end(); it++)
        {
            coeff(0)    = 1.0;
            for (int k = 1; k < N; k++) 
                coeff(k)    = coeff(k - 1) * it->second;

            //to_add.row(i_ci).segment(it->first*N,N)<<-coeff;
            for (int k = 0; k < N; k++)
                to_add.insert(i_ci, it->first * N + k) = -coeff(k);

            b(i_ci++)    = -(B(it->first, 0) + _MARGIN_EX); 
        }

        for (list<pair<int,double> >::iterator
            it = r.begin(); it != r.end(); it++)
        {
            coeff(0)    = 1.0;
            for (int k = 1; k < N; k++)
                coeff(k)    = coeff(k - 1) * it->second;
            
            //to_add.row(i_ci).segment(it->first*N,N)<<coeff;
            for (int k = 0; k < N; k++)
                to_add.insert(i_ci, it->first * N + k) = coeff(k);

            b(i_ci++)    = B(it->first, 1) - _MARGIN_EX;
        }

        CI = combineRowsPr(CI, make_pair(to_add, b));
        return ret;
    }

    static bool checkDerExtremums(
        const VectorXd &T,
        const vector<double> & Lim,
        const VectorXd &P,
        const vector<MatrixXd> & Ex,
        pair<SMatrixXd, VectorXd> & CI,
        VectorXd & coeff_t)
    {
        //clog << "Velocity Extremums:\n" << Ex[0] << endl;
        //clog << "Acceleration Extremums:\n" << Ex[1] << endl;

        bool ret    = true;
        VectorXd t  = VectorXd::Zero(N);
        coeff_t << 0.0 , 0.0;
        
        for (int dgr = 2; dgr < 2 + Ex.size(); ++dgr)
        {
            list< pair<int, double> > l, r;

            for (int idx = 0; idx < M; ++idx)
            {
                VectorXd ex = VectorXd::Zero(N - dgr + 1);
                ex(0)   = 0.0;
                ex(1)   = T(idx);
                ex.segment(2, N - dgr - 1) = Ex[dgr - 2].col(idx);

                for (int j = 0; j < N - dgr + 1; j++)
                {
                    if (ex(j) < 0 || ex(j) > T(idx)) 
                        continue;

                    t(0)    = 1.0;
                    for (int k = 1; k < N; k++)
                        t(k)    = t(k - 1) * ex(j);

                    VectorXd p  = P.segment(idx * N, N);
                    for (int i = 1; i < dgr; i++)
                    {
                        for (int k = 0; k < N; k++)
                            p(k)    = (k + 1 < N) ? p(k + 1) * (k + 1) : 0.0;
                    }

                    double now = p.dot(t);
                    //clog << "Here, we check " << now << ", " << Lim[dgr - 2] << endl;
                    coeff_t(dgr - 2) = max(coeff_t(dgr - 2), now / Lim[dgr - 2]);
                    coeff_t(dgr - 2) = max(coeff_t(dgr - 2), now / -Lim[dgr - 2]);

                    if (now < -Lim[dgr - 2])
                    {
                        ret = false;
                        l.push_back(make_pair(idx, ex(j)));
                    }
                    if (now > Lim[dgr - 2])
                    {
                        ret = false;
                        r.push_back(make_pair(idx, ex(j)));
                    }
                }
            }

            SMatrixXd to_add(l.size() + r.size(), M * N);
            to_add.reserve(N * to_add.rows());
            VectorXd    b = VectorXd::Zero(l.size() + r.size());

            int i_ci = 0;
            RowVectorXd coeff, t;

            for (list<pair<int, double> >::iterator
                it  = l.begin(); it != l.end(); ++it)
            {
                coeff   = RowVectorXd::Ones(N);
                t       = RowVectorXd::Zero(N);
                
                t(dgr - 1)  = 1.0;
                for (int k = dgr; k < N; k++) 
                    t(k)    = t(k - 1) * it->second;

                for (int i = 0; i < dgr - 1; i++)
                    for (int k = 0; k < N; k++)
                        coeff(k)    *= (k - i);

                for (int k = dgr - 1; k < N; k++)
                    to_add.insert(i_ci, it -> first * N + k) = -coeff(k) * t(k);

                //clog<<"dgr  = "<< dgr << endl << coeff << endl << t <<endl;

                b(i_ci++)   = -Lim[dgr - 2] * (1.0 - _SAFE_RATE);
            }

            for (list<pair<int, double> >::iterator
                it  = r.begin(); it != r.end(); ++it)
            {
                coeff   = RowVectorXd::Ones(N);
                t       = RowVectorXd::Zero(N);

                t(dgr - 1) = 1.0;
                for (int k = dgr; k < N; k++)
                    t(k)    = t(k - 1) * it->second;
                    
                for (int i = 0; i < dgr - 1; i++)
                    for (int k = 0; k < N; k++)
                        coeff(k)    *= (k - i);

                for (int k = dgr - 1; k < N; k++)
                    to_add.insert(i_ci, it -> first * N + k) = coeff(k) * t(k);

                //clog<<"dgr  = "<< dgr << endl << coeff << endl << t <<endl;

                b(i_ci++)   = Lim[dgr - 2] * (1.0 - _SAFE_RATE);
            }

            CI  = combineRowsPr(CI, make_pair(to_add, b));
        }

        return ret;
    }

    static MatrixXd getConstrainsExtremums(
        const VectorXd &T,
        const MatrixXd &B,
        const VectorXd &P,
        const MatrixXd &ex)
    {
        if (ex.cols() == 0) 
            return MatrixXd::Zero(0, M * N + 1);

        int N_CI    = 0;

        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N - 2; ++j)
                N_CI += !(ex(j, i) < 0 || ex(j, i) > T(i));

        MatrixXd CI = MatrixXd::Zero(N_CI, M * N + 2);
        int i_ci    = 0;

        VectorXd t = VectorXd::Zero(N);
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N - 2; ++j)
            {
                if (ex(j, i) < 0 || ex(j, i) > T(i)) continue;

                t(0)    = 1.0;
                for (int k = 1; k < N; k++)
                    t(k)    = t(k - 1) * ex(j, i);

                CI.block<1, N>(i_ci, i * N) = t;    
                CI(i_ci, M * N)    = B(i, 0);
                CI(i_ci, M * N + 1)  = B(i, 1);
                i_ci += 1;
            }

        return CI;
    }

#ifdef _TRAJECTORY_GENERATOR_FLAG_USE_SPARSE_

static int _error_code = 0;

/* Some unknown bugs here. */
/* bug free now.
 */
    static VectorXd getPolyDer(
         const SMatrixXd & Q,
         const VectorXd & C,
         //const pair<SMatrixXd, VectorXd> & CE,
         const pair<SMatrixXd, VectorXd> & CI)
    {
        // number of variable
        const int N_X  = Q.rows();

        // linear cost 
        double c[N_X];
        for (int i = 0; i < N_X; i++) c[i] = C(i);

        // upper bound 
        double xupp[N_X];
        char ixupp[N_X];
        for (int i = 0 ; i < N_X; i++) xupp[i] = 0.0;
        memset(ixupp, 0, sizeof(ixupp));

        // lower bound
        double xlow[N_X];    
        char ixlow[N_X];
        for (int i = 0; i < N_X; i++) xlow[i] = 0.0;
        memset(ixlow, 0, sizeof(ixlow));


        //clog<<"QP: Fine"<<endl;
        vector<pair<pair<int, int>, double> > tmp;
        // quadratic cost matrix
        int N_Q = 0;

        for (int k = 0; k < Q.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(Q, k); it; ++it)
            if (it.col() <= it.row()) //lower trianglur matrix
            {
                N_Q += 1;
            }

        int iQ[N_Q], jQ[N_Q], i_q = 0;
        double dQ[N_Q];

        tmp.resize(N_Q);
        for (int k = 0; k < Q.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(Q, k); it; ++it)
                if (it.col() <= it.row())
                {
                    tmp[i_q++] = make_pair(make_pair(it.row(), it.col()), it.value());
                }

        sort(tmp.begin(), tmp.end());

        for (int i = 0; i < tmp.size(); ++i)
        {
            iQ[i] = tmp[i].first.first;
            jQ[i] = tmp[i].first.second;
            dQ[i] = tmp[i].second;
        }


        // equality constraints
        int N_CE = 0, N_CE_ROW = 0;
        int iCE[N_CE], jCE[N_CE];
        double dCE[N_CE], b[N_CE_ROW];

        // inequality constraints
        int N_CI = 0, N_CI_ROW = CI.first.rows();

        for (int k = 0; k < CI.first.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(CI.first, k); it; ++it)
            {
                N_CI += 1;
            }

        int iCI[N_CI], jCI[N_CI];
        char ilb[N_CI_ROW], iub[N_CI_ROW];
        double dCI[N_CI], lb[N_CI_ROW], ub[N_CI_ROW];

        int i_ci =0;

        tmp.resize(N_CI);
        for (int k = 0; k < CI.first.outerSize(); ++k)
            for (SMatrixXd::InnerIterator it(CI.first, k); it; ++it)
            {
                tmp[i_ci++]=make_pair(make_pair(it.row(),it.col()),it.value());
            }

        sort(tmp.begin(), tmp.end());

        for (int i = 0; i < tmp.size(); ++i)
        {
            iCI[i] = tmp[i].first.first;
            jCI[i] = tmp[i].first.second;
            dCI[i] = tmp[i].second;
        }

        for (int i = 0; i < N_CI_ROW; ++i)
        {
            lb[i]   = 0;
            ub[i]   = CI.second(i);
            ilb[i]  = 0;
            iub[i]  = 1;
        }

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
        int ierr    = s->solve(prob, vars, resid);
        _error_code = ierr;
        //clog<<"Just Fine log - 1"<<endl;
        delete s;
        delete qp;

        if (ierr == 0)
        {
            clog << "Successfully found the solution." << endl;
        }else{
            clog << "Something Wrong with the QP. ERR_CODE=" << ierr <<endl;
        }

        //clog<<"Just Fine log - 2"<<endl;
        VectorXd D = VectorXd::Zero(N_X);
        double d[N_X];
        vars->x->copyIntoArray(d);
        //clog<<"Just Fine log - 3"<<endl;
        for (int i = 0; i < N_X; i++)
            D(i) = d[i];

        //clog<<"Just Fine log - 4"<<endl;
        return D;
    }
#endif

    static VectorXd getCoeff(
        const double p_s,
        const double p_t,
        const MatrixXd & B,
        const MatrixXd & B_ori,
        const MatrixXd & E,
        const VectorXd & T,
        const RowVectorXd & vel,
        const RowVectorXd & acc,
        double & coeff_t)
    {
        //> Hessian Matrix 
        SMatrixXd H, H_DD, H_FD, Q = getCostMatrix(T);
        MatrixXd COST;

        //> Mapping Matrix
        SMatrixXd IP2D = getMappingMatrix(T);
        // clog << "1. IP2D." << endl;
        
        //IP2D   = MatrixXd::Identity(N*M,N*M);
       

        //> Endpoints
        pair<SMatrixXd, VectorXd> CE_0 = 
            getConstrainsEndpoints_D(T, B_ori, E, p_s, p_t, vel, acc);

        // clog << "2. Endpoints." << endl;
        // Corridors
        pair<pair<SMatrixXd, VectorXd>, pair<SMatrixXd, VectorXd> > CE_CI_1 = 
            getConstrainsOverlappedCorridors_D(T , B, E);

        // clog << "3. Corriors." << endl;

        VectorXd P, D, _coeff_t(2);
        pair<SMatrixXd, VectorXd> CE, CI;

        CE = combineRowsPr(CE_0, CE_CI_1.first);

        // clog << "3.5. CE."<<endl;
       
        SMatrixXd D2D, RDD, RDF, RD; 
        VectorXd d_f;
        int n_d, n_f;
        { // get those mapping matrices
            vector<bool> is_fixed((M + 1) * R, false);
            VectorXd d_F = VectorXd::Zero((M + 1) * R);
            for (int i = 0; i < CE.first.outerSize(); ++i)
            {
                for (auto it = SMatrixXd::InnerIterator(CE.first, i); it; ++it)
                {
                    is_fixed[it.col()] = true;
                    d_F[it.col()] = CE.second(it.row());
                }
            }
            // clog << "d_F = " << d_F.transpose() << endl;
        
            // clog << "4.1. preparation well" << endl;
            genMappingMatrices(T, is_fixed, M, N, R, D2D, RDF, RDD);
            RD = combineRowsSM(RDD, RDF);
            n_d = RDD.rows();
            n_f = RDF.rows();
            d_f = RDF * d_F; 
        }

        // clog << "4. mapping well" << endl;

        // constraints for the position boundary
        pair<SMatrixXd, VectorXd> CI_3 = make_pair(
            SMatrixXd(0, M * N), VectorXd::Zero(0));
        MatrixXd ex = MatrixXd(N - 2, 0);

        // constraints for the limits
        pair<SMatrixXd, VectorXd> CI_4 = make_pair(
            SMatrixXd(0, M * N), VectorXd::Zero(0));
        { // deal with the cost matrix
            auto A = IP2D * D2D.transpose() * RD.transpose();
            H = A.transpose() * Q * A;
            H_DD = H.block(0, 0, n_d, n_d);
            H_FD = H.block(n_d, 0, n_f, n_d);
        }
            
        vector<MatrixXd> derEx(2);
        derEx[0] = MatrixXd(N - 3, 0);
        derEx[1] = MatrixXd(N - 4, 0);

        vector<double> derLim(2);
        derLim[0] = max_vel * _LIM_RATE;
        derLim[1] = max_acc * _LIM_RATE;

        // clog << "5. Let us do the main loop" << endl;

        for (int loop_v = 0; loop_v < _N_LOOP; ++loop_v)
        {

            // the inequality constraints
            SMatrixXd A_d;
            VectorXd b_d;
            {
                auto ci = combineRowsSM(
                        CE_CI_1.second.first, 
                        CI_3.first * IP2D * D2D.transpose(), 
                        CI_4.first * IP2D * D2D.transpose());

                VectorXd ci_b(CE_CI_1.second.second.rows() + CI_3.second.rows() + CI_4.second.rows());
                ci_b << CE_CI_1.second.second, CI_3.second, CI_4.second;

                A_d = ci * RDD.transpose();
                b_d = ci_b - (ci * RDF.transpose() * d_f);
            }
            // clog << "6. Inequality done." << endl;


            // clog << "n_d = " << n_d << endl;
            // clog << "n_f = " << n_f << endl;

            // printSM(H_DD);
            // printSM(H_FD);
            // clog << "c = " << endl << (d_f.transpose() * H_FD) << endl;
            // printSM(A_d);
            // clog << "b_d" << endl << b_d.transpose() << endl; 
            VectorXd d_d(n_d);
            if (n_d > 0)
            {
                d_d   = getPolyDer(H_DD, d_f.transpose() * H_FD, make_pair(A_d, b_d));
            }
    
            //clog << "d_d = " << d_d.transpose() << endl;
            VectorXd d_all((M + 1) * R);
            d_all << d_d, d_f;
            // clog << "d_all = " << d_all.transpose() << endl;
            D = D2D.transpose() * RD.transpose() * d_all;

            // clog << "8.1 derivatives." << endl;
            P   = IP2D * D;
            COST = P.transpose() * Q * P;
            //clog<<"P:\n"<<P<<endl;
            //clog<<"D:\n"<<D<<endl;
            
            ex          = getExtremums(P, 1);
            derEx[0]    = getExtremums(P, 2);
            derEx[1]    = getExtremums(P, 3);


            //clog << "8. ex." << endl;
            if (!_CHECK_EX) 
                break;
            else
            {
                bool checkResult = 
                    checkExtremums(T, B, P, ex, CI_3) &&
                    checkDerExtremums(T, derLim, P, derEx, CI_4, _coeff_t);
                if (checkResult) 
                    break;
                else
                    _error_code = 1;
            }
            //clog << "The loop number: " << loop_v << endl;
            //clog << "9. check." << endl;
        }
        _qp_cost.push_back(COST.coeff(0, 0));
        //clog << "The COST = " << COST.coeff(0,0) << endl;

        //clog << "The _coeff_t: " << _coeff_t << endl;
        coeff_t = max(coeff_t, max(sqrt(_coeff_t(1)), _coeff_t(0)));

        return P;
    }

    static MatrixXd getTrajCoeff(
        const VectorXd &p_s,
        const VectorXd &p_t,
        const MatrixXd &B,
        const MatrixXd &B_ori,
        const MatrixXd &E,
        const VectorXd &T,
        const MatrixXd &vel,
        const MatrixXd &acc,
        double & coeff_t)
    {
        MatrixXd P = MatrixXd::Zero(M * N, _TOT_DIM);

        int _error_sum = 0;
        coeff_t = 0.1;
        for (int dim = 0; dim < _TOT_DIM; dim++)
        {
            P.block(0, dim, N * M, 1) = getCoeff(
                            p_s(dim),
                            p_t(dim),
                            B.block(0, dim << 1, M, 2),
                            B_ori.block(0, dim << 1, M, 2),
                            E.block(0, dim << 1, M - 1, 2),
                            T,
                            vel.row(dim),
                            acc.row(dim),
                            coeff_t);
            _error_sum += _error_code;
            //clog << "[ERROR] code = " << _error_code <<endl;
        }
        _error_code = _error_sum;

        return P;
    }

    pair<MatrixXd,VectorXd> TrajectoryGenerator::genPolyCoeffTime(
        const MatrixXd &PBE,
        const MatrixXd &inflated_path,
        const MatrixXd &vel,
        const MatrixXd &acc,
        const double maxVel,
        const double maxAcc,
        const double fVel,
        const double fAcc,
        double & coeff_t) 
    {
        assert(PBE.cols() == _TOT_BDY && inflated_path.cols() == _TOT_BDY);
        assert(vel.rows() == _TOT_DIM && vel.cols() == 2);
        assert(acc.rows() == _TOT_DIM && acc.cols() == 2);

        _qp_cost.clear();

        VectorXd p_s, p_t;
        MatrixXd B, B_, E, P;
        VectorXd T;
        // init()
        max_vel = maxVel;
        max_acc = maxAcc;
        f_vel = fVel;
        f_acc = fAcc;
        retInit(PBE, inflated_path, p_s, p_t, B, B_, E);

        // allocate time for each segment
        //T   = getTime_stupid(p_s, p_t, E, vel);
        T   = getTime_smart(p_s, p_t, E, vel);
        //T   *= 24.0 / T.sum();

        //clog<<"T:" <<T<<endl;
        // generate the coeff for polynomial traj
        //swap(B, B_);
#ifdef _TRAJECTORY_GENERATOR_FLAG_NO_INFLATION_
        assert(false);
        P   = getTrajCoeff(p_s, p_t, B, B, E, T, vel, acc, coeff_t); 
        coeff_t = 1.0;
#else
        P   = getTrajCoeff(p_s, p_t, B_, B, E, T, vel, acc, coeff_t);
#endif
        this->qp_cost = _qp_cost;
        if (_error_code > 0) T(0) = -1;
        return pair<MatrixXd,VectorXd>(P,T);
    }
}
