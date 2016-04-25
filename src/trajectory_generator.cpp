#include "voxel_trajectory/trajectorygenerator.h"
#include "voxel_trajectory/voxelmacro.h"

#include <cmath>
#include <utility>
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

#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseSeq.h>


using namespace Eigen;

typedef SparseMatrix<double> SparseXd;

class TrajectoryConfiguration
{
public:
    int nDim = 6;
    int nDer = 3;
    int  
} config;

class SolverConguration
{
};

class DimensionContent
{
};

class TrajectoryContent
{
};
