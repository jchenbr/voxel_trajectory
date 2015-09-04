#ifndef _TRAJECTORY_GENERATOR_RRT_PLANNER_H_
#define _TRAJECTORY_GENERATOR_RRT_PLANNER_H_

#include "voxel_trajectory/voxelmacro.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>
#include <ompl/util/Console.h>

#include <eigen3/Eigen/Dense>
#include <boost/function.hpp>
#include <vector>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace VoxelTrajectory
{ 

class RRTPlannerManager
{
private:
    typedef std::vector<Eigen::Vector3d> Path_t;
    typedef boost::function<bool (double, double, double)> Validator_t;

    double _cell_size = 0.2;
    double _goal_bias = 0.05;
    double _rrt_range = 0.05;
    double _rrt_timeout = 0.5;
    double _bdy[_TOT_BDY] = {-20, 20, -20, 20, 0, 3}; 


    Validator_t _validator;

    bool isStateVaild(const ob::State *state)
    {
        double x = state->as<ob::RealVectorStateSpace::StateType>()->values[_DIM_x];
        double y = state->as<ob::RealVectorStateSpace::StateType>()->values[_DIM_y];
        double z = state->as<ob::RealVectorStateSpace::StateType>()->values[_DIM_z];
        return _validator(x, y, z);
    }

public:

#if 0
    RRTPlannerManager(Validator_t validator)
    {
        //clog << "[rrt] " << " 0 " << endl;
        _validator = validator;

        ob::RealVectorBounds bounds(_TOT_DIM);
        bounds.setLow(_DIM_x, _bdy[_BDY_x]);
        bounds.setHigh(_DIM_x, _bdy[_BDY_X]);
        bounds.setLow(_DIM_y, _bdy[_BDY_y]);
        bounds.setHigh(_DIM_y, _bdy[_BDY_Y]);
        bounds.setLow(_DIM_z, _bdy[_BDY_z]);
        bounds.setHigh(_DIM_z, _bdy[_BDY_Z]);

        //clog << "[rrt] " << " a " << endl;

        std::vector<double> sz {_cell_size, _cell_size, _cell_size};
        _space = 
            ob::StateSpacePtr(new ob::RealVectorStateSpace(_TOT_DIM));
        _space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        _space->as<ob::RealVectorStateSpace>()->setup();
        //_space->as<ob::RealVectorStateSpace>()->getDefaultProjection()->setCellSizes(sz);
	
        clog << "[rrt] " << " b " << endl;

        _space_info = ob::SpaceInformationPtr(new ob::SpaceInformation(_space));
        _space_info->setStateValidityChecker(
                boost::bind(&RRTPlannerManager::isStateVaild, this, _1)
                );
        //clog << "[rrt] " << " c " << endl;

        auto method_ptr = new og::RRTstar(_space_info);
        method_ptr->setPrune(false);
        method_ptr->setRange(_rrt_range);
        //clog << "[rrt] " << " d " << endl;

        _planner = ob::PlannerPtr(method_ptr);
        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(_space_info));
        _planner->setProblemDefinition(pdef);
        _planner->setup();
        //clog << "[rrt] " << " e " << endl;

        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
        //clog << "[rrt] " << " f " << endl;
    }
    bool planWithSrcDest
        (
            const Eigen::Vector3d & src_vec, 
            const Eigen::Vector3d & dest_vec,
            Path_t & output
        )
    {
        ob::ScopedState<> src(_space), dest(_space);

        src[_DIM_x] = src_vec[_DIM_x];
        src[_DIM_y] = src_vec[_DIM_y];
        src[_DIM_z] = src_vec[_DIM_z];

        dest[_DIM_x] = dest_vec[_DIM_x];
        dest[_DIM_y] = dest_vec[_DIM_y];
        dest[_DIM_z] = dest_vec[_DIM_z];

        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(_space_info));
        pdef->setStartAndGoalStates(src, dest, _goal_bias);

        _planner->setProblemDefinition(pdef);

        auto _prob_status = _planner->solve(_rrt_timeout);

        if (_prob_status)
        {
            auto _path = dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath());
            output.clear();
            int len = _path.getStateCount();
            for (int idx = 0; idx < len; ++idx)
            {
                auto state = _path.getState(idx)->as<ob::RealVectorStateSpace::StateType>()->values;

                output.push_back(Eigen::Vector3d(state[_DIM_x], state[_DIM_y], state[_DIM_z]));
            }
            return true;
        }
        else
        {
            return false;
        }
    }
#endif

    RRTPlannerManager(Validator_t validator)
    {
        //clog << "[rrt] " << " 0 " << endl;
        _validator = validator;
    }

    bool planWithSrcDest
        (
            const Eigen::Vector3d & src_vec, 
            const Eigen::Vector3d & dest_vec,
            Path_t & output
        )
    {
        ob::StateSpacePtr space(new ob::RealVectorStateSpace(_TOT_DIM));

        ob::RealVectorBounds bounds(_TOT_DIM);
        bounds.setLow(_DIM_x, _bdy[_BDY_x]);
        bounds.setHigh(_DIM_x, _bdy[_BDY_X]);
        bounds.setLow(_DIM_y, _bdy[_BDY_y]);
        bounds.setHigh(_DIM_y, _bdy[_BDY_Y]);
        bounds.setLow(_DIM_z, _bdy[_BDY_z]);
        bounds.setHigh(_DIM_z, _bdy[_BDY_Z]);

        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        si->setStateValidityChecker(boost::bind(&RRTPlannerManager::isStateVaild, this, _1));

        ob::ScopedState<> src(space);
        ob::ScopedState<> dest(space);

        src[_DIM_x] = src_vec[_DIM_x];
        src[_DIM_y] = src_vec[_DIM_y];
        src[_DIM_z] = src_vec[_DIM_z];

        dest[_DIM_x] = dest_vec[_DIM_x];
        dest[_DIM_y] = dest_vec[_DIM_y];
        dest[_DIM_z] = dest_vec[_DIM_z];

        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

        pdef->setStartAndGoalStates(src, dest, _goal_bias);

        auto method = new og::RRTstar(si);
        //auto method = new og::RRTConnect(si);
        ob::PlannerPtr planner(method);

        planner->setProblemDefinition(pdef);
        planner->setup();

        ob::PlannerStatus  _prob_status = planner->solve(_rrt_timeout);

        if (_prob_status)
        {
            auto _path = dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath());
            output.clear();
            int len = _path.getStateCount();
            for (int idx = 0; idx < len; ++idx)
            {
                auto state = _path.getState(idx)->as<ob::RealVectorStateSpace::StateType>()->values;

                output.push_back(Eigen::Vector3d(state[_DIM_x], state[_DIM_y], state[_DIM_z]));
            }
            return true;
        }
        else
        {
            return false;
        }
    }
};

}

#endif
