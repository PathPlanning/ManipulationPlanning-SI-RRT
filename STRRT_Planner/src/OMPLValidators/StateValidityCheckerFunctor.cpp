#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <OMPLValidators/StateValidityCheckerFunctor.hpp>
#include <algorithm>

namespace ob = ompl::base;
namespace og = ompl::geometric;

MDP::StateValidityCheckerFunctor::StateValidityCheckerFunctor(const ob::SpaceInformationPtr &si, MDP::ConfigReader::SceneTask _scene_task)
    : ob::StateValidityChecker(si), collision_manager(_scene_task), scene_task(_scene_task)
{
    // init collision_manager
}

bool MDP::StateValidityCheckerFunctor::isValid(const ob::State *state) const
{

    validation_counter++;
    // get time 
    float t = si_->getStateSpace()->as<ob::SpaceTimeStateSpace>()->getStateTime(state);
    // std::cout<<t<<" "<<this->scene_task.frame_count/this->scene_task.fps<<std::endl;
    assert((int)(t/this->scene_task.fps)<this->scene_task.frame_count); 
    // get robot position
    auto state_pos_ptr = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    std::vector<double> pos;
    for (int i = 0; i < scene_task.robot_joint_count; i++)
    {
        pos.push_back(state_pos_ptr[i]);
    }

    // check collision
    return !(this->collision_manager.check_collision(pos, t));
}

double MDP::StateValidityCheckerFunctor::clearance(const ob::State *state) const
{
    // get time
    double t = si_->getStateSpace()->as<ob::SpaceTimeStateSpace>()->getStateTime(state);

    // get robot position
    auto state_pos_ptr = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;
    std::vector<double> pos;
    for (int i = 0; i < scene_task.robot_joint_count; i++)
    {
        pos.push_back(state_pos_ptr[i]);
    }

    // check distance
    bool is_collision;
    std::vector<MDP::CollisionManager::robot_link_distance_to_obj> distances =  this->collision_manager.get_distances(pos, t, is_collision);
    auto min_distance = [ ] ( auto &  lhs, const  auto&  rhs)->bool {
            return lhs.min_distance < rhs.min_distance;
    };
    return std::min_element(distances.begin(), distances.end(),min_distance)->min_distance;
}

int MDP::StateValidityCheckerFunctor::get_count_of_validation() const
{
    return this->validation_counter;
}

std::vector<std::pair<float, float>> MDP::StateValidityCheckerFunctor::get_planned_robot_limits() const
{
    return this->collision_manager.get_planned_robot_limits();
}
