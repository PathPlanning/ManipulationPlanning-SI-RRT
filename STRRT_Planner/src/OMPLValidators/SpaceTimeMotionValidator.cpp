#include <OMPLValidators/SpaceTimeMotionValidator.hpp>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/util/RandomNumbers.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
MDP::SpaceTimeMotionValidator::SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si, MDP::ConfigReader::SceneTask _scene_task)
    : ob::MotionValidator(si), scene_task(_scene_task),si(si),
      validation_counter(0), robot_joint_count(_scene_task.robot_joint_count){};

bool MDP::SpaceTimeMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const 
{   
    validation_counter++;
    if (!si->isValid(s2))
    {
        return false;
    }
    std::vector<double> delta_coords(robot_joint_count);
    for (int i = 0; i < robot_joint_count; i++)
    {
        // std::cout<<"s1 "<<s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i]<<" s2 "<<s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i]<<std::endl;
        delta_coords[i] = abs(s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i] -
                              s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i]);
    }

    double t1 = s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double t2 = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double delta_t = t2 - t1;

    if (delta_t <= 0) 
    {
        return false;
    }

    if ((std::sqrt(std::inner_product(delta_coords.begin(), delta_coords.end(), delta_coords.begin(), 0.0)) / delta_t) > scene_task.robot_joint_max_velocity[0])
    {
        return false;
    }

    int interpolation_steps = std::max((int)(std::sqrt(std::inner_product(delta_coords.begin(), delta_coords.end(), delta_coords.begin(), 0.0)) / 0.1),(int)(2*delta_t*scene_task.fps+0.5)) + 2;
    auto interpolated_state = si->allocState();
    for (int i = 0; i < interpolation_steps; ++i)
    {
        
        double t = static_cast<double>(i) / interpolation_steps;
        si->getStateSpace()->interpolate(s1, s2, t, interpolated_state);


        
        if (!si->isValid(interpolated_state))
        {
            si->freeState(interpolated_state);
            return false;
        }
        

    }
    si->freeState(interpolated_state);
    return true;
}

bool MDP::SpaceTimeMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const 
{
    si->copyState(lastValid.first,s1);
    lastValid.second = 0;
    validation_counter++;

    if (!si->isValid(s2))
    {
        return false;
    }

    std::vector<double> delta_coords(robot_joint_count);
    for (int i = 0; i < robot_joint_count; i++)
    {
        // std::cout<<"s1 "<<s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i]<<" s2 "<<s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i]<<std::endl;
        delta_coords[i] = abs(s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i] -
                              s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i]);
    }

    double t1 = s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double t2 = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double delta_t = t2 - t1;

    if (delta_t <= 0) 
    {
        return false;
    }

    if ((std::sqrt(std::inner_product(delta_coords.begin(), delta_coords.end(), delta_coords.begin(), 0.0)) / delta_t) > scene_task.robot_joint_max_velocity[0])
    {
        return false;
    }

    int interpolation_steps = std::max((int)(std::sqrt(std::inner_product(delta_coords.begin(), delta_coords.end(), delta_coords.begin(), 0.0)) / 0.1),(int)(delta_t*scene_task.fps+0.5)) + 2;
    auto interpolated_state = si->allocState();
    for (int i = 0; i < interpolation_steps; ++i)
    {
        
        double t = static_cast<double>(i) / interpolation_steps;
        si->getStateSpace()->interpolate(s1, s2, t, interpolated_state);


        
        if (!si->isValid(interpolated_state))
        {
            si->freeState(interpolated_state);
            return false;
        }
        si->copyState(lastValid.first,interpolated_state);
        lastValid.second = t;
    }
    si->freeState(interpolated_state);
    return true;
    
}

int MDP::SpaceTimeMotionValidator::get_count_of_validation() const
{
    return validation_counter;
}
