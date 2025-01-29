#pragma once
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
#include <CollisionManager/CollisionManager.hpp>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace MDP
{

    class StateValidityCheckerFunctor : public ob::StateValidityChecker
    {
    public:
        StateValidityCheckerFunctor(const ob::SpaceInformationPtr &si, MDP::ConfigReader::SceneTask _scene_task);


        
        std::vector<std::pair<float, float>> get_planned_robot_limits() const;
        bool isValid(const ob::State *state) const override;

        double clearance(const ob::State *state) const override;

        int get_count_of_validation() const;

    private:
        MDP::ConfigReader::SceneTask scene_task;
        mutable MDP::CollisionManager collision_manager;
        mutable int validation_counter = 0;
    };

}