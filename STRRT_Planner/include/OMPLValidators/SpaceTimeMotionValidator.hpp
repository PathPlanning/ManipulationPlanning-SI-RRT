#pragma once
// #include <ompl/geometric/planners/rrt/RRTstar.h>
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
#include <config_read_writer/config_read.hpp>

namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace MDP
{
class SpaceTimeMotionValidator : public ob::MotionValidator {
public:
    SpaceTimeMotionValidator(const ob::SpaceInformationPtr& si, MDP::ConfigReader::SceneTask _scene_task);

    bool checkMotion(const ob::State* s1, const ob::State* s2) const override;

    bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const override;

    int get_count_of_validation() const;

private:
    const ob::SpaceInformationPtr si;
    double collision_check_interpolation;
    mutable int validation_counter;
    int robot_joint_count;
    MDP::ConfigReader::SceneTask scene_task;
};
}
