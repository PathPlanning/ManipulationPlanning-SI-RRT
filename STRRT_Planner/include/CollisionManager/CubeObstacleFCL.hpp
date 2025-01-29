#pragma once

#include <hpp/fcl/shape/geometric_shapes.h>
#include <vector>
#include <Eigen/Core>
#include <CollisionManager/RobotObstacleFCL.hpp>
#include <CollisionManager/ObjectObstacleFCL.hpp>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>

namespace MDP
{

    class CubeObstaclesFCL: public ObjectObstacleFCL
    {
    public:
        CubeObstaclesFCL(const std::vector<float> dimension, const std::vector<MDP::ObstacleCoordinate> positions, std::string _name,const bool _is_static);
        CubeObstaclesFCL(const MDP::CubeObstacleJsonInfo json_obstacle_info);
        ~CubeObstaclesFCL();                                                 // destructor
        CubeObstaclesFCL(const CubeObstaclesFCL &other) = delete;            // copy constructor
        CubeObstaclesFCL(CubeObstaclesFCL &&other) = default;                 // move constructor
        CubeObstaclesFCL &operator=(const CubeObstaclesFCL &other) = delete; // copy assignment
        CubeObstaclesFCL &operator=(CubeObstaclesFCL &&other) = default;      // move assignment


    private:
        hpp::fcl::Box* box;
        std::vector<float> dimension;
    };
}