#include <hpp/fcl/shape/geometric_shapes.h>
#include <vector>
#include <cassert>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>
#include <CollisionManager/CubeObstacleFCL.hpp>
#include <CollisionManager/ObjectObstacleFCL.hpp>

MDP::CubeObstaclesFCL::CubeObstaclesFCL(const std::vector<float> _dimension, std::vector<MDP::ObstacleCoordinate> _positions, std::string _name, const bool _is_static)
    : ObjectObstacleFCL(_positions, _name, static_cast<hpp::fcl::ShapeBase *>(this->box = new hpp::fcl::Box(_dimension[0], _dimension[1], _dimension[2])), _is_static), dimension(_dimension)
{
}

MDP::CubeObstaclesFCL::CubeObstaclesFCL(const MDP::CubeObstacleJsonInfo json_obstacle_info)
    : ObjectObstacleFCL(json_obstacle_info.get_coordinates(), json_obstacle_info.get_name(), static_cast<hpp::fcl::ShapeBase *>(this->box = new hpp::fcl::Box(json_obstacle_info.get_dimensions()[0], json_obstacle_info.get_dimensions()[1], json_obstacle_info.get_dimensions()[2])), json_obstacle_info.get_is_static()), dimension(json_obstacle_info.get_dimensions())
{
}

MDP::CubeObstaclesFCL::~CubeObstaclesFCL()
{
    delete this->box;
}
