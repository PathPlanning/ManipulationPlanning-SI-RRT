#include <hpp/fcl/shape/geometric_shapes.h>
#include <vector>
#include <cassert>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/SphereObstacleJsonInfo.hpp>
#include <CollisionManager/SphereObstacleFCL.hpp>
#include <CollisionManager/ObjectObstacleFCL.hpp>

MDP::SphereObstaclesFCL::SphereObstaclesFCL(const float _radius, std::vector<MDP::ObstacleCoordinate> _positions, std::string _name, const bool _is_static)
    : radius(_radius), ObjectObstacleFCL(_positions, _name, static_cast<hpp::fcl::ShapeBase *>(this->sphere = new hpp::fcl::Sphere(_radius)), _is_static)
{
}

MDP::SphereObstaclesFCL::SphereObstaclesFCL(const MDP::SphereObstacleJsonInfo json_obstacle_info)

    : ObjectObstacleFCL(json_obstacle_info.get_coordinates(), json_obstacle_info.get_name(), static_cast<hpp::fcl::ShapeBase *>(this->sphere = new hpp::fcl::Sphere(json_obstacle_info.get_radius())), json_obstacle_info.get_is_static()), radius(json_obstacle_info.get_radius())
{
}

MDP::SphereObstaclesFCL::~SphereObstaclesFCL()
{
    delete this->sphere;
}

float MDP::SphereObstaclesFCL::get_radius() const
{
    return this->radius;
}
