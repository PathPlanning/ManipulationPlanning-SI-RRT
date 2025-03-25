#pragma once
#include <coal/shape/geometric_shapes.h>
#include <vector>
#include <Eigen/Core>
#include <CollisionManager/RobotObstacleFCL.hpp>
#include <CollisionManager/ObjectObstacleFCL.hpp>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/SphereObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>

namespace MDP
{
   
    class SphereObstaclesFCL : public ObjectObstacleFCL
    {
    public:
        SphereObstaclesFCL(const float _radius, const std::vector<MDP::ObstacleCoordinate> positions, std::string _name, const std::string& _type, const bool _is_static);
        SphereObstaclesFCL(const MDP::SphereObstacleJsonInfo json_obstacle_info);
        ~SphereObstaclesFCL();                                                   // destructor
        SphereObstaclesFCL(const SphereObstaclesFCL &other) = delete;            // copy constructor
        SphereObstaclesFCL(SphereObstaclesFCL &&other) = default;                // move constructor
        SphereObstaclesFCL &operator=(const SphereObstaclesFCL &other) = delete; // copy assignment
        SphereObstaclesFCL &operator=(SphereObstaclesFCL &&other) = default;     // move assignment

        float get_radius() const;

    private:
        const float radius;
        coal::Sphere *sphere;
    };
}