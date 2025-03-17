#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include "hpp/fcl/collision.h"
#include "hpp/fcl/collision_data.h"
#include "config_read_writer/CubeObstacleJsonInfo.hpp"
namespace MDP
{

    class SphereObstacleJsonInfo: public ObstacleJsonInfo
    {
    public:
        SphereObstacleJsonInfo(std::string _name, std::string _type,
                             std::vector<std::vector<float>> coordinates_raw, 
                             float fps, float radius, 
                             bool _is_static);
        ~SphereObstacleJsonInfo() = default;                                            // destructor
        SphereObstacleJsonInfo(const SphereObstacleJsonInfo &other) = default;            // copy constructor
        SphereObstacleJsonInfo(SphereObstacleJsonInfo &&other) = default;                 // move constructor
        SphereObstacleJsonInfo &operator=(const SphereObstacleJsonInfo &other) = default; // copy assignment
        SphereObstacleJsonInfo &operator=(SphereObstacleJsonInfo &&other) = default;      // move assignment

        float get_radius() const;
        
    private:
        float radius;
    };

} // namespace MDP
