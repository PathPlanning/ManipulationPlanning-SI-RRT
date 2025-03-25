#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include "coal/collision.h"
#include "coal/collision_data.h"
#include "config_read_writer/ObstacleJsonInfo.hpp"
namespace MDP
{

    class CubeObstacleJsonInfo: public ObstacleJsonInfo
    {
    public:
        CubeObstacleJsonInfo(std::string _name, std::string _type,
                             std::vector<std::vector<float>> coordinates_raw, 
                             float fps, std::vector<float> _dimensions, 
                             bool _is_static);
        ~CubeObstacleJsonInfo() = default;                                            // destructor
        CubeObstacleJsonInfo(const CubeObstacleJsonInfo &other) = default;            // copy constructor
        CubeObstacleJsonInfo(CubeObstacleJsonInfo &&other) = default;                 // move constructor
        CubeObstacleJsonInfo &operator=(const CubeObstacleJsonInfo &other) = default; // copy assignment
        CubeObstacleJsonInfo &operator=(CubeObstacleJsonInfo &&other) = default;      // move assignment

        
        std::vector<float> get_dimensions() const;

        
    private:
        const std::vector<float> dimensions;
    };

} // namespace MDP
