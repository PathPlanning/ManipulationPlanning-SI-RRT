#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include "coal/collision.h"
#include "coal/collision_data.h"
#include <config_read_writer/CubeObstacleJsonInfo.hpp>

//TODO: Add inheritance from ObstacleJsonInfo.
namespace MDP
{

    class RobotObstacleJsonInfo
    {
    public:
        struct PathState
            {
                std::vector<double> configuration_coordinates;
                double time;
                PathState(const std::vector<double> _configuration_coordinates, const double _time) : configuration_coordinates(_configuration_coordinates), time(_time) {};
            };

        RobotObstacleJsonInfo(const std::string& _name, const std::string& _type, const std::string& _urdf_file_path,
                            const std::vector<std::string>& _robot_joints_order,
                            const std::vector<PathState>& _trajectory,
                            const std::vector<std::vector<float>>& base_coordinates_raw,
                            const float& fps,  
                            bool _is_static);                                    // constructor
        ~RobotObstacleJsonInfo() = default;                                            // destructor
        RobotObstacleJsonInfo(const RobotObstacleJsonInfo &other) = default;            // copy constructor
        RobotObstacleJsonInfo(RobotObstacleJsonInfo &&other) = default;                 // move constructor
        RobotObstacleJsonInfo &operator=(const RobotObstacleJsonInfo &other) = default; // copy assignment
        RobotObstacleJsonInfo &operator=(RobotObstacleJsonInfo &&other) = default;      // move assignment

        std::string get_name() const;
        std::string get_type() const;
        std::string get_urdf_file_path() const;
        std::vector<std::string> get_robot_joints_order() const;
        std::vector<MDP::ObstacleCoordinate> get_base_coordinates() const;
        bool get_is_static() const;
        std::vector<MDP::RobotObstacleJsonInfo::PathState> get_trajectory() const;

        
    private:
        std::string name; // name of the obstacle
        std::string type; // type of the obstacle ("dynamic_robot")
        std::string urdf_file_path;  // path to robot urdf (from MDP folder)
        std::vector<std::string> robot_joints_order; // robot joint order, names joints in trajectoryu space vector
        std::vector<MDP::ObstacleCoordinate> base_coordinates; // coordinastes of robot's base
        std::vector<PathState> trajectory; // trajectory of robot joints
        bool is_static; // if is_static, then robot's base is stationary
    };

} // namespace MDP
