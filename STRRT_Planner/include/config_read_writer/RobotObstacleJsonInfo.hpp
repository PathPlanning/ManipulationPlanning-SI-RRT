#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include "hpp/fcl/collision.h"
#include "hpp/fcl/collision_data.h"
#include <config_read_writer/CubeObstacleJsonInfo.hpp>

//TODO: Add inheritance from ObstacleJsonInfo.
namespace MDP
{

    class RobotObstacleJsonInfo
    {
    public:
        RobotObstacleJsonInfo(const std::string& _name, const std::string& _type, const std::string& _urdf_file_path,
                            const std::vector<std::string>& _robot_joints_order,
                            const std::vector<std::vector<double>>& _trajectory, 
                            std::vector<std::vector<float>>& base_coordinates_raw,
                            const float& fps,  
                            const bool& _is_static);                                    // constructor
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
        std::vector<std::vector<double>> get_trajectory() const;

        
    private:
        const std::string name; // name of the obstacle
        const std::string type; // type of the obstacle ("dynamic_robot")
        const std::string urdf_file_path;  // path to robot urdf (from MDP folder)
        const std::vector<std::string> robot_joints_order; // robot joint order, names joints in trajectoryu space vector
        std::vector<MDP::ObstacleCoordinate> base_coordinates; // coordinastes of robot's base
        const std::vector<std::vector<double>> trajectory; // trajectory of robot joints
        const bool is_static; // if is_static, then robot's base is stationary
    };

} // namespace MDP
