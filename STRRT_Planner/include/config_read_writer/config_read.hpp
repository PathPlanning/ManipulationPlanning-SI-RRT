#pragma once

#include <chrono>
#include <cmath>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>
#include <config_read_writer/RobotObstacleJsonInfo.hpp>
#include <config_read_writer/SphereObstacleJsonInfo.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace MDP {
class ConfigReader {
public:
  /*
  Reads the json and creates SceneTask struct
  */
  explicit ConfigReader(const std::string &path_to_scene_json); // constructor

  ~ConfigReader() = default; // destructor

  struct SceneTask {
    std::vector<double> start_configuration;
    std::vector<double> end_configuration;
    unsigned int frame_count;
    unsigned int fps;
    std::vector<std::shared_ptr<MDP::ObstacleJsonInfo>> obstacles;
    std::vector<MDP::RobotObstacleJsonInfo> robot_obstacles;
    std::string robot_urdf_path;
    std::vector<double> robot_base_coords;
    std::vector<double> robot_base_quat_rot;
    std::vector<std::string> robot_joints_order;
    std::vector<double> robot_joint_max_velocity;
    std::vector<double> robot_capsules_radius;
    int robot_joint_count;
    std::vector<MDP::ObstacleCoordinate> robot_base_position_vector;
    SceneTask(std::vector<double> _start_configuration,
              std::vector<double> _end_configuration, unsigned int _frame_count,
              unsigned int _fps,
              std::vector<std::shared_ptr<MDP::ObstacleJsonInfo>> _obstacles,
              std::vector<MDP::RobotObstacleJsonInfo> _robot_obstacles,
              std::string _robot_urdf_path,
              std::vector<double> _robot_base_coords,
              std::vector<double> _robot_base_quat_rot,
              std::vector<std::string> _robot_joints_order,
              std::vector<MDP::ObstacleCoordinate> _robot_base_position_vector,
              std::vector<double> _robot_joint_max_velocity,
              std::vector<double> _robot_capsules_radius,
              int _robot_joint_count)
        : robot_base_position_vector(_robot_base_position_vector),
          start_configuration(_start_configuration),
          end_configuration(_end_configuration), frame_count(_frame_count),
          fps(_fps), obstacles(_obstacles), robot_obstacles(_robot_obstacles),
          robot_urdf_path(_robot_urdf_path),
          robot_base_coords(_robot_base_coords),
          robot_base_quat_rot(_robot_base_quat_rot),
          robot_joints_order(_robot_joints_order),
          robot_joint_count(_robot_joint_count),
          robot_capsules_radius(_robot_capsules_radius),
          robot_joint_max_velocity(_robot_joint_max_velocity){};

    ~SceneTask() = default;                                 // destructor
    SceneTask(const SceneTask &other) = default;            // copy constructor
    SceneTask(SceneTask &&other) = default;                 // move constructor
    SceneTask &operator=(const SceneTask &other) = default; // copy assignment
    SceneTask &operator=(SceneTask &&other) = default;      // move assignment
  };

  const SceneTask &get_scene_task() const;

private:
  ConfigReader(const ConfigReader &other) = delete; // copy constructor
  ConfigReader(ConfigReader &&other) = delete;      // move constructor
  ConfigReader &
  operator=(const ConfigReader &other) = delete;          // copy assignment
  ConfigReader &operator=(ConfigReader &&other) = delete; // move assignment
  const SceneTask scene_task;
};

} // namespace MDP

namespace {
MDP::ConfigReader::SceneTask json_parser(const std::string &path_to_scene_json);
}
