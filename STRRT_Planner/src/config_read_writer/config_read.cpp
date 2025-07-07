#include <chrono>
#include <cmath>
#include <config_read_writer/config_read.hpp>
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

// Helper function to check if a key exists in JSON object and throw error if not
void check_json_key(const rapidjson::Value& json_obj, const std::string& key) {
  if (!json_obj.HasMember(key.c_str())) {
    std::cout << "Missing required JSON key: " << key << std::endl;
    throw std::runtime_error("Missing required JSON key: " + key);
  }
}

// Helper function to check if it is valid JSON object
 void check_json_object(const std::string& filename,const rapidjson::Value& json_obj,const std::string& key ) {
  if (!json_obj.IsObject()) {
    std::cout << "Not JSON Object: " << key << " filename: " <<filename <<std::endl;
    throw std::runtime_error("Not JSON Object: " );
  }
}
MDP::ConfigReader::SceneTask
json_parser(const std::string &path_to_scene_json) {    
  try
    {

  std::ifstream scene_file(path_to_scene_json);

  if (!scene_file.is_open()) {
    throw std::runtime_error("Failed to open scene task json file.");
  }

  rapidjson::IStreamWrapper scene_wrapper(scene_file);
  rapidjson::Document scene_parsed_data;
  scene_parsed_data.ParseStream(scene_wrapper);
  scene_parsed_data.filename=path_to_scene_json;
  // Check for required top-level keys
  check_json_key(scene_parsed_data, "start_configuration");
  check_json_key(scene_parsed_data, "end_configuration");
  check_json_key(scene_parsed_data, "frame_count");
  check_json_key(scene_parsed_data, "fps");
  check_json_key(scene_parsed_data, "obstacles");

  std::vector<double> start_configuration;
  std::vector<double> end_configuration;
  for (auto &v : scene_parsed_data["start_configuration"].GetArray()) {
    start_configuration.push_back(v.GetDouble());
  }
  for (auto &v : scene_parsed_data["end_configuration"].GetArray()) {

    end_configuration.push_back(v.GetDouble());
  }

  int frame_count = scene_parsed_data["frame_count"].GetInt();
  int fps = scene_parsed_data["fps"].GetInt();

  std::vector<std::shared_ptr<MDP::ObstacleJsonInfo>> obstacles{};
  std::vector<MDP::RobotObstacleJsonInfo> robot_obstacles{};
  for (rapidjson::Value &obstacles_json :
       scene_parsed_data["obstacles"].GetArray()) {
    // Check for required obstacle keys
    check_json_key(obstacles_json, "type");
    check_json_key(obstacles_json, "name");

    std::string obstacle_type = obstacles_json["type"].GetString();
    std::string obstacle_name = obstacles_json["name"].GetString();
    // std::cout<<obstacle_type<<std::endl;
    if (obstacle_type == "static_box") {
      std::vector<std::vector<float>> obstacles_coords;
      obstacles_coords.push_back(std::vector<float>());
      
      check_json_key(obstacles_json, "positions");
      for (rapidjson::Value &coordinates :
           obstacles_json["positions"].GetArray()[0].GetArray()) {
        obstacles_coords[0].push_back(coordinates.GetDouble());
      }

      check_json_key(obstacles_json, "dimensions");
      std::vector<float> dimensions;
      for (rapidjson::Value &dimension_unit :
           obstacles_json["dimensions"].GetArray()) {
            
        dimensions.push_back(dimension_unit.GetDouble());
      }

      assert(dimensions.size() == 3);
      obstacles.push_back(std::make_shared<MDP::CubeObstacleJsonInfo>(
          obstacle_name, obstacle_type, obstacles_coords, fps, dimensions,
          true));
    } else if (obstacle_type == "dynamic_box") {

      std::vector<std::vector<float>> obstacles_coords;
      check_json_key(obstacles_json, "positions");
      for (rapidjson::Value &coordinate :
           obstacles_json["positions"].GetArray()) {

        obstacles_coords.push_back(std::vector<float>());
        for (rapidjson::Value &coordinate_unit : coordinate.GetArray()) {
          obstacles_coords.back().push_back(coordinate_unit.GetDouble());
        }
      }
      std::vector<float> dimensions;
      check_json_key(obstacles_json, "dimensions");
      for (rapidjson::Value &dimension_unit :
           obstacles_json["dimensions"].GetArray()) {

        dimensions.push_back(dimension_unit.GetDouble());
      }
      // std::cout<<dimensions.size()<<std::endl;
      assert(dimensions.size() == 3);
      obstacles.push_back(std::make_shared<MDP::CubeObstacleJsonInfo>(
          obstacle_name, obstacle_type, obstacles_coords, fps, dimensions,
          false));
    } else if (obstacle_type == "dynamic_robot") {
      check_json_key(obstacles_json, "urdf_file_path");
      std::string urdf_file_path = obstacles_json["urdf_file_path"].GetString();

      std::vector<std::string> robot_joints_order;
      check_json_key(obstacles_json, "robot_joints_order");
      for (rapidjson::Value &joint :
           obstacles_json["robot_joints_order"].GetArray()) {

        robot_joints_order.push_back(joint.GetString());
      }

      std::vector<std::vector<float>>
          robot_base_coordinates_raw; // TODO: remove hardcode and make dynamic
                                      // base_coord
      robot_base_coordinates_raw.emplace_back();
      check_json_key(obstacles_json, "robot_base_coords");
      check_json_key(obstacles_json, "robot_base_quat_rot");
      for (rapidjson::Value &base_coordinate :
           obstacles_json["robot_base_coords"].GetArray()) {

        robot_base_coordinates_raw[0].push_back(base_coordinate.GetDouble());
      }
      for (rapidjson::Value &base_coordinate_rot :
           obstacles_json["robot_base_quat_rot"].GetArray()) {

        robot_base_coordinates_raw[0].push_back(
            base_coordinate_rot.GetDouble());
      }

      std::vector<std::vector<double>> robot_trajectory;
      check_json_key(obstacles_json, "trajectory");
      for (rapidjson::Value &coordinate :
           obstacles_json["trajectory"].GetArray()) {

        robot_trajectory.push_back(std::vector<double>());
        for (rapidjson::Value &coordinate_unit : coordinate.GetArray()) {

          robot_trajectory.back().push_back(coordinate_unit.GetDouble());
        }
      }
      robot_obstacles.emplace_back(obstacle_name, obstacle_type, urdf_file_path,
                                   robot_joints_order, robot_trajectory,
                                   robot_base_coordinates_raw, fps, true);
    } else if (obstacle_type == "static_sphere") {
      std::vector<std::vector<float>> obstacles_coords;
      obstacles_coords.push_back(std::vector<float>());
      check_json_key(obstacles_json, "positions");
      for (rapidjson::Value &coordinates :
           obstacles_json["positions"].GetArray()[0].GetArray()) {

        obstacles_coords[0].push_back(coordinates.GetDouble());
      }
      check_json_key(obstacles_json, "radius");
      float radius = (obstacles_json["radius"].GetDouble());

      obstacles.push_back(std::make_shared<MDP::SphereObstacleJsonInfo>(
          obstacle_name, obstacle_type, obstacles_coords, fps, radius, true));
    } else if (obstacle_type == "dynamic_sphere") {
      std::vector<std::vector<float>> obstacles_coords;
      check_json_key(obstacles_json, "positions");
      for (rapidjson::Value &coordinate :
           obstacles_json["positions"].GetArray()) {

        obstacles_coords.push_back(std::vector<float>());
        for (rapidjson::Value &coordinate_unit : coordinate.GetArray()) {
          obstacles_coords.back().push_back(coordinate_unit.GetDouble());
        }
      }
      check_json_key(obstacles_json, "radius");
      float radius = (obstacles_json["radius"].GetDouble());
      assert(radius > 0);
      obstacles.push_back(std::make_shared<MDP::SphereObstacleJsonInfo>(
          obstacle_name, obstacle_type, obstacles_coords, fps, radius, false));
    }
  }
  check_json_key(scene_parsed_data, "robot_urdf");

  std::string robot_urdf_path = scene_parsed_data["robot_urdf"].GetString();
  std::vector<double> robot_base_coords;
  std::vector<double> robot_base_quat_rot;
  check_json_key(scene_parsed_data, "robot_base_coords");
  check_json_key(scene_parsed_data, "robot_base_quat_rot");
  check_json_key(scene_parsed_data, "robot_joints_order");
  check_json_key(scene_parsed_data, "robot_joint_max_velocity");
  check_json_key(scene_parsed_data, "robot_capsules_radius");

  for (auto &v : scene_parsed_data["robot_base_coords"].GetArray()) {

    robot_base_coords.push_back(v.GetDouble());
  }
  for (auto &v : scene_parsed_data["robot_base_quat_rot"].GetArray()) {

    robot_base_quat_rot.push_back(v.GetDouble());
  }
  std::vector<std::string> robot_joints_order;
  for (auto &v : scene_parsed_data["robot_joints_order"].GetArray()) {

    robot_joints_order.push_back(v.GetString());
  }

  int robot_joint_count = robot_joints_order.size();

  std::vector<double> robot_joint_max_velocity;
  for (auto &v : scene_parsed_data["robot_joint_max_velocity"].GetArray()) {
    
    robot_joint_max_velocity.push_back(v.GetDouble());
  }

  std::vector<double> robot_capsules_radius;
  for (auto &v : scene_parsed_data["robot_capsules_radius"].GetArray()) {

    robot_capsules_radius.push_back(v.GetDouble());
  }
  std::vector<MDP::ObstacleCoordinate> robot_base_position_vector;
  robot_base_position_vector.push_back(MDP::ObstacleCoordinate(
      robot_base_coords[0], robot_base_coords[1], robot_base_coords[2],
      robot_base_quat_rot[0], robot_base_quat_rot[1], robot_base_quat_rot[2],
      robot_base_quat_rot[3], 0, 0));



    scene_file.close();
  return MDP::ConfigReader::SceneTask(
      start_configuration, end_configuration, frame_count, fps, obstacles,
      robot_obstacles, robot_urdf_path, robot_base_coords, robot_base_quat_rot,
      robot_joints_order, robot_base_position_vector, robot_joint_max_velocity,
      robot_capsules_radius, robot_joint_count);
    }
    catch (const char* error_message)
    {

        std::cout <<"filename: "<<path_to_scene_json<<" "<< error_message << std::endl;
    }
}

/*
Get scene task. Returns SceneTask struct
*/
const MDP::ConfigReader::SceneTask &MDP::ConfigReader::get_scene_task() const {
  return this->scene_task;
}

MDP::ConfigReader::ConfigReader(const std::string &path_to_scene_json)
    : scene_task(::json_parser(path_to_scene_json)){};
