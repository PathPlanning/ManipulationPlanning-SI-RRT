#pragma once

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <map>
#include <CollisionManager/CollisionManager.hpp>

namespace MDP
{
    // class to write obstacle positions to json for debugging and blender visualization
    class ObstaclePosWriter
    {
        /*
        stores obstacle positions and saves it in json
        */
    public:
        ObstaclePosWriter(const MDP::ConfigReader::SceneTask _scene_task); // constructor
        ~ObstaclePosWriter() = default; // destructor

        void save_json(const std::string path_to_result_folder, const std::string file_name, int random_seed);

    private:
        void write_obstacle_to_json(const MDP::ObjectObstacleFCL *obstacle);
        void write_robot_obstacle_to_json(MDP::RobotObstacleFCL &robot_obstacle);
        void write_plannable_robot_to_json();


        void add_string_to_json(rapidjson::Value &json, const std::string &name, const std::string &value);
        void write_obstacle_position_to_json(rapidjson::Value &json, const MDP::ObstacleCoordinate &position);
        void parse_obstacle_position_to_json(rapidjson::Value &json, const MDP::ObjectObstacleFCL *obstacle);
         
        ObstaclePosWriter(const ObstaclePosWriter &other) = delete;            // copy constructor
        ObstaclePosWriter(ObstaclePosWriter &&other) = delete;                 // move constructor
        ObstaclePosWriter &operator=(const ObstaclePosWriter &other) = delete; // copy assignment
        ObstaclePosWriter &operator=(ObstaclePosWriter &&other) = delete;      // move assignment

        CollisionManager collision_manager;

        rapidjson::Document data_to_export;
        rapidjson::Value data_to_export_value;
        rapidjson::Document::AllocatorType allocator;


    };

}
