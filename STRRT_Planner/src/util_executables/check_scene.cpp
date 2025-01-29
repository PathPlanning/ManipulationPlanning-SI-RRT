//  this programm check scene, if it is valid (valid start and goal poses, no collision at the goal at last frame, no obstacles cross manipulator's base)
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
#include "config_read_writer/config_read.hpp"
#include <CollisionManager/CollisionManager.hpp>
#include "config_read_writer/ResultsWriter.hpp"

void parse_my_args(int argc, char **argv, std::string &path_to_scene_json)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_scene_json>" << std::endl;
        exit(EXIT_FAILURE);
    }
    path_to_scene_json = argv[1];
}

void check_args(const std::string &path_to_scene_json)
{
    if (!std::ifstream(path_to_scene_json))
    {
        throw std::runtime_error("Invalid file or directory path");
    }
}

int main(int argc, char **argv)
{
    std::string path_to_scene_json;

    parse_my_args(argc, argv, path_to_scene_json);
    check_args(path_to_scene_json);
    MDP::ConfigReader SceneTask(path_to_scene_json);
    MDP::ResultsWriter::get_instance().setup(path_to_scene_json, "", "", MDP::ResultsWriter::PlannerType::STRRT_STAR);
    MDP::ResultsWriter::get_instance().algorithm_start();

    // TODO: optimise collision manager construction and dont construct in space validity checker, or find another way to get joint limits
    std::shared_ptr<MDP::CollisionManager> collision_manager = std::make_shared<MDP::CollisionManager>(SceneTask.get_scene_task());
    std::vector<std::pair<float, float>> joint_limits = collision_manager->get_planned_robot_limits();

    for (int joint_ind = 0; joint_ind < SceneTask.get_scene_task().robot_joint_count; joint_ind++)
    {
        if (joint_limits[joint_ind].first == joint_limits[joint_ind].second)
        {
            std::cout << "False" << std::endl;
            std::cout << "invalid urdf joint limits" << std::endl;
            return EXIT_FAILURE;
        }
        if ((joint_limits[joint_ind].first > SceneTask.get_scene_task().start_configuration[joint_ind]) || (joint_limits[joint_ind].second < SceneTask.get_scene_task().start_configuration[joint_ind]))
        {
            std::cout << "False" << std::endl;
            std::cout << "start pose is out of bounds!" << std::endl;
            return EXIT_FAILURE;
        }

        if ((joint_limits[joint_ind].first > SceneTask.get_scene_task().end_configuration[joint_ind]) || (joint_limits[joint_ind].second < SceneTask.get_scene_task().end_configuration[joint_ind]))
        {
            std::cout << "False" << std::endl;
            std::cout << "start pose is out of bounds!" << std::endl;
            return EXIT_FAILURE;
        }
    }
    std::vector<double> start_conf = SceneTask.get_scene_task().start_configuration;
    if (collision_manager->check_collision_frame_no_wrapper(start_conf, 0))
    {
        std::cout << "False" << std::endl;
        std::cout << "start pose is in collision at frame 0!" << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<double> end_conf = SceneTask.get_scene_task().end_configuration;

    if (collision_manager->check_collision_frame_no_wrapper(end_conf, SceneTask.get_scene_task().frame_count - 1))
    {
        std::cout << "False" << std::endl;
        std::cout << "goal pose is in collision at last frame!" << std::endl;
        return EXIT_FAILURE;
    }

    // int low_bound_frame = collision_manager->get_goal_frame_low_bound();
    // if (low_bound_frame != 0)
    // {
    //     std::cout << "False" << std::endl;
    //     std::cout << "goal pose is in collision at  frame " << low_bound_frame << std::endl;
    //     return EXIT_FAILURE;
    // }

    // if(collision_manager->check_base_joints_collisiion(start_conf,2)){
    //     std::cout << "False" << std::endl;
    //     std::cout << "detected collision at base joints!" << std::endl;
    //     return EXIT_FAILURE;
    // }
    std::cout << "True" << std::endl;
    return 0;
}