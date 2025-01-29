//  ./STRRT_Planner   ../../STRRT/scene_task.json ./ ../../STRRT/strrt_config.json
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
#include "config_read_writer/STRRTConfigReader.hpp"
#include "config_read_writer/ResultsWriter.hpp"
#include "CollisionManager/CollisionManager.hpp"
#include "Planner.hpp"

void parse_my_args(int argc, char **argv, std::string &path_to_scene_json, std::string &path_to_result_folder, std::string &path_to_strrt_config_json)
{
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_scene_json> <path_to_result_folder> <path_to_strrt_config_json>" << std::endl;
        exit(EXIT_FAILURE);
    }

    path_to_scene_json = argv[1];
    path_to_result_folder = argv[2];
    path_to_strrt_config_json = argv[3];
}

void check_args(const std::string &path_to_scene_json, const std::string &path_to_result_folder, const std::string &path_to_strrt_config_json)
{
    if (!std::ifstream(path_to_scene_json) || !std::ifstream(path_to_strrt_config_json) || !std::filesystem::is_directory(path_to_result_folder))
    {
        throw std::runtime_error("Invalid file or directory path");
    }
}

int main(int argc, char **argv)
{
    std::string path_to_scene_json;
    std::string path_to_result_folder;
    std::string path_to_strrt_config_json;
    parse_my_args(argc, argv, path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);
    check_args(path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);
    MDP::ResultsWriter::get_instance().setup(path_to_scene_json, path_to_strrt_config_json, path_to_result_folder, MDP::ResultsWriter::PlannerType::MSIRRT);
    MDP::ResultsWriter::get_instance().algorithm_start();
    MDP::ConfigReader SceneTask(path_to_scene_json);

    MDP::STRRTConfigReader STRRTConfigReader(path_to_strrt_config_json);

    // TODO: optimise collision manager construction and dont construct in space validity checker, or find another way to get joint limits
    std::shared_ptr<MDP::CollisionManager> collision_manager = std::make_shared<MDP::CollisionManager>(SceneTask.get_scene_task());
    
    collision_manager->benchmark_broadphase();
    return 0;
}