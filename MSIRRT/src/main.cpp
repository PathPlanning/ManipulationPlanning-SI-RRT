//  ./MSIRRT/build/   ./STRRT/scene_task.json ./ ./STRRT/strrt_config.json
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
#include "config_read_writer/RobotObstacleJsonInfo.hpp"
#include "config_read_writer/ObstaclePosWriter.hpp"
#include "CollisionManager/CollisionManager.hpp"
#include "PlannerConnect.hpp"

void parse_my_args(int argc, char **argv, std::string &path_to_scene_json, std::string &path_to_result_folder, std::string &path_to_strrt_config_json, int &random_seed)
{
    if (argc < 5)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_scene_json> <path_to_result_folder> <path_to_strrt_config_json> <random_seed>" << std::endl;
        exit(EXIT_FAILURE);
    }

    path_to_scene_json = argv[1];
    path_to_result_folder = argv[2];
    path_to_strrt_config_json = argv[3];
    random_seed = atoi(argv[4]);
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
    int random_seed;
    parse_my_args(argc, argv, path_to_scene_json, path_to_result_folder, path_to_strrt_config_json, random_seed);
    check_args(path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);
    MDP::ResultsWriter::get_instance().setup(path_to_scene_json, path_to_strrt_config_json, path_to_result_folder, MDP::ResultsWriter::PlannerType::MSIRRT);
    MDP::ResultsWriter::get_instance().algorithm_start();
    MDP::ConfigReader SceneTask(path_to_scene_json);

    std::srand(random_seed);

    MDP::STRRTConfigReader STRRTConfigReader(path_to_strrt_config_json);
    // MDP::ObstaclePosWriter obs_pos_writer(SceneTask.get_scene_task());
    // obs_pos_writer.save_json(path_to_result_folder,path_to_scene_json.substr(path_to_scene_json.find_last_of("/\\") + 1),random_seed);

    // TODO: optimise collision manager construction and dont construct in space validity checker, or find another way to get joint limits
    std::shared_ptr<MDP::CollisionManager> collision_manager = std::make_shared<MDP::CollisionManager>(SceneTask.get_scene_task());
    std::vector<std::pair<float, float>> joint_limits = collision_manager->get_planned_robot_limits();

    MDP::MSIRRT::PlannerConnect plannerConnect(SceneTask.get_scene_task(), random_seed);

    MDP::ResultsWriter::get_instance().config_end();

    bool result = plannerConnect.solve();

    MDP::ResultsWriter::get_instance().solver_end();
    rapidjson::Value path_result;
    path_result.SetObject();
    path_result.AddMember("has_result", result, MDP::ResultsWriter::get_instance().get_json_allocator());
    path_result.AddMember("start_tree_vertices", plannerConnect.get_start_tree_vertices_count(), MDP::ResultsWriter::get_instance().get_json_allocator());
    path_result.AddMember("goal_tree_vertices", plannerConnect.get_goal_tree_vertices_count(), MDP::ResultsWriter::get_instance().get_json_allocator());

    if (result)
    {
        std::vector<MDP::MSIRRT::Vertex *> path = plannerConnect.get_final_path();
        assert(plannerConnect.check_path(path));

        std::vector<MDP::RobotObstacleJsonInfo::PathState> result_path;
        MDP::MSIRRT::Vertex *previous_state = nullptr;
        for (MDP::MSIRRT::Vertex *state : path)
        {
            if (previous_state)
            {
                if (state->parent)
                {
                    if (state->departure_from_parent_time != state->parent->arrival_time)
                    {
                        std::vector<double> point(SceneTask.get_scene_task().robot_joint_count);
                        for (int x = 0; x < SceneTask.get_scene_task().robot_joint_count; ++x)
                        {
                            point[x] = previous_state->coords[x];
                        }
                        result_path.emplace_back(point, ((double)state->departure_from_parent_time) / (double)SceneTask.get_scene_task().fps+SceneTask.get_scene_task().start_time);
                    }
                }
            }

            std::vector<double> point(SceneTask.get_scene_task().robot_joint_count);
            for (int x = 0; x < SceneTask.get_scene_task().robot_joint_count; ++x)
            {
                point[x] = state->coords[x];
            }
            result_path.emplace_back(point, ((double)state->arrival_time) / (double)SceneTask.get_scene_task().fps +SceneTask.get_scene_task().start_time);

            previous_state = state;
        }
        rapidjson::Value path_json;
        double path_cost = result_path.back().time-SceneTask.get_scene_task().start_time;
        MDP::ResultsWriter::get_instance().convert_path_to_json(result_path, path_json);
        path_result.AddMember("final_path", path_json, MDP::ResultsWriter::get_instance().get_json_allocator());
        path_result.AddMember("path_cost", path_cost, MDP::ResultsWriter::get_instance().get_json_allocator());
    }
    MDP::ResultsWriter::get_instance().save_json(std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())), path_result, random_seed);

    return 0;
}