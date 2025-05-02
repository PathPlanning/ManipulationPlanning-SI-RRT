#include <config_read_writer/ResultsWriter.hpp>
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
#include <algorithm>
void MDP::ResultsWriter::setup(const std::string &_path_to_scene_json, const std::string &_path_to_strrt_config_json, const std::string &_path_to_result_folder, const PlannerType _planner_type)
{
    this->path_to_scene_json = _path_to_scene_json;
    this->path_to_result_folder = _path_to_result_folder;
    this->planner_type = _planner_type;
    this->path_to_strrt_config_json = _path_to_strrt_config_json;
    this->has_been_settep_up = true;

    this->data_to_export.SetObject();
    this->allocator = data_to_export.GetAllocator();
}

void MDP::ResultsWriter::algorithm_start()
{
    assert(this->has_been_settep_up);
    this->algorithm_full_time.reset();
    this->algorithm_config_time.reset();
}

void MDP::ResultsWriter::config_end()
{
    assert(this->has_been_settep_up);
    assert(this->algorithm_full_time.get_is_running());
    this->algorithm_config_time.pause_timer();
    this->algorithm_solving_time.reset();
}
void MDP::ResultsWriter::solver_end()
{
    assert(this->has_been_settep_up);
    assert(this->algorithm_full_time.get_is_running());
    assert(!this->algorithm_config_time.get_is_running());
    algorithm_solving_time.pause_timer();
}

void MDP::ResultsWriter::safe_interval_construction_start(){
    this->collision_check_safe_interval_caching_time.reset();

}
void MDP::ResultsWriter::safe_interval_construction_end(){
    assert(this->collision_check_safe_interval_caching_time.get_is_running());
    this->collision_check_safe_interval_caching_time.pause_timer();

}


// NOTE!!! rapidjson::Value does not have copy constructor, only move constructor!!!
void MDP::ResultsWriter::save_json(const std::string file_name, rapidjson::Value &additional_final_planner_metadata, int random_seed)
{

    std::string planner_type_str = "undefined"; // TODO^ optimise enum
    if (this->planner_type == MDP::ResultsWriter::PlannerType::STRRT_STAR)
    {
        planner_type_str = "STRRT*";
    }
    else if (this->planner_type == MDP::ResultsWriter::PlannerType::DRGBT)
    {
        planner_type_str = "DRGBT";
    }
    else if (this->planner_type == MDP::ResultsWriter::PlannerType::MSIRRT)
    {
        planner_type_str = "MSIRRT";
    }

    data_to_export.AddMember("planner_type", rapidjson::StringRef(planner_type_str.c_str()), allocator);
    data_to_export.AddMember("path_to_scene_json", rapidjson::StringRef(this->path_to_scene_json.c_str()), allocator);
    data_to_export.AddMember("path_to_config_json", rapidjson::StringRef(this->path_to_strrt_config_json.c_str()), allocator);
    data_to_export.AddMember("full_execution_time_ns", this->algorithm_full_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("planner_config_time_ns", this->algorithm_config_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("algorithm_solving_time_ns", this->algorithm_solving_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("collision_check_time_ns", this->algorithm_collision_check_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("distance_check_time_ns", this->algorithm_distance_check_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("forward_kinematics_in_collision_check_time_ns", this->algorithm_forward_kinematics_in_collision_check_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("forward_kinematics_in_distance_check_time_ns", this->algorithm_forward_kinematics_in_distance_check_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("total_forward_kinematics_time_ns", this->algorithm_forward_kinematics_check_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("collision_check_safe_interval_caching_time_ns", this->collision_check_safe_interval_caching_time.get_elapsed_time_ns(), allocator);
    data_to_export.AddMember("number_of_collision_checks", this->collision_check_counter, allocator);
    // data_to_export.AddMember("number_of_broadphase_collision_checks", 0, allocator); // TODO Replace with actual
    // data_to_export.AddMember("number_of_narrowphase_collision_checks", 0, allocator); // TODO Replace with actual
    data_to_export.AddMember("number_of_distances_checks", this->distance_check_counter, allocator);
    data_to_export.AddMember("number_of_forward_kinematics", this->forward_kinematics_check_counter, allocator);

    data_to_export.AddMember("final_planner_data", additional_final_planner_metadata, allocator);

    rapidjson::Value path_intermediate_results(rapidjson::kArrayType);

    for (PlannerResults &intermediate_result : results_path_cost_metadata)
    {
        rapidjson::Value path_result;
        path_result.SetObject();

        rapidjson::Value path_json;
        this->convert_path_to_json(intermediate_result.path, path_json);
        path_result.AddMember("path", path_json, allocator);
        path_result.AddMember("cost", intermediate_result.cost, allocator);
        path_result.AddMember("time_of_obtaining_result_from_solve_start_ns", intermediate_result.timestamp_ns, allocator);
        path_result.AddMember("additional_metadata", intermediate_result.planner_metadata, allocator);
        path_intermediate_results.PushBack(path_result, allocator);
    }

    data_to_export.AddMember("path_results", path_intermediate_results, allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    data_to_export.Accept(writer);
    std::string filename = this->path_to_result_folder + "/" + planner_type_str + "_planner_logs_" + file_name + "_" + std::to_string(random_seed) + ".json";

    std::ofstream result_file(filename);
    result_file << buffer.GetString();
    result_file.close();
}
// https://stackoverflow.com/questions/2146792/how-do-you-generate-random-strings-in-c
namespace
{
    char get_rand_char()
    {
        static std::string charset("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890");
        return charset[rand() % charset.size()];
    }

    std::string generate_random_string(size_t n)
    {
        char rbuf[n];
        std::generate(rbuf, rbuf + n, &get_rand_char);
        return std::string(rbuf, n);
    }
}

namespace
{
    std::string generateRandomString(int n) {
        std::string characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
        std::string randomString;
        
        for (int i = 0; i < n; i++) {
            randomString += characters[rand() % characters.length()];     
        }
        
        return randomString;
    }
}
void MDP::ResultsWriter::add_results(std::vector<MDP::ResultsWriter::PathState> path, double cost, rapidjson::Value &&planner_metadata)
{
    assert(this->algorithm_solving_time.get_is_running());
    this->results_path_cost_metadata.emplace_back(PlannerResults(path, cost, std::move(planner_metadata), this->algorithm_solving_time.get_elapsed_time_ns()));
}

uint64_t MDP::ResultsWriter::get_collision_check_count() const
{
    return this->collision_check_counter;
}

rapidjson::Document::AllocatorType &MDP::ResultsWriter::get_json_allocator()
{
    return this->allocator;
}

rapidjson::Value &MDP::ResultsWriter::convert_path_to_json(const std::vector<PathState> &path, rapidjson::Value &path_array)
{
    path_array.SetArray();
    for (const PathState &state : path)
    {
        rapidjson::Value point_array(rapidjson::kArrayType);
        for (double v : state.configuration_coordinates)
        {
            point_array.PushBack(v, this->get_json_allocator());
        }

        rapidjson::Value time_point_array;
        time_point_array.SetObject();

        time_point_array.AddMember("time", state.time, this->get_json_allocator());
        time_point_array.AddMember("robot_angles", point_array, this->get_json_allocator());

        path_array.PushBack(time_point_array, this->get_json_allocator());
    }
    return path_array;
}

void MDP::ResultsWriter::save_error_json(const std::string &error_msg)
{

    data_to_export.AddMember("error_msg", rapidjson::StringRef(error_msg.c_str()), this->allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    data_to_export.Accept(writer);

    std::ofstream result_file(this->path_to_result_folder + "/" + "planning_error.json");
    result_file << buffer.GetString();
    result_file.close();
}

MDP::ResultsWriter::Timer::Timer() : name("") // constructor
{
    this->is_running = false;
    this->duration_ns = std::chrono::nanoseconds::zero();
}

MDP::ResultsWriter::Timer::Timer(const std::string name_) : name(name_) // constructor
{
    this->is_running = false;
    this->duration_ns = std::chrono::nanoseconds::zero();
}

void MDP::ResultsWriter::Timer::reset() // reset timer
{
    this->is_running = true;
    this->duration_ns = std::chrono::nanoseconds::zero();
    this->start_time = std::chrono::steady_clock::now();
}

void MDP::ResultsWriter::Timer::pause_timer()
{
    if (this->is_running)
    {
        std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
        this->duration_ns += end_time - start_time;
        this->is_running = false;
    }
}

void MDP::ResultsWriter::Timer::continue_timer()
{
    if (!this->is_running)
    {
        this->start_time = std::chrono::steady_clock::now();
        this->is_running = true;
    }
}

uint64_t MDP::ResultsWriter::Timer::get_elapsed_time_ns()
{
    if (this->is_running)
    {
        std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(this->duration_ns + (current_time - this->start_time)).count();
    }
    else
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(this->duration_ns).count();
    }
}

bool MDP::ResultsWriter::Timer::get_is_running() const
{
    return this->is_running;
}

std::string MDP::ResultsWriter::Timer::get_name() const
{
    return this->name;
}
