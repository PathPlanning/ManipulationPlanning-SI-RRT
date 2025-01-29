#include <config_read_writer/STRRTConfigReader.hpp>
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

MDP::STRRTConfigReader::STRRTConfig strrt_json_parser(const std::string &path_to_strrt_config_json)
{
    std::ifstream strrt_config_file(path_to_strrt_config_json);

    if (!strrt_config_file.is_open())
    {
        throw std::runtime_error("Failed to open strrt config json file.");
    }

    rapidjson::IStreamWrapper strrt_config_wrapper(strrt_config_file);
    rapidjson::Document strrt_config_parsed_data;
    strrt_config_parsed_data.ParseStream(strrt_config_wrapper);

    double max_planning_time_pts = strrt_config_parsed_data["max_planning_time_pts"].GetDouble();
    double max_allowed_time_to_move = strrt_config_parsed_data["max_allowed_time_to_move"].GetDouble();
    bool show_GUI = strrt_config_parsed_data["show_GUI"].GetBool();
    double collision_check_interpolation_angle = strrt_config_parsed_data["collision_check_interpolation_angle"].GetDouble();
    double iteration_time_step_sec = strrt_config_parsed_data["iteration_time_step_sec"].GetDouble();
    bool stop_if_path_found = strrt_config_parsed_data["stop_if_path_found"].GetBool();
    double max_range = strrt_config_parsed_data["max_range"].GetDouble();
    double optimum_approx_factor = strrt_config_parsed_data["optimum_approx_factor"].GetDouble();
    std::string rewiring_method_string = strrt_config_parsed_data["rewiring_method"].GetString();
    MDP::STRRTConfigReader::RewireType rewiring_method;
    if (rewiring_method_string == "off")
    {
        rewiring_method = MDP::STRRTConfigReader::RewireType::OFF;
    }
    else if (rewiring_method_string == "radius")
    {
        rewiring_method = MDP::STRRTConfigReader::RewireType::RADIUS;
    }
    else if (rewiring_method_string == "k_neighbours")
    {
        rewiring_method = MDP::STRRTConfigReader::RewireType::K_NEIGHBOURS;
    }
    else
    {
        std::cerr << "Invalid rewiring method in strrt_config_json" << std::endl;
        assert(false);
    }

    double rewire_factor = strrt_config_parsed_data["rewire_factor"].GetDouble();
    double initial_batch_size = strrt_config_parsed_data["initial_batch_size"].GetDouble();
    double time_bound_factor_increase = strrt_config_parsed_data["time_bound_factor_increase"].GetDouble();
    double inital_time_bound_factor = strrt_config_parsed_data["inital_time_bound_factor"].GetDouble();
    bool sample_uniform_for_unbounded_time = strrt_config_parsed_data["sample_uniform_for_unbounded_time"].GetBool();
    bool ignore_low_bound_time = strrt_config_parsed_data["ignore_low_bound_time"].GetBool();
    return MDP::STRRTConfigReader::STRRTConfig(max_planning_time_pts,
                                               max_allowed_time_to_move,
                                               show_GUI,
                                               collision_check_interpolation_angle,
                                               iteration_time_step_sec,
                                               stop_if_path_found,
                                               max_range,
                                               optimum_approx_factor,
                                               rewiring_method,
                                               rewire_factor,
                                               initial_batch_size,
                                               time_bound_factor_increase,
                                               inital_time_bound_factor,
                                               sample_uniform_for_unbounded_time,
                                               ignore_low_bound_time);
}

const MDP::STRRTConfigReader::STRRTConfig &MDP::STRRTConfigReader::get_strrt_config() const
{
    return this->strrt_config;
}

MDP::STRRTConfigReader::STRRTConfigReader(const std::string &path_to_strrt_config_json) : strrt_config(::strrt_json_parser(path_to_strrt_config_json)) {};
