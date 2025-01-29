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
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/RobotObstacleJsonInfo.hpp>


namespace MDP{
class STRRTConfigReader{
    public:
        /*
        Reads the json and creates STRRTConfig struct
        */
        explicit STRRTConfigReader(const std::string& path_to_strrt_config_json);//constructor
        
        ~STRRTConfigReader() = default;//destructor
        
        enum class RewireType{
            OFF,
            RADIUS,
            K_NEIGHBOURS
                    };

        struct STRRTConfig{
            const double max_planning_time_pts;
            const double max_allowed_time_to_move;
            const bool show_GUI;
            const double collision_check_interpolation_angle;
            const double iteration_time_step_sec;
            const bool stop_if_path_found;
            const double max_range;
            const double optimum_approx_factor;
            const MDP::STRRTConfigReader::RewireType rewiring_method;
            const double rewire_factor;
            const double initial_batch_size;
            const double time_bound_factor_increase;
            const double inital_time_bound_factor;
            const bool sample_uniform_for_unbounded_time;
            const bool ignore_low_bound_time;
            STRRTConfig( double _max_planning_time_pts,
             double _max_allowed_time_to_move,
             bool _show_GUI,
             double _collision_check_interpolation_angle,
             double _iteration_time_step_sec,
             bool _stop_if_path_found,
             double _max_range,
             double _optimum_approx_factor,
             MDP::STRRTConfigReader::RewireType _rewiring_method,
             double _rewire_factor,
             double _initial_batch_size,
             double _time_bound_factor_increase,
             double _inital_time_bound_factor,
             bool _sample_uniform_for_unbounded_time,
             bool _ignore_low_bound_time):
                       max_planning_time_pts(_max_planning_time_pts),
                       max_allowed_time_to_move(_max_allowed_time_to_move), 
                       show_GUI(_show_GUI),
                       collision_check_interpolation_angle(_collision_check_interpolation_angle),
                       iteration_time_step_sec(_iteration_time_step_sec),
                       stop_if_path_found(_stop_if_path_found),
                       max_range(_max_range),
                       optimum_approx_factor(_optimum_approx_factor),
                       rewiring_method(_rewiring_method),
                       rewire_factor(_rewire_factor),
                       initial_batch_size(_initial_batch_size),
                       time_bound_factor_increase(_time_bound_factor_increase),
                       inital_time_bound_factor(_inital_time_bound_factor),
                       sample_uniform_for_unbounded_time(_sample_uniform_for_unbounded_time),
                       ignore_low_bound_time(_ignore_low_bound_time){
                       };

            ~STRRTConfig() = default;//destructor
            STRRTConfig(const STRRTConfig& other) = default;// copy constructor
            STRRTConfig(STRRTConfig&& other) = default;// move constructor
            STRRTConfig& operator=(const STRRTConfig& other) = default; // copy assignment
            STRRTConfig& operator=(STRRTConfig&& other) = default; // move assignment

        };

        const STRRTConfig& get_strrt_config() const;
       

    private:
        STRRTConfigReader(const STRRTConfigReader& other) = delete;// copy constructor
        STRRTConfigReader(STRRTConfigReader&& other) = delete;// move constructor
        STRRTConfigReader& operator=(const STRRTConfigReader& other) = delete; // copy assignment
        STRRTConfigReader& operator=(STRRTConfigReader&& other) = delete; // move assignment
        const STRRTConfig strrt_config;
};

}

namespace{
    MDP::STRRTConfigReader::STRRTConfig strrt_json_parser(const std::string &path_to_strrt_config_json);
}
