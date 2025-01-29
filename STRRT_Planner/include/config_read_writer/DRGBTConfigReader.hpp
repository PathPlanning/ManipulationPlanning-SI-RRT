#pragma once

#include <string>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

namespace MDP {

class DRGBTConfigReader {
public:
    explicit DRGBTConfigReader(const std::string& path_to_json);
    ~DRGBTConfigReader() = default;

    // Struct to hold DRGBT configuration data
    struct DRGBTConfig {
        const int max_num_iter;
        const double max_iter_time;
        const int max_planning_time;
        const int init_horizon_size;
        const double treshold_weight;
        const double d_crit;
        const int max_num_validity_checks;
        const int max_num_modify_attempts;
        const std::string static_planner_type;
        const bool real_time_scheduling;
        const int max_time_task1;
        const bool use_spline;
        const bool guaranteed_safe_motion;
    };

    // Struct to hold RGBMT configuration data
    struct RGBMTConfig {
        const int max_num_iter;
        const int max_num_states;
        const int max_planning_time;
        const bool terminate_when_path_is_found;
    };

    // Struct to hold RGBTConnect configuration data
    struct RGBTConnectConfig {
        const int max_num_iter;
        const int max_num_states;
        const int max_planning_time;
        const int num_layers;
    };

    // Struct to hold RBTConnect configuration data
    struct RBTConnectConfig {
        const int max_num_iter;
        const int max_num_states;
        const int max_planning_time;
        const double d_crit;
        const int delta;
        const int num_spines;
        const int num_iter_spine;
        const bool use_expanded_bubble;
    };

    // Struct to hold RVS configuration data
    struct RVSConfig {
        const double equality_threshold;
        const int num_interpolation_validity_checks;
    };

    // Struct to hold RRTConnect configuration data
    struct RRTConnectConfig {
        const int max_num_iter;
        const int max_num_states;
        const int max_planning_time;
        const int max_extension_steps;
        const double eps_step;
    };

    // Struct to hold the entire configuration data
    struct Config {
        const int number_of_tries;
        const bool ignore_low_bound_time;

        const DRGBTConfig drgbt;
        const RGBMTConfig rgbmt;
        const RGBTConnectConfig rgbt_connect;
        const RBTConnectConfig rbt_connect;
        const RVSConfig rvs;
        const RRTConnectConfig rrt_connect;
    };

    const Config& get_config() const;

private:
    const Config config;

    // Disable copy and move operations
    DRGBTConfigReader(const DRGBTConfigReader&) = delete;
    DRGBTConfigReader(DRGBTConfigReader&&) = delete;
    DRGBTConfigReader& operator=(const DRGBTConfigReader&) = delete;
    DRGBTConfigReader& operator=(DRGBTConfigReader&&) = delete;
};

}