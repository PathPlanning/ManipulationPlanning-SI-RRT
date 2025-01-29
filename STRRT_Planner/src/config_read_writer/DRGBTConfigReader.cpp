#include "Config.hpp"
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <fstream>
#include <stdexcept>

namespace MDP {

DRGBTConfigReader::DRGBTConfigReader(const std::string& path_to_json)
    : config([&]() {
        std::ifstream config_file(path_to_json);
        if (!config_file.is_open()) {
            throw std::runtime_error("Failed to open config JSON file.");
        }

        rapidjson::IStreamWrapper config_wrapper(config_file);
        rapidjson::Document parsed_json;
        parsed_json.ParseStream(config_wrapper);

        return Config{
            parsed_json["number_of_tries"].GetInt(),
            parsed_json["ignore_low_bound_time"].GetBool(),

            // DRGBT Configuration
            {
                parsed_json["DRGBT_MAX_NUM_ITER"].GetInt(),
                parsed_json["DRGBT_MAX_ITER_TIME"].GetDouble(),
                parsed_json["DRGBT_MAX_PLANNING_TIME"].GetInt(),
                parsed_json["DRGBT_INIT_HORIZON_SIZE"].GetInt(),
                parsed_json["DRGBT_TRESHOLD_WEIGHT"].GetDouble(),
                parsed_json["DRGBT_D_CRIT"].GetDouble(),
                parsed_json["DRGBT_MAX_NUM_VALIDITY_CHECKS"].GetInt(),
                parsed_json["DRGBT_MAX_NUM_MODIFY_ATTEMPTS"].GetInt(),
                parsed_json["DRGBT_STATIC_PLANNER_TYPE"].GetString(),
                parsed_json["DRGBT_REAL_TIME_SCHEDULING"].GetBool(),
                parsed_json["DRGBT_MAX_TIME_TASK1"].GetInt(),
                parsed_json["DRGBT_USE_SPLINE"].GetBool(),
                parsed_json["DRGBT_GUARANTEED_SAFE_MOTION"].GetBool()
            },

            // RGBMT Configuration
            {
                parsed_json["RGBMT*_MAX_NUM_ITER"].GetInt(),
                parsed_json["RGBMT*_MAX_NUM_STATES"].GetInt(),
                parsed_json["RGBMT*_MAX_PLANNING_TIME"].GetInt(),
                parsed_json["RGBMT*_TERMINATE_WHEN_PATH_IS_FOUND"].GetBool()
            },

            // RGBTConnect Configuration
            {
                parsed_json["RGBTConnect_MAX_NUM_ITER"].GetInt(),
                parsed_json["RGBTConnect_MAX_NUM_STATES"].GetInt(),
                parsed_json["RGBTConnect_MAX_PLANNING_TIME"].GetInt(),
                parsed_json["RGBTConnect_NUM_LAYERS"].GetInt()
            },

            // RBTConnect Configuration
            {
                parsed_json["RBTConnect_MAX_NUM_ITER"].GetInt(),
                parsed_json["RBTConnect_MAX_NUM_STATES"].GetInt(),
                parsed_json["RBTConnect_MAX_PLANNING_TIME"].GetInt(),
                parsed_json["RBTConnect_D_CRIT"].GetDouble(),
                parsed_json["RBTConnect_DELTA"].GetInt(),
                parsed_json["RBTConnect_NUM_SPINES"].GetInt(),
                parsed_json["RBTConnect_NUM_ITER_SPINE"].GetInt(),
                parsed_json["RBTConnect_USE_EXPANDED_BUBBLE"].GetBool()
            },

            // RVS Configuration
            {
                parsed_json["RVS_EQUALITY_THRESHOLD"].GetDouble(),
                parsed_json["RVS_NUM_INTERPOLATION_VALIDITY_CHECKS"].GetInt()
            },

            // RRTConnect Configuration
            {
                parsed_json["RRTConnect_MAX_NUM_ITER"].GetInt(),
                parsed_json["RRTConnect_MAX_NUM_STATES"].GetInt(),
                parsed_json["RRTConnect_MAX_PLANNING_TIME"].GetInt(),
                parsed_json["RRTConnect_MAX_EXTENSION_STEPS"].GetInt(),
                parsed_json["RRTConnect_EPS_STEP"].GetDouble()
            }
        };
    }()) {}

const DRGBTConfigReader::Config& DRGBTConfigReader::get_config() const {
    return this->config;
}

}