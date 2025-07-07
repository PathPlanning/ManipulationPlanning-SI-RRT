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
#include <map>
namespace MDP
{
    // singletone class for results collection
    class ResultsWriter
    {
        /*
        stores benchmark and result data and saves it in json
        */
    public:
        enum class PlannerType
        {
            STRRT_STAR,
            DRGBT,
            MSIRRT
        };
        class Timer
        {

        public:
            Timer();                                       // constructor
            Timer(const std::string _name);                // constructor
            ~Timer() = default;                            // destructor
            Timer(const Timer &other) = default;           // copy constructor
            Timer(Timer &&other) = delete;                 // move constructor
            Timer &operator=(const Timer &other) = delete; // copy assignment
            Timer &operator=(Timer &&other) = delete;      // move assignment

            void reset();          // reset timer, set counted time to zero and start counting again;
            void set_to_zero();    // set counted time to zero without starting counting again;
            void pause_timer();    // stop timer, save elapsed time to counter
            void continue_timer(); // start paused timer
            bool get_is_running() const;
            uint64_t get_elapsed_time_ns(); // get the time, that the timer was running
            std::string get_name() const;

        private:
            uint64_t time_in_ns;
            std::chrono::time_point<std::chrono::steady_clock> start_time;
            std::chrono::nanoseconds duration_ns;
            bool is_running;
            std::string name;
        };

        struct PathState
        {
            const std::vector<double> configuration_coordinates;
            const double time;
            PathState(const std::vector<double> _configuration_coordinates, const double _time) : configuration_coordinates(_configuration_coordinates), time(_time) {};
        };

        struct PlannerResults
        {
            const std::vector<PathState> path; // planned path
            const double cost;                 // cost of the path
            rapidjson::Value planner_metadata; // metadata of planner to add to JSON (tree, node count, etc.)
            // NOTE!!! rapidjson::Value does not have a copy constructor, only move constructor!!!
            const uint64_t timestamp_ns;                                                                                                                                                                                                                                     // timestamp of obtaining results, timer startsright before the planner->solve() call
                                                                                                                                                                                                                                                                             // in other word, it is the time from planner->solve() call till getting path result.
            PlannerResults(const std::vector<PathState> _path, const double _cost, rapidjson::Value &&_planner_metadata, const unsigned long long _timestamp_ns) : path(_path), cost(_cost), planner_metadata(std::move(_planner_metadata)), timestamp_ns(_timestamp_ns) {}; // TODO^ probably don't need std move here...
            PlannerResults(const PlannerResults &other) = delete;                                                                                                                                                                                                            // copy constructor
            PlannerResults(PlannerResults &&other) = default;                                                                                                                                                                                                                // move constructor
            PlannerResults &operator=(const PlannerResults &other) = delete;                                                                                                                                                                                                 // copy assignment
            PlannerResults &operator=(PlannerResults &&other) = default;                                                                                                                                                                                                     // move assignment
        };

        static ResultsWriter &get_instance()
        {
            static ResultsWriter instance;
            return instance;
        };

        ~ResultsWriter() = default; // destructor

        void setup(const std::string &_path_to_scene_json, const std::string &_path_to_strrt_config_json, const std::string &_path_to_result_folder, const PlannerType _planner_type);

        void algorithm_start();
        void config_end();
        void solver_end();

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> timer_function_wrapper(Timer &timer, F &&fn, Args &&...args);
        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> timer_counter_function_wrapper(Timer &timer, uint64_t &counter, F &&fn, Args &&...args);

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> collision_check_wrapper(F &&fn, Args &&...args);

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> safe_interval_wrapper(F &&fn, Args &&...args);

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> distance_check_wrapper(F &&fn, Args &&...args);

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> forward_kinematics_collision_check_wrapper(F &&fn, Args &&...args);

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> forward_kinematics_distance_check_wrapper(F &&fn, Args &&...args);

        template <typename F, typename... Args>
        std::result_of_t<F && (Args && ...)> forward_kinematics_check_wrapper(F &&fn, Args &&...args);

        void save_json(const std::string file_name, rapidjson::Value &additional_final_planner_metadata, int random_seed);

        // add intermediate or final path planning result.
        // path - vector of MDP::ResultsWriter::PathState
        // double cost - path cost (if planner is optimal)

        // NOTE!!! rapidjson::Value does not have copy constructor, only move constructor!!!
        void add_results(std::vector<MDP::ResultsWriter::PathState> path, double cost, rapidjson::Value &&planner_metadata);

        uint64_t get_collision_check_count() const;
        rapidjson::Document::AllocatorType &get_json_allocator();

        rapidjson::Value &convert_path_to_json(const std::vector<PathState> &path, rapidjson::Value &path_json);

        void save_error_json(const std::string &error_msg);

        uint64_t collision_check_broadphase_counter = 0;
        uint64_t collision_check_narrowphase_counter = 0;
        uint64_t safe_interval_broadphase_counter = 0;
        uint64_t safe_interval_narrowphase_counter = 0;

        uint64_t collision_count = 0;
        uint64_t number_of_safe_intervals =0;
        uint64_t sum_of_safe_interval_frames = 0;
        Timer collision_check_init_time{"collision_check_init_time"};                                                       // time spend by initialising and build aabb trees for collision check
        Timer safe_intervals_init_time{"safe_intervals_init_time"};                                                         // time spend by initialising and build aabb trees for safe_intervals

        void restart_collision_checker();
    private:
        ResultsWriter() = default;                                     // constructor
        ResultsWriter(const ResultsWriter &other) = delete;            // copy constructor
        ResultsWriter(ResultsWriter &&other) = delete;                 // move constructor
        ResultsWriter &operator=(const ResultsWriter &other) = delete; // copy assignment
        ResultsWriter &operator=(ResultsWriter &&other) = delete;      // move assignment

        bool has_been_settep_up = false;
        std::string path_to_scene_json;
        std::string path_to_strrt_config_json;
        std::string path_to_result_folder;
        PlannerType planner_type;

        Timer algorithm_full_time{"algorithm_full_time"};                                                                   // time from algorithm start to save_json() call
        Timer algorithm_config_time{"algorithm_config_time"};                                                               // time from algorithm start to start of planner solving
        Timer algorithm_collision_check_time{"algorithm_collision_check_time"};                                             // time in collision_check function
        Timer algorithm_safe_interval_time{"algorithm_safe_interval_time"};                                                 // time in geT_safe_intevals function
        Timer algorithm_distance_check_time{"algorithm_distance_check_time"};                                               // time in get distance function
        Timer algorithm_forward_kinematics_in_collision_check_time{"algorithm_forward_kinematics_in_collision_check_time"}; // time in solving forward kinematics when checking collision
        Timer algorithm_forward_kinematics_in_distance_check_time{"algorithm_forward_kinematics_in_distance_check_time"};   // time in solving forward kinematics
        Timer algorithm_forward_kinematics_check_time{"algorithm_forward_kinematics_check_time"};                           // time in solving forward kinematics
        Timer algorithm_solving_time{"algorithm_solving_time"};                                                             // time spend by planner->solve function

        uint64_t collision_check_counter = 0;
        uint64_t safe_interval_counter = 0;
        uint64_t distance_check_counter = 0;
        uint64_t forward_kinematics_check_counter = 0;
        
        std::vector<PlannerResults> results_path_cost_metadata;

        rapidjson::Document data_to_export;
        rapidjson::Document::AllocatorType allocator;
    };

}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::timer_function_wrapper(MDP::ResultsWriter::Timer &timer, F &&fn, Args &&...args)
{

    timer.continue_timer();
    std::result_of_t<F && (Args && ...)> result = fn(std::forward<Args>(args)...);
    timer.pause_timer();
    return result;
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::timer_counter_function_wrapper(MDP::ResultsWriter::Timer &timer, uint64_t &counter, F &&fn, Args &&...args)
{
    counter++;
    return this->timer_function_wrapper(timer, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::collision_check_wrapper(F &&fn, Args &&...args)
{
    return this->timer_counter_function_wrapper(this->algorithm_collision_check_time, this->collision_check_counter, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::safe_interval_wrapper(F &&fn, Args &&...args)
{
    return this->timer_counter_function_wrapper(this->algorithm_safe_interval_time, this->safe_interval_counter, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::distance_check_wrapper(F &&fn, Args &&...args)
{
    return this->timer_counter_function_wrapper(this->algorithm_distance_check_time, this->distance_check_counter, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::forward_kinematics_collision_check_wrapper(F &&fn, Args &&...args)
{
    return this->timer_function_wrapper(this->algorithm_forward_kinematics_in_collision_check_time, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::forward_kinematics_distance_check_wrapper(F &&fn, Args &&...args)
{
    return this->timer_function_wrapper(this->algorithm_forward_kinematics_in_distance_check_time, std::forward<F>(fn), std::forward<Args>(args)...);
}

template <typename F, typename... Args>
std::result_of_t<F && (Args && ...)> MDP::ResultsWriter::forward_kinematics_check_wrapper(F &&fn, Args &&...args)
{
    return this->timer_counter_function_wrapper(this->algorithm_forward_kinematics_check_time, this->forward_kinematics_check_counter, std::forward<F>(fn), std::forward<Args>(args)...);
}

namespace{
    std::string generateRandomString(int n);
}