#pragma once

#include <vector>
#include "config_read_writer/config_read.hpp"
#include "CollisionManager/CollisionManager.hpp"
#include "Tree.hpp"
#include "Vertex.hpp"
#include <random>
#include <chrono>
namespace MDP::MSIRRT
{

    class Planner
    {
    public:
        Planner(MDP::ConfigReader::SceneTask scene_task_);
        ~Planner();                                        // destructor
        Planner(const Planner &other) = delete;            // copy constructor
        Planner(Planner &&other) = delete;                 // move constructor
        Planner &operator=(const Planner &other) = delete; // copy assignment
        Planner &operator=(Planner &&other) = delete;      // move assignment

        bool solve();
        std::vector<MDP::MSIRRT::Vertex *> get_final_path() const;

    private:
        MDP::ConfigReader::SceneTask scene_task;
        MDP::CollisionManager collision_manager;
        bool check_planner_termination_condition() const;

        bool is_coords_in_limits(const MDP::MSIRRT::Vertex &q) const;
        bool is_coords_in_limits(const MDP::MSIRRT::Vertex::VertexCoordType &coords) const;

        bool is_goal(const MDP::MSIRRT::Vertex::VertexCoordType& coord);
        double get_random_between_0_1();
        MDP::MSIRRT::Vertex::VertexCoordType get_random_configuration();

        MDP::MSIRRT::Vertex *get_nearest_node(const MDP::MSIRRT::Vertex::VertexCoordType& coords);
        std::vector<std::pair<MDP::MSIRRT::Vertex *,int> > get_nearest_node_by_radius(const MDP::MSIRRT::Vertex::VertexCoordType& coords, double raduis,MDP::MSIRRT::Tree* tree);

        bool extend(MDP::MSIRRT::Vertex::VertexCoordType &coords_of_new);
        std::vector<MDP::MSIRRT::Vertex *> set_parent(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand);
        std::vector<MDP::MSIRRT::Vertex *> rewire(MDP::MSIRRT::Vertex *node);
        std::pair<int, int> calculate_delta(MDP::MSIRRT::Vertex *candidate_node, MDP::MSIRRT::Vertex::VertexCoordType &end_coords, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand, int &safe_interval_ind);
        bool is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType& start_coords, const  MDP::MSIRRT::Vertex::VertexCoordType& end_coords, double& start_time, double& end_time);
        bool is_collision_state(MDP::MSIRRT::Vertex::VertexCoordType& coords, int& time);

        MDP::MSIRRT::Tree start_tree, orphan_tree;
        MDP::MSIRRT::Vertex *root_node = nullptr;

        MDP::MSIRRT::Vertex::VertexCoordType goal_coords;
        float max_planning_time = 180;
        bool stop_when_path_found = true;
        std::uniform_real_distribution<> probability_gen;
        int dof = -1;
        double goal_bias = 0.4;
        double vmax = 3.14;
        double planner_range;
        bool goal_reached = false;
        MDP::MSIRRT::Vertex *goal_node = nullptr;
        std::vector<std::pair<float, float>> robot_limits;
        std::chrono::time_point<std::chrono::steady_clock> solver_start_time;
        std::random_device rd; // Will be used to obtain a seed for the random number engine
        std::mt19937 gen;
        bool goal_sampled;
        std::vector<std::pair<int, int>> goal_safe_intervals;
    };
}
