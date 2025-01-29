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

    class PlannerConnect
    {
    public:
        PlannerConnect(MDP::ConfigReader::SceneTask scene_task_, int random_seed);
        ~PlannerConnect();                                               // destructor
        PlannerConnect(const PlannerConnect &other) = delete;            // copy constructor
        PlannerConnect(PlannerConnect &&other) = delete;                 // move constructor
        PlannerConnect &operator=(const PlannerConnect &other) = delete; // copy assignment
        PlannerConnect &operator=(PlannerConnect &&other) = delete;      // move assignment

        bool solve();
        std::vector<MDP::MSIRRT::Vertex*> get_final_path() const;
        bool check_path(std::vector<MDP::MSIRRT::Vertex*>& path);

    private:
        MDP::ConfigReader::SceneTask scene_task;
        MDP::CollisionManager collision_manager;
        bool check_planner_termination_condition() const;

        bool is_coords_in_limits(const MDP::MSIRRT::Vertex &q) const;
        bool is_coords_in_limits(const MDP::MSIRRT::Vertex::VertexCoordType &coords) const;

        bool is_goal(const MDP::MSIRRT::Vertex::VertexCoordType &coord);
        double get_random_between_0_1();
        MDP::MSIRRT::Vertex::VertexCoordType get_random_configuration();
        std::vector<MDP::MSIRRT::Vertex *> extend_naive(MDP::MSIRRT::Vertex::VertexCoordType &coords_of_new);

        MDP::MSIRRT::Vertex *get_nearest_node(const MDP::MSIRRT::Vertex::VertexCoordType &coords);
        std::vector<std::pair<MDP::MSIRRT::Vertex *, int>> get_nearest_node_by_radius(const MDP::MSIRRT::Vertex::VertexCoordType &coords, double raduis, MDP::MSIRRT::Tree *tree);

        bool extend(MDP::MSIRRT::Vertex::VertexCoordType &coords_of_new);
        std::vector<MDP::MSIRRT::Vertex *> set_parent(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand);
        std::vector<MDP::MSIRRT::Vertex *> rewire(MDP::MSIRRT::Vertex *node);
        std::pair<int, int> calculate_delta(MDP::MSIRRT::Vertex *candidate_node, MDP::MSIRRT::Vertex::VertexCoordType &end_coords, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand, int &safe_interval_ind);
        bool is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType &start_coords, const MDP::MSIRRT::Vertex::VertexCoordType &end_coords, double &start_time, double &end_time);
        bool is_collision_state(MDP::MSIRRT::Vertex::VertexCoordType &coords, int &time);
        // bool is_collision_motion(const MDP::MSIRRT::Vertex::VertexCoordType& start_coords, const  MDP::MSIRRT::Vertex::VertexCoordType& end_coords, double& start_time, double& end_time,MDP::MSIRRT::Vertex::VertexCoordType& last_valid_coord, int& last_valid_time);
        std::vector<MDP::MSIRRT::Vertex*> grow_tree(MDP::MSIRRT::Vertex::VertexCoordType &coord_rand, std::vector<std::pair<int, int>> &safe_intervals_of_coord_rand);
        bool connect_trees(MDP::MSIRRT::Vertex::VertexCoordType& coord_rand, std::vector<std::pair<int, int>>& safe_intervals_of_coord_rand,std::vector<MDP::MSIRRT::Vertex* > new_nodes);
        void swap_trees();
        void prune_goal_tree();
        MDP::MSIRRT::Tree *start_tree;
        MDP::MSIRRT::Tree *goal_tree;
        MDP::MSIRRT::Tree *orphan_tree;
        MDP::MSIRRT::Tree *current_tree;
        MDP::MSIRRT::Tree *other_tree;
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
        std::pair<MDP::MSIRRT::Vertex *,MDP::MSIRRT::Vertex *> goal_nodes = std::pair<MDP::MSIRRT::Vertex *,MDP::MSIRRT::Vertex *>(nullptr,nullptr);
        MDP::MSIRRT::Vertex * finish_node;
        std::vector<std::pair<float, float>> robot_limits;
        std::chrono::time_point<std::chrono::steady_clock> solver_start_time;
        std::random_device rd; // Will be used to obtain a seed for the random number engine
        std::mt19937 gen;
        bool goal_sampled;
        std::vector<std::pair<int, int>> goal_safe_intervals;
    };
}
