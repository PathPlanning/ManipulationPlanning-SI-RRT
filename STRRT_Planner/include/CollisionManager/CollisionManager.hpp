#pragma once

#include <vector>
#include <CollisionManager/RobotObstacleFCL.hpp>
#include <CollisionManager/ObjectObstacleFCL.hpp>
#include <CollisionManager/CubeObstacleFCL.hpp>
#include <CollisionManager/SphereObstacleFCL.hpp>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>
#include "hpp/fcl/broadphase/broadphase_bruteforce.h"
#include "hpp/fcl/broadphase/broadphase_spatialhash.h"
#include "hpp/fcl/broadphase/broadphase_SaP.h"
#include "hpp/fcl/broadphase/broadphase_SSaP.h"
#include "hpp/fcl/broadphase/broadphase_interval_tree.h"
#include "hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "hpp/fcl/broadphase/default_broadphase_callbacks.h"
#include "hpp/fcl/broadphase/detail/sparse_hash_table.h"
#include "hpp/fcl/broadphase/detail/spatial_hash.h"
#include "hpp/fcl/broadphase/broadphase_callbacks.h"

namespace MDP
{
    struct SafeIntervalCollisionCallback : hpp::fcl::CollisionCallBackBase
    {
        SafeIntervalCollisionCallback(std::vector<hpp::fcl::ComputeCollision> *collision_pairs_, int collision_object_count_) : collision_pairs(collision_pairs_),
                                                                                                                                collision_object_count(collision_object_count_) {};

        void init() {}
        std::vector<hpp::fcl::ComputeCollision> *collision_pairs;
        bool collide(hpp::fcl::CollisionObject *o1, hpp::fcl::CollisionObject *o2);
        int collision_object_count;
        std::vector<int> frames_in_collision;

        ~SafeIntervalCollisionCallback() {};
    };

    struct CollisionCallback : hpp::fcl::CollisionCallBackBase
    {
        CollisionCallback(std::vector<hpp::fcl::ComputeCollision> *collision_pairs_, int collision_object_count_) : collision_pairs(collision_pairs_),
                                                                                                                                collision_object_count(collision_object_count_) {};

        void init() {this->is_collided=false;}
        std::vector<hpp::fcl::ComputeCollision> *collision_pairs;
        bool collide(hpp::fcl::CollisionObject *o1, hpp::fcl::CollisionObject *o2);
        int collision_object_count;
        bool is_collided = false;

        ~CollisionCallback() {};
    };

    struct DistanceCallback : hpp::fcl::DistanceCallBackBase
    {
        DistanceCallback(std::vector<hpp::fcl::ComputeCollision> *collision_pairs_, int collision_object_count_) : collision_pairs(collision_pairs_),
                                                                                                                                collision_object_count(collision_object_count_) {};

        void init() {this->is_collided=false;dist = 1000000000000000;robot_links_points.clear();obj_points.clear();}
        std::vector<hpp::fcl::ComputeCollision> *collision_pairs;
        bool distance(hpp::fcl::CollisionObject* o1, hpp::fcl::CollisionObject* o2, hpp::fcl::FCL_REAL& dist) override; 
        std::vector<Eigen::Vector3d> robot_links_points;
        std::vector<Eigen::Vector3d> obj_points;
        int collision_object_count;
        
        bool is_collided = false;
        double dist = 10000000000000;
        ~DistanceCallback() {};
    };

    class CollisionManager
    {
    public:
        CollisionManager(const MDP::ConfigReader::SceneTask _scene_task);
        ~CollisionManager();                                                 // destructor
        CollisionManager(const CollisionManager &other) = delete;            // copy constructor
        CollisionManager(CollisionManager &&other) = default;                // move constructor
        CollisionManager &operator=(const CollisionManager &other) = delete; // copy assignment
        CollisionManager &operator=(CollisionManager &&other) = default;     // move assignment

        struct distance
        {
            bool is_collided;
            float min_distance;
            Eigen::Vector3d neares_point_obj1;
            Eigen::Vector3d neares_point_obj2;
        };

        struct robot_link_distance_to_obj
        {
            float min_distance;
            std::vector<Eigen::Vector3d> neares_point_robot;
            std::vector<Eigen::Vector3d> neares_point_obj;
            robot_link_distance_to_obj(float _min_distance, std::vector<Eigen::Vector3d> _neares_point_robot, std::vector<Eigen::Vector3d> _neares_point_obj) : min_distance(_min_distance),
                                                                                                                                                                neares_point_robot(_neares_point_robot),
                                                                                                                                                                neares_point_obj(_neares_point_obj) {};
        };

        int get_goal_frame_low_bound();
        bool check_collision(const std::vector<double>& robot_angles, float& time);
        bool check_collision_frame(const std::vector<double>& robot_angles, int& frame);
        bool check_collision_frame_no_wrapper(std::vector<double> robot_angles, int frame);
        std::vector<robot_link_distance_to_obj> get_distances(std::vector<double> robot_angles, float time, bool &is_collision);
        bool check_robot_selfcollision(std::vector<double> robot_angles);
        bool check_base_joints_collisiion(std::vector<double> robot_angles, int last_base_joint_ind);
        std::vector<std::pair<int, int>> get_safe_intervals_naive(std::vector<double> robot_angles);
        std::vector<std::pair<int, int>> get_safe_intervals_naive_fastcc(std::vector<double> robot_angles);
        std::vector<std::pair<int, int>> get_safe_intervals_naive_fastcc_oop(std::vector<double> robot_angles);
        std::vector<std::pair<int, int>> get_safe_intervals(const std::vector<double>& robot_angles);

        std::vector<std::pair<float, float>> get_planned_robot_limits() const;

        void benchmark_broadphase();

    private:
        bool check_collision_private_naive(std::vector<double> robot_angles, int frame);
        bool check_collision_private(const std::vector<double>& robot_angles, int& frame);
        std::vector<robot_link_distance_to_obj> get_distances_private(std::vector<double> robot_angles, float time, bool &is_collision);
        std::vector<robot_link_distance_to_obj> get_distances_naive_private(std::vector<double> robot_angles, float time, bool &is_collision);

        void move_dynamic_obstacles(int frame) const;
        bool collide_objects(const hpp::fcl::CollisionGeometry *o1, const hpp::fcl::Transform3f tf1,
                             const hpp::fcl::CollisionGeometry *o2, const hpp::fcl::Transform3f tf2) const;
        distance measure_distance(const hpp::fcl::CollisionGeometry *o1, const hpp::fcl::Transform3f tf1,
                                  const hpp::fcl::CollisionGeometry *o2, const hpp::fcl::Transform3f tf2) const;

        std::vector<MDP::ObjectObstacleFCL *> obstacles;
        std::vector<MDP::RobotObstacleFCL> robot_obstacles;
        MDP::RobotObstacleFCL planned_robot;
        MDP::ConfigReader::SceneTask scene_task;

        hpp::fcl::Transform3f *positions; // for cache optimisation. array, with struct [frame1[obj1[x,y,z,qx,qy,qz,w],obj2[x,y,z,qx,qy,qz,w].,.], frame2[...],..]
        std::vector<hpp::fcl::ComputeCollision> collision_pairs;
        int frame_count;
        int collision_robot_links_count;
        int collision_object_count;

        std::pair<int, int> *frames_and_ids;
        std::vector<hpp::fcl::CollisionObject *> all_spheres;
        hpp::fcl::BroadPhaseCollisionManager *broadphase_manager;

        std::vector<hpp::fcl::BroadPhaseCollisionManager *>frame_broadphase_managers;
        std::vector<std::vector<hpp::fcl::CollisionObject *>> by_frame_spheres;
    };
}