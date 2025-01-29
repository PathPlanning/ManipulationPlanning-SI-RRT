#include <vector>
#include <CollisionManager/CollisionManager.hpp>
#include <CollisionManager/ObjectObstacleFCL.hpp>
#include <CollisionManager/CubeObstacleFCL.hpp>
#include <CollisionManager/SphereObstacleFCL.hpp>
#include <CollisionManager/RobotObstacleFCL.hpp>
#include <iostream>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/broadphase/default_broadphase_callbacks.h>
#include <hpp/fcl/shape/convex.h>
#include "config_read_writer/ResultsWriter.hpp"
#include "hpp/fcl/broadphase/broadphase_callbacks.h"
#include <random>

MDP::CollisionManager::CollisionManager(const MDP::ConfigReader::SceneTask _scene_task) : planned_robot(_scene_task.robot_urdf_path, _scene_task.robot_base_position_vector, _scene_task.robot_joints_order, std::vector<std::vector<double>>()),
                                                                                          scene_task(_scene_task)
{ // init robot (in initializer list)

    // init obstacles
    for (auto obstacle : scene_task.obstacles)
    { // TODO: divide obstacles into static and dynamic one.
        if (obstacle->get_type() == "static_box")
        {
            this->obstacles.push_back(static_cast<MDP::ObjectObstacleFCL *>(new MDP::CubeObstaclesFCL(*static_cast<CubeObstacleJsonInfo *>(obstacle.get()))));
        }
        else if (obstacle->get_type() == "dynamic_box")
        {
            this->obstacles.push_back(static_cast<MDP::ObjectObstacleFCL *>(new MDP::CubeObstaclesFCL(*static_cast<CubeObstacleJsonInfo *>(obstacle.get()))));
        }
        else if (obstacle->get_type() == "static_sphere")
        {
            this->obstacles.push_back(static_cast<MDP::ObjectObstacleFCL *>(new MDP::SphereObstaclesFCL(*static_cast<SphereObstacleJsonInfo *>(obstacle.get()))));
        }
        else if (obstacle->get_type() == "dynamic_sphere")
        {
            this->obstacles.push_back(static_cast<MDP::ObjectObstacleFCL *>(new MDP::SphereObstaclesFCL(*static_cast<SphereObstacleJsonInfo *>(obstacle.get()))));
        }
    }

    for (MDP::RobotObstacleJsonInfo obstacle : scene_task.robot_obstacles)
    {
        if (obstacle.get_type() == "dynamic_robot")
        {
            this->robot_obstacles.emplace_back(obstacle.get_urdf_file_path(), obstacle.get_base_coordinates(), obstacle.get_robot_joints_order(), obstacle.get_trajectory());
        }
    }

    this->frame_count = this->scene_task.frame_count;
    this->collision_object_count = this->obstacles.size();

    // fill positions array
    this->positions = new hpp::fcl::Transform3f[frame_count * collision_object_count];

    for (int obstacle_id = 0; obstacle_id < collision_object_count; obstacle_id++)
    {
        MDP::ObjectObstacleFCL *obstacle = this->obstacles[obstacle_id];
        for (int frame = 0; frame < this->frame_count; frame++)
        {
            if (obstacle->get_is_static())
            {
                MDP::ObstacleCoordinate obj_position = obstacle->get_position(0);
                // frame* collision_object_count is frame offset
                // obstacle_id is object offset
                this->positions[frame * collision_object_count + obstacle_id].setQuatRotation(obj_position.rotation);
                this->positions[frame * collision_object_count + obstacle_id].setTranslation(obj_position.pos);
            }
            else
            {
                MDP::ObstacleCoordinate obj_position = obstacle->get_position(frame);
                // frame* collision_object_count is frame offset
                // obstacle_id is object offset
                this->positions[frame * collision_object_count + obstacle_id].setQuatRotation(obj_position.rotation);
                this->positions[frame * collision_object_count + obstacle_id].setTranslation(obj_position.pos);
            }
        }
    }
    // fill collision pairs
    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> robot_joints = this->planned_robot.get_collision_object_for_robot_angles(scene_task.start_configuration);
    this->collision_robot_links_count = 4; // TODO: -2 is without first 2 links and without last one. We don't check collisions for them (now). remove magic number.

    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        for (int obstacle_id = 0; obstacle_id < collision_object_count; obstacle_id++)
        {
            MDP::ObjectObstacleFCL *obstacle = this->obstacles[obstacle_id];
            this->collision_pairs.emplace_back(robot_joints[robot_joint_id].collision_object.get(), obstacle->get_collision_object());
        }
    }

    // construct safeinterval broadphase colliison manager

    hpp::fcl::DynamicAABBTreeArrayCollisionManager *temp = new hpp::fcl::DynamicAABBTreeArrayCollisionManager();
    temp->tree_init_level = 2;
    this->broadphase_manager = temp;
    // get all spheres for all frames_and_ids and set their time
    this->frames_and_ids = new std::pair<int, int>[scene_task.frame_count * collision_object_count];

    for (int obstacle_id = 0; obstacle_id < collision_object_count; obstacle_id++)
    {
        MDP::ObjectObstacleFCL *obstacle = this->obstacles[obstacle_id];
        hpp::fcl::ShapeBase *obj = obstacle->get_collision_object();

        for (int frame = 0; frame < this->frame_count; frame++)
        {

            if (obstacle->get_is_static())
            {
                MDP::ObstacleCoordinate obj_position = obstacle->get_position(0);

                this->all_spheres.push_back(new hpp::fcl::CollisionObject(std::shared_ptr<hpp::fcl::CollisionGeometry>(obj->clone()), obj_position.rotation.toRotationMatrix(), obj_position.pos));
                this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].first = frame;
                this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].second = obstacle_id;
                this->all_spheres.back()->setUserData(this->frames_and_ids + frame + scene_task.frame_count * obstacle_id);
                this->all_spheres.back()->collisionGeometry()->computeLocalAABB();
            }
            else
            {
                MDP::ObstacleCoordinate obj_position = obstacle->get_position(frame);

                this->all_spheres.push_back(new hpp::fcl::CollisionObject(std::shared_ptr<hpp::fcl::CollisionGeometry>(obj->clone()), obj_position.rotation.toRotationMatrix(), obj_position.pos));
                this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].first = frame;
                this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].second = obstacle_id;
                this->all_spheres.back()->setUserData(this->frames_and_ids + frame + scene_task.frame_count * obstacle_id);
                this->all_spheres.back()->collisionGeometry()->computeLocalAABB();
            }
        }
    }

    this->broadphase_manager->registerObjects(this->all_spheres);
    this->broadphase_manager->setup();

    for (int frame = 0; frame < this->scene_task.frame_count; frame++)
    {

        this->by_frame_spheres.emplace_back();
        for (int obstacle_id = 0; obstacle_id < collision_object_count; obstacle_id++)
        {
            MDP::ObjectObstacleFCL *obstacle = this->obstacles[obstacle_id];
            hpp::fcl::ShapeBase *obj = obstacle->get_collision_object();

            if (obstacle->get_is_static())
            {
                MDP::ObstacleCoordinate obj_position = obstacle->get_position(0);
                this->by_frame_spheres[frame].push_back(new hpp::fcl::CollisionObject(std::shared_ptr<hpp::fcl::CollisionGeometry>(obj->clone()), obj_position.rotation.toRotationMatrix(), obj_position.pos));
                // this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].first = frame;
                // this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].second = obstacle_id;
                // this->all_spheres.back()->setUserData(this->frames_and_ids + frame + scene_task.frame_count * obstacle_id);
                this->by_frame_spheres[frame].back()->collisionGeometry()->computeLocalAABB();
                this->by_frame_spheres[frame].back()->setUserData(this->frames_and_ids + frame + scene_task.frame_count * obstacle_id);
            }
            else
            {

                MDP::ObstacleCoordinate obj_position = obstacle->get_position(frame);
                this->by_frame_spheres[frame].push_back(new hpp::fcl::CollisionObject(std::shared_ptr<hpp::fcl::CollisionGeometry>(obj->clone()), obj_position.rotation.toRotationMatrix(), obj_position.pos));

                // this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].first = frame;
                // this->frames_and_ids[frame + scene_task.frame_count * obstacle_id].second = obstacle_id;
                // this->all_spheres.back()->setUserData(this->frames_and_ids + frame + scene_task.frame_count * obstacle_id);
                this->by_frame_spheres[frame].back()->collisionGeometry()->computeLocalAABB();
                this->by_frame_spheres[frame].back()->setUserData(this->frames_and_ids + frame + scene_task.frame_count * obstacle_id);
            }
        }
        // hpp::fcl::Vec3f lower_limit, upper_limit;

        // hpp::fcl::SpatialHashingCollisionManager<>::computeBound(this->all_spheres, lower_limit, upper_limit);
        // hpp::fcl::FCL_REAL cell_size =
        //     std::min(std::min((upper_limit[0] - lower_limit[0]) / 20,
        //                       (upper_limit[1] - lower_limit[1]) / 20),
        //              (upper_limit[2] - lower_limit[2]) / 20);

        this->frame_broadphase_managers.push_back(new hpp::fcl::DynamicAABBTreeArrayCollisionManager());
        static_cast<hpp::fcl::DynamicAABBTreeArrayCollisionManager *>((this->frame_broadphase_managers[frame]))->tree_init_level = 2;

        // this->frame_broadphase_managers.push_back(new hpp::fcl::SpatialHashingCollisionManager<
        // hpp::fcl::detail::SparseHashTable<hpp::fcl::AABB, hpp::fcl::CollisionObject *, hpp::fcl::detail::SpatialHash>>(
        // cell_size, lower_limit, upper_limit));
        this->frame_broadphase_managers[frame]->registerObjects(this->by_frame_spheres[frame]);
        this->frame_broadphase_managers[frame]->setup();
    }
}

MDP::CollisionManager::~CollisionManager()
{
    for (int i = 0; i < this->obstacles.size(); i++)
    {
        delete this->obstacles[i];
    }

    this->obstacles.clear();

    for (int i = 0; i < this->all_spheres.size(); i++)
    {
        delete this->all_spheres[i];
    }

    for (int i = 0; i < this->by_frame_spheres.size(); i++)
    {
        for (int j = 0; j < this->by_frame_spheres[i].size(); j++)
        {
            delete this->by_frame_spheres[i][j];
        }
    }

    for (int i = 0; i < this->frame_broadphase_managers.size(); i++)
    {
        delete this->frame_broadphase_managers[i];
    }

    this->all_spheres.clear();

    delete[] this->positions;
    delete[] this->frames_and_ids;
    delete this->broadphase_manager;
}
bool MDP::CollisionManager::check_collision(const std::vector<double> &robot_angles, float &time)

{
    int frame = (int)(time * (this->scene_task.fps));
    return MDP::ResultsWriter::get_instance().collision_check_wrapper(std::bind(&MDP::CollisionManager::check_collision_private, this, std::placeholders::_1, std::placeholders::_2), robot_angles, frame);
}

bool MDP::CollisionManager::check_collision_frame(const std::vector<double> &robot_angles, int &frame)
{
    return MDP::ResultsWriter::get_instance().collision_check_wrapper(std::bind(&MDP::CollisionManager::check_collision_private, this, std::placeholders::_1, std::placeholders::_2), robot_angles, frame);
}

bool MDP::CollisionManager::check_collision_frame_no_wrapper(std::vector<double> robot_angles, int frame)
{
    return this->check_collision_private(robot_angles, frame);
}

bool MDP::CollisionManager::check_collision_private(const std::vector<double> &robot_angles, int &frame)
{

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> obstacle_joints;

    // bool is_collided = false;

    MDP::CollisionCallback collision_callback(&(this->collision_pairs), this->collision_object_count);
    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        collision_callback.is_collided = false;
        hpp::fcl::CollisionObject joint_coll_obj(main_robot_collision_models[robot_joint_id].collision_object, main_robot_collision_models[robot_joint_id].transform);
        joint_coll_obj.setUserData(new int(robot_joint_id));
        // std::cout<<&this->collision_pairs<<std::endl;
        // std::cout <<(robot_joint_id)<<std::endl;
        // std::cout<<joint_coll_obj.getTransform().getTranslation()<<" "<<((hpp::fcl::Capsule*)(joint_coll_obj.collisionGeometry().get()))->radius<<" "<<((hpp::fcl::Capsule*)(joint_coll_obj.collisionGeometry().get()))->halfLength<<std::endl;
        this->frame_broadphase_managers[frame]->collide(&joint_coll_obj, &collision_callback);
        delete (int *)joint_coll_obj.getUserData();

        if (collision_callback.is_collided)
        {
            return true;
        }
    }

    return false;
}

bool MDP::CollisionManager::check_collision_private_naive(std::vector<double> robot_angles, int frame)
{
    // TODO: optimise for static and dynamic obstacless
    //  move robots and dynamic obstacles
    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> obstacle_joints;
    // for (MDP::RobotObstacleFCL obstacle_robot : (this->robot_obstacles))
    // {
    //     std::vector<MDP::RobotObstacleFCL::JointCollisionObject> result_obstacle_joints = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_frame, &obstacle_robot, std::placeholders::_1), frame);
    //     obstacle_joints.insert(obstacle_joints.end(), result_obstacle_joints.begin(), result_obstacle_joints.end());
    // }

    bool is_collided = false;
    // for(auto ang:robot_angles){
    //     std::cout<<ang<<" ";
    // }
    // std::cout<<std::endl;

    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {

        // std::cout<<main_robot_collision_models[robot_joint_id].name<<std::endl;
        // std::cout<<main_robot_collision_models[robot_joint_id].transform.getTranslation()<<std::endl;
        // std::cout<<main_robot_collision_models[robot_joint_id].transform.getQuatRotation()<<std::endl;
        // std::cout<<main_robot_collision_models[robot_joint_id].transform.getRotation()<<std::endl;
        for (int obstacle_id = 0; obstacle_id < this->collision_object_count; obstacle_id++)
        {
            hpp::fcl::CollisionRequest col_req;
            col_req.security_margin = 0.000000000000000000001;
            // col_req.enable_contact = false;
            hpp::fcl::CollisionResult col_res;

            this->collision_pairs[robot_joint_id * collision_object_count + obstacle_id](main_robot_collision_models[robot_joint_id].transform, this->positions[frame * collision_object_count + obstacle_id], col_req, col_res);
            is_collided = col_res.isCollision();
            if (is_collided)
            {
                // std::cout<<obstacle_id<<" "<<robot_joint_id<<std::endl;
                return is_collided;
            }
        }
    }

    // uncomment for multirobot
    //  for (MDP::RobotObstacleFCL::JointCollisionObject &joint_mesh_and_transfrom : main_robot_collision_models) // for robot joint mesh
    //  {

    //     for (auto &obstacle : obstacle_joints)
    //     {

    //         is_collided = this->collide_objects(joint_mesh_and_transfrom.collision_object.get(), joint_mesh_and_transfrom.transform,
    //                                             obstacle.collision_object.get(), obstacle.transform);
    //         if (is_collided)
    //         {
    //             // std::cout<<"collision detected!!!"<<std::endl;
    //             //         std::cout<<joint_mesh_and_transfrom.name<<std::endl;
    //             //         std::cout<<obstacle.name<<std::endl;
    //             return true;
    //         }
    //     }
    //     if (is_collided)
    //     {
    //         return true;
    //     }
    // }
    return is_collided;
}

bool MDP::CollisionManager::collide_objects(const hpp::fcl::CollisionGeometry *o1, const hpp::fcl::Transform3f tf1,
                                            const hpp::fcl::CollisionGeometry *o2, const hpp::fcl::Transform3f tf2) const
{

    hpp::fcl::CollisionRequest col_req;
    col_req.security_margin = 0.000000000000000000001;
    hpp::fcl::CollisionResult col_res;
    hpp::fcl::collide(o1, tf1, o2, tf2, col_req, col_res);

    bool is_collided = col_res.isCollision();
    return is_collided;
}

void MDP::CollisionManager::move_dynamic_obstacles(int frame) const
{

    // for (auto &box_obstacle : dynamic_cube_obstacles)
    // {

    //     assert(frame < scene_task.frame_count);
    //     box_obstacle.set_position(frame);
    // }
}
std::vector<MDP::CollisionManager::robot_link_distance_to_obj> MDP::CollisionManager::get_distances(std::vector<double> robot_angles, float time, bool &is_collision)
{
    return MDP::ResultsWriter::get_instance().distance_check_wrapper(std::bind(&MDP::CollisionManager::get_distances_private, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), robot_angles, time, is_collision);
}

std::vector<MDP::CollisionManager::robot_link_distance_to_obj> MDP::CollisionManager::get_distances_private(std::vector<double> robot_angles, float time, bool &is_collision)
{
    is_collision = false;

    std::vector<MDP::CollisionManager::robot_link_distance_to_obj> results;

    // move robots and dynamic obstacles
    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_distance_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);
    int frame = (int)(time * (this->scene_task.fps));


    MDP::DistanceCallback distance_callback(&this->collision_pairs, this->collision_object_count);
    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        distance_callback.init();
        float min_distance = std::numeric_limits<float>::max();
        std::vector<Eigen::Vector3d> robot_links_points;
        std::vector<Eigen::Vector3d> obj_points;

        hpp::fcl::CollisionObject joint_coll_obj(main_robot_collision_models[robot_joint_id].collision_object, main_robot_collision_models[robot_joint_id].transform);
        joint_coll_obj.setUserData(new int(robot_joint_id));
        this->frame_broadphase_managers[frame]->distance(&joint_coll_obj, &distance_callback);
        delete (int *)joint_coll_obj.getUserData();

        if (distance_callback.is_collided)
        {
            is_collision = true;
            return results;
        }
        results.emplace_back(distance_callback.dist, distance_callback.robot_links_points, distance_callback.obj_points);
    }

    // multirobot is not implemented.
    return results;
};

std::vector<MDP::CollisionManager::robot_link_distance_to_obj> MDP::CollisionManager::get_distances_naive_private(std::vector<double> robot_angles, float time, bool &is_collision)
{
    is_collision = false;

    std::vector<MDP::CollisionManager::robot_link_distance_to_obj> results;

    // move robots and dynamic obstacles
    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_distance_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);
    int frame = (int)(time * (this->scene_task.fps));

    // std::vector<MDP::RobotObstacleFCL::JointCollisionObject> obstacle_joints;
    // for (MDP::RobotObstacleFCL obstacle_robot : (this->robot_obstacles))
    // {
    //     std::vector<MDP::RobotObstacleFCL::JointCollisionObject> result_obstacle_joints = MDP::ResultsWriter::get_instance().forward_kinematics_distance_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_frame, &obstacle_robot, std::placeholders::_1), (int)(time * (this->scene_task.frame_counts;)));
    //     obstacle_joints.insert(obstacle_joints.end(), result_obstacle_joints.begin(), result_obstacle_joints.end());
    // }

    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        float min_distance = std::numeric_limits<float>::max();
        std::vector<Eigen::Vector3d> robot_links_points;
        std::vector<Eigen::Vector3d> obj_points;
        for (int obstacle_id = 0; obstacle_id < this->collision_object_count; obstacle_id++)
        {
            hpp::fcl::CollisionRequest col_req;
            col_req.security_margin = 0.000000000000000000001;
            // col_req.enable_contact = false;
            hpp::fcl::CollisionResult col_res;

            // std::cout<<this->positions[frame * collision_object_count + obstacle_id].getTranslation()<<std::endl;
            // std::cout<<this->positions[frame * collision_object_count + obstacle_id].getRotation()<<std::endl;
            // std::cout<<frame<<std::endl;
            // for(auto ang:robot_angles){
            //     std::cout<<ang<<" ";
            // }
            // std::cout<<std::endl;
            // std::cout<<main_robot_collision_models[robot_joint_id].transform.getTranslation()<<std::endl;
            // std::cout<<main_robot_collision_models[robot_joint_id].transform.getRotation()<<std::endl;

            this->collision_pairs[robot_joint_id * collision_object_count + obstacle_id](main_robot_collision_models[robot_joint_id].transform, this->positions[frame * collision_object_count + obstacle_id], col_req, col_res);
            is_collision = col_res.isCollision();
            if (is_collision)
            {
                is_collision = true;
                return results;
            }
            // std::cout<<"distance: "<<col_res.distance_lower_bound<<std::endl;

            if (col_res.distance_lower_bound < min_distance)
            {
                min_distance = col_res.distance_lower_bound;
            }
            else if (col_res.distance_lower_bound > 10)
            {
                assert(false);
            }
            robot_links_points.push_back(col_res.nearest_points.at(0));
            obj_points.push_back(col_res.nearest_points.at(1));
        }
        results.emplace_back(min_distance, robot_links_points, obj_points); // Now DRGBT don't use nearest points, so, we will return them as empty.
    }

    // multirobot is not implemented.
    return results;
};

MDP::CollisionManager::distance MDP::CollisionManager::measure_distance(const hpp::fcl::CollisionGeometry *o1, const hpp::fcl::Transform3f tf1,
                                                                        const hpp::fcl::CollisionGeometry *o2, const hpp::fcl::Transform3f tf2) const
{
    MDP::CollisionManager::distance result;
    result.is_collided = false;
    hpp::fcl::DistanceRequest dis_req;
    hpp::fcl::DistanceResult dis_res;
    hpp::fcl::distance(o1, tf1, o2, tf2, dis_req, dis_res);

    if (dis_res.min_distance <= 0)
    {
        result.is_collided = true;
        result.min_distance = 0;
        return result;
    }

    result.min_distance = dis_res.min_distance;

    result.neares_point_obj1 = dis_res.nearest_points.at(0);
    result.neares_point_obj2 = dis_res.nearest_points.at(1);
    return result;
}

std::vector<std::pair<float, float>> MDP::CollisionManager::get_planned_robot_limits() const
{
    return this->planned_robot.get_limits();
}

int MDP::CollisionManager::get_goal_frame_low_bound()
{
    for (int frame = this->scene_task.frame_count - 1; frame >= 0; frame--)
    {
        std::vector<double> end_configuration_f(scene_task.end_configuration.begin(), scene_task.end_configuration.end());
        bool collision_result = this->check_collision_frame(end_configuration_f, frame); // ACHTUNG!!! TODO: conversion from frame to double and fropm double to frame may cause bugs
        if (collision_result)
        {
            if (frame == this->scene_task.frame_count - 1)
            {
                throw std::runtime_error("there is collision in goal state at the last frame!");
            }
            return (frame + 1);
        }
    }
    return 0;
}

bool MDP::CollisionManager::check_robot_selfcollision(std::vector<double> robot_angles)
{
    return false;
    // move robots and dynamic obstacles
    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);

    bool is_collided = false;
    int ind = 0;
    for (MDP::RobotObstacleFCL::JointCollisionObject &joint_mesh_and_transfrom : main_robot_collision_models) // for robot joint mesh
    {
        if (ind == main_robot_collision_models.size() - 2)
        {
            return false;
        }
        // don't check neighbour link
        for (int another_joint_ind = ind + 2; another_joint_ind < main_robot_collision_models.size(); another_joint_ind++)
        {
            MDP::RobotObstacleFCL::JointCollisionObject &another_joint_mesh_and_transfrom = main_robot_collision_models[another_joint_ind];

            bool self_collision = this->collide_objects(joint_mesh_and_transfrom.collision_object.get(), joint_mesh_and_transfrom.transform,
                                                        another_joint_mesh_and_transfrom.collision_object.get(), another_joint_mesh_and_transfrom.transform);
            if (self_collision)
            {

                return true;
            }
        }
        ind++;
    }
}

bool MDP::CollisionManager::check_base_joints_collisiion(std::vector<double> robot_angles, int last_base_joint_ind)
{
    return false;
}

std::vector<std::pair<int, int>> MDP::CollisionManager::get_safe_intervals_naive(std::vector<double> robot_angles)
{
    std::vector<std::pair<int, int>> results;

    int start_safe_frame = 0;
    bool is_safe = false;

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);

    for (int frame = 0; frame < this->scene_task.frame_count; frame++)
    {
        bool is_collided = false;
        for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
        {
            for (int obstacle_id = 0; obstacle_id < this->collision_object_count; obstacle_id++)
            {
                hpp::fcl::CollisionRequest col_req;
                col_req.security_margin = 0.000000000000000000001;
                hpp::fcl::CollisionResult col_res;
                // hpp::fcl::collide(main_robot_collision_models[robot_joint_id].collision_object.get(),main_robot_collision_models[robot_joint_id].transform,this->obstacles[obstacle_id]->get_collision_object(), this->positions[frame * collision_object_count + obstacle_id], col_req, col_res);
                this->collision_pairs[robot_joint_id * collision_object_count + obstacle_id](main_robot_collision_models[robot_joint_id].transform, this->positions[frame * collision_object_count + obstacle_id], col_req, col_res);
                is_collided = col_res.isCollision();
                hpp::fcl::collide(main_robot_collision_models[robot_joint_id].collision_object.get(), main_robot_collision_models[robot_joint_id].transform, this->obstacles[obstacle_id]->get_collision_object(), this->positions[frame * collision_object_count + obstacle_id], col_req, col_res);
                assert(is_collided == col_res.isCollision());
                if (is_collided)
                {
                    // std::cout << frame << " " << robot_joint_id << " " << obstacle_id << std::endl;
                    // std::cout << static_cast<hpp::fcl::Sphere*>(this->obstacles[obstacle_id]->get_collision_object())->radius<<std::endl;
                    // std::cout<< this->positions[frame * collision_object_count + obstacle_id].getTranslation()<<std::endl;
                    // std::cout<< this->positions[frame * collision_object_count + obstacle_id].getQuatRotation()<<std::endl;
                    // std::cout<< main_robot_collision_models[robot_joint_id].transform.getTranslation()<<std::endl;
                    // std::cout<< main_robot_collision_models[robot_joint_id].transform.getQuatRotation()<<std::endl;
                    // std::cout << static_cast<hpp::fcl::Capsule*>(main_robot_collision_models[robot_joint_id].collision_object.get())->radius<<std::endl;
                    // std::cout << static_cast<hpp::fcl::Capsule*>(main_robot_collision_models[robot_joint_id].collision_object.get())->halfLength<<std::endl;
                    break;
                }
            }
            if (is_collided)
            {
                break;
            }
        }
        if ((!is_collided) && (!is_safe))
        {
            start_safe_frame = frame;
            is_safe = true;
        }
        else if ((is_collided) && (is_safe))
        {
            results.emplace_back(start_safe_frame, frame - 1);
            is_safe = false;
        }
    }
    if (is_safe)
    {
        results.emplace_back(start_safe_frame, this->scene_task.frame_count - 1);
        is_safe = false;
    }
    if (results.size() == 0)
    {
        assert(start_safe_frame == 0);
        assert(is_safe == true);
        results.emplace_back(0, scene_task.frame_count - 1);
    }
    return results;
}

std::vector<std::pair<int, int>> MDP::CollisionManager::get_safe_intervals_naive_fastcc(std::vector<double> robot_angles)
{

    std::vector<std::pair<int, int>> results;

    int start_safe_frame = 0;
    bool is_safe = false;

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);
    MDP::CollisionCallback collision_callback(&this->collision_pairs, this->collision_object_count);
    std::vector<hpp::fcl::CollisionObject> joint_coll_obj;

    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        joint_coll_obj.emplace_back(main_robot_collision_models[robot_joint_id].collision_object, main_robot_collision_models[robot_joint_id].transform);
        int *joint_ind = new int(robot_joint_id);
        joint_coll_obj.back().setUserData(joint_ind);
    }

    for (int frame = 0; frame < this->scene_task.frame_count; frame++)
    {

        bool is_collided = false;
        for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
        {
            collision_callback.init();
            this->frame_broadphase_managers[frame]->collide(&(joint_coll_obj[robot_joint_id]), &collision_callback);
            is_collided = collision_callback.is_collided;
            if (is_collided)
            {
                break;
            }
        }

        if ((!is_collided) && (!is_safe))
        {
            start_safe_frame = frame;
            is_safe = true;
        }
        else if ((is_collided) && (is_safe))
        {
            results.emplace_back(start_safe_frame, frame - 1);
            is_safe = false;
        }
    }
    if (is_safe)
    {
        results.emplace_back(start_safe_frame, this->scene_task.frame_count - 1);
        is_safe = false;
    }
    if (results.size() == 0)
    {
        assert(start_safe_frame == 0);
        assert(is_safe == true);
        results.emplace_back(0, scene_task.frame_count - 1);
    }

    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        delete (int *)joint_coll_obj[robot_joint_id].getUserData();
    }

    return results;
}

std::vector<std::pair<int, int>> MDP::CollisionManager::get_safe_intervals_naive_fastcc_oop(std::vector<double> robot_angles)
{

    std::vector<std::pair<int, int>> results;

    int start_safe_frame = 0;
    bool is_safe = false;

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);

    for (int frame = 0; frame < this->scene_task.frame_count; frame++)
    {
        bool is_collided = this->check_collision_private(robot_angles, frame);

        if ((!is_collided) && (!is_safe))
        {
            start_safe_frame = frame;
            is_safe = true;
        }
        else if ((is_collided) && (is_safe))
        {
            results.emplace_back(start_safe_frame, frame - 1);
            is_safe = false;
        }
    }
    if (is_safe)
    {
        results.emplace_back(start_safe_frame, this->scene_task.frame_count - 1);
        is_safe = false;
    }
    if (results.size() == 0)
    {
        assert(start_safe_frame == 0);
        assert(is_safe == true);
        results.emplace_back(0, scene_task.frame_count - 1);
    }

    return results;
}

std::vector<std::pair<int, int>> MDP::CollisionManager::get_safe_intervals(const std::vector<double> &robot_angles)
{
    std::vector<std::pair<int, int>> results;

    int start_safe_frame = 0;
    bool is_safe = false;

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> main_robot_collision_models = MDP::ResultsWriter::get_instance().forward_kinematics_collision_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles, &(this->planned_robot), std::placeholders::_1), robot_angles);

    MDP::SafeIntervalCollisionCallback collision_callback(&this->collision_pairs, this->collision_object_count);
    for (int robot_joint_id = 0; robot_joint_id < this->collision_robot_links_count; robot_joint_id++)
    {
        hpp::fcl::CollisionObject joint_coll_obj(main_robot_collision_models[robot_joint_id].collision_object, main_robot_collision_models[robot_joint_id].transform);
        int *joint_ind = new int(robot_joint_id);
        joint_coll_obj.setUserData(joint_ind);
        // std::cout<<"1"<<std::endl;
        this->broadphase_manager->collide(&joint_coll_obj, &collision_callback);
        delete joint_ind;
        // std::cout<<"2"<<std::endl;
    }

    std::vector<int> frames_in_collision(collision_callback.frames_in_collision);

    if (frames_in_collision.size() == 0)
    {
        results.emplace_back(0, scene_task.frame_count - 1);
        return results;
    }
    // sort and delete duplicates
    std::sort(frames_in_collision.begin(), frames_in_collision.end());
    frames_in_collision.erase(std::unique(frames_in_collision.begin(), frames_in_collision.end()), frames_in_collision.end());

    int last_collision_frame = -1;
    for (const int collision_frame : frames_in_collision)
    {
        if ((collision_frame - last_collision_frame) != 1)
        {
            results.emplace_back(last_collision_frame + 1, collision_frame - 1);
        }

        last_collision_frame = collision_frame;
    }
    if (frames_in_collision.back() != (this->scene_task.frame_count - 1))
    {
        results.emplace_back(frames_in_collision.back() + 1, this->scene_task.frame_count - 1);
    }
    return results;
}

bool MDP::DistanceCallback::distance(hpp::fcl::CollisionObject *o1, hpp::fcl::CollisionObject *o2, hpp::fcl::FCL_REAL& dist)
{
    if (this->is_collided)
    {
        return this->is_collided;
    }
    // collide
    hpp::fcl::DistanceRequest dis_req;
    hpp::fcl::DistanceResult dis_res;
    hpp::fcl::CollisionRequest col_req;
    col_req.security_margin = 0.000000000000000000001;
    hpp::fcl::CollisionResult col_res;
    int *robot_joint_id;
    std::pair<int, int> *obstacle_frame_and_id;
    if (!((o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE && o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE) || (o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE && o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE)))
    {
        // std::cout<<"AABB"<<std::endl;
        // assert(false);
        hpp::fcl::distance(o1, o2, dis_req, dis_res);
        dist = dis_res.min_distance;
        this->dist = dis_res.min_distance;
        this->is_collided = dis_res.min_distance < 0;
        this->robot_links_points.push_back(dis_res.nearest_points.at(0));
        this->obj_points.push_back(dis_res.nearest_points.at(1));
        return this->is_collided;
    }
    else if ((o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE) && (o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE))
    {

        robot_joint_id = (int *)o1->getUserData();
        obstacle_frame_and_id = (std::pair<int, int> *)o2->getUserData();
        (this->collision_pairs->at((*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)))(o1->getTransform(), o2->getTransform(), col_req, col_res);
        // bool res = col_res.isCollision();
        // col_res.clear();
        // hpp::fcl::collide(o1, o2, col_req, col_res);
        // std::cout <<(*robot_joint_id)<<" "<<this->collision_object_count<<std::endl;

        // assert(res == col_res.isCollision());
    }
    else if ((o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE) && (o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE))
    {

        robot_joint_id = (int *)o2->getUserData();
        obstacle_frame_and_id = (std::pair<int, int> *)o1->getUserData();
        (this->collision_pairs->at((*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)))(o2->getTransform(), o1->getTransform(), col_req, col_res);
        // bool res = col_res.isCollision();
        // col_res.clear();
        // hpp::fcl::collide(o1, o2, col_req, col_res);
        // std::cout<<this->collision_pairs<<std::endl;
        // std::cout<<(*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)<<std::endl;
        // std::cout <<(*robot_joint_id)<<" "<<this->collision_object_count<<" "<<obstacle_frame_and_id->first<<" "<<obstacle_frame_and_id->second<<std::endl;
        // std::cout<<o1->getTransform().getTranslation()<<std::endl;
        // std::cout<<o2->getTransform().getTranslation()<<" "<<((hpp::fcl::Capsule*)(o2->collisionGeometry().get()))->radius<<" "<<((hpp::fcl::Capsule*)(o2->collisionGeometry().get()))->halfLength<<std::endl;
        // assert(res == col_res.isCollision());
    }
    else
    {
        assert(false);
    }
    // std::cout<<"sphere"<<std::endl;
    // std::cout<<"robot_joint_id "<<*robot_joint_id<<std::endl;
    // std::cout<<"obstacle_frame_and_id "<<obstacle_frame_and_id->first <<" "<<obstacle_frame_and_id->second<<std::endl;
    // hpp::fcl::collide(o1, o2, col_req, col_res);
    dist = col_res.distance_lower_bound;

    this->is_collided = col_res.isCollision();
    this->dist = col_res.distance_lower_bound; // distance_lower_bound == distance for simple shapes. TODO: check that info
    this->robot_links_points.push_back(col_res.nearest_points.at(0));
    this->obj_points.push_back(col_res.nearest_points.at(1));

    return this->is_collided;
}

bool MDP::CollisionCallback::collide(hpp::fcl::CollisionObject *o1, hpp::fcl::CollisionObject *o2)
{
    if (this->is_collided)
    {
        return this->is_collided;
    }
    // collide
    hpp::fcl::CollisionRequest col_req;
    col_req.security_margin = 0.000000000000000000001;
    hpp::fcl::CollisionResult col_res;

    int *robot_joint_id;
    std::pair<int, int> *obstacle_frame_and_id;
    if (!((o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE && o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE) || (o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE && o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE)))
    {
        // std::cout<<"AABB"<<std::endl;
        assert(false);
        hpp::fcl::collide(o1, o2, col_req, col_res);
        return col_res.isCollision();
    }
    else if ((o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE) && (o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE))
    {
        robot_joint_id = (int *)o1->getUserData();
        obstacle_frame_and_id = (std::pair<int, int> *)o2->getUserData();
        (this->collision_pairs->at((*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)))(o1->getTransform(), o2->getTransform(), col_req, col_res);
        // bool res = col_res.isCollision();
        // col_res.clear();
        // hpp::fcl::collide(o1, o2, col_req, col_res);
        // std::cout <<(*robot_joint_id)<<" "<<this->collision_object_count<<std::endl;

        // assert(res == col_res.isCollision());
    }
    else if ((o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE) && (o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE))
    {

        robot_joint_id = (int *)o2->getUserData();
        obstacle_frame_and_id = (std::pair<int, int> *)o1->getUserData();
        (this->collision_pairs->at((*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)))(o2->getTransform(), o1->getTransform(), col_req, col_res);
        // bool res = col_res.isCollision();
        // col_res.clear();
        // hpp::fcl::collide(o1, o2, col_req, col_res);
        // std::cout<<this->collision_pairs<<std::endl;
        // std::cout<<(*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)<<std::endl;
        // std::cout <<(*robot_joint_id)<<" "<<this->collision_object_count<<" "<<obstacle_frame_and_id->first<<" "<<obstacle_frame_and_id->second<<std::endl;
        // std::cout<<o1->getTransform().getTranslation()<<std::endl;
        // std::cout<<o2->getTransform().getTranslation()<<" "<<((hpp::fcl::Capsule*)(o2->collisionGeometry().get()))->radius<<" "<<((hpp::fcl::Capsule*)(o2->collisionGeometry().get()))->halfLength<<std::endl;
        // assert(res == col_res.isCollision());
    }
    else
    {
        assert(false);
    }
    // std::cout<<"sphere"<<std::endl;
    // std::cout<<"robot_joint_id "<<*robot_joint_id<<std::endl;
    // std::cout<<"obstacle_frame_and_id "<<obstacle_frame_and_id->first <<" "<<obstacle_frame_and_id->second<<std::endl;
    // hpp::fcl::collide(o1, o2, col_req, col_res);
    this->is_collided = col_res.isCollision();

    return this->is_collided;
}

bool MDP::SafeIntervalCollisionCallback::collide(hpp::fcl::CollisionObject *o1, hpp::fcl::CollisionObject *o2)
{

    // collide
    hpp::fcl::CollisionRequest col_req;
    col_req.security_margin = 0.000000000000000000001;
    hpp::fcl::CollisionResult col_res;

    int *robot_joint_id;
    std::pair<int, int> *obstacle_frame_and_id;
    if (!((o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE && o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE) || (o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE && o1->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_CAPSULE)))
    {
        std::cout << "AABB" << std::endl;
        assert(false);
        hpp::fcl::collide(o1, o2, col_req, col_res);
        return col_res.isCollision();
    }
    else if (o2->getNodeType() == hpp::fcl::NODE_TYPE::GEOM_SPHERE)
    {
        robot_joint_id = (int *)o1->getUserData();
        obstacle_frame_and_id = (std::pair<int, int> *)o2->getUserData();
        (this->collision_pairs->at((*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)))(o1->getTransform(), o2->getTransform(), col_req, col_res);
    }
    else
    {

        robot_joint_id = (int *)o2->getUserData();
        obstacle_frame_and_id = (std::pair<int, int> *)o1->getUserData();
        (this->collision_pairs->at((*robot_joint_id) * this->collision_object_count + (obstacle_frame_and_id->second)))(o2->getTransform(), o1->getTransform(), col_req, col_res);
    }
    // std::cout<<"sphere"<<std::endl;
    // std::cout<<"robot_joint_id "<<*robot_joint_id<<std::endl;
    // std::cout<<"obstacle_frame_and_id "<<obstacle_frame_and_id->first <<" "<<obstacle_frame_and_id->second<<std::endl;
    // hpp::fcl::collide(o1, o2, col_req, col_res);
    bool is_collided = col_res.isCollision();

    // if is_collision, then add frame to vector
    if (is_collided)
    {
        this->frames_in_collision.push_back(obstacle_frame_and_id->first);
    }

    return false; // return false, because if return true, than broadphase ends.
}

void MDP::CollisionManager::benchmark_broadphase()
{

    std::vector<MDP::ResultsWriter::Timer> timers;
    hpp::fcl::BroadPhaseCollisionManager *original_broadphase_manager = this->broadphase_manager;
    std::vector<hpp::fcl::BroadPhaseCollisionManager *> managers;
    // managers.push_back(new hpp::fcl::NaiveCollisionManager());
    managers.push_back(new hpp::fcl::SSaPCollisionManager());
    managers.push_back(new hpp::fcl::SaPCollisionManager());
    managers.push_back(new hpp::fcl::IntervalTreeCollisionManager());
    hpp::fcl::Vec3f lower_limit, upper_limit;
    lower_limit[0] = -1;
    lower_limit[1] = -1;
    lower_limit[2] = -1;

    upper_limit[0] = -1;
    upper_limit[1] = -1;
    upper_limit[2] = -1;
    hpp::fcl::SpatialHashingCollisionManager<>::computeBound(this->all_spheres, lower_limit, upper_limit);
    hpp::fcl::FCL_REAL cell_size =
        std::min(std::min((upper_limit[0] - lower_limit[0]) / 20,
                          (upper_limit[1] - lower_limit[1]) / 20),
                 (upper_limit[2] - lower_limit[2]) / 20);

    // cell_size = 0.1;
    managers.push_back(
        new hpp::fcl::SpatialHashingCollisionManager<
            hpp::fcl::detail::SparseHashTable<hpp::fcl::AABB, hpp::fcl::CollisionObject *, hpp::fcl::detail::SpatialHash>>(
            cell_size, lower_limit, upper_limit));

    managers.push_back(
        new hpp::fcl::SpatialHashingCollisionManager<
            hpp::fcl::detail::SparseHashTable<hpp::fcl::AABB, hpp::fcl::CollisionObject *, hpp::fcl::detail::SpatialHash>>(
            cell_size, lower_limit, upper_limit));

    managers.push_back(new hpp::fcl::DynamicAABBTreeCollisionManager());
    managers.push_back(new hpp::fcl::DynamicAABBTreeArrayCollisionManager());

    {
        hpp::fcl::DynamicAABBTreeCollisionManager *m = new hpp::fcl::DynamicAABBTreeCollisionManager();
        m->tree_init_level = 2;
        managers.push_back(m);
    }

    {
        hpp::fcl::DynamicAABBTreeArrayCollisionManager *m =
            new hpp::fcl::DynamicAABBTreeArrayCollisionManager();
        m->tree_init_level = 2;
        managers.push_back(m);
    }

    {
        hpp::fcl::DynamicAABBTreeArrayCollisionManager *m =
            new hpp::fcl::DynamicAABBTreeArrayCollisionManager();
        m->tree_init_level = 12;
        m->max_tree_nonbalanced_level = 0;

        managers.push_back(m);
    }
    {
        hpp::fcl::DynamicAABBTreeArrayCollisionManager *m =
            new hpp::fcl::DynamicAABBTreeArrayCollisionManager();
        m->tree_init_level = 3;
        managers.push_back(m);
    }
    {
        hpp::fcl::DynamicAABBTreeArrayCollisionManager *m =
            new hpp::fcl::DynamicAABBTreeArrayCollisionManager();
        m->tree_init_level = 4;
        managers.push_back(m);
    }
    {
        hpp::fcl::DynamicAABBTreeArrayCollisionManager *m =
            new hpp::fcl::DynamicAABBTreeArrayCollisionManager();
        m->tree_init_level = 11;
        m->max_tree_nonbalanced_level = 3;
        managers.push_back(m);
    }

    timers.resize(managers.size());

    for (size_t i = 0; i < managers.size(); ++i)
    {
        timers[i].reset();
        managers[i]->registerObjects(this->all_spheres);
        timers[i].pause_timer();
        std::cout << i << "manager register time: " << timers[i].get_elapsed_time_ns() << std::endl;
    }

    for (size_t i = 0; i < managers.size(); ++i)
    {
        timers[i].reset();
        managers[i]->setup();
        timers[i].pause_timer();
        std::cout << i << "manager setup time: " << timers[i].get_elapsed_time_ns() << std::endl;
    }
    std::random_device r;
    std::mt19937 gen(r());
    std::uniform_real_distribution<> probability_gen(0.0, 1.0);

    std::vector<std::pair<float, float>> robot_limits = this->get_planned_robot_limits();
    std::vector<double> robot_angles;
    std::vector<long long> results;
    results.resize(managers.size());
    for (int i = 0; i < results.size(); i++)
    {
        results[i] = 0;
    }

    // delete this->broadphase_manager;

    robot_angles.resize(6);
    long long naive_si_time = 0;
    for (int j = 0; j < 100000; j++)
    {
        std::cout << "------------------------------" << std::endl;
        for (int i = 0; i < 6; i++)
        {
            robot_angles[i] = (probability_gen(gen) * (robot_limits[i].second - robot_limits[i].first) + robot_limits[i].first);
            std::cout << robot_angles[i] << ", ";
        }
        std::cout << std::endl;

        MDP::ResultsWriter::Timer rt;
        rt.reset();
        std::vector<std::pair<int, int>> res = this->get_safe_intervals_naive(robot_angles);
        rt.pause_timer();
        std::cout << "real interval: " << rt.get_elapsed_time_ns() << " " << res.size() << std::endl;
        for (std::pair<int, int> r : res)
        {
            std::cout << "[" << r.first << " " << r.second << "]  ";
        }
        std::cout << std::endl;
        rt.reset();
        res = this->get_safe_intervals_naive_fastcc(robot_angles);
        rt.pause_timer();
        naive_si_time+=rt.get_elapsed_time_ns();
        std::cout << "real interval: " << rt.get_elapsed_time_ns() << " " << res.size() << std::endl;
        for (std::pair<int, int> r : res)
        {
            std::cout << "[" << r.first << " " << r.second << "]  ";
        }
        std::cout << std::endl;

        rt.reset();
        res = this->get_safe_intervals_naive_fastcc_oop(robot_angles);
        rt.pause_timer();
        std::cout << "real interval: " << rt.get_elapsed_time_ns() << " " << res.size() << std::endl;
        for (std::pair<int, int> r : res)
        {
            std::cout << "[" << r.first << " " << r.second << "]  ";
        }
        std::cout << std::endl;
        for (size_t i = 0; i < managers.size(); i++)
        {
            timers[i].reset();
            this->broadphase_manager = managers[i];
            std::vector<std::pair<int, int>> ints = this->get_safe_intervals(robot_angles);
            timers[i].pause_timer();
            results[i] += timers[i].get_elapsed_time_ns();
            std::cout << i << "manager collide time: " << timers[i].get_elapsed_time_ns() << " safe_interval_size: " << ints.size() << std::endl;
            for (std::pair<int, int> r : ints)
            {
                std::cout << "[" << r.first << " " << r.second << "]  ";
            }
            std::cout << std::endl;
        }
    }
    
    std::cout <<  "naive_si_time: " << naive_si_time << std::endl;

    for (int i = 0; i < results.size(); i++)
    {
        std::cout << i << "manager collide time: " << results[i] << std::endl;
    }
}
