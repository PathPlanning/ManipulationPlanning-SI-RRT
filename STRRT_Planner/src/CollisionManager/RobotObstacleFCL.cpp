#include <coal/shape/geometric_shapes.h>
#include <vector>
#include <cassert>
#include <urdf/model.h>
#include "coal/math/transform.h"
#include "coal/mesh_loader/loader.h"
#include "coal/BVH/BVH_model.h"
#include <CollisionManager/RobotObstacleFCL.hpp>
#include <CollisionManager/CubeObstacleFCL.hpp>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/RobotObstacleJsonInfo.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include "config_read_writer/ResultsWriter.hpp"

MDP::RobotObstacleFCL::RobotObstacleFCL(const std::string &urdf_path, const std::vector<MDP::ObstacleCoordinate> robot_base_position, const std::vector<std::string> _robot_joint_order, const std::vector<MDP::RobotObstacleJsonInfo::PathState> _trajectory, const MDP::ConfigReader::SceneTask &_scene_task, const bool _is_static) : limits(_robot_joint_order.size()), robot_joint_order(_robot_joint_order), base_position(robot_base_position), trajectory(_trajectory), scene_task(_scene_task), is_static(_is_static)
{

    this->load_urdf_from_file(urdf_path); // load urdf model
    const size_t last_slash_idx = urdf_path.rfind('/');
    const std::string urdf_root_path = urdf_path.substr(0, last_slash_idx) + "/";
    this->load_joint_collision_models(urdf_root_path); // load collision models
    this->load_kdl_tree(urdf_path);                    // load kdl tree
    transforms.resize(link2mesh.size());
}

MDP::RobotObstacleFCL::~RobotObstacleFCL() {}; // destructor

void MDP::RobotObstacleFCL::load_kdl_tree(const std::string &urdf_path)
{

    if (!kdl_parser::treeFromFile(urdf_path, this->robot_tree))
        throw std::runtime_error("Failed to construct kdl tree");

    this->robot_tree.getChain(this->robot_tree.getRootSegment()->second.segment.getName(), this->tip_link_name, this->robot_chain);
    this->degrees_of_freedom = this->robot_chain.getNrOfJoints();
    initial_base_frame = this->robot_tree.getRootSegment()->second.segment.getFrameToTip();
}

void MDP::RobotObstacleFCL::load_urdf_from_file(const std::string &urdf_path)
{

    if (!this->model.initFile(urdf_path))
    {
        throw std::runtime_error("Failed to parse urdf file");
    }
}

void MDP::RobotObstacleFCL::load_joint_collision_models(const std::string &urdf_root_path)
{

    urdf::LinkSharedPtr current_link = model.root_link_;
    coal::NODE_TYPE bv_type = coal::BV_AABB;
    coal::MeshLoader loader(bv_type);
    while (current_link.get())
    {
        if (!current_link->collision.get())
        {
            if (current_link->child_links.size() > 0)
            {
                assert(current_link->child_joints.size() > 0);

                urdf::JointSharedPtr curr_joint = current_link->child_joints.at(0);
                if (curr_joint->limits)
                {
                    this->joint_name2limits.emplace(curr_joint->name, std::pair<float, float>(curr_joint->limits->lower, curr_joint->limits->upper));
                }

                int joint_ind = -1;
                for (int i = 0; i < robot_joint_order.size(); i++)
                {
                    if (curr_joint->name == robot_joint_order.at(i))
                    {
                        joint_ind = i;
                        if (curr_joint->limits)
                        {
                            limits[i] = std::pair<float, float>((float)curr_joint->limits->lower, (float)curr_joint->limits->upper);
                        }

                        break;
                    }
                }
                if (joint_ind != -1)
                {
                    joint_name2ind[curr_joint->name] = joint_ind;
                }
                Eigen::Vector3d joint_origin_translation;
                joint_origin_translation[0] = curr_joint->parent_to_joint_origin_transform.position.x;
                joint_origin_translation[1] = curr_joint->parent_to_joint_origin_transform.position.y;
                joint_origin_translation[2] = curr_joint->parent_to_joint_origin_transform.position.z;

                double x, y, z, w;
                curr_joint->parent_to_joint_origin_transform.rotation.getQuaternion(x, y, z, w);
                Eigen::Quaterniond q(w, x, y, z);
                Eigen::Matrix3d joint_origin_rotation = q.toRotationMatrix();
                this->fk_chain.emplace_back(std::shared_ptr<coal::ShapeBase>(nullptr), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), joint_origin_translation, joint_origin_rotation, current_link->name, curr_joint->name, curr_joint->type != urdf::Joint::FIXED, true);
                current_link = current_link->child_links.at(0);
            }
            else
            {
                this->fk_chain.emplace_back(std::shared_ptr<coal::ShapeBase>(nullptr), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), current_link->name, "", false, false);
                this->tip_link_name = current_link->name;
                current_link = nullptr;
            }
            continue;
        }
        if (current_link->collision->geometry->type == urdf::Geometry::MESH)
        {
            const auto mesh_ptr{dynamic_cast<const urdf::Mesh *>(current_link->collision->geometry.get())};
            link_name2filepath[current_link->name] = urdf_root_path + mesh_ptr->filename;
            coal::BVHModelPtr_t bvh = loader.load(urdf_root_path + mesh_ptr->filename);
            bvh->buildConvexHull(true, "Qt");
            std::shared_ptr<coal::ConvexBase32> link_mesh = bvh->convex;
            // std::shared_ptr<coal::ConvexBase32> link_mesh = loadConvexMesh(urdf_root_path + mesh_ptr->filename);
            link2mesh[current_link->name] = link_mesh;
            collision_objects.push_back(link_mesh);
        }
        else if (current_link->collision->geometry->type == urdf::Geometry::BOX)
        {
            const auto box_ptr{dynamic_cast<const urdf::Box *>(current_link->collision->geometry.get())};
            std::shared_ptr<coal::Box> link_box(new coal::Box(box_ptr->dim.x, box_ptr->dim.y, box_ptr->dim.z));
            link2mesh[current_link->name] = link_box;
            collision_objects.push_back(link_box);
        }
        else if (current_link->collision->geometry->type == urdf::Geometry::CYLINDER)
        {
            const auto cylinder_ptr{dynamic_cast<const urdf::Cylinder *>(current_link->collision->geometry.get())};
            // std::shared_ptr<coal::Cylinder> link_cylinder(new coal::Cylinder(cylinder_ptr->radius,cylinder_ptr->length));
            std::shared_ptr<coal::Capsule> link_cylinder(new coal::Capsule(cylinder_ptr->radius, cylinder_ptr->length));
            link2mesh[current_link->name] = link_cylinder;
            collision_objects.push_back(link_cylinder);

            coal::Transform3s origin_transform;
            coal::Vec3s translation(current_link->collision->origin.position.x, current_link->collision->origin.position.y, current_link->collision->origin.position.z);
            double x{0}, y{0}, z{0}, w{0};
            current_link->collision->origin.rotation.getQuaternion(x, y, z, w);
            coal::Quaternion3f q(w, x, y, z);
            origin_transform.setQuatRotation(q);
            origin_transform.setTranslation(translation);
            link2transform[current_link->name] = origin_transform;
        }

        if (current_link->child_links.size() > 0)
        {
            assert((current_link->child_joints.size() > 0));
            urdf::JointSharedPtr curr_joint = current_link->child_joints.at(0);
            if (curr_joint->limits)
            {
                this->joint_name2limits.emplace(curr_joint->name, std::pair<float, float>(curr_joint->limits->lower, curr_joint->limits->upper));
            }
            int joint_ind = -1;
            for (int i = 0; i < robot_joint_order.size(); i++)
            {
                if (curr_joint->name == robot_joint_order.at(i))
                {
                    joint_ind = i;
                    if (curr_joint->limits)
                    {
                        limits[i] = std::pair<float, float>((float)curr_joint->limits->lower, (float)curr_joint->limits->upper);
                    }

                    break;
                }
            }
            if (joint_ind != -1)
            {
                joint_name2ind[curr_joint->name] = joint_ind;
            }
            Eigen::Vector3d joint_origin_translation;
            joint_origin_translation[0] = curr_joint->parent_to_joint_origin_transform.position.x;
            joint_origin_translation[1] = curr_joint->parent_to_joint_origin_transform.position.y;
            joint_origin_translation[2] = curr_joint->parent_to_joint_origin_transform.position.z;

            double x, y, z, w;
            curr_joint->parent_to_joint_origin_transform.rotation.getQuaternion(x, y, z, w);
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d joint_origin_rotation = q.toRotationMatrix();
            this->fk_chain.emplace_back(link2mesh[current_link->name], link2transform[current_link->name].getTranslation(), link2transform[current_link->name].getRotation(), joint_origin_translation, joint_origin_rotation, current_link->name, curr_joint->name, curr_joint->type != urdf::Joint::FIXED, true);

            current_link = current_link->child_links.at(0);
        }
        else
        {
            this->fk_chain.emplace_back(link2mesh[current_link->name], link2transform[current_link->name].getTranslation(), link2transform[current_link->name].getRotation(), Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero(), current_link->name, "", false, false);

            this->tip_link_name = current_link->name;
            current_link = nullptr;
        }
    }
}

std::vector<MDP::RobotObstacleFCL::JointCollisionObject> MDP::RobotObstacleFCL::get_collision_object_for_robot_angles(const std::vector<double> angles)
{

    return MDP::ResultsWriter::get_instance().forward_kinematics_check_wrapper(std::bind(&MDP::RobotObstacleFCL::get_collision_object_for_robot_angles_private, this, std::placeholders::_1), angles);
}

std::vector<MDP::RobotObstacleFCL::JointCollisionObject> MDP::RobotObstacleFCL::get_collision_object_for_robot_angles_private_naive(const std::vector<float> angles)
{
    this->robot_chain.segments[0].setFrameToTip(KDL::Frame(this->initial_base_frame.M * KDL::Rotation::Quaternion(base_position[0].quat_x, base_position[0].quat_y, base_position[0].quat_z, base_position[0].quat_w), this->initial_base_frame.p + KDL::Vector(base_position[0].pos[0], base_position[0].pos[1], base_position[0].pos[2]))); // TODO: remove [0] and check if robot is static

    // calculate FK
    // KDL::TreeFkSolverPos_recursive tree_fk_solver(this->robot_tree);
    KDL::ChainFkSolverPos_recursive chain_fk_solver(this->robot_chain);
    std::vector<KDL::Frame> frames_fk(robot_chain.getNrOfSegments());
    KDL::JntArray joint_pos{this->degrees_of_freedom};

    for (size_t i = 0; i < this->degrees_of_freedom; i++)
    {
        joint_pos(i) = 0;
    }

    for (int i = 0; i < robot_joint_order.size(); i++)
    {
        joint_pos(joint_name2ind.at(robot_joint_order.at(i))) = angles.at(i);
    }

    for (size_t i = 0; i < this->robot_chain.getNrOfSegments(); i++)
    {
        KDL::Frame cart_pos{};
        assert(angles.size() == robot_chain.getNrOfJoints());
        chain_fk_solver.JntToCart(joint_pos, cart_pos, i + 1);

        frames_fk.at(i) = cart_pos;
    }

    // calculate FCL transforms

    KDL::Frame tf{};
    std::vector<JointCollisionObject> result;
    for (size_t i = 2; i < this->robot_chain.getNrOfSegments(); i++)
    {
        std::string link_name = robot_chain.getSegment(i).getName();
        // std::cout<<link_name<<std::endl;
        if (link2mesh.find(link_name) == link2mesh.end())
        {
            continue;
        }

        tf = frames_fk.at(i);

        double x{0}, y{0}, z{0}, w{0};
        tf.M.GetQuaternion(x, y, z, w);
        coal::Vec3s t(tf.p[0], tf.p[1], tf.p[2]);
        coal::Quaternion3f q(w, x, y, z);

        coal::Transform3s origin_transform = link2transform[link_name];

        // URDF order: first translation, then rotation.
        transforms.at(i).setQuatRotation(q);
        // origin_transform.getTranslation() is a translation relative to joint local basis, need to convert to world basis
        transforms.at(i).setTranslation(coal::Vec3s(0, 0, 0));

        coal::Vec3s origin_in_world_basis = transforms.at(i).transform(origin_transform.getTranslation());
        transforms.at(i).setTranslation(t + origin_in_world_basis);
        // std::cout<<"joint_translation" <<std::endl;
        // std::cout<<t <<std::endl;
        // std::cout<<"origin_in_world_basis" <<std::endl;
        // std::cout<<origin_in_world_basis <<std::endl;
        // std::cout<<"link_trans" <<std::endl;
        // std::cout<<transforms.at(i).getTranslation() <<std::endl;

        // std::cout<<"joint_rotation" <<std::endl;
        // std::cout<<transforms.at(i).getRotation()<<std::endl;
        // std::cout<<"origin_rotation" <<std::endl;
        // std::cout<<origin_transform.getRotation()<<std::endl;

        // DON'T TOUCH OR OPTIMISE!!! EIGEN MULTIPLICATION WORKS CORRECTLY ONLY THIS WAY
        coal::Matrix3s rot = transforms.at(i).getRotation();
        rot = rot * origin_transform.getRotation();
        transforms.at(i).setRotation(rot);
        ///////
        // std::cout<<"combined_rotation" <<std::endl;
        // std::cout<<transforms.at(i).getRotation()<<std::endl;

        result.push_back(JointCollisionObject(link_name, link2mesh.at(link_name), transforms.at(i), i));
    }

    return result;
}

std::vector<MDP::RobotObstacleFCL::JointCollisionObject> MDP::RobotObstacleFCL::get_collision_object_for_robot_angles_private(const std::vector<double> angles)
{

    std::vector<float> angles_f;
    // for (int i = 0; i < angles.size(); i++)
    // {
    //     angles_f.push_back((float)angles[i]);
    // }
    // std::vector<JointCollisionObject> result1 = this->get_collision_object_for_robot_angles_private_naive(angles_f);
    std::vector<JointCollisionObject> result;
    Eigen::Vector3d translation;
    translation.setZero();
    translation[0] = this->base_position[0].x;
    translation[1] = this->base_position[0].y;
    translation[2] = this->base_position[0].z;
    Eigen::Matrix3d rotation(this->base_position[0].rotation.toRotationMatrix());
    // rotation.setZero();
    // rotation.
    // rotation.diagonal() << 1, 1, 1;
    int dof = 0;
    int link_id = 0;
    for (size_t i = 0; i < this->fk_chain.size(); i++)
    {
        std::string link_name = this->fk_chain[i].link_name;
        // std::cout<<link_name<<std::endl;

        if (this->fk_chain[i].link_mesh == nullptr)
        {
            if (this->fk_chain[i].has_next)
            {
                if (this->fk_chain[i].not_fixed)
                {
                    translation += rotation * this->fk_chain[i].joint_origin_translation;
                    rotation = rotation * this->fk_chain[i].joint_origin_rotation;
                    rotation = rotation * Eigen::AngleAxisd(angles[dof], Eigen::Vector3d::UnitZ());

                    dof++;
                }
            }
            continue;
        }

        // coal::Quaternion3f q(rotation);

        // URDF order: first translation, then rotation.
        // transforms.at(link_id).setQuatRotation(q);
        // origin_transform.getTranslation() is a translation relative to joint local basis, need to convert to world basis

        coal::Vec3s origin_in_world_basis = rotation*this->fk_chain[i].link_origin_translation;
        transforms.at(link_id).setTranslation(translation + origin_in_world_basis);
        // std::cout << "link_name: " + link_name << std::endl;

        // std::cout << "joint_rotation" << std::endl;
        // std::cout << transforms.at(link_id).getRotation() << std::endl;
        // std::cout << "joint translation " << std::endl;
        // std::cout << transforms.at(link_id).getTranslation() << std::endl;

        // std::cout << "origin_rotation" << std::endl;
        // std::cout << rotation*this->fk_chain[i].link_origin_rotation << std::endl;

        transforms.at(link_id).setRotation(rotation*this->fk_chain[i].link_origin_rotation);
        // std::cout<<"combined_rotation" <<std::endl;
        // std::cout<<transforms.at(link_id).getRotation()<<std::endl;
        // std::cout<<"joint number "<<result.size()<<std::endl;
        // std::cout<<"joint pos"<<transforms.at(link_id).getTranslation()<<std::endl;
        // std::cout<<"joint rot"<<transforms.at(link_id).getQuatRotation()<<std::endl;

        result.push_back(JointCollisionObject(link_name, this->fk_chain[i].link_mesh, transforms.at(link_id), link_id));
        link_id++;

        if (this->fk_chain[i].has_next)
        {
            if (this->fk_chain[i].not_fixed)
            {
                translation += rotation * this->fk_chain[i].joint_origin_translation;
                rotation = rotation * this->fk_chain[i].joint_origin_rotation;
                rotation = rotation * Eigen::AngleAxisd(angles[dof], Eigen::Vector3d::UnitZ());

                dof++;
            }
        }
    }

    // result.erase(result.begin(), result.begin() + 2); // Delete base links models

    // assert(result1.size() == result.size());

    // for (int joint_ind = 0; joint_ind < result1.size(); joint_ind++)
    // {
    //     assert((result1[joint_ind].transform.getRotation() - result[joint_ind].transform.getRotation()).norm()<0.0001);
    //     assert((result1[joint_ind].transform.getTranslation() - result[joint_ind].transform.getTranslation()).norm()<0.0001);
    // }
    // for (int joint_ind = 0; joint_ind < result.size(); joint_ind++)
    // {
    //     std::cout<<result[joint_ind].transform.getTranslation()<<std::endl;
    //     std::cout<<result[joint_ind].transform.getQuatRotation()<<std::endl;   
    // }

    return result;
}

void MDP::RobotObstacleFCL::set_trajectory(const std::vector<MDP::RobotObstacleJsonInfo::PathState> _trajectory)
{
    this->trajectory = _trajectory;
}

std::vector<MDP::RobotObstacleFCL::JointCollisionObject> MDP::RobotObstacleFCL::get_collision_object_for_frame(const long long frame)
{
    assert(this->trajectory.size()>0);
    assert(frame < this->scene_task.frame_count);
    assert(frame >= 0);

    double time = (double)frame/(double)this->scene_task.fps+this->scene_task.start_time;

    if (trajectory.back().time<=time+0.00001){
        return this->get_collision_object_for_robot_angles(trajectory.back().configuration_coordinates);
    }
    for (int state_idx = 0; state_idx < this->trajectory.size() - 1; state_idx++)
    {
        if (this->trajectory[state_idx].time <= time +0.00001 &&
            this->trajectory[state_idx + 1].time > time+0.00001)
        {
            std::vector<double> result_coord;
            std::vector<double> left_coord = this->trajectory[state_idx].configuration_coordinates;
            double left_time = this->trajectory[state_idx].time;

            std::vector<double> right_coord = this->trajectory[state_idx+1].configuration_coordinates;
            double right_time = this->trajectory[state_idx+1].time;
            for (int angle_id = 0; angle_id < right_coord.size(); angle_id++)
            {
                double k = (right_coord[angle_id] - left_coord[angle_id]) / (right_time - left_time);
                double b = right_coord[angle_id] - (right_coord[angle_id] - left_coord[angle_id]) / (right_time - left_time) * right_time;
                result_coord.push_back(k * time + b);
            }
            return this->get_collision_object_for_robot_angles(result_coord);
        }
    }
    // std::cout<<"frame: "<<frame<<" time: "<<time<<" frame_count: "<<this->scene_task.frame_count<<" trajectory.back().time: "<<trajectory.back().time<<std::endl;
    // for (int state_idx = 0; state_idx < this->trajectory.size() - 1; state_idx++)
    // {
    //     std::cout<<"state_idx: "<<state_idx<<" time: "<<this->trajectory[state_idx].time<<" frame: "<<this->to_frame(this->trajectory[state_idx].time)<<std::endl;
    // }
    assert(false);

    // this->robot_chain.segments[0].setFrameToTip(KDL::Frame(this->initial_base_frame.M * KDL::Rotation::Quaternion(base_position[0].quat_x, base_position[0].quat_y, base_position[0].quat_z, base_position[0].quat_w), this->initial_base_frame.p + KDL::Vector(base_position[0].pos[0], base_position[0].pos[1], base_position[0].pos[2]))); // TODO: remove [0] and check if robot is static

    // if (this->to_frame(trajectory.back().time)<=frame){
    //     return this->get_collision_object_for_robot_angles(trajectory.back().configuration_coordinates);
    // }
    //do binary search and find interval between times, where frame is located, then, linearly interpolate angles


    // int left_bound = trajectory.size()/2;
    // for (int interval_id=0; interval_id<trajectory.size()-1; interval_id++){
    //     if (frame < this->to_frame(trajectory[left_bound].time)){
    //         //move bound to the right
    //         left_bound += (trajectory.size() - left_bound) /2;

    //     }
    //     else if (frame > this->to_frame(trajectory[left_bound+1].time)){
    //         //move bound to the left
    //         left_bound -= (left_bound) /2;

    //     }
    //     else{
    //         //linearly interpolate and return
    //         std::vector<double> result_coord;
    //         std::vector<double> left_coord = trajectory[left_bound].configuration_coordinates;
    //         int left_frame = this->to_frame(trajectory[left_bound].time);
    //         std::vector<double> right_coord = trajectory[left_bound+1].configuration_coordinates;
    //         int right_frame = this->to_frame(trajectory.[left_bound+1].time);
    //         for(int angle_id=0;angle_id<left_coord.size();angle_id++){
    //             double k = (right_coord[angle_id]-left_coord[angle_id])/(right_frame-left_frame);
    //             double b = right_coord[angle_id]- (right_coord[angle_id]-left_coord[angle_id])/(right_frame-left_frame)*right_frame;
    //             result_coord.push_back(k*frame+b);
    //         }
    //         return this->get_collision_object_for_robot_angles(result_coord);
    //     }
    // }


    // Binary search to find interval between times where frame is located, then linearly interpolate angles
    // int left = 0;
    // int right = trajectory.size() - 1;
    // int left_bound = -1;

    // // Binary search for the correct interval
    // while (left <= right) {
    //     int mid = left + (right - left) / 2;
    //     int mid_frame = this->to_frame(trajectory[mid].time);

    //     if (mid < trajectory.size() - 1) {
    //         int next_frame = this->to_frame(trajectory[mid + 1].time);
    //         // Check if frame is in the interval [mid, mid+1]
    //         if (frame >= mid_frame && frame <= next_frame) {
    //             left_bound = mid;
    //             break;
    //         }
    //         else if (frame < mid_frame) {
    //             right = mid - 1;
    //         }
    //         else {
    //             left = mid + 1;
    //         }
    //     }
    //     else {
    //         // Handle edge case where mid is the last element
    //         if (frame >= mid_frame) {
    //             // assert(false);
    //             left_bound = mid - 1; // Use previous interval
    //         }
    //         else {
    //             right = mid - 1;
    //         }
    //         break;
    //     }
    // }

    // // Check if we found a valid interval
    // if (left_bound >= 0 && left_bound < trajectory.size() - 1) {
    //     // Linearly interpolate and return
    //     std::vector<double> result_coord;
    //     std::vector<double> left_coord = trajectory[left_bound].configuration_coordinates;
    //     int left_frame = this->to_frame(trajectory[left_bound].time);
    //     std::vector<double> right_coord = trajectory[left_bound + 1].configuration_coordinates;
    //     int right_frame = this->to_frame(trajectory[left_bound + 1].time); // Fixed syntax error

    //     for (int angle_id = 0; angle_id < left_coord.size(); angle_id++) {
    //         // Linear interpolation: y = y1 + (y2-y1) * (x-x1) / (x2-x1)
    //         double interpolated_value = left_coord[angle_id] +
    //             (right_coord[angle_id] - left_coord[angle_id]) *
    //             (frame - left_frame) / (right_frame - left_frame);
    //         result_coord.push_back(interpolated_value);
    //     }
    //     return this->get_collision_object_for_robot_angles(result_coord);
    // }


    // assert(false);

    // for (int state_idx = 0; state_idx < this->trajectory.size() - 1; state_idx++)
    // {
    //     if (to_frame(this->trajectory[state_idx].time) <= frame &&
    //         to_frame(this->trajectory[state_idx + 1].time) >= frame)
    //     {   
    //         std::vector<double> result_coord;
    //         std::vector<double> left_coord = this->trajectory[state_idx].configuration_coordinates;
    //         int left_frame = to_frame(this->trajectory[state_idx].time);

    //         std::vector<double> right_coord = this->trajectory[state_idx+1].configuration_coordinates;
    //         int right_frame = to_frame(this->trajectory[state_idx+1].time);
    //         for (int angle_id = 0; angle_id < right_coord.size(); angle_id++)
    //         {
    //             double k = (right_coord[angle_id] - left_coord[angle_id]) / (right_frame - left_frame);
    //             double b = right_coord[angle_id] - (right_coord[angle_id] - left_coord[angle_id]) / (right_frame - left_frame) * right_frame;
    //             result_coord.push_back(k * frame + b);
    //         }
    //         return this->get_collision_object_for_robot_angles(result_coord);
    //     }
    // }
    // assert(false);
}

int MDP::RobotObstacleFCL::to_frame(const double& time) const{
    return (time - this->scene_task.start_time)*this->scene_task.fps;
}
std::vector<std::pair<float, float>> MDP::RobotObstacleFCL::get_limits() const
{
    return limits;
}

std::vector<std::shared_ptr<coal::ShapeBase>> MDP::RobotObstacleFCL::get_geometric_shapes()
{
    return collision_objects;
}

std::string MDP::RobotObstacleFCL::get_urdf_path() const
{
    return this->urdf_path;
}

const std::vector<MDP::ObstacleCoordinate>& MDP::RobotObstacleFCL::get_base_position() const
{
    return this->base_position;
}

const std::vector<MDP::RobotObstacleJsonInfo::PathState>& MDP::RobotObstacleFCL::get_trajectory() const
{
    return this->trajectory;
}

bool MDP::RobotObstacleFCL::get_is_static() const
{
    return this->is_static;
}

std::string MDP::RobotObstacleFCL::get_filepath_for_link_name(const std::string& link_name) const
{
    assert(this->link_name2filepath.find(link_name)!=this->link_name2filepath.end());
    return this->link_name2filepath.find(link_name)->second;
}