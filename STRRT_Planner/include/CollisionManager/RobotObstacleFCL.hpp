#pragma once
#include <hpp/fcl/shape/geometric_shapes.h>
#include <vector>
#include <Eigen/Core>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>

namespace MDP
{

    class RobotObstacleFCL
    {
    public:
        struct JointCollisionObject
        {

            std::string name;
            std::shared_ptr<hpp::fcl::CollisionGeometry> collision_object;
            hpp::fcl::Transform3f transform;
            int joint_number;
            JointCollisionObject(std::string _name, std::shared_ptr<hpp::fcl::CollisionGeometry> _collision_object, hpp::fcl::Transform3f _transform, unsigned int _joint_number) : name(_name), collision_object(_collision_object), transform(_transform), joint_number(_joint_number) {};
            ~JointCollisionObject() = default;                                 // destructor
            JointCollisionObject(const JointCollisionObject &other) = default; // copy constructor
            JointCollisionObject(JointCollisionObject &&other) = default;      // move constructor

            JointCollisionObject &operator=(JointCollisionObject &other) = default;  // copy assignment
            JointCollisionObject &operator=(JointCollisionObject &&other) = default; // move assignment
        };
        struct ForwardKinematicChain
        {
            ForwardKinematicChain(std::shared_ptr<hpp::fcl::ShapeBase> link_mesh_,
                                  const Eigen::Vector3d link_origin_translation_,
                                  const Eigen::Matrix3d link_origin_rotation_,
                                  const Eigen::Vector3d joint_origin_translation_,
                                  const Eigen::Matrix3d joint_origin_rotation_,
                                  const std::string link_name_,
                                  const std::string joint_name_,
                                  const bool not_fixed_,
                                  const bool has_next_) : link_mesh(link_mesh_), link_origin_translation(link_origin_translation_),
                                                          link_origin_rotation(link_origin_rotation_), joint_origin_translation(joint_origin_translation_),
                                                          joint_origin_rotation(joint_origin_rotation_), link_name(link_name_), joint_name(joint_name_),
                                                          not_fixed(not_fixed_), has_next(has_next_) {};

            std::shared_ptr<hpp::fcl::ShapeBase> link_mesh;
            const Eigen::Vector3d link_origin_translation;
            const Eigen::Matrix3d link_origin_rotation;
            const Eigen::Vector3d joint_origin_translation;
            const Eigen::Matrix3d joint_origin_rotation;
            const std::string link_name;
            const std::string joint_name;
            const bool not_fixed;
            const bool has_next;
        };
        RobotObstacleFCL(const std::string &urdf_path,
                         const std::vector<MDP::ObstacleCoordinate> robot_base_position,
                         const std::vector<std::string> _robot_joints_order,
                         const std::vector<std::vector<double>> _trajectory); // constructor
        ~RobotObstacleFCL();                                                  // destructor
        RobotObstacleFCL(const RobotObstacleFCL &other) = default;            // copy constructor
        RobotObstacleFCL(RobotObstacleFCL &&other) = default;                 // move constructor
        RobotObstacleFCL &operator=(const RobotObstacleFCL &other) = default; // copy assignment
        RobotObstacleFCL &operator=(RobotObstacleFCL &&other) = default;      // move assignment

        // void set_position(const unsigned int frame);
        // void set_position(const MDP::ObstacleCoordinate position);
        // hpp::fcl::Box& get_collision_object() const;
        std::vector<MDP::RobotObstacleFCL::JointCollisionObject> get_collision_object_for_robot_angles(const std::vector<double> angles);
        std::vector<MDP::RobotObstacleFCL::JointCollisionObject> get_collision_object_for_frame(const long long frame);

        std::vector<std::shared_ptr<hpp::fcl::ShapeBase>> get_geometric_shapes();
        hpp::fcl::Transform3f &get_transform() const;

        void set_trajectory(const std::vector<std::vector<double>> _trajectory);
        std::vector<std::pair<float, float>> get_limits() const;

    private:
        std::vector<ForwardKinematicChain> fk_chain;
        std::vector<MDP::RobotObstacleFCL::JointCollisionObject> get_collision_object_for_robot_angles_private(const std::vector<double> angles);
        std::vector<MDP::RobotObstacleFCL::JointCollisionObject> get_collision_object_for_robot_angles_private_naive(const std::vector<float> angles);

        KDL::Frame initial_base_frame;
        std::string urdf_path;
        mutable std::vector<hpp::fcl::Transform3f> transforms;
        urdf::Model model{};
        const std::vector<std::string> robot_joint_order;
        std::vector<MDP::ObstacleCoordinate> base_position;
        std::vector<std::vector<double>> trajectory;
        std::vector<std::pair<float, float>> limits;

        void load_kdl_tree(const std::string &urdf_path);
        void load_urdf_from_file(const std::string &urdf_path);
        void load_joint_collision_models(const std::string &urdf_root_path);
        // std::shared_ptr<hpp::fcl::ShapeBase> loadConvexMesh(const std::string &file_name);

        std::map<std::string, std::shared_ptr<hpp::fcl::ShapeBase>> link2mesh;
        std::map<std::string, hpp::fcl::Transform3f> link2transform;
        std::map<std::string, int> joint_name2ind;
        std::map<std::string, std::pair<float, float>> joint_name2limits;
        std::vector<std::shared_ptr<hpp::fcl::ShapeBase>> collision_objects;
        std::string tip_link_name;
        unsigned int degrees_of_freedom;
        KDL::Tree robot_tree;
        KDL::Chain robot_chain;
    };
}