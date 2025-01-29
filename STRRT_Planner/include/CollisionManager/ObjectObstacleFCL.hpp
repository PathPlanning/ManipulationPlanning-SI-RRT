#pragma once
#include <hpp/fcl/shape/geometric_shapes.h>
#include <vector>
#include <Eigen/Core>
#include <CollisionManager/RobotObstacleFCL.hpp>
#include <config_read_writer/config_read.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>
namespace MDP
{

    class ObjectObstacleFCL
    {
    public:
        ObjectObstacleFCL(const std::vector<MDP::ObstacleCoordinate> positions, const std::string _name, hpp::fcl::ShapeBase* _collision_object, const bool _is_static);
        ~ObjectObstacleFCL();                                                 // destructor
        ObjectObstacleFCL(const ObjectObstacleFCL &other) = delete;            // copy constructor
        ObjectObstacleFCL(ObjectObstacleFCL &&other) = default;                 // move constructor
        ObjectObstacleFCL &operator=(const ObjectObstacleFCL &other) = delete; // copy assignment
        ObjectObstacleFCL &operator=(ObjectObstacleFCL &&other) = default;      // move assignment

        void set_position(const unsigned int frame) const;
        void set_position(const MDP::ObstacleCoordinate position) const;
        hpp::fcl::ShapeBase* get_collision_object() const;
        MDP::ObstacleCoordinate get_position(const unsigned int frame) const;
        hpp::fcl::Transform3f get_transform() const;
        std::string get_name() const;
        bool get_is_static() const;

    private:
        mutable hpp::fcl::ShapeBase* collision_object; //TODO remove all mutables because they are quickfix
        std::vector<MDP::ObstacleCoordinate> positions;
        mutable hpp::fcl::Transform3f transform;
        std::string name;
        const bool is_static;
    };
}