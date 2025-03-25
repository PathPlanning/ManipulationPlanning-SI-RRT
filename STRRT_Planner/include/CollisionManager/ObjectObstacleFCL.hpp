#pragma once
#include <coal/shape/geometric_shapes.h>
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
        ObjectObstacleFCL(const std::vector<MDP::ObstacleCoordinate>& _positions, const std::string& _name, const std::string& _type, coal::ShapeBase* _collision_object, const bool& _is_static);
        ~ObjectObstacleFCL();                                                 // destructor
        ObjectObstacleFCL(const ObjectObstacleFCL &other) = delete;            // copy constructor
        ObjectObstacleFCL(ObjectObstacleFCL &&other) = default;                 // move constructor
        ObjectObstacleFCL &operator=(const ObjectObstacleFCL &other) = delete; // copy assignment
        ObjectObstacleFCL &operator=(ObjectObstacleFCL &&other) = default;      // move assignment

        void set_position(const unsigned int frame) const;
        void set_position(const MDP::ObstacleCoordinate position) const;
        coal::ShapeBase* get_collision_object() const;
        MDP::ObstacleCoordinate get_position(const unsigned int frame) const;
        coal::Transform3s get_transform() const;
        std::string get_name() const;
        std::string get_type() const;
        bool get_is_static() const;

    private:
        mutable coal::ShapeBase* collision_object; //TODO remove all mutables because they are quickfix
        std::vector<MDP::ObstacleCoordinate> positions;
        mutable coal::Transform3s transform;
        std::string name;
        std::string type;
        const bool is_static;
    };
}