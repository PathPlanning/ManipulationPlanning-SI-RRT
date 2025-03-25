#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include "coal/collision.h"
#include "coal/collision_data.h"
namespace MDP
{

    struct ObstacleCoordinate
    {
        const float x;
        const float y;
        const float z;
        const coal::Vec3s pos;
        const float quat_x;
        const float quat_y;
        const float quat_z;
        const float quat_w;
        const coal::Quaternion3f rotation;
        const float time_millisec;
        const int frame;
        ObstacleCoordinate(float _x, float _y, float _z,
                           float _quat_x, float _quat_y, float _quat_z, float _quat_w,
                           float _time_millisec,
                           int _frame) : rotation(quat_w, quat_x, quat_y, quat_z), pos(x, y, z), x(_x), y(_y), z(_z),
                                         quat_x(_quat_x), quat_y(_quat_y), quat_z(_quat_z), quat_w(_quat_w),
                                         time_millisec(_time_millisec),
                                         frame(_frame) {};

        ~ObstacleCoordinate() = default;                                          // destructor
        ObstacleCoordinate(const ObstacleCoordinate &other) = default;            // copy constructor
        ObstacleCoordinate(ObstacleCoordinate &&other) = default;                 // move constructor
        ObstacleCoordinate &operator=(const ObstacleCoordinate &other) = default; // copy assignment
        ObstacleCoordinate &operator=(ObstacleCoordinate &&other) = default;      // move assignment
    };

    class ObstacleJsonInfo
    {
    public:
        enum class ObstacleType
        {
            BOX,
            SPHERE
        };

        ObstacleJsonInfo(std::string _name, std::string _type,
                         std::vector<std::vector<float>> coordinates_raw,
                         float fps,
                         bool _is_static,
                         ObstacleType _obstacle_type);
        ~ObstacleJsonInfo() = default;                                        // destructor
        ObstacleJsonInfo(const ObstacleJsonInfo &other) = default;            // copy constructor
        ObstacleJsonInfo(ObstacleJsonInfo &&other) = default;                 // move constructor
        ObstacleJsonInfo &operator=(const ObstacleJsonInfo &other) = default; // copy assignment
        ObstacleJsonInfo &operator=(ObstacleJsonInfo &&other) = default;      // move assignment

        std::string get_name() const;
        std::string get_type() const;
        const std::vector<MDP::ObstacleCoordinate> get_coordinates() const;
        bool get_is_static() const;
        ObstacleType get_obstacle_type() const;

    private:
        const ObstacleType obstacle_type;
        const std::string name;
        const std::string type;
        std::vector<MDP::ObstacleCoordinate> coordinates;
        const bool is_static;
    };

} // namespace MDP
