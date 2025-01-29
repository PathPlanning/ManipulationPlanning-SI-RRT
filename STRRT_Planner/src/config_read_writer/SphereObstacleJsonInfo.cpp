#include <vector>
#include <string>
#include <config_read_writer/SphereObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>
#include <cassert>
MDP::SphereObstacleJsonInfo::SphereObstacleJsonInfo(std::string _name, std::string _type,
                                                    std::vector<std::vector<float>> coordinates_raw,
                                                    float fps, float _radius,
                                                    bool _is_static)
    : MDP::ObstacleJsonInfo(_name, _type, coordinates_raw, fps, _is_static, MDP::ObstacleJsonInfo::ObstacleType::SPHERE), radius(_radius) {};

float MDP::SphereObstacleJsonInfo::get_radius() const
{
    return this->radius;
}