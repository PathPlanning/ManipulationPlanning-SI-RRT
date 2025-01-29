#include <vector>
#include <string>
#include <config_read_writer/CubeObstacleJsonInfo.hpp>
#include <config_read_writer/ObstacleJsonInfo.hpp>
#include <cassert>
MDP::CubeObstacleJsonInfo::CubeObstacleJsonInfo(std::string _name, std::string _type,
                                                std::vector<std::vector<float>> coordinates_raw,
                                                float fps, std::vector<float> _dimensions,
                                                bool _is_static) : MDP::ObstacleJsonInfo(_name, _type, coordinates_raw, fps, _is_static,MDP::ObstacleJsonInfo::ObstacleType::BOX), dimensions(_dimensions) {};

std::vector<float> MDP::CubeObstacleJsonInfo::get_dimensions() const
{
    return this->dimensions;
}