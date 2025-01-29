#include <vector>
#include <string>
#include <config_read_writer/ObstacleJsonInfo.hpp>
#include <cassert>

MDP::ObstacleJsonInfo::ObstacleJsonInfo(std::string _name, std::string _type,
                                                        std::vector<std::vector<float>> coordinates_raw,
                                                        float fps,
                                                        bool _is_static, ObstacleType _obstacle_type) : name(_name),
                                                                           type(_type),
                                                                           is_static(_is_static),
                                                                           obstacle_type(_obstacle_type)
{
    if (is_static) // if static, store data only for 1 frame
    {
        assert(coordinates_raw[0].size()==7);
        this->coordinates.push_back(ObstacleCoordinate(coordinates_raw[0][0], //x
                                                       coordinates_raw[0][1],//y
                                                       coordinates_raw[0][2],//z
                                                       coordinates_raw[0][3],//quat_x
                                                       coordinates_raw[0][4],//quat_y
                                                       coordinates_raw[0][5],//quat_z
                                                       coordinates_raw[0][6],//quat_w
                                                       0,//time millisec
                                                       0)); //frame_id
    }
    else{
        for (unsigned long long  frame_id=0; frame_id<coordinates_raw.size();frame_id++){
            assert(coordinates_raw[0].size()==7);
            this->coordinates.push_back(ObstacleCoordinate(coordinates_raw[frame_id][0], //x
                                                       coordinates_raw[frame_id][1],//y
                                                       coordinates_raw[frame_id][2],//z
                                                       coordinates_raw[frame_id][3],//quat_x
                                                       coordinates_raw[frame_id][4],//quat_y
                                                       coordinates_raw[frame_id][5],//quat_z
                                                       coordinates_raw[frame_id][6],//quat_w
                                                       frame_id*1000/fps,//time millisec
                                                       frame_id)); //frame_id
        }
    }
};
std::string MDP::ObstacleJsonInfo::get_type() const{
    return this->type;
}
const std::vector<MDP::ObstacleCoordinate> MDP::ObstacleJsonInfo::get_coordinates() const
{
    return coordinates;
}
bool MDP::ObstacleJsonInfo::get_is_static() const
{
    return this->is_static;
}

std::string MDP::ObstacleJsonInfo::get_name() const
{
    return this->name;
}

MDP::ObstacleJsonInfo::ObstacleType MDP::ObstacleJsonInfo::get_obstacle_type() const{
    return this->obstacle_type;
}
