#include <vector>
#include <string>
#include <config_read_writer/RobotObstacleJsonInfo.hpp>
#include <cassert>

MDP::RobotObstacleJsonInfo::RobotObstacleJsonInfo(const std::string &_name, const std::string &_type, const std::string &_urdf_file_path,
                                                  const std::vector<std::string> &_robot_joints_order, const std::vector<MDP::RobotObstacleJsonInfo::PathState> &_trajectory,
                                                  const std::vector<std::vector<float>> &base_coordinates_raw,
                                                  const float &fps,
                                                  bool _is_static) : name(_name),
                                                                            type(_type),
                                                                            urdf_file_path(_urdf_file_path),
                                                                            robot_joints_order(_robot_joints_order),
                                                                            trajectory(_trajectory),
                                                                            is_static(_is_static)
{
    if (is_static) // if static, store data only for 1 frame
    {
        assert(base_coordinates_raw[0].size() == 7);
        this->base_coordinates.push_back(ObstacleCoordinate(base_coordinates_raw[0][0], // x
                                                            base_coordinates_raw[0][1], // y
                                                            base_coordinates_raw[0][2], // z
                                                            base_coordinates_raw[0][3], // quat_x
                                                            base_coordinates_raw[0][4], // quat_y
                                                            base_coordinates_raw[0][5], // quat_z
                                                            base_coordinates_raw[0][6], // quat_w
                                                            0,                          // time millisec
                                                            0));                        // frame_id
    }
    else
    {
        // assert(false); //, "dynamic robot base is not implemented"
        //
        for (unsigned long long frame_id = 0; frame_id < base_coordinates_raw.size(); frame_id++)
        {
            assert(base_coordinates_raw[0].size() == 7);
            this->base_coordinates.push_back(ObstacleCoordinate(base_coordinates_raw[frame_id][0], // x
                                                                base_coordinates_raw[frame_id][1], // y
                                                                base_coordinates_raw[frame_id][2], // z
                                                                base_coordinates_raw[frame_id][3], // quat_x
                                                                base_coordinates_raw[frame_id][4], // quat_y
                                                                base_coordinates_raw[frame_id][5], // quat_z
                                                                base_coordinates_raw[frame_id][6], // quat_w
                                                                frame_id * 1000 / fps,             // time millisec
                                                                frame_id));                        // frame_id
        }
    }
};

std::vector<MDP::RobotObstacleJsonInfo::PathState> MDP::RobotObstacleJsonInfo::get_trajectory() const
{
    return this->trajectory;
}

std::string MDP::RobotObstacleJsonInfo::get_type() const
{
    return this->type;
}

std::vector<MDP::ObstacleCoordinate> MDP::RobotObstacleJsonInfo::get_base_coordinates() const
{
    return this->base_coordinates;
}

bool MDP::RobotObstacleJsonInfo::get_is_static() const
{
    return this->is_static;
}

std::string MDP::RobotObstacleJsonInfo::get_name() const
{
    return this->name;
}
std::string MDP::RobotObstacleJsonInfo::get_urdf_file_path() const
{
    return this->urdf_file_path;
}
std::vector<std::string> MDP::RobotObstacleJsonInfo::get_robot_joints_order() const
{
    return this->robot_joints_order;
}

