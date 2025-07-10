#include <config_read_writer/ObstaclePosWriter.hpp>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <map>
#include <algorithm>

MDP::ObstaclePosWriter::ObstaclePosWriter(const MDP::ConfigReader::SceneTask _scene_task) : collision_manager(_scene_task)
{
    this->data_to_export.SetArray();
    this->allocator = data_to_export.GetAllocator();
};

void MDP::ObstaclePosWriter::save_json(const std::string path_to_result_folder, const std::string file_name, int random_seed)
{
    data_to_export.Clear();
    data_to_export.Clear();

    for (const MDP::ObjectObstacleFCL *obstacle : this->collision_manager.get_obstacles())
    {
        write_obstacle_to_json(obstacle);
    }

    for (MDP::RobotObstacleFCL &robot_obstacle : this->collision_manager.get_robot_obstacles())
    {
        write_robot_obstacle_to_json(robot_obstacle);
    }

    write_plannable_robot_to_json();

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    data_to_export.Accept(writer);
    std::string filename = path_to_result_folder + "/position_logs_" + file_name + "_" + std::to_string(random_seed) + ".json";

    std::ofstream result_file(filename);
    result_file << buffer.GetString();
    result_file.close();
}

void MDP::ObstaclePosWriter::add_string_to_json(rapidjson::Value &json, const std::string &name, const std::string &value)
{
    json.AddMember(rapidjson::Value{}.SetString(name.c_str(), name.length(), allocator),
                   rapidjson::Value{}.SetString(value.c_str(), value.length(), allocator),
                   this->allocator);
}

void MDP::ObstaclePosWriter::write_obstacle_position_to_json(rapidjson::Value &json, const MDP::ObstacleCoordinate &position)
{
    json.AddMember("x", position.pos.x(), this->allocator);
    json.AddMember("y", position.pos.y(), this->allocator);
    json.AddMember("z", position.pos.z(), this->allocator);
    json.AddMember("frame", position.frame, this->allocator);
    json.AddMember("q_w", position.quat_w, this->allocator);
    json.AddMember("q_x", position.quat_x, this->allocator);
    json.AddMember("q_y", position.quat_y, this->allocator);
    json.AddMember("q_z", position.quat_z, this->allocator);
}

void MDP::ObstaclePosWriter::parse_obstacle_position_to_json(rapidjson::Value &json, const MDP::ObjectObstacleFCL *obstacle)
{
    rapidjson::Value position_array(rapidjson::kArrayType);

    rapidjson::Value coordinates;
    coordinates.SetObject();
    for (int frame = 0; frame < this->collision_manager.get_scene_task().frame_count; frame++)
    {
        rapidjson::Value coordinate;
        coordinate.SetObject();

        write_obstacle_position_to_json(coordinate, obstacle->get_position(frame));

        position_array.PushBack(coordinate, allocator);
    }
    json.AddMember("positions", position_array, this->allocator);
}

void MDP::ObstaclePosWriter::write_obstacle_to_json(const MDP::ObjectObstacleFCL *obstacle)
{
    rapidjson::Value obstacle_json;
    obstacle_json.SetObject();
    add_string_to_json(obstacle_json, "name", obstacle->get_name());
    add_string_to_json(obstacle_json, "type", obstacle->get_type());
    parse_obstacle_position_to_json(obstacle_json, obstacle);

    this->data_to_export.PushBack(obstacle_json, allocator);
}

void MDP::ObstaclePosWriter::write_robot_obstacle_to_json(MDP::RobotObstacleFCL &robot_obstacle)
{

    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> init_coll_obj = robot_obstacle.get_collision_object_for_frame(0);

    std::vector<rapidjson::Value> transforms_per_link(init_coll_obj.size());

    int frame_count = this->collision_manager.get_scene_task().frame_count;
    for (int frame = 0; frame < frame_count; frame++)
    {
        std::vector<MDP::RobotObstacleFCL::JointCollisionObject> frame_coll_obj = robot_obstacle.get_collision_object_for_frame(frame);
        for (size_t joint_idx = 0; joint_idx < frame_coll_obj.size(); ++joint_idx)
        {
            rapidjson::Value transform_json(rapidjson::kObjectType);

            transform_json.AddMember("x", frame_coll_obj[joint_idx].transform.getTranslation()[0], this->allocator);
            transform_json.AddMember("y", frame_coll_obj[joint_idx].transform.getTranslation()[1], this->allocator);
            transform_json.AddMember("z", frame_coll_obj[joint_idx].transform.getTranslation()[2], this->allocator);
            transform_json.AddMember("q_w", frame_coll_obj[joint_idx].transform.getQuatRotation().w(), this->allocator);
            transform_json.AddMember("q_x", frame_coll_obj[joint_idx].transform.getQuatRotation().x(), this->allocator);
            transform_json.AddMember("q_y", frame_coll_obj[joint_idx].transform.getQuatRotation().y(), this->allocator);
            transform_json.AddMember("q_z", frame_coll_obj[joint_idx].transform.getQuatRotation().z(), this->allocator);

            if (frame == 0)
            {
                transforms_per_link[joint_idx].SetArray();
            }

            transforms_per_link[joint_idx].PushBack(transform_json, allocator);
        }
    }

    for (size_t joint_idx = 0; joint_idx < init_coll_obj.size(); ++joint_idx)
    {
        rapidjson::Value link_json(rapidjson::kObjectType);
        add_string_to_json(link_json, "name", init_coll_obj[joint_idx].name);
        link_json.AddMember("joint_number", init_coll_obj[joint_idx].joint_number, this->allocator);

        if (init_coll_obj[joint_idx].collision_object->getNodeType() == coal::NODE_TYPE::GEOM_CAPSULE)
        {
            add_string_to_json(link_json, "type", "capsule");
            std::shared_ptr<coal::Capsule> capsule = std::dynamic_pointer_cast<coal::Capsule>(init_coll_obj[joint_idx].collision_object);
            link_json.AddMember("radius", capsule->radius, this->allocator);
            link_json.AddMember("length", capsule->halfLength * 2, this->allocator);
        }
        else if (init_coll_obj[joint_idx].collision_object->getNodeType() == coal::NODE_TYPE::GEOM_CONVEX32)
        {
            add_string_to_json(link_json, "type", "mesh");
            link_json.AddMember("filepath", rapidjson::StringRef(robot_obstacle.get_filepath_for_link_name(init_coll_obj[joint_idx].name).c_str()), this->allocator);
        }
        else
        {
            assert(false);
        }
        link_json.AddMember("transforms", transforms_per_link[joint_idx], allocator);

        this->data_to_export.PushBack(link_json, allocator);
    }
}

void MDP::ObstaclePosWriter::write_plannable_robot_to_json()
{
    MDP::RobotObstacleFCL robot = this->collision_manager.get_planned_robot();
    std::vector<MDP::RobotObstacleFCL::JointCollisionObject> init_coll_obj = robot.get_collision_object_for_robot_angles(this->collision_manager.get_scene_task().start_configuration);

    std::vector<rapidjson::Value> transforms_per_link(init_coll_obj.size());

    int frame_count = this->collision_manager.get_scene_task().frame_count;
    for (int frame = 0; frame < frame_count; ++frame)
    {
        std::vector<MDP::RobotObstacleFCL::JointCollisionObject> frame_coll_obj;
        if (frame < frame_count / 2)
        {
            frame_coll_obj = robot.get_collision_object_for_robot_angles(this->collision_manager.get_scene_task().start_configuration);
        }
        else
        {
            frame_coll_obj = robot.get_collision_object_for_robot_angles(this->collision_manager.get_scene_task().end_configuration);
        }
        for (size_t joint_idx = 0; joint_idx < frame_coll_obj.size(); ++joint_idx)
        {
            rapidjson::Value transform_json(rapidjson::kObjectType);

            transform_json.AddMember("x", frame_coll_obj[joint_idx].transform.getTranslation()[0], this->allocator);
            transform_json.AddMember("y", frame_coll_obj[joint_idx].transform.getTranslation()[1], this->allocator);
            transform_json.AddMember("z", frame_coll_obj[joint_idx].transform.getTranslation()[2], this->allocator);
            transform_json.AddMember("q_w", frame_coll_obj[joint_idx].transform.getQuatRotation().w(), this->allocator);
            transform_json.AddMember("q_x", frame_coll_obj[joint_idx].transform.getQuatRotation().x(), this->allocator);
            transform_json.AddMember("q_y", frame_coll_obj[joint_idx].transform.getQuatRotation().y(), this->allocator);
            transform_json.AddMember("q_z", frame_coll_obj[joint_idx].transform.getQuatRotation().z(), this->allocator);

            if (frame == 0)
            {
                transforms_per_link[joint_idx].SetArray();
            }

            transforms_per_link[joint_idx].PushBack(transform_json, allocator);
        }
    }

    for (size_t joint_idx = 0; joint_idx < init_coll_obj.size(); ++joint_idx)
    {
        rapidjson::Value link_json(rapidjson::kObjectType);
        add_string_to_json(link_json, "name", init_coll_obj[joint_idx].name);
        link_json.AddMember("joint_number", init_coll_obj[joint_idx].joint_number, this->allocator);

        if (init_coll_obj[joint_idx].collision_object->getNodeType() == coal::NODE_TYPE::GEOM_CAPSULE)
        {
            add_string_to_json(link_json, "type", "capsule");
            std::shared_ptr<coal::Capsule> capsule = std::dynamic_pointer_cast<coal::Capsule>(init_coll_obj[joint_idx].collision_object);
            link_json.AddMember("radius", capsule->radius, this->allocator);
            link_json.AddMember("length", capsule->halfLength * 2, this->allocator);
        }
        else if (init_coll_obj[joint_idx].collision_object->getNodeType() == coal::NODE_TYPE::GEOM_CONVEX32)
        {
            add_string_to_json(link_json, "type", "mesh");
            link_json.AddMember("filepath", rapidjson::StringRef(robot.get_filepath_for_link_name(init_coll_obj[joint_idx].name).c_str()), this->allocator);
        }
        else
        {
            assert(false);
        }
        link_json.AddMember("transforms", transforms_per_link[joint_idx], allocator);

        this->data_to_export.PushBack(link_json, allocator);
    }
}