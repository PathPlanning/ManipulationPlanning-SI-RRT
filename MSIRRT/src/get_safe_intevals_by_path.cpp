#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <utility>              // for std::pair
#include <cmath>                // for std::abs
#include <rapidjson/document.h>  // for parsing JSON
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>    // for writing JSON
#include <rapidjson/stringbuffer.h> // for working with strings
#include <rapidjson/prettywriter.h> // for formatted JSON output
#include <rapidjson/istreamwrapper.h>
#include "config_read_writer/config_read.hpp"
#include "config_read_writer/STRRTConfigReader.hpp"
#include "config_read_writer/ResultsWriter.hpp"
#include "CollisionManager/CollisionManager.hpp"
// Функция для парсинга и извлечения данных
std::vector<std::pair<double, std::vector<double>>> extractFinalPlannerData(const std::string &jsonFilePath)
{
    std::ifstream ifs(jsonFilePath);
    if (!ifs.is_open())
    {
        throw std::runtime_error("Unable to open file: " + jsonFilePath);
    }

    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    std::vector<std::pair<double, std::vector<double>>> result;

    if (doc.HasMember("final_planner_data") && doc["final_planner_data"].HasMember("final_path"))
    {
        const auto &finalPath = doc["final_planner_data"]["final_path"];

        for (const auto &entry : finalPath.GetArray())
        {
            double time = entry["time"].GetDouble();
            std::vector<double> robotAngles;
            for (const auto &angle : entry["robot_angles"].GetArray())
            {
                robotAngles.push_back(angle.GetDouble());
            }
            result.emplace_back(time, robotAngles);
        }
    }

    return result;
}

// Функция для линейной интерполяции между двумя значениями
double linearInterpolate(double start, double end, double t)
{
    return start + t * (end - start);
}

// Функция для интерполяции между двумя точками пути
std::vector<std::pair<double, std::vector<double>>> interpolatePath(
    const std::pair<double, std::vector<double>> &startPoint,
    const std::pair<double, std::vector<double>> &endPoint,
    int numSteps)
{
    std::vector<std::pair<double, std::vector<double>>> interpolatedPath;

    // Линейная интерполяция времени и углов
    for (int i = 0; i <= numSteps; ++i)
    {
        double t = static_cast<double>(i) / numSteps;
        double interpolatedTime = linearInterpolate(startPoint.first, endPoint.first, t);

        std::vector<double> interpolatedAngles;
        for (size_t j = 0; j < startPoint.second.size(); ++j)
        {
            double interpolatedAngle = linearInterpolate(startPoint.second[j], endPoint.second[j], t);
            interpolatedAngles.push_back(interpolatedAngle);
        }

        interpolatedPath.emplace_back(interpolatedTime, interpolatedAngles);
    }

    return interpolatedPath;
}

// Функция для создания интерполированного пути для всего плана
std::vector<std::pair<double, std::vector<double>>> generateInterpolatedPath(
    const std::vector<std::pair<double, std::vector<double>>> &path, int numSteps)
{
    std::vector<std::pair<double, std::vector<double>>> fullInterpolatedPath;

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        // Интерполируем между каждой парой соседних точек
        auto interpolatedSegment = interpolatePath(path[i], path[i + 1], numSteps);
        // Добавляем интерполированные точки в итоговый путь (кроме последней, чтобы не дублировать)
        fullInterpolatedPath.reserve(fullInterpolatedPath.size() + std::distance(interpolatedSegment.begin(), interpolatedSegment.end()));
        fullInterpolatedPath.insert(fullInterpolatedPath.end(), interpolatedSegment.begin(), interpolatedSegment.end());
    }

    // Добавляем последнюю точку (необходима для точности)
    fullInterpolatedPath.push_back(path.back());

    return fullInterpolatedPath;
}

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_json_file> <path_to_scene_task_json> <num_steps>" << std::endl;
        return 1;
    }

    std::string path_to_scene_json = argv[2];
    MDP::ConfigReader SceneTask(path_to_scene_json);

    std::shared_ptr<MDP::CollisionManager> collision_manager = std::make_shared<MDP::CollisionManager>(SceneTask.get_scene_task());

    std::string jsonFilePath = argv[1];
    int numSteps = std::stoi(argv[3]);

    try
    {
        // Извлечение исходных данных
        auto originalPath = extractFinalPlannerData(jsonFilePath);

        // Генерация интерполированного пути
        auto interpolatedPath = generateInterpolatedPath(originalPath, numSteps);


        rapidjson::Document doc;
        doc.SetObject();
        rapidjson::Document::AllocatorType &allocator = doc.GetAllocator();

        // Создаем массив для интерполированного пути
        rapidjson::Value finalPath(rapidjson::kArrayType);

        for (const auto &[time, robotAngles] : interpolatedPath)
        {
            // Создаем объект для каждой точки пути
            rapidjson::Value point(rapidjson::kObjectType);

            // Добавляем время
            point.AddMember("time", time, allocator);

            // Добавляем углы робота
            rapidjson::Value angles(rapidjson::kArrayType);
            for (double angle : robotAngles)
            {
                angles.PushBack(angle, allocator);
            }
            point.AddMember("robot_angles", angles, allocator);
            std::vector<std::pair<int,int>> safe_intervals = collision_manager->get_safe_intervals(robotAngles);
            rapidjson::Value safe_intervals_json(rapidjson::kArrayType);
            for (std::pair<int,int> safe_int : safe_intervals)
            {    rapidjson::Value safe_int_json(rapidjson::kArrayType);
                safe_int_json.PushBack(safe_int.first,allocator);
                safe_int_json.PushBack(safe_int.second,allocator);
                safe_intervals_json.PushBack(safe_int_json, allocator);
            }
            point.AddMember("safe_intervals", safe_intervals_json, allocator);

            // Добавляем точку в финальный путь
            finalPath.PushBack(point, allocator);
        }

        // Помещаем финальный путь в JSON структуру
        doc.AddMember("final_interpolated_path", finalPath, allocator);

        // Записываем JSON в файл
        std::ofstream ofs("./safe_intervals.json");
        if (!ofs.is_open())
        {
            std::string f = "./safe_intervals.json";
            throw std::runtime_error("Unable to open output file: " + f);
        }

        // Используем PrettyWriter для форматированного вывода
        rapidjson::StringBuffer buffer;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
        doc.Accept(writer);

        ofs << buffer.GetString();
        ofs.close();
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
