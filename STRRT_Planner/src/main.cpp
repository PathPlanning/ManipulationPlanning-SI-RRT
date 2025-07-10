//  ./STRRT_Planner   ../../STRRT/scene_task.json ./ ../../STRRT/strrt_config.json
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/SimpleSetup.h>
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
#include "config_read_writer/config_read.hpp"
#include "config_read_writer/STRRTConfigReader.hpp"
#include "config_read_writer/ResultsWriter.hpp"
#include "config_read_writer/RobotObstacleJsonInfo.hpp"

#include <OMPLValidators/SpaceTimeMotionValidator.hpp>
#include <OMPLValidators/StateValidityCheckerFunctor.hpp>
namespace ob = ompl::base;
namespace og = ompl::geometric;

void StrrtWritePlannerData(const ob::Planner *planner, rapidjson::Value &json_object, bool add_tree_structure)
{

    assert(json_object.IsObject());

    // add num of vertices and edges

    ompl::base::PlannerData data(planner->getSpaceInformation());
    planner->getPlannerData(data);

    json_object.AddMember("number_of_vertices", data.numVertices(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("number_of_edges", data.numEdges(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("number_of_start_vertices", data.numStartVertices(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("number_of_goal_vertices", data.numGoalVertices(), MDP::ResultsWriter::get_instance().get_json_allocator());

    // add num of state and motion validations
    json_object.AddMember("number_of_collision_checks", MDP::ResultsWriter::get_instance().get_collision_check_count(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("number_of_valid_motion", data.numEdges(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("number_of_invalid_motion", data.numStartVertices(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("number_of_checked_motion", data.numGoalVertices(), MDP::ResultsWriter::get_instance().get_json_allocator());
    json_object.AddMember("fraction_of_valid_motion", data.numGoalVertices(), MDP::ResultsWriter::get_instance().get_json_allocator());

    // add additional properties
    //TODO: produces invalid bytes, must uncomment and fix
    // for (auto const &[key, value] : data.properties)
    // {
    //     json_object.AddMember(rapidjson::StringRef(key.c_str()), rapidjson::StringRef(value.c_str()), MDP::ResultsWriter::get_instance().get_json_allocator());
    // }

    // for (auto const &[key, value] : planner->getPlannerProgressProperties())
    // {
    //     json_object.AddMember(rapidjson::StringRef(key.c_str()), rapidjson::StringRef(value().c_str()), MDP::ResultsWriter::get_instance().get_json_allocator());
    // }

    // add tree structure, if needed
    if (add_tree_structure)
    {
        assert(false);
        // not implemented
    }
}
void StrrtReportIntermediateSolution(const ob::Planner *planner, const std::vector<const ob::State *> &path, const ob::Cost cost, ob::PlannerTerminationCondition& ptc_exact, const double& start_time)
{
    ptc_exact.terminate();
    std::vector<MDP::RobotObstacleJsonInfo::PathState> result_path;
    int robot_joint_counts = planner->getSpaceInformation()->getStateSpace()->as<ob::SpaceTimeStateSpace>()->getSpaceComponent()->getDimension();
    for (const ob::State *state : path)
    {
        double point_time = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position+start_time;
        std::vector<double> point(robot_joint_counts);
        for (int x = 0; x < robot_joint_counts; ++x)
        {
            point[x] = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[x];
        }
        result_path.emplace_back(point, point_time);
    }
    rapidjson::Value planner_metadata;
    planner_metadata.SetObject();
    StrrtWritePlannerData(planner, planner_metadata, false);
    MDP::ResultsWriter::get_instance().add_results(result_path, cost.value(), std::move(planner_metadata));
}
void parse_my_args(int argc, char **argv, std::string &path_to_scene_json, std::string &path_to_result_folder, std::string &path_to_strrt_config_json, int& random_seed)
{
    if (argc < 5)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_scene_json> <path_to_result_folder> <path_to_strrt_config_json> <random_seed>" << std::endl;
        exit(EXIT_FAILURE);
    }

    path_to_scene_json = argv[1];
    path_to_result_folder = argv[2];
    path_to_strrt_config_json = argv[3];
    random_seed = atoi(argv[4]);
}

void check_args(const std::string &path_to_scene_json, const std::string &path_to_result_folder, const std::string &path_to_strrt_config_json)
{
    if (!std::ifstream(path_to_scene_json) || !std::ifstream(path_to_strrt_config_json) || !std::filesystem::is_directory(path_to_result_folder))
    {
        throw std::runtime_error("Invalid file or directory path");
    }
}

int main(int argc, char **argv)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    std::string path_to_scene_json;
    std::string path_to_result_folder;
    std::string path_to_strrt_config_json;
    int random_seed;
    parse_my_args(argc, argv, path_to_scene_json, path_to_result_folder, path_to_strrt_config_json, random_seed);
    check_args(path_to_scene_json, path_to_result_folder, path_to_strrt_config_json);
    std::srand(random_seed);
    ompl::RNG::setSeed(random_seed);
    MDP::ResultsWriter::get_instance().setup(path_to_scene_json, path_to_strrt_config_json, path_to_result_folder, MDP::ResultsWriter::PlannerType::STRRT_STAR);
    MDP::ResultsWriter::get_instance().algorithm_start();
    MDP::ConfigReader SceneTask(path_to_scene_json);

    MDP::STRRTConfigReader STRRTConfigReader(path_to_strrt_config_json);

    // TODO: optimise collision manager construction and dont construct in space validity checker, or find another way to get joint limits
    std::shared_ptr<MDP::CollisionManager> collision_manager = std::make_shared<MDP::CollisionManager>(SceneTask.get_scene_task());
    std::vector<std::pair<float, float>> joint_limits = collision_manager->get_planned_robot_limits();
   

    auto vector_space = std::make_shared<ob::RealVectorStateSpace>(SceneTask.get_scene_task().robot_joint_count);
    ob::RealVectorBounds vector_space_bounds(SceneTask.get_scene_task().robot_joint_count);
    for (int joint_ind = 0; joint_ind < SceneTask.get_scene_task().robot_joint_count; joint_ind++)
    {
        if(joint_limits[joint_ind].first == joint_limits[joint_ind].second){
            MDP::ResultsWriter::get_instance().save_error_json("invalid urdf joint limits!");
            assert(joint_limits[joint_ind].first != joint_limits[joint_ind].second);
        }
        
        vector_space_bounds.setLow(joint_ind, joint_limits[joint_ind].first);
        vector_space_bounds.setHigh(joint_ind, joint_limits[joint_ind].second);
    }

    int low_bound_frame = 0;
    if(!STRRTConfigReader.get_strrt_config().ignore_low_bound_time){
         low_bound_frame  = collision_manager->get_goal_frame_low_bound();
    }
    
    double low_bound_time =  (double)low_bound_frame/ (double)SceneTask.get_scene_task().fps;

    vector_space->setBounds(vector_space_bounds);
    auto time_space = std::make_shared<ob::TimeStateSpace>();
    auto space = std::make_shared<ob::SpaceTimeStateSpace>(vector_space, SceneTask.get_scene_task().robot_joint_max_velocity[0],0.5); // TODO: max velocity for each joint
    
    space->setTimeBounds(0.0,  (double)(((double)SceneTask.get_scene_task().frame_count)/(double)SceneTask.get_scene_task().fps -0.001)); // -0.001 to fix floating point rounding errors
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    std::shared_ptr<MDP::StateValidityCheckerFunctor> state_cheker = std::make_shared<MDP::StateValidityCheckerFunctor>(si, SceneTask.get_scene_task());

    si->setStateValidityChecker(ob::StateValidityCheckerPtr(state_cheker.get()));
    si->setMotionValidator(std::make_shared<MDP::SpaceTimeMotionValidator>(si,SceneTask.get_scene_task()));
    si->setup();

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    for (int i = 0; i < SceneTask.get_scene_task().robot_joint_count; i++)
    {
        start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = SceneTask.get_scene_task().start_configuration[i];
        goal->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = SceneTask.get_scene_task().end_configuration[i];
    }
    start->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = 0.0;
    goal->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = (double)(SceneTask.get_scene_task().frame_count) / (double)SceneTask.get_scene_task().fps  -0.001; // -0.001 to fix floating point rounding errors
    // std::cout<<(double)(SceneTask.get_scene_task().frame_count) / (double)SceneTask.get_scene_task().fps << " "<<((double)(SceneTask.get_scene_task().frame_count) / (double)SceneTask.get_scene_task().fps) * (double)SceneTask.get_scene_task().fps<<std::endl;
    
    if(!si->satisfiesBounds(start.get())){
        MDP::ResultsWriter::get_instance().save_error_json("start pose does not satisfy bounds");
        assert(si->satisfiesBounds(start.get()));
    }
    if(!si->satisfiesBounds(goal.get())){
        MDP::ResultsWriter::get_instance().save_error_json("goal pose does not satisfy bounds");
        assert(si->satisfiesBounds(goal.get()));
    }

    if(!si->isValid(start.get())){
        MDP::ResultsWriter::get_instance().save_error_json("start pose is not valid (self crossing detected)!");
        assert(si->isValid(start.get()));
    }

    // Don't need for multiagent task
    // if(!si->isValid(goal.get())){

    //     MDP::ResultsWriter::get_instance().save_error_json("goal pose is not valid (self crossing detected)!");
    //     assert(si->isValid(goal.get()));
    // }

    
    std::shared_ptr<ob::ProblemDefinition> pdef = std::make_shared<ob::ProblemDefinition>(si);

    pdef->setStartAndGoalStates(start, goal);

    ob::PlannerTerminationCondition ptc_exact = ob::exactSolnPlannerTerminationCondition(pdef);
    pdef->setIntermediateSolutionCallback([&ptc_exact,&SceneTask](const ob::Planner *planner, const std::vector<const ob::State *> &path, const ob::Cost cost) -> void{StrrtReportIntermediateSolution(planner,path,cost,ptc_exact,SceneTask.get_scene_task().start_time);});
    std::shared_ptr<og::STRRTstar> planner = std::make_shared<og::STRRTstar>(si);

    planner->setProblemDefinition(pdef);
    planner->setRange(STRRTConfigReader.get_strrt_config().max_range);                           // set maximum extend distance
    planner->setOptimumApproxFactor(STRRTConfigReader.get_strrt_config().optimum_approx_factor); // if approx factor is near 1, planner will make shorted (in space) path,
    // when factor is near 0, planner will make shorter (in time) path.
    if (STRRTConfigReader.get_strrt_config().rewiring_method == MDP::STRRTConfigReader::RewireType::OFF)
    {
        planner->setRewiringToOff();
    }
    else if (STRRTConfigReader.get_strrt_config().rewiring_method == MDP::STRRTConfigReader::RewireType::RADIUS)
    {
        planner->setRewiringToRadius();
    }
    else if (STRRTConfigReader.get_strrt_config().rewiring_method == MDP::STRRTConfigReader::RewireType::K_NEIGHBOURS)
    {
        planner->setRewiringToKNearest();
    }
    planner->setRewireFactor(STRRTConfigReader.get_strrt_config().rewire_factor);
    planner->setBatchSize(STRRTConfigReader.get_strrt_config().initial_batch_size);
    planner->setTimeBoundFactorIncrease(STRRTConfigReader.get_strrt_config().time_bound_factor_increase);
    planner->setInitialTimeBoundFactor(STRRTConfigReader.get_strrt_config().inital_time_bound_factor);
    planner->setSampleUniformForUnboundedTime(STRRTConfigReader.get_strrt_config().sample_uniform_for_unbounded_time);
    // planner->setMinTimeBound(low_bound_time);
    planner->setup();
    MDP::ResultsWriter::get_instance().config_end();
    bool plan_result = false;
    if (STRRTConfigReader.get_strrt_config().stop_if_path_found)
    {
        plan_result = planner->solve(ob::plannerOrTerminationCondition(ob::timedPlannerTerminationCondition(STRRTConfigReader.get_strrt_config().max_planning_time_pts), ptc_exact));
    }
    else
    {
        plan_result = planner->solve(ob::timedPlannerTerminationCondition(STRRTConfigReader.get_strrt_config().max_planning_time_pts));
    }
    MDP::ResultsWriter::get_instance().solver_end();

    rapidjson::Value path_result;
    path_result.SetObject();
    path_result.AddMember("has_result", pdef->hasExactSolution(), MDP::ResultsWriter::get_instance().get_json_allocator());
    if (plan_result)
    {
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();  
        std::vector<MDP::RobotObstacleJsonInfo::PathState> result_path;
        for (const ob::State *state : path->getStates())
        {
            double point_time = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position + SceneTask.get_scene_task().start_time;
            std::vector<double> point(SceneTask.get_scene_task().robot_joint_count);
            for (int x = 0; x < SceneTask.get_scene_task().robot_joint_count; ++x)
            {
                point[x] = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[x];
            }
            result_path.emplace_back(point, point_time);
        }
        rapidjson::Value path_json;
        double path_cost = result_path.back().time-SceneTask.get_scene_task().start_time;
        MDP::ResultsWriter::get_instance().convert_path_to_json(result_path,path_json);
        path_result.AddMember("final_path", path_json, MDP::ResultsWriter::get_instance().get_json_allocator());
        path_result.AddMember("path_cost", path_cost, MDP::ResultsWriter::get_instance().get_json_allocator());
        //don't uncomment, can break output json
        // path_result.AddMember("path_length", path->length(), MDP::ResultsWriter::get_instance().get_json_allocator());
        // path_result.AddMember("is_path_valid", path->check(), MDP::ResultsWriter::get_instance().get_json_allocator());
        // path_result.AddMember("path_smoothnes", path->smoothness(), MDP::ResultsWriter::get_instance().get_json_allocator());
        // path_result.AddMember("path_clearence", path->clearance(), MDP::ResultsWriter::get_instance().get_json_allocator());
    
    }

    path_result.AddMember("min_time_bound", low_bound_time, MDP::ResultsWriter::get_instance().get_json_allocator());
    path_result.AddMember("min_frame_bound", low_bound_frame, MDP::ResultsWriter::get_instance().get_json_allocator());
    StrrtWritePlannerData(planner.get(), path_result, false);
    MDP::ResultsWriter::get_instance().save_json(std::to_string(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())), path_result, random_seed);
    return 0;
}