"""
Process multiagent mass test

python3 ./multiagent_eval/do_mass_test_multithread.py --path_to_tasks ./multiagent_tests --path_to_strrt_config_json ./STRRT/strrt_config.json 

"""

import argparse
import os
import json
import copy
import shutil
from scipy import interpolate
import numpy as np
from typing import List
from concurrent.futures import ProcessPoolExecutor
NUM_CPUS = 5



def create_one_agent_scene_task_json(start_configuration:List[float], goal_configuration:List[float], goal_count, start_frame:int, obstacles:List[dict], scene_parsed_data: dict, robot: dict,filename_prefix: str, file_dir_path:str, frame_count:int)->str:
    """
    Creates a one-agent scene task json file for a given robot
    
    Args:
        start_configuration (List[float]): The initial configuration of the robot
        goal_configuration (List[float]): The goal configuration of the robot   
        start_frame (int): The start frame of the robot
        obstacles (List[dict]): The obstacles in the scene
        scene_parsed_data (dict): The scene parsed data
        robot (dict): The robot to be planned
        filename_prefix (str): The prefix of the filename
        file_dir_path (str): The path to the file
        frame_count (int): Number of frames in scene
        
    Returns:
        str: The path to the created file
    """
    
    data = copy.deepcopy(scene_parsed_data)

    for key, value in robot.items(): # copy robot to scene_task.json
        data[key] = value 
        
    data['start_configuration'] = start_configuration
    data['end_configuration'] = goal_configuration
    data['frame_count'] = frame_count
    # add another robots as static obstacles, if they don't have a planned path
    planned_robots = [another_robot["name"] for another_robot in obstacles if 'name' in another_robot]
    planned_robots.append(robot["name"])
    for another_robot in data["robots"]:
        if another_robot["name"] in planned_robots:
            continue
        r = copy.deepcopy(another_robot)
        r["type"] = "dynamic_robot"
        r["trajectory"] = [r['start_configuration'] for _ in range(data['frame_count'])]
        r["urdf_file_path"] = 'Blender/robots/xarm6/xarm6.urdf' #r["robot_urdf"]
        data["obstacles"].append(r)
        
    del data["robots"]

    # add obstacles
    for obstacle in obstacles:
        
        if len(obstacle["trajectory"]) <= start_frame :
            # add as static obstacle
            r = copy.deepcopy(obstacle)
            r["type"] = "dynamic_robot"
            r["trajectory"] = [r["trajectory"][-1] for _ in range(data['frame_count'])]
            r["urdf_file_path"] = 'Blender/robots/xarm6/xarm6s.urdf' #r["robot_urdf"]
            continue
        
        new_obstacle = copy.deepcopy(obstacle)
        new_obstacle["trajectory"] = new_obstacle["trajectory"][start_frame:]
        if len(new_obstacle["trajectory"]) < data['frame_count']:
            new_obstacle["trajectory"].extend([new_obstacle["trajectory"][-1] for _ in range(data['frame_count'] - len(new_obstacle["trajectory"]))])
        data["obstacles"].append(new_obstacle)
        
    filename = filename_prefix +'scene_task_robot_'+robot["name"]+f"_goal_{goal_count}"+".json"
    file_path = os.path.join(file_dir_path,filename)
    # print(file_path)
    with open(file_path, 'w') as f:
        json.dump(data, f)

    return file_path


def interpolate_raw_path(strrtlogs, fps, frame_count):
    path = []
    for raw_path in strrtlogs["final_planner_data"]["final_path"]:
        path.append(list((*raw_path["robot_angles"], (raw_path["time"]*fps))))
        
    array = np.array(path)
    
    assert(len(array[:, -1].tolist()) == len(array[:, -1])) # check if all frame numbers are unique
    interp = []
    for i in range(array.shape[1]-1):
        interp_x = interpolate.interp1d(
            array[:, -1], array[:, i], kind='linear', fill_value='extrapolate')
        interp.append(interp_x)
    # print(np.max(array[:, -1]))
    time_grid = np.arange(0, np.ceil(np.max(array[:, -1])), 1)
    # print(time_grid)
    # Interpolate x and y coordinates onto the time grid
    grid_data = []
    for i in interp:
        grid_data.append(i(time_grid))

    # Combine x and y grids into a single array
    interpolated_array = np.column_stack(
        (grid_data[0], grid_data[1], grid_data[2], grid_data[3], grid_data[4], grid_data[5]))
    
    ##uncomment this if need frame_count and populate last frames with last pose
    # final_array = np.ones(shape = (frame_count,array.shape[1]-1))*array[-1][:-1]
    
    # final_array[:interpolated_array.shape[0],:] = interpolated_array[:,:]
    # return final_array.tolist()
    
    return interpolated_array.tolist()

def process_one_task_MSIRRT(test_dir_path:str ,path_to_strrt_config_json:str) -> None:
    """processes one multiagent task
    Args:
        test_dir_path (str): path to multiagent task
        
    """
    path_to_json = os.path.join(test_dir_path,"multiagent_scene_task.json")
    
    if os.path.exists(os.path.join(test_dir_path,"multiagent_scene_task.json","generated_tasks_msirrt","robot_trajectories.json")):
        return
    if os.path.exists(os.path.join(test_dir_path,"multiagent_scene_task.json","generated_tasks_msirrt")):
        shutil.rmtree(os.path.join(test_dir_path,"multiagent_scene_task.json","generated_tasks_msirrt"))
    
    with open(path_to_json, 'r') as file:
        data = file.read()

    # parse scene-task json data
    scene_parsed_data = json.loads(data)

    # for every robot in scene
    # create scene task for 1 robot
    # plan
    # get plan results, get path
    # create scene task for next robot

    planned_robots = []
    path_to_generated_tasks = os.path.join(test_dir_path,"generated_tasks_msirrt")
    # print(path_to_generated_tasks)
    if os.path.isdir(path_to_generated_tasks):
        shutil.rmtree(path_to_generated_tasks)
    os.makedirs(path_to_generated_tasks, exist_ok=False)
    
    init_scene_parsed_data = copy.deepcopy(scene_parsed_data)
    

    obstacles = []

    for robot in init_scene_parsed_data["robots"]:
        start_configuration = robot['start_configuration']
        result_trajectory = []
        start_frame = 0
        for goal_count, goal_configuration in enumerate(robot['end_configuration']):
            need_to_go = True
            max_frame_count = 600
            attempt=1
            while need_to_go:
                scene_task_filepath = create_one_agent_scene_task_json( 
                start_configuration, goal_configuration, goal_count, start_frame, obstacles,
                scene_parsed_data, robot, "msirrt_", path_to_generated_tasks,max_frame_count*attempt)
                print(
                    f"./MSIRRT/build/MSIRRT_Planner {scene_task_filepath} {path_to_generated_tasks} {path_to_strrt_config_json} 1")
                os.system(
                    f"./MSIRRT/build/MSIRRT_Planner {scene_task_filepath} {path_to_generated_tasks} {path_to_strrt_config_json} 1")
                
                result_filename = list(filter(lambda x: x.startswith("MSIRRT_planner_logs"), os.listdir(path_to_generated_tasks)))
                if len(result_filename) ==0:
                    attempt+=1
                    if attempt>10:
                        break
                    continue
                
                result_filename = result_filename[0]
                new_result_filename = f"start_frame_{start_frame}_" + result_filename[:-5] + f'_for_{robot["name"]}.json'
                new_result_filepath = os.path.join(path_to_generated_tasks,new_result_filename) 
                shutil.copyfile(os.path.join(path_to_generated_tasks,result_filename), new_result_filepath)
                os.remove(os.path.join(path_to_generated_tasks,result_filename))
                with open(new_result_filepath, 'r') as file:
                    strrtlogs_raw = file.read()
                
                #check if planning was successful
                strrtlogs = json.loads(strrtlogs_raw)
                # print(strrtlogs["final_planner_data"])
                if not strrtlogs["final_planner_data"]["has_result"]:
                    attempt+=1
                    if attempt>10:
                        break
                    continue
                break

            if not strrtlogs["final_planner_data"]["has_result"]:
                with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
                    obstacles["error"] = "no_path_found"
                    json.dump(obstacles, file)
                return 
            assert(strrtlogs["final_planner_data"]["has_result"])
            trajectory = []

            end_frame = int(np.ceil(strrtlogs['final_planner_data']['final_path'][-1]['time']*scene_parsed_data['fps']))
            trajectory = interpolate_raw_path(
                strrtlogs, scene_parsed_data['fps'], end_frame)
            result_trajectory.append(trajectory)
            start_frame += end_frame
            start_configuration = goal_configuration

        robot['type'] = "dynamic_robot"
        robot['trajectory'] = [configuration for trajectory in result_trajectory for configuration in trajectory]
        robot["urdf_file_path"] = robot["robot_urdf"]
        
        obstacles.append(copy.deepcopy(robot))
        # print([x["name"] for x in obstacles])
        
        with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
            json.dump(obstacles, file)
 
 

def process_one_task_STRRT(test_dir_path:str ,path_to_strrt_config_json:str) -> None:
    """processes one multiagent task
    Args:
        test_dir_path (str): path to multiagent task
        
    """
    path_to_json = os.path.join(test_dir_path,"multiagent_scene_task.json")
    
    if os.path.exists(os.path.join(test_dir_path,"multiagent_scene_task.json","generated_tasks_strrt","robot_trajectories.json")):
        return
    if os.path.exists(os.path.join(test_dir_path,"multiagent_scene_task.json","generated_tasks_strrt")):
        shutil.rmtree(os.path.join(test_dir_path,"multiagent_scene_task.json","generated_tasks_strrt"))
    
    with open(path_to_json, 'r') as file:
        data = file.read()

    # parse scene-task json data
    scene_parsed_data = json.loads(data)

    # for every robot in scene
    # create scene task for 1 robot
    # plan
    # get plan results, get path
    # create scene task for next robot

    planned_robots = []
    path_to_generated_tasks = os.path.join(test_dir_path,"generated_tasks_strrt")
    # print(path_to_generated_tasks)
    if os.path.isdir(path_to_generated_tasks):
        shutil.rmtree(path_to_generated_tasks)
    os.makedirs(path_to_generated_tasks, exist_ok=False)
    
    init_scene_parsed_data = copy.deepcopy(scene_parsed_data)
    

    obstacles = []

    for robot in init_scene_parsed_data["robots"]:
        start_configuration = robot['start_configuration']
        result_trajectory = []
        start_frame = 0
        for goal_count, goal_configuration in enumerate(robot['end_configuration']):
            need_to_go = True
            max_frame_count = 600
            attempt=1
            while need_to_go:
                scene_task_filepath = create_one_agent_scene_task_json( 
                start_configuration, goal_configuration, goal_count, start_frame, obstacles,
                scene_parsed_data, robot, "msirrt_", path_to_generated_tasks,max_frame_count*attempt)
                print(
                    f"./STRRT_Planner/build/STRRT_Planner {scene_task_filepath} {path_to_generated_tasks} {path_to_strrt_config_json} 1")
                os.system(
                    f"./STRRT_Planner/build/STRRT_Planner {scene_task_filepath} {path_to_generated_tasks} {path_to_strrt_config_json} 1")
                
                result_filename = list(filter(lambda x: x.startswith("STRRT*_planner_logs"), os.listdir(path_to_generated_tasks)))
                if len(result_filename) ==0:
                    attempt+=1
                    if attempt>10:
                        break
                    continue
                
                result_filename = result_filename[0]
                new_result_filename = f"start_frame_{start_frame}_" + result_filename[:-5] + f'_for_{robot["name"]}.json'
                new_result_filepath = os.path.join(path_to_generated_tasks,new_result_filename) 
                shutil.copyfile(os.path.join(path_to_generated_tasks,result_filename), new_result_filepath)
                os.remove(os.path.join(path_to_generated_tasks,result_filename))
                with open(new_result_filepath, 'r') as file:
                    strrtlogs_raw = file.read()
                
                #check if planning was successful
                strrtlogs = json.loads(strrtlogs_raw)
                # print(strrtlogs["final_planner_data"])
                if not strrtlogs["final_planner_data"]["has_result"]:
                    attempt+=1
                    if attempt>10:
                        break
                    continue
                break

            if not strrtlogs["final_planner_data"]["has_result"]:
                with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
                    obstacles["error"] = "no_path_found"
                    json.dump(obstacles, file)
                return 
            assert(strrtlogs["final_planner_data"]["has_result"])
            trajectory = []

            end_frame = int(np.ceil(strrtlogs['final_planner_data']['final_path'][-1]['time']*scene_parsed_data['fps']))
            trajectory = interpolate_raw_path(
                strrtlogs, scene_parsed_data['fps'], end_frame)
            result_trajectory.append(trajectory)
            start_frame += end_frame
            start_configuration = goal_configuration

        robot['type'] = "dynamic_robot"
        robot['trajectory'] = [configuration for trajectory in result_trajectory for configuration in trajectory]
        robot["urdf_file_path"] = robot["robot_urdf"]
        
        obstacles.append(copy.deepcopy(robot))
        # print([x["name"] for x in obstacles])
        
        with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
            json.dump(obstacles, file)
        
def main(path_to_tasks: str, path_to_strrt_config_json: str) -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt

    Args:
        path_to_tasks (str): path to multiagent tasks
        path_to_strrt_config_json (str): path to .json, where strrt hyperparametrs is described
    """

    tasks_by_robot_count = os.listdir(path_to_tasks)
        
    futures = []
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:
        for robot_count in tasks_by_robot_count:
            test_cases = os.listdir(os.path.join(path_to_tasks,robot_count))
            for test_case in test_cases:
                for i in range(10):
                    test_dir_path = os.path.join(path_to_tasks,robot_count,test_case)
                    ()
                    futures.append(executor.submit(process_one_task_MSIRRT, test_dir_path, path_to_strrt_config_json))
                    
        
    for future in futures:
        future.result()


def check_args(args):
    if not os.path.exists(args.path_to_tasks):
        raise FileNotFoundError(f"Tasks path {args.path_to_tasks} does not exist")
        
    if not os.path.exists(args.path_to_strrt_config_json):
        raise FileNotFoundError(f"STRRT config path {args.path_to_strrt_config_json} does not exist")
    
def parse_args():
    parser = argparse.ArgumentParser(description='Process multiagent mass test')
    parser.add_argument('--path_to_tasks', type=str, help='Path to multiagent tasks', default='./multiagent_tests')
    parser.add_argument('--path_to_strrt_config_json', type=str, help='Path to strrt config json', default='./STRRT/strrt_config.json')
    
    args = parser.parse_args()
    check_args(args)
    return args

if __name__ == '__main__':
    args = parse_args()
    
    main(path_to_tasks = args.path_to_tasks, path_to_strrt_config_json = args.path_to_strrt_config_json)


