"""
Process multiagent mass test

python3 ./multiagent_eval/do_mass_test_multithread.py --path_to_tasks ./multiagent_tests --path_to_strrt_config_json ./STRRT/strrt_config.json 

"""
import random
import argparse
import os
import json
import copy
import time
from scipy import interpolate
import numpy as np
from typing import List
from concurrent.futures import ProcessPoolExecutor
from tqdm import tqdm
import subprocess
NUM_CPUS = 128
MAX_ATTEMPTS=5
MAX_FILE_FIND_ATTEMPS = 10
SAVE_INTERMEDIATE_RESULT = False
NUMBER_OF_SEED_ITERATIONS = 10

def create_one_agent_scene_task_json(start_configuration:List[float], goal_configuration:List[float], goal_count, start_time:float, obstacles:List[dict], scene_parsed_data: dict, robot: dict,filename_prefix: str, file_dir_path:str, frame_count:int,attempt)->str:
    """
    Creates a one-agent scene task json file for a given robot
    
    Args:
        start_configuration (List[float]): The initial configuration of the robot
        goal_configuration (List[float]): The goal configuration of the robot   
        start_time (float): The start time of the robot
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
    data['fps'] = 30
    data["urdf_file_path"] = 'Blender/robots/xarm6/xarm6cyl.urdf' #r["robot_urdf"]
    data['start_time'] = start_time
    
    # add another robots as static obstacles, if they don't have a planned path
    planned_robots = [another_robot["name"] for another_robot in obstacles if 'name' in another_robot]
    planned_robots.append(robot["name"])
    for another_robot in data["robots"]:
        if another_robot["name"] in planned_robots:
            continue
        r = copy.deepcopy(another_robot)
        r["type"] = "static_robot"
        r["trajectory"] = [{"time": 0.0,
                            "robot_angles": r['start_configuration']}]
        r["urdf_file_path"] = 'Blender/robots/xarm6/xarm6cyl.urdf' #r["robot_urdf"]
        data["obstacles"].append(r)
        
    del data["robots"]

    # add obstacles
    for obstacle in obstacles:
        new_obstacle = copy.deepcopy(obstacle)
        data["obstacles"].append(new_obstacle)
        
    filename = filename_prefix +'scene_task_robot_'+robot["name"]+f"_goal_{goal_count}_attempt_{attempt}"+".json"
    file_path = os.path.join(file_dir_path,filename)
    # print(file_path)
    with open(file_path, 'w') as f:
        json.dump(data, f)
    q=0
    while True:
        q+=1
        try:
            with open(file_path, 'r') as f:
                zsdasf = json.load(f)
                z = zsdasf["start_configuration"]
                break
        except BaseException:
            print(f"waiting json dump of file {file_path}")
            time.sleep(5)
        if q >10:
            with open(file_path, 'w') as f:
                json.dump(data, f)
        time.sleep(0.1)
    return file_path


def process_one_task(test_dir_path:str ,path_to_strrt_config_json:str,seed,test_count_number,full_test_number, planner_bin_path:str,planner_prefix:str,planner_logs_prefix:str) -> None:
    """processes one multiagent task
    Args:
        test_dir_path (str): path to multiagent task
        
    """
    path_to_json = os.path.join(test_dir_path,"multiagent_scene_task.json")
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
    path_to_generated_tasks = os.path.join(test_dir_path,f"generated_tasks_{planner_prefix}_seed_{seed}")
    # print(path_to_generated_tasks)
    # if os.path.isdir(path_to_generated_tasks):
    #     shutil.rmtree(path_to_generated_tasks)
    if os.path.exists(path_to_generated_tasks):
        return 
    os.makedirs(path_to_generated_tasks, exist_ok=False)
    
    init_scene_parsed_data = copy.deepcopy(scene_parsed_data)
    

    obstacles = []

    for robot in init_scene_parsed_data["robots"]:
        start_configuration = robot['start_configuration']
        result_trajectory = []
        start_time = 0.0
        for goal_count, goal_configuration in enumerate(robot['end_configuration']):
            need_to_go = True
            attempt=1
            strrtlogs = {}
            strrtlogs["final_planner_data"] = {}
            strrtlogs["final_planner_data"]["has_result"] = False
            while need_to_go:
                frame_count = int(500*attempt**1.3) # heuristic
                
                scene_task_filepath = create_one_agent_scene_task_json( 
                start_configuration, goal_configuration, goal_count, start_time+0.02, obstacles,
                scene_parsed_data, robot, f"{planner_prefix}_", path_to_generated_tasks,frame_count*attempt,attempt)

                command = f"{planner_bin_path} {scene_task_filepath} {path_to_generated_tasks} {path_to_strrt_config_json} {seed}"
                
                # print(command)
                result = subprocess.run(command, shell=True)
                if result.returncode != 0:
                    print("error occurred!")
                    print(command)
                    attempt+=1
                    if attempt>MAX_ATTEMPTS:
                        break
                    continue
                
                file_finding_attempt = 0
                while True:
                    file_finding_attempt+=1
                    if file_finding_attempt>MAX_FILE_FIND_ATTEMPS:
                        attempt+=1
                        break
                    result_filename = list(filter(lambda x: x.startswith(planner_logs_prefix), os.listdir(path_to_generated_tasks)))
                    if len(result_filename) == 0:
                        assert(file_finding_attempt<=MAX_FILE_FIND_ATTEMPS)
                        print("didn't found file, waiting...")
                        time.sleep(1)
                        continue
                    try:
                        result_filename = result_filename[0]
                        new_result_filename = f"start_time_{start_time}_" + result_filename[:-5] + f'_for_{robot["name"]}.json'
                        new_result_filepath = os.path.join(path_to_generated_tasks,new_result_filename) 
                        
                        os.rename(os.path.join(path_to_generated_tasks,result_filename), new_result_filepath)
                        with open(new_result_filepath, 'r') as file:
                            strrtlogs_raw = file.read()
                        
                        #check if planning was successful
                        strrtlogs = json.loads(strrtlogs_raw)
                        break
                    except json.JSONDecodeError as e:
                        print(f"json decode error: {e}")
                        assert(file_finding_attempt<=MAX_FILE_FIND_ATTEMPS)
                        time.sleep(1)
                        continue
                    except UnicodeDecodeError as e:
                        print(f"UnicodeDecodeError error: {e}, {command}")
                        assert(file_finding_attempt<=MAX_FILE_FIND_ATTEMPS)
                        time.sleep(1)
                        continue
                    
                if file_finding_attempt>MAX_FILE_FIND_ATTEMPS:
                    continue
                # print(strrtlogs["final_planner_data"])
                assert(strrtlogs["path_to_scene_json"] == scene_task_filepath)

                if not strrtlogs["final_planner_data"]["has_result"]:
                    attempt+=1
                    if attempt>MAX_ATTEMPTS:
                        break
                    continue
                break

            if not strrtlogs["final_planner_data"]["has_result"]:
                with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
                    obstacles.append({"result":False})
                    obstacles.append({"error":"no_path_found"})
                    json.dump(obstacles, file)
                return 
            assert(strrtlogs["final_planner_data"]["has_result"])

            end_time = strrtlogs['final_planner_data']['final_path'][-1]['time']
            result_trajectory.append(strrtlogs["final_planner_data"]["final_path"])
            start_time = end_time 
            start_configuration = goal_configuration

        robot['type'] = "dynamic_robot"
        robot['trajectory'] = [configuration for trajectory in result_trajectory for configuration in trajectory]
        robot["urdf_file_path"] = robot["robot_urdf"]
        # print(test_dir_path)
        for i in range(len(robot['trajectory'])-1):
            if not robot['trajectory'][i]["time"]-0.0001 <= robot['trajectory'][i+1]["time"]:
                print(robot['trajectory'][i]["time"],robot['trajectory'][i+1]["time"])
                print(test_dir_path, test_count_number,robot["name"])
                print(result_trajectory)
            assert(robot['trajectory'][i]["time"]-0.0001 <= robot['trajectory'][i+1]["time"])

        obstacles.append(copy.deepcopy(robot))

        if SAVE_INTERMEDIATE_RESULT:
            with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
                json.dump(obstacles, file)

    with open(os.path.join(path_to_generated_tasks,"robot_trajectories.json"), 'w') as file:
        obstacles.append({"result":True})
        json.dump(obstacles, file)
    print(f'done test №{test_count_number}/{full_test_number}')
 
 
def main(path_to_tasks: str, path_to_strrt_config_json: str) -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt

    Args:
        path_to_tasks (str): path to multiagent tasks
        path_to_strrt_config_json (str): path to .json, where strrt hyperparametrs is described
    """

    tasks_by_robot_count = os.listdir(path_to_tasks)
        
    futures = []
    tests = []
    for robot_count in tasks_by_robot_count:
        test_cases = os.listdir(os.path.join(path_to_tasks,robot_count))
        for test_case in test_cases:
            test_dir_path = os.path.join(path_to_tasks,robot_count,test_case)
            tests.append(test_dir_path)
        
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:
        c = 0
        NUMBER_OF_SEED_ITERATIONS = 5
        random_seeds = [random.randint(0, 1000000) for _ in range(NUMBER_OF_SEED_ITERATIONS)]
        for test_dir_path in tqdm(tests):
            for i in random_seeds:
                c+=1
                # futures.append(executor.submit(process_one_task, test_dir_path, path_to_strrt_config_json,i,c,len(tests)*NUMBER_OF_SEED_ITERATIONS,'./MSIRRT/build/MSIRRT_Planner','msirrt',"MSIRRT_planner_logs"))
                futures.append(executor.submit(process_one_task, test_dir_path, path_to_strrt_config_json,i,c,len(tests)*NUMBER_OF_SEED_ITERATIONS,'./STRRT_Planner/build/STRRT_Planner','strrt',"STRRT*_planner_logs"))
                    
        
    for future in tqdm(futures):
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


