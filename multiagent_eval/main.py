"""
Process multiagent blender scene and plan path for manipulator with STRRT* and DRGBT

 python3 ./main.py --path_to_scene_json ./../scene_task_multiagent.json --path_to_strrt_config_json ./STRRT/strrt_config.json 

"""

import argparse
import os
import json
import datetime
import copy
import shutil
from scipy import interpolate
import numpy as np


def parse_my_args(argv=None):
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument("--path_to_scene_json",
                        type=str,
                        default='./scene_task.json',
                        help="path to .json, where scene is described")
    PARSER.add_argument("--path_to_result_folder",
                        type=str,
                        default='./',
                        help="path to folder, where results is stored")
    PARSER.add_argument("--path_to_strrt_config_json",
                        type=str,
                        default='./strrt_config.json',
                        help="path to .json, where strrt hyperparametrs is described")
    # parse the arguments
    return PARSER.parse_args(argv)


def check_args(args):
    """
    Checks command line arguments, and throws exceptions, if pathes are invalid
    """
    return True
    if (not os.path.isfile(args.path_to_scene_json)) \
            or (not os.path.isdir(args.path_to_result_folder)) \
    or (not os.path.isfile(args.path_to_strrt_config_json)):
        raise FileNotFoundError


def create_one_agent_scene_task_json(scene_parsed_data: dict, robot: dict,filename_prefix: str):
    data = copy.deepcopy(scene_parsed_data)
    print(data.keys())
    del data["plannable_robots"]

    for key, value in robot.items():
        data[key] = value

    filename = filename_prefix +'scene_task_'+robot["name"]+".json"

    with open(filename, 'w') as f:
        json.dump(data, f)

    return filename


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

    time_grid = np.arange(0, np.ceil(np.max(array[:, -1])), 1)
    print(time_grid)
    # Interpolate x and y coordinates onto the time grid
    grid_data = []
    for i in interp:
        grid_data.append(i(time_grid))

    # Combine x and y grids into a single array
    interpolated_array = np.column_stack(
        (grid_data[0], grid_data[1], grid_data[2], grid_data[3], grid_data[4], grid_data[5]))
    
    final_array = np.ones(shape = (frame_count,array.shape[1]-1))*array[-1][:-1]
    
    final_array[:interpolated_array.shape[0],:] = interpolated_array[:,:]
    return final_array.tolist()

def main(path_to_scene_json: str, path_to_result_folder: str, path_to_strrt_config_json: str) -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt

    Args:
        path_to_scene_json (str): path to .json, where scene is described
        path_to_result_folder (str): path to folder, where results is stored
        path_to_strrt_config_json (str): path to .json, where strrt hyperparametrs is described
    """

    with open(path_to_scene_json, 'r') as file:
        data = file.read()

    # parse scene-task json data
    scene_parsed_data = json.loads(data)

    # for every robot in scene
    # create scene task for 1 robot
    # plan
    # get plan results, get path
    # create scene task for next robot

    planned_robots = []
    os.makedirs("multiagent", exist_ok=True)
    init_scene_parsed_data = copy.deepcopy(scene_parsed_data)
    for robot in scene_parsed_data["plannable_robots"]:

        scene_task_filename = create_one_agent_scene_task_json(
            scene_parsed_data, robot, "strrt_")
        os.chdir("../")
        print(
            f"./STRRT_Planner/build/STRRT_Planner ./multiagent_eval/{scene_task_filename} ./multiagent_eval/multiagent {path_to_strrt_config_json}")
        os.system(
            f"./STRRT_Planner/build/STRRT_Planner ./multiagent_eval/{scene_task_filename} ./multiagent_eval/multiagent {path_to_strrt_config_json}")
        os.chdir("./multiagent_eval")
        result_filename = os.listdir('./multiagent')[0]
        new_result_filename = result_filename[:-
                                              5] + f'_for_{robot["name"]}.json'
        shutil.copyfile('./multiagent/'+result_filename, new_result_filename)
        os.remove('./multiagent/'+result_filename)
        with open(new_result_filename, 'r') as file:
            strrtlogs_raw = file.read()
        strrtlogs = json.loads(strrtlogs_raw)
        print(strrtlogs["final_planner_data"])
        assert (strrtlogs["final_planner_data"]["has_result"])
        trajectory = []

        trajectory = interpolate_raw_path(
            strrtlogs, scene_parsed_data['fps'], scene_parsed_data['frame_count'])
        robot['type'] = "dynamic_robot"
        robot['trajectory'] = trajectory
        robot["urdf_file_path"] = robot["robot_urdf"]
        planned_robots.append(robot)
        scene_parsed_data["obstacles"].append(robot)
        
    scene_parsed_data = copy.deepcopy(init_scene_parsed_data)
    
    for robot in scene_parsed_data["plannable_robots"]:

        scene_task_filename = create_one_agent_scene_task_json(
            scene_parsed_data, robot, "drgbt_")
        os.chdir("../")
        print(
            f"./RPMPLv2/build/src/main ./multiagent_eval/{scene_task_filename} ./multiagent_eval/multiagent {path_to_strrt_config_json}")
        os.system(
            f"./RPMPLv2/build/src/main ./multiagent_eval/{scene_task_filename} ./multiagent_eval/multiagent {path_to_strrt_config_json}")
        os.chdir("./multiagent_eval")
        result_filename = os.listdir('./multiagent')[0]
        new_result_filename = result_filename[:-
                                              5] + f'_for_{robot["name"]}.json'
        shutil.copyfile('./multiagent/'+result_filename, new_result_filename)
        os.remove('./multiagent/'+result_filename)
        with open(new_result_filename, 'r') as file:
            strrtlogs_raw = file.read()
        strrtlogs = json.loads(strrtlogs_raw)
        print(strrtlogs["final_planner_data"])
        assert (strrtlogs["final_planner_data"]["has_result"])
        trajectory = []

        trajectory = interpolate_raw_path(
            strrtlogs, scene_parsed_data['fps'], scene_parsed_data['frame_count'])
        robot['type'] = "dynamic_robot"
        robot['trajectory'] = trajectory
        robot["urdf_file_path"] = robot["robot_urdf"]
        planned_robots.append(robot)
        scene_parsed_data["obstacles"].append(robot)



if __name__ == '__main__':
    args = parse_my_args()
    check_args(args)
    main(args.path_to_scene_json, args.path_to_result_folder,
         args.path_to_strrt_config_json)
