"""
Create massive tests for testing multiagent execution

python3 ./multiagent_eval/create_mass_test_multithread.py

"""

import os
import json
import numpy as np
import copy
import subprocess
from concurrent.futures import ProcessPoolExecutor
NUM_CPUS = 8


def get_robot(name, coords, rotation):
    return {
        "name": name,
        "start_configuration": None,
        "end_configuration": None,
        "robot_urdf": "./Blender/robots/xarm6/xarm6cyl.urdf",
        "robot_joint_max_velocity": [3.1415, 3.1415, 3.1415, 3.1415, 3.1415, 3.1415],
        "robot_capsules_radius": [0.047, 0.12, 0.11, 0.09, 0.05, 0.038],
        "robot_joints_order": [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ],
        "robot_base_coords": coords,
        "robot_base_quat_rot": rotation,
    }


def create_scene(kolvo_robotov: int) -> dict:
    """Generates scene for kolvo_robotov number of robots, that placed in circle

    Args:
        kolvo_robotov (int): number of robots

    Returns:
        dict: generated json
    """

    default_scene = {
        "robots": [],
        "frame_count": 500,
        "fps": 30,
        "blender_file_path": "./Blender/scenes/flying boxes.blend",
        "obstacles": [],
    }
    workplace_radius = 0.55
    if kolvo_robotov == 8:
        workplace_radius = 0.6
    elif kolvo_robotov == 2:
        workplace_radius = 0.4
    for robot_number in range(kolvo_robotov):
        coords = [
            workplace_radius * np.cos(robot_number * 2 * np.pi / kolvo_robotov),
            workplace_radius * np.sin(robot_number * 2 * np.pi / kolvo_robotov),
            0,
        ]
        rotation = [0, 0, 0, 1]
        default_scene["robots"].append(
            get_robot(f"robot_number_{robot_number}", coords, rotation)
        )

    return default_scene


def is_collision(scene, get_angles_func,dir):

    data = copy.deepcopy(scene)
    for key, value in data["robots"][0].items():
        data[key] = value

    data["start_configuration"] = get_angles_func(data["robots"][0])
    data["end_configuration"] = get_angles_func(data["robots"][0])
    for robot in data["robots"][1:]:
        robot["type"] = "static_robot"
        robot["trajectory"] = [{"time":0.0,"robot_angles":get_angles_func(robot)}]
        robot["urdf_file_path"] = robot["robot_urdf"]
        data["obstacles"].append(robot)

    del data["robots"]
    data['frame_count']=1
    filename = os.path.join(dir,"check_collisions_start.json")

    with open(filename, "w") as f:
        json.dump(data, f)
    # print(f"./STRRT_Planner/build/check_scene {filename}")
    result = subprocess.run(
        f"./STRRT_Planner/build/check_scene {filename}", shell=True
    )
    if result.returncode != 0:
        # raise BaseException
        return True
    return False

def test_goal(scene, testing_robot, goal_angles, dir):
    data = copy.deepcopy(scene)
    for key, value in testing_robot.items():
        data[key] = value

    data["start_configuration"] = goal_angles
    data["end_configuration"] = goal_angles
    
    for another_robot in data["robots"]:
        if another_robot["name"] == testing_robot["name"]:
            continue
        another_robot["type"] = "static_robot"
        another_robot["trajectory"] = [{"time":0.0,"robot_angles":another_robot["start_configuration"]}]
        another_robot["urdf_file_path"] = another_robot["robot_urdf"]
        data["obstacles"].append(another_robot)

    del data["robots"]
    data['frame_count']=1
    filename = os.path.join(dir,"check_collisions_goal.json")

    with open(filename, "w") as f:
        json.dump(data, f)
    # print(f"./STRRT_Planner/build/check_scene {filename}")
    result = subprocess.run(
        f"./STRRT_Planner/build/check_scene {filename}", shell=True
    )
    if result.returncode != 0:
        # raise BaseException
        return True
    

    data = copy.deepcopy(scene)
    for key, value in testing_robot.items():
        data[key] = value

    # data["start_configuration"] = goal_angles
    data["end_configuration"] = goal_angles
    
    for another_robot in data["robots"]:
        if another_robot["name"] == testing_robot["name"]:
            continue
        another_robot["type"] = "dynamic_robot"
        another_robot["trajectory"] = [{"time":0.0,"robot_angles":another_robot["start_configuration"]}]
        another_robot["urdf_file_path"] = another_robot["robot_urdf"]
        data["obstacles"].append(another_robot)

    del data["robots"]
    data['frame_count']=500
    filename = os.path.join(dir,"check_collisions.json")

    with open(filename, "w") as f:
        json.dump(data, f)
    print(f"./MSIRRT/build/MSIRRT_Planner {filename} {dir} ./STRRT/strrt_config.json 1")
    result = subprocess.run(
        f"./MSIRRT/build/MSIRRT_Planner {filename} {dir} ./STRRT/strrt_config.json 1", shell=True
    )
    
    result_filename = os.path.join(dir,[x for x in os.listdir(dir) if 'MSIRRT' in x][0])
    # print(result_filename)
    with open(result_filename, "r") as f:
        plann_result  = json.load(f)
    os.remove(result_filename)
    if plann_result["final_planner_data"]["has_result"]:
        return False
    # raise BaseException
    return True

def get_random_angles(robot_urdf_path):
    # TODO: rewrite, read urdf and generate based on limits from uredf
    return [
        np.random.random() * 4 * 3.1415 - 2 * 3.1415,
        np.random.random() * 2 * 2.059 - 2.059,
        np.random.random() * (0.19198 + 3.927) - 3.927,
        np.random.random() * 4 * 3.1415 - 2 * 3.1415,
        np.random.random() * (3.14159265359 + 1.69297) - 1.69297,
        np.random.random() * 4 * 3.1415 - 2 * 3.1415,
    ]


def set_start_coordinates(scene: dict,dir) -> dict:
    unchanged_scene = copy.deepcopy(scene)
    nado = True
    while nado:
        print("create_start_coordinates")
        scene = copy.deepcopy(unchanged_scene)
        for robot in scene["robots"]:  # for every robot
            # sample random configuration
            # robot["start_configuration"] = get_random_angles(robot["robot_urdf"])
            robot["start_configuration"]=[0]*6
        # check if collision
        # if collision, repeat
        for i in range(len(scene['robots'])):
            nado = is_collision(scene, lambda s: s["start_configuration"],dir)
            scene['robots'].append(scene['robots'].pop(0))
            if nado:
                break
            
    return scene


def set_goal_coordinates(scene: dict, number_of_goals: int, dir) -> dict:
    for robot in scene["robots"]:
        robot["end_configuration"] = []
    unchanged_scene = copy.deepcopy(scene)
    for robot in scene["robots"]:  # for every robot
        for goal_number in range(number_of_goals):
            unchanged_scene = copy.deepcopy(scene)
            dont_have_a_goal = True
            while dont_have_a_goal:
                print("create_goal_coordinates")
                goal_angles = get_random_angles(robot["robot_urdf"])
                dont_have_a_goal = test_goal(scene,robot, goal_angles,dir)
                if dont_have_a_goal:
                    print(f"no goal{goal_number} found for {robot['name']}")
                
            robot["end_configuration"].append(
                goal_angles                    
            )
        
        #return to start after all goals
        robot["end_configuration"].append(
                robot["start_configuration"]                    
            )
        
    return scene


def save_test(scene: dict, path: str) -> None:
    with open(path, "w") as f:
        json.dump(scene, f)

def create_one_scene(kolvo_robotov,test_case_number,number_of_goals):
    scene = create_scene(kolvo_robotov)
    path = (
        f"multiagent_tests/{kolvo_robotov}_robots/test_number{test_case_number}"
    )
    os.makedirs(path, exist_ok=False)
    scene = set_start_coordinates(scene,path)  # create start coords
    scene = set_goal_coordinates(scene, number_of_goals,path)  # create goal coords
    save_test(scene, path+"/multiagent_scene_task.json")
    
def main() -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt"""


    robot_number = [2, 4, 6, 8]
    number_of_test_cases = 3
    number_of_goals = 10
    os.makedirs("multiagent_tests", exist_ok=False)
    
    futures = []
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:
        for kolvo_robotov in robot_number:
            os.makedirs(f"multiagent_tests/{kolvo_robotov}_robots", exist_ok=False)
            for test_case_number in range(number_of_test_cases):
                futures.append(executor.submit(create_one_scene,kolvo_robotov,test_case_number,number_of_goals))
        
    for future in futures:
        future.result()
        
if __name__ == "__main__":
    main()
