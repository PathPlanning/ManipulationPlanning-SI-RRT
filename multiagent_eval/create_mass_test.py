"""
create massive tests for testing multiagent execution

 python3 ./multiagent_eval/create_mass_test.py

"""

import os
import json
import numpy as np
import copy
import subprocess


def get_robot(name, coords, rotation):
    return {
        "name": name,
        "start_configuration": None,
        "end_configuration": None,
        "robot_urdf": "./Blender/robots/xarm6/xarm6vis.urdf",
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
        "frame_count": 3000,
        "fps": 30,
        "blender_file_path": "./Blender/scenes/flying boxes.blend",
        "obstacles": [],
    }
    workplace_radius = 0.35

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


def is_collision(scene, get_angles_func):

    data = copy.deepcopy(scene)
    for key, value in data["robots"][0].items():
        data[key] = value

    data["start_configuration"] = get_angles_func(data["robots"][0])
    data["end_configuration"] = get_angles_func(data["robots"][0])
    for robot in data["robots"][1:]:
        robot["type"] = "dynamic_robot"
        robot["positions"] = [get_angles_func(robot)]
        robot["urdf_file_path"] = robot["robot_urdf"]
        data["obstacles"].append(robot)

    del data["robots"]
    data['frame_count']=1
    filename = "check_collisions.json"

    with open(filename, "w") as f:
        json.dump(data, f)

    result = subprocess.run(
        f"./STRRT_Planner/build/check_scene ./check_collisions.json", shell=True
    )
    if result.returncode != 0:
        return True
    return False


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


def set_start_coordinates(scene: dict) -> dict:
    unchanged_scene = copy.deepcopy(scene)
    nado = True
    while nado:
        print("create_start_coordinates")
        scene = copy.deepcopy(unchanged_scene)
        for robot in scene["robots"]:  # for every robot
            # sample random configuration
            robot["start_configuration"] = get_random_angles(robot["robot_urdf"])

        # check if collision
        # if collision, repeat
        nado = is_collision(scene, lambda s: s["start_configuration"])
    return scene


def set_goal_coordinates(scene: dict, number_of_goals: int) -> dict:
    for robot in scene["robots"]:
        robot["end_configuration"] = []
    unchanged_scene = copy.deepcopy(scene)
    for goal_number in range(number_of_goals):
        unchanged_scene = copy.deepcopy(scene)
        nado = True
        
        while nado:
            print("create_goal_coordinates")
            scene = copy.deepcopy(unchanged_scene)
            for robot in scene["robots"]:  # for every robot
                
                # sample random configuration
                robot["end_configuration"].append(
                    get_random_angles(robot["robot_urdf"])
                )
            # check if collision
            # if collision, repeat
            nado = is_collision(scene, lambda s: s["end_configuration"][goal_number])
        
    return scene


def save_test(scene: dict, path: str) -> None:
    with open(path, "w") as f:
        json.dump(scene, f)


def main() -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt"""

    robot_number = [2, 4, 6, 8]
    number_of_test_cases = 100
    number_of_goals = 10
    os.makedirs("multiagent_tests", exist_ok=False)
    for kolvo_robotov in robot_number:
        os.makedirs(f"multiagent_tests/{kolvo_robotov}_robots", exist_ok=False)
        scene = create_scene(kolvo_robotov)
        for test_case_number in range(number_of_test_cases):
            path = (
                f"multiagent_tests/{kolvo_robotov}_robots/test_number{test_case_number}"
            )
            os.makedirs(path, exist_ok=False)
            scene = set_start_coordinates(scene)  # create start coords
            scene = set_goal_coordinates(scene, number_of_goals)  # create goal coords
            save_test(scene, path+"/multiagent_scene_task.json")


if __name__ == "__main__":
    main()
