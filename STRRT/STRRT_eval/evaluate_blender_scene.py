"""
Process blender scene and plan path for manipulator with STRRT*

 python3 ./evaluate_blender_scene.py --path_to_scene_json ./../scene_task.json --path_to_strrt_config_json ./../strrt_config.json 
"""
import argparse
import os
import numpy as np
from os.path import abspath, dirname, join
import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    sys.path.insert(
        0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from functools import partial
import pybullet as pyb
import pybullet_data
import json
import math
import time
import matplotlib.pyplot as plt
from scipy import interpolate
import datetime


class SpaceTimeStateValidator():
    """
    Functor for storing obstacle data and validation of space time position
    """

    def __init__(self,
                 obstacles: list,
                 fps: int,
                 frame_count: int,
                 robot_urdf: str,
                 robot_base_coords: list,
                 robot_base_quat_rot: list,
                 robot_joints_order: tuple[str],
                 show_GUI: bool) -> None:
        """Class constructor

        Args:
            obstacles (list): list of obstacles with positions
            fps (int): frames per second
            frame_count (int): total frame count
            robot_urdf (str): path to robot urdf
            robot_base_coords (list): coordinates of robot base
            robot_base_quat_rot (list): rotation of robot base in quaternion (x,y,z,w)
            robot_joints_order (tuple of str): names of moveble joints in order
            show_GUI (bool): show pybullet GUI if true
        """
        obstacle_dict = {}
        for obstacle in obstacles:
            obstacle_dict[obstacle['name']] = obstacle

        self.obstacles = obstacle_dict
        self.fps = fps
        self.frame_count = frame_count
        self.robot_urdf = robot_urdf
        self.robot_base_coords = robot_base_coords
        self.robot_base_quat_rot = robot_base_quat_rot
        self.robot_joints_order = robot_joints_order
        self.show_GUI = show_GUI
        self.validation_counter = 0  # counter of state validation calls

        if self.show_GUI:
            self.server_id = pyb.connect(pyb.GUI)
        else:
            self.server_id = pyb.connect(pyb.DIRECT)

        pyb.setAdditionalSearchPath(
            pybullet_data.getDataPath(), physicsClientId=self.server_id)

        self.robot_id = pyb.loadURDF(
            robot_urdf, robot_base_coords, robot_base_quat_rot)

        self.num_cubes = len(obstacles)
        self.cube_ids = {}
        self.obstacle_robots = {}
        for obstacle_name, obstacle in self.obstacles.items():
            if obstacle["type"] in ["dynamic_box","static_box"]:
                cube_id = pyb.createCollisionShape(pyb.GEOM_BOX, halfExtents=[
                    x/2 for x in obstacle['dimensions']])
                cube_visual_id = pyb.createVisualShape(pyb.GEOM_BOX, halfExtents=[
                    x/2 for x in obstacle['dimensions']], rgbaColor=[1, 0, 0, 1])
                cube_body_id = pyb.createMultiBody(
                    baseCollisionShapeIndex=cube_id, baseVisualShapeIndex=cube_visual_id, basePosition=0)
                self.cube_ids[obstacle_name] = cube_body_id
                pos = obstacle['positions'][0][:3]
                quat_rot = obstacle['positions'][0][3:]
                pyb.resetBasePositionAndOrientation(cube_body_id, pos, quat_rot)
            elif obstacle["type"] == "dynamic_robot":
                robot_id =  (pyb.loadURDF(
                obstacle["urdf_file_path"], obstacle["robot_base_coords"], obstacle["robot_base_quat_rot"]))
                joint_name2index={}
                joint_name2bounds = {}
                for joint_index in range(pyb.getNumJoints(robot_id)):
                    joint_info = pyb.getJointInfo(robot_id,joint_index)
                    if joint_info[1].decode('utf-8') in obstacle["robot_joints_order"]:
                        joint_name2index[joint_info[1].decode('utf-8')] = joint_index
                        joint_name2bounds[joint_info[1].decode('utf-8')] = (joint_info[8], joint_info[9])
                        
                self.obstacle_robots[obstacle_name] ={"robot_id":robot_id,
                                                      "robot_joints_order":robot_joints_order,
                                                      "joint_name2index":joint_name2index,
                                                      "joint_name2bounds":joint_name2bounds}
            
            self.joint_name2index={}
            self.joint_name2bounds = {}
            for joint_index in range(pyb.getNumJoints(self.robot_id)):
                joint_info = pyb.getJointInfo(self.robot_id,joint_index)
                if joint_info[1].decode('utf-8') in self.robot_joints_order:
                    self.joint_name2index[joint_info[1].decode('utf-8')] = joint_index
                    self.joint_name2bounds[joint_info[1].decode('utf-8')] = (joint_info[8], joint_info[9])
                
        self.bounds = []
        for joint_index, joint_name in enumerate(self.robot_joints_order):
            self.bounds.append(self.joint_name2bounds[joint_name])

    def __call__(self, spaceInformation, state) -> bool:

        self.validation_counter += 1
        t = spaceInformation.getStateSpace().getStateTime(state)

        for joint_index, joint_name in enumerate(self.robot_joints_order):
            joint_angle = state[0][joint_index]
            pyb.resetJointState(self.robot_id, self.joint_name2index[joint_name], joint_angle)

        for cube_name, cube_object in self.cube_ids.items():
            if self.obstacles[cube_name]['type'] == "dynamic_box":
                pos = self.obstacles[cube_name]['positions'][math.floor(
                    t*self.fps)][:3]
                quat_rot = self.obstacles[cube_name]['positions'][math.floor(
                    t*self.fps)][3:]
                pyb.resetBasePositionAndOrientation(cube_object, pos, quat_rot)

        for robot_obstacle_name, robot_object in self.obstacle_robots.items():
            for joint_index, joint_name in enumerate(robot_object["robot_joints_order"]):
                joint_angle = self.obstacles[robot_obstacle_name]['trajectory'][math.floor(
                    t*self.fps)][joint_index]
                pyb.resetJointState(robot_object["robot_id"], robot_object["joint_name2index"][joint_name], joint_angle)

        # Проверка коллизий для каждой части робота
        for joint_index, joint_name in enumerate(self.robot_joints_order):
            contacts = pyb.getContactPoints(self.robot_id, -1, joint_index)
            if contacts:
                return False
        for cube in self.cube_ids.values():
            contacts_cube1 = pyb.getClosestPoints(
                self.robot_id, cube, distance=0.0)
            if contacts_cube1:
                return False
        
        for robot_object in self.obstacle_robots.values():
            contacts_robot_obstacle = pyb.getClosestPoints(
                self.robot_id, robot_object["robot_id"], distance=0.0)
            if contacts_robot_obstacle:
                return False
            
        return True

    def get_robot_bounds(self) -> list[tuple]:
        """Returns bounds of robot from urdf

        Returns:
            list[tuple]: list of tuple(min,max)
        """
        
        return self.bounds

    def animate(self, data):
        if self.show_GUI:
            for configuration in data:
                for joint_index, joint_name in enumerate(self.robot_joints_order):
                    pyb.resetJointState(self.robot_id, self.joint_name2index[joint_name],
                                        configuration[joint_index])
                t = configuration[-1]
                for cube_name, cube_object in self.cube_ids.items():
                    if self.obstacles[cube_name]['type'] == "dynamic_box":
                        pos = self.obstacles[cube_name]['positions'][math.floor(
                            t*self.fps)][:3]
                        quat_rot = self.obstacles[cube_name]['positions'][math.floor(
                            t*self.fps)][3:]
                        pyb.resetBasePositionAndOrientation(cube_object, pos, quat_rot)
                time.sleep(1 / 24)

    def get_count_of_validation(self):
        return self.validation_counter

    def __del__(self):
        """
        destructor, disconnects pybullet
        """
        pyb.disconnect()


class SpaceTimeMotionValidator(ob.MotionValidator):
    def __init__(self, si, collision_check_interpolation: int,robot_joint_count:int):
        """Motion validator

        Args:
            si : ompl si
            collision_check_interpolation (int): how many points to validate between to states
            robot_joint_count (int) : robot joint count
        """
        super(SpaceTimeMotionValidator, self).__init__(si)
        self.si = si
        self.collision_check_interpolation = collision_check_interpolation
        self.validation_counter = 0  # counter of state validation calls
        self.robot_joint_count = robot_joint_count

    def checkMotion(self, s1, s2):
        self.validation_counter += 1
        if not self.si.isValid(s2):
            return False
        # delta_pos = self.si.getStateSpace().distanceSpace(s1, s2)
        delta_coords = np.abs(np.array([s2[0][i] for i in range(self.robot_joint_count)])-np.array([s1[0][i] for i in range(self.robot_joint_count)]))
        # delta_t = self.si.getStateSpace().distanceTime(s1, s2)
        t1 = self.si.getStateSpace().getStateTime(s1)
        t2 = self.si.getStateSpace().getStateTime(s2)
        delta_t = t2 - t1
        if delta_t <= 0:
            return False
        # if (delta_pos / delta_t) > self.si.getStateSpace().getVMax():
        #     return False
        if np.any((delta_coords / delta_t) > self.si.getStateSpace().getVMax()):
            return False
        interpolation_steps = int(np.linalg.norm(delta_coords)/self.collision_check_interpolation)+2
        for i in range(interpolation_steps):
            interpolated_state = self.si.getStateSpace().allocState()
            t = i/interpolation_steps  # [0,1]
            self.si.getStateSpace().interpolate(s1, s2, t, interpolated_state)
            # print(self.si.getStateSpace().getStateTime(s2) - self.si.getStateSpace().getStateTime(interpolated_state))
            if not self.si.isValid(interpolated_state):
                return False
        return True

    def get_count_of_validation(self):
        return self.validation_counter


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

    if (not os.path.isfile(args.path_to_scene_json)) \
            or (not os.path.isdir(args.path_to_result_folder)) \
                or (not os.path.isfile(args.path_to_strrt_config_json)):
        raise FileNotFoundError


def main(path_to_scene_json: str, path_to_result_folder: str, path_to_strrt_config_json: str) -> None:
    """Plan path for given blender scene with stRRT

    Args:
        path_to_scene_json (str): path to .json, where scene is described
        path_to_result_folder (str): path to folder, where results is stored
        path_to_strrt_config_json (str): path to .json, where strrt hyperparametrs is described
    """
    start_time = time.time()
    with open(path_to_scene_json, 'r') as file:
        data = file.read()

    # parse scene-task json data
    scene_parsed_data = json.loads(data)

    start_configuration = scene_parsed_data["start_configuration"]
    end_configuration = scene_parsed_data["end_configuration"]
    frame_count = scene_parsed_data["frame_count"]
    fps = scene_parsed_data["fps"]
    obstacles = scene_parsed_data["obstacles"]
    robot_urdf = scene_parsed_data["robot_urdf"]
    robot_base_coords = scene_parsed_data["robot_base_coords"]
    robot_base_quat_rot = scene_parsed_data["robot_base_quat_rot"]
    robot_joints_order = tuple(scene_parsed_data["robot_joints_order"])
    robot_joint_max_velocity = tuple(scene_parsed_data["robot_joint_max_velocity"])
    robot_joint_count = len(robot_joints_order)

    with open(path_to_strrt_config_json, 'r') as file:
        data = file.read()

    # parse planner json configurations
    planner_parsed_config = json.loads(data)
    max_planning_time_pts = planner_parsed_config["max_planning_time_pts"]
    max_time_to_move = planner_parsed_config["max_allowed_time_to_move"]
    show_GUI = planner_parsed_config["show_GUI"]
    collision_check_interpolation = planner_parsed_config["collision_check_interpolation_angle"]
    iteration_time_step_sec = planner_parsed_config["iteration_time_step_sec"]
    stop_if_path_found = planner_parsed_config["stop_if_path_found"]


    # initialise state validator
    state_validator_functor = SpaceTimeStateValidator(
        obstacles=obstacles,
        fps=fps,
        frame_count=frame_count,
        robot_urdf=robot_urdf,
        robot_base_coords=robot_base_coords,
        robot_base_quat_rot=robot_base_quat_rot,  # [x,y,z,w]
        robot_joints_order=robot_joints_order,
        show_GUI=show_GUI
    )

    vector_space = ob.RealVectorStateSpace(robot_joint_count)
    bounds = ob.RealVectorBounds(robot_joint_count)

    # set robot angle bounds from urdf
    robot_bounds = state_validator_functor.get_robot_bounds()
    for i in range(robot_joint_count):
        # check start and end configuration angle bounds also
        assert (start_configuration[i] > robot_bounds[i][0])
        assert (start_configuration[i] < robot_bounds[i][1])
        assert (end_configuration[i] > robot_bounds[i][0])
        assert (end_configuration[i] < robot_bounds[i][1])
        bounds.setLow(i, robot_bounds[i][0])
        bounds.setHigh(i, robot_bounds[i][1])
    vector_space.setBounds(bounds)

    space = ob.SpaceTimeStateSpace(vector_space, robot_joint_max_velocity[0])
    space.setTimeBounds(0.0, min(frame_count/fps, max_time_to_move))

    si = ob.SpaceInformation(space)
    motion_validator = SpaceTimeMotionValidator(
        si, collision_check_interpolation,robot_joint_count)
    si.setMotionValidator(motion_validator)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(
        partial(state_validator_functor, si)))
    si.setup()
    pdef = ob.ProblemDefinition(si)

    start = ob.State(space)
    goal = ob.State(space)
    for i in range(robot_joint_count):
        start()[0][i] = start_configuration[i]
        goal()[0][i] = end_configuration[i]
        

    pdef.setStartAndGoalStates(start, goal)

    strrt = og.STRRTstar(si)
    strrt.setRange(robot_joint_max_velocity[0])
    strrt.setProblemDefinition(pdef)
    strrt.setup()

    results = []
    total_time = 0
    # print(pdef.getGoal())
    # print("could sample ",pdef.getGoal().couldSample())
    # print("can sample ",pdef.getGoal().canSample())
    # print(pdef.getGoal())
    # print(dir(pdef.getGoal()))
    
    # raise BaseException
    for i in range(max_planning_time_pts//iteration_time_step_sec +1):
        print(total_time)
        total_time+=iteration_time_step_sec
        result = strrt.solve(iteration_time_step_sec)
        print("Done planning.")
        end_time = time.time()
        execution_time = end_time - start_time
        print(result)
        if result and pdef.hasSolution():
            path = []
            states = pdef.getSolutionPath().getStates()
            for point_ind in range(pdef.getSolutionPath().getStateCount()):
                point_time = space.getStateTime(states[point_ind])
                point = [states[point_ind][0][x] for x in range(robot_joint_count)]
                path.append([point_time, point])
            results.append({
                "total_time":total_time,
                "path_length":pdef.getSolutionPath().length(),
                "path":path
            })
            print("Found path of length", pdef.getSolutionPath().length())
            print("Path:", pdef.getSolutionPath().printAsMatrix())
            if stop_if_path_found:
                break
        else:
            print("No solution found.")

    # export data to json

    path = []
    states = pdef.getSolutionPath().getStates()
    for point_ind in range(pdef.getSolutionPath().getStateCount()):
        point_time = space.getStateTime(states[point_ind])
        point = [states[point_ind][0][x] for x in range(robot_joint_count)]
        path.append([point_time, point])

    data_to_export = {
        "planner_type": "STRRT*",
        "path_to_scene_json": path_to_scene_json,
        "path_to_strrt_config_json": path_to_strrt_config_json,
        "execution time ms": execution_time * 1000,
        "number of state validations": state_validator_functor.get_count_of_validation(),
        "number of motion validations": motion_validator.get_count_of_validation(),
        "path": path
    }

    with open(os.path.join(path_to_result_folder, f'strrt_planner_logs_{datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}.json'), 'w') as f:
        json.dump(data_to_export, f)

    if show_GUI:
        def parse_path(data_str):
            lines = data_str.split('\n')
            table = []
            for line in lines:
                if line.strip():
                    row = [float(num) for num in line.split()]
                    table.append(row)
            return np.array(table)
        path = parse_path(pdef.getSolutionPath().printAsMatrix())
        path.shape

        array = np.array(path)

        interp = []
        for i in range(array.shape[1]-1):
            interp_x = interpolate.interp1d(
                array[:, -1], array[:, i], kind='linear', fill_value='extrapolate')
            interp.append(interp_x)

        time_grid = np.arange(0, 100, 100/1000)

        # Interpolate x and y coordinates onto the time grid
        grid_data = []
        for i in interp:
            grid_data.append(i(time_grid))

        # Combine x and y grids into a single array
        interpolated_array = np.column_stack(
            (grid_data[0], grid_data[1], grid_data[2], grid_data[3], grid_data[4], grid_data[5],  time_grid))
        print(interpolated_array.shape)
        for i in range(len(interpolated_array)):
            if interpolated_array[i][-1] > array[-1][-1]:
                for j in range(len(interpolated_array[i])-1):
                    interpolated_array[i][j] = path[-1][j]

        interpolated_array1 = interpolated_array[interpolated_array[:, -1]
                                                 < path[-1, -1]*2]

        state_validator_functor.animate(interpolated_array1)
        
    return data_to_export


if __name__ == '__main__':
    args = parse_my_args()
    check_args(args)
    main(args.path_to_scene_json, args.path_to_result_folder,
         args.path_to_strrt_config_json)
