import bpy
import json
import xml.etree.ElementTree as ET
import math
from typing import List, Tuple, Dict, Optional
import os


import bpy
import mathutils
import math,os
from typing import Mapping, MutableMapping, Sequence, Optional
import json
import time
import functools

def get_path_from_file(path_file:str)->list[tuple[float,list]]:
    """Get path from file

    Arguments:
        path_file: path to file, containing manipulator path (from raw DRGBT log)

    Returns:
        list[tuple[float,list]]: list of states with time and list of angles
    """
    path_time = []

    with open(path_file,'r') as f:
        for line in f:
            path_str, time_str = line.split(';')
            time = float(time_str)
            q = list2 = [float(x) for x in path_str[1:-1].split() if x]
            path_time.append((time,q))
    return path_time



def move_robot(robot_name:str,angles:Sequence[float],joint_order:tuple[str],frame:Optional[int]= None):
    """
    Moves robot joints

    Args:
        robot_name: str name of robot in blender
        angles: list of 6 angles in radians
        joint_order: tuple of joint names in order
        frame: frame, to which it will insert keyframe. if None, no keyframe will be created
        
    """
    assert(len(angles) == len(joint_order))
    bpy.context.view_layer.objects.active = bpy.data.objects[robot_name]
    bones = [bpy.context.view_layer.objects[robot_name].pose.bones[bone_name] for bone_name in joint_order]
    
    for joint, angle in zip(bones, angles):
        joint.rotation_euler = (0.0,angle,0.0)
        
    if frame is not None:
#        bpy.context.scene.frame_set(frame=frame)
#        bpy.ops.anim.keyframe_insert(type='Rotation')
        for joint in bones:
#            bpy.ops.anim.keyframe_insert(type='DEFAULT')
            joint.keyframe_insert('rotation_euler', frame=frame)

    

def open_json(json_path:str)->dict:
    """open json file

    Args:
        json_path (str): path to json file

    Raises:
        FileNotFoundError: if file not found

    Returns:
        dict: json struct
    """
    if not os.path.isfile(json_path):
        raise FileNotFoundError
    
    with open(json_path,'r') as f:
        print(json_path)
        data = json.load(f)
    return data

# angles1 = [0]*6
# move_robot(angles1,frame = 1)

# angles1 = [3.1415/5]*6
# move_robot(angles1,frame = 50)

# for path, time in path_time:
#     move_robot(path,frame = int(time*30))
    

def animate_trajectory(manipulator_path,joint_order,robot_name):

    for  c,state in enumerate(manipulator_path):
        move_robot(robot_name,state,frame = c, joint_order = joint_order)

def create_robot(robot_json_data:Dict)->None:
    """
    Create robot at blender scene with its json description 
    """
    
    bpy.ops.import_robot.urdf_filebrowser(filepath=robot_json_data['robot_urdf'])
    
    new_robot = False
    
    for obj in bpy.context.scene.objects:
        print(obj.name)
        if obj.name =='Armature':
            new_robot = obj
            break
        
    assert(new_robot)
    
    new_robot.name = robot_json_data['name']
    
    new_robot.location = tuple(robot_json_data['robot_base_coords'])
    
    #xyzw to wxyz 
    rot = list(robot_json_data['robot_base_quat_rot'])
    
    rot.insert(rot.pop(),0)
    
    new_robot.rotation_quaternion = rot
    
    #set trajectories
    animation_callback = functools.partial(animate_trajectory,robot_json_data['trajectory'],robot_json_data['robot_joints_order'],robot_json_data['name'])
    animation_callback()
    
def main(json_path):    
    # for each robot instance in json
    # create robot
    # set trajectory
    
    if not os.path.isfile(json_path):
        raise FileNotFoundError
    
    with open(json_path,'r') as f:
        robot_trajectories = json.load(f) 
    
    for robot in robot_trajectories:
        create_robot(robot)
        


if __name__=="__main__":
    json_path = r""
    main(json_path)