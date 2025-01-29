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
    bones = [bpy.context.view_layer.objects.active.pose.bones[bone_name] for bone_name in joint_order]
    
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
    

def animate_path(manipulator_path,fps,joint_order):

    for  time,path in manipulator_path:
        move_robot("robot",path,frame = int(time*fps), joint_order = joint_order)


def main(path_to_log_file:str):
    os.chdir(bpy.path.abspath("//"))
    os.chdir('../../')
    print(os.getcwd())
    planner_logs = open_json(path_to_log_file)#parse json file\
        
    #get blender file
    scene_task = open_json("C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/"+planner_logs["path_to_scene_json"])
    blender_file_path = scene_task["blender_file_path"]
    

    joint_order = scene_task['robot_joints_order']
    print(joint_order)
    move_robot("robot_start_pos",scene_task["start_configuration"],joint_order,0)
    move_robot("robot_goal_pos",scene_task["end_configuration"],joint_order,0)
    #strrt_config = open_json(planner_logs["path_to_strrt_config_json"])#get strrt config
    
    manipulator_path = planner_logs["path"]#get manipulator path
    
    
    animation_callback = functools.partial(animate_path,manipulator_path,scene_task['fps'],joint_order)
    animation_callback()
    
        
if __name__=="__main__":
    path_to_log_file = "C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/drgbt_logs_flying_box_simplified.json"
    assert(os.path.isfile(path_to_log_file),"no input json is given!")
    main(path_to_log_file)