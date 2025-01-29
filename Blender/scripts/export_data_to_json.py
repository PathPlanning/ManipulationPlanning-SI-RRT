import bpy
import json
import xml.etree.ElementTree as ET
import math
from typing import List, Tuple, Dict, Optional

def parse_urdf(file_path: str) -> Tuple[List[str], Dict[str, Optional[Tuple[float, float]]]]:
    """
    Parses a URDF file to extract the names of movable joints and their bounds.

    Parameters:
    file_path (str): The path to the URDF file.

    Returns:
    Tuple[List[str], Dict[str, Optional[Tuple[float, float]]]]:
        A tuple containing:
        - A list of movable joint names.
        - A dictionary where the keys are joint names and the values are tuples
          of (lower_bound, upper_bound) for the joint limits. If a joint does not
          have limits specified, the value will be None.
    
    Movable joints are considered to be of types 'revolute' and 'prismatic'.
    """
    tree = ET.parse(file_path)
    root = tree.getroot()

    movable_joints: List[str] = []
    joint_bounds: Dict[str, Optional[Tuple[float, float]]] = {}

    for joint in root.findall('joint'):
        joint_type = joint.get('type')
        if joint_type in ['revolute', 'prismatic']:
            joint_name = joint.get('name')
            movable_joints.append(joint_name)
            
            limit = joint.find('limit')
            if limit is not None:
                lower = limit.get('lower')
                upper = limit.get('upper')
                joint_bounds[joint_name] = (float(lower), float(upper))
            else:
                joint_bounds[joint_name] = None
    return movable_joints, joint_bounds




def get_robot_angles(obj: bpy.types.Object, 
                              movable_bones: List[str], 
                              rotation_bounds: Dict[str, Tuple[float, float]]) -> Dict[str, float]:
    """
    Retrieve the Y-axis rotations of specified bones in the given Blender object,
    adjusting rotations to satisfy specified bounds if necessary.

    Parameters:
    obj (bpy.types.Object): The Blender object containing the armature.
    movable_bones (List[str]): List of names of bones to check and adjust.
    rotation_bounds (Dict[str, Tuple[float, float]]): Dictionary with bone names as keys and tuples of (lower_bound, upper_bound) as values.

    Returns:
    Dict[str, float]: A dictionary with bone names as keys and their Y-axis rotations as values.
    """
    if obj.type != 'ARMATURE':
        raise ValueError("The provided object is not an armature.")
    
    bone_rotations: Dict[str, float] = {}
    
    for bone in obj.pose.bones:
        if bone.name in movable_bones:
            y_rotation = bone.matrix_basis.to_euler('XYZ')[1]


            if bone.name in rotation_bounds:
                lower_bound, upper_bound = rotation_bounds[bone.name]
                
                y_rotation = y_rotation + 2 * math.pi*math.floor(max(0,(lower_bound - y_rotation)//(2 * math.pi))) \
                    - 2 * math.pi*math.floor(max(0,(y_rotation - upper_bound)//(2 * math.pi)))
                # Adjust y_rotation to fit within the bounds

                
                # If still out of bounds, set to nearest bound
                if y_rotation < lower_bound:
                    y_rotation = lower_bound
                elif y_rotation > upper_bound:
                    y_rotation = upper_bound
            
            bone_rotations[bone.name] = y_rotation
    
    return bone_rotations


def get_object_positions(obj, end_frame, in_euler: bool = False):
    """
    Get obj position in frame

    Args:
        obj: blender object
        end_frame:int, number of last frame
        in_euler:bool if true, angle returned in euler, else returned in quaternion [x,y,z,w]

    Returns:
        list[list[float]] list of position at each frame
    """

    positions = []

    # Iterate through frames
    for frame in range(0, end_frame + 1):
        # Set the current frame
        bpy.context.scene.frame_set(frame)
        pos = list(obj.matrix_world.to_translation())  # to_quaternion
        if in_euler:
            rotation = list(obj.matrix_world.to_euler('XYZ'))
        else:
            quat = obj.matrix_world.to_quaternion()
            rotation = [quat.x, quat.y, quat.z, quat.w]
        positions.append((*pos, *rotation))

    return positions

def is_object_animated(obj):
    """
    Check if a Blender object or any of its parents are animated in the scene,
    including path animations on curve objects.

    Parameters:
    obj (bpy.types.Object): The Blender object to check.

    Returns:
    bool: True if the object or any of its parents are animated, False otherwise.
    """
    def is_animated(o):
        """Check if the specific object is animated."""
        # Check if the object has animation data
        if o.animation_data is not None:
            # Check if there are any action keyframes
            if o.animation_data.action is not None:
                if o.animation_data.action.fcurves:
                    return True

            # Check if there are any drivers
            if o.animation_data.drivers:
                return True

        # Check if the object has shape key animation
        if hasattr(o.data, 'shape_keys') and o.data.shape_keys is not None:
            if o.data.shape_keys.animation_data is not None:
                if o.data.shape_keys.animation_data.action is not None:
                    if o.data.shape_keys.animation_data.action.fcurves:
                        return True

        # Check if the object has constraints, specifically Follow Path
        if o.constraints is not None:
            for constraint in o.constraints:
                if constraint.type == 'FOLLOW_PATH':
                    if constraint.target and constraint.target.animation_data:
                        return True


        return False

    # Check the object itself
    if is_animated(obj):
        return True

    # Check all parent objects
    parent = obj.parent
    while parent is not None:
        if is_animated(parent):
            return True
        parent = parent.parent

    # Check if the object's parent or itself is a curve with path animation
    def has_path_animation(o):
        if o.type == 'CURVE':
            if o.data.animation_data is not None:
                if o.data.animation_data.action is not None:
                    if o.data.animation_data.action.fcurves:
                        return True
        return False

    if has_path_animation(obj):
        return True

    parent = obj.parent
    while parent is not None:
        if has_path_animation(parent):
            return True
        parent = parent.parent

    return False



def get_robot_trajectory(obj: bpy.types.Object, 
                         movable_bones: List[str], 
                         rotation_bounds: Dict[str, Tuple[float, float]], 
                         end_frame: int) -> List[List[float]]:
    """
    Generate the trajectory of the robot by recording the Y-axis rotations of specified bones over a range of frames.

    Parameters:
    obj (bpy.types.Object): The Blender object containing the armature.
    movable_bones (List[str]): List of names of bones to check and adjust.
    rotation_bounds (Dict[str, Tuple[float, float]]): Dictionary with bone names as keys and tuples of (lower_bound, upper_bound) as values.
    end_frame (int): The last frame number to record the trajectory.

    Returns:
    List[List[float]]: A list of lists where each inner list contains the Y-axis rotations of the movable bones at a specific frame.
    """
    trajectory: List[List[float]] = []

    # Iterate through frames
    for frame in range(1, end_frame + 1):
        # Set the current frame
        bpy.context.scene.frame_set(frame)
        angle_dict = get_robot_angles(obj, movable_bones, rotation_bounds)
        angles = [angle_dict[bone] for bone in movable_bones]
        trajectory.append(angles)

    return trajectory


def create_robot_obstacle(name,urdf_file_path , end_frame ,movable_joints ,
joint_bounds):
    
    bpy.context.scene.frame_set(0)
    obj = bpy.data.objects[name]
    robot_pos = list(obj.matrix_world.to_translation())
    robot_quat = obj.matrix_world.to_quaternion()
    robot_rotation = [robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w]
    
    
    trajectory = get_robot_trajectory(obj,movable_joints,joint_bounds,end_frame)
    result = {"name": name,
            "type":"dynamic_robot",
            "urdf_file_path": urdf_file_path,
            "robot_joints_order":movable_joints,
            "robot_base_coords": robot_pos,
            "robot_base_quat_rot": robot_rotation,
            "trajectory": trajectory}
    return result

def main(robot_urdf_path,END_FRAME,FPS,directory):
    # GET DATA

    bpy.context.scene.frame_set(0)
    movable_joints, joint_bounds = parse_urdf(robot_urdf_path)
    start_configuration = get_robot_angles(bpy.data.objects["robot_start_pos"],movable_joints, joint_bounds)
    end_configuration = get_robot_angles(bpy.data.objects["robot_goal_pos"],movable_joints, joint_bounds)

    start_configuration = [start_configuration[bone] for bone in movable_joints]
    end_configuration = [end_configuration[bone] for bone in movable_joints]

    obstacles = []
    obs_exp = []
    collection = bpy.data.collections["Obstacles"]

    for obj in list(bpy.data.collections["Obstacles"].all_objects):
        obs_exp.append(obj.name)
    print(obs_exp)
    

    #obstacle_names = ["robot_stand","shelf_0","shelf_1","shelf_2","shelf_3",
    #"shelf_4","bottom","side_x_plus","side_x_minus","side_y_minus","side_y_plus",
    #"top","obstacle left","obstacle right","moving_robot1","moving_robot2"]

    ### ADD BOXES
    # obstacle_names = ['floor','table','table_leg_1','table_leg_2','table_leg_3','table_leg_4']

    obstacle_names = obs_exp
    for obj_name in obstacle_names:
        obj = bpy.data.objects[obj_name]
        if is_object_animated(obj):
            obstacle = {
                "name": obj.name,
                "type":"dynamic_box",
                "dimensions": list(obj.dimensions),
                "positions": get_object_positions(obj, END_FRAME)
            }
        else:
            positions = []
            
            bpy.context.scene.frame_set(0)
            pos = list(obj.matrix_world.to_translation())
            quat = obj.matrix_world.to_quaternion()
            rotation = [quat.x, quat.y, quat.z, quat.w]
            positions.append((*pos, *rotation))
            obstacle = {
                "name": obj.name,
                "type":"static_box",
                "dimensions": list(obj.dimensions),
                "positions": positions
            }

        obstacles.append(obstacle)

    ### ADD ROBOTS AS OBSTACLES

    # obstacles.append(create_robot_obstacle(name = "robot_obstacle",movable_joints = movable_joints,urdf_file_path = robot_urdf_path, end_frame = END_FRAME,
    # joint_bounds=joint_bounds))

    # obstacles.append(create_robot_obstacle(name = "robot_obstacle_2",urdf_file_path = robot_urdf_path, end_frame = END_FRAME,movable_joints = movable_joints,
    # joint_bounds=joint_bounds))
    # GENERATE JSON FILE

    bpy.context.scene.frame_set(0)
    obj = bpy.data.objects["robot_start_pos"]
    robot_pos = list(obj.matrix_world.to_translation())
    robot_quat = obj.matrix_basis.to_quaternion()
    robot_rotation = [robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w]
    print(robot_pos)
    print(robot_rotation)

    blender_file_path = bpy.data.filepath
    json_dict = {
        "start_configuration": start_configuration,
        "end_configuration": end_configuration,
        "robot_joint_max_velocity":[3.1415,3.1415,3.1415,3.1415,3.1415,3.1415], # TODO: remove hardcode
        "robot_capsules_radius": [0.047, 0.12, 0.11, 0.09, 0.05, 0.0380], # TODO: remove hardcode 
        "robot_urdf":"./Blender/robots/xarm6/xarm6vis.urdf",# robot_urdf_path,
        "robot_joints_order": movable_joints,
        "robot_base_coords": robot_pos,
        "robot_base_quat_rot": robot_rotation,  # [x,y,z,w] 
        "frame_count": END_FRAME,
        "fps": FPS,
        "blender_file_path": "./Blender/scenes/flying boxes.blend",#blender_file_path,
        "obstacles": obstacles,
    }


    with open(directory+'\\scene_task.json', 'w') as f:
        f.write(json.dumps(json_dict))

if __name__=='__main__':
    robot_urdf_path = './Blender/robots/xarm6/xarm6vis.urdf'
    END_FRAME=1800
    FPS = 30
    directory = "./Blender/scenes"
    
    main(robot_urdf_path,END_FRAME,FPS,directory)