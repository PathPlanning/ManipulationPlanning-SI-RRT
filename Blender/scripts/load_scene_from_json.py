import bpy
import json
import mathutils
import math
import os
from typing import Mapping, MutableMapping, Sequence, Optional
import functools

def open_json(json_path: str) -> dict:
    """Open json file

    Args:
        json_path (str): path to json file

    Raises:
        FileNotFoundError: if file not found

    Returns:
        dict: json struct
    """
    if not os.path.isfile(json_path):
        raise FileNotFoundError
    
    with open(json_path, 'r') as f:
        print(f"Loading scene from: {json_path}")
        data = json.load(f)
    return data

def clear_scene():
    """Clear all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

def create_box(name: str, dimensions: list, position: list, rotation: list):
    """Create a box object"""
    bpy.ops.mesh.primitive_cube_add(size=1)
    box = bpy.context.active_object
    box.name = name
    
    # Set dimensions
    box.scale = mathutils.Vector(dimensions)
    
    # Set position and rotation
    box.location = mathutils.Vector(position)
    box.rotation_quaternion = mathutils.Quaternion(rotation)
    
    # Apply scale to mesh
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    
    return box

def create_sphere(name: str, radius: float, position: list, rotation: list):
    """Create a sphere object"""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius)
    sphere = bpy.context.active_object
    sphere.name = name
    
    # Set position and rotation
    sphere.location = mathutils.Vector(position)
    sphere.rotation_quaternion = mathutils.Quaternion(rotation)
    
    return sphere

def create_robot_from_urdf(urdf_path: str, name: str, base_position: list, base_rotation: list, joint_config: list):
    """Create a robot object from URDF file"""
    try:
        # Import URDF
        bpy.ops.import_scene.urdf(filepath=urdf_path)
        
        # Get the imported robot (usually the armature)
        robot = None
        for obj in bpy.context.selected_objects:
            if obj.type == 'ARMATURE':
                robot = obj
                break
        
        if not robot:
            # If no armature found, use the first selected object
            robot = bpy.context.selected_objects[0]
        
        robot.name = name
        
        # Set base position and rotation
        robot.location = mathutils.Vector(base_position)
        robot.rotation_quaternion = mathutils.Quaternion(base_rotation)
        
        # Set joint angles if provided
        if joint_config and hasattr(robot, 'pose') and robot.pose:
            print(f"Note: Joint configuration {joint_config} would need to be applied to robot joints")
        
        return robot
        
    except Exception as e:
        print(f"Warning: Could not import URDF {urdf_path}: {e}")
        # Create a placeholder cube for the robot
        bpy.ops.mesh.primitive_cube_add(size=0.5)
        robot = bpy.context.active_object
        robot.name = name
        robot.location = mathutils.Vector(base_position)
        robot.rotation_quaternion = mathutils.Quaternion(base_rotation)
        return robot

def move_robot(robot_name: str, angles: Sequence[float], joint_order: tuple, frame: Optional[int] = None):
    """
    Moves robot joints

    Args:
        robot_name: str name of robot in blender
        angles: list of angles in radians
        joint_order: tuple of joint names in order
        frame: frame, to which it will insert keyframe. if None, no keyframe will be created
    """
    if robot_name not in bpy.data.objects:
        print(f"Warning: Robot {robot_name} not found in scene")
        return
    
    robot = bpy.data.objects[robot_name]
    if not hasattr(robot, 'pose') or not robot.pose:
        print(f"Warning: Robot {robot_name} has no pose data")
        return
    
    assert len(angles) == len(joint_order)
    
    bones = []
    for bone_name in joint_order:
        if bone_name in robot.pose.bones:
            bones.append(robot.pose.bones[bone_name])
        else:
            print(f"Warning: Bone {bone_name} not found in robot {robot_name}")
    
    for joint, angle in zip(bones, angles):
        joint.rotation_euler = (0.0, angle, 0.0)
        
    if frame is not None:
        for joint in bones:
            joint.keyframe_insert('rotation_euler', frame=frame)

def create_animation_for_object(obj, positions_data: list, frame_count: int):
    """Create animation for an object based on position data"""
    if not positions_data:
        return
    
    # Set animation data
    obj.animation_data_create()
    obj.animation_data.action = bpy.data.actions.new(name=f"{obj.name}_action")
    
    # Create F-curves for location and rotation
    loc_fcurves = []
    rot_fcurves = []
    
    for i in range(3):  # x, y, z
        loc_fcurves.append(obj.animation_data.action.fcurves.new(data_path="location", index=i))
    
    for i in range(4):  # w, x, y, z (quaternion)
        rot_fcurves.append(obj.animation_data.action.fcurves.new(data_path="rotation_quaternion", index=i))
    
    # Add keyframes
    for frame_data in positions_data:
        frame = frame_data.get('frame', 0)
        position = frame_data.get('position', [0, 0, 0])
        rotation = frame_data.get('rotation', [1, 0, 0, 0])  # w, x, y, z
        
        # Location keyframes
        for i in range(3):
            loc_fcurves[i].keyframe_points.insert(frame, position[i])
        
        # Rotation keyframes
        for i in range(4):
            rot_fcurves[i].keyframe_points.insert(frame, rotation[i])
    
    # Set scene frame range
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = frame_count - 1

def animate_trajectory(robot_name: str, trajectory: list, joint_order: list, fps: int):
    """Animate robot trajectory"""
    for traj_point in trajectory:
        time = traj_point.get('time', 0)
        configuration = traj_point.get('configuration', [])
        frame = int(time * fps)
        
        if len(configuration) == len(joint_order):
            move_robot(robot_name, configuration, joint_order, frame)

def setup_scene_view():
    """Set up camera and lighting for better visualization"""
    
    # Set up camera
    bpy.ops.object.camera_add(location=(5, -5, 3))
    camera = bpy.context.active_object
    camera.rotation_euler = (math.radians(60), 0, math.radians(45))
    bpy.context.scene.camera = camera
    
    # Add lighting
    bpy.ops.object.light_add(type='SUN', location=(5, 5, 10))
    sun = bpy.context.active_object
    sun.data.energy = 5.0
    
    # Add ambient light
    bpy.ops.object.light_add(type='AREA', location=(0, 0, 5))
    area_light = bpy.context.active_object
    area_light.data.energy = 2.0
    area_light.data.size = 10.0

def load_scene_from_json(json_file_path: str):
    """Load scene from JSON file and create Blender objects"""
    
    # Clear existing scene
    clear_scene()
    
    # Load JSON data
    scene_data = open_json(json_file_path)
    
    # Get scene info
    scene_info = scene_data.get('scene_info', {})
    frame_count = scene_info.get('frame_count', 100)
    fps = scene_info.get('fps', 30)
    
    # Set scene FPS
    bpy.context.scene.render.fps = fps
    
    # Create main robot
    main_robot_data = scene_data.get('main_robot', {})
    if main_robot_data:
        urdf_path = main_robot_data.get('urdf_path', '')
        start_config = main_robot_data.get('start_configuration', [])
        base_pos_data = main_robot_data.get('base_position', [0, 0, 0, 0, 0, 0, 1])
        
        # Extract position and rotation from base_position array
        if len(base_pos_data) >= 7:
            base_position = base_pos_data[:3]  # x, y, z
            base_rotation = base_pos_data[3:7]  # quat_x, quat_y, quat_z, quat_w
        else:
            base_position = [0, 0, 0]
            base_rotation = [0, 0, 0, 1]
        
        # Create robot
        robot = create_robot_from_urdf(urdf_path, "main_robot", base_position, base_rotation, start_config)
        
        # Set material for main robot
        if robot.data.materials:
            robot.data.materials[0].diffuse_color = (0.2, 0.6, 1.0, 1.0)  # Blue
        else:
            mat = bpy.data.materials.new(name="RobotMaterial")
            mat.diffuse_color = (0.2, 0.6, 1.0, 1.0)
            robot.data.materials.append(mat)
    
    # Create obstacles
    obstacles = scene_data.get('obstacles', [])
    for i, obstacle_data in enumerate(obstacles):
        name = obstacle_data.get('name', f'obstacle_{i}')
        obstacle_type = obstacle_data.get('type', '')
        is_static = obstacle_data.get('is_static', True)
        shape = obstacle_data.get('shape', '')
        positions = obstacle_data.get('positions', [])
        
        # Get first position for initial placement
        if positions:
            first_pos = positions[0]
            position = first_pos.get('position', [0, 0, 0])
            rotation = first_pos.get('rotation', [1, 0, 0, 0])
        else:
            position = [0, 0, 0]
            rotation = [1, 0, 0, 0]
        
        # Create object based on shape
        if shape == 'box':
            dimensions = obstacle_data.get('dimensions', [1, 1, 1])
            obj = create_box(name, dimensions, position, rotation)
        elif shape == 'sphere':
            radius = obstacle_data.get('radius', 0.5)
            obj = create_sphere(name, radius, position, rotation)
        else:
            # Default to box
            obj = create_box(name, [1, 1, 1], position, rotation)
        
        # Set material based on type
        mat = bpy.data.materials.new(name=f"{name}_Material")
        if 'static' in obstacle_type.lower():
            mat.diffuse_color = (0.8, 0.2, 0.2, 1.0)  # Red for static
        else:
            mat.diffuse_color = (0.8, 0.8, 0.2, 1.0)  # Yellow for dynamic
        
        if obj.data.materials:
            obj.data.materials[0] = mat
        else:
            obj.data.materials.append(mat)
        
        # Create animation if not static and has multiple positions
        if not is_static and len(positions) > 1:
            create_animation_for_object(obj, positions, frame_count)
    
    # Create robot obstacles
    robot_obstacles = scene_data.get('robot_obstacles', [])
    for i, robot_obstacle_data in enumerate(robot_obstacles):
        name = robot_obstacle_data.get('name', f'robot_obstacle_{i}')
        urdf_path = robot_obstacle_data.get('urdf_path', '')
        base_position = robot_obstacle_data.get('base_position', [0, 0, 0])
        base_rotation = robot_obstacle_data.get('base_rotation', [0, 0, 0, 1])
        trajectory = robot_obstacle_data.get('trajectory', [])
        
        # Create robot obstacle
        robot_obstacle = create_robot_from_urdf(urdf_path, name, base_position, base_rotation, [])
        
        # Set material for robot obstacles
        if robot_obstacle.data.materials:
            robot_obstacle.data.materials[0].diffuse_color = (0.2, 0.8, 0.2, 1.0)  # Green
        else:
            mat = bpy.data.materials.new(name=f"{name}_Material")
            mat.diffuse_color = (0.2, 0.8, 0.2, 1.0)
            robot_obstacle.data.materials.append(mat)
        
        # Create animation from trajectory if available
        if trajectory:
            # For now, we'll just set the initial pose
            # In a full implementation, you would calculate the actual robot pose from joint configuration
            print(f"Note: Trajectory animation for {name} would need to be implemented")
    
    # Set up camera and lighting
    setup_scene_view()
    
    print(f"Scene loaded successfully from {json_file_path}")
    print(f"Frame count: {frame_count}, FPS: {fps}")

def main(json_file_path: str = ""):
    """Main function to run the script"""
    
    # If no path is specified, try to find the JSON file in the current directory
    if not json_file_path:
        # Look for JSON files in the current directory
        current_dir = os.path.dirname(bpy.data.filepath) if bpy.data.filepath else os.getcwd()
        json_files = [f for f in os.listdir(current_dir) if f.endswith('.json')]
        
        if json_files:
            json_file_path = os.path.join(current_dir, json_files[0])
            print(f"Found JSON file: {json_file_path}")
        else:
            print("No JSON file found. Please specify the path to your exported scene JSON file.")
            return
    
    # Check if file exists
    if not os.path.exists(json_file_path):
        print(f"Error: File {json_file_path} does not exist.")
        return
    
    # Load the scene
    load_scene_from_json(json_file_path)

if __name__ == "__main__":
    # You can change this path to your exported JSON file
    json_file_path = "/path/to/your/exported_scene.json"
    main(json_file_path) 