import functools
import json
import os
import sys
from typing import Optional, Sequence
import math
import bpy

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)

def arrange_nodes(node_tree: bpy.types.NodeTree, verbose: bool = False) -> None:
    max_num_iters = 2000
    epsilon = 1e-05
    target_space = 50.0

    second_stage = False

    fix_horizontal_location = True
    fix_vertical_location = True
    fix_overlaps = True

    if verbose:
        print("-----------------")
        print("Target nodes:")
        for node in node_tree.nodes:
            print("- " + node.name)

    # In the first stage, expand nodes overly
    target_space *= 2.0

    # Gauss-Seidel-style iterations
    previous_squared_deltas_sum = sys.float_info.max
    for i in range(max_num_iters):
        squared_deltas_sum = 0.0

        if fix_horizontal_location:
            for link in node_tree.links:
                k = 0.9 if not second_stage else 0.5
                threshold_factor = 2.0

                x_from = link.from_node.location[0]
                x_to = link.to_node.location[0]
                w_from = link.from_node.width
                signed_space = x_to - x_from - w_from
                C = signed_space - target_space
                grad_C_x_from = -1.0
                grad_C_x_to = 1.0

                # Skip if the distance is sufficiently large
                if C >= target_space * threshold_factor:
                    continue

                lagrange = C / (grad_C_x_from * grad_C_x_from + grad_C_x_to * grad_C_x_to)
                delta_x_from = -lagrange * grad_C_x_from
                delta_x_to = -lagrange * grad_C_x_to

                link.from_node.location[0] += k * delta_x_from
                link.to_node.location[0] += k * delta_x_to

                squared_deltas_sum += k * k * (delta_x_from * delta_x_from + delta_x_to * delta_x_to)

        if fix_vertical_location:
            k = 0.5 if not second_stage else 0.05
            socket_offset = 20.0

            def get_from_socket_index(node: bpy.types.Node, node_socket: bpy.types.NodeSocket) -> int:
                for i in range(len(node.outputs)):
                    if node.outputs[i] == node_socket:
                        return i
                assert False

            def get_to_socket_index(node: bpy.types.Node, node_socket: bpy.types.NodeSocket) -> int:
                for i in range(len(node.inputs)):
                    if node.inputs[i] == node_socket:
                        return i
                assert False

            for link in node_tree.links:
                from_socket_index = get_from_socket_index(link.from_node, link.from_socket)
                to_socket_index = get_to_socket_index(link.to_node, link.to_socket)
                y_from = link.from_node.location[1] - socket_offset * from_socket_index
                y_to = link.to_node.location[1] - socket_offset * to_socket_index
                C = y_from - y_to
                grad_C_y_from = 1.0
                grad_C_y_to = -1.0
                lagrange = C / (grad_C_y_from * grad_C_y_from + grad_C_y_to * grad_C_y_to)
                delta_y_from = -lagrange * grad_C_y_from
                delta_y_to = -lagrange * grad_C_y_to

                link.from_node.location[1] += k * delta_y_from
                link.to_node.location[1] += k * delta_y_to

                squared_deltas_sum += k * k * (delta_y_from * delta_y_from + delta_y_to * delta_y_to)

        if fix_overlaps and second_stage:
            k = 0.9
            margin = 0.5 * target_space

            # Examine all node pairs
            for node_1 in node_tree.nodes:
                for node_2 in node_tree.nodes:
                    if node_1 == node_2:
                        continue

                    x_1 = node_1.location[0]
                    x_2 = node_2.location[0]
                    w_1 = node_1.width
                    w_2 = node_2.width
                    cx_1 = x_1 + 0.5 * w_1
                    cx_2 = x_2 + 0.5 * w_2
                    rx_1 = 0.5 * w_1 + margin
                    rx_2 = 0.5 * w_2 + margin

                    # Note: "dimensions" and "height" may not be correct depending on the situation
                    def get_height(node: bpy.types.Node) -> float:
                        if node.dimensions.y > epsilon:
                            return node.dimensions.y
                        elif math.fabs(node.height - 100.0) > epsilon:
                            return node.height
                        else:
                            return 200.0

                    y_1 = node_1.location[1]
                    y_2 = node_2.location[1]
                    h_1 = get_height(node_1)
                    h_2 = get_height(node_2)
                    cy_1 = y_1 - 0.5 * h_1
                    cy_2 = y_2 - 0.5 * h_2
                    ry_1 = 0.5 * h_1 + margin
                    ry_2 = 0.5 * h_2 + margin

                    C_x = math.fabs(cx_1 - cx_2) - (rx_1 + rx_2)
                    C_y = math.fabs(cy_1 - cy_2) - (ry_1 + ry_2)

                    # If no collision, just skip
                    if C_x >= 0.0 or C_y >= 0.0:
                        continue

                    # Solve collision for the "easier" direction
                    if C_x > C_y:
                        grad_C_x_1 = 1.0 if cx_1 - cx_2 >= 0.0 else -1.0
                        grad_C_x_2 = -1.0 if cx_1 - cx_2 >= 0.0 else 1.0
                        lagrange = C_x / (grad_C_x_1 * grad_C_x_1 + grad_C_x_2 * grad_C_x_2)
                        delta_x_1 = -lagrange * grad_C_x_1
                        delta_x_2 = -lagrange * grad_C_x_2

                        node_1.location[0] += k * delta_x_1
                        node_2.location[0] += k * delta_x_2

                        squared_deltas_sum += k * k * (delta_x_1 * delta_x_1 + delta_x_2 * delta_x_2)
                    else:
                        grad_C_y_1 = 1.0 if cy_1 - cy_2 >= 0.0 else -1.0
                        grad_C_y_2 = -1.0 if cy_1 - cy_2 >= 0.0 else 1.0
                        lagrange = C_y / (grad_C_y_1 * grad_C_y_1 + grad_C_y_2 * grad_C_y_2)
                        delta_y_1 = -lagrange * grad_C_y_1
                        delta_y_2 = -lagrange * grad_C_y_2

                        node_1.location[1] += k * delta_y_1
                        node_2.location[1] += k * delta_y_2

                        squared_deltas_sum += k * k * (delta_y_1 * delta_y_1 + delta_y_2 * delta_y_2)

        if verbose:
            print("Iteration #" + str(i) + ": " + str(previous_squared_deltas_sum - squared_deltas_sum))

        # Check the termination conditiion
        if math.fabs(previous_squared_deltas_sum - squared_deltas_sum) < epsilon:
            if second_stage:
                break
            else:
                target_space = 0.5 * target_space
                second_stage = True

        previous_squared_deltas_sum = squared_deltas_sum

def set_output_properties(
    scene: bpy.types.Scene,
    resolution_percentage: int = 100,
    output_file_path: str = "",
    res_x: int = 1920,
    res_y: int = 1080,
) -> None:
    scene.render.resolution_percentage = resolution_percentage
    scene.render.resolution_x = res_x
    scene.render.resolution_y = res_y

    if output_file_path:
        scene.render.filepath = output_file_path


def set_cycles_renderer(
    scene: bpy.types.Scene,
    camera_object: bpy.types.Object,
    num_samples: int,
    use_denoising: bool = True,
    use_motion_blur: bool = False,
    use_transparent_bg: bool = False,
    prefer_cuda_use: bool = True,
    use_adaptive_sampling: bool = False,
) -> None:
    scene.camera = camera_object

    # scene.render.image_settings.file_format = "PNG"
    scene.render.engine = "CYCLES"
    scene.render.use_motion_blur = use_motion_blur

    scene.render.film_transparent = use_transparent_bg
    scene.view_layers[0].cycles.use_denoising = use_denoising

    scene.cycles.use_adaptive_sampling = use_adaptive_sampling
    scene.cycles.samples = num_samples

    # Enable GPU acceleration
    # Source - https://blender.stackexchange.com/a/196702
 
    # Call get_devices() to let Blender detects GPU device (if any)
    bpy.context.preferences.addons["cycles"].preferences.get_devices()
    
    if prefer_cuda_use:
        bpy.context.scene.cycles.device = "GPU"

        # Change the preference setting
        bpy.context.preferences.addons["cycles"].preferences.compute_device_type = (
            "CUDA"
        )

    # Let Blender use all available devices, include GPU and CPU
    f = True
    for d in bpy.context.preferences.addons["cycles"].preferences.devices:
        if f:
            d["use"] = 1 
            f = False
        else:
            d["use"] = 0

    # Display the devices to be used for rendering
    print("----")
    bpy.context.preferences.addons["cycles"].preferences.get_devices()
    for device in bpy.context.preferences.addons["cycles"].preferences.devices:
        print(f"\t{device.name} ({device.type}) {device.use}")
    print("----")

    bpy.context.scene.render.engine = 'CYCLES'

    # Set all light path bounces to 4
    cycles = bpy.context.scene.cycles
    cycles.max_bounce = 3
    cycles.diffuse_bounces = 3
    cycles.glossy_bounces = 3
    cycles.transmission_bounces = 3
    cycles.volume_bounces = 0
    cycles.transparent_max_bounces = 3

def get_path_from_file(path_file: str) -> list[tuple[float, list]]:
    """Get path from file

    Arguments:
        path_file: path to file, containing manipulator path (from raw DRGBT log)

    Returns:
        list[tuple[float,list]]: list of states with time and list of angles
    """
    path_time = []

    with open(path_file, "r") as f:
        for line in f:
            path_str, time_str = line.split(";")
            time = float(time_str)
            q = [float(x) for x in path_str[1:-1].split() if x]
            path_time.append((time, q))
    return path_time

def build_environment_texture_background(world: bpy.types.World, hdri_path: str, rotation: float = 0.0) -> None:
    world.use_nodes = True
    node_tree = world.node_tree

    environment_texture_node = node_tree.nodes.new(type="ShaderNodeTexEnvironment")
    environment_texture_node.image = bpy.data.images.load(hdri_path)

    mapping_node = node_tree.nodes.new(type="ShaderNodeMapping")
    if bpy.app.version >= (2, 81, 0):
        mapping_node.inputs["Rotation"].default_value = (0.0, 0.0, rotation)
    else:
        mapping_node.rotation[2] = rotation

    tex_coord_node = node_tree.nodes.new(type="ShaderNodeTexCoord")

    node_tree.links.new(tex_coord_node.outputs["Generated"], mapping_node.inputs["Vector"])
    node_tree.links.new(mapping_node.outputs["Vector"], environment_texture_node.inputs["Vector"])
    # print(node_tree.nodes.keys())
    node_tree.links.new(environment_texture_node.outputs["Color"], node_tree.nodes["World Output"].inputs["Surface"])

    arrange_nodes(node_tree)

def move_robot(
    robot_name: str,
    angles: Sequence[float],
    joint_order: tuple[str],
    frame: Optional[int] = None,
):
    """
    Moves robot joints

    Args:
        robot_name: str name of robot in blender
        angles: list of 6 angles in radians
        joint_order: tuple of joint names in order
        frame: frame, to which it will insert keyframe. if None, no keyframe will be created

    """
    assert len(angles) == len(joint_order)
    bpy.context.view_layer.objects.active = bpy.data.objects[robot_name]
    bones = [
        bpy.context.view_layer.objects.active.pose.bones[bone_name]
        for bone_name in joint_order
    ]

    for joint, angle in zip(bones, angles):
        joint.rotation_euler = (0.0, angle, 0.0)

    if frame is not None:
        #        bpy.context.scene.frame_set(frame=frame)
        #        bpy.ops.anim.keyframe_insert(type='Rotation')
        for joint in bones:
            #            bpy.ops.anim.keyframe_insert(type='DEFAULT')
            joint.keyframe_insert("rotation_euler", frame=frame)



def open_json(json_path: str) -> dict:
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

    with open(json_path, "r") as f:
        print(json_path)
        data = json.load(f)
    return data


def animate_path(manipulator_path, fps, joint_order):
    for state in manipulator_path:
        move_robot(
            "robot",
            state["robot_angles"],
            frame=int(state["time"] * fps),
            joint_order=joint_order,
        )


def visualise_one_task(
    path_to_log_file: str, path_to_scene_task: str, path_to_blender_scene: str
):
    """Visualise one task (render video)

    Args:
        path_to_log_file (str): path to planner output file
        path_to_scene_task (str): path to scene task description
        path_to_blender_scene (str): path to blender scene
    """
    # open file
    bpy.ops.wm.open_mainfile(filepath=path_to_blender_scene)

    # parse json file
    print(path_to_log_file)
    planner_logs = open_json(path_to_log_file)

    scene_task = open_json(path_to_scene_task)

    # animate robot
    joint_order = scene_task["robot_joints_order"]
    move_robot("robot_start_pos", scene_task["start_configuration"], joint_order, 0)
    move_robot("robot_goal_pos", scene_task["end_configuration"], joint_order, 0)

    manipulator_path = planner_logs["final_planner_data"][
        "final_path"
    ]  # get manipulator path

    animation_callback = functools.partial(
        animate_path, manipulator_path, scene_task["fps"], joint_order
    )
    animation_callback()

    # render video
    # thanks to https://github.com/yuki-koyama/blender-cli-rendering/blob/master/03_ibl.py

    scene = bpy.data.scenes["Scene"]
    world = scene.world
    build_environment_texture_background(world=world,hdri_path='/home/panzer/git/ManipulationPlanning-SI-RRT/Blender/scripts/open_box_background.jpg')
    output_file_path = os.path.dirname(path_to_log_file) + "/visualisation.mp4"

    # ## Camera
    # bpy.ops.object.camera_add(location=(5.0, -10.0, 2.0))
    # camera_object = bpy.context.object
    camera_object = bpy.data.objects["Camera"]
    # utils.add_track_to_constraint(camera_object, center_suzanne)
    # utils.set_camera_params(camera_object.data, center_suzanne, lens=50, fstop=0.2)

    # ## Lights
    # utils.build_environment_texture_background(world, hdri_path)

    # Render Setting
    resolution_percentage = 10
    num_samples = 1
    set_output_properties(scene, resolution_percentage, output_file_path)
    set_cycles_renderer(scene, camera_object, num_samples)


def main():
    visualise_one_task(
        path_to_log_file="/home/panzer/git/ManipulationPlanning-SI-RRT/Blender/scripts/MSIRRT_planner_logs_1729819221.json",
        path_to_blender_scene="/home/panzer/git/ManipulationPlanning-SI-RRT/Blender/scripts/mass_test.blend",
        path_to_scene_task="/home/panzer/git/ManipulationPlanning-SI-RRT/Blender/scripts/scene_task.json",
    )


if __name__ == "__main__":
    main()
