import bpy
import json
import xml.etree.ElementTree as ET
import math
from typing import List, Tuple, Dict, Optional
import os
import bmesh

    

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

    
def place_mesh_obstacle(obstacle):
    raise NotImplementedError

def create_capsule(radius, length, name):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=64, v_segments=64, radius=radius)
    delta_Z = length/2
    bm.verts.ensure_lookup_table()
    for vert in bm.verts:
        if vert.co[2] < 0:
            vert.co[2] -= delta_Z
        elif vert.co[2] > 0:
            vert.co[2] += delta_Z

    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    mesh.update()
    bm.free()

    object = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(object)

    return object

def set_positions(object, positions):
    for frame,position in enumerate(positions):
        current_location = [position['x'],position['y'],position['z']]
        current_rotation = [position['q_w'],position['q_x'],position['q_y'],position['q_z']]
        object.location = current_location
        object.keyframe_insert(data_path='location', frame=frame)
        object.rotation_mode = 'QUATERNION'
        object.rotation_quaternion = current_rotation  

        object.keyframe_insert(data_path="location", frame=frame)
        object.keyframe_insert(data_path="rotation_quaternion", frame=frame)

def place_capsule_obstacle(obstacle):
    capsule = create_capsule(obstacle['radius'],obstacle['length'],obstacle['name'])
    set_positions(capsule,obstacle['transforms'])
    
    
def place_obstacle(obstacle):
    if obstacle['type'] == 'capsule':
        place_capsule_obstacle(obstacle)
    elif obstacle['type'] == 'mesh':
        place_mesh_obstacle(obstacle)
    
def main(json_path):    
    
    if not os.path.isfile(json_path):
        raise FileNotFoundError
    
    with open(json_path,'r') as f:
        obstacles = json.load(f) 
    
    for obstacle in obstacles:
        place_obstacle(obstacle)

if __name__=="__main__":
    json_path = r""
    main(json_path)