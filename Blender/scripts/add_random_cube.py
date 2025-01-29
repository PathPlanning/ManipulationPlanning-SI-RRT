import bpy
import mathutils
import math,os
from typing import Mapping, MutableMapping, Sequence, Optional
import json
import time
import functools
import random
import numpy as np

END_TIME = 20
END_FRAME = 20*30
FPS = END_FRAME/END_TIME
WORKSPACE_DIMENSION=3

def random_value(min,max):
    return random.random()*(max-min)+min

def add_cube():
    cube_size = random_value(min=0.05,max=0.1)
    
    start_x = random_value(min=-1,max=1)
    start_y = random_value(min=-1,max=1) 
    start_z = random_value(min=-1,max=1)
    cube_start_location = np.array((start_x,start_y,start_z))
    сube_speed = np.array((random_value(min=-1,max=1),random_value(min=-1,max=1),random_value(min=-1,max=1)))
    
    
    
    bpy.ops.mesh.primitive_cube_add(size = cube_size, location = [1,1,1], rotation=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0))
    # our created cube is the active one
    obj = bpy.context.active_object
    # Remove object from all collections not used in a scene
    bpy.ops.collection.objects_remove_all()
    # add it to our specific collection
    bpy.data.collections['Obstacles'].objects.link(obj)
    obj.location = cube_start_location
    obj.keyframe_insert(data_path='location', frame=0)
    
    cube_current_location = cube_start_location
    for frame in range(1,END_FRAME+1):
        
        
        сube_speed[np.abs(cube_current_location+сube_speed*(1/FPS)) > WORKSPACE_DIMENSION/2] = сube_speed[np.abs(cube_current_location+сube_speed*(1/FPS)) > WORKSPACE_DIMENSION/2] * -1
            
        cube_current_location = cube_current_location+сube_speed*(1/FPS)
        obj.location = cube_current_location
        obj.keyframe_insert(data_path='location', frame=frame)

    
   


def main():
    random.seed(42)
    for i in range(50):
        add_cube()
    
        
if __name__=="__main__":
    main()