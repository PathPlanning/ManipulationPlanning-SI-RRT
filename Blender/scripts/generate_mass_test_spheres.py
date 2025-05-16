# & "C:/Program Files/Blender Foundation/Blender 4.1/blender.exe"  --background --python ./generate_mass_test_spheres.py
from multiprocessing import current_process
import os,random,sys,subprocess
import subprocess

try:
    import bpy
    dir = os.path.dirname(bpy.data.filepath)
    if not dir in sys.path:
        sys.path.append(dir )
    dir = "C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/scripts"
    if not dir in sys.path:
        sys.path.append(dir )
    dir = "C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning"
    if not dir in sys.path:
        sys.path.append(dir )
    import export_data_to_json
    import animate_path
except BaseException as e:
    pass

import numpy as np
from concurrent.futures import ProcessPoolExecutor

NUM_CPUS = 8

    


END_TIME = 20
END_FRAME = 20*30
FPS = 30
WORKSPACE_SIZE=2
X_MAX = 1
X_MIN = -1
Y_MAX = 1
Y_MIN = -1
Z_MAX = 2
Z_MIN = 0.5
ROBOT_URDF_PATH = 'C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/robots/xarm6/xarm6cyl.urdf'
ANGLE_TRESHOLD = np.pi
BLENDER_MASS_TEST_FILEPATH ='C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/scripts/mass_test spheres.blend'
SPHERE_COUNT = [x for x in range(0,301,20)]
SPHERE_COUNT[0]+=1

NUMBER_OF_ROBOT_STATES = 1
NUMBER_OF_BATCHES = 50
def random_value(min,max):
    return random.random()*(max-min)+min

def add_sphere():
    sphere_size = random_value(min=0.05,max=0.1)
    sphere_speed = 4
    start_x = random_value(min=-1,max=1)
    start_y = random_value(min=-1,max=1) 
    start_z = random_value(min=0.5,max=2)
    sphere_start_location = np.array((start_x,start_y,start_z))
    sphere_speed = np.array((random_value(min=-sphere_speed,max=sphere_speed),random_value(min=-sphere_speed,max=sphere_speed),random_value(min=-sphere_speed,max=sphere_speed)))
    
    
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=sphere_size, enter_editmode=False, align='WORLD', location=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0))
    # our created sphere is the active one
    obj = bpy.context.active_object
    obj.name = 'sphere_' + obj.name
    # Remove object from all collections not used in a scene
    bpy.ops.collection.objects_remove_all()
    # add it to our specific collection
    bpy.data.collections['Obstacles'].objects.link(obj)
    obj.location = sphere_start_location
    obj.keyframe_insert(data_path='location', frame=0)
    
    sphere_current_location = sphere_start_location
    for frame in range(1,END_FRAME+1):
        if (sphere_current_location[0]+sphere_speed[0]*(1/FPS)>X_MAX )or (sphere_current_location[0]+sphere_speed[0]*(1/FPS)<X_MIN):
            sphere_speed[0] = -sphere_speed[0]
        if (sphere_current_location[1]+sphere_speed[1]*(1/FPS)>Y_MAX )or (sphere_current_location[1]+sphere_speed[1]*(1/FPS)<Y_MIN):
            sphere_speed[1] = -sphere_speed[1]
        if (sphere_current_location[2]+sphere_speed[2]*(1/FPS)>Z_MAX )or (sphere_current_location[2]+sphere_speed[2]*(1/FPS)<Z_MIN):
            sphere_speed[2] = -sphere_speed[2]
        sphere_current_location = sphere_current_location+sphere_speed*(1/FPS)
        obj.location = sphere_current_location
        obj.keyframe_insert(data_path='location', frame=frame)

    
   

def create_robot_random_angles():
    nado = True
    while nado:
        movable_joints, joint_bounds_dict = export_data_to_json.parse_urdf(ROBOT_URDF_PATH)
        joint_bounds = [joint_bounds_dict[joint] for joint in movable_joints]

        start_position = [random_value(j[0],j[1]) for j  in joint_bounds]
        # animate_path.move_robot('robot_start_pos',start_position,movable_joints,frame = 0)
        
        for _ in range(100):
            end_position = [random_value(j[0],j[1]) for j  in joint_bounds]
            if np.linalg.norm(np.array(end_position)-np.array(start_position),ord=1)>ANGLE_TRESHOLD:
                break
        assert(np.linalg.norm(np.array(end_position)-np.array(start_position),ord=1)>ANGLE_TRESHOLD) 
        
        animate_path.move_robot('robot_start_pos',start_position,movable_joints,frame = 0)
        animate_path.move_robot('robot_goal_pos',end_position,movable_joints,frame = 0)
        export_data_to_json.main(robot_urdf_path=ROBOT_URDF_PATH,END_FRAME=END_FRAME,FPS=FPS,directory = f"./") #save as
                
        result = subprocess.run(f'wsl sh -c "  cd /home/panzer/git/Manipulator-Dynamic-Planning && ./STRRT_Planner/build/check_scene /mnt/c/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/scripts/scene_task.json"', shell=True)
        os.remove('./scene_task.json')
        nado = (result.returncode != 0)
    
        print(result.stdout)
        print(result.returncode)
    return (start_position,end_position)
    
def create_single_test(test_suite,i,robot_test_angles,movable_joints):
    
        last_valid_scene = BLENDER_MASS_TEST_FILEPATH
        last_created_spheres = 0

        nado = True
        os.mkdir(f"./{i}_spheres/test_number_{test_suite}")
        
        while nado:
            bpy.ops.wm.open_mainfile(filepath=last_valid_scene)
            created_spheres = last_created_spheres
            valid = True
            for sphere_number in range(i - created_spheres):
                add_sphere()
                created_spheres+=1
                
            
            animate_path.move_robot('robot_start_pos',robot_test_angles[0],movable_joints,frame = 0)
            animate_path.move_robot('robot_goal_pos',robot_test_angles[1],movable_joints,frame = 0)
            bpy.ops.wm.save_as_mainfile(filepath=f"./{i}_spheres/test_number_{test_suite}/mass_test.blend")
            bpy.ops.wm.open_mainfile(filepath=f"./{i}_spheres/test_number_{test_suite}/mass_test.blend")
            export_data_to_json.main(robot_urdf_path=ROBOT_URDF_PATH,END_FRAME=END_FRAME,FPS=FPS,directory = f"./{i}_spheres/test_number_{test_suite}") #save as
            
            result = subprocess.run(f'wsl sh -c "  cd /home/panzer/git/Manipulator-Dynamic-Planning && ./STRRT_Planner/build/check_scene /mnt/c/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/scripts/{i}_spheres/test_number_{test_suite}/scene_task.json"', shell=True)
            if (result.returncode != 0):
                # shutil.rmtree(f"./{i}_spheres")
                valid = False

            print(result.stdout)
            print(result.returncode)
            if valid:   
                last_valid_scene = f"./{i}_spheres/test_number_{test_suite}/mass_test.blend"
                last_created_spheres = created_spheres
                nado = False

def execute_single_test(test_suite,sphere_count,robot_test_angles,movable_joints):
    
    print(f'"C:/Program Files/Blender Foundation/Blender 4.1/blender.exe"  --background --python ./create_single_test_spheres.py {test_suite} "{sphere_count}" "{robot_test_angles}" "{movable_joints}"')
    # os.system(f'"C:/Program Files/Blender Foundation/Blender 4.1/blender.exe"  --background --python ./create_single_test_spheres.py {test_suite} {sphere_count} "{robot_test_angles}" "{movable_joints}"')
    subprocess.run(f'"C:/Program Files/Blender Foundation/Blender 4.1/blender.exe"  --background --python ./create_single_test_spheres.py {test_suite} "{sphere_count}" "{robot_test_angles}" "{movable_joints}"')

def create_test():
    movable_joints, joint_bounds = export_data_to_json.parse_urdf(ROBOT_URDF_PATH)
    bpy.ops.wm.open_mainfile(filepath=BLENDER_MASS_TEST_FILEPATH) # open mass test file
    
    #create test
    angles = {"start_configuration": [
    3.0, 1.5, -2.6,
    0.0, 0.0, 0.0
  ],
  "end_configuration": [
    3.0, -1.8, -2.6,
    0.0, 0.0, 0.0
  ]}
    # robot_test_angles = create_robot_random_angles()
    robot_test_angles = (angles["start_configuration"],angles["end_configuration"])
    # print(robot_test_angles,movable_joints)
    for i in SPHERE_COUNT:
        os.mkdir(f"./{i}_spheres")    
    
    futures = []
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:
        for test_suite in range(NUMBER_OF_BATCHES):
            futures.append(executor.submit(execute_single_test,test_suite,SPHERE_COUNT,robot_test_angles,movable_joints ))
    
    for future in futures:
        future.result()
    
        
    

def main():
    create_test()
    
        
if __name__=="__main__":   
    main()