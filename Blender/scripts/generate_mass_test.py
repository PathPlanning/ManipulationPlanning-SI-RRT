# & "C:\Program Files\Blender Foundation\Blender 4.1\blender.exe"  --background --python .\generate_mass_test.py
import bpy, math,os,random,sys,shutil,subprocess
import numpy as np

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

END_TIME = 20
END_FRAME = 20*30
FPS = 30
WORKSPACE_DIMENSION=2
ROBOT_URDF_PATH = 'C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/robots/xarm6/xarm6vis.urdf'
ANGLE_TRESHOLD = np.pi
BLENDER_MASS_TEST_FILEPATH ='C:/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/scenes/mass test 2 50 drgbt RGBTSTtar.blend'
CUBE_COUNT = list(range(50))
NUMBER_OF_ROBOT_STATES = 50
def random_value(min,max):
    return random.random()*(max-min)+min

def add_cube():
    cube_size = random_value(min=0.05,max=0.1)
    
    start_x = random_value(min=-1,max=1)
    start_y = random_value(min=-1,max=1) 
    start_z = random_value(min=-1,max=1)
    cube_start_location = np.array((start_x,start_y,start_z))
    сube_speed = np.array((random_value(min=-0.2,max=0.2),random_value(min=-0.2,max=0.2),random_value(min=-0.2,max=0.2)))
    
    
    
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
    
def create_test():
    movable_joints, joint_bounds = export_data_to_json.parse_urdf(ROBOT_URDF_PATH)
    bpy.ops.wm.open_mainfile(filepath=BLENDER_MASS_TEST_FILEPATH) # open mass test file
    
    #create test
    robot_test_angles = []
    for test_suite in range(NUMBER_OF_ROBOT_STATES):
        robot_test_angles.append(create_robot_random_angles())
    last_valid_scene = BLENDER_MASS_TEST_FILEPATH
    last_created_cubes = 0
    for i in CUBE_COUNT:
        nado = True
        while nado:
            bpy.ops.wm.open_mainfile(filepath=last_valid_scene)
            created_cubes = last_created_cubes
            valid = True
            for cube_number in range(i - created_cubes):
                add_cube()
                created_cubes+=1
            os.mkdir(f"./{i}_cubes")
            for test_suite in range(NUMBER_OF_ROBOT_STATES):
                os.mkdir(f"./{i}_cubes/test_number_{test_suite}")
                
                animate_path.move_robot('robot_start_pos',robot_test_angles[test_suite][0],movable_joints,frame = 0)
                animate_path.move_robot('robot_goal_pos',robot_test_angles[test_suite][1],movable_joints,frame = 0)
                bpy.ops.wm.save_as_mainfile(filepath=f"./{i}_cubes/test_number_{test_suite}/mass_test.blend")
                bpy.ops.wm.open_mainfile(filepath=f"./{i}_cubes/test_number_{test_suite}/mass_test.blend")
                export_data_to_json.main(robot_urdf_path=ROBOT_URDF_PATH,END_FRAME=END_FRAME,FPS=FPS,directory = f"./{i}_cubes/test_number_{test_suite}") #save as
                
                result = subprocess.run(f'wsl sh -c "  cd /home/panzer/git/Manipulator-Dynamic-Planning && ./STRRT_Planner/build/check_scene /mnt/c/Users/kerim/Desktop/git/Manipulator-Dynamic-Planning/Blender/scripts/{i}_cubes/test_number_{test_suite}/scene_task.json"', shell=True)
                if (result.returncode != 0):
                    shutil.rmtree(f"./{i}_cubes")
                    valid = False
                    break

                print(result.stdout)
                print(result.returncode)
            if valid:   
                last_valid_scene = f"./{i}_cubes/test_number_0/mass_test.blend"
                last_created_cubes = created_cubes
                nado = False


def main():
    random.seed(42)
    create_test()
    
        
if __name__=="__main__":   
    main()