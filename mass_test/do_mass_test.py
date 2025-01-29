"""
Process multiagent blender scene and plan path for manipulator with STRRT* and DRGBT

python3 ./mass_test/do_mass_test.py

"""

import argparse
import os
import random
from concurrent.futures import ProcessPoolExecutor
NUM_CPUS = 32

def parse_my_args(argv=None):
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument("--path_to_strrt_config_json",
                        type=str,
                        default='./STRRT/strrt_config.json',
                        help="path to .json, where strrt hyperparametrs is described")
    # parse the arguments
    return PARSER.parse_args(argv)


def plan_strrt(test_dir,strrt_config,scene_task,random_seed):
    
    print(f"STRRT_Planner/build/STRRT_Planner {test_dir}/{scene_task} {test_dir} {strrt_config} {random_seed}")
    os.system(f"STRRT_Planner/build/STRRT_Planner {test_dir}/{scene_task} {test_dir} {strrt_config} {random_seed}")

def plan_msirrt(test_dir,strrt_config,scene_task,random_seed):
    
    print(f"MSIRRT/build/MSIRRT_Planner {test_dir}/{scene_task} {test_dir} {strrt_config} {random_seed}")
    os.system(f"MSIRRT/build/MSIRRT_Planner {test_dir}/{scene_task} {test_dir} {strrt_config} {random_seed}")

def plan_drgbt(test_dir,drgbt_config,scene_task,random_seed):   
    print(f"./RPMPLv2/build/src/main {test_dir}/{scene_task} {test_dir} {drgbt_config} {random_seed}")
    os.system(f"./RPMPLv2/build/src/main {test_dir}/{scene_task} {test_dir} {drgbt_config} {random_seed}")

def main( path_to_strrt_config_json: str) -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt

    Args:
        path_to_scene_json (str): path to .json, where scene is described
        path_to_result_folder (str): path to folder, where results is stored
        path_to_strrt_config_json (str): path to .json, where strrt hyperparametrs is described
    """

    test_dirs = [f"./mass_test/tests/{i}_spheres" for i in range(0,301,20)]
    test_dirs[0] ="./mass_test/tests/1_spheres"
    
    random.seed()
    futures = []
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:
        for test_dir in test_dirs:
            test_numbers = os.listdir(test_dir)
            for test_number_dir in test_numbers:
                for i in range(10):
                #     print("----------")
                    futures.append(executor.submit(plan_strrt, os.path.join(test_dir,test_number_dir),path_to_strrt_config_json,'scene_task.json',random.getrandbits(32)))
                    # futures.append(executor.submit(plan_drgbt, os.path.join(test_dir,test_number_dir),path_to_strrt_config_json,'scene_task.json',random.getrandbits(32)))
                    futures.append(executor.submit(plan_msirrt, os.path.join(test_dir,test_number_dir),path_to_strrt_config_json,'scene_task.json',random.getrandbits(32)))
    
    for future in futures:
        future.result()
        
if __name__ == '__main__':
    args = parse_my_args()
    main(args.path_to_strrt_config_json)
