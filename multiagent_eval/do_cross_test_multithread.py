"""
Process multiagent mass test

python3 ./multiagent_eval/do_mass_test_multithread.py --path_to_tasks ./multiagent_tests --path_to_strrt_config_json ./STRRT/strrt_config.json 

"""
import random
import argparse
import os
import json
import copy
import time
from scipy import interpolate
import numpy as np
from typing import List
from concurrent.futures import ProcessPoolExecutor
from tqdm import tqdm
import subprocess
NUM_CPUS = 128
MAX_ATTEMPTS=5
MAX_FILE_FIND_ATTEMPS = 10
SAVE_INTERMEDIATE_RESULT = False
NUMBER_OF_SEED_ITERATIONS = 10

def process_one_task(test_path:str ,path_to_strrt_config_json:str,test_planner_prefix:str, planner_bin_path:str,planner_prefix:str,planner_logs_prefix:str) -> None:
    """processes one multiagent task
    Args:
        test_dir_path (str): path to multiagent task
        
    """
    tests = [os.path.join(test_path,f) for f in os.listdir(test_path) if test_planner_prefix in f]
    
    for test in tests:
        with open(test, 'r') as file:
            data = file.read()
        scene_parsed_data = json.loads(data)
        start_time = scene_parsed_data["start_time"] 
        name = scene_parsed_data["name"]
        
        test_dir_path = test_path
        
        seed = int(os.path.basename(test_dir_path).split('_')[-1])
        need_to_go = True
        attempt=1
        strrtlogs = {}
        strrtlogs["final_planner_data"] = {}
        strrtlogs["final_planner_data"]["has_result"] = False
        while need_to_go:
            command = f"{planner_bin_path} {test} {test_dir_path} {path_to_strrt_config_json} {seed}"
            
            # print(command)
            result = subprocess.run(command, shell=True)
            if result.returncode != 0:
                print("error occurred!")
                print(command)
                attempt+=1
                if attempt>MAX_ATTEMPTS:
                    break
                continue
            
            file_finding_attempt = 0
            while True:
                file_finding_attempt+=1
                if file_finding_attempt>MAX_FILE_FIND_ATTEMPS:
                    attempt+=1
                    break
                result_filename = list(filter(lambda x: x.startswith(planner_logs_prefix), os.listdir(test_dir_path)))
                if len(result_filename) == 0:
                    assert(file_finding_attempt<=MAX_FILE_FIND_ATTEMPS)
                    print("didn't found file, waiting...",command)
                    time.sleep(1)
                    continue
                try:
                    result_filename = result_filename[0]
                    new_result_filename = f"start_time_{start_time-0.02}_" + result_filename[:-5] + f'_for_{name}.json'
                    new_result_filepath = os.path.join(test_dir_path,new_result_filename) 
                    
                    os.rename(os.path.join(test_dir_path,result_filename), new_result_filepath)
                    with open(new_result_filepath, 'r') as file:
                        strrtlogs_raw = file.read()
                    
                    #check if planning was successful
                    strrtlogs = json.loads(strrtlogs_raw)
                    break
                except json.JSONDecodeError as e:
                    print(f"json decode error: {e}")
                    assert(file_finding_attempt<=MAX_FILE_FIND_ATTEMPS)
                    time.sleep(1)
                    continue
                except UnicodeDecodeError as e:
                    print(f"UnicodeDecodeError error: {e}, {command}")
                    assert(file_finding_attempt<=MAX_FILE_FIND_ATTEMPS)
                    time.sleep(1)
                    continue
                
            if file_finding_attempt>MAX_FILE_FIND_ATTEMPS:
                continue
            
            break
        if "path_to_scene_json" not in strrtlogs:
            print(f"path_to_scene_json is not in strrtlogs  {test}")
            continue
        if not strrtlogs["path_to_scene_json"] == test:
            print(f"path_to_scene_json is not equal to test_path: {strrtlogs['path_to_scene_json']} != {test}")
            continue
        assert(strrtlogs["path_to_scene_json"] == test)
    print(f"processed {test_path}")

 
def main(path_to_tasks: str, path_to_strrt_config_json: str) -> None:
    """Plan path for given multiagent blender scene with stRRT and drgbt

    Args:
        path_to_tasks (str): path to multiagent tasks
        path_to_strrt_config_json (str): path to .json, where strrt hyperparametrs is described
    """

        
    futures = []
    tests = []
    msirrt_files = []
    strrt_files = []
        
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:
        c = 0
        tasks_by_robot_count = os.listdir(path_to_tasks)
        
        for robot_count in tasks_by_robot_count:
            test_cases = os.listdir(os.path.join(path_to_tasks,robot_count))
            for test_case in test_cases:
                test_dir_path = os.path.join(path_to_tasks,robot_count,test_case)
                msirrt_dirs = [f for f in os.listdir(test_dir_path) if 'msirrt' in f]
                strrt_dirs = [f for f in os.listdir(test_dir_path) if 'strrt' in f]
                for msirrt_test in msirrt_dirs:
                    test_path = os.path.join(test_dir_path,msirrt_test)
                    futures.append(executor.submit(process_one_task, test_path, path_to_strrt_config_json,'msirrt','./STRRT_Planner/build/STRRT_Planner','strrt',"STRRT*_planner_logs"))
                for strrt_test in strrt_dirs:
                    test_path = os.path.join(test_dir_path,strrt_test)
                    futures.append(executor.submit(process_one_task, test_path, path_to_strrt_config_json,'strrt','./MSIRRT/build/MSIRRT_Planner','msirrt',"MSIRRT_planner_logs"))

                    
                    
        
    for future in tqdm(futures):
        future.result()


def check_args(args):
    if not os.path.exists(args.path_to_tasks):
        raise FileNotFoundError(f"Tasks path {args.path_to_tasks} does not exist")
        
    if not os.path.exists(args.path_to_strrt_config_json):
        raise FileNotFoundError(f"STRRT config path {args.path_to_strrt_config_json} does not exist")
    
def parse_args():
    parser = argparse.ArgumentParser(description='Process multiagent mass test')
    parser.add_argument('--path_to_tasks', type=str, help='Path to multiagent tasks', default='./multiagent_tests')
    parser.add_argument('--path_to_strrt_config_json', type=str, help='Path to strrt config json', default='./STRRT/strrt_config.json')
    
    args = parser.parse_args()
    check_args(args)
    return args

if __name__ == '__main__':
    args = parse_args()
    
    main(path_to_tasks = args.path_to_tasks, path_to_strrt_config_json = args.path_to_strrt_config_json)


