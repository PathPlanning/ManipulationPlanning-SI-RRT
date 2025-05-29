#!/usr/bin/env python3
"""
Mass visualization script for robot manipulation planning test results.

This script processes multiple test directories and generates visualization videos
for successful planning results. It uses parallel processing to handle multiple
Blender rendering tasks simultaneously.

Features:
- Parallel processing of multiple test cases
- Filtering by planner success status
- Decimation to limit number of videos per planner type
- Progress tracking with tqdm
"""

import os
import subprocess
from concurrent.futures import ProcessPoolExecutor
from tqdm import tqdm
import json
from pathlib import Path
from typing import List, Optional

# Number of parallel processes.
NUM_CPUS: int = 3

def visualise_one_task(path_to_log_file, path_to_scene_task,path_to_blender_scene):
    subprocess.run(f"blender --background --python ./Blender/scripts/visualise_one_test.py -- {path_to_log_file} {path_to_scene_task} {path_to_blender_scene} >> /dev/null", shell=True)
    return f"blender --background --python ./Blender/scripts/visualise_one_test.py -- {path_to_log_file} {path_to_scene_task} {path_to_blender_scene} >> /dev/null"

def planner_success(planner_logs_file_path:str):
    with open(planner_logs_file_path,'r') as f:
        logs = json.load(f)
    return logs['final_planner_data']['has_result']

def main(mass_test_dir:str)->None:

    futures = []
    with ProcessPoolExecutor(max_workers=NUM_CPUS) as executor:  
        for sphere_number in os.listdir(mass_test_dir):
            for test_suite in os.listdir(os.path.join(mass_test_dir,sphere_number)):
                current_dir = os.path.join(mass_test_dir,sphere_number,test_suite)
                path_to_blender_scene = os.path.join(current_dir,"mass_test.blend")
                path_to_scene_task = os.path.join(current_dir,"scene_task.json")
                log_files = [filename for filename in os.listdir(current_dir) if ("planner_logs" in filename) and (".json" == filename[-5:]) ]
                #decimation. leave only 3 biggest success log of each planner
                log_files.sort(key = lambda s: os.path.getsize(os.path.join(current_dir,s)),reverse=True)
                new_log_files = []
                MSIRRT_count = 0
                STRRT_count = 0
                DRGBT_count = 0
                for log_file in log_files:
                    path_to_log_file = os.path.join(current_dir,log_file)
                    # print(os.path.dirname(path_to_log_file)+"/visualisation_of_"+Path(path_to_log_file).stem+".mp4")
                    # if os.path.exists(os.path.dirname(path_to_log_file)+"/visualisation_of_"+Path(path_to_log_file).stem+".mp4"):
                    #     continue

                    if 'MSIRRT' in log_file and (MSIRRT_count<2):
                        if planner_success(path_to_log_file):
                            new_log_files.append(path_to_log_file)
                            MSIRRT_count+=1
                    if 'STRRT' in log_file and (STRRT_count<2):
                        if planner_success(path_to_log_file):
                            new_log_files.append(path_to_log_file)
                            STRRT_count+=1
                    if 'DRGBT' in log_file and (DRGBT_count<2):
                        # if planner_success(path_to_log_file):
                        new_log_files.append(path_to_log_file)
                        DRGBT_count+=1
                        
                for log_file in new_log_files:                    
                    # print(log_file)
                    # print(planner_success(log_file))
                    futures.append(executor.submit(visualise_one_task, log_file, path_to_scene_task,path_to_blender_scene))
        # print('processing futures:')
        for future in (pbar:=tqdm(futures)):
            command = future.result()
            print(command)



if __name__ == "__main__":
    mass_test_dir = "/mnt/tests"
    main(mass_test_dir)
