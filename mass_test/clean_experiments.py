"""
python3 ./mass_test/clean_experiments.py

"""

import argparse
import os
import shutil


def main() -> None:


    
    path_to_tasks = './multiagent_tests'
    tasks_by_robot_count = os.listdir(path_to_tasks)
        
    futures = []
    for robot_count in tasks_by_robot_count:
        test_cases = os.listdir(os.path.join(path_to_tasks,robot_count))
        for test_case in test_cases:
            for i in range(10):
                test_dir_path = os.path.join(path_to_tasks,robot_count,test_case,"generated_tasks_msirrt_seed_0")
                if os.path.exists(test_dir_path):
                    print(test_dir_path)
                    shutil.rmtree(test_dir_path)
        
if __name__ == '__main__':
    main()
