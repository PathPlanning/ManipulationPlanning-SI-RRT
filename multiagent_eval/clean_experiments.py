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
                to_delete = [os.path.join(path_to_tasks,robot_count,test_case,x) for x in os.listdir(os.path.join(path_to_tasks,robot_count,test_case)) if "generated_tasks_strrt_seed_" in x]
                to_delete = filter(os.path.isdir,to_delete)
                for directory in to_delete:                
                    print(directory)
                    shutil.rmtree(directory)
        
if __name__ == '__main__':
    main()
