"""
Clean planning results and visualisation videos and .blend files from mass test.

python3 ./mass_test/clean_experiments.py

To delete visualisation set delete_videos = True

To delete logs set delete_logs = True

Set test directories at test_dirs
"""

import argparse
import os

delete_videos = False
delete_logs = False

test_dirs = [f"./mass_test/tests/{i}_spheres" for i in range(0,301,20)]
test_dirs[0] ="./mass_test/tests/1_spheres"

def main() -> None:


    
    
    futures = []
    for test_dir in test_dirs:
        test_numbers = os.listdir(test_dir)
        for test_number_dir in test_numbers:
            files = os.listdir(os.path.join(test_dir,test_number_dir))
            for file in files:
                if delete_videos:
                    if '.mp4' in file:
                        print(os.path.join(test_dir,test_number_dir,file))
                        os.remove(os.path.join(test_dir,test_number_dir,file))
                    elif 'json.blend' in file:#,"mass_test.blend"]:
                        print(os.path.join(test_dir,test_number_dir,file))
                        os.remove(os.path.join(test_dir,test_number_dir,file))
                if delete_logs:
                    if ('MSIRRT_planner_logs' in file or 'DRGBT_planner_logs' in file or 'STRRT*_planner_logs' in file) and (file[-5:]=='.json'):
                        print(os.path.join(test_dir,test_number_dir,file))
                        os.remove(os.path.join(test_dir,test_number_dir,file))
    
    for future in futures:
        future.result()
        
if __name__ == '__main__':
    main()
