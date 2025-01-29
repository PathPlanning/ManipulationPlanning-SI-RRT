"""
Process multiagent blender scene and plan path for manipulator with STRRT* and DRGBT

python3 ./mass_test/clean_experiments.py

"""

import argparse
import os

def parse_my_args(argv=None):
    PARSER = argparse.ArgumentParser()
    PARSER.add_argument("--path_to_strrt_config_json",
                        type=str,
                        default='./STRRT/strrt_config.json',
                        help="path to .json, where strrt hyperparametrs is described")
    # parse the arguments
    return PARSER.parse_args(argv)



def main( path_to_strrt_config_json: str) -> None:


    test_dirs = [f"./mass_test/tests/{i}_spheres" for i in range(0,301,20)]
    test_dirs[0] ="./mass_test/tests/1_spheres"
    
    
    futures = []
    for test_dir in test_dirs:
        test_numbers = os.listdir(test_dir)
        for test_number_dir in test_numbers:
            files = os.listdir(os.path.join(test_dir,test_number_dir))
            for file in files:
                if 'MSIRRT' in file:#,"mass_test.blend"]:
                    print(os.path.join(test_dir,test_number_dir,file))
                    os.remove(os.path.join(test_dir,test_number_dir,file))
    
    for future in futures:
        future.result()
        
if __name__ == '__main__':
    args = parse_my_args()
    main(args.path_to_strrt_config_json)
