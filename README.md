

# ManipulationPlanning-SI-RRT
Combination of Rapidly-Exporing Random Trees (RRT) and Safe Interval Path Planning (SIPP)  for high-DOF planning in dynamic environments, i.e. planning a path for a manipulator when moving obstacles are present (and their trajectories are know/accurately predicted) 

[[Paper](https://arxiv.org/abs/2412.19567)]

![experiments](<supplementary material/output1.gif>)

![experiments](<supplementary material/output.gif>)

![experiments](<supplementary material/0001-1376.gif>)

# MVP. Updates will be soon.

## Quick start

Download dataset https://disk.yandex.ru/d/-73LOGO5kOSYuA and extract ./tests directory in ./mass_test/tests 

Set number of parallel jobs in ./mass_test/do_mass_test.py NUM_CPUS. best practice - set NUM_CPUS as number of cores in CPU - 1

Download docker container, remove tests results, enter the container

```
make pull_docker
make clean_experiments_result
make enter_debug_docker
```


build planners and start mass test execution

```
if [ ! -d ./STRRT_Planner ]
    then
    cd ./app
    fi
make build_planner_debug
make mass_tests
 ```

## Analyse data

Use ./mass_test/final_analysis.ipynb to analyse planner execution data

Data, that has been used in the paper is available at https://disk.yandex.ru/d/-73LOGO5kOSYuA

Feel free to open issues!