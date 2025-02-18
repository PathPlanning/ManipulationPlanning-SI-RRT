

# ManipulationPlanning-SI-RRT
Combination of Rapidly-Exporing Random Trees (RRT) and Safe Interval Path Planning (SIPP)  for high-DOF planning in dynamic environments, i.e. planning a path for a manipulator when moving obstacles are present (and their trajectories are know/accurately predicted) 

<!--Блок информации о репозитории в бейджах-->
<!-- ![Static Badge](https://img.shields.io/badge/OkulusDev-Oxygen-Oxygen)
![GitHub top language](https://img.shields.io/github/languages/top/OkulusDev/Oxygen)
![GitHub](https://img.shields.io/github/license/OkulusDev/Oxygen)
![GitHub Repo stars](https://img.shields.io/github/stars/OkulusDev/Oxygen)
![GitHub issues](https://img.shields.io/github/issues/OkulusDev/Oxygen)
 -->



<p align="center">

![experiments](<supplementary material/output1.gif>)

![experiments](<supplementary material/output.gif>)

![experiments](<supplementary material/0001-1376.gif>)
<p\>

> [!Important]  
> **This repository contains code for the paper:**
>
> Kerimov N., Onegin A, Yakovlev K. *Safe Interval Randomized Path Planning For Manipulators*.
>
> **[[Full text on arXiv](https://arxiv.org/abs/2408.14948)]**

## Overview

This repository contains the source code for path planning in dynamic enviroments for manipulators and Blender plugins and code for creating and visualisation test cases. 

### Supported Algorithms

The following path planning algorithms are used:

* `DRGBT`: reactive planner for manipulators. We forked authorss implementation from RPMPLv2 library and modified it to use our collisiion detection functions.
* `ST-RRT*`: deliberative planner for planning in space-time. We forked authors'implementation from OMPL library.
* `SI-RRT`: deliberative planner that uses safe intervals for planning in dynamic enviroment.

This repository can also be used as a base framework for implementing custom AMAPF algorithms.

<!-- ## Repository Structure

The repository is organized into several branches:

* **`main` branch** [[**Link**](https://github.com/PathPlanning/TP-SWAP/tree/main)]: Contains the core implementation of the AMAPF algorithms, including all supported methods.
* **`experiments` branch** [[**Link**](https://github.com/PathPlanning/TP-SWAP/tree/experiments)]: Includes scripts and resources for running full-scale experiments, as described in the referenced paper. This branch provides tools for task generation, experiment execution, and result analysis.
* **`supplementary` branch** [[**Link**](https://github.com/PathPlanning/TP-SWAP/tree/supplementary)]: Contains extended experimental results analysis. This branch stands as supplementary material for the paper. -->

## Quick start

Download dataset https://disk.yandex.ru/d/-73LOGO5kOSYuA and extract ./tests directory in ./mass_test/tests 

Set number of parallel jobs in ./mass_test/do_mass_test.py NUM_CPUS. best practice - set NUM_CPUS as number of cores in CPU - 1

Download docker container, remove tests results from the dataset, enter the container

```
make pull_docker
make clean_experiments_result
make enter_debug_docker
```

```
cd ./app
```

build planners and start mass test execution

```
make build_planner_debug
make mass_tests
```

## Analyse data

Use ./mass_test/final_analysis.ipynb to analyse planner execution data

Data, that has been used in the paper is available at https://disk.yandex.ru/d/-73LOGO5kOSYuA

<!-- ## Installation and Launch

### Main Requirements

To use the repository, install the following software and libraries:

* `coal`
* `Blender`
 
It is recomended to build *coal* from sources, using *-march=native -03* compiler flags  -->

<!-- 
### Installation

After all requirements was installed, you should clone this repo in separate folder and run installation process (tested on Linux and macOS):

To set up this repository, follow these steps (tested on Linux and macOS):

1. Clone the repository:

```bash
git clone git@github.com:PathPlanning/TP-SWAP.git tp-swap
```

2. Install the package:

```bash
cd tp-swap
pip install -e .
``` -->
<!-- 
### Launching Algorithms

To run and evaluate the algorithms, clone this reporitory into separate folder second time and switch to the `experiments` branch, which includes all necessary scripts and tools for conducting experiments.

```bash
cd ..
git clone git@github.com:PathPlanning/TP-SWAP.git tp-swap-exp
cd tp-swap-exp
git checkout experiments
```

Detailed instructions for running experiments, generating tasks, and processing results can be found in the README of the `experiments` branch.

 -->
## Citing This Work

If you use this repository in your research, please cite the following paper:

```bibtex
@misc{kerimov2025safeintervalrandomizedpath,
      title={Safe Interval Randomized Path Planning For Manipulators}, 
      author={Nuraddin Kerimov and Aleksandr Onegin and Konstantin Yakovlev},
      year={2025},
      eprint={2412.19567},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2412.19567}, 
}
```

## Contact

For questions or further information, please contact:

* Kerimov Nuraddin (*kerimov.nm@phystech.edu*)



