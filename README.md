# ManipulationPlanning-SI-RRT

Combination of Rapidly-Exploring Random Trees (RRT) and Safe Interval Path Planning (SIPP) for high-DOF planning in dynamic environments, i.e., planning a path for a manipulator when moving obstacles are present (and their trajectories are known/accurately predicted).




![experiments](<supplementary material/output1.gif>)

![experiments](<supplementary material/output.gif>)

![experiments](<supplementary material/0001-1376.gif>)

[![Watch the demo video](https://img.youtube.com/vi/inTmRr0GXL8/0.jpg)](https://youtu.be/inTmRr0GXL8?si=BcaILcrB6HbhIhyT)


> [!Important]  
> **This repository contains code for the paper:**
>
> Kerimov N., Onegin A, Yakovlev K. *Safe Interval Randomized Path Planning For Manipulators*.
>
> **[[Full text on arXiv](https://arxiv.org/abs/2412.19567)]**

## Table of Contents

- [Overview](#overview)
- [Supported Algorithms](#supported-algorithms)
- [Repository Structure](#repository-structure)
- [Code Structure](#code-structure)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Visualization Examples](#visualization-examples)
- [Analyze Data](#analyze-data)
- [Citing This Work](#citing-this-work)
- [Contact](#contact)

## Overview

This repository contains the source code for path planning in dynamic environments for manipulators, Blender plugins, and code for creating and visualization of test cases.

## Supported Algorithms

The following path planning algorithms are implemented:

### DRGBT (Dynamic Rapidly-exploring Generalized Bur Tree)

**Description**: A reactive planner for manipulators that dynamically replans the path based on distance to obstacles.

**Implementation**: Forked from RPMPLv2 library and modified to use our collision detection functions.

### ST-RRT* (Space-Time RRT*)

**Description**: A deliberative anytime planner that works directly in the space-time domain to find collision-free paths.

**Implementation**: Forked from OMPL library and adapted for our framework.

### SI-RRT (Safe Interval RRT)

**Description**: Our novel deliberative planner that uses safe intervals for planning in dynamic environments.


**Implementation**: Original implementation developed for this research.

## Repository Structure

The repository is organized into several branches:

* **`main` branch** [[**Link**](https://github.com/PathPlanning/ManipulationPlanning-SI-RRT/tree/main)]: Contains up-to-date implementation of the algorithm, including multiagent tasks.
* **`reproducibility-version` branch** [[**Link**](https://github.com/PathPlanning/ManipulationPlanning-SI-RRT/tree/reproducibility-version)]: Contains version of the algorithm that was used in the paper's experiments.

## Code Structure

The codebase is organized into the following main directories:

- **`MSIRRT/`**: Our implementation of the Safe Interval RRT algorithm

- **`STRRT_Planner/`**: Space-Time RRT* implementation

- **`RPMPLv2/`**: DRGBT implementation, submoduled from https://github.com/PathPlanning/RPMPLv2/tree/new-compiler

- **`Blender/`**: Scripts and plugins for Blender visualization:
  - `urdf_importer/`: Blender add-on for importing URDF files. Compress it to .zip and install in Blender. Originally from https://github.com/Victorlouisdg/blender-urdf-importer , heavily modified
  - `scenes/`: Example Blender scene files for simulation and visualization
  - `robots/`: URDF and mesh files for supported robot models
  - `scripts/`: Utility scripts for exporting, importing, batch processing Blender scenes and creating mass tests

  For detailed information, prease refer to  [Blender Guide](./docs/Blender_guide.md).

- **`mass_test/`**: Scripts for running batch experiments
  - `do_mass_test.py`: Main script for executing tests in parallel
  - `clean_experiments.py`: Utility for cleaning test results
  - `final_analysis.ipynb`: Jupyter notebook for analyzing test results

- **`docker/`**: Docker configuration files for reproducible environments

- **`tests/`**: Test cases and benchmark scenarios

## Installation

For detailed installation instructions and dependencies please refer to our [Installation Guide](./docs/installation_dependencies.md).

## Quick Start

Follow these steps to quickly get started with the framework:

1. **Download the test dataset**:
   ```bash
   # Download the dataset
   wget https://disk.yandex.ru/d/-73LOGO5kOSYuA -O dataset.zip
   
   # Extract the test directory into the correct location
   unzip dataset.zip -d ./mass_test/tests
   ```

2. **Configure parallel execution**:
   - Open `./mass_test/do_mass_test.py` and set the `NUM_CPUS` variable
   - Recommended: Set `NUM_CPUS` to the number of CPU cores minus 1

3. **Set up the Docker environment**:
   ```bash
   # Pull the pre-built Docker image
   make pull_docker
   
   # Clean any existing test results
   make clean_experiments_result
   
   # Enter the Docker container
   make enter_debug_docker
   ```

4. **Build the planners and run tests**:
   ```bash
   # Navigate to the app directory inside the Docker container
   cd ./app
   
   # Build the planners in debug mode
   make build_planner_debug
   
   # Run the batch tests
   make mass_tests
   ```

5. **Verify successful execution**:
   - Check for the presence of result files in the `mass_test/tests` directory
   - Ensure no error messages were printed during execution



## Analyze Data

To analyze the results of your experiments open the Jupyter notebook `./mass_test/final_analysis.ipynb`

The dataset used in the original paper is available at: https://disk.yandex.ru/d/-73LOGO5kOSYuA

## Citing This Work

If you use this repository in your research, please cite the following paper:

```bibtex
@article{Kerimov_Onegin_Yakovlev_2025,
   title={Safe Interval Randomized Path Planning For Manipulators}, 
   volume={35}, 
   url={https://ojs.aaai.org/index.php/ICAPS/article/view/36120}, DOI={10.1609/icaps.v35i1.36120}, 
   number={1}, 
   journal={Proceedings of the International Conference on Automated Planning and Scheduling}, author={Kerimov, Nuraddin and Onegin, Aleksandr and Yakovlev, Konstantin},
   year={2025},
   month={Sep.}, 
   pages={213–217} 
}
```

## Contact

For questions or further information, please contact:

* Kerimov Nuraddin (*kerimov.nm@phystech.edu*)



