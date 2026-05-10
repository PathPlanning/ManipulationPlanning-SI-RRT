# ManipulationPlanning-SI-RRT

[![ICAPS 2025 Best Student Paper](https://img.shields.io/badge/ICAPS%202025-Best%20Student%20Paper%20%F0%9F%9A%80-blue?style=for-the-badge)](https://ojs.aaai.org/index.php/ICAPS/article/view/36120)
[![ICAPS 2025 Best Student Paper](https://img.shields.io/badge/ICAPS%202025-Best%20Student%20Paper%20%F0%9F%9A%80-blue?style=flat-square)](https://ojs.aaai.org/index.php/ICAPS/article/view/36120)
[![Paper](https://img.shields.io/badge/Paper-ICAPS%202025-2ea44f?style=flat-square)](https://ojs.aaai.org/index.php/ICAPS/article/view/36120)
[![PDF](https://img.shields.io/badge/PDF-Official-red?style=flat-square)](https://ojs.aaai.org/index.php/ICAPS/article/view/36120/38274)
[![arXiv](https://img.shields.io/badge/arXiv-2412.19567-b31b1b?style=flat-square&logo=arxiv)](https://arxiv.org/abs/2412.19567)
[![DOI](https://img.shields.io/badge/DOI-10.1609%2Ficaps.v35i1.36120-blue?style=flat-square)](https://doi.org/10.1609/icaps.v35i1.36120)

Combination of Rapidly-Exploring Random Trees (RRT) and Safe Interval Path Planning (SIPP) for high-DOF planning in dynamic environments, i.e., planning a path for a manipulator when moving obstacles are present and their trajectories are known or accurately predicted.

> [!IMPORTANT]  
> 🚀 **Best Student Paper Award — ICAPS 2025**
>
> This repository contains the code for the paper:
>
> **Kerimov N., Onegin A., Yakovlev K.**  
> *Safe Interval Randomized Path Planning For Manipulators*.
>
> **Official ICAPS publication:**  
> - [[Paper page](https://ojs.aaai.org/index.php/ICAPS/article/view/36120)]  
> - [[Official PDF](https://ojs.aaai.org/index.php/ICAPS/article/view/36120/38274)]  
> - [[Full text on arXiv](https://arxiv.org/abs/2412.19567)]

![experiments](<supplementary material/output1.gif>)

![experiments](<supplementary material/output.gif>)

![experiments](<supplementary material/0001-1376.gif>)

[![Watch the demo video](https://img.youtube.com/vi/inTmRr0GXL8/0.jpg)](https://youtu.be/inTmRr0GXL8?si=BcaILcrB6HbhIhyT)

## Table of Contents

- [Overview](#overview)
- [Supported Algorithms](#supported-algorithms)
- [Repository Structure](#repository-structure)
- [Code Structure](#code-structure)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Analyze Data](#analyze-data)
- [Citing This Work](#citing-this-work)
- [Contact](#contact)

## Overview

This repository contains the source code for path planning in dynamic environments for manipulators, Blender plugins, and utilities for creating, running, and visualizing benchmark scenarios.

## Supported Algorithms

The following path planning algorithms are implemented:

### DRGBT — Dynamic Rapidly-exploring Generalized Bur Tree

**Description:** A reactive planner for manipulators that dynamically replans the path based on the distance to obstacles.

**Implementation:** Forked from the RPMPLv2 library and modified to use our collision detection functions.

### ST-RRT* — Space-Time RRT*

**Description:** A deliberative anytime planner that operates directly in the space-time domain to find collision-free paths.

**Implementation:** Forked from the OMPL library and adapted for our framework.

### SI-RRT — Safe Interval RRT

**Description:** Our deliberative planner that uses safe intervals for planning in dynamic environments.

**Implementation:** Original implementation developed for this research.

## Repository Structure

The repository is organized into several branches:

- **`main` branch** [[Link](https://github.com/PathPlanning/ManipulationPlanning-SI-RRT/tree/main)]: Contains the up-to-date implementation of the algorithm, including multi-agent tasks.
- **`reproducibility-version` branch** [[Link](https://github.com/PathPlanning/ManipulationPlanning-SI-RRT/tree/reproducibility-version)]: Contains the version of the algorithm used in the paper experiments.

## Code Structure

The codebase is organized into the following main directories:

- **`MSIRRT/`**: Implementation of the Safe Interval RRT algorithm.
- **`STRRT_Planner/`**: Space-Time RRT* implementation.
- **`RPMPLv2/`**: DRGBT implementation, submoduled from [RPMPLv2](https://github.com/PathPlanning/RPMPLv2/tree/new-compiler).
- **`Blender/`**: Scripts and plugins for Blender visualization:
  - `urdf_importer/`: Blender add-on for importing URDF files. Compress it to `.zip` and install it in Blender. Originally based on [blender-urdf-importer](https://github.com/Victorlouisdg/blender-urdf-importer), with substantial modifications.
  - `scenes/`: Example Blender scene files for simulation and visualization.
  - `robots/`: URDF and mesh files for supported robot models.
  - `scripts/`: Utility scripts for exporting, importing, batch processing Blender scenes, and creating large-scale tests.

  For detailed information, please refer to the [Blender Guide](./docs/Blender_guide.md).

- **`mass_test/`**: Scripts for running batch experiments:
  - `do_mass_test.py`: Main script for executing tests in parallel.
  - `clean_experiments.py`: Utility for cleaning test results.
  - `final_analysis.ipynb`: Jupyter notebook for analyzing test results.

- **`docker/`**: Docker configuration files for reproducible environments.
- **`tests/`**: Test cases and benchmark scenarios.

## Installation

For detailed installation instructions and dependencies, please refer to the [Installation Guide](./docs/installation_dependencies.md).

## Quick Start

Follow these steps to quickly get started with the framework:

1. **Download the test dataset**:

   ```bash
   wget https://disk.yandex.ru/d/-73LOGO5kOSYuA -O dataset.zip
   unzip dataset.zip -d ./mass_test/tests
   ```

2. **Configure parallel execution**:

   Open `./mass_test/do_mass_test.py` and set the `NUM_CPUS` variable.

   Recommended value: the number of available CPU cores minus one.

3. **Set up the Docker environment**:

   ```bash
   make pull_docker
   make clean_experiments_result
   make enter_debug_docker
   ```

4. **Build the planners and run tests**:

   ```bash
   cd ./app
   make build_planner_debug
   make mass_tests
   ```

5. **Verify successful execution**:

   Check that result files are present in the `mass_test/tests` directory and that no error messages were printed during execution.

## Analyze Data

To analyze experiment results, open the Jupyter notebook:

```text
./mass_test/final_analysis.ipynb
```

The dataset used in the original paper is available at:

```text
https://disk.yandex.ru/d/-73LOGO5kOSYuA
```

## Citing This Work

If you use this repository in your research, please cite the following paper:

```bibtex
@article{Kerimov_Onegin_Yakovlev_2025,
   title={Safe Interval Randomized Path Planning For Manipulators},
   volume={35},
   url={https://ojs.aaai.org/index.php/ICAPS/article/view/36120},
   DOI={10.1609/icaps.v35i1.36120},
   number={1},
   journal={Proceedings of the International Conference on Automated Planning and Scheduling},
   author={Kerimov, Nuraddin and Onegin, Aleksandr and Yakovlev, Konstantin},
   year={2025},
   month={Sep.},
   pages={213–217}
}
```

## Contact

For questions or further information, please contact:

* Kerimov Nuraddin (*[kerimov.nm@phystech.edu](mailto:kerimov.nm@phystech.edu)*)

If you encounter any issues with the code, documentation, or reproducibility, please feel free to open a GitHub issue. Feedback and suggestions are welcome.