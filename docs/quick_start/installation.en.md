# Installation

This document contains step-by-step instruction on how to build [AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware) with `driving_log_replayer_v2`.

## Requirements

- CPU amd64
- Ubuntu 24.04
- ROS jazzy
- Python 3.12
- NVIDIA GPU (required if running perception)
- [zstd](https://github.com/facebook/zstd)
  - sudo apt install zstd

## How to build

1. Navigate to the Autoware workspace:

   ```shell
   cd autoware
   ```

2. Add dependency packages:

   ```shell
   nano simulator.repos
   # add repositories below.
   ```

   ```shell
     simulator/perception_eval:
       type: git
       url: https://github.com/tier4/autoware_perception_evaluation.git
       version: main
     simulator/driving_log_replayer_v2:
       type: git
       url: https://github.com/tier4/driving_log_replayer_v2.git
       version: main
     simulator/vendor/ros2_numpy:
       type: git
       url: https://github.com/Box-Robotics/ros2_numpy.git
       version: jazzy
     simulator/vendor/ros2bag_extensions:
       type: git
       url: https://github.com/tier4/ros2bag_extensions.git
       version: main
     simulator/tool/autoware_tools:
       type: git
       url: https://github.com/autowarefoundation/autoware_tools.git
       version: main
   ```

3. Import Simulator dependencies:

   ```shell
   vcs import src < simulator.repos
   ```

4. (Optional) Basically, the main branch of driving_log_replayer_v2 is intended to be used with the latest autoware, so import nightly.repos as needed.

   ```shell
   vcs import src < autoware-nightly.repos
   vcs import src < tools-nightly.repos
   ```

5. Update rosdep:

   ```shell
   rosdep update
   ```

6. Install dependent ROS packages:

   ```shell
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

7. Build the workspace:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   When `driving_log_replayer_v2` configures, it installs the extra Python dependencies in `requirements.txt` (Pydantic v2, nuscenes stack, `driving_log_replayer_v2_analyzer` tools such as Plotly) into the same Python used by colcon. On Ubuntu 24.04 this uses `PIP_BREAK_SYSTEM_PACKAGES=1` (handled in `CMakeLists.txt`). To skip that step (e.g. you already installed the file in a venv or custom image), build with `-DDRIVING_LOG_REPLAYER_V2_INSTALL_PYTHON_DEPS=OFF` and install dependencies yourself. After install, the file is also under `share/driving_log_replayer_v2/requirements.txt` in the package prefix.
