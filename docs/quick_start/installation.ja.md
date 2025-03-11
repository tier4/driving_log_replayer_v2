# インストール

[AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware)を`driving_log_replayer_v2`と一緒にビルドする方法を解説します。

## Requirements

- CPU amd64
- Ubuntu 22.04
- ROS humble
- Python 3.10
- NVIDIA GPU (required if running perception)
- [zstd](https://github.com/facebook/zstd)
  - sudo apt install zstd

## ビルド方法

1. Autoware workspace に移動する:

   ```shell
   cd autoware
   ```

2. 依存パッケージを追加する:

   ```shell
   nano simulator.repos
   # 以下の内容を追加する
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
       version: humble
     simulator/vendor/ros2bag_extensions:
       type: git
       url: https://github.com/tier4/ros2bag_extensions.git
       version: main
   ```

3. simulator の依存パッケージをインポートする:

   ```shell
   vcs import src < simulator.repos
   ```

4. (Optional) 基本的にdriving_log_replayer_v2のmainブランチは最新のautowareと共に利用することを前提としているため、必要に応じてnightly.reposをimportする

   ```shell
   vcs import src < autoware-nightly.repos
   vcs import src < tools-nightly.repos
   ```

5. 依存解決のために rosdep を更新する:

   ```shell
   rosdep update
   ```

6. rosdep で依存のパッケージをインストールする:

   ```shell
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

7. ワークスペースをビルドする:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
