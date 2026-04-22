# インストール

[AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware)を`driving_log_replayer_v2`と一緒にビルドする方法を解説します。

## Requirements

- CPU amd64
- Ubuntu 24.04
- ROS jazzy
- Python 3.12
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

   Ubuntu 24.04 以降（Python 3.12）では、rosdep が **pip** ルールを使う際に [PEP 668](https://peps.python.org/pep-0668/) により `PIP_BREAK_SYSTEM_PACKAGES=1` が必要です（CI も同じ）。

   ```shell
   PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

7. ワークスペースをビルドする:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   `driving_log_replayer_v2` の CMake 設定時に、倉庫上の `requirements.txt` へ記載の追加の Python 依存（Pydantic v2、nuscenes 系、`driving_log_replayer_v2_analyzer` 用の Plotly 等）を、colcon が用いる Python 環境へ `pip` で導入します。Ubuntu 24.04 では PEP 668 のため、CMake 内で `PIP_BREAK_SYSTEM_PACKAGES=1` を付与しています。事前に仮想環境などで入れ済みの場合は、`-DDRIVING_LOG_REPLAYER_V2_INSTALL_PYTHON_DEPS=OFF` を付けて本処理を抑止し、同じ内容の依存を手動で入れてください。インストール後は、パッケージ prefix 内の `share/driving_log_replayer_v2/requirements.txt` にも同ファイルが置かれます。
