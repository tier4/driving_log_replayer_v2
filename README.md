# driving_log_replayer_v2 for ROS 2 Autoware.Universe

driving_log_replayer_v2 is a ROS package that evaluates the functionality of Autoware.Universe

## Requirements

- ROS 2 humble
- [Python 3.10](https://www.python.org/)

### Optional

If you want to change the rosbag format from ros1 to ros2.

- [rosbags](https://gitlab.com/ternaris/rosbags)
  - `pip3 install rosbags`

## Installation

Use colcon build

```shell
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to driving_log_replayer_v2
```

### How to install driving_log_replayer_v2_cli package

Use pipx **Do not use pip**

```shell
# install
pipx install git+https://github.com/tier4/driving_log_replayer_v2.git

# upgrade
pipx upgrade driving-log-replayer-v2

# uninstall
pipx uninstall driving-log-replayer-v2
```

### Shell Completion

Execute the following command so that you can complete the command in the shell.

#### bash

```shell
_DLR2_COMPLETE=bash_source dlr2 > $HOME/.dlr2-complete.bash
_DLR2_COMPLETE=bash_source dlr2 > $HOME/.dlr2-analyzer-complete.bash

echo "source $HOME/.dlr2-complete.bash" >> ~/.bashrc
echo "source $HOME/.dlr2-analyzer-complete.bash" >> ~/.bashrc
```

#### fish

```shell
_DLR2_COMPLETE=fish_source dlr2 > $HOME/.config/fish/completions/dlr2.fish
_DLR2_ANALYZER_COMPLETE=fish_source dlr2-analyzer > $HOME/.config/fish/completions/dlr2-analyzer.fish
```

## Usage

refer [document](https://tier4.github.io/driving_log_replayer_v2/)

## (For Developer) Release Process

This package uses `catkin_pkg` to manage releases.

Refer [this page](https://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage)

### Release command

Can only be executed by users with repository maintainer privileges

```shell
# create change log
catkin_generate_changelog
# edit CHANGELOG.rst
# update package version in pyproject.toml
# edit ReleaseNotes.md
# commit and create pull request
# merge pull request
catkin_prepare_release
# When you type the command, it automatically updates CHANGELOG.rst and creates a git tag
git checkout main
git merge develop
git push origin main
```
