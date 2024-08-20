# Run driving_log_replayer_v2 evaluation

```shell
cd ${AUTOWARE_WORKSPACE}
source install/setup.bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_file}
# example
# ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=$HOME/driving_log_replayer_v2/yabloc.yaml
```
