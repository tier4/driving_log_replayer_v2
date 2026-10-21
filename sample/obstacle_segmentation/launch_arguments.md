# Obstacle segmentation execution arguments

Keep your scenario unchanged. Add the following arguments to your existing DLR launch command:

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  additional_launches:='[{"package":"aip_hd785_gen2_perception_launch","file":"radars_replay.launch.xml","arguments":{"use_sim_time":"true"}}]' \
  evaluation_topic:=/perception/radar_lidar_fusion/debug/appended_radar_points \
  evaluation_topic_rate:='null'
```

Rate monitoring uses existing diagnostics only. To monitor the original obstacle pointcloud:

```bash
evaluation_topic:=/perception/obstacle_segmentation/pointcloud \
evaluation_topic_rate:='{"module":"perception","node_name_suffix":"obstacle_segmentation_pointcloud"}'
```

Both omitted retain legacy behavior. Explicit `null` disables rate monitoring (Skipped).
An explicitly configured but missing/expired diagnostic is Invalid, emits a warning and fails the rate condition.
The target topic is added to bag recording automatically. Scenario format versions do not need changing.
Additional launches must be installed in the sourced workspace.

For DDS integration tests, source the workspace and set DLR_RUN_ROS_INTEGRATION=1; run
`pytest driving_log_replayer_v2/test/unittest/launch/test_evaluation_topic_ros.py`.
