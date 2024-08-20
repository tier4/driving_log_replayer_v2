# Command

The driving_log_replayer_v2 can be started by specifying the scenario path.

```shell
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=${scenario_path} [output_dir:=${output_dir} dataset_dir:=${dataset_dir}]
```

## Run driving_log_replayer_v2 with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
