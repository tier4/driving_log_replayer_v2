"""
Launch topic relay node for multi-run LIVE topic renaming
This provides real-time topic renaming (zero post-processing overhead)
"""
from launch import LaunchContext
from launch.actions import LogInfo
from launch_ros.actions import Node


def launch_live_topic_relay(context: LaunchContext) -> list:
    """
    Launch a relay node to rename LIVE trajectory topic in real-time

    This subscribes to the original planning output and republishes to a renamed topic.
    The recorder captures the renamed topic with zero overhead.
    """
    conf = context.launch_configurations
    live_prefix = conf.get("live_topic_prefix", "")

    if not live_prefix:
        return [LogInfo(msg="No live_topic_prefix specified, skipping topic relay")]

    base_topic = conf.get("base_trajectory_topic",
                         "/planning/trajectory_generator/diffusion_planner_node/output/trajectory")
    renamed_topic = f"/{live_prefix}{base_topic}"

    print(f"Launching topic relay: {base_topic} → {renamed_topic}")

    # Use topic_tools/relay node
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='live_trajectory_relay',
        namespace='/driving_log_replayer_v2',
        parameters=[{'use_sim_time': True}],
        arguments=[base_topic, renamed_topic],
        output='screen'
    )

    return [relay_node]
