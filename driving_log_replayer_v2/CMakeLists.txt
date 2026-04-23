cmake_minimum_required(VERSION 3.14)
project(driving_log_replayer_v2)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Python3 REQUIRED COMPONENTS Interpreter) 
if(Python3_VERSION VERSION_GREATER_EQUAL "3.10" AND Python3_VERSION VERSION_LESS "3.12")
  set(ROS_REQUIREMENTS "${CMAKE_CURRENT_SOURCE_DIR}/../requirements/humble.txt")
  execute_process(COMMAND bash -c "python3 -m pip install -r ${ROS_REQUIREMENTS}")
elseif(Python3_VERSION VERSION_GREATER_EQUAL "3.12")
  set(ROS_REQUIREMENTS "${CMAKE_CURRENT_SOURCE_DIR}/../requirements/jazzy.txt")
  execute_process(COMMAND bash -c "PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --ignore-installed -r ${ROS_REQUIREMENTS}")
else()
  message(
    FATAL_ERROR
    "Python version ${Python3_VERSION} is not supported. "
    "Use Python 3.10+ (3.10–3.11 or 3.12+ as configured for this build)."
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest)
  ament_add_pytest_test(driving_log_replayer_v2
    test
  )
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/perception_database_result.py
  scripts/perception_2d_evaluator_node.py
  scripts/traffic_light_evaluator_node.py
  scripts/obstacle_segmentation_evaluator_node.py
  scripts/performance_diag_evaluator_node.py
  scripts/localization_evaluator_node.py
  scripts/localization_update_result_jsonl.py
  scripts/yabloc_evaluator_node.py
  scripts/eagleye_evaluator_node.py
  scripts/ar_tag_based_localizer_evaluator_node.py
  scripts/annotationless_perception_evaluator_node.py
  scripts/planning_control_evaluator_node.py
  scripts/initial_pose_node.py
  scripts/goal_pose_node.py
  scripts/engage_node.py
  scripts/diagnostics_evaluator_node.py
  scripts/publish_topic_from_rosbag_node.py
  scripts/perception_reproducer_evaluator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES "${ROS_REQUIREMENTS}"
  DESTINATION share/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  sample
)
