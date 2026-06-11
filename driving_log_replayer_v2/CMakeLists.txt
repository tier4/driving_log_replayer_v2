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

# real_log_sim_comparison: ファイル散らばり解消のため evaluator_node は
# driving_log_replayer_v2/real_log_sim_comparison/ 配下に集約。
# launch から `<use_case>_evaluator_node.py` の固定命名で呼ばれるため RENAME で互換維持。
install(PROGRAMS
  driving_log_replayer_v2/real_log_sim_comparison/evaluator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME real_log_sim_comparison_evaluator_node.py
)

install(FILES "${ROS_REQUIREMENTS}"
  DESTINATION share/${PROJECT_NAME})

# real_log_sim_comparison: sample も use case ディレクトリへ集約 (cloud / local 統一)。
# share install パス sample/real_log_sim_comparison は cloud Web.Auto evaluator 互換のため維持
# (登録済み scenario の Conditions が install-share の絶対パスを参照する)。
# `install(DIRECTORY)` だと `--symlink-install` モードで PATTERN EXCLUDE が効かず
# sample/out/ 配下の実行結果 (untracked, 数GB) まで symlink install されてしまうため、
# 必要ファイルを明示列挙する。
install(FILES
  driving_log_replayer_v2/real_log_sim_comparison/sample/scenario.yaml
  driving_log_replayer_v2/real_log_sim_comparison/sample/README.ja.md
  DESTINATION share/${PROJECT_NAME}/sample/real_log_sim_comparison
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  sample
)
