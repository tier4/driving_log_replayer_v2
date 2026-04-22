cmake_minimum_required(VERSION 3.14)
project(driving_log_replayer_v2)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

# Extra Python packages (Pydantic v2, nuscenes, analyzer plotly, etc.): not fully covered by apt/rosdep.
# Ubuntu 24.04+ (PEP 668): PIP_BREAK_SYSTEM_PACKAGES=1 allows pip into the same env as /usr/bin/python3
# (typical for Autoware/colcon). Set OFF to skip if you pre-install (venv, custom image) or use rosdep+manual pip.
option(
  DRIVING_LOG_REPLAYER_V2_INSTALL_PYTHON_DEPS
  "Run pip install -r ../requirements.txt during configure (recommended for colcon+ROS use)"
  ON
)

# Repo: <...>/driving_log_replayer_v2/CMakeLists.txt -> <...>/driving_log_replayer_v2/../ = clone root
get_filename_component(
  _DRIVING_LOG_REPLAYER_V2_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE
)
set(_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS "${_DRIVING_LOG_REPLAYER_V2_REPO_ROOT}/requirements.txt")
set(_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS "${_DRIVING_LOG_REPLAYER_V2_REPO_ROOT}/constraints-pip.txt")

if(DRIVING_LOG_REPLAYER_V2_INSTALL_PYTHON_DEPS AND EXISTS "${_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS}")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_dlr2_pip_env "PIP_BREAK_SYSTEM_PACKAGES=1")
  # Python 3.12: old setuptools in pip build env hits pkgutil.ImpImporter; versions in constraints-pip.txt
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${_dlr2_pip_env}
    ${Python3_EXECUTABLE} -m pip install -q -U -r "${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS}"
    RESULT_VARIABLE _driving_log_replayer_v2_pip
  )
  if(_driving_log_replayer_v2_pip EQUAL 0)
    if(EXISTS "${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS}")
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${_dlr2_pip_env}
        ${Python3_EXECUTABLE} -m pip install -q -c "${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS}" -r "${_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS}"
        RESULT_VARIABLE _driving_log_replayer_v2_pip
      )
    else()
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${_dlr2_pip_env}
        ${Python3_EXECUTABLE} -m pip install -q -r "${_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS}"
        RESULT_VARIABLE _driving_log_replayer_v2_pip
      )
    endif()
  endif()
  if(NOT _driving_log_replayer_v2_pip EQUAL 0)
    message(
      WARNING
      "pip for driving_log_replayer_v2 failed (exit ${_driving_log_replayer_v2_pip}). On Python 3.12: ${Python3_EXECUTABLE} -m pip install -U -r ${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS}; then -m pip install -c ${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS} -r ${_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS}. Or colcon with -DDRIVING_LOG_REPLAYER_V2_INSTALL_PYTHON_DEPS=OFF."
    )
  endif()
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

if(EXISTS "${_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS}")
  install(FILES "${_DRIVING_LOG_REPLAYER_V2_REQUIREMENTS}"
    DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS "${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS}")
  install(FILES "${_DRIVING_LOG_REPLAYER_V2_PIP_CONSTRAINTS}"
    DESTINATION share/${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  sample
)
