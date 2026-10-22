cmake_minimum_required(VERSION 3.8)
project(lab_replica)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config description launch models worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/build_models.py
  scripts/capture_camera.py
  scripts/action_console.py
  scripts/check_goal.py
  scripts/do_action.py
  scripts/generate_problem.py
  scripts/goal_console.py
  scripts/replan_problem.py
  scripts/rg6_driver.py
  scripts/run_solution.py
  scripts/skill_server.py
  scripts/solve.py
  scripts/workspace_view.py
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
