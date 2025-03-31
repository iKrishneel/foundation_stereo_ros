cmake_minimum_required(VERSION 3.8)
project(foundation_stereo_ros)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(LIB_DIR "${CMAKE_SOURCE_DIR}/foundation_stereo")

if(EXISTS ${LIB_DIR})
  message(STATUS "requirements.txt found, installing dependencies.")

  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pip install -r ${LIB_DIR}/requirements.txt --ignore-installed --no-build-isolation
    RESULT_VARIABLE install_result
    ERROR_VARIABLE install_error
  )

  if(NOT install_result EQUAL 0)
    message(FATAL_ERROR "Failed to install dependencies from requirements.txt: ${install_error}")
  else()
    message(STATUS "Dependencies installed successfully.")
  endif()
else()
  message(WARNING "requirements.txt not found. Skipping installation.")
endif()

execute_process(
  COMMAND ${Python3_EXECUTABLE} -m pip install e ${LIB_DIR}
  RESULT_VARIABLE install_result
  OUTPUT_VARIABLE install_output
  ERROR_VARIABLE install_error
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

install(PROGRAMS
  ${PROJECT_NAME}/foundation_stereo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)

ament_package()
