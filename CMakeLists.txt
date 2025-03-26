cmake_minimum_required(VERSION 3.8)
project(foundation_stereo_ros)

include(ExternalProject)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external")

ExternalProject_Add(
    FoundationStereo
    GIT_REPOSITORY https://github.com/NVlabs/FoundationStereo.git
    GIT_TAG master
    PREFIX ${EXTERNAL_DIR}
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${CMAKE_COMMAND} -E echo "No build step needed for Python projects"
    INSTALL_COMMAND ${CMAKE_COMMAND} -E echo "No install step needed for Python projects"
)

ExternalProject_Get_Property(FoundationStereo SOURCE_DIR)
# set(FOUNDATION_STEREO_DIR ${SOURCE_DIR})

# execute_process(
#   COMMAND bash -c "awk '/- pip:/,/- dependencies:/' '${FOUNDATION_STEREO_DIR}/environment.yml' | sed '1d;$d' | sed 's/^[[:space:]]*-\\s*//g' > '${FOUNDATION_STEREO_DIR}/requirements.txt'"
#   WORKING_DIRECTORY ${FOUNDATION_STEREO_DIR}
#   RESULT_VARIABLE result
#   OUTPUT_VARIABLE output
#   ERROR_VARIABLE error_output
#   COMMAND_ERROR_IS_FATAL ANY
# )

# add_custom_command(
#   TARGET FoundationStereo
#   POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E echo "Installing pip dependencies..."
#   COMMAND pip3 install -r ${FOUNDATION_STEREO_DIR}/requirements.txt --no-build-isolation --ignore-installed
#   WORKING_DIRECTORY ${FOUNDATION_STEREO_DIR}
#   )

install(PROGRAMS
  ${PROJECT_NAME}/foundation_stereo_node.py
  DESTINATION lib/${PROJECT_NAME}
  )

install(TARGETS DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

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
