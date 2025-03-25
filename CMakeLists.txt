cmake_minimum_required(VERSION 3.8)
project(foundation_stereo_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# add_executable(foundation_stereo_node src/foundation_stereo_node.cpp)
# target_include_directories(foundation_stereo_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(foundation_stereo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   foundation_stereo_node
#   "sensor_msgs/Image"
#   "sensor_msgs/CameraInfo"
#   "sensor_msgs/PointCloud2"
#   "cv_bridge"
# )

# install(TARGETS foundation_stereo_node
#   DESTINATION lib/${PROJECT_NAME})

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
