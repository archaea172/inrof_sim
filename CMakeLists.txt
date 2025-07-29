cmake_minimum_required(VERSION 3.8)
project(inrof_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(control_pkg REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# add omni
add_executable(omni_controler
  src/omni_control.cpp
)
ament_target_dependencies(omni_controler
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
)
install(
  TARGETS omni_controler
  DESTINATION lib/${PROJECT_NAME}
)
# create mppi libray
add_library(mppi_library SHARED
  src/mppi_library.cpp
)
target_include_directories(mppi_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(mppi_library PUBLIC
  ${EIGEN3_LIBRARIES}
)

# add pursuit
add_executable(pursuit_controler
  src/pursuit.cpp
  include/${PROJECT_NAME}/pursuit.hpp
)
ament_target_dependencies(pursuit_controler
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mppi_library
  EXPORT export_mppi_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
target_link_libraries(pursuit_controler 
  mppi_library
  Eigen3::Eigen
)
install(
  TARGETS pursuit_controler
  DESTINATION lib/${PROJECT_NAME}
)
target_include_directories(pursuit_controler
  PRIVATE include/${PROJECT_NAME}
)
ament_export_dependencies(rclcpp rclcpp_lifecycle geometry_msgs Eigen3)
ament_export_targets(export_mppi_library HAS_LIBRARY_TARGET)
target_compile_options(pursuit_controler PRIVATE
    -O3           # 最適化レベル3
    -march=native # 実行するマシンのCPUに最適化
    -fopenmp
)


# add twist converter
add_executable(twist_converter 
  src/twist_convert.cpp
)
ament_target_dependencies(twist_converter 
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

install(
  TARGETS twist_converter 
  DESTINATION lib/${PROJECT_NAME}
)

# add pose converter
add_executable(pose_converter 
  src/pose_converter.cpp
)
ament_target_dependencies(pose_converter 
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

install(
  TARGETS pose_converter 
  DESTINATION lib/${PROJECT_NAME}
)

# launch
install(
  DIRECTORY launch worlds models hooks config
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

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

ament_package()
