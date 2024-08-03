cmake_minimum_required(VERSION 3.8)
project(rrt_planner)

# Default to C99
set(CMAKE_C_STANDARD 99)


# Default to C++14
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rcl REQUIRED)
# find_package(rosidl_typesupport_interface REQUIRED)
# find_package(rcpputils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(utils REQUIRED)
find_package(planner_core REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(OpenMP REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  # ${ament_cmake_INCLUDE_DIRS}
  # ${OpenMP_INCLUDE_DIRS}
  # ${nav2_core_INCLUDE_DIRS}
  # ${planner_core_INCLUDE_DIRS}
  # ${rcl_INCLUDE_DIRS}
  # ${rosidl_typesupport_interface_INCLUDE_DIRS}
  # ${rcpputils_INCLUDE_DIRS}
  # ${builtin_interfaces_INCLUDE_DIRS}
  # ${libstatistics_collector_INCLUDE_DIRS}
  # ${rcl_lifecycle_INCLUDE_DIRS}
  # ${map_msgs_INCLUDE_DIRS}
  # ${tf2_msgs_INCLUDE_DIRS}
  # ${class_loader_INCLUDE_DIRS}
  # ${pluginlib_INCLUDE_DIRS}
)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}") 
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") 
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 
endif()

set(dependencies
  angles
  # rcl
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  utils
  planner_core
  # eigen3_cmake_module
  # rosidl_typesupport_interface
  # rcpputils
)

add_library(${PROJECT_NAME} SHARED
  src/rrt_planner.cpp
  src/rrt.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core rrt_planner_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  # EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  # INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(FILES rrt_planner_plugin.xml 
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_export_include_directories(include ${OMPL_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
