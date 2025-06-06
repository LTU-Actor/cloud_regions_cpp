cmake_minimum_required(VERSION 3.8)
project(cloud_regions_cpp)
cmake_policy(SET CMP0074 OLD)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(pcl_ros REQUIRED) 
# find_package(pcl REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(regions src/regions.cpp)
ament_target_dependencies(regions std_msgs geometry_msgs sensor_msgs visualization_msgs pcl_ros pcl_conversions)

install(TARGETS
  regions
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
      launch
      config
      rviz
      DESTINATION share/${PROJECT_NAME}
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

ament_package()
