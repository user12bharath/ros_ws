cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_executable(square_driver src/square_driver.cpp)
ament_target_dependencies(square_driver ${dependencies})

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance ${dependencies})

add_executable(target_reactor src/target_reactor.cpp)
ament_target_dependencies(target_reactor ${dependencies})

install(TARGETS
  square_driver
  obstacle_avoidance
  target_reactor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
