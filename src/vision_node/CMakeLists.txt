cmake_minimum_required(VERSION 3.8)
project(vision_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED)

add_executable(vision_monitor src/vision_monitor_node.cpp)
ament_target_dependencies(vision_monitor ${dependencies})

add_executable(face_detector src/face_detector_node.cpp)
ament_target_dependencies(face_detector ${dependencies})
target_link_libraries(face_detector ${OpenCV_LIBS})

add_executable(sim_camera_detector src/sim_camera_detector.cpp)
ament_target_dependencies(sim_camera_detector ${dependencies})
target_link_libraries(sim_camera_detector ${OpenCV_LIBS})

add_executable(yolo_camera_detector src/yolo_camera_detector.cpp)
ament_target_dependencies(yolo_camera_detector ${dependencies})
target_link_libraries(yolo_camera_detector ${OpenCV_LIBS})

install(TARGETS
  vision_monitor
  face_detector
  sim_camera_detector
  yolo_camera_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(PROGRAMS
  vision_node/yolo_detector_py.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detector_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
