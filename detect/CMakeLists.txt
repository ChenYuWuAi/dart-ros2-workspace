cmake_minimum_required(VERSION 3.8)
project(detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(info REQUIRED)

find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(SOURCES src/main.cpp src/detect.cpp)

# detect node executable
add_executable(detect_node ${SOURCES})

ament_target_dependencies(
  detect_node
  rclcpp
  std_msgs
  geometry_msgs
  info
  sensor_msgs
  image_transport
  cv_bridge)

target_include_directories(
  detect_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(detect_node ${OpenCV_LIBRARIES})

# camera node executable
add_executable(camera_node src/node_camera.cpp camera_driver/DHVideoCapture.cpp)

# structure judge
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "aarch64")
  target_include_directories(
    camera_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/cameraDriver/DHCamera_arm64 # here
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/
    ${OpenCV_INCLUDE_DIRS})

  target_link_directories(
    camera_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/cameraDriver/DHCamera_arm64
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/cameraDriver/DHCamera_arm64/lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  message(STATUS "x86_64")
  target_include_directories(
    camera_node
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/cameraDriver/DHCamera # here
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/
    ${OpenCV_INCLUDE_DIRS})

  target_link_directories(
    camera_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/cameraDriver/DHCamera/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_driver/cameraDriver/DHCamera)
endif()

target_link_libraries(camera_node ${OpenCV_LIBS} MVSDK)

ament_target_dependencies(camera_node rclcpp sensor_msgs image_transport
  cv_bridge)

add_executable(parameter_node src/Parameters.cpp)
target_include_directories(parameter_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(parameter_node ${OpenCV_LIBRARIES})
target_sources(parameter_node PRIVATE src/detect.cpp)
ament_target_dependencies(parameter_node)

install(TARGETS detect_node camera_node parameter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and  license is
  # added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
