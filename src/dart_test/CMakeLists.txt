cmake_minimum_required(VERSION 3.8)
project(dart_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dart_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# mock dartmcu node
add_executable(mock_dartmcu_node src/dart_test_node.cpp)
target_include_directories(mock_dartmcu_node PUBLIC include ..)
ament_target_dependencies(mock_dartmcu_node rclcpp dart_msgs)
install(TARGETS mock_dartmcu_node DESTINATION lib/${PROJECT_NAME})

# system identification node
add_executable(system_identification_node src/system_identification_node.cpp)
target_include_directories(system_identification_node PUBLIC include ..)
ament_target_dependencies(system_identification_node rclcpp dart_msgs)
install(TARGETS system_identification_node DESTINATION lib/${PROJECT_NAME})

ament_package()
