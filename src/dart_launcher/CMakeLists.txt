cmake_minimum_required(VERSION 3.18)
project(dart_launcher)

# CCache 是否存在
if(EXISTS "/usr/bin/ccache")
    message(STATUS "CCache found, using ccache.")
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
    set(CMAKE_C_COMPILER_LAUNCHER ccache)
else()
    message(STATUS "CCache not found, not using ccache.")
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()


# 将编译优化从 -O0 改为 -O3，并启用 ARM 平台的优化选项
# 更新编译优化选项以适配 Cortex-A76
# If Arm Cortex-A76 is used, set the optimization flags accordingly
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        message(STATUS "aarch64")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a -mtune=cortex-a76 -ffast-math")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        message(STATUS "x86_64")
        # x86_64 处理器的优化选项
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")
else()
        message(STATUS "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== Options ==========

# option(EXAMPLE_USE_EXTERNAL_JSON "Use an external JSON library" OFF)
# option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# ========== Options ==========

# ========== General Dependencies =========
# json and lvgl
add_subdirectory(thirdparty)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dart_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(yaml-cpp REQUIRED)

# # 查找 GSL 库
# find_package(GSL REQUIRED)

# ========== General Dependencies ==========

include_directories(include)
include_directories(..)

# ========= Add Executables ==========
# ==== Add node_dart_param_gateway target ====
add_executable(node_dart_param_gateway src/node_dart_param_gateway.cpp ${CMAKE_CURRENT_LIST_DIR}/../dart_comm_share/src/dart_launcher_param.cpp)
target_include_directories(node_dart_param_gateway PRIVATE thirdparty/json/include)
target_link_directories(node_dart_param_gateway PRIVATE nlohmann_json::nlohmann_json)
ament_target_dependencies(node_dart_param_gateway rclcpp std_msgs dart_msgs std_srvs rclcpp_lifecycle)

# # 看门狗
add_executable(node_dart_launcher_lifecycle src/node_dart_launcher_lifecycle.cpp)
ament_target_dependencies(node_dart_launcher_lifecycle rclcpp std_msgs dart_msgs std_srvs std_msgs lifecycle_msgs rclcpp_lifecycle)
# Add node_dart_app target
add_executable(node_dart_app src/node_dart_app.cpp src/get_ip.cpp)
ament_target_dependencies(node_dart_app rclcpp_lifecycle rclcpp std_msgs dart_msgs std_srvs sensor_msgs cv_bridge)

# Add Lvgl OpenCV
target_include_directories(node_dart_app PRIVATE
    ${PROJECT_SOURCE_DIR}
    include
    thirdparty/dart-ui/generated thirdparty/dart-ui/custom thirdparty/dart-ui/generated/guider_customer_fonts thirdparty/dart-ui/generated/guider_fonts thirdparty/dart-ui/generated/images
    ${OpenCV_INCLUDE_DIRS}
    thirdparty/json/include
)

target_include_directories(lvgl PUBLIC thirdparty)

# # 递归添加子目录${PROJECT_SOURCE_DIR}/generated/*
FILE(GLOB_RECURSE LV_SOURCES thirdparty/dart-ui/custom/*.c thirdparty/dart-ui/custom/*.cpp
    thirdparty/dart-ui/generated/*.c thirdparty/dart-ui/generated/*.cpp
    thirdparty/dart-ui/generated/guider_customer_fonts/*.c thirdparty/dart-ui/generated/guider_customer_fonts/*.cpp
    thirdparty/dart-ui/generated/guider_fonts/*.c thirdparty/dart-ui/generated/guider_fonts/*.cpp
    thirdparty/dart-ui/generated/images/*.c thirdparty/dart-ui/generated/images/*.cpp
)
target_sources(node_dart_app PRIVATE ${LV_SOURCES})

# LibDRM & SDL2
include(${CMAKE_CURRENT_LIST_DIR}/thirdparty/lvgl/tests/FindLibDRM.cmake)
include_directories(${Libdrm_INCLUDE_DIRS})
target_include_directories(lvgl PUBLIC ${Libdrm_INCLUDE_DIRS})

# find_package(SDL2)
# find_package(SDL2_image)
# include_directories(${SDL2_INCLUDE_DIRS} ${SDL2_IMAGE_INCLUDE_DIRS})

# target_link_libraries(node_dart_app lvgl lvgl::thorvg ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARIES} ${Libdrm_LIBRARIES} m pthread)
target_link_libraries(node_dart_app lvgl lvgl::thorvg ${Libdrm_LIBRARIES} nlohmann_json::nlohmann_json)

# # Add node_dart_config target
# add_executable(node_dart_config src/node_dart_config.cpp src/dart_config.cpp)
# target_include_directories(node_dart_config PRIVATE include)

# # yaml lib
# target_link_libraries(node_dart_config yaml-cpp ${GSL_LIBRARIES})

# # target_link_libraries(node_dart_param_gateway yaml-cpp)
# ament_target_dependencies(node_dart_config rclcpp std_msgs dart_msgs std_srvs)

# # Set the path to the YAML file relative to the ROS 2 install directory
# # set(YAML_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/")
# set(YAML_PATH "${CMAKE_CURRENT_LIST_DIR}/config/")

# # Pass the YAML path as a definition with double quotes
# add_definitions(-DYAML_PATH=\"${YAML_PATH}\")

# # Dart Calibration Config
# add_executable(dart_calibration src/dart_calibration.cpp src/dart_algorithm.cpp)
# target_link_libraries(dart_calibration yaml-cpp ${GSL_LIBRARIES} ${GSL_CBLAS_LIBRARIES})

install(TARGETS
    node_dart_app
    node_dart_param_gateway
    # node_dart_config
    node_dart_launcher_lifecycle
    # dart_calibration
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_SHARED_LIBS)
#     # 提取所有的.so，不要保留/lib路径
#     install(DIRECTORY
#         lvgl/lib/
#         DESTINATION lib
#         FILES_MATCHING PATTERN "*.so"
#     )

#     install(DIRECTORY
#         libsockcanpp/lib/
#         DESTINATION lib
#         FILES_MATCHING PATTERN "*.so"
#     )

#     install(DIRECTORY
#         src/generated/lib/
#         DESTINATION lib
#         FILES_MATCHING PATTERN "*.so"
#     )
# endif()

ament_package()

# Ensure compile_commands.json is generated
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
