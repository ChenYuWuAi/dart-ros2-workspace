if(EXAMPLE_USE_EXTERNAL_JSON)
    find_package(nlohmann_json 3.12.0 REQUIRED)
else()
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    add_subdirectory(json)
endif()

# 禁用ccache
set(CMAKE_CXX_COMPILER_LAUNCHER "")
set(LV_CONF_BUILD_DISABLE_EXAMPLES ON)
set(LV_CONF_BUILD_DISABLE_DEMOS ON)

add_subdirectory(lvgl)
# 此时修改BUILD_SHARED_LIBS的值
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

target_include_directories(lvgl PUBLIC ${CMAKE_SOURCE_DIR})