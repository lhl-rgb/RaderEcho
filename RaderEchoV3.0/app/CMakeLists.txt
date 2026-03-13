cmake_minimum_required(VERSION 3.16)

# 应用程序 CMakeLists.txt
project(RadarApp LANGUAGES CXX)

add_executable(RadarApp main.cpp)

# 链接核心库
target_link_libraries(RadarApp PRIVATE radar_core)

# 设置输出目录
set_target_properties(RadarApp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
