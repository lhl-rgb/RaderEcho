cmake_minimum_required(VERSION 3.16)

# 核心库 CMakeLists.txt
project(RaderEchoV3Core LANGUAGES CXX)

set(CORE_SOURCES
    src/ClutterGenerator.cpp
    src/ClutterManager.cpp
    src/GridManager.cpp
    src/TargetManager.cpp
    src/SignalProcessor.cpp
    src/ParameterManager.cpp
    src/RadarSimulator.cpp
    src/Logger.cpp
    src/BeamCodeManager.cpp
)

set(CORE_HEADERS
    include/Types.h
    include/ClutterGenerator.h
    include/ClutterManager.h
    include/GridManager.h
    include/TargetManager.h
    include/SignalProcessor.h
    include/ParameterManager.h
    include/RadarSimulator.h
    include/Logger.h
    include/BeamCodeManager.h
)

# 创建静态库
add_library(radar_core STATIC ${CORE_SOURCES} ${CORE_HEADERS})

# 包含目录
target_include_directories(radar_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接数学库（Unix-like 系统）
if(UNIX)
    target_link_libraries(radar_core PRIVATE m)
endif()
