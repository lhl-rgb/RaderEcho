cmake_minimum_required(VERSION 3.16)

# 项目名称
project(RaderEchoV3 VERSION 3.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(MSVC)
    add_compile_options(/W4)
    # Debug 模式使用 /Od 禁用优化，Release 模式使用 /O2
    add_compile_options($<$<CONFIG:Debug>:/Od>)
    add_compile_options($<$<CONFIG:Release>:/O2>)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

# 添加核心库
add_subdirectory(core)

# 添加应用程序
add_subdirectory(app)
