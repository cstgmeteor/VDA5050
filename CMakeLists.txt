cmake_minimum_required(VERSION 3.10)  # 最低CMake版本
project(VDA5050 LANGUAGES CXX)       # 项目名称和语言

set(CMAKE_CXX_STANDARD 17)          # 设置C++标准为C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 强制要求标准支持

# 添加头文件路径（include目录）
include_directories(include)

# 自动收集源文件（src目录下所有.cpp文件）
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

# 生成可执行文件（根据实际需求命名）
add_executable(VDA5050_App ${SRC_FILES})

target_link_libraries(VDA5050_App 
    PRIVATE 
    pthread 
    mosquitto
)