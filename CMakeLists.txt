cmake_minimum_required(VERSION 3.10)
project(can_motor_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# 添加 include 目录
include_directories(include)

# 生成可执行文件
add_executable(can_control
    src/motor_cfg.cpp
    main.cpp
)

# 链接 socketcan 所需库
target_link_libraries(can_control
    pthread
)
