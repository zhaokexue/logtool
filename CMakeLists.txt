cmake_minimum_required(VERSION 3.15)
project(logtool CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(logtool
  src/main.cpp
)

target_include_directories(logtool PRIVATE include)

add_executable(logdump
  src/logdump.cpp
)

target_include_directories(logdump PRIVATE include)

add_executable(log_web_viewer
  src/log_web_viewer.cpp
)

target_include_directories(log_web_viewer PRIVATE include)