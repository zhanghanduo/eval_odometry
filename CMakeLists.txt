cmake_minimum_required(VERSION 3.7)
project(eval_odometry)

set(CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories(
        include
)

set(SOURCE_FILES src/evaluate.cpp src/matrix.cpp)
add_executable(eval_odometry ${SOURCE_FILES})