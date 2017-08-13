cmake_minimum_required(VERSION 2.9)
project(eval_odometry)

set(CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(gflags REQUIRED)
#find_package (glog REQUIRED)

include_directories(
        include
)

set(SOURCE_FILES1 src/evaluate_custom.cpp src/matrix.cpp)
set(SOURCE_FILES2 src/evaluate_kitti.cpp src/matrix.cpp)
set(SOURCE_FILES3 src/evaluate_kitti_time.cpp src/matrix.cpp)

add_executable(eval_odometry ${SOURCE_FILES1})
target_link_libraries(eval_odometry gflags)

add_executable(eval_kitti ${SOURCE_FILES2})

add_executable(eval_kitti_time ${SOURCE_FILES3})
target_link_libraries(eval_kitti_time gflags)