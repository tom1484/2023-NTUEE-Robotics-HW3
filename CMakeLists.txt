cmake_minimum_required(VERSION 3.10)
project("Robotics-HW3")

set(OpenCV_DIR ~/opencv-4.8.0/build)
find_package(OpenCV REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(hw3_b hw3_b.cpp)
target_link_libraries(hw3_b ${OpenCV_LIBS})

add_executable(hw3_a hw3_a.cpp)
target_link_libraries(hw3_a ${OpenCV_LIBS})
