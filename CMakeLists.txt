cmake_minimum_required(VERSION 2.8.8)
project(imageICP)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        ${PCL}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV})

add_library(${PROJECT_NAME} SHARED
        include/imageICP.cpp
        include/imageICP.h
        include/points.cpp
        include/points.h)

target_link_libraries(${PROJECT_NAME}
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(test main.cpp )
target_link_libraries(test ${PROJECT_NAME})