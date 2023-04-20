cmake_minimum_required(VERSION 3.24)
project(Listener)
set(CMAKE_CXX_STANDARD 17)
find_package(Threads)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
find_package(Pangolin REQUIRED)
SET(OpenCV_DIR "/home/moi/APP/include/opencv/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
set(ZeroMQ_DIR /home/moi/APP/include/zmq/lib/cmake/ZeroMQ)
find_package(ZeroMQ REQUIRED)
message(STATUS "ZeroMQ_VERSION: ${ZeroMQ_VERSION}")
message(STATUS "ZeroMQ_INCLUDE_DIR: ${ZeroMQ_INCLUDE_DIR}")
message(STATUS "ZeroMQ_LIBRARY: ${ZeroMQ_LIBRARY}")


add_executable(Listener
        main.cpp
        vis.cpp
        dispatch.cpp
        ConvertImage.cpp
        )
target_include_directories(Listener INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ZeroMQ_INCLUDE_DIR}
        )
target_link_libraries(Listener
        ${OpenCV_LIBRARIES}
        ${ZeroMQ_LIBRARY}
        ${Pangolin_LIBRARIES})
