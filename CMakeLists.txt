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

# 查找OpenGL库
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
add_executable(Listener
        main.cpp
        vis.cpp
        dispatch.cpp
        ConvertImage.cpp
        points.h
)
target_include_directories(Listener INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ZeroMQ_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIRS}
        )
target_link_libraries(Listener PRIVATE
        ${OpenCV_LIBRARIES}
        ${ZeroMQ_LIBRARY}
        ${Pangolin_LIBRARIES}
        OpenGL::GL glfw
        fmt
)
message(STATUS "
----------------------------------------------------------------------
   Configured on host
     host type                       = ${CMAKE_SYSTEM_NAME}
     host operating system           = ${CMAKE_SYSTEM_VERSION}
     host CPU architecture           = ${CMAKE_SYSTEM_PROCESSOR}

   General build flags:
     CC                              = ${CMAKE_C_COMPILER}
     CXX                             = ${CMAKE_CXX_COMPILER}

   Other configuration settings:
     Installation prefix            = ${CMAKE_INSTALL_PREFIX}
     BUILD_TEST                     = ${BUILD_TEST}

   library dependencies：
     Zmq include path               = ${ZeroMQ_INCLUDE_DIR}
     Zmq library                    = ${ZeroMQ_LIBRARY}

     Pangolin include path               = ${Pangolin_INCLUDE_DIR}
     Pangolin library                    = ${Pangolin_LIBRARIES}

     OpenCV include path               = ${OpenCV_INCLUDE_DIRS}
     OpenCV library                    = ${OpenCV_LIBRARIES}



     Eigen include path               = ${ZeroMQ_INCLUDE_DIR}


----------------------------------------------------------------------
")