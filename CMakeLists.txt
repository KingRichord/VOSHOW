cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)
message("-- CMAKE_VERSION:   ${CMAKE_VERSION}")

cmake_policy(SET CMP0058 NEW)
cmake_policy(SET CMP0072 NEW)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


project(lightvis VERSION 0.1.0 LANGUAGES CXX)

include(SuperBuildDepends)
include(GNUInstallDirs)

superbuild_depend(eigen)
superbuild_depend(sophus)
superbuild_depend(glfw)
superbuild_depend(glbinding)
superbuild_depend(nuklear)
find_package(OpenCV REQUIRED)
add_library(lightvis
  ${CMAKE_CURRENT_SOURCE_DIR}/include/image.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/lightvis.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/shader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/image.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lightvis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lightvis_font_roboto.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lightvis_font_roboto.cpp
)

target_include_directories(lightvis
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lightvis
  PUBLIC
    depends::glbinding
    depends::sophus
    Eigen3::Eigen
  PRIVATE
    depends::glfw
    depends::nuklear
    ${OpenCV_LIBRARIES}
)