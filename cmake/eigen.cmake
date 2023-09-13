if(NOT TARGET depends::eigen)
    FetchContent_Declare(
            depends-eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG        3.4.0
    )
    FetchContent_GetProperties(depends-eigen)
    if(NOT depends-eigen_POPULATED)
        message(STATUS "Fetching eigen sources")
        FetchContent_Populate(depends-eigen)
        # 注意：要小写depends-sophus  参考：https://cmake.org/cmake/help/latest/module/FetchContent.html#command:fetchcontent_populate
        message("-- depends-eigen SRC; ${depends-eigen_SOURCE_DIR}")
        message("-- depends-eigen BIN: ${depends-eigen_BINARY_DIR}")
        message(STATUS "Fetching eigen sources - done")
    endif()

    add_subdirectory(${depends-eigen_SOURCE_DIR} ${depends-eigen_BINARY_DIR})
    add_library(depends::eigen INTERFACE IMPORTED GLOBAL)
    target_link_libraries(depends::eigen INTERFACE Eigen3::Eigen)
    set(depends-eigen-source-dir ${depends-eigen_SOURCE_DIR}} CACHE INTERNAL "" FORCE)
    set(depends-eigen-binary-dir ${depends-eigen_BINARY_DIR} CACHE INTERNAL "" FORCE)
    mark_as_advanced(depends-eigen-source-dir)
    mark_as_advanced(depends-eigen-binary-dir)
endif()