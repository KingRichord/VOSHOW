if(NOT TARGET depends::Sophus)
    FetchContent_Declare(
            depends-sophus
            GIT_REPOSITORY https://github.com/strasdat/Sophus.git
            GIT_TAG        main-1.x
    )
    FetchContent_GetProperties(depends-sophus)
    if(NOT depends-sophus_POPULATED)
        message(STATUS "Fetching Sophus sources")
        FetchContent_Populate(depends-sophus)
        # 注意：要小写depends-sophus  参考：https://cmake.org/cmake/help/latest/module/FetchContent.html#command:fetchcontent_populate
        message("SRC; ${depends-sophus_SOURCE_DIR}")
        message("BIN: ${depends-sophus_BINARY_DIR}")
        message(STATUS "Fetching Sophus sources - done")
    endif()
    set(SOPHUS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(SOPHUS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(SOPHUS_INSTALL OFF CACHE BOOL "" FORCE)
    set(SOPHUS_USE_BASIC_LOGGING OFF CACHE BOOL "" FORCE)

    add_subdirectory(${depends-sophus_SOURCE_DIR} ${depends-sophus_BINARY_DIR})
    add_library(depends::sophus INTERFACE IMPORTED GLOBAL)
    target_link_libraries(depends::sophus INTERFACE Sophus::Sophus)
    set(depends-sophus-source-dir ${depends-sophus_SOURCE_DIR}} CACHE INTERNAL "" FORCE)
    set(depends-sophus-binary-dir ${depends-sophus_BINARY_DIR} CACHE INTERNAL "" FORCE)
    mark_as_advanced(depends-sophus-source-dir)
    mark_as_advanced(depends-sophus-binary-dir)
endif()