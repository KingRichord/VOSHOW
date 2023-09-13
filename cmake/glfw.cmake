if(NOT TARGET depends::glfw)
  FetchContent_Declare(
    depends-glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3
  )
  # 获取外部依赖项的属性
  FetchContent_GetProperties(depends-glfw)
  # 检查 depends-glfw 是否下载完成
  if(NOT depends-glfw_POPULATED)
    message(STATUS "Fetching GLFW sources")
    # 下载
    FetchContent_Populate(depends-glfw)
    # 输出代码拉取成功
    message(STATUS "Fetching GLFW sources - done")
  endif()
  # 关闭GLFW中的一些配置属性
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  # 将指定的源代码目录添加为子项目，并在构建过程中编译和链接该子项目
  add_subdirectory(${depends-glfw_SOURCE_DIR} ${depends-glfw_BINARY_DIR})
  add_library(depends::glfw INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::glfw INTERFACE glfw)
  set(depends-glfw-source-dir ${depends-glfw_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-glfw-binary-dir ${depends-glfw_BINARY_DIR} CACHE INTERNAL "" FORCE)
  # 变量标记为高级选项，当一个变量被标记为高级选项时，它将不会在图形界面中显示，
  # 并且用户无法轻易看到或修改它。这通常用于隐藏一些不常用或对用户来说不必关注的选项。
  mark_as_advanced(depends-glfw-source-dir)
  mark_as_advanced(depends-glfw-binary-dir)
endif()
