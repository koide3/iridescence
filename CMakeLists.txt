cmake_minimum_required(VERSION 3.16)
project(iridescence VERSION 1.0.0 LANGUAGES C CXX)

option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_EXT_TESTS "Build test optional libraries" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CMAKE_POSITION_INDEPENDENT_CODE "Generate position-independent code" ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS "ON" CACHE STRING "Export global symbols" FORCE)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(OpenGL_GL_PREFERENCE GLVND)

if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

find_package(OpenGL REQUIRED)

find_package(glm)
find_package(glfw3)
find_package(Eigen3)
find_package(PNG)
find_package(JPEG)
find_package(assimp QUIET)


if(NOT glm_FOUND)
  message(STATUS "System GLM not found. Download GLM 1.0.1.")
  include(FetchContent)
  FetchContent_Populate(
    glm
    URL https://github.com/g-truc/glm/archive/refs/tags/1.0.1.tar.gz
  )
  add_library(glm::glm INTERFACE IMPORTED GLOBAL)
  set_target_properties(glm::glm PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${glm_SOURCE_DIR}"
  )
endif()

if(NOT glfw3_FOUND)
  message(STATUS "System GLFW3 not found. Download GLFW 3.4.")
  include(FetchContent)
  FetchContent_Populate(
    glfw3
    URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
  )
  add_subdirectory("${glfw3_SOURCE_DIR}")
  # set_target_properties(glm PROPERTIES
  #   INTERFACE_INCLUDE_DIRECTORIES "${glm_SOURCE_DIR}"
  # )
endif()

if(NOT Eigen3_FOUND)
  message(STATUS "System Eigen not found. Download Eigen 3.4.0.")
  include(FetchContent)
  FetchContent_Populate(
    Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  )
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen3_SOURCE_DIR}"
  )
endif()

include(GNUInstallDirs)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/iridescence CACHE PATH "Location of header files" )

###########
## Build ##
###########

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)
add_definitions(-DDATA_INSTALL_PATH="${CMAKE_INSTALL_FULL_DATADIR}/iridescence/data")
add_definitions(-DDATA_PATH_GUESS="${CMAKE_SOURCE_DIR}/data")

if(MSVC)
  add_compile_definitions(NOMINMAX)
  add_compile_definitions(_USE_MATH_DEFINES)
  add_compile_options(/wd4018 /wd4244 /wd4267 /wd4305 /wd4312)
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
  # Enable parallel compilation
  add_compile_options(/MP)
else()
  list(APPEND EXTRA_LIBRARIES dl)
  list(APPEND EXTRA_LIBRARIES pthread)
endif()

if(${assimp_FOUND})
  list(APPEND EXTRA_LIBRARIES assimp)
  list(APPEND EXTRA_SOURCE src/glk/io/mesh_io.cpp)
endif()

if(${PNG_FOUND})
  list(APPEND EXTRA_LIBRARIES PNG::PNG)
  list(APPEND EXTRA_SOURCE src/glk/io/png_io.cpp)
else()
  list(APPEND EXTRA_SOURCE src/glk/io/png_io_dummy.cpp)
endif()

if(${JPEG_FOUND})
  list(APPEND EXTRA_LIBRARIES JPEG::JPEG)
  list(APPEND EXTRA_SOURCE src/glk/io/jpeg_io.cpp)
else()
  list(APPEND EXTRA_SOURCE src/glk/io/jpeg_io_dummy.cpp)
endif()

add_library(iridescence
  # GL and IMGUI
  thirdparty/gl3w/gl3w.cpp
  thirdparty/imgui/imgui.cpp
  thirdparty/imgui/imgui_demo.cpp
  thirdparty/imgui/imgui_draw.cpp
  thirdparty/imgui/imgui_tables.cpp
  thirdparty/imgui/imgui_widgets.cpp
  thirdparty/ImGuizmo/ImCurveEdit.cpp
  thirdparty/ImGuizmo/ImGradient.cpp
  thirdparty/ImGuizmo/ImGuizmo.cpp
  thirdparty/ImGuizmo/ImSequencer.cpp
  thirdparty/implot/implot.cpp
  thirdparty/implot/implot_demo.cpp
  thirdparty/implot/implot_items.cpp
  thirdparty/imgui/backends/imgui_impl_glfw.cpp
  thirdparty/imgui/backends/imgui_impl_opengl3.cpp
  # glk
  src/glk/path_std.cpp
  src/glk/hash.cpp
  src/glk/mesh.cpp
  src/glk/mesh_model.cpp
  src/glk/lines.cpp
  src/glk/thin_lines.cpp
  src/glk/trajectory.cpp
  src/glk/gridmap.cpp
  src/glk/drawable_container.cpp
  src/glk/async_buffer_copy.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/pointnormals_buffer.cpp
  src/glk/point_correspondences.cpp
  src/glk/normal_distributions.cpp
  src/glk/indexed_pointcloud_buffer.cpp
  src/glk/splatting.cpp
  src/glk/colormap.cpp
  src/glk/html_colors.cpp
  src/glk/texture.cpp
  src/glk/glsl_shader.cpp
  src/glk/frame_buffer.cpp
  src/glk/pixel_buffer.cpp
  src/glk/shader_storage_buffer.cpp
  src/glk/query.cpp
  src/glk/debug_output.cpp
  src/glk/transform_feedback.cpp
  src/glk/texture_renderer.cpp
  src/glk/primitives/primitives.cpp
  src/glk/io/ascii_io.cpp
  src/glk/io/ply_io.cpp
  src/glk/io/image_io.cpp
  src/glk/effects/plain_rendering.cpp
  src/glk/effects/screen_space_splatting.cpp
  src/glk/effects/screen_space_lighting.cpp
  src/glk/effects/screen_space_ambient_occlusion.cpp
  src/glk/effects/screen_space_attribute_estimation.cpp
  src/glk/effects/naive_screen_space_ambient_occlusion.cpp
  # guik
  src/guik/gl_canvas.cpp
  src/guik/model_control.cpp
  src/guik/hovered_drawings.cpp
  src/guik/hovered_primitives.cpp
  src/guik/imgui_application.cpp
  src/guik/screen_capture.cpp
  src/guik/recent_files.cpp
  src/guik/camera/camera_control.cpp
  src/guik/camera/orbit_camera_control_xy.cpp
  src/guik/camera/orbit_camera_control_xz.cpp
  src/guik/camera/topdown_camera_control.cpp
  src/guik/camera/arcball_camera_control.cpp
  src/guik/camera/static_camera_control.cpp
  src/guik/camera/fps_camera_control.cpp
  src/guik/camera/projection_control.cpp
  src/guik/camera/basic_projection_control.cpp
  src/guik/viewer/plot_data.cpp
  src/guik/viewer/shader_setting.cpp
  src/guik/viewer/light_viewer.cpp
  src/guik/viewer/light_viewer_context.cpp
  src/guik/viewer/light_viewer_context_util.cpp
  src/guik/viewer/async_light_viewer.cpp
  src/guik/viewer/async_light_viewer_context.cpp
  src/guik/viewer/viewer_ui.cpp
  src/guik/viewer/info_window.cpp
  src/guik/viewer/anonymous.cpp
  ${EXTRA_SOURCE}
)
target_include_directories(iridescence PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/examples>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ImGuizmo>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/implot>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/portable-file-dialogs>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gl3w>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/rapidhash>
  $<INSTALL_INTERFACE:include/iridescence>
)
target_link_libraries(iridescence PUBLIC
  Eigen3::Eigen
  OpenGL::GL
  glfw
  ${EXTRA_LIBRARIES}
)
target_link_libraries(iridescence PRIVATE
  glm::glm
)

target_compile_definitions(iridescence PRIVATE
  GL3W_EXPORTS
  GLK_EXPORTS
)
set_target_properties(iridescence PROPERTIES
  VERSION   ${PROJECT_VERSION}
  SOVERSION 1
)

if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(thirdparty/pybind11)

  pybind11_add_module(pyridescence
    src/python/main.cpp
    src/python/glk.cpp
    src/python/guik.cpp
    src/python/pfd.cpp
    src/python/imgui.cpp
  )
  target_link_libraries(pyridescence PRIVATE
    iridescence
  )
endif()

# build examples
if(BUILD_EXAMPLES)
  file(GLOB example_sources "src/example/*.cpp")

  foreach(example_src IN LISTS example_sources)
    get_filename_component(example_name ${example_src} NAME_WE)

    if(${example_name} STREQUAL "ext_light_viewer_pointcloud" OR ${example_name} STREQUAL "ext_light_viewer_kitti")
      find_package(PCL CONFIG)
      if(NOT PCL_FOUND)
        continue()
      endif()
      target_link_libraries(iridescence PUBLIC
        ${PCL_LIBRARIES}
      )
      target_include_directories(iridescence PUBLIC
        ${PCL_INCLUDE_DIRS}
      )

    endif()

    add_executable(${example_name}
      ${example_src}
    )
    target_link_libraries(${example_name}
      iridescence
    )
  endforeach()
endif()

if(BUILD_EXT_TESTS)
  find_package(fmt REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(assimp REQUIRED)
  find_package(GTest REQUIRED)

  enable_testing()

  file(GLOB test_sources "src/test/*.cpp")
  foreach(test_src IN LISTS test_sources)
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name}
      ${test_src}
    )
    target_link_libraries(${test_name}
      iridescence
      fmt::fmt
      spdlog::spdlog
      GTest::gtest_main
    )

    gtest_discover_tests(${test_name} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  endforeach()
endif()

#############
## Install ##
#############

install(DIRECTORY include/ DESTINATION ${INCLUDE_INSTALL_DIR})
install(DIRECTORY thirdparty/gl3w/GL/ DESTINATION ${INCLUDE_INSTALL_DIR}/GL)
install(FILES thirdparty/imgui/imgui.h thirdparty/imgui/imconfig.h thirdparty/imgui/imgui_internal.h thirdparty/imgui/imstb_textedit.h DESTINATION  ${INCLUDE_INSTALL_DIR})
install(FILES thirdparty/implot/implot.h thirdparty/implot/implot_internal.h DESTINATION  ${INCLUDE_INSTALL_DIR})
install(FILES thirdparty/portable-file-dialogs/portable-file-dialogs.h DESTINATION  ${INCLUDE_INSTALL_DIR})
install(FILES thirdparty/ImGuizmo/ImGuizmo.h DESTINATION  ${INCLUDE_INSTALL_DIR})
install(DIRECTORY data/ DESTINATION ${CMAKE_INSTALL_DATADIR}/iridescence/data)

install(TARGETS iridescence
  EXPORT iridescence-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
set(CMAKE_CONFIG_INSTALL_DIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/iridescence"
  CACHE PATH "Install directory for CMake config files"
)
include(CMakePackageConfigHelpers)
install(EXPORT iridescence-targets
  FILE iridescence-targets.cmake
  NAMESPACE Iridescence::
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/iridescence-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/iridescence-config.cmake"
  INSTALL_DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
  PATH_VARS INCLUDE_INSTALL_DIR
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/iridescence-config-version.cmake"
  VERSION ${VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/iridescence-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/iridescence-config-version.cmake"
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
)

if(BUILD_PYTHON_BINDINGS)
  if(DEFINED SKBUILD_PROJECT_NAME)
    set(PYTHON_INSTALL_DIR .)
  elseif(NOT DEFINED PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages)
  endif()
  install(TARGETS pyridescence
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
    COMPONENT python
  )

  # install stubs in src/python/stubs
  install(DIRECTORY src/python/stubs/ DESTINATION ${PYTHON_INSTALL_DIR})
endif()

#############
## Package ##
#############

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_DEBIAN_PACKAGE_NAME "libiridescence-dev")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "k.koide <k.koide@aist.go.jp>")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libglfw3-dev")

set(GNUInstallDirs)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "3D visualization library for rapid prototyping of 3D algorithms")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VENDOR "k.koide")

include(CPack)
