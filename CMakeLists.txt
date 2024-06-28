cmake_minimum_required(VERSION 3.5)
project(iridescence VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_EXT_TESTS "Build test optional libraries" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CMAKE_POSITION_INDEPENDENT_CODE "Generate position-independent code" ON)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CMAKE_POSITION_INDEPENDENT_CODE "Generate position-independent code" ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

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

find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PNG)
find_package(JPEG)
find_package(assimp QUIET)

include(GNUInstallDirs)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/iridescence CACHE PATH "Location of header files" )

###########
## Build ##
###########

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)
add_definitions(-DDATA_PATH_GUESS="${CMAKE_SOURCE_DIR}/data")

add_library(gl_imgui
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
)
target_include_directories(gl_imgui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ImGuizmo>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/implot>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/portable-file-dialogs>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gl3w>
)
target_link_libraries(gl_imgui
  ${OPENGL_LIBRARIES}
  glfw
  dl
)

if(${assimp_FOUND})
  list(APPEND EXTRA_LIBRARIES assimp)
  list(APPEND EXTRA_SOURCE src/glk/io/mesh_io.cpp)
endif()

if(${PNG_FOUND})
  list(APPEND EXTRA_LIBRARIES PNG::PNG)
  list(APPEND EXTRA_SOURCE src/glk/io/png_io.cpp)
endif()

if(${JPEG_FOUND})
  list(APPEND EXTRA_LIBRARIES JPEG::JPEG)
  list(APPEND EXTRA_SOURCE src/glk/io/jpeg_io.cpp)
endif()

add_library(iridescence
  src/glk/path_std.cpp
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
  $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/examples>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ImGuizmo>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/portable-file-dialogs>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/gl3w>
  ${GLM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(iridescence
  gl_imgui
  pthread
  ${EXTRA_LIBRARIES}
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
  file(COPY src/example/light_viewer.py DESTINATION .)
  # target_compile_definitions(pyridescence PRIVATE VERSION_INFO=${IRIDESCENCE_VERSION_INFO})
endif()

# build examples
if(BUILD_EXAMPLES)
  file(GLOB example_sources "src/example/*.cpp")

  foreach(example_src IN LISTS example_sources)
    get_filename_component(example_name ${example_src} NAME_WE)

    if(${example_name} STREQUAL "ext_light_viewer_pointcloud" OR ${example_name} STREQUAL "ext_light_viewer_kitti")
      if(NOT PCL_FOUND)
        continue()
      endif()
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

install(TARGETS gl_imgui iridescence
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

endif()

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
endif()