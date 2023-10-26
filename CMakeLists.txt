cmake_minimum_required(VERSION 3.5)
project(iridescence)

option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)
option(BUILD_EXT_TESTS "Build test optional libraries" OFF)

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

# find_package(PCL)
find_package(Boost REQUIRED filesystem program_options)
find_package(Eigen3 REQUIRED)
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(assimp QUIET)

###########
## Build ##
###########

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)
add_definitions(-DDATA_PATH_GUESS="${CMAKE_SOURCE_DIR}/data")

add_library(gl_imgui SHARED
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
  thirdparty/implot/implot_items.cpp
  thirdparty/imgui/backends/imgui_impl_glfw.cpp
  thirdparty/imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(gl_imgui PUBLIC
  include
  thirdparty/imgui
  thirdparty/imgui/backends
  thirdparty/ImGuizmo
  thirdparty/implot
  thirdparty/portable-file-dialogs
  thirdparty/gl3w
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

add_library(iridescence SHARED
  src/glk/path_std.cpp
  src/glk/mesh.cpp
  src/glk/mesh_model.cpp
  src/glk/lines.cpp
  src/glk/thin_lines.cpp
  src/glk/trajectory.cpp
  src/glk/gridmap.cpp
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
  src/glk/io/png_io.cpp
  src/glk/io/jpeg_io.cpp
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
  src/guik/camera/projection_control.cpp
  src/guik/camera/basic_projection_control.cpp
  src/guik/viewer/light_viewer.cpp
  src/guik/viewer/light_viewer_context.cpp
  src/guik/viewer/light_viewer_context_util.cpp
  src/guik/viewer/viewer_ui.cpp
  src/guik/viewer/info_window.cpp
  src/guik/viewer/anonymous.cpp
  ${EXTRA_SOURCE}
)
target_include_directories(iridescence PUBLIC
  include
  thirdparty/imgui
  thirdparty/imgui/examples
  thirdparty/ImGuizmo
  thirdparty/portable-file-dialogs
  thirdparty/gl3w
  ${PNG_INCLUDE_DIRS}
  ${JPEG_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(iridescence
  gl_imgui
  pthread
  ${PNG_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${Boost_LIBRARIES}
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

  file(GLOB test_sources "src/test/*.cpp")
  foreach(test_src IN LISTS test_sources)
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name}
      ${test_src}
    )
    target_link_libraries(${test_name}
      fmt
      spdlog
      iridescence
    )
  endforeach()
endif()

# install
file(COPY data DESTINATION ${CMAKE_BINARY_DIR})
install(TARGETS gl_imgui iridescence DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/iridescence)
install(DIRECTORY thirdparty/gl3w/GL/ DESTINATION include/iridescence/GL)
install(FILES thirdparty/imgui/imgui.h thirdparty/imgui/imconfig.h thirdparty/imgui/imgui_internal.h thirdparty/imgui/imstb_textedit.h DESTINATION include/iridescence)
install(FILES thirdparty/implot/implot.h DESTINATION include/iridescence)
install(FILES thirdparty/portable-file-dialogs/portable-file-dialogs.h DESTINATION include/iridescence)
install(FILES thirdparty/ImGuizmo/ImGuizmo.h DESTINATION include/iridescence)
install(DIRECTORY data/ DESTINATION share/iridescence/data)
