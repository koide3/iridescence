cmake_minimum_required(VERSION 2.8.3)
project(iridescence)

option(BUILD_PYTHON_BINDINGS "Build python bindings" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)

find_package(PCL)
find_package(PNG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

find_package(catkin COMPONENTS roslib)

set(built_in_ros FALSE)
if(catkin_FOUND AND NOT "${CATKIN_DEVEL_PREFIX}" MATCHES "${CMAKE_CURRENT_SOURCE_DIR}.*")
  set(built_in_ros TRUE)
endif()
message(STATUS "built_in_ros:${built_in_ros}")

###################################
## catkin specific configuration ##
###################################
if(built_in_ros)
  # check if the package is built in a catkin workspace
  catkin_package(
    INCLUDE_DIRS include thirdparty/imgui thirdparty/ImGuizmo thirdparty/gl3w thirdparty/portable-file-dialogs
    LIBRARIES dl glfw gl_imgui iridescence ${catkin_LIBRARIES}
    #  CATKIN_DEPENDS roscpp rospy
    DEPENDS roslib
  )
endif()

###########
## Build ##
###########

include_directories(
  include
  thirdparty/imgui
  thirdparty/imgui/examples
  thirdparty/ImGuizmo
  thirdparty/portable-file-dialogs
  thirdparty/gl3w
  ${PNG_INCLUDE_DIRS}
  ${GLM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

###########
## Build ##
###########

add_library(gl_imgui SHARED
  thirdparty/gl3w/gl3w.c
  thirdparty/imgui/imgui.cpp
  thirdparty/imgui/imgui_demo.cpp
  thirdparty/imgui/imgui_draw.cpp
  thirdparty/imgui/imgui_widgets.cpp
  thirdparty/ImGuizmo/ImCurveEdit.cpp
  thirdparty/ImGuizmo/ImGradient.cpp
  thirdparty/ImGuizmo/ImGuizmo.cpp
  thirdparty/ImGuizmo/ImSequencer.cpp
  thirdparty/imgui/examples/imgui_impl_glfw.cpp
  thirdparty/imgui/examples/imgui_impl_opengl3.cpp
)
target_link_libraries(gl_imgui
  ${OPENGL_LIBRARIES}
  glfw
  dl
)

set(path_src path_std.cpp)
if(built_in_ros)
  set(path_src path_ros.cpp)
endif()

add_library(iridescence SHARED
  src/glk/${path_src}
  src/glk/mesh.cpp
  src/glk/lines.cpp
  src/glk/thin_lines.cpp
  src/glk/trajectory.cpp
  src/glk/gridmap.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/pointnormals_buffer.cpp
  src/glk/point_correspondences.cpp
  src/glk/normal_distributions.cpp
  src/glk/colormap.cpp
  src/glk/texture.cpp
  src/glk/glsl_shader.cpp
  src/glk/frame_buffer.cpp
  src/glk/query.cpp
  src/glk/transform_feedback.cpp
  src/glk/texture_renderer.cpp
  src/glk/primitives/primitives.cpp
  src/glk/io/ply_io.cpp
  src/glk/io/png_io.cpp
  src/glk/effects/plain_rendering.cpp
  src/glk/effects/screen_space_splatting.cpp
  src/glk/effects/screen_space_lighting.cpp
  src/glk/effects/screen_space_ambient_occlusion.cpp
  src/glk/effects/screen_space_attribute_estimation.cpp
  src/glk/effects/naive_screen_space_ambient_occlusion.cpp
  src/guik/gl_canvas.cpp
  src/guik/model_control.cpp
  src/guik/imgui_application.cpp
  src/guik/recent_files.cpp
  src/guik/camera/orbit_camera_control_xy.cpp
  src/guik/camera/orbit_camera_control_xz.cpp
  src/guik/camera/topdown_camera_control.cpp
  src/guik/camera/projection_control.cpp
  src/guik/viewer/light_viewer.cpp
  src/guik/viewer/light_viewer_context.cpp
  src/guik/viewer/viewer_ui.cpp
  src/guik/viewer/info_window.cpp
  src/guik/viewer/anonymous.cpp
)
target_link_libraries(iridescence
  gl_imgui
  pthread
  ${PCL_LIBRARIES}
  ${PNG_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
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
  target_link_libraries(pyridescence
    iridescence
  )
  file(COPY src/example/light_viewer.py DESTINATION .)
  # target_compile_definitions(pyridescence PRIVATE VERSION_INFO=${IRIDESCENCE_VERSION_INFO})
endif()

# copy shader files
file(COPY data DESTINATION .)

# build examples
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
