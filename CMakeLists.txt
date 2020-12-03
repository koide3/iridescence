cmake_minimum_required(VERSION 2.8.3)
project(iridescent)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

find_package(PCL)
# find_package(G2O)

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
    INCLUDE_DIRS include thirdparty/imgui thirdparty/gl3w thirdparty/portable-file-dialogs
    LIBRARIES dl glfw gl_imgui iridescent ${catkin_LIBRARIES}
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

add_library(iridescent SHARED
  src/glk/${path_src}
  src/glk/mesh.cpp
  src/glk/lines.cpp
  src/glk/thin_lines.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/pointnormals_buffer.cpp
  src/glk/point_correspondences.cpp
  src/glk/colormap.cpp
  src/glk/glsl_shader.cpp
  src/glk/frame_buffer.cpp
  src/glk/texture_renderer.cpp
  src/glk/primitives/primitives.cpp
  src/glk/loaders/ply_loader.cpp
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
)
target_link_libraries(iridescent
  gl_imgui
  pthread
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

# set(ext_src_files src/glk/ext/graph/factor_graph.cpp)
# if(G2O_FOUND)
#   set(ext_src_files ${ext_src_files} src/glk/ext/graph/factor_graph_g2o.cpp)
# endif()
#
# add_library(iridescent_ext SHARED
#   ${ext_src_files}
# )
# target_link_libraries(iridescent_ext
#   iridescent
#   ${G2O_LIBRARIES}
# )

# copy shader files
file(COPY data DESTINATION .)

# build examples
file(GLOB example_sources "src/example/*.cpp")

foreach(example_src IN LISTS example_sources)
  get_filename_component(example_name ${example_src} NAME_WE)

  if(${example_name} STREQUAL "light_viewer_pointcloud" OR ${example_name} STREQUAL "light_viewer_kitti")
    if(NOT PCL_FOUND)
      continue()
    endif()
  endif()

  add_executable(${example_name}
    ${example_src}
  )
  target_link_libraries(${example_name}
    iridescent
  )
endforeach()
