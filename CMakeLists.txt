cmake_minimum_required(VERSION 2.8.3)
project(gl_test_field)

# add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GLM REQUIRED)
find_package(OpenGL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

find_package(PCL REQUIRED)

find_package(catkin COMPONENTS roslib)

###################################
## catkin specific configuration ##
###################################
if(catkin_FOUND)
  # check if the package is built in a catkin workspace
  get_filename_component(devel_directory ${CATKIN_DEVEL_PREFIX} NAME_WE)
  if(${devel_directory} STREQUAL "devel")
    catkin_package(
      INCLUDE_DIRS include thirdparty/imgui thirdparty/gl3w thirdparty/portable-file-dialogs
      LIBRARIES dl glfw gl_imgui gl_test_field ${catkin_LIBRARIES}
      #  CATKIN_DEPENDS roscpp rospy
      DEPENDS roslib
    )
  endif()
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

add_library(gl_imgui
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
if(catkin_FOUND AND ${devel_directory} STREQUAL "devel")
  set(path_src path_ros.cpp)
endif()

add_library(gl_test_field
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
  src/guik/camera_control.cpp
  src/guik/projection_control.cpp
  src/guik/imgui_application.cpp
  src/guik/recent_files.cpp
  src/guik/viewer/light_viewer.cpp
  src/guik/viewer/light_viewer_context.cpp
)
target_link_libraries(gl_test_field
  gl_imgui
  pthread
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# copy shader files
file(COPY data DESTINATION .)

# build examples
file(GLOB example_sources "src/example/*.cpp")

foreach(example_src IN LISTS example_sources)
  get_filename_component(example_name ${example_src} NAME_WE)

  if(${example_name} STREQUAL "light_viewer_pointcloud" AND NOT PCL_FOUND)
    continue()
  endif()

  add_executable(${example_name}
    ${example_src}
  )
  target_link_libraries(${example_name}
    gl_test_field
  )
endforeach()
