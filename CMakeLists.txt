cmake_minimum_required(VERSION 2.8.3)
project(gl_test_field)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(GLM REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

###########
## Build ##
###########

include_directories(
  include
  thirdparty/imgui
  thirdparty/imgui/examples
  thirdparty/portable-file-dialogs
  ${GLM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLEW)

###########
## Build ##
###########

add_library(imgui
  thirdparty/imgui/imgui.cpp
  thirdparty/imgui/imgui_demo.cpp
  thirdparty/imgui/imgui_draw.cpp
  thirdparty/imgui/imgui_widgets.cpp
  thirdparty/imgui/examples/imgui_impl_glfw.cpp
  thirdparty/imgui/examples/imgui_impl_opengl3.cpp
)

add_executable(gl_test_field
  src/gl_test_field.cpp
  src/glk/primitives/primitives.cpp
)
target_link_libraries(gl_test_field
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  glfw
  imgui
)

file(COPY data DESTINATION .)