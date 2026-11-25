#!/usr/bin/python3
import numpy
from pyridescence import *

viewer = guik.LightViewer.instance()

# main menu example
show_demo_window = False


def main_menu_ui():
  global show_demo_window
  imgui.begin_main_menu_bar()
  if imgui.begin_menu('File'):
    if imgui.menu_item('Quit'):
      viewer.close()
    imgui.end_menu()

  if imgui.menu_item('Demo'):
    show_demo_window = not show_demo_window

  imgui.end_main_menu_bar()

  if show_demo_window:
    # see imgui_demo.cpp to learn how to use imgui
    imgui.show_demo_window()


viewer.register_ui_callback('main_menu_ui', main_menu_ui)

# UI callback can be unregistered by giving None
# viewer.register_ui_callback('main_menu_ui', None)

# control model matrix with guik.ModelControl (gizmo)
model_control = guik.ModelControl('model control window')


def model_control_ui():
  imgui.set_next_window_pos([50, 50], imgui.Cond_FirstUseEver)
  model_control.draw_ui()
  # Note: gizmo must not be drawn twice on a rendering frame
  model_control.draw_gizmo()
  viewer.update_drawable('coord', glk.primitives.coordinate_system(), guik.VertexColor(model_control.model_matrix()))


viewer.register_ui_callback('model_control_ui', model_control_ui)

# control model matrix with your own GUI
my_transform = numpy.identity(4, dtype=numpy.float32)
my_transform[:3, 3] = [1.0, 1.0, 1.0]


def my_model_control_ui():
  imgui.set_next_window_pos([50, 250], imgui.Cond_FirstUseEver)
  imgui.begin('my model control window', True, imgui.WindowFlags_AlwaysAutoResize)
  if imgui.button('+X'):
    my_transform[0, 3] += 1.0
  imgui.same_line()
  if imgui.button('+Y'):
    my_transform[1, 3] += 1.0
  imgui.same_line()
  if imgui.button('+Z'):
    my_transform[2, 3] += 1.0
  imgui.end()

  viewer.update_drawable('my_coord', glk.primitives.coordinate_system(), guik.VertexColor(my_transform))


viewer.register_ui_callback('my_model_control_ui', my_model_control_ui)

# create sub viewer
sub_viewer = viewer.sub_viewer('sub viewer', (512, 512))
sub_viewer.set_pos([50, 450], imgui.Cond_FirstUseEver)
sub_viewer.update_drawable('icosahedron', glk.primitives.wire_icosahedron(), guik.Rainbow())

# Note: The C++ version also shows low-level rendering with guik::GLCanvas, which is not available in the Python API.

viewer.spin()
