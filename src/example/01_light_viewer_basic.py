#!/usr/bin/python3
import numpy
from scipy.spatial.transform import Rotation

from pyridescence import *

# Create a viewer instance (global singleton)
viewer = guik.LightViewer.instance()

angle = 0.0

# Define a callback for UI rendering
def ui_callback():
  # In the callback, you can call ImGui commands to create your UI.
  # Here, we use "DragFloat" and "Button" to create a simple UI.

  global angle
  _, angle = imgui.drag_float('angle', angle, 0.01)

  if imgui.button('close'):
    viewer.close()

# Register a callback for UI rendering
viewer.register_ui_callback('ui', ui_callback)

# Spin the viewer until it gets closed
while viewer.spin_once():
  # Objects to be rendered are called "drawables" and managed with unique names.
  # Here, solid and wire spheres are registered to the viewer respectively with the "Rainbow" and "FlatColor" coloring schemes.
  # The "Rainbow" coloring scheme encodes the height of each fragment using the turbo colormap by default.
  transform = numpy.identity(4)
  transform[:3, :3] = Rotation.from_rotvec([0.0, 0.0, angle]).as_matrix()
  viewer.update_drawable('sphere', glk.primitives.sphere(), guik.Rainbow(transform))
  viewer.update_drawable('wire_sphere', glk.primitives.wire_sphere(), guik.FlatColor(0.1, 0.7, 1.0, 1.0, transform))
