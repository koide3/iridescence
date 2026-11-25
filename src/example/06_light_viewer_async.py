#!/usr/bin/python3
import time
import numpy
from pyridescence import *

# AsyncViewer creates and runs the viewer in a background thread.
# This is helpful in situations you want to keep the viewer interactive while the main thread is doing some blocking operations.
# Note that this "fire-and-forget" style usage brings some limitations of possible operations and data copy overhead.
async_viewer = guik.async_viewer()

# Because the viewer is running in another thread, calling the standard viewer functions in this main thread is unsafe.
# Thus, the below line may cause segfaults and program crashes.
# guik.viewer().update_sphere('sphere0', guik.FlatBlue())

# Use AsyncViewer interfaces for safe viewer data manipulation.
time.sleep(1.0)
async_viewer.update_wire_sphere('sphere1', guik.FlatRed().translate(0.0, 0.0, 0.0))

time.sleep(1.0)
async_viewer.update_wire_sphere('sphere2', guik.FlatGreen().translate(2.0, 0.0, 0.0))

time.sleep(1.0)
async_viewer.update_wire_sphere('sphere3', guik.FlatBlue().translate(4.0, 0.0, 0.0))

time.sleep(1.0)
ys = numpy.arange(-1.0, 1.0, 0.01)
line = numpy.stack([numpy.sin(ys * 10.0) * 0.1 + 6.0, ys, numpy.zeros_like(ys)], axis=-1).astype(numpy.float32)

# We cannot directly create glk.ThinLines here because it creates OpenGL buffers that must be done in the viewer thread
# dline = glk.ThinLines(line, line_strip=True)

# Instead, we can use glk.DelayedConstructor to make the construction in the viewer thread in lazy manner
dline = glk.DelayedConstructor(glk.ThinLines, line, line_strip=True)
async_viewer.update_drawable('line', dline, guik.FlatOrange())

# emplace_drawable is a shorthand of update_drawable + DelayedConstructor
async_viewer.emplace_drawable('line2', glk.ThinLines, line, line_strip=True, shader_setting=guik.FlatOrange().translate(1.0, 0.0, 0.0))


# Register a UI callback, which will be called in the viewer thread.
def ui_callback():
  if imgui.button('Remove red sphere'):
    # Because this callback will be called in the viewer thread,
    # it is safe to use the standard viewer interfaces.
    guik.LightViewer.instance().remove_drawable('sphere1')


async_viewer.register_ui_callback('ui_callback', ui_callback)

# `async_sub_viewer` causes a syncronization with the visualization thread.
# Do not call it frequently.
async_sub_viewer = async_viewer.async_sub_viewer('sub_viewer')
async_sub_viewer.update_wire_icosahedron('icosahedron', guik.FlatRed())

# Wait for the viewer to be closed.
# To terminate the async viewer immediately, call `guik.async_destroy` instead.
guik.async_wait()
