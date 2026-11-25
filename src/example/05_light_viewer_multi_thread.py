#!/usr/bin/python3
import time
import threading
import numpy
from pyridescence import *


# This function will run on a background thread
def run():
  for i in range(500):
    if not guik.running():
      break

    points = numpy.random.uniform(-10.0, 10.0, (128, 3)).astype(numpy.float32)
    color = numpy.random.uniform(0.0, 1.0, 3)

    # The following code doesn't work here because GL objects must be created in the GUI thread
    # cloud_buffer = glk.PointCloudBuffer(points)

    # Request to invoke a function in the GUI thread.
    def task(i=i, points=points, color=color):
      # This function will be called just before rendering in the GUI thread.
      cloud_buffer = glk.PointCloudBuffer(points)
      viewer.update_drawable(f'cloud_{i}', cloud_buffer, guik.FlatColor(*color, 1.0).add('point_scale', 3.0))

    viewer.invoke(task)

    # `append_text` and `clear_text` are thread-safe.
    viewer.append_text(f'{i} : add points')

    time.sleep(0.1)


# Usually, `guik.LightViewer.instance()` (or equivalent `guik.viewer()`) should be first called on the main thread,
# because once you create a viewer instance on a thread,
# you must do all GL operations and destroy that instance on that thread.
viewer = guik.LightViewer.instance()

thread = threading.Thread(target=run, daemon=True)
thread.start()

# Use spin_once() in a Python loop instead of spin() so that the background Python thread can take the GIL between frames.
while viewer.spin_once():
  pass
