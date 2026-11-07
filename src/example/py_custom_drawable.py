#!/usr/bin/python3
# Custom drawable implemented with PyOpenGL (pip install PyOpenGL)
import numpy
from OpenGL import GL  # Do not alias as "gl" because "from pyridescence import *" exports pyridescence.gl
from pyridescence import *


class Torus(glk.Drawable):
  def __init__(self, n=64):
    super().__init__()  # Mandatory for Python-derived drawables
    self.n = n

    # Triangle indices of the (n x n) torus grid
    i, j = numpy.meshgrid(numpy.arange(n), numpy.arange(n), indexing='ij')
    v00, v01 = i * n + j, i * n + (j + 1) % n
    v10, v11 = ((i + 1) % n) * n + j, ((i + 1) % n) * n + (j + 1) % n
    indices = numpy.stack([v00, v10, v11, v00, v11, v01], axis=-1).astype(numpy.uint32)
    self.num_indices = indices.size

    # GL objects must be created on the GUI thread after the viewer is created
    self.vao = GL.glGenVertexArrays(1)
    self.vbo, self.ebo = GL.glGenBuffers(2)
    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
    GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL.GL_STATIC_DRAW)
    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0)
    self.update(0.0)

  def update(self, t):
    # Torus whose tube radius varies along the ring and over time
    u, v = numpy.meshgrid(*[numpy.linspace(0, 2 * numpy.pi, self.n, endpoint=False)] * 2, indexing='ij')
    r = 0.7 + 0.3 * numpy.sin(3.0 * u + 2.0 * t)
    x = (2.0 + r * numpy.cos(v)) * numpy.cos(u)
    y = (2.0 + r * numpy.cos(v)) * numpy.sin(u)
    z = r * numpy.sin(v)
    vertices = numpy.stack([x, y, z], axis=-1).reshape(-1, 3).astype(numpy.float32)

    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
    GL.glBufferData(GL.GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL.GL_DYNAMIC_DRAW)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

  # Called every frame with the active shader whose uniforms are already set by the viewer
  def draw(self, shader):
    position_loc = shader.attrib('vert_position')

    GL.glBindVertexArray(self.vao)
    GL.glEnableVertexAttribArray(position_loc)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
    GL.glVertexAttribPointer(position_loc, 3, GL.GL_FLOAT, GL.GL_FALSE, 0, None)

    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
    GL.glDrawElements(GL.GL_TRIANGLES, self.num_indices, GL.GL_UNSIGNED_INT, None)

    GL.glDisableVertexAttribArray(position_loc)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
    GL.glBindVertexArray(0)
    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0)


viewer = guik.viewer()

torus = Torus()
viewer.update_drawable('torus', torus, guik.Rainbow())

# UI callbacks are called on the GUI thread, so GL buffers can be updated here
def callback():
  torus.update(imgui.get_time())

viewer.register_ui_callback('update', callback)
viewer.spin()
