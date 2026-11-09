#!/usr/bin/python3
# Custom drawable rendered with a custom GLSL shader (pip install PyOpenGL)
import numpy
from OpenGL import GL  # Do not alias as "gl" because "from pyridescence import *" exports pyridescence.gl
from pyridescence import *

# Vertex shader: displaces a flat grid into an animated wave surface on the GPU
VERTEX_SHADER = """
#version 330
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

uniform float time;
uniform float amplitude;

in vec3 vert_position;

out vec3 frag_normal;   // normal in world space
out float frag_height;  // normalized height in [-1, 1]

void main() {
  vec2 p = vert_position.xy;
  float sx = sin(2.0 * p.x + time);
  float cx = cos(2.0 * p.x + time);
  float sy = sin(1.5 * p.y + 0.7 * time);
  float cy = cos(1.5 * p.y + 0.7 * time);

  float h = amplitude * sx * cy;
  vec3 normal = normalize(vec3(-2.0 * amplitude * cx * cy, 1.5 * amplitude * sx * sy, 1.0));

  frag_normal = normalize(mat3(model_matrix) * normal);
  frag_height = sx * cy;
  gl_Position = projection_matrix * view_matrix * model_matrix * vec4(p, h, 1.0);
}
"""

# Fragment shader: must write the same outputs as the built-in shaders so that
# picking (info), screen-space effects (normal), and partial rendering keep working
FRAGMENT_SHADER = """
#version 330
uniform bool info_enabled;
uniform ivec4 info_values;
uniform bool normal_enabled;
uniform bool partial_rendering_enabled;
uniform int dynamic_object;

uniform vec3 low_color;
uniform vec3 high_color;

in vec3 frag_normal;
in float frag_height;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;
layout (location=2) out vec3 normal;
layout (location=3) out int dynamic_flag;

void main() {
  vec3 n = normalize(frag_normal);
  vec3 base = mix(low_color, high_color, 0.5 * frag_height + 0.5);
  float shade = 0.3 + 0.7 * abs(dot(n, normalize(vec3(0.3, 0.5, 1.0))));
  color = vec4(base * shade, 1.0);

  if (info_enabled) {
    info = info_values;
  }
  if (normal_enabled) {
    normal = n;
  }
  if (partial_rendering_enabled) {
    dynamic_flag = dynamic_object;
  }
}
"""


class WaveSurface(glk.Drawable):
  def __init__(self, size=10.0, n=128):
    super().__init__()  # Mandatory for Python-derived drawables

    # GL objects (including shaders) must be created on the GUI thread after the viewer is created
    self.shader = glk.GLSLShader()
    if not self.shader.attach_source_code(VERTEX_SHADER, gl.VERTEX_SHADER) or \
       not self.shader.attach_source_code(FRAGMENT_SHADER, gl.FRAGMENT_SHADER) or \
       not self.shader.link_program():
      raise RuntimeError('failed to build the wave surface shader')

    self.time = 0.0
    self.amplitude = 0.5
    self.low_color = numpy.array([0.1, 0.3, 0.9], dtype=numpy.float32)
    self.high_color = numpy.array([1.0, 0.85, 0.3], dtype=numpy.float32)

    # Flat (n x n) grid on the XY plane. The height is computed in the vertex shader.
    x, y = numpy.meshgrid(numpy.linspace(-size / 2, size / 2, n), numpy.linspace(-size / 2, size / 2, n), indexing='ij')
    vertices = numpy.stack([x, y, numpy.zeros_like(x)], axis=-1).reshape(-1, 3).astype(numpy.float32)

    i, j = numpy.meshgrid(numpy.arange(n - 1), numpy.arange(n - 1), indexing='ij')
    v00, v01, v10, v11 = i * n + j, i * n + j + 1, (i + 1) * n + j, (i + 1) * n + j + 1
    indices = numpy.stack([v00, v10, v11, v00, v11, v01], axis=-1).astype(numpy.uint32)
    self.num_indices = indices.size

    self.vao = GL.glGenVertexArrays(1)
    self.vbo, self.ebo = GL.glGenBuffers(2)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
    GL.glBufferData(GL.GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL.GL_STATIC_DRAW)
    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
    GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL.GL_STATIC_DRAW)
    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0)

  # Called every frame with the viewer's active shader whose uniforms are already set
  def draw(self, viewer_shader):
    # Switch to the custom shader and take over the viewer's uniforms
    # (model/view/projection matrices, info_values, etc.).
    # The custom shader must be in use before copying because set_uniform writes to the active program.
    self.shader.use()
    viewer_shader.copy_cached_uniforms(self.shader)

    self.shader.set_uniform('time', self.time)
    self.shader.set_uniform('amplitude', self.amplitude)
    self.shader.set_uniform('low_color', self.low_color)
    self.shader.set_uniform('high_color', self.high_color)

    position_loc = self.shader.attrib('vert_position')
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

    # Restore the viewer's shader for the following drawables
    viewer_shader.use()


def main():
  viewer = guik.viewer()

  wave = WaveSurface()
  viewer.update_drawable('wave', wave, guik.Rainbow())

  # UI callbacks are called on the GUI thread every frame
  def callback():
    wave.time += imgui.get_io().delta_time

    imgui.begin('wave', True, imgui.WindowFlags_AlwaysAutoResize)
    _, wave.amplitude = imgui.slider_float('amplitude', wave.amplitude, 0.0, 2.0)
    _, wave.low_color = imgui.color_edit3('low color', wave.low_color)
    _, wave.high_color = imgui.color_edit3('high color', wave.high_color)
    imgui.end()

  viewer.register_ui_callback('wave_ui', callback)
  viewer.spin()

if __name__ == '__main__':
  main()