# クックブック

## 点群エディタ

[src/example/ext_pointcloud_editor.cpp](https://github.com/koide3/iridescence/blob/master/src/example/ext_pointcloud_editor.cpp)

<details>
  <summary>ext_pointcloud_editor.cpp</summary>
```cpp linenums="1"
#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>

class PointCloudEditor {
public:
  PointCloudEditor() {
    auto viewer = guik::LightViewer::instance();

    cube_matrix.reset(new guik::ModelControl("cube_matrix"));

    viewer->register_ui_callback("ui", [this] { ui_callback(); });
    viewer->spin();
  }

private:
  void ui_callback() {
    auto viewer = guik::LightViewer::instance();

    ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // 点群の読み込み    
    if (ImGui::Button("Load point cloud")) {
      for (const auto& filename : pfd::open_file("Select a PLY file").result()) {
        // PLYから点群を読み込み、点リストに追加
        auto ply = glk::load_ply(filename);
        if (ply) {
          points.insert(points.end(), ply->vertices.begin(), ply->vertices.end());
        }
      }

      // ビューワに点を表示
      if (!points.empty()) {
        cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
        viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      }
    }

    // 点群の保存
    if (ImGui::Button("Save point cloud")) {
      auto path = pfd::save_file("Select a destination path to save PLY").result();
      if (!path.empty() && !points.empty()) {
        glk::save_ply_binary(path, points.data(), points.size());
      }
    }

    // フィルタリング領域を表す立方体を表示
    ImGui::Separator();
    cube_matrix->draw_gizmo_ui();
    cube_matrix->draw_gizmo();
    viewer->update_drawable("cube", glk::Primitives::cube(), guik::FlatColor({1.0f, 0.5f, 0.0f, 0.5f}, cube_matrix->model_matrix()).make_transparent());

    // フィルタリング領域内の点を検索
    if (ImGui::Button("Select points")) {
      // 立方体のモデル行列の逆行列は、ワールド座標系の点を立方体座標系に変換します
      Eigen::Matrix4f inv_cube_matrix = cube_matrix->model_matrix().inverse();

      selected_points.clear();
      neg_selected_points.clear();
      for (int i = 0; i < points.size(); i++) {
        Eigen::Vector3f pt = (Eigen::Affine3f(inv_cube_matrix) * points[i]);
        // [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)] の範囲にある点は立方体の内部にあります
        if ((pt.array() > Eigen::Array3f::Constant(-0.5f)).all() && (pt.array() < Eigen::Array3f::Constant(0.5f)).all()) {
          selected_points.emplace_back(i);
        } else {
          neg_selected_points.emplace_back(i);
        }
      }

      // 選択された点を大きなオレンジ色の点で表示
      viewer->update_drawable("selected", std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, selected_points), guik::FlatOrange().set_point_scale(2.0f));
    }

    // 選択された点を点リストから削除
    if (ImGui::Button("Remove points")) {
      // 選択されていない点のみを残す
      std::vector<Eigen::Vector3f> filtered;
      std::transform(neg_selected_points.begin(), neg_selected_points.end(), std::back_inserter(filtered), [&](const auto i) { return points[i]; });
      points = std::move(filtered);

      selected_points.clear();
      neg_selected_points.clear();

      cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
      viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      viewer->remove_drawable("selected");
    }

    ImGui::End();
  }

private:
  std::unique_ptr<guik::ModelControl> cube_matrix;      // フィルタリング領域を表す立方体のモデル行列

  std::vector<Eigen::Vector3f> points;                  // 点群
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;  // 点のCloudBuffer
  std::vector<unsigned int> selected_points;            // 選択された点
  std::vector<unsigned int> neg_selected_points;        // 選択されていない点
};

int main(int argc, char** argv) {
  PointCloudEditor editor;
  return 0;
}
```
</details>

- portable-file-dialogsを使用したPLYファイルの入出力
- Gizmoベースのモデル行列操作
- ```PointCloudBuffer``` と ```IndexedPointCloudBuffer``` を使用した点群レンダリング

<iframe width="560" height="315" src="https://www.youtube.com/embed/pHPGS8VNN2o" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


## Custom shader and drawable in Python


[src/example/py_custom_shader.cpp](https://github.com/koide3/iridescence/blob/master/src/example/py_custom_shader.cpp)


<details>
  <summary>ext_pointcloud_editor.cpp</summary>

```py linenums="1"
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
```

</details>

<img width="1920" height="1080" alt="Image" src="https://github.com/user-attachments/assets/0a4a66a4-b280-455e-9869-006caf8e8302" />