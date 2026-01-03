#include <glk/trajectory.hpp>

#include <numeric>

#include <glk/thin_lines.hpp>
#include <glk/primitives/primitives.hpp>

namespace glk {

Trajectory::Trajectory(const std::vector<Eigen::Isometry3f>& trajectory) : trajectory(trajectory) {
  std::vector<Eigen::Vector3f> line_vertices;
  for (int i = 1; i < trajectory.size(); i++) {
    line_vertices.push_back(trajectory[i - 1].translation());
    line_vertices.push_back(trajectory[i].translation());
  }
  lines.reset(new glk::ThinLines(line_vertices));
}

Trajectory::Trajectory(int num_frames, const std::function<Eigen::Isometry3f(int)>& adapter) {
  trajectory.resize(num_frames);
  for (int i = 0; i < num_frames; i++) {
    trajectory[i] = adapter(i);
  }

  std::vector<Eigen::Vector3f> line_vertices;
  for (int i = 1; i < trajectory.size(); i++) {
    line_vertices.push_back(trajectory[i - 1].translation());
    line_vertices.push_back(trajectory[i].translation());
  }
  lines.reset(new glk::ThinLines(line_vertices));
}

Trajectory::~Trajectory() {}

void Trajectory::draw(glk::GLSLShader& shader) const {
  shader.set_uniform("color_mode", 2);
  for (int i = 0; i < trajectory.size(); i++) {
    shader.set_uniform("model_matrix", trajectory[i].matrix());
    glk::Primitives::coordinate_system()->draw(shader);
  }

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
  shader.set_uniform("material_color", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
  lines->draw(shader);
}

}  // namespace glk