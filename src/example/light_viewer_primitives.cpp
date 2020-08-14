#include <glk/lines.hpp>
#include <glk/thin_lines.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> line_vertices;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> line_colors;
  for(double x = -5.0; x <= 5.0; x += 0.1) {
    line_vertices.push_back(Eigen::Vector3f(x, std::cos(x), 1.0f));
    line_vertices.push_back(Eigen::Vector3f(x + 0.1, std::cos(x + 0.1), 1.0f));

    double p = (x + 5.0) / 10.0;
    line_colors.push_back(Eigen::Vector4f(p, 1 - p, 0.0f, 1.0f));
    line_colors.push_back(Eigen::Vector4f(p, 1 - p, 0.0f, 1.0f));
  }

  // thin lines (GL_LINES)
  viewer->update_drawable("thin_lines", std::make_shared<glk::ThinLines>(line_vertices), guik::FlatColor(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f)));

  // lines with thickness
  Eigen::Matrix4f transform = (Eigen::Translation3f(Eigen::Vector3f::UnitY() * 2.0f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("lines", std::make_shared<glk::Lines>(0.1f, line_vertices, line_colors), guik::FlatColor(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), transform));

  // colored lines
  transform = (Eigen::Translation3f(Eigen::Vector3f::UnitY() * 4.0f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("colored_lines", std::make_shared<glk::Lines>(0.1f, line_vertices, line_colors), guik::VertexColor(transform));

  // coordinate systems
  for(double x = -5.0f; x <= 5.0f; x += 2.0) {
    transform = (Eigen::Translation3f(Eigen::Vector3f(x, 6.0f, 1.0f)) * Eigen::Quaternionf::UnitRandom() * Eigen::Isometry3f::Identity()).matrix();
    viewer->update_drawable("coord_" + std::to_string(x), glk::Primitives::primitive_ptr(glk::Primitives::COORDINATE_SYSTEM), guik::VertexColor(transform));
  }

  // primitives
  transform = (Eigen::Translation3f(Eigen::Vector3f(-5.0f, 9.0f, 1.0f)) * Eigen::UniformScaling<float>(0.5f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::ICOSAHEDRON), guik::FlatColor(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f), transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(-2.5f, 9.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("sphere", glk::Primitives::primitive_ptr(glk::Primitives::SPHERE), guik::FlatColor(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(2.5f, 9.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("cube", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::FlatColor(Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(5.0f, 9.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("cone", glk::Primitives::primitive_ptr(glk::Primitives::CONE), guik::FlatColor(Eigen::Vector4f(1.0f, 1.0f, 0.0f, 1.0f), transform));


  for(double t = 0.0; ; t += 0.001) {
    transform = (Eigen::Translation3f(Eigen::Vector3f(std::cos(t) * 20.0, std::sin(t) * 20.0, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
    viewer->update_drawable("moving_sphere", glk::Primitives::primitive_ptr(glk::Primitives::SPHERE), guik::FlatColor(Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f), transform));

    if(!viewer->spin_once()) {
      break;
    }
  }

  return 0;
}