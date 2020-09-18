#include <glk/lines.hpp>
#include <glk/thin_lines.hpp>
#include <glk/colormap.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  // sine wave
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> line_vertices;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> line_colors;
  for(double x = -5.0; x <= 5.0; x += 0.1) {
    line_vertices.push_back(Eigen::Vector3f(x, std::cos(x), 1.0f));
    line_vertices.push_back(Eigen::Vector3f(x + 0.1, std::cos(x + 0.1), 1.0f));

    Eigen::Vector4f color = glk::colormapf(glk::COLORMAP_TYPE::TURBO, (x + 5.0) / 10.0);
    line_colors.push_back(color);
    line_colors.push_back(color);
  }

  // rendering of drawables can be selectively turned off
  bool draw_lines = true;
  viewer->register_ui_callback("line_rendering_setting", [&] { ImGui::Checkbox("lines", &draw_lines); });
  viewer->add_drawable_filter("line_filter", [&](const std::string& drawable_name) { return draw_lines || drawable_name.find("line") == std::string::npos; });

  // thin lines (GL_LINES)
  viewer->update_drawable("thin_lines", std::make_shared<glk::ThinLines>(line_vertices), guik::FlatColor(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f)));

  // lines with thickness
  Eigen::Matrix4f transform = (Eigen::Translation3f(Eigen::Vector3f::UnitY() * 2.0f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("lines", std::make_shared<glk::Lines>(0.1f, line_vertices, line_colors), guik::FlatColor(Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), transform));

  // colored lines
  transform = (Eigen::Translation3f(Eigen::Vector3f::UnitY() * 4.0f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("colored_lines", std::make_shared<glk::Lines>(0.1f, line_vertices, line_colors), guik::VertexColor(transform));

  // coordinate systems
  bool draw_coords = true;
  viewer->register_ui_callback("coord_rendering_setting", [&] { ImGui::Checkbox("coords", &draw_coords); });
  viewer->add_drawable_filter("coord_filter", [&](const std::string& drawable_name) { return draw_coords || drawable_name.find("coord_") == std::string::npos; });

  for(double x = -5.0f; x <= 5.0f; x += 2.0) {
    transform = (Eigen::Translation3f(Eigen::Vector3f(x, 6.0f, 1.0f)) * Eigen::Quaternionf::UnitRandom() * Eigen::Isometry3f::Identity()).matrix();
    viewer->update_drawable("coord_" + std::to_string(x), glk::Primitives::primitive_ptr(glk::Primitives::COORDINATE_SYSTEM), guik::VertexColor(transform));
  }

  // primitives
  bool draw_primitives = true;
  viewer->register_ui_callback("privimive_rendering_setting", [&] { ImGui::Checkbox("primitives", &draw_primitives); });
  viewer->add_drawable_filter("primitive_filter", [&](const std::string& drawable_name) { return draw_primitives || drawable_name.find("solid_") == std::string::npos; });

  transform = (Eigen::Translation3f(Eigen::Vector3f(-5.0f, 9.0f, 1.0f)) * Eigen::UniformScaling<float>(0.5f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("solid_icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::ICOSAHEDRON), guik::Rainbow(transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(-2.5f, 9.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("solid_sphere", glk::Primitives::primitive_ptr(glk::Primitives::SPHERE), guik::Rainbow(transform));

  transform = (Eigen::Isometry3f::Identity() * Eigen::Translation3f(Eigen::Vector3f(0, 9.0f, 0.0f)) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(8.0f)).matrix();
  viewer->update_drawable("solid_bunny", glk::Primitives::primitive_ptr(glk::Primitives::BUNNY), guik::Rainbow(transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(2.5f, 9.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("solid_cube", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::Rainbow(transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(5.0f, 9.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("solid_cone", glk::Primitives::primitive_ptr(glk::Primitives::CONE), guik::Rainbow(transform));

  // wireframes
  bool draw_wireframes = true;
  viewer->register_ui_callback("wireframe_rendering_setting", [&] { ImGui::Checkbox("wireframes", &draw_wireframes); });
  viewer->add_drawable_filter("wireframe_filter", [&](const std::string& drawable_name) { return draw_wireframes || drawable_name.find("wire_") == std::string::npos; });

  transform = (Eigen::Translation3f(Eigen::Vector3f(-5.0f, 12.0f, 1.0f)) * Eigen::UniformScaling<float>(0.5f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("wire_icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_ICOSAHEDRON), guik::Rainbow(transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(-2.5f, 12.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("wire_sphere", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_SPHERE), guik::Rainbow(transform));

  transform = (Eigen::Isometry3f::Identity() * Eigen::Translation3f(Eigen::Vector3f(0, 12.0f, 0.0f)) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(8.0f)).matrix();
  viewer->update_drawable("wire_bunny", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_BUNNY), guik::Rainbow(transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(2.5f, 12.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("wire_cube", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_CUBE), guik::Rainbow(transform));

  transform = (Eigen::Translation3f(Eigen::Vector3f(5.0f, 12.0f, 1.0f)) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("wire_cone", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_CONE), guik::Rainbow(transform));

  viewer->spin();

  return 0;
}