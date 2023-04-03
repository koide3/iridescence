#include <glk/lines.hpp>
#include <glk/thin_lines.hpp>
#include <glk/colormap.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  // sine wave
  std::vector<Eigen::Vector3f> line_vertices;
  std::vector<Eigen::Vector4f> line_colors;
  for (double x = -5.0; x <= 5.0; x += 0.1) {
    line_vertices.push_back(Eigen::Vector3f(x, std::cos(x), 1.0f));
    line_colors.push_back(glk::colormapf(glk::COLORMAP::TURBO, (x + 5.0) / 10.0));
  }

  // thin lines (GL_LINES)
  const bool line_strip = true;
  viewer->update_drawable("thin_lines", std::make_shared<glk::ThinLines>(line_vertices, line_strip), guik::FlatColor(0.0f, 1.0f, 0.0f, 1.0f));

  // thick lines with flat and vertex colors
  viewer->update_drawable("lines", std::make_shared<glk::Lines>(0.1f, line_vertices, line_colors, line_strip), guik::FlatColor(0.0f, 1.0f, 0.0f, 1.0f).translate(0.0f, 2.0f, 0.0f));
  viewer->update_drawable("colored_lines", std::make_shared<glk::Lines>(0.1f, line_vertices, line_colors, line_strip), guik::VertexColor().translate(0.0f, 4.0f, 0.0f));

  // coordinate systems
  bool draw_coords = true;
  viewer->register_ui_callback("coord_rendering_setting", [&] { ImGui::Checkbox("coords", &draw_coords); });
  viewer->register_drawable_filter("coord_filter", [&](const std::string& drawable_name) { return draw_coords || drawable_name.find("coord_") == std::string::npos; });

  for (double x = -5.0f; x <= 5.0f; x += 2.0) {
    Eigen::Affine3f transform = Eigen::Translation3f(x, 6.0f, 1.0f) * Eigen::Quaternionf::UnitRandom();
    viewer->update_drawable("coord_" + std::to_string(x), glk::Primitives::coordinate_system(), guik::VertexColor(transform));
  }

  // primitives
  viewer->update_drawable("solid_icosahedron", glk::Primitives::icosahedron(), guik::Rainbow().scale(0.5f).translate(-5.0f, 9.0f, 1.0f));
  viewer->update_drawable("solid_sphere", glk::Primitives::sphere(), guik::Rainbow().translate(-2.5f, 9.0f, 1.0f));
  viewer->update_drawable(
    "solid_bunny",
    glk::Primitives::bunny(),
    guik::Rainbow(Eigen::Translation3f(0.0f, 9.0f, 0.0f) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(8.0f)));
  viewer->update_drawable("solid_cube", glk::Primitives::cube(), guik::Rainbow().translate(2.5f, 9.0f, 1.0f));
  viewer->update_drawable("solid_cone", glk::Primitives::cone(), guik::Rainbow().translate(5.0f, 9.0f, 1.0f));

  // wireframes
  viewer->update_drawable("wire_icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_ICOSAHEDRON), guik::Rainbow().scale(0.5f).translate(-5.0f, 12.0f, 1.0f));
  viewer->update_drawable("wire_sphere", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_SPHERE), guik::Rainbow().translate(-2.5f, 12.0f, 1.0f));
  viewer->update_drawable(
    "wire_bunny",
    glk::Primitives::primitive_ptr(glk::Primitives::WIRE_BUNNY),
    guik::Rainbow(Eigen::Translation3f(0.0f, 12.0f, 0.0f) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(8.0f)));
  viewer->update_drawable("wire_cube", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_CUBE), guik::Rainbow().translate(2.5f, 12.0f, 1.0f));
  viewer->update_drawable("wire_cone", glk::Primitives::primitive_ptr(glk::Primitives::WIRE_CONE), guik::Rainbow().translate(5.0f, 12.0f, 1.0f));

  // transparent
  viewer->update_drawable(
    "trans_icosahedron",
    glk::Primitives::icosahedron(),
    guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f).scale(0.5f).translate(-5.0f, 15.0f, 1.0f).make_transparent());
  viewer->update_drawable("trans_sphere", glk::Primitives::sphere(), guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f).translate(-2.5f, 15.0f, 1.0f).make_transparent());
  viewer->update_drawable(
    "trans_bunny",
    glk::Primitives::bunny(),
    guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f, Eigen::Translation3f(0.0f, 15.0f, 0.0f) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(8.0f))
      .make_transparent());
  viewer->update_drawable("trans_cube", glk::Primitives::cube(), guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f).translate(2.5f, 15.0f, 1.0f).make_transparent());
  viewer->update_drawable("trans_cone", glk::Primitives::cone(), guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f).translate(5.0f, 15.0f, 1.0f).make_transparent());

  // rendering of drawables can be selectively turned off
  bool draw_lines = true;
  bool draw_primitives = true;
  bool draw_wireframes = true;
  bool draw_transparent = true;
  viewer->register_ui_callback("rendering_switch", [&] {
    ImGui::Checkbox("lines", &draw_lines);
    ImGui::Checkbox("primitives", &draw_primitives);
    ImGui::Checkbox("wireframes", &draw_wireframes);
    ImGui::Checkbox("transparent", &draw_transparent);
  });
  viewer->register_drawable_filter("drawable_filter", [&](const std::string& drawable_name) {
    if (!draw_lines && drawable_name.find("line") != std::string::npos) {
      return false;
    }
    if (!draw_primitives && drawable_name.find("solid_") != std::string::npos) {
      return false;
    }
    if (!draw_wireframes && drawable_name.find("wire_") != std::string::npos) {
      return false;
    }
    if (!draw_transparent && drawable_name.find("trans_") != std::string::npos) {
      return false;
    }

    return true;
  });

  // hovered 2D drawings
  auto hovered = glk::make_shared<guik::HoveredDrawings>();
  viewer->register_ui_callback("hovered", hovered->create_callback());

  // Put hovered texts
  hovered->add_text({8.0f, 0.0f, 1.0f}, "thin_lines", IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128));
  hovered->add_text({8.0f, 3.0f, 1.0f}, "lines");
  hovered->add_text({8.0f, 6.0f, 1.0f}, "coords");
  hovered->add_text({8.0f, 9.0f, 1.0f}, "solid");
  hovered->add_text({8.0f, 12.0f, 1.0f}, "wire");
  hovered->add_text({8.0f, 15.0f, 1.0f}, "transparent");

  // Put texts on drawables
  hovered->add_text_on("trans_icosahedron", "icosahedron");
  hovered->add_text_on("trans_sphere", "sphere");
  hovered->add_text_on("trans_bunny", "bunny");
  hovered->add_text_on("trans_cube", "cube");
  hovered->add_text_on("trans_cone", "cone");

  // Put texts at 3D points
  hovered->add_circle({-5.0f, 18.0f, 1.0f}, IM_COL32(255, 255, 255, 255), 12.0f);
  hovered->add_cross({-2.5f, 18.0f, 1.0f}, IM_COL32(255, 255, 255, 255), 10.0f);
  hovered->add_triangle({0.0f, 18.0f, 1.0f}, IM_COL32(255, 255, 255, 255), 20.0f, 1.0f, false, true);
  hovered->add_rect({2.5f, 18.0f, 1.0f}, IM_COL32(255, 255, 255, 255), {20.0f, 20.0f});
  hovered->add_filled_rect({5.0f, 18.0f, 1.0f}, IM_COL32(255, 255, 255, 255), {20.0f, 20.0f});

  hovered->add_text({-5.0f, 18.0f, 1.0f}, "circle", IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128), {0.0f, 25.0f});
  hovered->add_text({-2.5f, 18.0f, 1.0f}, "cross", IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128), {0.0f, 25.0f});
  hovered->add_text({0.0f, 18.0f, 1.0f}, "triangle", IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128), {0.0f, 25.0f});
  hovered->add_text({2.5f, 18.0f, 1.0f}, "rect", IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128), {0.0f, 25.0f});
  hovered->add_text({5.0f, 18.0f, 1.0f}, "filled_rect", IM_COL32(255, 255, 255, 255), IM_COL32(0, 0, 0, 128), {0.0f, 25.0f});

  // Change the coloring bandwidth of the guik::Rainbow coloring scheme
  Eigen::Vector2f z_range(-3.0f, 5.0f);
  viewer->register_ui_callback("control_z_range", [&] {
    if (ImGui::DragFloatRange2("z_range", &z_range[0], &z_range[1], 0.01f)) {
      viewer->shader_setting().add("z_range", z_range);
    }
  });

  viewer->spin();

  return 0;
}