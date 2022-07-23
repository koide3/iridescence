#include <guik/hovered_drawings.hpp>
#include <guik/hovered_primitives.hpp>

namespace guik {

struct HoveredDrawingsData {
  std::shared_ptr<guik::LightViewerContext> context;

  std::vector<std::string> drawable_names;
  std::vector<std::shared_ptr<HoveredDrawing>> drawable_drawings;

  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> pts;
  std::vector<std::shared_ptr<HoveredDrawing>> drawings;
};

HoveredDrawings::HoveredDrawings(const std::shared_ptr<guik::LightViewerContext>& context) : data(new HoveredDrawingsData{context}) {}

HoveredDrawings::~HoveredDrawings() {}

void HoveredDrawings::add(const Eigen::Vector3f& pt, const std::shared_ptr<HoveredDrawing>& drawing) {
  data->pts.emplace_back(pt[0], pt[1], pt[2], 1.0f);
  data->drawings.emplace_back(drawing);
}

void HoveredDrawings::add(const std::string& drawable_name, const std::shared_ptr<HoveredDrawing>& drawing) {
  data->drawable_names.emplace_back(drawable_name);
  data->drawable_drawings.emplace_back(drawing);
}

void HoveredDrawings::add_text(const Eigen::Vector3f& pt, const std::string& text, const std::uint32_t fg_color, const std::uint32_t bg_color, const Eigen::Vector2f& offset) {
  add(pt, glk::make_shared<HoveredText>(text, fg_color, bg_color, offset));
}

void HoveredDrawings::add_text_on(
  const std::string& drawable_name,
  const std::string& text,
  const std::uint32_t fg_color,
  const std::uint32_t bg_color,
  const Eigen::Vector2f& offset) {
  add(drawable_name, glk::make_shared<HoveredText>(text, fg_color, bg_color, offset));
}

void HoveredDrawings::add_cross(const Eigen::Vector3f& pt, const std::uint32_t color, const float size, const float thickness) {
  add(pt, glk::make_shared<HoveredCross>(color, size, thickness));
}

void HoveredDrawings::add_cross_on(const std::string& drawable_name, const std::uint32_t color, const float size, const float thickness) {
  add(drawable_name, glk::make_shared<HoveredCross>(color, size, thickness));
}

void HoveredDrawings::add_circle(const Eigen::Vector3f& pt, const std::uint32_t color, const float radius, const int num_segments, const float thickness) {
  add(pt, glk::make_shared<HoveredCircle>(color, radius, num_segments, thickness));
}

void HoveredDrawings::add_circle_on(const std::string& drawable_name, const std::uint32_t color, const float radius, const int num_segments, const float thickness) {
  add(drawable_name, glk::make_shared<HoveredCircle>(color, radius, num_segments, thickness));
}

void HoveredDrawings::add_triangle(const Eigen::Vector3f& pt, const std::uint32_t color, const float height, const float thickness, const bool upsidedown, const bool centering) {
  add(pt, glk::make_shared<HoveredTriangle>(color, height, thickness, upsidedown, centering));
}

void HoveredDrawings::add_triangle_on(
  const std::string& drawable_name,
  const std::uint32_t color,
  const float height,
  const float thickness,
  const bool upsidedown,
  const bool centering) {
  add(drawable_name, glk::make_shared<HoveredTriangle>(color, height, thickness, upsidedown, centering));
}

void HoveredDrawings::add_filled_triangle(const Eigen::Vector3f& pt, const std::uint32_t color, const float height, const bool upsidedown, const bool centering) {
  add(pt, glk::make_shared<HoveredFilledTriangle>(color, height, upsidedown, centering));
}

void HoveredDrawings::add_filled_triangle_on(const std::string& drawable_name, const std::uint32_t color, const float height, const bool upsidedown, const bool centering) {
  add(drawable_name, glk::make_shared<HoveredFilledTriangle>(color, height, upsidedown, centering));
}

void HoveredDrawings::add_rect(const Eigen::Vector3f& pt, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  add(pt, glk::make_shared<HoveredRect>(color, size, offset));
}

void HoveredDrawings::add_rect_on(const std::string& drawable_name, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  add(drawable_name, glk::make_shared<HoveredRect>(color, size, offset));
}

void HoveredDrawings::add_filled_rect(const Eigen::Vector3f& pt, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  add(pt, glk::make_shared<HoveredFilledRect>(color, size, offset));
}

void HoveredDrawings::add_filled_rect_on(const std::string& drawable_name, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  add(drawable_name, glk::make_shared<HoveredFilledRect>(color, size, offset));
}

void HoveredDrawings::add_image(
  const Eigen::Vector3f& pt,
  const std::shared_ptr<glk::Texture>& texture,
  const Eigen::Vector2f& size,
  const Eigen::Vector2f& offset,
  const std::uint32_t bg_color,
  const std::uint32_t border_color,
  const float border_thickness) {
  add(pt, glk::make_shared<HoveredImage>(texture, size, offset, bg_color, border_color, border_thickness));
}

void HoveredDrawings::add_image_on(
  const std::string& drawable_name,
  const std::shared_ptr<glk::Texture>& texture,
  const Eigen::Vector2f& size,
  const Eigen::Vector2f& offset,
  const std::uint32_t bg_color,
  const std::uint32_t border_color,
  const float border_thickness) {
  add(drawable_name, glk::make_shared<HoveredImage>(texture, size, offset, bg_color, border_color, border_thickness));
}

void HoveredDrawings::clear() {
  data->drawable_names.clear();
  data->drawable_drawings.clear();

  data->pts.clear();
  data->drawings.clear();
}

std::function<void()> HoveredDrawings::create_callback() {
  return [data = data] {
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> all_pts;
    std::vector<std::shared_ptr<HoveredDrawing>> all_drawings;

    all_pts.reserve(data->pts.size() + data->drawable_names.size());
    all_drawings.reserve(data->drawings.size() + data->drawable_drawings.size());
    std::copy(data->pts.begin(), data->pts.end(), std::back_inserter(all_pts));
    std::copy(data->drawings.begin(), data->drawings.end(), std::back_inserter(all_drawings));

    for (int i = 0; i < data->drawable_drawings.size(); i++) {
      const auto drawable = data->context->find_drawable(data->drawable_names[i]);
      if (drawable.first) {
        const auto model_matrix = drawable.first->get<Eigen::Matrix4f>("model_matrix");
        if (model_matrix) {
          all_pts.emplace_back(model_matrix->rightCols<1>());
          all_drawings.emplace_back(data->drawable_drawings[i]);
        }
      }
    }

    if (all_pts.empty()) {
      return;
    }

    const auto& mapped_pts = Eigen::Map<const Eigen::Matrix<float, 4, -1>>(all_pts[0].data(), 4, all_pts.size());
    Eigen::Matrix<float, 4, -1> uvz = data->context->get_projection_control()->projection_matrix() * data->context->get_camera_control()->view_matrix() * mapped_pts;
    for (int i = 0; i < all_pts.size(); i++) {
      uvz.col(i) /= uvz.col(i).w();
    }

    const Eigen::Vector2i canvas_size = data->context->canvas_size();
    uvz.topRows(2) = (uvz.array().topRows(2) * 0.5f + 0.5f).colwise() * canvas_size.array().cast<float>();
    uvz.row(1) = canvas_size[1] - uvz.array().row(1);

    ImDrawList* drawlist;
    if (data->context->canvas_tl().isZero()) {
      drawlist = ImGui::GetBackgroundDrawList();
    } else {
      drawlist = ImGui::GetWindowDrawList();
      uvz.topRows(2).colwise() += data->context->canvas_tl().cast<float>();
    }

    for (int i = 0; i < all_pts.size(); i++) {
      const float z = uvz(2, i);
      if (z < -1.0f || z >= 1.0f) {
        continue;
      }

      const auto& drawing = all_drawings[i];
      drawing->draw(drawlist, uvz.block<3, 1>(0, i));
    }
  };
}
}  // namespace guik