#include <guik/hovered_drawings.hpp>
#include <guik/hovered_primitives.hpp>

#include <iostream>

namespace guik {

constexpr std::uint64_t DRAWABLE_BIT = 1ULL << 63;
constexpr std::uint64_t DRAWING_ID_MASK = DRAWABLE_BIT - 1;

std::uint64_t calc_drawing_id(bool drawable, std::uint64_t id) {
  return drawable ? (DRAWABLE_BIT | id) : id;
}
bool is_drawable_drawing(std::uint64_t id) {
  return (id & DRAWABLE_BIT) != 0;
}
std::uint64_t get_drawing_id(std::uint64_t id) {
  return id & DRAWING_ID_MASK;
}

struct HoveredDrawingsData {
  guik::LightViewerContext* context;
  std::shared_ptr<guik::LightViewerContext> shared_context;  // Sub viewer context

  std::vector<int> removed_drawable_drawing_ids;
  std::vector<std::string> drawable_names;
  std::vector<std::shared_ptr<HoveredDrawing>> drawable_drawings;

  std::vector<int> removed_drawing_ids;
  std::vector<Eigen::Vector4f> pts;
  std::vector<std::shared_ptr<HoveredDrawing>> drawings;
};

HoveredDrawings::HoveredDrawings(guik::LightViewerContext* context) : data(new HoveredDrawingsData{context}) {}
HoveredDrawings::HoveredDrawings(std::shared_ptr<guik::LightViewerContext> context) : data(new HoveredDrawingsData{context.get(), context}) {}

HoveredDrawings::~HoveredDrawings() {}

std::uint64_t HoveredDrawings::add(const Eigen::Vector3f& pt, const std::shared_ptr<HoveredDrawing>& drawing) {
  if (!data->removed_drawing_ids.empty()) {
    const int id = data->removed_drawing_ids.back();
    data->removed_drawing_ids.pop_back();

    data->pts[id] = Eigen::Vector4f(pt[0], pt[1], pt[2], 1.0f);
    data->drawings[id] = drawing;
    return calc_drawing_id(false, id);
  }

  data->pts.emplace_back(pt[0], pt[1], pt[2], 1.0f);
  data->drawings.emplace_back(drawing);
  return calc_drawing_id(false, data->pts.size() - 1);
}

std::uint64_t HoveredDrawings::add(const std::string& drawable_name, const std::shared_ptr<HoveredDrawing>& drawing) {
  if (!data->removed_drawable_drawing_ids.empty()) {
    const int id = data->removed_drawable_drawing_ids.back();
    data->removed_drawable_drawing_ids.pop_back();

    data->drawable_names[id] = drawable_name;
    data->drawable_drawings[id] = drawing;
    return calc_drawing_id(true, id);
  }

  data->drawable_names.emplace_back(drawable_name);
  data->drawable_drawings.emplace_back(drawing);
  return calc_drawing_id(true, data->drawable_names.size() - 1);
}

std::uint64_t
HoveredDrawings::add_text(const Eigen::Vector3f& pt, const std::string& text, const std::uint32_t fg_color, const std::uint32_t bg_color, const Eigen::Vector2f& offset) {
  return add(pt, glk::make_shared<HoveredText>(text, fg_color, bg_color, offset));
}

std::uint64_t
HoveredDrawings::add_text_on(const std::string& drawable_name, const std::string& text, const std::uint32_t fg_color, const std::uint32_t bg_color, const Eigen::Vector2f& offset) {
  return add(drawable_name, glk::make_shared<HoveredText>(text, fg_color, bg_color, offset));
}

std::uint64_t HoveredDrawings::add_cross(const Eigen::Vector3f& pt, const std::uint32_t color, const float size, const float thickness) {
  return add(pt, glk::make_shared<HoveredCross>(color, size, thickness));
}

std::uint64_t HoveredDrawings::add_cross_on(const std::string& drawable_name, const std::uint32_t color, const float size, const float thickness) {
  return add(drawable_name, glk::make_shared<HoveredCross>(color, size, thickness));
}

std::uint64_t HoveredDrawings::add_circle(const Eigen::Vector3f& pt, const std::uint32_t color, const float radius, const int num_segments, const float thickness) {
  return add(pt, glk::make_shared<HoveredCircle>(color, radius, num_segments, thickness));
}

std::uint64_t HoveredDrawings::add_circle_on(const std::string& drawable_name, const std::uint32_t color, const float radius, const int num_segments, const float thickness) {
  return add(drawable_name, glk::make_shared<HoveredCircle>(color, radius, num_segments, thickness));
}

std::uint64_t
HoveredDrawings::add_triangle(const Eigen::Vector3f& pt, const std::uint32_t color, const float height, const float thickness, const bool upsidedown, const bool centering) {
  return add(pt, glk::make_shared<HoveredTriangle>(color, height, thickness, upsidedown, centering));
}

std::uint64_t HoveredDrawings::add_triangle_on(
  const std::string& drawable_name,
  const std::uint32_t color,
  const float height,
  const float thickness,
  const bool upsidedown,
  const bool centering) {
  return add(drawable_name, glk::make_shared<HoveredTriangle>(color, height, thickness, upsidedown, centering));
}

std::uint64_t HoveredDrawings::add_filled_triangle(const Eigen::Vector3f& pt, const std::uint32_t color, const float height, const bool upsidedown, const bool centering) {
  return add(pt, glk::make_shared<HoveredFilledTriangle>(color, height, upsidedown, centering));
}

std::uint64_t
HoveredDrawings::add_filled_triangle_on(const std::string& drawable_name, const std::uint32_t color, const float height, const bool upsidedown, const bool centering) {
  return add(drawable_name, glk::make_shared<HoveredFilledTriangle>(color, height, upsidedown, centering));
}

std::uint64_t HoveredDrawings::add_rect(const Eigen::Vector3f& pt, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  return add(pt, glk::make_shared<HoveredRect>(color, size, offset));
}

std::uint64_t HoveredDrawings::add_rect_on(const std::string& drawable_name, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  return add(drawable_name, glk::make_shared<HoveredRect>(color, size, offset));
}

std::uint64_t HoveredDrawings::add_filled_rect(const Eigen::Vector3f& pt, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  return add(pt, glk::make_shared<HoveredFilledRect>(color, size, offset));
}

std::uint64_t HoveredDrawings::add_filled_rect_on(const std::string& drawable_name, const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) {
  return add(drawable_name, glk::make_shared<HoveredFilledRect>(color, size, offset));
}

std::uint64_t HoveredDrawings::add_image(
  const Eigen::Vector3f& pt,
  const std::shared_ptr<glk::Texture>& texture,
  const Eigen::Vector2f& size,
  const Eigen::Vector2f& offset,
  const std::uint32_t bg_color,
  const std::uint32_t border_color,
  const float border_thickness) {
  return add(pt, glk::make_shared<HoveredImage>(texture, size, offset, bg_color, border_color, border_thickness));
}

std::uint64_t HoveredDrawings::add_image_on(
  const std::string& drawable_name,
  const std::shared_ptr<glk::Texture>& texture,
  const Eigen::Vector2f& size,
  const Eigen::Vector2f& offset,
  const std::uint32_t bg_color,
  const std::uint32_t border_color,
  const float border_thickness) {
  return add(drawable_name, glk::make_shared<HoveredImage>(texture, size, offset, bg_color, border_color, border_thickness));
}

void HoveredDrawings::remove_drawing(std::uint64_t drawing_id) {
  const bool drawable = is_drawable_drawing(drawing_id);
  const std::uint64_t id = get_drawing_id(drawing_id);

  auto& removed_ids = drawable ? data->removed_drawable_drawing_ids : data->removed_drawing_ids;
  auto& drawings = drawable ? data->drawable_drawings : data->drawings;

  if (drawings.size() <= id) {
    std::cerr << "warning: invalid drawing id " << drawing_id << std::endl;
    return;
  }

  if (!drawings[id]) {
    std::cerr << "warning: already removed drawing id " << drawing_id << std::endl;
    return;
  }

  drawings[id].reset();
  removed_ids.emplace_back(id);
}

void HoveredDrawings::clear() {
  data->drawable_names.clear();
  data->drawable_drawings.clear();

  data->pts.clear();
  data->drawings.clear();
}

std::function<void()> HoveredDrawings::create_callback() {
  return [data = data] {
    std::vector<Eigen::Vector4f> all_pts;
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
      if (drawing) {
        drawing->draw(drawlist, uvz.block<3, 1>(0, i));
      }
    }
  };
}
}  // namespace guik