#ifndef GUIK_HOVERED_DRAWINGS_HPP
#define GUIK_HOVERED_DRAWINGS_HPP

#include <guik/viewer/light_viewer.hpp>

namespace guik {

struct HoveredDrawingsData;

class HoveredDrawing {
public:
  virtual ~HoveredDrawing() {}
  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) = 0;
};

class HoveredDrawings {
public:
  HoveredDrawings(guik::LightViewerContext* context = guik::LightViewer::instance());
  HoveredDrawings(std::shared_ptr<guik::LightViewerContext> context);
  ~HoveredDrawings();

  std::uint64_t add(const Eigen::Vector3f& pt, const std::shared_ptr<HoveredDrawing>& drawing);
  std::uint64_t add(const std::string& drawable_name, const std::shared_ptr<HoveredDrawing>& drawing);

  std::uint64_t add_text(
    const Eigen::Vector3f& pt,
    const std::string& text,
    const std::uint32_t fg_color = 0xFFFFFFFF,
    const std::uint32_t bg_color = 0x80000000,
    const Eigen::Vector2f& offset = {0.0f, 0.0f});
  std::uint64_t add_text_on(
    const std::string& drawable_name,
    const std::string& text,
    const std::uint32_t fg_color = 0xFFFFFFFF,
    const std::uint32_t bg_color = 0x80000000,
    const Eigen::Vector2f& offset = {0.0f, 0.0f});

  std::uint64_t add_cross(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float size = 7.07f, const float thickness = 1.0f);
  std::uint64_t add_cross_on(const std::string& drawable_name, const std::uint32_t color = 0xFFFFFFFF, const float size = 7.07f, const float thickness = 1.0f);

  std::uint64_t
  add_circle(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float radius = 10.0f, const int num_segments = 32, const float thickness = 1.0f);
  std::uint64_t
  add_circle_on(const std::string& drawable_name, const std::uint32_t color = 0xFFFFFFFF, const float radius = 10.0f, const int num_segments = 32, const float thickness = 1.0f);

  std::uint64_t add_triangle(
    const Eigen::Vector3f& pt,
    const std::uint32_t color = 0xFFFFFFFF,
    const float height = 20.0f,
    const float thickness = 1.0f,
    const bool upsidedown = true,
    const bool centering = false);
  std::uint64_t add_triangle_on(
    const std::string& drawable,
    const std::uint32_t color = 0xFFFFFFFF,
    const float height = 20.0f,
    const float thickness = 1.0f,
    const bool upsidedown = true,
    const bool centering = false);

  std::uint64_t
  add_filled_triangle(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float height = 20.0f, const bool upsidedown = true, const bool centering = false);
  std::uint64_t add_filled_triangle_on(
    const std::string& drawable_name,
    const std::uint32_t color = 0xFFFFFFFF,
    const float height = 20.0f,
    const bool upsidedown = true,
    const bool centering = false);

  std::uint64_t
  add_rect(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  std::uint64_t
  add_rect_on(const std::string& drawable, const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});

  std::uint64_t
  add_filled_rect(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  std::uint64_t add_filled_rect_on(
    const std::string& drawable_name,
    const std::uint32_t color = 0xFFFFFFFF,
    const Eigen::Vector2f& size = {15.0f, 15.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f});

  std::uint64_t add_image(
    const Eigen::Vector3f& pt,
    const std::shared_ptr<glk::Texture>& texture,
    const Eigen::Vector2f& size = {0.0f, 0.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f},
    const std::uint32_t bg_color = 0,
    const std::uint32_t border_color = 0,
    const float border_thickness = 1.0f);
  std::uint64_t add_image_on(
    const std::string& drawable_name,
    const std::shared_ptr<glk::Texture>& texture,
    const Eigen::Vector2f& size = {0.0f, 0.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f},
    const std::uint32_t bg_color = 0,
    const std::uint32_t border_color = 0,
    const float border_thickness = 1.0f);

  void remove_drawing(std::uint64_t drawing_id);
  void clear();

  std::function<void()> create_callback();

private:
  std::shared_ptr<HoveredDrawingsData> data;
};
}  // namespace guik

#endif