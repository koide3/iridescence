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
  HoveredDrawings(const std::shared_ptr<guik::LightViewerContext>& context = guik::LightViewer::instance());
  ~HoveredDrawings();

  void add(const Eigen::Vector3f& pt, const std::shared_ptr<HoveredDrawing>& drawing);
  void add(const std::string& drawable_name, const std::shared_ptr<HoveredDrawing>& drawing);

  void add_text(
    const Eigen::Vector3f& pt,
    const std::string& text,
    const std::uint32_t fg_color = 0xFFFFFFFF,
    const std::uint32_t bg_color = 0x80000000,
    const Eigen::Vector2f& offset = {0.0f, 0.0f});
  void add_text(
    const std::string& drawable_name,
    const std::string& text,
    const std::uint32_t fg_color = 0xFFFFFFFF,
    const std::uint32_t bg_color = 0x80000000,
    const Eigen::Vector2f& offset = {0.0f, 0.0f});

  void add_cross(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float size = 7.07f, const float thickness = 1.0f);
  void add_cross(const std::string& drawable_name, const std::uint32_t color = 0xFFFFFFFF, const float size = 7.07f, const float thickness = 1.0f);

  void add_circle(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float radius = 10.0f, const int num_segments = 32, const float thickness = 1.0f);
  void add_circle(const std::string& drawable_name, const std::uint32_t color = 0xFFFFFFFF, const float radius = 10.0f, const int num_segments = 32, const float thickness = 1.0f);

  void add_triangle(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float height = 20.0f, const float thickness = 1.0f, const bool upsidedown = true, const bool centering = false);
  void add_triangle(
    const std::string& drawable,
    const std::uint32_t color = 0xFFFFFFFF,
    const float height = 20.0f,
    const float thickness = 1.0f,
    const bool upsidedown = true,
    const bool centering = false);

  void
  add_filled_triangle(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const float height = 20.0f, const bool upsidedown = true, const bool centering = false);
  void add_filled_triangle(
    const std::string& drawable_name,
    const std::uint32_t color = 0xFFFFFFFF,
    const float height = 20.0f,
    const bool upsidedown = true,
    const bool centering = false);

  void add_rect(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  void add_rect(const std::string& drawable, const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});

  void
  add_filled_rect(const Eigen::Vector3f& pt, const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  void add_filled_rect(
    const std::string& drawable_name,
    const std::uint32_t color = 0xFFFFFFFF,
    const Eigen::Vector2f& size = {15.0f, 15.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f});

  void add_image(
    const Eigen::Vector3f& pt,
    const std::shared_ptr<glk::Texture>& texture,
    const Eigen::Vector2f& size = {0.0f, 0.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f},
    const std::uint32_t bg_color = 0,
    const std::uint32_t border_color = 0,
    const float border_thickness = 1.0f);
  void add_image(
    const std::string& drawable_name,
    const std::shared_ptr<glk::Texture>& texture,
    const Eigen::Vector2f& size = {0.0f, 0.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f},
    const std::uint32_t bg_color = 0,
    const std::uint32_t border_color = 0,
    const float border_thickness = 1.0f);

  void clear();

  std::function<void()> create_callback();

private:
  std::shared_ptr<HoveredDrawingsData> data;
};
}  // namespace guik

#endif