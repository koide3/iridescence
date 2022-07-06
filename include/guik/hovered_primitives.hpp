#ifndef GUIK_HOVERED_PRIMITIVES_HPP
#define GUIK_HOVERED_PRIMITIVES_HPP

#include <guik/hovered_drawings.hpp>

namespace guik
{

class HoveredText : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredText(const std::string& text, const std::uint32_t fg_color = 0xFFFFFFFF, const std::uint32_t bg_color = 0x80000000, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  virtual ~HoveredText() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::string text;
  const std::uint32_t fg_color;
  const std::uint32_t bg_color;
  const Eigen::Vector2f offset;
};

class HoveredCross : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredCross(const std::uint32_t color = 0xFFFFFFFF, const float size = 5.0f, const float thickness = 1.0f);
  virtual ~HoveredCross() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::uint32_t color;
  const float size;
  const float thickness;
};

class HoveredCircle : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredCircle(const std::uint32_t color = 0xFFFFFFFF, const float radius = 5.0f, const int num_segments = 32, const float thickness = 1.0f);
  virtual ~HoveredCircle() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::uint32_t color;
  const float radius;
  const int num_segments;
  const float thickness;
};

class HoveredTriangle : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredTriangle(const std::uint32_t color = 0xFFFFFFFF, const float height = 5.0f, const float thickness = 1.0f, const bool upsidedown = true, const bool centering = false);
  virtual ~HoveredTriangle() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::uint32_t color;
  const float half_width;
  const float height;
  const float thickness;
  const bool centering;
};

class HoveredFilledTriangle : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredFilledTriangle(const std::uint32_t color = 0xFFFFFFFF, const float height = 5.0f, const bool upsidedown = true, const bool centering = false);
  virtual ~HoveredFilledTriangle() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::uint32_t color;
  const float half_width;
  const float height;
  const bool centering;
};

class HoveredRect : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredRect(const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  virtual ~HoveredRect() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::uint32_t color;
  const Eigen::Vector2f size;
  const Eigen::Vector2f offset;
};

class HoveredFilledRect : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredFilledRect(const std::uint32_t color = 0xFFFFFFFF, const Eigen::Vector2f& size = {15.0f, 15.0f}, const Eigen::Vector2f& offset = {0.0f, 0.0f});
  virtual ~HoveredFilledRect() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const std::uint32_t color;
  const Eigen::Vector2f size;
  const Eigen::Vector2f offset;
};

class HoveredImage : public HoveredDrawing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoveredImage(
    const std::shared_ptr<glk::Texture>& texture,
    const Eigen::Vector2f& size = {0.0f, 0.0f},
    const Eigen::Vector2f& offset = {0.0f, 0.0f},
    const std::uint32_t bg_color = 0,
    const std::uint32_t border_color = 0,
    const float border_thickness = 1.0f);
  virtual ~HoveredImage() override;

  virtual void draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) override;

private:
  const Eigen::Vector2f size;
  const Eigen::Vector2f offset;

  const std::uint32_t bg_color;
  const std::uint32_t border_color;
  const float border_thickness;

  const std::shared_ptr<glk::Texture> texture;
};

} // namespace guik

#endif