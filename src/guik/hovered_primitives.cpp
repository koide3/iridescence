#include <guik/hovered_primitives.hpp>

namespace guik {

// HoveredText
HoveredText::HoveredText(const std::string& text, const std::uint32_t fg_color, const std::uint32_t bg_color, const Eigen::Vector2f& offset)
: text(text),
  fg_color(fg_color),
  bg_color(bg_color),
  offset(offset) {}

HoveredText::~HoveredText() {}

void HoveredText::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  const ImVec2 text_size = ImGui::CalcTextSize(text.data(), text.data() + text.size());
  const Eigen::Vector2f tl = uvz.head<2>() - Eigen::Vector2f(text_size[0] / 2, text_size[1] / 2) + offset;

  if (bg_color) {
    drawlist->AddRectFilled(ImVec2(tl.x(), tl.y()), ImVec2(tl.x() + text_size.x, tl.y() + text_size.y), bg_color);
  }

  drawlist->AddText(ImVec2(tl.x(), tl.y()), fg_color, text.data(), text.data() + text.size());
}

// HoveredCross
HoveredCross::HoveredCross(const std::uint32_t color, const float size, const float thickness) : color(color), size(size), thickness(thickness) {}

HoveredCross::~HoveredCross() {}

void HoveredCross::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  drawlist->AddLine(ImVec2(uvz.x() - size, uvz.y() - size), ImVec2(uvz.x() + size, uvz.y() + size), color, thickness);
  drawlist->AddLine(ImVec2(uvz.x() + size, uvz.y() - size), ImVec2(uvz.x() - size, uvz.y() + size), color, thickness);
}

// HoveredCircle
HoveredCircle::HoveredCircle(const std::uint32_t color, const float radius, const int num_segments, const float thickness)
: color(color),
  radius(radius),
  num_segments(num_segments),
  thickness(thickness) {}

HoveredCircle::~HoveredCircle() {}

void HoveredCircle::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  drawlist->AddCircle(ImVec2(uvz.x(), uvz.y()), radius, color, num_segments, thickness);
}

// HoveredTriangle
HoveredTriangle::HoveredTriangle(const std::uint32_t color, const float height, const float thickness, const bool upsidedown, const bool centering)
: color(color),
  half_width(height / 1.73),
  height(upsidedown ? -height : height),
  thickness(thickness),
  centering(centering) {}

HoveredTriangle::~HoveredTriangle() {}

void HoveredTriangle::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  const Eigen::Vector2f offset = centering ? Eigen::Vector2f(0.0f, -height / 2.0f) : Eigen::Vector2f(0.0f, 0.0f);
  const Eigen::Vector2f tl = uvz.head<2>() + offset;
  drawlist->AddTriangle(ImVec2(tl.x(), tl.y()), ImVec2(tl.x() + half_width, tl.y() + height), ImVec2(tl.x() - half_width, tl.y() + height), color, thickness);
}

// HoveredFilledTriangle
HoveredFilledTriangle::HoveredFilledTriangle(const std::uint32_t color, const float height, const bool upsidedown, const bool centering)
: color(color),
  half_width(height / 1.73),
  height(upsidedown ? -height : height),
  centering(centering) {}

HoveredFilledTriangle::~HoveredFilledTriangle() {}

void HoveredFilledTriangle::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  const Eigen::Vector2f offset = centering ? Eigen::Vector2f(0.0f, -height / 2.0f) : Eigen::Vector2f(0.0f, 0.0f);
  const Eigen::Vector2f tl = uvz.head<2>() + offset;
  drawlist->AddTriangleFilled(ImVec2(tl.x(), tl.y()), ImVec2(tl.x() + half_width, tl.y() + height), ImVec2(tl.x() - half_width, tl.y() + height), color);
}

// HoveredRect
HoveredRect::HoveredRect(const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset) : color(color), size(size), offset(offset) {}
HoveredRect::~HoveredRect() {}

void HoveredRect::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  const Eigen::Vector2f tl = uvz.head<2>() - size * 0.5f + offset;
  const Eigen::Vector2f br = tl + size;
  drawlist->AddRect(ImVec2(tl.x(), tl.y()), ImVec2(br.x(), br.y()), color);
}

// HoveredFilledRect
HoveredFilledRect::HoveredFilledRect(const std::uint32_t color, const Eigen::Vector2f& size, const Eigen::Vector2f& offset)
: color(color),
  size(size),
  offset(offset) {}
HoveredFilledRect::~HoveredFilledRect() {}

void HoveredFilledRect::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  const Eigen::Vector2f tl = uvz.head<2>() - size * 0.5f + offset;
  const Eigen::Vector2f br = tl + size;
  drawlist->AddRectFilled(ImVec2(tl.x(), tl.y()), ImVec2(br.x(), br.y()), color);
}

// HoveredImage
HoveredImage::HoveredImage(
  const std::shared_ptr<glk::Texture>& texture,
  const Eigen::Vector2f& size,
  const Eigen::Vector2f& offset,
  const std::uint32_t bg_color,
  const std::uint32_t border_color,
  const float border_thickness)
: texture(texture),
  size(size[0] < 1.0f ? texture->size().cast<float>() : size),
  offset(offset),
  bg_color(bg_color),
  border_color(border_color),
  border_thickness(border_thickness) {}

HoveredImage::~HoveredImage() {}

void HoveredImage::draw(ImDrawList* drawlist, const Eigen::Vector3f& uvz) {
  const Eigen::Vector2f tl = uvz.head<2>() - size / 2.0f + offset;
  const Eigen::Vector2f br = tl + size;

  if(bg_color) {
    drawlist->AddRectFilled(ImVec2(tl.x(), tl.y()), ImVec2(br.x(), br.y()), bg_color);
  }

  drawlist->AddImage(reinterpret_cast<void*>(texture->id()), ImVec2(tl.x(), tl.y()), ImVec2(br.x(), br.y()), ImVec2(0, 0), ImVec2(1, 1));

  if (border_color) {
    drawlist->AddRect(ImVec2(tl.x(), tl.y()), ImVec2(br.x(), br.y()), border_color, 0.0f, 15, border_thickness);
  }
}

}  // namespace guik