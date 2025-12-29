#include <guik/viewer/shader_setting.hpp>

namespace guik {

ShaderSetting::ShaderSetting() : ShaderSetting::ShaderSetting(ColorMode::FLAT_COLOR, Eigen::Matrix4f::Identity().eval()) {}

ShaderSetting::ShaderSetting(int color_mode) : ShaderSetting::ShaderSetting(color_mode, Eigen::Matrix4f::Identity().eval()) {}

ShaderSetting::ShaderSetting(int color_mode, const Eigen::Matrix4f& transform) : transparent(false), params(6) {
  params[0] = glk::make_unique<ShaderParameter<int>>("color_mode", color_mode);
  params[1] = glk::make_unique<ShaderParameter<float>>("point_scale", 1.0f);
  params[2] = glk::make_unique<ShaderParameter<Eigen::Matrix4f>>("model_matrix", transform);
}

ShaderSetting::ShaderSetting(const ShaderSetting& other) : transparent(other.transparent), params(other.params.size()) {
  std::transform(other.params.begin(), other.params.end(), params.begin(), [](const auto& p) { return p ? p->clone() : nullptr; });
}

ShaderSetting& ShaderSetting::operator=(const ShaderSetting& other) {
  if (this != &other) {
    transparent = other.transparent;
    params.resize(other.params.size());
    std::transform(other.params.begin(), other.params.end(), params.begin(), [](const auto& p) { return p ? p->clone() : nullptr; });
  }
  return *this;
}

ShaderSetting ShaderSetting::clone() const {
  ShaderSetting cloned;
  cloned.transparent = transparent;
  cloned.params.resize(params.size());
  std::transform(params.begin(), params.end(), cloned.params.begin(), [](const auto& p) { return p ? p->clone() : nullptr; });
  return cloned;
}

void ShaderSetting::set(glk::GLSLShader& shader) const {
  for (const auto& param : params) {
    if (!param) {
      continue;
    }
    param->set(shader);
  }
}

ShaderSetting& ShaderSetting::static_object() {
  return add("dynamic_object", 0);
}

ShaderSetting& ShaderSetting::dynamic_object() {
  return add("dynamic_object", 1);
}

Eigen::Vector4f ShaderSetting::material_color() const {
  return cast<Eigen::Vector4f>("material_color");
}

ShaderSetting& ShaderSetting::set_color(float r, float g, float b, float a) {
  if (a < 0.999f) {
    transparent = true;
  }
  return add("material_color", Eigen::Vector4f(r, g, b, a));
}

ShaderSetting& ShaderSetting::set_alpha(float alpha) {
  Eigen::Vector4f color(1.0f, 1.0f, 1.0f, 1.0f);
  auto found = this->get<Eigen::Vector4f>("material_color");
  if (found) {
    color = found.value();
  }

  color.w() = alpha;
  this->add("material_color", color);
  transparent = alpha < 0.999f;
  return *this;
}

ShaderSetting& ShaderSetting::make_transparent() {
  transparent = true;
  return *this;
}

float ShaderSetting::point_scale() const {
  auto p = static_cast<ShaderParameter<float>*>(params[1].get());
  return p->value;
}

ShaderSetting& ShaderSetting::set_point_scale(float scaling) {
  auto p = static_cast<ShaderParameter<float>*>(params[1].get());
  p->value = scaling;
  return *this;
}

float ShaderSetting::point_size() const {
  return get<float>("point_size").value_or(1.0f);
}
ShaderSetting& ShaderSetting::set_point_size(float size) {
  return add("point_size", size);
}
ShaderSetting& ShaderSetting::set_point_size_offset(float offset) {
  return add("point_size_offset", offset);
}

// Point shape mode
ShaderSetting& ShaderSetting::set_point_shape_mode(guik::PointShapeMode::MODE mode) {
  return add("point_shape_mode", mode);
}
ShaderSetting& ShaderSetting::set_point_shape_circle() {
  return set_point_shape_mode(guik::PointShapeMode::CIRCLE);
}
ShaderSetting& ShaderSetting::set_point_shape_rectangle() {
  return set_point_shape_mode(guik::PointShapeMode::RECTANGLE);
}

// Point scale mode
ShaderSetting& ShaderSetting::set_point_scale_mode(guik::PointScaleMode::MODE mode) {
  return add("point_scale_mode", mode);
}
ShaderSetting& ShaderSetting::set_point_scale_screenspace() {
  return set_point_scale_mode(guik::PointScaleMode::SCREENSPACE);
}
ShaderSetting& ShaderSetting::set_point_scale_metric() {
  return set_point_scale_mode(guik::PointScaleMode::METRIC);
}

ShaderSetting& ShaderSetting::set_point_shape(float point_size, bool metric, bool circle) {
  set_point_size(point_size);
  if (metric) {
    set_point_scale_metric();
  } else {
    set_point_scale_screenspace();
  }
  if (circle) {
    set_point_shape_circle();
  } else {
    set_point_shape_rectangle();
  }
  return *this;
}

ShaderSetting& ShaderSetting::set_old_default_point_shape() {
  set_point_scale_screenspace();
  set_point_shape_rectangle();
  set_point_size(10.0f);
  return *this;
}

ShaderSetting& ShaderSetting::remove_model_matrix() {
  params[2] = nullptr;
  return *this;
}

int ShaderSetting::color_mode() const {
  auto p = static_cast<ShaderParameter<int>*>(params[0].get());
  return p->value;
}

ShaderSetting& ShaderSetting::set_color_mode(int color_mode) {
  auto p = static_cast<ShaderParameter<int>*>(params[0].get());
  p->value = color_mode;
  return *this;
}

Eigen::Matrix4f ShaderSetting::model_matrix() const {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  return p->value;
}

Eigen::Vector3f ShaderSetting::translation() const {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  return p->value.block<3, 1>(0, 3);
}

Eigen::Matrix3f ShaderSetting::rotation() const {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  return p->value.block<3, 3>(0, 0);
}

ShaderSetting& ShaderSetting::translate(float tx, float ty, float tz) {
  return translate(Eigen::Vector3f(tx, ty, tz));
}

ShaderSetting& ShaderSetting::translate(const Eigen::Vector3f& translation) {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  p->value.block<3, 1>(0, 3) += translation;
  return *this;
}

ShaderSetting& ShaderSetting::rotate(const float angle, const Eigen::Vector3f& axis) {
  const Eigen::Vector3f ax = axis.eval().template cast<float>();
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  p->value = p->value * (Eigen::Isometry3f::Identity() * Eigen::AngleAxisf(angle, axis)).matrix();
  return *this;
}

ShaderSetting& ShaderSetting::scale(float scaling) {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  p->value.block<4, 3>(0, 0) *= scaling;
  return *this;
}

ShaderSetting& ShaderSetting::scale(float sx, float sy, float sz) {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  p->value.col(0) *= sx;
  p->value.col(1) *= sy;
  p->value.col(2) *= sz;
  return *this;
}

ShaderSetting& ShaderSetting::scale(const Eigen::Vector3f& scaling) {
  auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
  p->value.col(0) *= scaling[0];
  p->value.col(1) *= scaling[1];
  p->value.col(2) *= scaling[2];
  return *this;
}

}  // namespace guik
