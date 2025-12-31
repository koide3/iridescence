#include <guik/viewer/shader_setting.hpp>

namespace guik {

ShaderSetting::ShaderSetting() : ShaderSetting::ShaderSetting(ColorMode::FLAT_COLOR, Eigen::Matrix4f::Identity().eval()) {}

ShaderSetting::ShaderSetting(int color_mode) : ShaderSetting::ShaderSetting(color_mode, Eigen::Matrix4f::Identity().eval()) {}

ShaderSetting::ShaderSetting(int color_mode, const Eigen::Matrix4f& transform) : transparent(false) {
  params.reserve(6);
  params.emplace_back(ShaderParameter(glk::hash("color_mode"), color_mode));
  params.emplace_back(ShaderParameter(glk::hash("point_scale"), 1.0f));
  params.emplace_back(ShaderParameter(glk::hash("model_matrix"), transform));
}

ShaderSetting::ShaderSetting(const ShaderSetting& other) : transparent(other.transparent), params(other.params) {}

ShaderSetting& ShaderSetting::operator=(const ShaderSetting& other) {
  if (this != &other) {
    transparent = other.transparent;
    params = other.params;
  }
  return *this;
}

ShaderSetting ShaderSetting::clone() const {
  ShaderSetting cloned;
  cloned.transparent = transparent;
  cloned.params = params;
  return cloned;
}

void ShaderSetting::set(glk::GLSLShader& shader) const {
  for (const auto& param : params) {
    if (!param.valid()) {
      continue;
    }
    param.set(shader);
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

ShaderSetting& ShaderSetting::set_rainbow_range(const Eigen::Vector2f& minmax_z) {
  return add("z_range", minmax_z);
}

ShaderSetting& ShaderSetting::set_rainbow_axis(const Eigen::Vector3f& axis) {
  return add("colormap_axis", axis);
}

float ShaderSetting::point_scale() const {
  auto& p = params[1];
  return p.get_value<float>();
}

ShaderSetting& ShaderSetting::set_point_scale(float scaling) {
  auto& p = params[1];
  p.set_value<float>(scaling);
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
  params[2].invalidate();
  return *this;
}

int ShaderSetting::color_mode() const {
  const auto& p = params[0];
  return p.get_value<int>();
}

ShaderSetting& ShaderSetting::set_color_mode(int color_mode) {
  auto& p = params[0];
  p.set_value<int>(color_mode);
  return *this;
}

Eigen::Matrix4f ShaderSetting::model_matrix() const {
  const auto& p = params[2];
  return p.get_value<Eigen::Matrix4f>();
}

Eigen::Vector3f ShaderSetting::translation() const {
  const auto& p = params[2];
  return p.get_value<Eigen::Matrix4f>().block<3, 1>(0, 3);
}

Eigen::Matrix3f ShaderSetting::rotation() const {
  const auto& p = params[2];
  return p.get_value<Eigen::Matrix4f>().block<3, 3>(0, 0);
}

ShaderSetting& ShaderSetting::translate(float tx, float ty, float tz) {
  return translate(Eigen::Vector3f(tx, ty, tz));
}

ShaderSetting& ShaderSetting::translate(const Eigen::Vector3f& translation) {
  auto& p = params[2];
  p.mat4f.block<3, 1>(0, 3) += translation;
  return *this;
}

ShaderSetting& ShaderSetting::rotate(const float angle, const Eigen::Vector3f& axis) {
  const Eigen::Vector3f ax = axis.eval().template cast<float>();

  auto& p = params[2];
  p.mat4f = p.mat4f * (Eigen::Isometry3f::Identity() * Eigen::AngleAxisf(angle, ax.normalized())).matrix();
  return *this;
}

ShaderSetting& ShaderSetting::scale(float scaling) {
  auto& p = params[2];
  p.mat4f.block<4, 3>(0, 0) *= scaling;
  return *this;
}

ShaderSetting& ShaderSetting::scale(float sx, float sy, float sz) {
  auto& p = params[2];
  p.mat4f.col(0) *= sx;
  p.mat4f.col(1) *= sy;
  p.mat4f.col(2) *= sz;
  return *this;
}

ShaderSetting& ShaderSetting::scale(const Eigen::Vector3f& scaling) {
  auto& p = params[2];
  p.mat4f.col(0) *= scaling[0];
  p.mat4f.col(1) *= scaling[1];
  p.mat4f.col(2) *= scaling[2];
  return *this;
}

}  // namespace guik
