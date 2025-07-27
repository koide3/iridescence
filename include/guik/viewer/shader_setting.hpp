#ifndef GUIK_SHADER_SETTING_HPP
#define GUIK_SHADER_SETTING_HPP

#include <memory>
#include <optional>
#include <unordered_map>

#include <glk/drawable.hpp>
#include <glk/make_shared.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>

namespace guik {

struct ColorMode {
  enum MODE { RAINBOW = 0, FLAT_COLOR = 1, VERTEX_COLOR = 2, TEXTURE_COLOR = 3 };
};

struct PointScaleMode {
  enum MODE { SCREENSPACE = 0, METRIC = 1 };
};

struct PointShapeMode {
  enum MODE { RECTANGLE = 0, CIRCLE = 1 };
};

struct ShaderParameterInterface {
public:
  using Ptr = std::shared_ptr<ShaderParameterInterface>;
  ShaderParameterInterface(const std::string& name) : name(name) {}
  virtual ~ShaderParameterInterface() {}

  virtual void set(glk::GLSLShader& shader) const = 0;
  virtual Ptr clone() const = 0;

public:
  std::string name;
};

template <typename T>
struct ShaderParameter : public ShaderParameterInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ShaderParameter(const std::string& name, const T& value) : ShaderParameterInterface(name), value(value) {}
  virtual ~ShaderParameter() override {}

  virtual void set(glk::GLSLShader& shader) const override { shader.set_uniform(name, value); }

  virtual Ptr clone() const override { return glk::make_shared<ShaderParameter<T>>(name, value); }

public:
  T value;
};

/// @brief Shader setting class holds rendering settings for the object.
struct ShaderSetting {
public:
  using Ptr = std::shared_ptr<ShaderSetting>;

  ShaderSetting()
  : transparent(false),
    params(
      {glk::make_shared<ShaderParameter<int>>("color_mode", ColorMode::FLAT_COLOR),
       glk::make_shared<ShaderParameter<float>>("point_scale", 1.0f),
       glk::make_shared<ShaderParameter<Eigen::Matrix4f>>("model_matrix", Eigen::Matrix4f::Identity())})  //
  {}

  ShaderSetting(int color_mode)
  : transparent(false),
    params(
      {glk::make_shared<ShaderParameter<int>>("color_mode", color_mode),
       glk::make_shared<ShaderParameter<float>>("point_scale", 1.0f),
       glk::make_shared<ShaderParameter<Eigen::Matrix4f>>("model_matrix", Eigen::Matrix4f::Identity())})  //
  {}

  template <typename Transform>
  ShaderSetting(int color_mode, const Transform& transform)
  : transparent(false),
    params(
      {glk::make_shared<ShaderParameter<int>>("color_mode", color_mode),
       glk::make_shared<ShaderParameter<float>>("point_scale", 1.0f),
       glk::make_shared<ShaderParameter<Eigen::Matrix4f>>("model_matrix", (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix())})  //
  {}

  virtual ~ShaderSetting() {}

  ShaderSetting clone() const {
    ShaderSetting cloned;
    cloned.transparent = transparent;
    cloned.params.resize(params.size());
    std::transform(params.begin(), params.end(), cloned.params.begin(), [](const auto& p) { return p->clone(); });
    return cloned;
  }

  /// @brief Add a new parameter to the shader setting.
  /// @param name   Parameter name
  /// @param value  Parameter value
  /// @return       This object
  template <typename T>
  ShaderSetting& add(const std::string& name, const T& value) {
    for (int i = 0; i < params.size(); i++) {
      if (params[i]->name == name) {
        auto p = dynamic_cast<ShaderParameter<T>*>(params[i].get());
        if (p) {
          p->value = value;
        } else {
          params[i].reset(new ShaderParameter<T>(name, value));
        }
        return *this;
      }
    }

    params.push_back(glk::make_shared<ShaderParameter<T>>(name, value));
    return *this;
  }

  /// @brief Get a parameter value from the shader setting.
  /// @param name   Parameter name
  /// @return       Parameter value if found, otherwise std::nullopt
  template <typename T>
  std::optional<T> get(const std::string& name) {
    for (const auto& param : params) {
      if (param->name != name) {
        continue;
      }

      auto p = dynamic_cast<ShaderParameter<T>*>(param.get());
      if (p == nullptr) {
        continue;
      }

      return p->value;
    }

    return std::nullopt;
  }

  /// @brief Get a parameter value from the shader setting. This method avoid type checking. Fast but unsafe.
  /// @param name   Parameter name
  /// @return       Parameter value
  template <typename T>
  const T& cast(const std::string& name) const {
    for (const auto& param : params) {
      if (param->name != name) {
        continue;
      }

      auto p = static_cast<ShaderParameter<T>*>(param.get());
      return p->value;
    }

    std::cerr << "error: " << name << " not found in the param list" << std::endl;
    abort();
  }

  /// @brief Set shader parameters to the shader program.
  /// @param shader  Shader program
  void set(glk::GLSLShader& shader) const {
    for (const auto& param : params) {
      if (!param) {
        continue;
      }
      param->set(shader);
    }
  }

  // Object type
  ShaderSetting& static_object() { return add("dynamic_object", 0); }

  ShaderSetting& dynamic_object() { return add("dynamic_object", 1); }

  ShaderSetting& dymamic_object() {
    std::cerr << "warning : dymamic_object() is deprecated. Use dynamic_object() instead. Sorry for the silly typo!!" << std::endl;
    return add("dynamic_object", 1);
  }

  // Color
  Eigen::Vector4f material_color() const { return cast<Eigen::Vector4f>("material_color"); }

  ShaderSetting& set_color(float r, float g, float b, float a) {
    if (a < 0.999f) {
      transparent = true;
    }
    return add("material_color", Eigen::Vector4f(r, g, b, a));
  }

  template <typename Color>
  ShaderSetting& set_color(const Color& color_) {
    const Eigen::Vector4f color = color_.template cast<float>();
    if (color.w() < 0.999f) {
      transparent = true;
    }
    return add("material_color", color);
  }

  ShaderSetting& set_alpha(float alpha) {
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

  ShaderSetting& make_transparent() {
    transparent = true;
    return *this;
  }

  // Point size and scale
  float point_scale() const {
    auto p = static_cast<ShaderParameter<float>*>(params[1].get());
    return p->value;
  }

  ShaderSetting& set_point_scale(float scaling) {
    auto p = static_cast<ShaderParameter<float>*>(params[1].get());
    p->value = scaling;
    return *this;
  }

  ShaderSetting& set_point_size(float size) { return add("point_size", size); }
  ShaderSetting& set_point_size_offset(float offset) { return add("point_size_offset", offset); }

  // Point shape mode
  ShaderSetting& set_point_shape_mode(guik::PointShapeMode::MODE mode) { return add("point_shape_mode", mode); }
  ShaderSetting& set_point_shape_circle() { return set_point_shape_mode(guik::PointShapeMode::CIRCLE); }
  ShaderSetting& set_point_shape_rectangle() { return set_point_shape_mode(guik::PointShapeMode::RECTANGLE); }

  // Point scale mode
  ShaderSetting& set_point_scale_mode(guik::PointScaleMode::MODE mode) { return add("point_scale_mode", mode); }
  ShaderSetting& set_point_scale_screenspace() { return set_point_scale_mode(guik::PointScaleMode::SCREENSPACE); }
  ShaderSetting& set_point_scale_metric() { return set_point_scale_mode(guik::PointScaleMode::METRIC); }

  // Calling `remove_model_matrix` invalidates model matrix operations (translate, rotate, and scale)
  ShaderSetting& remove_model_matrix() {
    params[2] = nullptr;
    return *this;
  }

  int color_mode() const {
    auto p = static_cast<ShaderParameter<int>*>(params[0].get());
    return p->value;
  }

  ShaderSetting& set_color_mode(int color_mode) {
    auto p = static_cast<ShaderParameter<int>*>(params[0].get());
    p->value = color_mode;
    return *this;
  }

  Eigen::Matrix4f model_matrix() const {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    return p->value;
  }

  Eigen::Vector3f translation() const {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    return p->value.block<3, 1>(0, 3);
  }

  Eigen::Matrix3f rotation() const {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    return p->value.block<3, 3>(0, 0);
  }

  template <typename Transform>
  ShaderSetting& set_model_matrix(const Transform& transform) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix();
    return *this;
  }

  template <typename Transform>
  ShaderSetting& transform(const Transform& transform) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = p->value * (Eigen::Isometry3f::Identity() * transform.template cast<float>()).matrix();
    return *this;
  }

  ShaderSetting& translate(float tx, float ty, float tz) { return translate(Eigen::Vector3f(tx, ty, tz)); }

  ShaderSetting& translate(const Eigen::Vector3f& translation) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value.block<3, 1>(0, 3) += translation;
    return *this;
  }

  template <typename Vector>
  ShaderSetting& translate(const Vector& translation) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    const auto trans = translation.eval();
    p->value.block<3, 1>(0, 3) += trans.template cast<float>().template head<3>();
    return *this;
  }

  ShaderSetting& rotate(const float angle, const Eigen::Vector3f& axis) {
    const Eigen::Vector3f ax = axis.eval().template cast<float>();
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = p->value * (Eigen::Isometry3f::Identity() * Eigen::AngleAxisf(angle, axis)).matrix();
    return *this;
  }

  template <typename Vector>
  ShaderSetting& rotate(const float angle, const Vector& axis) {
    const Eigen::Vector3f ax = axis.eval().template cast<float>();
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = p->value * (Eigen::Isometry3f::Identity() * Eigen::AngleAxisf(angle, ax)).matrix();
    return *this;
  }

  template <typename Scalar>
  ShaderSetting& rotate(const Eigen::Quaternion<Scalar>& quat) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = p->value * (Eigen::Isometry3f::Identity() * quat.template cast<float>()).matrix();
    return *this;
  }

  template <typename Scalar, int Dim>
  ShaderSetting& rotate(const Eigen::Matrix<Scalar, Dim, Dim>& rot) {
    Eigen::Isometry3f R = Eigen::Isometry3f::Identity();
    R.linear() = rot.template cast<float>().template block<3, 3>(0, 0);

    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = p->value * R.matrix();
    return *this;
  }

  ShaderSetting& scale(float scaling) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value.block<4, 3>(0, 0) *= scaling;
    return *this;
  }

  ShaderSetting& scale(float sx, float sy, float sz) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value.col(0) *= sx;
    p->value.col(1) *= sy;
    p->value.col(2) *= sz;
    return *this;
  }

  ShaderSetting& scale(const Eigen::Vector3f& scaling) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value.col(0) *= scaling[0];
    p->value.col(1) *= scaling[1];
    p->value.col(2) *= scaling[2];
    return *this;
  }

  template <typename Vector>
  std::enable_if_t<!std::is_arithmetic_v<Vector>, ShaderSetting&> scale(const Vector& scaling) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    const auto s = scaling.eval();
    p->value.col(0) *= s[0];
    p->value.col(1) *= s[1];
    p->value.col(2) *= s[2];
    return *this;
  }

public:
  bool transparent;
  std::vector<ShaderParameterInterface::Ptr> params;
};

template <>
inline ShaderSetting& ShaderSetting::add(const std::string& name, const Eigen::Isometry3f& value) {
  return add<Eigen::Matrix4f>(name, value.matrix());
}

template <>
inline ShaderSetting& ShaderSetting::add(const std::string& name, const Eigen::Affine3f& value) {
  return add<Eigen::Matrix4f>(name, value.matrix());
}

/// @brief Rainbow coloring scheme that assigns colors to the height (z-value) of pixels of the object.
///        The coloring band can be changed by setting `z_range` param.
///        The direction for the color mapping can be changed by setting `colormap_axis` param.
struct Rainbow : public ShaderSetting {
public:
  Rainbow() : ShaderSetting(ColorMode::RAINBOW) {}

  template <typename Transform>
  Rainbow(const Transform& transform) : ShaderSetting(ColorMode::RAINBOW, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~Rainbow() override {}
};

/// @brief Flat coloring scheme that assigns a single color to the object.
///        If the alpha value is less than 0.999f, the object is rendered as transparent.
struct FlatColor : public ShaderSetting {
public:
  explicit FlatColor(float r, float g, float b, float a = 1.0f) : ShaderSetting(ColorMode::FLAT_COLOR) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", Eigen::Vector4f(r, g, b, a)));
    transparent = a < 0.999f;
  }

  explicit FlatColor(const Eigen::Vector4f& color) : ShaderSetting(ColorMode::FLAT_COLOR) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
    transparent = color.w() < 0.999f;
  }

  template <typename Transform>
  explicit FlatColor(const Eigen::Vector4f& color, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
    transparent = color.w() < 0.999f;
  }

  template <typename Color>
  explicit FlatColor(const Color& color) : ShaderSetting(ColorMode::FLAT_COLOR) {
    const Eigen::Vector4f c = color.eval().template cast<float>();
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", c));
    transparent = color.w() < 0.999f;
  }

  template <typename Transform>
  explicit FlatColor(float r, float g, float b, float a, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", Eigen::Vector4f(r, g, b, a)));
    transparent = a < 0.999f;
  }

  template <typename Color, typename Transform>
  explicit FlatColor(const Color& color, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    const Eigen::Vector4f c = color.eval().template cast<float>();
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", c));
    transparent = color.w() < 0.999f;
  }

  template <typename T>
  explicit FlatColor(std::initializer_list<T> i) : ShaderSetting(ColorMode::FLAT_COLOR) {
    Eigen::Vector4f color = Eigen::Vector4f::Zero();
    std::copy(i.begin(), i.end(), color.data());
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
    transparent = color.w() < 0.999f;
  }

  template <typename T, typename Transform>
  explicit FlatColor(std::initializer_list<T> i, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    Eigen::Vector4f color = Eigen::Vector4f::Zero();
    std::copy(i.begin(), i.end(), color.data());
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
    transparent = color.w() < 0.999f;
  }

  virtual ~FlatColor() override {}
};

/* primitive colors */
/// @brief Flat red color
struct FlatRed : public FlatColor {
  FlatRed() : FlatColor(1.0f, 0.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatRed(const Transform& transform) : FlatColor(1.0f, 0.0f, 0.0f, 1.0f, transform) {}
};

/// @brief Flat green color
struct FlatGreen : public FlatColor {
  FlatGreen() : FlatColor(0.0f, 1.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatGreen(const Transform& transform) : FlatColor(0.0f, 1.0f, 0.0f, 1.0f, transform) {}
};

/// @brief Flat blue color
struct FlatBlue : public FlatColor {
  FlatBlue() : FlatColor(0.0f, 0.0f, 1.0f, 1.0f) {}

  template <typename Transform>
  FlatBlue(const Transform& transform) : FlatColor(0.0f, 0.0f, 1.0f, 1.0f, transform) {}
};

/// @brief Flat orange color
struct FlatOrange : public FlatColor {
  FlatOrange() : FlatColor(1.0f, 0.5f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatOrange(const Transform& transform) : FlatColor(1.0f, 0.5f, 0.0f, 1.0f, transform) {}
};

/// @brief Flat white color
struct FlatWhite : public FlatColor {
  FlatWhite() : FlatColor(1.0f, 1.0f, 1.0f, 1.0f) {}

  template <typename Transform>
  FlatWhite(const Transform& transform) : FlatColor(1.0f, 1.0f, 1.0f, 1.0f, transform) {}
};

/// @brief Flat light gray color
struct FlatGray : public FlatColor {
  FlatGray() : FlatColor(0.7f, 0.7f, 0.7f, 1.0f) {}

  template <typename Transform>
  FlatGray(const Transform& transform) : FlatColor(0.7f, 0.7f, 0.7f, 1.0f, transform) {}
};

/// @brief Flat dark gray color
struct FlatDarkGray : public FlatColor {
  FlatDarkGray() : FlatColor(0.3f, 0.3f, 0.3f, 1.0f) {}

  template <typename Transform>
  FlatDarkGray(const Transform& transform) : FlatColor(0.3f, 0.3f, 0.3f, 1.0f, transform) {}
};

/// @brief Flat black color
struct FlatBlack : public FlatColor {
  FlatBlack() : FlatColor(0.0f, 0.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatBlack(const Transform& transform) : FlatColor(0.0f, 0.0f, 0.0f, 1.0f, transform) {}
};

/// @brief Vertex color scheme that assigns colors based on vertex color attributes.
struct VertexColor : public ShaderSetting {
public:
  VertexColor() : ShaderSetting(ColorMode::VERTEX_COLOR) {}

  template <typename Transform>
  VertexColor(const Transform& transform) : ShaderSetting(ColorMode::VERTEX_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~VertexColor() override {}
};

/// @brief Texture color scheme that assigns colors based on the binded texture.
struct TextureColor : public ShaderSetting {
public:
  TextureColor() : ShaderSetting(ColorMode::TEXTURE_COLOR) {}

  template <typename Transform>
  TextureColor(const Transform& transform) : ShaderSetting(ColorMode::TEXTURE_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~TextureColor() override {}
};

}  // namespace guik

#endif