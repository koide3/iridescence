#ifndef GUIK_SHADER_SETTING_HPP
#define GUIK_SHADER_SETTING_HPP

#include <memory>
#include <unordered_map>
#include <boost/optional.hpp>

#include <glk/drawable.hpp>
#include <glk/make_shared.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>

namespace guik {

struct ColorMode {
  enum MODE { RAINBOW = 0, FLAT_COLOR = 1, VERTEX_COLOR = 2, TEXTURE_COLOR = 3 };
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

  ShaderSetting& make_transparent() {
    transparent = true;
    return *this;
  }

  ShaderSetting& static_object() { return add("dynamic_object", 0); }

  ShaderSetting& dymamic_object() { return add("dynamic_object", 1); }

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

  template <typename T>
  boost::optional<T> get(const std::string& name) {
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

    return boost::none;
  }

  template <typename T>
  const T& cast(const std::string& name) {
    for (const auto& param : params) {
      if (param->name != name) {
        continue;
      }

      auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(param.get());
      return p->value;
    }

    std::cerr << "error: " << name << " not found in the param list" << std::endl;
    abort();
  }

  void set(glk::GLSLShader& shader) const {
    for (const auto& param : params) {
      param->set(shader);
    }
  }

  float point_scale() const {
    auto p = static_cast<ShaderParameter<float>*>(params[1].get());
    return p->value;
  }

  ShaderSetting& set_point_scale(float scaling) {
    auto p = static_cast<ShaderParameter<float>*>(params[1].get());
    p->value = scaling;
    return *this;
  }

  Eigen::Matrix4f model_matrix() const {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    return p->value;
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

  template <typename Scalar, int Dim>
  ShaderSetting& translate(const Eigen::Matrix<Scalar, Dim, 1>& translation) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value.block<3, 1>(0, 3) += translation.template cast<float>().template head<3>();
    return *this;
  }

  ShaderSetting& rotate(const float angle, const Eigen::Vector3f& axis) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value = p->value * (Eigen::Isometry3f::Identity() * Eigen::AngleAxisf(angle, axis)).matrix();
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

  template <typename Scalar, int Dim>
  ShaderSetting& scale(const Eigen::Matrix<Scalar, Dim, 1>& scaling) {
    auto p = static_cast<ShaderParameter<Eigen::Matrix4f>*>(params[2].get());
    p->value.col(0) *= scaling[0];
    p->value.col(1) *= scaling[1];
    p->value.col(2) *= scaling[2];
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

struct Rainbow : public ShaderSetting {
public:
  Rainbow() : ShaderSetting(ColorMode::RAINBOW) {}

  template <typename Transform>
  Rainbow(const Transform& transform) : ShaderSetting(ColorMode::RAINBOW, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~Rainbow() override {}
};

struct FlatColor : public ShaderSetting {
public:
  FlatColor(float r, float g, float b, float a = 1.0f) : ShaderSetting(ColorMode::FLAT_COLOR) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", Eigen::Vector4f(r, g, b, a)));
  }

  FlatColor(const Eigen::Vector4f& color) : ShaderSetting(ColorMode::FLAT_COLOR) {  //
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
  }

  template <typename Transform>
  FlatColor(float r, float g, float b, float a, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", Eigen::Vector4f(r, g, b, a)));
  }

  template <typename Transform>
  FlatColor(const Eigen::Vector4f& color, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    params.push_back(glk::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
  }

  virtual ~FlatColor() override {}
};

/* primitive colors */
struct FlatRed : public FlatColor {
  FlatRed() : FlatColor(1.0f, 0.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatRed(const Transform& transform) : FlatColor(1.0f, 0.0f, 0.0f, 1.0f, transform) {}
};

struct FlatGreen : public FlatColor {
  FlatGreen() : FlatColor(0.0f, 1.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatGreen(const Transform& transform) : FlatColor(0.0f, 1.0f, 0.0f, 1.0f, transform) {}
};

struct FlatBlue : public FlatColor {
  FlatBlue() : FlatColor(0.0f, 0.0f, 1.0f, 1.0f) {}

  template <typename Transform>
  FlatBlue(const Transform& transform) : FlatColor(0.0f, 0.0f, 1.0f, 1.0f, transform) {}
};

struct FlatOrange : public FlatColor {
  FlatOrange() : FlatColor(1.0f, 0.5f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatOrange(const Transform& transform) : FlatColor(1.0f, 0.5f, 0.0f, 1.0f, transform) {}
};

struct VertexColor : public ShaderSetting {
public:
  VertexColor() : ShaderSetting(ColorMode::VERTEX_COLOR) {}

  template <typename Transform>
  VertexColor(const Transform& transform) : ShaderSetting(ColorMode::VERTEX_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~VertexColor() override {}
};

struct TextureColor : public ShaderSetting {
public:
  TextureColor() : ShaderSetting(ColorMode::TEXTURE_COLOR) {}

  template <typename Transform>
  TextureColor(const Transform& transform) : ShaderSetting(ColorMode::TEXTURE_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~TextureColor() override {}
};

}  // namespace guik

#endif