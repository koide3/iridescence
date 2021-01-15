#ifndef GUIK_SHADER_SETTING_HPP
#define GUIK_SHADER_SETTING_HPP

#include <memory>
#include <unordered_map>
#include <boost/optional.hpp>

#include <glk/drawble.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>

namespace guik {

struct ColorMode {
  enum MODE { RAINBOW = 0, FLAT_COLOR = 1, VERTEX_COLOR = 2 };
};

struct ShaderParameterInterface {
public:
  using Ptr = std::shared_ptr<ShaderParameterInterface>;
  ShaderParameterInterface(const std::string& name) : name(name) {}
  virtual ~ShaderParameterInterface() {}

  virtual void set(glk::GLSLShader& shader) const = 0;

public:
  std::string name;
};

template<typename T>
struct ShaderParameter : public ShaderParameterInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ShaderParameter(const std::string& name, const T& value) : ShaderParameterInterface(name), value(value) {}
  virtual ~ShaderParameter() override {}

  virtual void set(glk::GLSLShader& shader) const override {
    shader.set_uniform(name, value);
  }

public:
  T value;
};

struct ShaderSetting {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<ShaderSetting>;

  ShaderSetting() : transparent(false) {
    params.resize(3);
    params[0].reset(new ShaderParameter<int>("color_mode", ColorMode::FLAT_COLOR));
    params[1].reset(new ShaderParameter<float>("point_scale", 1.0f));
    params[2].reset(new ShaderParameter<Eigen::Matrix4f>("model_matrix", Eigen::Matrix4f::Identity()));
  }

  ShaderSetting(int color_mode) : transparent(false) {
    params.resize(3);
    params[0].reset(new ShaderParameter<int>("color_mode", color_mode));
    params[1].reset(new ShaderParameter<float>("point_scale", 1.0f));
    params[2].reset(new ShaderParameter<Eigen::Matrix4f>("model_matrix", Eigen::Matrix4f::Identity()));
  }

  template<typename Transform>
  ShaderSetting(int color_mode, const Transform& transform) : transparent(false) {
    params.resize(3);
    params[0].reset(new ShaderParameter<int>("color_mode", color_mode));
    params[1].reset(new ShaderParameter<float>("point_scale", 1.0f));
    params[2].reset(new ShaderParameter<Eigen::Matrix4f>("model_matrix", (transform * Eigen::Isometry3f::Identity()).matrix()));
  }
  virtual ~ShaderSetting() {}

  ShaderSetting& make_transparent() {
    transparent = true;
    return *this;
  }

  template<typename T>
  ShaderSetting& add(const std::string& name, const T& value) {
    for(int i = 0; i < params.size(); i++) {
      if(params[i]->name == name) {
        params[i].reset(new ShaderParameter<T>(name, value));
        return *this;
      }
    }

    params.push_back(std::shared_ptr<ShaderParameter<T>>(new ShaderParameter<T>(name, value)));
    return *this;
  }

  template<typename T>
  boost::optional<T> get(const std::string& name) {
    for(const auto& param : params) {
      if(param->name != name) {
        continue;
      }

      auto p = std::dynamic_pointer_cast<ShaderParameter<T>>(param);
      if(p == nullptr) {
        continue;
      }

      return p->value;
    }

    return boost::none;
  }

  void set(glk::GLSLShader& shader) const {
    for(const auto& param : params) {
      param->set(shader);
    }
  }

public:
  bool transparent;
  std::vector<ShaderParameterInterface::Ptr> params;
};

template<>
inline ShaderSetting& ShaderSetting::add(const std::string& name, const Eigen::Isometry3f& value) {
  return add<Eigen::Matrix4f>(name, value.matrix());
}

struct Rainbow : public ShaderSetting {
public:
  Rainbow() : ShaderSetting(ColorMode::RAINBOW) {}

  template<typename Transform>
  Rainbow(const Transform& transform) : ShaderSetting(ColorMode::RAINBOW, (transform * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~Rainbow() override {}
};

struct FlatColor : public ShaderSetting {
public:
  FlatColor(const Eigen::Vector4f& color) : ShaderSetting(ColorMode::FLAT_COLOR) {
    params.push_back(std::shared_ptr<ShaderParameter<Eigen::Vector4f>>(new ShaderParameter<Eigen::Vector4f>("material_color", color)));
  }

  template<typename Transform>
  FlatColor(const Eigen::Vector4f& color, const Transform& transform) : ShaderSetting(ColorMode::FLAT_COLOR, (transform * Eigen::Isometry3f::Identity()).matrix()) {
    params.push_back(std::shared_ptr<ShaderParameter<Eigen::Vector4f>>(new ShaderParameter<Eigen::Vector4f>("material_color", color)));
  }

  virtual ~FlatColor() override {}
};

struct VertexColor : public ShaderSetting {
public:
  VertexColor() : ShaderSetting(ColorMode::VERTEX_COLOR) {}

  template<typename Transform>
  VertexColor(const Transform& transform) : ShaderSetting(ColorMode::VERTEX_COLOR, (transform * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~VertexColor() override {}
};

}  // namespace guik

#endif