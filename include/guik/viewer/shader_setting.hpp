#ifndef GUIK_SHADER_SETTING_HPP
#define GUIK_SHADER_SETTING_HPP

#include <memory>
#include <unordered_map>
#include <ros/package.h>

#include <glk/drawble.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>

namespace guik {

struct ShaderParameterInterface {
public:
  using Ptr = std::shared_ptr<ShaderParameterInterface>;
  virtual ~ShaderParameterInterface() {}

  virtual void set(glk::GLSLShader& shader) const = 0;
};

template<typename T>
struct ShaderParameter : public ShaderParameterInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ShaderParameter(const std::string& name, const T& value) : name(name), value(value) {}
  virtual ~ShaderParameter() override {}

  virtual void set(glk::GLSLShader& shader) const override {
    shader.set_uniform(name, value);
  }

public:
  std::string name;
  T value;
};

struct ShaderSetting {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<ShaderSetting>;

  ShaderSetting() {
    add("color_mode", 1);
    add("point_scale", 1.0f);
    add("model_matrix", Eigen::Matrix4f::Identity().eval());
  }

  template<typename T>
  ShaderSetting& add(const std::string& name, const T& value) {
    params.push_back(std::make_shared<ShaderParameter<T>>(name, value));
    return *this;
  }

  void set(glk::GLSLShader& shader) const {
    for(const auto& param: params) {
      param->set(shader);
    }
  }

public:
  std::vector<ShaderParameterInterface::Ptr> params;
};

}  // namespace guik

#endif