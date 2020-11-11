#ifndef GUIK_SHADER_SETTING_HPP
#define GUIK_SHADER_SETTING_HPP

#include <memory>
#include <unordered_map>
#include <boost/optional.hpp>

#include <glk/drawble.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>

namespace guik {

struct ShaderParameterInterface {
public:
  using Ptr = std::shared_ptr<ShaderParameterInterface>;
  ShaderParameterInterface(const std::string& name) : name (name) {}
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

  ShaderSetting() {
    params.resize(3);
    params[0] = std::make_shared<ShaderParameter<int>>("color_mode", 1);
    params[1] = std::make_shared<ShaderParameter<float>>("point_scale", 1.0f);
    params[2] = std::make_shared<ShaderParameter<Eigen::Matrix4f>>("model_matrix", Eigen::Matrix4f::Identity());
  }

  ShaderSetting(int color_mode) {
    params.resize(3);
    params[0] = std::make_shared<ShaderParameter<int>>("color_mode", color_mode);
    params[1] = std::make_shared<ShaderParameter<float>>("point_scale", 1.0f);
    params[2] = std::make_shared<ShaderParameter<Eigen::Matrix4f>>("model_matrix", Eigen::Matrix4f::Identity());
  }

  ShaderSetting(int color_mode, const Eigen::Matrix4f& model_matrix) {
    params.resize(3);
    params[0] = std::make_shared<ShaderParameter<int>>("color_mode", color_mode);
    params[1] = std::make_shared<ShaderParameter<float>>("point_scale", 1.0f);
    params[2] = std::make_shared<ShaderParameter<Eigen::Matrix4f>>("model_matrix", model_matrix);
  }
  virtual ~ShaderSetting() {}

  template<typename T>
  ShaderSetting& add(const std::string& name, const T& value) {
    for(int i=0; i<params.size(); i++) {
      if(params[i]->name == name) {
        params[i] = std::make_shared<ShaderParameter<T>>(name, value);
        return *this;
      }
    }

    params.push_back(std::make_shared<ShaderParameter<T>>(name, value));
    return *this;
  }

  template<typename T>
  boost::optional<T> get(const std::string& name) {
    for(const auto& param :params) {
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
    for(const auto& param: params) {
      param->set(shader);
    }
  }

public:
  std::vector<ShaderParameterInterface::Ptr> params;
};

struct Rainbow : public ShaderSetting {
public:
  Rainbow() : ShaderSetting(0) {}

  Rainbow(const Eigen::Matrix4f& matrix) : ShaderSetting(0, matrix) {}

  virtual ~Rainbow() override {}
};


struct FlatColor : public ShaderSetting {
public:
  FlatColor(const Eigen::Vector4f& color) : ShaderSetting(1) {
    params.push_back(std::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
  }

  FlatColor(const Eigen::Vector4f& color, const Eigen::Matrix4f& matrix) : ShaderSetting(1, matrix) {
    params.push_back(std::make_shared<ShaderParameter<Eigen::Vector4f>>("material_color", color));
  }

  virtual ~FlatColor() override {}
};

struct VertexColor : public ShaderSetting {
public:
  VertexColor() : ShaderSetting(2) {}

  VertexColor(const Eigen::Matrix4f& matrix) : ShaderSetting(2, matrix) {}

  virtual ~VertexColor() override {}
};

}  // namespace guik

#endif