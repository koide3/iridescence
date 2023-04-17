#ifndef GLK_GLSL_SHADER_HPP
#define GLK_GLSL_SHADER_HPP

#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include <optional>
#include <unordered_set>
#include <unordered_map>

#include <GL/gl3w.h>
#include <Eigen/Core>
#include <glk/make_shared.hpp>

namespace glk {

class GLSLShader {
public:
  GLSLShader();
  ~GLSLShader();

  bool attach_source(const std::string& filename, GLuint shader_type);
  bool attach_source(const std::vector<std::string>& filenames, GLuint shader_type);

  bool attach_source(const std::string& filename, const std::unordered_set<std::string>& include_filenames, const std::string& defines, GLuint shader_type);
  bool attach_source(const std::vector<std::string>& filenames, const std::unordered_set<std::string>& include_filenames, const std::string& defines, GLuint shader_type);

  bool add_feedback_varying(const std::string& name);

  bool link_program();

  bool init(const std::string& shader_path);
  bool init(const std::string& vertex_shader_path, const std::string& fragment_shader_path);
  bool init(const std::vector<std::string>& vertex_shader_paths, const std::vector<std::string>& fragment_shader_paths);

  GLuint id() const { return shader_program; }
  void use() const { glUseProgram(shader_program); }
  void unuse() const { glUseProgram(0); }

  GLint attrib(const std::string& name);
  GLint uniform(const std::string& name);
  GLint subroutine(GLenum shader_type, const std::string& name);
  GLint subroutine_uniform(GLenum shader_type, const std::string& name);

  int get_uniformi(const std::string& name) {
    int value;
    glGetUniformiv(shader_program, uniform(name), &value);
    return value;
  }

  float get_uniformf(const std::string& name) {
    float value;
    glGetUniformfv(shader_program, uniform(name), &value);
    return value;
  }

  Eigen::Vector4f get_uniform4f(const std::string& name) {
    Eigen::Vector4f vec;
    glGetUniformfv(shader_program, uniform(name), vec.data());
    return vec;
  }

  Eigen::Matrix4f get_uniform_matrix4f(const std::string& name) {
    Eigen::Matrix4f mat;
    glGetUniformfv(shader_program, uniform(name), mat.data());
    return mat;
  }

  template <typename T>
  T get_uniform_cache(const std::string& name) const {
    const auto found = uniform_variable_cache.find(name);
    if (found == uniform_variable_cache.end()) {
      std::cerr << "warning: failed to find uniform variable cache " << name << std::endl;
      return T();
    }
    return *reinterpret_cast<const T*>(found->second.get());
  }

  template <typename T>
  std::optional<T> get_uniform_cache_safe(const std::string& name) const {
    const auto found = uniform_variable_cache.find(name);
    if (found == uniform_variable_cache.end()) {
      return std::nullopt;
    }

    return *std::reinterpret_pointer_cast<const T>(found->second);
  }

  void set_uniform(const std::string& name, int value) {
    glUniform1i(uniform(name), value);
    set_uniform_cache(name, value);
  }
  void set_uniform(const std::string& name, float value) {
    glUniform1f(uniform(name), value);
    set_uniform_cache(name, value);
  }
  void set_uniform(const std::string& name, const Eigen::Vector2f& vector) {
    glUniform2fv(uniform(name), 1, vector.data());
    set_uniform_cache(name, vector);
  }
  void set_uniform(const std::string& name, const Eigen::Vector3f& vector) {
    glUniform3fv(uniform(name), 1, vector.data());
    set_uniform_cache(name, vector);
  }
  void set_uniform(const std::string& name, const Eigen::Vector4f& vector) {
    glUniform4fv(uniform(name), 1, vector.data());
    set_uniform_cache(name, vector);
  }
  void set_uniform(const std::string& name, const Eigen::Vector2i& vector) {
    glUniform4iv(uniform(name), 1, vector.data());
    set_uniform_cache(name, vector);
  }
  void set_uniform(const std::string& name, const Eigen::Vector3i& vector) {
    glUniform3iv(uniform(name), 1, vector.data());
    set_uniform_cache(name, vector);
  }
  void set_uniform(const std::string& name, const Eigen::Vector4i& vector) {
    glUniform4iv(uniform(name), 1, vector.data());
    set_uniform_cache(name, vector);
  }
  void set_uniform(const std::string& name, const Eigen::Matrix4f& matrix) {
    glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
    set_uniform_cache(name, matrix);
  }
  void set_uniform(const std::string& name, const std::vector<int>& vectors) {
    glUniform1iv(uniform(name), vectors.size(), vectors.data());
    set_uniform_cache(name, vectors);
  }
  void set_uniform(const std::string& name, const std::vector<float>& vectors) {
    glUniform1fv(uniform(name), vectors.size(), vectors.data());
    set_uniform_cache(name, vectors);
  }
  template <typename Allocator>
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector2f, Allocator>& vectors) {
    glUniform2fv(uniform(name), vectors.size(), vectors[0].data());
    set_uniform_cache(name, vectors);
  }
  template <typename Allocator>
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector3f, Allocator>& vectors) {
    glUniform3fv(uniform(name), vectors.size(), vectors[0].data());
    set_uniform_cache(name, vectors);
  }
  template <typename Allocator>
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector4f, Allocator>& vectors) {
    glUniform4fv(uniform(name), vectors.size(), vectors[0].data());
    set_uniform_cache(name, vectors);
  }
  void set_uniform(const std::string& name, const Eigen::Matrix4d& matrix_) {
    Eigen::Matrix4f matrix = matrix_.cast<float>();
    glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
  }

  void set_subroutine(GLenum shader_type, const std::string& loc, const std::string& func);

private:
  GLuint read_shader_from_file(const std::string& filename, const std::unordered_map<std::string, std::string>& include_map, const std::string& defines, GLuint shader_type);

  template <typename T>
  void set_uniform_cache(const std::string& name, const T& value) {
    const auto found = uniform_variable_cache.find(name);
    if (found != uniform_variable_cache.end()) {
      *reinterpret_cast<T*>(found->second.get()) = value;
    } else {
      uniform_variable_cache.emplace_hint(found, name, glk::make_shared<T>(value));
    }
  }

private:
  std::vector<GLuint> shaders;
  std::vector<std::string> feedback_varyings;

  GLuint shader_program;
  std::unordered_map<std::string, GLint> attrib_cache;
  std::unordered_map<std::string, GLint> uniform_cache;
  std::unordered_map<std::string, GLint> subroutine_cache;

  std::unordered_map<std::string, std::shared_ptr<void>> uniform_variable_cache;
};

}  // namespace glk

#endif