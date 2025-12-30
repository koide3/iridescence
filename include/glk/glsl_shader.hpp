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
#include <glk/hash.hpp>
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

  GLint attrib(std::uint64_t name, const char* debug_msg = nullptr);
  GLint attrib(const std::string& name);
  GLint uniform(std::uint64_t name, const char* debug_msg = nullptr);
  GLint uniform(const std::string& name);
  GLint subroutine(GLenum shader_type, std::uint64_t name, const char* debug_msg = nullptr);
  GLint subroutine(GLenum shader_type, const std::string& name);
  GLint subroutine_uniform(GLenum shader_type, std::uint64_t name, const char* debug_msg = nullptr);
  GLint subroutine_uniform(GLenum shader_type, const std::string& name);

  int get_uniformi(std::uint64_t name);
  int get_uniformi(const std::string& name);

  float get_uniformf(std::uint64_t name);
  float get_uniformf(const std::string& name);

  Eigen::Vector4f get_uniform4f(std::uint64_t name);
  Eigen::Vector4f get_uniform4f(const std::string& name);

  Eigen::Matrix4f get_uniform_matrix4f(std::uint64_t name);
  Eigen::Matrix4f get_uniform_matrix4f(const std::string& name);

  template <typename T>
  T get_uniform_cache(std::uint64_t name) const {
    const auto found = std::find_if(uniform_variable_cache.begin(), uniform_variable_cache.end(), [=](const auto& pair) { return pair.first == name; });
    if (found == uniform_variable_cache.end()) {
      std::cerr << "warning: failed to find uniform variable cache " << name << std::endl;
      return T();
    }
    return *reinterpret_cast<const T*>(found->second.get());
  }

  template <typename T>
  std::optional<T> get_uniform_cache_safe(std::uint64_t name) const {
    const auto found = std::find_if(uniform_variable_cache.begin(), uniform_variable_cache.end(), [=](const auto& pair) { return pair.first == name; });
    if (found == uniform_variable_cache.end()) {
      return std::nullopt;
    }
    return *reinterpret_cast<const T*>(found->second.get());
  }

  // int
  void set_uniform(std::uint64_t name, int value, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, int value);

  // float
  void set_uniform(std::uint64_t name, float value, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, float value);

  // vec2f
  void set_uniform(std::uint64_t name, const Eigen::Vector2f& vector, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Vector2f& vector);

  // vec3f
  void set_uniform(std::uint64_t name, const Eigen::Vector3f& vector, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Vector3f& vector);

  // vec4f
  void set_uniform(std::uint64_t name, const Eigen::Vector4f& vector, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Vector4f& vector);

  // vec2i
  void set_uniform(std::uint64_t name, const Eigen::Vector2i& vector, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Vector2i& vector);

  // vec3i
  void set_uniform(std::uint64_t name, const Eigen::Vector3i& vector, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Vector3i& vector);

  // vec4i
  void set_uniform(std::uint64_t name, const Eigen::Vector4i& vector, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Vector4i& vector);

  // matrix4f
  void set_uniform(std::uint64_t name, const Eigen::Matrix4f& matrix, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Matrix4f& matrix);

  // array of int
  void set_uniform(std::uint64_t name, const std::vector<int>& vectors, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const std::vector<int>& vectors);

  // array of float
  void set_uniform(std::uint64_t name, const std::vector<float>& vectors, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const std::vector<float>& vectors);

  // array of vec2f
  void set_uniform(std::uint64_t name, const std::vector<Eigen::Vector2f>& vectors, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector2f>& vectors);

  // array of vec3f
  void set_uniform(std::uint64_t name, const std::vector<Eigen::Vector3f>& vectors, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector3f>& vectors);

  // array of vec4f
  void set_uniform(std::uint64_t name, const std::vector<Eigen::Vector4f>& vectors, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector4f>& vectors);
  // matrix4d
  void set_uniform(std::uint64_t name, const Eigen::Matrix4d& matrix_, const char* debug_msg = nullptr);
  void set_uniform(const std::string& name, const Eigen::Matrix4d& matrix_);

  void set_subroutine(GLenum shader_type, const std::string& loc, const std::string& func);

private:
  GLuint read_shader_from_file(const std::string& filename, const std::unordered_map<std::string, std::string>& include_map, const std::string& defines, GLuint shader_type);

  template <typename T>
  void set_uniform_cache(std::uint64_t h, const T& value) {
    const auto found = std::find_if(uniform_variable_cache.begin(), uniform_variable_cache.end(), [&h](const auto& pair) { return pair.first == h; });
    if (found != uniform_variable_cache.end()) {
      *reinterpret_cast<T*>(found->second.get()) = value;
    } else {
      uniform_variable_cache.emplace_back(h, glk::make_shared<T>(value));
    }
  }

private:
  std::vector<GLuint> shaders;
  std::vector<std::string> feedback_varyings;

  GLuint shader_program;
  std::vector<std::pair<std::uint64_t, GLint>> attrib_cache;
  std::vector<std::pair<std::uint64_t, GLint>> uniform_cache;

  std::vector<std::pair<std::uint64_t, GLint>> subroutine_cache;
  std::vector<std::pair<std::uint64_t, std::shared_ptr<void>>> uniform_variable_cache;
};

}  // namespace glk

#endif