#ifndef GLK_GLSL_SHADER_HPP
#define GLK_GLSL_SHADER_HPP

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

#include <GL/gl3w.h>
#include <Eigen/Core>

namespace glk {

class GLSLShader {
public:
  GLSLShader();
  ~GLSLShader();

  bool init(const std::string& shader_path);
  bool init(const std::string& vertex_shader_path, const std::string& fragment_shader_path);
  bool init(const std::vector<std::string>& vertex_shader_paths, const std::vector<std::string>& fragment_shader_paths);

  GLuint id() const {
    return shader_program;
  }
  void use() const {
    glUseProgram(shader_program);
  }
  void unuse() const {
    glUseProgram(0);
  }

  GLint attrib(const std::string& name);
  GLint uniform(const std::string& name);

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

  void set_uniform(const std::string& name, int value) {
    glUniform1i(uniform(name), value);
  }
  void set_uniform(const std::string& name, float value) {
    glUniform1f(uniform(name), value);
  }
  void set_uniform(const std::string& name, const Eigen::Vector2f& vector) {
    glUniform2fv(uniform(name), 1, vector.data());
  }
  void set_uniform(const std::string& name, const Eigen::Vector3f& vector) {
    glUniform3fv(uniform(name), 1, vector.data());
  }
  void set_uniform(const std::string& name, const Eigen::Vector4f& vector) {
    glUniform4fv(uniform(name), 1, vector.data());
  }
  void set_uniform(const std::string& name, const Eigen::Vector2i& vector) {
    glUniform4iv(uniform(name), 1, vector.data());
  }
  void set_uniform(const std::string& name, const Eigen::Vector3i& vector) {
    glUniform3iv(uniform(name), 1, vector.data());
  }
  void set_uniform(const std::string& name, const Eigen::Vector4i& vector) {
    glUniform4iv(uniform(name), 1, vector.data());
  }
  void set_uniform(const std::string& name, const Eigen::Matrix4f& matrix) {
    glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
  }

  void set_uniform(const std::string& name, const std::vector<float>& vectors) {
    glUniform1fv(uniform(name), vectors.size(), vectors.data());
  }
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& vectors) {
    glUniform2fv(uniform(name), vectors.size(), vectors[0].data());
  }
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vectors) {
    glUniform3fv(uniform(name), vectors.size(), vectors[0].data());
  }
  void set_uniform(const std::string& name, const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& vectors) {
    glUniform4fv(uniform(name), vectors.size(), vectors[0].data());
  }

  void set_uniform(const std::string& name, const Eigen::Matrix4d& matrix_) {
    Eigen::Matrix4f matrix = matrix_.cast<float>();
    glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
  }

private:
  GLuint read_shader_from_file(const std::string& filename, GLuint shader_type);

private:
  GLuint shader_program;
  std::unordered_map<std::string, GLint> attrib_cache;
  std::unordered_map<std::string, GLint> uniform_cache;
};

}  // namespace glk

#endif