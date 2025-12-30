#include <glk/glsl_shader.hpp>

#include <regex>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <unordered_map>

#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/hash.hpp>
#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

GLSLShader::GLSLShader() {
  shader_program = 0;
}

GLSLShader::~GLSLShader() {
  if (shader_program) {
    glDeleteProgram(shader_program);
  }
}

bool GLSLShader::attach_source(const std::string& filename, const std::unordered_set<std::string>& include_filenames, const std::string& defines, GLuint shader_type) {
  std::unordered_map<std::string, std::string> include_map;
  for (const auto& include_filename : include_filenames) {
    const std::string filename = std::filesystem::path(include_filename).filename().string();
    include_map[filename] = include_filename;
  }

  GLuint shader = read_shader_from_file(filename, include_map, defines, shader_type);
  if (shader == GL_FALSE) {
    return false;
  }

  shaders.push_back(shader);
  return true;
}

bool GLSLShader::attach_source(
  const std::vector<std::string>& filenames,
  const std::unordered_set<std::string>& include_filenames,
  const std::string& defines,
  GLuint shader_type) {
  for (const auto& filename : filenames) {
    if (!attach_source(filename, include_filenames, defines, shader_type)) {
      return false;
    }
  }
  return true;
}

bool GLSLShader::attach_source(const std::string& filename, GLuint shader_type) {
  return attach_source(filename, {}, "", shader_type);
}

bool GLSLShader::attach_source(const std::vector<std::string>& filenames, GLuint shader_type) {
  for (const auto& filename : filenames) {
    if (!attach_source(filename, shader_type)) {
      return false;
    }
  }
  return true;
}

bool GLSLShader::add_feedback_varying(const std::string& name) {
  feedback_varyings.push_back(name);
  return true;
}

bool GLSLShader::link_program() {
  if (shader_program) {
    glUseProgram(0);
    glDeleteShader(shader_program);
    attrib_cache.clear();
    uniform_cache.clear();
  }

  shader_program = glCreateProgram();
  for (auto shader : shaders) {
    glAttachShader(shader_program, shader);
    glDeleteShader(shader);
  }

  if (!feedback_varyings.empty()) {
    std::vector<const GLchar*> varyings;
    for (const auto& v : feedback_varyings) {
      varyings.push_back(v.c_str());
    }

    glTransformFeedbackVaryings(shader_program, varyings.size(), varyings.data(), GL_INTERLEAVED_ATTRIBS);
  }

  shaders.clear();
  glLinkProgram(shader_program);

  GLint result = GL_FALSE;
  int info_log_length;

  glGetProgramiv(shader_program, GL_LINK_STATUS, &result);
  glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &info_log_length);
  std::vector<char> error_message(info_log_length);
  glGetProgramInfoLog(shader_program, info_log_length, nullptr, error_message.data());

  if (result != GL_TRUE) {
    std::cerr << bold_red << "error : failed to link program" << reset << std::endl;
    std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
    return false;
  }

  // Find attribs
  glUseProgram(shader_program);
  GLint num_attribs = 0;
  glGetProgramiv(shader_program, GL_ACTIVE_ATTRIBUTES, &num_attribs);
  for (GLint i = 0; i < num_attribs; i++) {
    char name[256];
    GLsizei len = 0;
    GLint size = 0;
    GLenum type = 0;
    glGetActiveAttrib(shader_program, i, sizeof(name), &len, &size, &type, name);
    std::string name_str(name, len);
    GLint location = glGetAttribLocation(shader_program, name_str.c_str());

    attrib_cache.emplace_back(glk::hash(name_str), location);
  }

  // Find uniforms
  GLint num_uniforms = 0;
  glGetProgramiv(shader_program, GL_ACTIVE_UNIFORMS, &num_uniforms);
  for (GLint i = 0; i < num_uniforms; i++) {
    char name[256];
    GLsizei len = 0;
    GLint size = 0;
    GLenum type = 0;
    glGetActiveUniform(shader_program, i, sizeof(name), &len, &size, &type, name);
    std::string name_str(name, len);
    GLint location = glGetUniformLocation(shader_program, name_str.c_str());

    uniform_cache.emplace_back(glk::hash(name_str), location);
  }

  glUseProgram(0);

  return true;
}

bool GLSLShader::init(const std::string& shader_path) {
  return init(shader_path + ".vert", shader_path + ".frag");
}

bool GLSLShader::init(const std::string& vertex_shader_path, const std::string& fragment_shader_path) {
  std::vector<std::string> vertex_shader_paths = {vertex_shader_path};
  std::vector<std::string> fragment_shader_paths = {fragment_shader_path};
  return init(vertex_shader_paths, fragment_shader_paths);
}

bool GLSLShader::init(const std::vector<std::string>& vertex_shader_paths, const std::vector<std::string>& fragment_shader_paths) {
  attach_source(vertex_shader_paths, GL_VERTEX_SHADER);
  attach_source(fragment_shader_paths, GL_FRAGMENT_SHADER);
  return link_program();
}

GLint GLSLShader::attrib(std::uint64_t name) const {
  auto found = std::find_if(attrib_cache.begin(), attrib_cache.end(), [&name](const auto& pair) { return pair.first == name; });
  if (found != attrib_cache.end()) {
    return found->second;
  } else {
    return -1;
  }
}

GLint GLSLShader::attrib(const std::string& name) const {
  return attrib(glk::hash(name));
}

GLint GLSLShader::uniform(std::uint64_t name) const {
  auto found = std::find_if(uniform_cache.begin(), uniform_cache.end(), [&name](const auto& pair) { return pair.first == name; });
  if (found != uniform_cache.end()) {
    return found->second;
  } else {
    return -1;
  }
}

GLint GLSLShader::uniform(const std::string& name) const {
  return uniform(glk::hash(name));
}

GLint GLSLShader::subroutine(GLenum shader_type, std::uint64_t name) const {
  auto found = std::find_if(subroutine_cache.begin(), subroutine_cache.end(), [&name](const auto& pair) { return pair.first == name; });
  if (found != subroutine_cache.end()) {
    return found->second;
  } else {
    return -1;
  }
}

GLint GLSLShader::subroutine(GLenum shader_type, const std::string& name) {
  const std::uint64_t h = glk::hash(name);
  GLint id = subroutine(shader_type, h);
  if (id >= -1) {
    return id;
  }

  id = glGetSubroutineIndex(shader_program, shader_type, name.c_str());
  if (id == -1) {
    std::cerr << bold_yellow << "warning : subroutine " << name << " not found" << reset << std::endl;
  }

  subroutine_cache.emplace_back(h, id);
  return id;
}

GLint GLSLShader::subroutine_uniform(GLenum shader_type, std::uint64_t name) const {
  auto found = std::find_if(uniform_cache.begin(), uniform_cache.end(), [&name](const auto& pair) { return pair.first == name; });
  if (found != uniform_cache.end()) {
    return found->second;
  } else {
    return -1;
  }
}

GLint GLSLShader::subroutine_uniform(GLenum shader_type, const std::string& name) {
  const std::uint64_t h = glk::hash(name);
  GLint id = subroutine_uniform(shader_type, h);
  if (id >= -1) {
    return id;
  }

  id = glGetSubroutineUniformLocation(shader_program, shader_type, name.c_str());
  if (id == -1) {
    std::cerr << bold_yellow << "warning : subroutine uniform " << name << " not found" << reset << std::endl;
  }

  uniform_cache.emplace_back(h, id);
  return id;
}

void GLSLShader::set_subroutine(GLenum shader_type, const std::string& loc, const std::string& func) {
  GLint num_subroutines = 0;

  glGetProgramStageiv(shader_program, shader_type, GL_ACTIVE_SUBROUTINE_UNIFORM_LOCATIONS, &num_subroutines);

  GLint index = subroutine_uniform(shader_type, loc);
  GLint func_index = subroutine(shader_type, func);

  std::vector<GLuint> indices(num_subroutines, 0);

  if (index < 0 || index >= num_subroutines) {
    std::cerr << bold_red << "error : subroutine index out of range!!" << reset << std::endl;
    std::cerr << bold_red << "      : num_subroutines:" << num_subroutines << " subroutine_index:" << index << reset << std::endl;
    return;
  }

  indices[index] = func_index;
  glUniformSubroutinesuiv(shader_type, num_subroutines, indices.data());
}

GLuint
GLSLShader::read_shader_from_file(const std::string& filename, const std::unordered_map<std::string, std::string>& include_map, const std::string& defines, GLuint shader_type) {
  GLuint shader_id = glCreateShader(shader_type);

  const auto read_source = [](const std::string& filename) -> std::string {
    std::ifstream ifs(filename);
    if (!ifs) {
      std::cerr << bold_red << "error: failed to open " << filename << reset << std::endl;
      return "";
    }

    std::stringstream sst;
    sst << ifs.rdbuf();

    return sst.str();
  };

  std::string source = read_source(filename);
  if (source.empty()) {
    return GL_FALSE;
  }

  std::regex include_pattern("(//)?\\s*#include\\s*<([^>]+)>");
  std::smatch match;
  while (std::regex_search(source, match, include_pattern)) {
    const std::string include_filename = match.str(2);

    std::string include_source;
    if (include_filename == "define") {
      include_source = defines;
    } else {
      const auto found = include_map.find(include_filename);
      if (found == include_map.end()) {
        return GL_FALSE;
      }

      include_source = read_source(found->second);
      if (include_source.empty()) {
        return GL_FALSE;
      }
    }

    std::stringstream sst;
    sst << match.prefix() << "\n";
    sst << include_source << "\n";
    sst << match.suffix();

    source = sst.str();
  }

  GLint result = GL_FALSE;
  int info_log_length = 0;

  char const* source_ptr = source.c_str();
  glShaderSource(shader_id, 1, &source_ptr, nullptr);
  glCompileShader(shader_id);

  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);

  std::vector<char> error_message(info_log_length);
  glGetShaderInfoLog(shader_id, info_log_length, nullptr, error_message.data());

  const auto split_lines = [](const std::string& text) {
    std::vector<std::string> tokens;

    size_t loc = 0;
    size_t found = 0;

    do {
      found = text.find_first_of('\n', loc);
      tokens.push_back(text.substr(loc, found - loc));
      loc = found + 1;
    } while (found != std::string::npos);

    return tokens;
  };

  if (result != GL_TRUE) {
    const std::string error_text(error_message.begin(), error_message.end());
    std::cerr << bold_red << "error : failed to compile shader " << filename << "\033[0m" << std::endl;

    std::vector<std::string> source_lines = split_lines(source);
    std::vector<std::string> error_lines = split_lines(std::string(error_message.begin(), error_message.end()));

    for (const auto& error_line : error_lines) {
      std::cerr << error_line << std::endl;

      std::smatch matched;
      if (std::regex_search(error_line, matched, std::regex("[0-9]+\\(([0-9]+)\\) : (error|warning)"))) {
        const int line = std::stoi(matched.str(1)) - 1;

        if (line < source_lines.size()) {
          std::cout << console::cyan << "       :            : L" << line << "= " << source_lines[line] << console::reset << std::endl;
        } else {
          std::cout << console::cyan << "       :            : L" << line << "= out of source" << console::reset << std::endl;
        }
      }
    }
  }

  return shader_id;
}

int GLSLShader::get_uniformi(std::uint64_t name) {
  int value;
  glGetUniformiv(shader_program, uniform(name), &value);
  return value;
}

int GLSLShader::get_uniformi(const std::string& name) {
  int value;
  glGetUniformiv(shader_program, uniform(name), &value);
  return value;
}

float GLSLShader::get_uniformf(std::uint64_t name) {
  float value;
  glGetUniformfv(shader_program, uniform(name), &value);
  return value;
}

float GLSLShader::get_uniformf(const std::string& name) {
  float value;
  glGetUniformfv(shader_program, uniform(name), &value);
  return value;
}

Eigen::Vector4f GLSLShader::get_uniform4f(std::uint64_t name) {
  Eigen::Vector4f vec;
  glGetUniformfv(shader_program, uniform(name), vec.data());
  return vec;
}

Eigen::Vector4f GLSLShader::get_uniform4f(const std::string& name) {
  Eigen::Vector4f vec;
  glGetUniformfv(shader_program, uniform(name), vec.data());
  return vec;
}

Eigen::Matrix4f GLSLShader::get_uniform_matrix4f(std::uint64_t name) {
  Eigen::Matrix4f mat;
  glGetUniformfv(shader_program, uniform(name), mat.data());
  return mat;
}

Eigen::Matrix4f GLSLShader::get_uniform_matrix4f(const std::string& name) {
  Eigen::Matrix4f mat;
  glGetUniformfv(shader_program, uniform(name), mat.data());
  return mat;
}

void GLSLShader::set_uniform(std::uint64_t name, int value) {
  glUniform1i(uniform(name), value);
  set_uniform_cache(name, value);
}

void GLSLShader::set_uniform(const std::string& name, int value) {
  return set_uniform(glk::hash(name), value);
}

void GLSLShader::set_uniform(std::uint64_t name, float value) {
  glUniform1f(uniform(name), value);
  set_uniform_cache(name, value);
}

void GLSLShader::set_uniform(const std::string& name, float value) {
  return set_uniform(glk::hash(name), value);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector2f& vector) {
  glUniform2fv(uniform(name), 1, vector.data());
  set_uniform_cache(name, vector);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector2f& vector) {
  return set_uniform(glk::hash(name), vector);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector3f& vector) {
  glUniform3fv(uniform(name), 1, vector.data());
  set_uniform_cache(name, vector);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector3f& vector) {
  return set_uniform(glk::hash(name), vector);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector4f& vector) {
  glUniform4fv(uniform(name), 1, vector.data());
  set_uniform_cache(name, vector);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector4f& vector) {
  return set_uniform(glk::hash(name), vector);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector2i& vector) {
  glUniform2iv(uniform(name), 1, vector.data());
  set_uniform_cache(name, vector);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector2i& vector) {
  return set_uniform(glk::hash(name), vector);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector3i& vector) {
  glUniform3iv(uniform(name), 1, vector.data());
  set_uniform_cache(name, vector);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector3i& vector) {
  return set_uniform(glk::hash(name), vector);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector4i& vector) {
  glUniform4iv(uniform(name), 1, vector.data());
  set_uniform_cache(name, vector);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector4i& vector) {
  return set_uniform(glk::hash(name), vector);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Matrix4f& matrix) {
  glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
  set_uniform_cache(name, matrix);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Matrix4f& matrix) {
  return set_uniform(glk::hash(name), matrix);
}

void GLSLShader::set_uniform(std::uint64_t name, const std::vector<int>& vectors) {
  glUniform1iv(uniform(name), vectors.size(), vectors.data());
  set_uniform_cache(name, vectors);
}

void GLSLShader::set_uniform(const std::string& name, const std::vector<int>& vectors) {
  return set_uniform(glk::hash(name), vectors);
}

void GLSLShader::set_uniform(std::uint64_t name, const std::vector<float>& vectors) {
  glUniform1fv(uniform(name), vectors.size(), vectors.data());
  set_uniform_cache(name, vectors);
}

void GLSLShader::set_uniform(const std::string& name, const std::vector<float>& vectors) {
  return set_uniform(glk::hash(name), vectors);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector2f* vectors, size_t count) {
  glUniform2fv(uniform(name), count, vectors[0].data());
  set_uniform_cache(name, vectors);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector2f* vectors, size_t count) {
  return set_uniform(glk::hash(name), vectors, count);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector3f* vectors, size_t count) {
  glUniform3fv(uniform(name), count, vectors[0].data());
  set_uniform_cache(name, vectors);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector3f* vectors, size_t count) {
  return set_uniform(glk::hash(name), vectors, count);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Vector4f* vectors, size_t count) {
  glUniform4fv(uniform(name), count, vectors[0].data());
  set_uniform_cache(name, vectors);
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector4f* vectors, size_t count) {
  return set_uniform(glk::hash(name), vectors, count);
}

void GLSLShader::set_uniform(std::uint64_t name, const Eigen::Matrix4d& matrix_) {
  Eigen::Matrix4f matrix = matrix_.cast<float>();
  glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
}

void GLSLShader::set_uniform(const std::string& name, const Eigen::Matrix4d& matrix_) {
  set_uniform(glk::hash(name), matrix_);
}

}  // namespace glk
