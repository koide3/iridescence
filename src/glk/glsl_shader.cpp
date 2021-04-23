#include <glk/glsl_shader.hpp>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>

#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

GLSLShader::GLSLShader() {
  shader_program = 0;
}

GLSLShader::~GLSLShader() {
  if(shader_program) {
    glDeleteProgram(shader_program);
  }
}

bool GLSLShader::attach_source(const std::string& filename, GLuint shader_type) {
  GLuint shader = read_shader_from_file(filename, shader_type);
  if(shader == GL_FALSE) {
    return false;
  }

  shaders.push_back(shader);
  return true;
}

bool GLSLShader::attach_source(const std::vector<std::string>& filenames, GLuint shader_type) {
  for(const auto& filename : filenames) {
    if(!attach_source(filename, shader_type)) {
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
  if(shader_program) {
    glUseProgram(0);
    glDeleteShader(shader_program);
    attrib_cache.clear();
    uniform_cache.clear();
  }

  shader_program = glCreateProgram();
  for(auto shader : shaders) {
    glAttachShader(shader_program, shader);
    glDeleteShader(shader);
  }

  if(!feedback_varyings.empty()) {
    std::vector<const GLchar*> varyings;
    for(const auto& v : feedback_varyings) {
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

  if(result != GL_TRUE) {
    std::cerr << bold_red << "error : failed to link program" << reset << std::endl;
    std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
    return false;
  }

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

GLint GLSLShader::attrib(const std::string& name) {
  auto found = attrib_cache.find(name);
  if(found != attrib_cache.end()) {
    return found->second;
  }

  GLint id = glGetAttribLocation(shader_program, name.c_str());
  if(id == -1) {
    std::cerr << bold_yellow << "warning : attrib " << name << " not found" << reset << std::endl;
  }

  attrib_cache[name] = id;
  return id;
}

GLint GLSLShader::uniform(const std::string& name) {
  auto found = uniform_cache.find(name);
  if(found != uniform_cache.end()) {
    return found->second;
  }

  GLint id = glGetUniformLocation(shader_program, name.c_str());
  if(id == -1) {
    std::cerr << bold_yellow << "warning : uniform " << name << " not found" << reset << std::endl;
  }

  uniform_cache[name] = id;
  return id;
}

GLuint GLSLShader::read_shader_from_file(const std::string& filename, GLuint shader_type) {
  GLuint shader_id = glCreateShader(shader_type);

  std::stringstream sst;

  std::ifstream ifs(filename);
  if(!ifs) {
    std::cerr << bold_red << "error: failed to open " << filename << reset << std::endl;
    return GL_FALSE;
  }

  sst << ifs.rdbuf();

  std::string source = sst.str();

  GLint result = GL_FALSE;
  int info_log_length = 0;

  char const* source_ptr = source.c_str();
  glShaderSource(shader_id, 1, &source_ptr, nullptr);
  glCompileShader(shader_id);

  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);

  std::vector<char> error_message(info_log_length);
  glGetShaderInfoLog(shader_id, info_log_length, nullptr, error_message.data());

  if(result != GL_TRUE) {
    std::cerr << bold_red << "error : failed to compile shader " << filename << "\033[0m" << std::endl;
    std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
  }

  return shader_id;
}

}  // namespace glk
