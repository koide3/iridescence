#include <glk/pointcloud_buffer.hpp>

#include <random>
#include <numeric>
#include <iostream>
#include <cassert>
#include <glk/colormap.hpp>
#include <glk/console_colors.hpp>
#include <glk/type_conversion.hpp>
#include <glk/async_buffer_copy.hpp>

namespace glk {

PointCloudBuffer::PointCloudBuffer(int stride, int num_points) {
  this->stride = stride;
  this->num_points = num_points;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, nullptr, GL_STATIC_DRAW);

  rendering_count = 0;
  points_rendering_budget = 8192;
  ebo = 0;
  cmap_bo = 0;
}

PointCloudBuffer::PointCloudBuffer(const float* data, int stride, int num_points) {
  this->stride = stride;
  this->num_points = num_points;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, nullptr, GL_STATIC_DRAW);

  if (data) {
    write_buffer_async(GL_ARRAY_BUFFER, stride * num_points, data);
  }

  rendering_count = 0;
  points_rendering_budget = 8192;
  ebo = 0;
  cmap_bo = 0;
}

PointCloudBuffer::PointCloudBuffer(const Eigen::Matrix<float, 3, -1>& points) : PointCloudBuffer(points.data(), sizeof(Eigen::Vector3f), points.cols()) {}

PointCloudBuffer::PointCloudBuffer(const Eigen::Matrix<double, 3, -1>& points) : PointCloudBuffer(points.cast<float>().eval()) {}

PointCloudBuffer::PointCloudBuffer(const Eigen::Vector3f* points, int num_points) : PointCloudBuffer(points->data(), sizeof(Eigen::Vector3f), num_points) {}

PointCloudBuffer::PointCloudBuffer(const Eigen::Vector4f* points, int num_points) : PointCloudBuffer(points->data(), sizeof(Eigen::Vector4f), num_points) {}

PointCloudBuffer::PointCloudBuffer(const Eigen::Vector3d* points, int num_points) : PointCloudBuffer(convert_to_vector<float, 3, 1>(points, num_points)) {}

PointCloudBuffer::PointCloudBuffer(const Eigen::Vector4d* points, int num_points) : PointCloudBuffer(convert_to_vector<float, 4, 1>(points, num_points)) {}

PointCloudBuffer::~PointCloudBuffer() {
  glDeleteVertexArrays(1, &vao);
  for (const auto& aux : aux_buffers) {
    glDeleteBuffers(1, &aux.buffer);
  }
  glDeleteBuffers(1, &vbo);

  if (ebo) {
    glDeleteBuffers(1, &ebo);
  }
}

void PointCloudBuffer::add_intensity(glk::COLORMAP colormap, const std::vector<float>& intensities, float scale) {
  add_intensity(colormap, intensities.data(), sizeof(float), intensities.size(), scale);
}

void PointCloudBuffer::add_intensity(glk::COLORMAP colormap, const std::vector<double>& intensities, float scale) {
  std::vector<float> intensities_(intensities.size());
  std::copy(intensities.begin(), intensities.end(), intensities_.begin());
  add_intensity(colormap, intensities_, scale);
}

void PointCloudBuffer::add_intensity(glk::COLORMAP colormap, const float* intensities, const int num_points, float scale) {
  add_intensity(colormap, intensities, sizeof(float), num_points, scale);
}

void PointCloudBuffer::add_intensity(glk::COLORMAP colormap, const double* intensities, const int num_points, float scale) {
  std::vector<double> intensities_(intensities, intensities + num_points);
  return add_intensity(colormap, intensities_, scale);
}

void PointCloudBuffer::add_color(const Eigen::Vector4f* colors, int num_points) {
  add_color(colors->data(), sizeof(float) * 4, num_points);
}

void PointCloudBuffer::add_color(const Eigen::Vector4d* colors, int num_points) {
  std::vector<Eigen::Vector4f> colors_f(num_points);
  std::transform(colors, colors + num_points, colors_f.begin(), [](const Eigen::Vector4d& c) { return c.cast<float>(); });
  add_color(colors_f);
}

void PointCloudBuffer::add_normals(const float* data, int stride, int num_points) {
  add_buffer("vert_normal", 3, data, stride, num_points);
}

void PointCloudBuffer::add_color(const float* data, int stride, int num_points) {
  add_buffer("vert_color", 4, data, stride, num_points);
}

void PointCloudBuffer::add_intensity(glk::COLORMAP colormap, const float* data, int stride, int num_points, float scale) {
  std::vector<Eigen::Vector4f> colors(num_points);
  for (int i = 0; i < num_points; i++) {
    colors[i] = glk::colormapf(colormap, scale * data[(stride / sizeof(float)) * i]);
  }

  add_color(colors[0].data(), sizeof(Eigen::Vector4f), num_points);
}

void PointCloudBuffer::set_colormap_buffer(const std::string& attribute_name) {
  auto found = std::find_if(aux_buffers.begin(), aux_buffers.end(), [&](const AuxBufferData& aux) { return aux.attribute_name == attribute_name; });
  if (found == aux_buffers.end()) {
    std::cerr << console::bold_yellow << "warning : attribute_name=" << attribute_name << " does not exist!!" << console::reset << std::endl;
  } else {
    cmap_bo = found->buffer;
  }
}

void PointCloudBuffer::add_colormap(std::vector<float>& cmap, float scale) {
  add_colormap(cmap.data(), sizeof(float), cmap.size(), scale);
}

void PointCloudBuffer::add_colormap(std::vector<double>& cmap, float scale) {
  std::vector<float> cmap_f(cmap.size());
  std::copy(cmap.begin(), cmap.end(), cmap_f.begin());
  add_colormap(cmap_f, scale);
}

void PointCloudBuffer::add_colormap(const float* data, int stride, int num_points, float scale) {
  if (std::abs(scale - 1.0f) > 1e-6f) {
    const int step = stride / sizeof(float);
    std::vector<float> scaled(num_points);
    for (int i = 0; i < num_points; i++) {
      scaled[i] = scale * data[step * i];
    }
    add_buffer("cmap", 1, scaled.data(), sizeof(float), num_points);
  } else {
    add_buffer("cmap", 1, data, stride, num_points);
  }

  set_colormap_buffer("cmap");
}

void PointCloudBuffer::add_buffer(const std::string& attribute_name, const std::vector<float>& data) {
  add_buffer(attribute_name, 1, data.data(), sizeof(float), data.size());
}

void PointCloudBuffer::add_buffer(const std::string& attribute_name, const std::vector<double>& data) {
  std::vector<float> data_f(data.size());
  std::copy(data.begin(), data.end(), data_f.begin());
  add_buffer(attribute_name, 1, data_f.data(), sizeof(float), data_f.size());
}

void PointCloudBuffer::add_buffer(const std::string& attribute_name, int dim, const float* data, int stride, int num_points) {
  assert(this->num_points == num_points);

  auto found = std::find_if(aux_buffers.begin(), aux_buffers.end(), [&](const AuxBufferData& aux) { return aux.attribute_name == attribute_name; });
  if (found != aux_buffers.end()) {
    glDeleteBuffers(1, &found->buffer);
    aux_buffers.erase(found);
  }

  glBindVertexArray(vao);

  GLuint buffer_id;
  glGenBuffers(1, &buffer_id);
  glBindBuffer(GL_ARRAY_BUFFER, buffer_id);
  glBufferData(GL_ARRAY_BUFFER, stride * num_points, nullptr, GL_STATIC_DRAW);

  if (data) {
    write_buffer_async(GL_ARRAY_BUFFER, stride * num_points, data);
  }

  aux_buffers.push_back(AuxBufferData{attribute_name, dim, stride, buffer_id});
}

void PointCloudBuffer::update_buffer_with_indices(GLuint buffer, const float* data_, int stride, const unsigned int* indices, int num_indices) {
  if (num_indices == 0) {
    return;
  }

  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, buffer);

  const char* data = reinterpret_cast<const char*>(data_);
  const unsigned int* index_begin = indices;

  const auto& update_sub = [&](const unsigned int* begin, const unsigned int* end) {
    const char* src = data + stride * std::distance(indices, begin);
    const size_t offset = stride * (*begin);
    const size_t size = stride * (*(end - 1) - (*begin) + 1);
    glBufferSubData(GL_ARRAY_BUFFER, offset, size, src);
  };

  for (const unsigned int* index_itr = indices + 1; index_itr != indices + num_indices; index_itr++) {
    const unsigned int last_index = *(index_itr - 1);
    const unsigned int curr_index = *index_itr;

    if (static_cast<int>(curr_index) - static_cast<int>(last_index) != 1) {
      update_sub(index_begin, index_itr);
      index_begin = index_itr;
    }
  }

  update_sub(index_begin, indices + num_indices);
}

void PointCloudBuffer::update_buffer_with_indices(const std::string& attribute_name, int dim, const float* data_, int stride, const unsigned int* indices, int num_indices) {
  using namespace glk::console;

  if (num_indices == 0) {
    return;
  }

  auto found = std::find_if(aux_buffers.begin(), aux_buffers.end(), [&](const AuxBufferData& aux) { return aux.attribute_name == attribute_name; });
  if (found == aux_buffers.end()) {
    std::cerr << yellow << "warning: attribute_name=" << attribute_name << " does not exist!!" << reset << std::endl;
    add_buffer(attribute_name, dim, nullptr, stride, num_points);
    return update_buffer_with_indices(attribute_name, dim, data_, stride, indices, num_indices);
  }

  if (found->dim != dim) {
    std::cerr << yellow << "warning: point attribute (" << attribute_name << ") dim mismatch!!" << reset << std::endl;
    return;
  }

  if (found->stride != stride) {
    std::cerr << yellow << "warning: point attribute (" << attribute_name << ") stride mismatch!!" << reset << std::endl;
    return;
  }

  update_buffer_with_indices(found->buffer, data_, stride, indices, num_indices);
}

void PointCloudBuffer::update_points_with_indices(const Eigen::Vector3f* points, const unsigned int* indices, int num_indices) {
  update_points_with_indices(points[0].data(), sizeof(float) * 3, indices, num_indices);
}

void PointCloudBuffer::update_points_with_indices(const Eigen::Vector4f* points, const unsigned int* indices, int num_indices) {
  update_points_with_indices(points[0].data(), sizeof(float) * 4, indices, num_indices);
}

void PointCloudBuffer::update_points_with_indices(const Eigen::Vector3d* points, const unsigned int* indices, int num_indices) {
  std::vector<Eigen::Vector3f> points_f(num_indices);
  std::transform(points, points + num_indices, points_f.begin(), [](const auto& p) { return p.template cast<float>(); });
  update_points_with_indices(points_f.data(), indices, num_indices);
}

void PointCloudBuffer::update_points_with_indices(const Eigen::Vector4d* points, const unsigned int* indices, int num_indices) {
  std::vector<Eigen::Vector4f> points_f(num_indices);
  std::transform(points, points + num_indices, points_f.begin(), [](const auto& p) { return p.template cast<float>(); });
  update_points_with_indices(points_f.data(), indices, num_indices);
}

void PointCloudBuffer::update_points_with_indices(const float* data, int stride, const unsigned int* indices, int num_indices) {
  using namespace glk::console;
  if (stride != this->stride) {
    std::cerr << yellow << "warning: point stride mismatch!! " << stride << " vs " << this->stride << reset << std::endl;
    return;
  }

  update_buffer_with_indices(vbo, data, stride, indices, num_indices);
}

void PointCloudBuffer::update_color_with_indices(const Eigen::Vector4f* colors, const unsigned int* indices, int num_indices) {
  update_buffer_with_indices("vert_color", 4, colors[0].data(), sizeof(float) * 4, indices, num_indices);
}

void PointCloudBuffer::update_color_with_indices(const Eigen::Vector4d* colors, const unsigned int* indices, int num_indices) {
  std::vector<Eigen::Vector4f> colors_f(num_indices);
  std::transform(colors, colors + num_indices, colors_f.begin(), [](const Eigen::Vector4d& c) { return c.cast<float>(); });
  update_color_with_indices(colors_f.data(), indices, num_indices);
}

void PointCloudBuffer::enable_partial_rendering(int points_budget) {
  if (ebo) {
    disable_partial_rendering();
  }

  this->points_rendering_budget = points_budget;

  std::vector<unsigned int> indices(num_points);
  std::iota(indices.begin(), indices.end(), 0);

  std::mt19937 mt;
  std::shuffle(indices.begin(), indices.end(), mt);

  const int block_size = 8192 * 2;
  for (int i = 0; i < indices.size(); i += block_size) {
    const int count = std::min<int>(block_size, indices.size() - i);
    std::sort(indices.begin() + i, indices.begin() + i + count);
  }

  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_points, nullptr, GL_STATIC_DRAW);
  write_buffer_async(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_points, indices.data());

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void PointCloudBuffer::disable_partial_rendering() {
  if (ebo == 0) {
    return;
  }

  glDeleteBuffers(1, &ebo);
  ebo = 0;
  rendering_count = 0;
}

void PointCloudBuffer::bind(glk::GLSLShader& shader) const {
  if (num_points == 0) {
    return;
  }
  const GLint position_loc = shader.attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, stride, 0);

  if (cmap_bo) {
    GLint cmap_loc = shader.attrib("vert_cmap");
    glEnableVertexAttribArray(cmap_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cmap_bo);
    glVertexAttribPointer(cmap_loc, 1, GL_FLOAT, GL_FALSE, sizeof(float), 0);
  }

  for (const auto& aux : aux_buffers) {
    if (aux.buffer == cmap_bo) {
      continue;
    }

    GLint attrib_loc = shader.attrib(aux.attribute_name);
    glEnableVertexAttribArray(attrib_loc);
    glBindBuffer(GL_ARRAY_BUFFER, aux.buffer);
    glVertexAttribPointer(attrib_loc, aux.dim, GL_FLOAT, GL_FALSE, aux.stride, 0);
  }
}

void PointCloudBuffer::unbind(glk::GLSLShader& shader) const {
  if (num_points == 0) {
    return;
  }
  const GLint position_loc = shader.attrib("vert_position");

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(position_loc);
  for (const auto& aux : aux_buffers) {
    if (aux.buffer == cmap_bo) {
      continue;
    }
    glDisableVertexAttribArray(shader.attrib(aux.attribute_name));
  }
  if (cmap_bo) {
    glDisableVertexAttribArray(shader.attrib("vert_cmap"));
  }
}

void PointCloudBuffer::draw(glk::GLSLShader& shader) const {
  if (num_points == 0) {
    return;
  }

  bind(shader);

  if (!ebo) {
    glDrawArrays(GL_POINTS, 0, num_points);
  } else {
    const int offset = ((rendering_count++) * points_rendering_budget) % num_points;
    const int count = std::max(points_rendering_budget, num_points - offset);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glDrawElements(GL_POINTS, points_rendering_budget, GL_UNSIGNED_INT, (void*)(offset * sizeof(unsigned int)));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  unbind(shader);
}

GLuint PointCloudBuffer::vba_id() const {
  return vao;
}
GLuint PointCloudBuffer::vbo_id() const {
  return vbo;
}

GLuint PointCloudBuffer::ebo_id() const {
  return ebo;
}

int PointCloudBuffer::get_aux_size() const {
  return aux_buffers.size();
}

const AuxBufferData& PointCloudBuffer::get_aux_buffer(int i) const {
  return aux_buffers[i];
}

size_t PointCloudBuffer::memory_usage() const {
  size_t bytes = stride * num_points;
  if (ebo) {
    bytes += sizeof(unsigned int) * num_points;
  }

  for (const auto& aux : aux_buffers) {
    bytes += aux.stride * num_points;
  }

  return bytes;
}

}  // namespace glk
