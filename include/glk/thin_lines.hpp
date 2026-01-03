#ifndef GLK_THIN_LINES_HPP
#define GLK_THIN_LINES_HPP

#include <GL/gl3w.h>

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/type_conversion.hpp>

namespace glk {

/**
 * @brief A class to draw a set of GL_LINES
 *
 */
class ThinLines : public Drawable {
public:
  ThinLines(const float* vertices, int num_vertices, bool line_strip = false);
  ThinLines(const float* vertices, const float* colors, int num_vertices, bool line_strip = false);
  ThinLines(const float* vertices, const float* colors, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);
  ThinLines(const float* vertices, const float* colors, const float* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);

  ThinLines(const Eigen::Vector3f* vertices, int num_vertices, bool line_strip = false);
  ThinLines(const Eigen::Vector3f* vertices, const Eigen::Vector4f* colors, int num_vertices, bool line_strip = false);
  ThinLines(const Eigen::Vector3f* vertices, const Eigen::Vector4f* colors, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);
  ThinLines(const Eigen::Vector3f* vertices, const float* cmap, int num_vertices, bool line_strip = false);
  ThinLines(const Eigen::Vector3f* vertices, const double* cmap, int num_vertices, bool line_strip = false);
  ThinLines(const Eigen::Vector3f* vertices, const float* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);
  ThinLines(const Eigen::Vector3f* vertices, const double* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);

  template <typename T1, int D1>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, int num_vertices, bool line_strip = false);
  template <typename T1, int D1>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);
  template <typename T1, int D1, typename T2, int D2>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const Eigen::Matrix<T2, D2, 1>* colors, int num_vertices, bool line_strip = false);
  template <typename T1, int D1, typename T2, int D2>
  ThinLines(
    const Eigen::Matrix<T1, D1, 1>* vertices,
    const Eigen::Matrix<T2, D2, 1>* colors,
    int num_vertices,
    const unsigned int* indices,
    int num_indices,
    bool line_strip = false);
  template <typename T1, int D1>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const float* cmap, int num_vertices, bool line_strip = false);
  template <typename T1, int D1>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const double* cmap, int num_vertices, bool line_strip = false);
  template <typename T1, int D1>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const float* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);
  template <typename T1, int D1>
  ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const double* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip = false);

  template <typename T1, int D1, typename Allocator>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, bool line_strip = false);
  template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices, const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors, bool line_strip = false);
  template <typename T1, int D1, typename Allocator>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<unsigned int>& indices, bool line_strip = false);
  template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2>
  ThinLines(
    const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices,
    const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors,
    const std::vector<unsigned int>& indices,
    bool line_strip = false);
  template <typename T1, int D1, typename Allocator>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<float>& cmap, bool line_strip = false);
  template <typename T1, int D1, typename Allocator>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<double>& cmap, bool line_strip = false);
  template <typename T1, int D1, typename Allocator>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<float>& cmap, const std::vector<unsigned int>& indices, bool line_strip = false);
  template <typename T1, int D1, typename Allocator>
  ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<double>& cmap, const std::vector<unsigned int>& indices, bool line_strip = false);

  virtual ~ThinLines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

  void set_line_width(float width) { line_width = width; }

private:
  ThinLines(const ThinLines&);
  ThinLines& operator=(const ThinLines&);

private:
  float line_width;
  int num_vertices;
  int num_indices;

  GLenum mode;  // line mode (GL_LINES/GL_LINE_STRIP)
  GLuint vao;   // vertex array object
  GLuint vbo;   // vertices
  GLuint cbo;   // colors
  GLuint cmbo;  // colormap
  GLuint ebo;   // indices
};

// template members

template <typename T1, int D1>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, int num_vertices, bool line_strip)
: ThinLines(convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), num_vertices, line_strip) {}

template <typename T1, int D1>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip)
: ThinLines(convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), static_cast<Eigen::Vector4f*>(nullptr), num_vertices, indices, num_indices, line_strip) {}

template <typename T1, int D1, typename T2, int D2>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const Eigen::Matrix<T2, D2, 1>* colors, int num_vertices, bool line_strip)
: ThinLines(
    convert_to_vector<float, 3, 1>(vertices, num_vertices).data(),
    colors ? convert_to_vector<float, 4, 1>(colors, num_vertices).data() : nullptr,
    num_vertices,
    line_strip) {}

template <typename T1, int D1, typename T2, int D2>
ThinLines::ThinLines(
  const Eigen::Matrix<T1, D1, 1>* vertices,
  const Eigen::Matrix<T2, D2, 1>* colors,
  int num_vertices,
  const unsigned int* indices,
  int num_indices,
  bool line_strip)
: ThinLines(
    convert_to_vector<float, 3, 1>(vertices, num_vertices).data(),
    colors ? convert_to_vector<float, 4, 1>(colors, num_vertices).data() : nullptr,
    num_vertices,
    indices,
    num_indices,
    line_strip) {}

template <typename T1, int D1>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const float* cmap, int num_vertices, bool line_strip)
: ThinLines(convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), cmap, num_vertices, nullptr, 0, line_strip) {}

template <typename T1, int D1>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const float* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip)
: ThinLines(convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), cmap, num_vertices, indices, num_indices, line_strip) {}

template <typename T1, int D1>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const double* cmap, int num_vertices, bool line_strip)
: ThinLines(convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), cmap, num_vertices, nullptr, 0, line_strip) {}

template <typename T1, int D1>
ThinLines::ThinLines(const Eigen::Matrix<T1, D1, 1>* vertices, const double* cmap, int num_vertices, const unsigned int* indices, int num_indices, bool line_strip)
: ThinLines(convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), cmap, num_vertices, indices, num_indices, line_strip) {}

template <typename T1, int D1, typename Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, bool line_strip) : ThinLines(vertices.data(), vertices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices, const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors, bool line_strip)
: ThinLines(vertices.data(), colors.data(), vertices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<unsigned int>& indices, bool line_strip)
: ThinLines(vertices.data(), static_cast<Eigen::Vector4f*>(nullptr), vertices.size(), indices.data(), indices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2>
ThinLines::ThinLines(
  const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices,
  const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors,
  const std::vector<unsigned int>& indices,
  bool line_strip)
: ThinLines(vertices.data(), colors.data(), vertices.size(), indices.data(), indices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<float>& cmap, bool line_strip)
: ThinLines(vertices.data(), cmap.data(), vertices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<float>& cmap, const std::vector<unsigned int>& indices, bool line_strip)
: ThinLines(vertices.data(), cmap.data(), vertices.size(), indices.data(), indices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<double>& cmap, bool line_strip)
: ThinLines(vertices.data(), cmap.data(), vertices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator>
ThinLines::ThinLines(const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, const std::vector<double>& cmap, const std::vector<unsigned int>& indices, bool line_strip)
: ThinLines(vertices.data(), cmap.data(), vertices.size(), indices.data(), indices.size(), line_strip) {}

}  // namespace glk

#endif