#ifndef GLK_LINES_HPP
#define GLK_LINES_HPP

#include <GL/gl3w.h>

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/type_conversion.hpp>

namespace glk {

/**
 * @brief A class to draw a set of lines
 *
 */
class Lines : public Drawable {
public:
  Lines(float line_width, const Eigen::Vector3f* vertices, int num_points, bool line_strip = false);
  Lines(float line_width, const Eigen::Vector3f* vertices, const Eigen::Vector4f* colors, int num_points, bool line_strip = false);
  Lines(float line_width, const Eigen::Vector3f* vertices, const Eigen::Vector4f* colors, const Eigen::Vector4i* infos, int num_points, bool line_strip = false);

  template <typename T1, int D1>
  Lines(float line_width, const Eigen::Matrix<T1, D1, 1>* vertices, int num_vertices, bool line_strip = false);
  template <typename T1, int D1, typename T2, int D2>
  Lines(float line_width, const Eigen::Matrix<T1, D1, 1>* vertices, const Eigen::Matrix<T2, D2, 1>* colors, int num_vertices, bool line_strip = false);
  template <typename T1, int D1, typename T2, int D2>
  Lines(
    float line_width,
    const Eigen::Matrix<T1, D1, 1>* vertices,
    const Eigen::Matrix<T2, D2, 1>* colors,
    const Eigen::Vector4i* infos,
    int num_vertices,
    bool line_strip = false);

  template <typename T1, int D1, typename Allocator>
  Lines(float line_width, const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, bool line_strip = false);
  template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2>
  Lines(
    float line_width,
    const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices,
    const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors,
    bool line_strip = false);
  template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2, typename Allocator3>
  Lines(
    float line_width,
    const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices,
    const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors,
    const std::vector<Eigen::Vector4i, Allocator3>& infos,
    bool line_strip = false);

  virtual ~Lines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  Lines(const Lines&);
  Lines& operator=(const Lines&);

private:
  int num_vertices;
  int num_indices;

  GLuint vao;  // vertex array object
  GLuint vbo;  // vertices
  GLuint cbo;  // colors
  GLuint ibo;  // infos
  GLuint ebo;  // elements
};

// template methods

template <typename T1, int D1>
Lines::Lines(float line_width, const Eigen::Matrix<T1, D1, 1>* vertices, int num_vertices, bool line_strip)
: Lines(line_width, convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), num_vertices, line_strip) {}

template <typename T1, int D1, typename T2, int D2>
Lines::Lines(float line_width, const Eigen::Matrix<T1, D1, 1>* vertices, const Eigen::Matrix<T2, D2, 1>* colors, int num_vertices, bool line_strip)
: Lines(line_width, convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), convert_to_vector<float, 4, 1>(colors, num_vertices).data(), num_vertices, line_strip) {}

template <typename T1, int D1, typename T2, int D2>
Lines::Lines(float line_width, const Eigen::Matrix<T1, D1, 1>* vertices, const Eigen::Matrix<T2, D2, 1>* colors, const Eigen::Vector4i* infos, int num_vertices, bool line_strip)
: Lines(line_width, convert_to_vector<float, 3, 1>(vertices, num_vertices).data(), convert_to_vector<float, 4, 1>(colors, num_vertices).data(), infos, num_vertices, line_strip) {}

template <typename T1, int D1, typename Allocator>
Lines::Lines(float line_width, const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator>& vertices, bool line_strip)
: Lines(line_width, vertices.data(), vertices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2>
Lines::Lines(float line_width, const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices, const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors, bool line_strip)
: Lines(line_width, vertices.data(), colors.data(), vertices.size(), line_strip) {}

template <typename T1, int D1, typename Allocator1, typename T2, int D2, typename Allocator2, typename Allocator3>
Lines::Lines(
  float line_width,
  const std::vector<Eigen::Matrix<T1, D1, 1>, Allocator1>& vertices,
  const std::vector<Eigen::Matrix<T2, D2, 1>, Allocator2>& colors,
  const std::vector<Eigen::Vector4i, Allocator3>& infos,
  bool line_strip)
: Lines(line_width, vertices.data(), colors.data(), infos.data(), vertices.size(), line_strip) {}

}  // namespace glk

#endif