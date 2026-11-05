#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <iostream>
#include <cstring>
#include <algorithm>
#include <glk/path.hpp>
#include <glk/lines.hpp>
#include <glk/texture.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/pointnormals_buffer.hpp>
#include <glk/voxelmap.hpp>
#include <glk/elevation_map.hpp>
#include <glk/gridmap.hpp>
#include <glk/drawable_container.hpp>
#include <glk/io/ply_io.hpp>
#include <glk/mesh_rendering_options.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_effect.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>

#include "py_drawable.hpp"

namespace py = pybind11;
bool check_valid_points(const py::array_t<float, py::array::c_style | py::array::forcecast>& points) {
  if (points.ndim() != 2) {
    std::cerr << "points must be 2-dimensional (ndim=" << points.ndim() << ")" << std::endl;
    return false;
  }
  if (points.shape(1) != 3 && points.shape(1) != 4) {
    std::cerr << "points must have 3 or 4 columns (cols=" << points.shape(1) << ")" << std::endl;
    return false;
  }
  return true;
}

bool check_valid_colors(const py::array_t<float, py::array::c_style | py::array::forcecast>& points) {
  if (points.ndim() != 2) {
    std::cerr << "points must be 2-dimensional (ndim=" << points.ndim() << ")" << std::endl;
    return false;
  }
  if (points.shape(1) != 4) {
    std::cerr << "points must have 4 columns (cols=" << points.shape(1) << ")" << std::endl;
    return false;
  }
  return true;
}

/// @brief Number of components per pixel for a pixel transfer format. Returns -1 for unknown formats.
int gl_format_num_components(GLenum format) {
  switch (format) {
    case GL_DEPTH_COMPONENT:
    case GL_RED:
    case GL_GREEN:
    case GL_BLUE:
    case GL_RED_INTEGER:
      return 1;
    case GL_RG:
    case GL_RG_INTEGER:
      return 2;
    case GL_RGB:
    case GL_BGR:
    case GL_RGB_INTEGER:
      return 3;
    case GL_RGBA:
    case GL_BGRA:
    case GL_RGBA_INTEGER:
      return 4;
    default:
      return -1;
  }
}

/// @brief Size (in bytes) of a component for a pixel transfer type. Returns 0 for unknown or packed types.
size_t gl_type_size(GLenum type) {
  switch (type) {
    case GL_UNSIGNED_BYTE:
    case GL_BYTE:
      return 1;
    case GL_UNSIGNED_SHORT:
    case GL_SHORT:
    case GL_HALF_FLOAT:
      return 2;
    case GL_UNSIGNED_INT:
    case GL_INT:
    case GL_FLOAT:
      return 4;
    default:
      return 0;
  }
}

/// @brief Create a texture from raw pixel data after validating the data size.
std::shared_ptr<glk::Texture> create_texture_from_bytes(const Eigen::Vector2i& size, GLenum internal_format, GLenum format, GLenum type, const void* data, size_t num_bytes) {
  if (size[0] <= 0 || size[1] <= 0) {
    throw std::runtime_error("texture size must be positive (size=" + std::to_string(size[0]) + "x" + std::to_string(size[1]) + ")");
  }

  if (num_bytes == 0) {
    // Create an uninitialized texture
    return std::make_shared<glk::Texture>(size, internal_format, format, type, nullptr);
  }

  const int num_components = gl_format_num_components(format);
  const size_t type_size = gl_type_size(type);
  if (num_components > 0 && type_size > 0) {
    const size_t required_bytes = static_cast<size_t>(size[0]) * size[1] * num_components * type_size;
    if (num_bytes < required_bytes) {
      throw std::runtime_error("insufficient pixel data (bytes=" + std::to_string(num_bytes) + " required=" + std::to_string(required_bytes) + ")");
    }
  }

  return std::make_shared<glk::Texture>(size, internal_format, format, type, data);
}

/// @brief Read texture pixels into a numpy array of the given dtype. Src must have the same size as the dtype.
template <typename Src>
py::array read_texture_pixels(const glk::Texture& texture, GLenum format, GLenum type, int num_components, const char* dtype) {
  const Eigen::Vector2i size = texture.size();
  std::vector<py::ssize_t> shape = {size[1], size[0]};
  if (num_components != 1) {
    shape.push_back(num_components);
  }

  const auto pixels = texture.read_pixels<Src>(format, type, num_components);
  py::array arr(py::dtype(dtype), shape);
  std::memcpy(arr.mutable_data(), pixels.data(), sizeof(Src) * pixels.size());
  return arr;
}

bool check_valid_voxels(const py::array_t<int, py::array::c_style | py::array::forcecast>& voxels) {
  if (voxels.ndim() != 2) {
    std::cerr << "voxels must be 2-dimensional (ndim=" << voxels.ndim() << ")" << std::endl;
    return false;
  }
  if (voxels.shape(1) != 3) {
    std::cerr << "voxels must have 3 columns (cols=" << voxels.shape(1) << ")" << std::endl;
    return false;
  }
  return true;
}

// Extract cells[:, index] (or cells itself for a 1-dimensional array) as a list of 1D vectors
std::vector<Eigen::Matrix<float, 1, 1>> extract_cell_values(const py::array_t<float, py::array::c_style | py::array::forcecast>& cells, int index) {
  if (cells.ndim() != 1 && cells.ndim() != 2) {
    throw std::runtime_error("cells must be 1- or 2-dimensional (ndim=" + std::to_string(cells.ndim()) + ")");
  }

  const int cols = cells.ndim() == 1 ? 1 : cells.shape(1);
  if (index < 0 || index >= cols) {
    throw std::runtime_error("index out of range (index=" + std::to_string(index) + " cols=" + std::to_string(cols) + ")");
  }

  std::vector<Eigen::Matrix<float, 1, 1>> values(cells.shape(0));
  for (int i = 0; i < cells.shape(0); i++) {
    values[i][0] = cells.data()[i * cols + index];
  }
  return values;
}

// Convert cell indices to std::vector while rejecting indices outside of the map (except for the invalid index marker)
std::vector<std::uint64_t> to_cell_indices(const glk::ElevationMapBuilder& builder, const py::array_t<std::uint64_t, py::array::c_style | py::array::forcecast>& cell_indices) {
  if (cell_indices.ndim() != 1) {
    throw std::runtime_error("cell_indices must be 1-dimensional (ndim=" + std::to_string(cell_indices.ndim()) + ")");
  }

  const std::uint64_t num_cells = static_cast<std::uint64_t>(builder.size.x()) * builder.size.y();
  std::vector<std::uint64_t> indices(cell_indices.data(), cell_indices.data() + cell_indices.shape(0));
  for (const auto index : indices) {
    if (index >= num_cells && index != std::numeric_limits<std::uint64_t>::max()) {
      throw std::runtime_error("cell index out of range (index=" + std::to_string(index) + ")");
    }
  }
  return indices;
}

void define_glk(py::module_& m) {
  py::module_ gl_ = m.def_submodule("gl", "");
  py::module_ glk_ = m.def_submodule("glk", "");
  py::module_ primitives_ = glk_.def_submodule("primitives", "");

  // constants
  gl_.attr("RED") = py::int_(static_cast<int>(GL_RED));
  gl_.attr("RG") = py::int_(static_cast<int>(GL_RG));
  gl_.attr("RGB") = py::int_(static_cast<int>(GL_RGB));
  gl_.attr("BGR") = py::int_(static_cast<int>(GL_BGR));
  gl_.attr("RGBA") = py::int_(static_cast<int>(GL_RGBA));
  gl_.attr("BGRA") = py::int_(static_cast<int>(GL_BGRA));
  gl_.attr("UNSIGNED_BYTE") = py::int_(static_cast<int>(GL_UNSIGNED_BYTE));
  gl_.attr("BYTE") = py::int_(static_cast<int>(GL_BYTE));
  gl_.attr("UNSIGNED_SHORT") = py::int_(static_cast<int>(GL_UNSIGNED_SHORT));
  gl_.attr("SHORT") = py::int_(static_cast<int>(GL_SHORT));
  gl_.attr("UNSIGNED_INT") = py::int_(static_cast<int>(GL_UNSIGNED_INT));
  gl_.attr("INT") = py::int_(static_cast<int>(GL_INT));
  gl_.attr("HALF_FLOAT") = py::int_(static_cast<int>(GL_HALF_FLOAT));
  gl_.attr("FLOAT") = py::int_(static_cast<int>(GL_FLOAT));
  gl_.attr("RED_INTEGER") = py::int_(static_cast<int>(GL_RED_INTEGER));
  gl_.attr("RG_INTEGER") = py::int_(static_cast<int>(GL_RG_INTEGER));
  gl_.attr("RGB_INTEGER") = py::int_(static_cast<int>(GL_RGB_INTEGER));
  gl_.attr("RGBA_INTEGER") = py::int_(static_cast<int>(GL_RGBA_INTEGER));
  gl_.attr("DEPTH_COMPONENT") = py::int_(static_cast<int>(GL_DEPTH_COMPONENT));
  gl_.attr("R8") = py::int_(static_cast<int>(GL_R8));
  gl_.attr("RG8") = py::int_(static_cast<int>(GL_RG8));
  gl_.attr("RGB8") = py::int_(static_cast<int>(GL_RGB8));
  gl_.attr("RGBA8") = py::int_(static_cast<int>(GL_RGBA8));
  gl_.attr("R16F") = py::int_(static_cast<int>(GL_R16F));
  gl_.attr("RG16F") = py::int_(static_cast<int>(GL_RG16F));
  gl_.attr("RGB16F") = py::int_(static_cast<int>(GL_RGB16F));
  gl_.attr("RGBA16F") = py::int_(static_cast<int>(GL_RGBA16F));
  gl_.attr("R32F") = py::int_(static_cast<int>(GL_R32F));
  gl_.attr("RG32F") = py::int_(static_cast<int>(GL_RG32F));
  gl_.attr("RGB32F") = py::int_(static_cast<int>(GL_RGB32F));
  gl_.attr("RGBA32F") = py::int_(static_cast<int>(GL_RGBA32F));
  gl_.attr("R8UI") = py::int_(static_cast<int>(GL_R8UI));
  gl_.attr("R32I") = py::int_(static_cast<int>(GL_R32I));
  gl_.attr("RGBA32I") = py::int_(static_cast<int>(GL_RGBA32I));
  gl_.attr("DEPTH_COMPONENT32F") = py::int_(static_cast<int>(GL_DEPTH_COMPONENT32F));
  gl_.attr("NEAREST") = py::int_(static_cast<int>(GL_NEAREST));
  gl_.attr("LINEAR") = py::int_(static_cast<int>(GL_LINEAR));
  gl_.attr("CLAMP_TO_EDGE") = py::int_(static_cast<int>(GL_CLAMP_TO_EDGE));
  gl_.attr("CLAMP_TO_BORDER") = py::int_(static_cast<int>(GL_CLAMP_TO_BORDER));
  gl_.attr("MIRRORED_REPEAT") = py::int_(static_cast<int>(GL_MIRRORED_REPEAT));
  gl_.attr("REPEAT") = py::int_(static_cast<int>(GL_REPEAT));
  gl_.attr("TEXTURE0") = py::int_(static_cast<int>(GL_TEXTURE0));

  // enums
  py::enum_<glk::COLORMAP>(glk_, "COLORMAP")
    .value("TURBO", glk::COLORMAP::TURBO)
    .value("JET", glk::COLORMAP::JET)
    .value("CIVIDIS", glk::COLORMAP::CIVIDIS)
    .value("OCEAN", glk::COLORMAP::OCEAN)
    .value("SPRING", glk::COLORMAP::SPRING)
    .value("SUMMER", glk::COLORMAP::SUMMER)
    .value("AUTUMN", glk::COLORMAP::AUTUMN)
    .value("WINTER", glk::COLORMAP::WINTER)
    .value("GREAN_YELLOW", glk::COLORMAP::GREAN_YELLOW)
    .value("BLUE_RED", glk::COLORMAP::BLUE_RED)
    .value("PUBUGN", glk::COLORMAP::PUBUGN)
    .value("TURBID", glk::COLORMAP::TURBID)
    .value("PASTEL", glk::COLORMAP::PASTEL)
    .value("HELIX", glk::COLORMAP::HELIX)
    .value("PHASE", glk::COLORMAP::PHASE)
    .value("VEGETATION", glk::COLORMAP::VEGETATION)
    .value("CURL", glk::COLORMAP::CURL)
    .value("COOL_WARM", glk::COLORMAP::COOL_WARM)
    .export_values();

  // classes
  // glk::GLSLShader
  py::class_<glk::GLSLShader>(glk_, "GLSLShader")
    .def("id", &glk::GLSLShader::id)
    .def("use", &glk::GLSLShader::use)
    .def("unuse", &glk::GLSLShader::unuse)
    .def("attrib", [](glk::GLSLShader& shader, const std::string& name) { return shader.attrib(name); }, py::arg("name"))
    .def("uniform", [](glk::GLSLShader& shader, const std::string& name) { return shader.uniform(name); }, py::arg("name"))
    .def("get_uniformi", [](glk::GLSLShader& shader, const std::string& name) { return shader.get_uniformi(name); }, py::arg("name"))
    .def("get_uniformf", [](glk::GLSLShader& shader, const std::string& name) { return shader.get_uniformf(name); }, py::arg("name"))
    .def("get_uniform4f", [](glk::GLSLShader& shader, const std::string& name) { return shader.get_uniform4f(name); }, py::arg("name"))
    .def("get_uniform_matrix4f", [](glk::GLSLShader& shader, const std::string& name) { return shader.get_uniform_matrix4f(name); }, py::arg("name"))
    // Float vector overloads come before int ones so that lists and float64 arrays are converted to float vectors.
    // Pass np.int32 arrays to set integer vector uniforms.
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, int value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, float value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Vector2f& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Vector3f& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Vector4f& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Matrix4f& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Vector2i& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Vector3i& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"))
    .def("set_uniform", [](glk::GLSLShader& shader, const std::string& name, const Eigen::Vector4i& value) { shader.set_uniform(name, value); }, py::arg("name"), py::arg("value"));

  // glk::Drawable
  py::class_<glk::Drawable, PyDrawable, std::shared_ptr<glk::Drawable>>(glk_, "Drawable")
    .def(py::init<>())
    .def("draw", &glk::Drawable::draw, py::arg("shader"));

  // glk::ThinLines
  py::class_<glk::ThinLines, glk::Drawable, std::shared_ptr<glk::ThinLines>>(glk_, "ThinLines")
    .def(
      py::init([](const py::array_t<float, py::array::c_style | py::array::forcecast>& points, bool line_strip, float line_width) {
        if (!check_valid_points(points)) {
          throw std::runtime_error("invalid points array");
        }

        return std::make_shared<glk::ThinLines>(points.data(), points.shape(0), line_strip, line_width);
      }),
      py::arg("points"),
      py::arg("line_strip") = false,
      py::arg("line_width") = 1.0f)
    .def(
      py::init([](
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& points,
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& colors,
                 bool line_strip,
                 float line_width) {
        if (!check_valid_points(points)) {
          throw std::runtime_error("invalid points array");
        }
        if (!check_valid_colors(colors)) {
          throw std::runtime_error("invalid colors array");
        }
        if (points.shape(0) != colors.shape(0)) {
          throw std::runtime_error("points and colors must have the same number of rows");
        }

        return std::make_shared<glk::ThinLines>(points.data(), colors.data(), points.shape(0), line_strip, line_width);
      }),
      py::arg("points"),
      py::arg("colors"),
      py::arg("line_strip") = false,
      py::arg("line_width") = 1.0f)
    .def(
      py::init([](const std::vector<Eigen::Vector3f>& points, bool line_strip, float line_width) { return std::make_shared<glk::ThinLines>(points, line_strip, line_width); }),
      "",
      py::arg("points"),
      py::arg("line_strip") = false,
      py::arg("line_width") = 1.0f)
    .def(
      py::init([](const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector4f>& colors, bool line_strip, float line_width) {
        return std::make_shared<glk::ThinLines>(points, colors, line_strip, line_width);
      }),
      "",
      py::arg("points"),
      py::arg("colors"),
      py::arg("line_strip") = false,
      py::arg("line_width") = 1.0f)
    // pyplot-like constructor
    .def(
      py::init([](const Eigen::VectorXf& x, const Eigen::VectorXf& y, const Eigen::VectorXf& z, const Eigen::VectorXf& c, bool line_strip, float line_width) {
        int size = std::max({(int)x.size(), (int)y.size(), (int)z.size(), (int)c.size()});
        if (x.size()) {
          size = std::min<int>(size, x.size());
        }
        if (y.size()) {
          size = std::min<int>(size, y.size());
        }
        if (z.size()) {
          size = std::min<int>(size, z.size());
        }
        if (c.size()) {
          size = std::min<int>(size, c.size());
        }

        Eigen::Matrix<float, -1, 3, Eigen::RowMajor> vertices = Eigen::Matrix<float, -1, 3, Eigen::RowMajor>::Zero(size, 3);
        if (x.size()) {
          vertices.col(0) = x.topRows(size);
        }
        if (y.size()) {
          vertices.col(1) = y.topRows(size);
        }
        if (z.size()) {
          vertices.col(2) = z.topRows(size);
        }

        if (c.size() && size > 0) {
          std::vector<Eigen::Vector4f> colors(size, Eigen::Vector4f::Zero());
          for (int i = 0; i < size; i++) {
            colors[i] = glk::colormapf(glk::COLORMAP::TURBO, c[i]);
          }

          return std::make_shared<glk::ThinLines>(vertices.data(), colors[0].data(), size, line_strip, line_width);
        }

        return std::make_shared<glk::ThinLines>(vertices.data(), size, line_strip, line_width);
      }),
      "",
      py::arg("x") = Eigen::VectorXf(),
      py::arg("y") = Eigen::VectorXf(),
      py::arg("z") = Eigen::VectorXf(),
      py::arg("c") = Eigen::VectorXf(),
      py::arg("line_strip") = true,
      py::arg("line_width") = 1.0f)
    .def("set_line_width", &glk::ThinLines::set_line_width, py::arg("width"));

  // glk::Lines
  py::class_<glk::Lines, glk::Drawable, std::shared_ptr<glk::Lines>>(glk_, "Lines")
    .def(py::init<float, const std::vector<Eigen::Vector3f>&, const std::vector<Eigen::Vector4f>&>(), py::arg("thickness"), py::arg("vertices"), py::arg("colors"))
    .def(
      py::init([](
                 float thickness,
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& vertices,
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& colors) {
        if (!check_valid_points(vertices)) {
          throw std::runtime_error("invalid vertices array");
        }
        if (!check_valid_colors(colors)) {
          throw std::runtime_error("invalid colors array");
        }
        if (vertices.shape(0) != colors.shape(0)) {
          throw std::runtime_error("vertices and colors must have the same number of rows");
        }

        std::vector<Eigen::Vector3f> vertices_vec(vertices.shape(0));
        std::vector<Eigen::Vector4f> colors_vec(colors.shape(0));

        for (int i = 0; i < vertices.shape(0); i++) {
          vertices_vec[i] = Eigen::Vector3f(vertices.at(i, 0), vertices.at(i, 1), vertices.at(i, 2));
          colors_vec[i] = Eigen::Vector4f(colors.at(i, 0), colors.at(i, 1), colors.at(i, 2), colors.at(i, 3));
        }

        return std::make_shared<glk::Lines>(thickness, vertices_vec, colors_vec);
      }),
      py::arg("thickness"),
      py::arg("vertices"),
      py::arg("colors"))
    // pyplot-like constructor
    .def(
      py::init([](const Eigen::VectorXf& x, const Eigen::VectorXf& y, const Eigen::VectorXf& z, const Eigen::VectorXf& c, float line_width, bool line_strip) {
        int size = std::max(x.size(), std::max(y.size(), z.size()));
        if (x.size()) {
          size = std::min<int>(size, x.size());
        }
        if (y.size()) {
          size = std::min<int>(size, y.size());
        }
        if (z.size()) {
          size = std::min<int>(size, z.size());
        }

        std::vector<Eigen::Vector3f> vertices((size - 1) * 2, Eigen::Vector3f::Zero());
        for (int i = 1; i < size && i < x.size(); i++) {
          vertices[(i - 1) * 2].x() = x[i - 1];
          vertices[(i - 1) * 2 + 1].x() = x[i];
        }

        for (int i = 1; i < size && i < y.size(); i++) {
          vertices[(i - 1) * 2].y() = y[i - 1];
          vertices[(i - 1) * 2 + 1].y() = y[i];
        }

        for (int i = 1; i < size && i < z.size(); i++) {
          vertices[(i - 1) * 2].z() = z[i - 1];
          vertices[(i - 1) * 2 + 1].z() = z[i];
        }

        std::vector<Eigen::Vector4f> colors;
        if (c.size()) {
          colors.reserve((size - 1) * 2);
          for (int i = 1; i < size && i < c.size(); i++) {
            colors.push_back(glk::colormapf(glk::COLORMAP::TURBO, c[i - 1]));
            colors.push_back(glk::colormapf(glk::COLORMAP::TURBO, c[i]));
          }
        }

        return std::make_shared<glk::Lines>(line_width, vertices, colors);
      }),
      "",
      py::arg("x") = Eigen::VectorXf(),
      py::arg("y") = Eigen::VectorXf(),
      py::arg("z") = Eigen::VectorXf(),
      py::arg("c") = Eigen::VectorXf(),
      py::arg("width") = 0.1f,
      py::arg("line_strip") = true);

  // glk::PointCloudBuffer
  py::class_<glk::PointCloudBuffer, glk::Drawable, std::shared_ptr<glk::PointCloudBuffer>>(glk_, "PointCloudBuffer")
    .def(
      py::init([](const py::array_t<float, py::array::c_style | py::array::forcecast>& points) {
        if (!check_valid_points(points)) {
          throw std::runtime_error("invalid points array");
        }

        return std::make_shared<glk::PointCloudBuffer>(points.data(), sizeof(float) * points.shape(1), points.shape(0));
      }),
      py::arg("points"))
    .def(py::init<const std::vector<Eigen::Vector3f>&>())
    .def(py::init([](const std::vector<Eigen::Vector3f>& points) { return std::make_shared<glk::PointCloudBuffer>(points); }), py::arg("points"))
    .def(
      "add_normals",
      [](glk::PointCloudBuffer& buffer, const std::vector<Eigen::Vector3f>& normals) { buffer.add_normals(normals); },
      py::arg("normals"))
    .def(
      "add_normals",
      [](glk::PointCloudBuffer& buffer, const py::array_t<float, py::array::c_style | py::array::forcecast>& normals) {
        if (!check_valid_points(normals)) {
          throw std::runtime_error("invalid normals array");
        }

        std::vector<Eigen::Vector3f> normals_vec(normals.shape(0));
        for (int i = 0; i < normals.shape(0); i++) {
          normals_vec[i] = Eigen::Vector3f(normals.at(i, 0), normals.at(i, 1), normals.at(i, 2));
        }
        buffer.add_normals(normals_vec);
      },
      py::arg("normals"))
    .def(
      "add_color",
      [](glk::PointCloudBuffer& buffer, const std::vector<Eigen::Vector3f>& colors) {
        std::vector<Eigen::Vector4f> colors_(colors.size());
        std::transform(colors.begin(), colors.end(), colors_.begin(), [](const Eigen::Vector3f& c) { return Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f); });
        buffer.add_color(colors_);
      },
      py::arg("colors"))
    .def(
      "add_color",
      [](glk::PointCloudBuffer& buffer, const std::vector<Eigen::Vector4f>& colors) { buffer.add_color(colors); },
      py::arg("colors"))
    .def(
      "add_color",
      [](glk::PointCloudBuffer& buffer, const py::array_t<float, py::array::c_style | py::array::forcecast>& colors) {
        if (!check_valid_colors(colors)) {
          throw std::runtime_error("invalid colors array");
        }

        std::vector<Eigen::Vector4f> colors_vec(colors.shape(0));
        for (int i = 0; i < colors.shape(0); i++) {
          colors_vec[i] = Eigen::Vector4f(colors.at(i, 0), colors.at(i, 1), colors.at(i, 2), colors.at(i, 3));
        }
        buffer.add_color(colors_vec);
      },
      py::arg("colors"))
    .def(
      "add_intensity",
      [](glk::PointCloudBuffer& buffer, glk::COLORMAP colormap, const std::vector<float>& intensities, float scale) { buffer.add_intensity(colormap, intensities, scale); },
      py::arg("colormap"),
      py::arg("intensities"),
      py::arg("scale") = 1.0f)
    .def("set_colormap_buffer", &glk::PointCloudBuffer::set_colormap_buffer, py::arg("colormap_buffer"))
    .def(
      "add_colormap",
      [](glk::PointCloudBuffer& buffer, std::vector<float>& cmap, float scale) { buffer.add_colormap(cmap, scale); },
      py::arg("cmap"),
      py::arg("scale") = 1.0f)
    .def(
      "add_buffer",
      [](glk::PointCloudBuffer& buffer, const std::string& attribute_name, const Eigen::MatrixXf& data) {
        buffer.add_buffer(attribute_name, data.cols(), data.data(), sizeof(float) * data.cols(), data.rows());
      },
      py::arg("attribute_name"),
      py::arg("data"));

  // glk::IndexedPointCloudBuffer
  py::class_<glk::IndexedPointCloudBuffer, glk::Drawable, std::shared_ptr<glk::IndexedPointCloudBuffer>>(
    glk_,
    "IndexedPointCloudBuffer",
    R"doc(Drawable that represents a point cloud with an index buffer.
It is useful for rendering a subset of points without duplicating the point cloud data.
)doc")
    .def(
      py::init([](const std::shared_ptr<glk::PointCloudBuffer>& cloud_buffer, const py::object& indices_obj) {
        if (cloud_buffer == nullptr) {
          throw std::runtime_error("cloud_buffer must not be None");
        }

        const py::array indices_arr = py::array::ensure(indices_obj);
        if (!indices_arr) {
          throw std::runtime_error("indices must be convertible to a numpy array");
        }
        if (indices_arr.ndim() != 1) {
          throw std::runtime_error("indices must be 1-dimensional (ndim=" + std::to_string(indices_arr.ndim()) + ")");
        }
        // Reject non-integer arrays (except for empty ones, e.g., [] that becomes a float64 array)
        const char kind = indices_arr.dtype().kind();
        if (indices_arr.size() != 0 && kind != 'i' && kind != 'u') {
          throw std::runtime_error(std::string("indices must be an integer array (dtype kind=") + kind + ")");
        }

        // Validate indices because out-of-range indices cause out-of-bounds reads on the GPU
        const auto indices = py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>::ensure(indices_arr);
        const std::int64_t num_points = cloud_buffer->size();
        std::vector<unsigned int> indices_vec(indices.size());
        for (py::ssize_t i = 0; i < indices.size(); i++) {
          const std::int64_t index = indices.data()[i];
          if (index < 0 || index >= num_points) {
            throw std::runtime_error("index out of range (index=" + std::to_string(index) + " num_points=" + std::to_string(num_points) + ")");
          }
          indices_vec[i] = static_cast<unsigned int>(index);
        }

        return std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, indices_vec);
      }),
      py::arg("cloud_buffer"),
      py::arg("indices"),
      R"doc(Create an indexed point cloud buffer.

Args:
    cloud_buffer: Point cloud buffer that holds the point data. It is shared, not copied.
    indices: 1-dimensional integer array of point indices to be rendered. Each index must be in [0, number of points in cloud_buffer).
)doc")
    .def("size", &glk::IndexedPointCloudBuffer::size, "Number of indices.")
    .def("__len__", &glk::IndexedPointCloudBuffer::size)
    .def("ebo_id", &glk::IndexedPointCloudBuffer::ebo_id, "OpenGL ID of the element (index) buffer.");

  // glk::PointNormalsBuffer
  py::class_<glk::PointNormalsBuffer, glk::Drawable, std::shared_ptr<glk::PointNormalsBuffer>>(
    glk_,
    "PointNormalsBuffer",
    R"doc(Drawable that visualizes point normals as line segments.
Only the normal lines (from p to p + n * normal_length) are drawn. Points themselves are not rendered.
)doc")
    .def(
      py::init([](
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& points,
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& normals,
                 double normal_length) {
        if (!check_valid_points(points)) {
          throw std::runtime_error("invalid points array");
        }
        if (!check_valid_points(normals)) {
          throw std::runtime_error("invalid normals array");
        }
        if (points.shape(0) != normals.shape(0)) {
          throw std::runtime_error("points and normals must have the same number of rows");
        }

        return std::make_shared<glk::PointNormalsBuffer>(
          points.data(),
          sizeof(float) * points.shape(1),
          normals.data(),
          sizeof(float) * normals.shape(1),
          points.shape(0),
          normal_length);
      }),
      py::arg("points"),
      py::arg("normals"),
      py::arg("normal_length") = 1.0,
      R"doc(Create a point normals buffer.

Args:
    points: Point positions (N x 3 or N x 4 array). The 4th column is ignored.
    normals: Normal vectors (N x 3 or N x 4 array). The 4th column is ignored.
    normal_length: Scale factor applied to each normal. It equals the drawn line length for unit normals.
)doc")
    .def(
      py::init([](const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, double normal_length) {
        if (points.size() != normals.size()) {
          throw std::runtime_error("points and normals must have the same size");
        }

        return std::make_shared<glk::PointNormalsBuffer>(
          points.empty() ? nullptr : points[0].data(),
          sizeof(Eigen::Vector3f),
          normals.empty() ? nullptr : normals[0].data(),
          sizeof(Eigen::Vector3f),
          points.size(),
          normal_length);
      }),
      py::arg("points"),
      py::arg("normals"),
      py::arg("normal_length") = 1.0,
      R"doc(Create a point normals buffer.

Args:
    points: List of point positions.
    normals: List of normal vectors. Must have the same size as points.
    normal_length: Scale factor applied to each normal. It equals the drawn line length for unit normals.
)doc");

  // glk::VoxelMapOptions
  py::class_<glk::VoxelMapOptions, std::shared_ptr<glk::VoxelMapOptions>>(glk_, "VoxelMapOptions")
    .def(py::init<>())
    .def("set_voxel_alpha", &glk::VoxelMapOptions::set_voxel_alpha, py::arg("alpha"))
    .def("set_voxel_color", &glk::VoxelMapOptions::set_voxel_color, py::arg("color"))
    .def("set_edge_alpha", &glk::VoxelMapOptions::set_edge_alpha, py::arg("alpha"))
    .def("set_edge_color", &glk::VoxelMapOptions::set_edge_color, py::arg("color"))
    .def_readwrite("edge_line_width", &glk::VoxelMapOptions::edge_line_width);

  // glk::MeshRenderingOptions
  py::class_<glk::MeshRenderingOptions, std::shared_ptr<glk::MeshRenderingOptions>>(glk_, "MeshRenderingOptions")
    .def(py::init<>())
    .def("set_face_alpha", &glk::MeshRenderingOptions::set_face_alpha, py::arg("alpha"))
    .def("set_face_color", &glk::MeshRenderingOptions::set_face_color, py::arg("color"))
    .def("set_edge_alpha", &glk::MeshRenderingOptions::set_edge_alpha, py::arg("alpha"))
    .def("set_edge_color", &glk::MeshRenderingOptions::set_edge_color, py::arg("color"))
    .def_readwrite("draw_faces", &glk::MeshRenderingOptions::draw_faces)
    .def_readwrite("draw_edges", &glk::MeshRenderingOptions::draw_edges)
    .def_readwrite("edge_line_width", &glk::MeshRenderingOptions::edge_line_width);

  // glk::VoxelMap
  py::class_<glk::VoxelMap, glk::Drawable, std::shared_ptr<glk::VoxelMap>>(glk_, "VoxelMap")
    .def(
      py::init([](const py::array_t<int, py::array::c_style | py::array::forcecast>& voxels, float resolution, const glk::MeshRenderingOptions& mesh_options) {
        if (!check_valid_voxels(voxels)) {
          throw std::runtime_error("invalid voxels array");
        }

        int n = voxels.shape(0);
        std::vector<Eigen::Vector3i> voxels_vec(n);

        for (int i = 0; i < n; i++) {
          voxels_vec[i] = Eigen::Vector3i(voxels.at(i, 0), voxels.at(i, 1), voxels.at(i, 2));
        }

        return std::make_shared<glk::VoxelMap>(voxels_vec.data(), voxels_vec.size(), resolution, mesh_options);
      }),
      py::arg("voxels"),
      py::arg("resolution"),
      py::arg("mesh_options") = glk::MeshRenderingOptions())
    .def(
      py::init([](const std::vector<Eigen::Vector3i>& voxels, float resolution, const glk::MeshRenderingOptions& voxel_options) {
        return std::make_shared<glk::VoxelMap>(voxels.data(), voxels.size(), resolution, voxel_options);
      }),
      py::arg("voxels"),
      py::arg("resolution"),
      py::arg("voxel_options") = glk::MeshRenderingOptions());

  // glk::ElevationMapBuilder
  using MapArray = Eigen::Array<float, -1, -1, Eigen::RowMajor>;
  py::class_<glk::ElevationMapBuilder, std::shared_ptr<glk::ElevationMapBuilder>>(glk_, "ElevationMapBuilder")
    .def(py::init<double, const Eigen::Vector2i&, const Eigen::Vector2f&>(), py::arg("resolution"), py::arg("size"), py::arg("origin"))
    .def(
      "calc_cell_indices",
      [](const glk::ElevationMapBuilder& builder, const py::array_t<float, py::array::c_style | py::array::forcecast>& cells) {
        if (cells.ndim() != 2 || cells.shape(1) < 2) {
          throw std::runtime_error("cells must be 2-dimensional with at least 2 columns");
        }

        std::vector<Eigen::Vector2f> cells_vec(cells.shape(0));
        for (int i = 0; i < cells.shape(0); i++) {
          cells_vec[i] = Eigen::Vector2f(cells.at(i, 0), cells.at(i, 1));
        }

        const auto cell_indices = builder.calc_cell_indices(cells_vec);
        return py::array_t<std::uint64_t>(cell_indices.size(), cell_indices.data());
      },
      py::arg("cells"))
    .def(
      "set_elevation",
      [](
        glk::ElevationMapBuilder& builder,
        const py::array_t<float, py::array::c_style | py::array::forcecast>& cells,
        int elevation_index,
        const py::array_t<std::uint64_t, py::array::c_style | py::array::forcecast>& cell_indices,
        bool average) {
        const auto values = extract_cell_values(cells, elevation_index);
        builder.set_elevation(values, 0, to_cell_indices(builder, cell_indices), average);
      },
      py::arg("cells"),
      py::arg("elevation_index"),
      py::arg("cell_indices"),
      py::arg("average") = false)
    .def(
      "set_cmap",
      [](
        glk::ElevationMapBuilder& builder,
        const py::array_t<float, py::array::c_style | py::array::forcecast>& cells,
        int cmap_index,
        const py::array_t<std::uint64_t, py::array::c_style | py::array::forcecast>& cell_indices,
        bool average) {
        const auto values = extract_cell_values(cells, cmap_index);
        builder.set_cmap(values, 0, to_cell_indices(builder, cell_indices), average);
      },
      py::arg("cells"),
      py::arg("cmap_index"),
      py::arg("cell_indices"),
      py::arg("average") = false)
    .def(
      "filter_elevation_map",
      [](glk::ElevationMapBuilder& builder, bool process_valid_cells, int window_size) {
        switch (window_size) {
          case 3:
            builder.filter_elevation_map<3>(process_valid_cells);
            break;
          case 5:
            builder.filter_elevation_map<5>(process_valid_cells);
            break;
          case 7:
            builder.filter_elevation_map<7>(process_valid_cells);
            break;
          default:
            throw std::runtime_error("window_size must be 3, 5, or 7 (window_size=" + std::to_string(window_size) + ")");
        }
      },
      py::arg("process_valid_cells") = true,
      py::arg("window_size") = 3)
    .def(
      "calc_cmap_gradient",
      [](glk::ElevationMapBuilder& builder, int window_size) {
        switch (window_size) {
          case 3:
            builder.calc_cmap_gradient<3>();
            break;
          case 5:
            builder.calc_cmap_gradient<5>();
            break;
          case 7:
            builder.calc_cmap_gradient<7>();
            break;
          default:
            throw std::runtime_error("window_size must be 3, 5, or 7 (window_size=" + std::to_string(window_size) + ")");
        }
      },
      py::arg("window_size") = 3)
    .def_readonly("resolution", &glk::ElevationMapBuilder::resolution)
    .def_readonly("size", &glk::ElevationMapBuilder::size)
    .def_readonly("origin", &glk::ElevationMapBuilder::origin)
    // Maps are returned as copies because the builder may reallocate them
    .def_property(
      "elevation_map",
      [](const glk::ElevationMapBuilder& builder) -> MapArray { return builder.elevation_map; },
      [](glk::ElevationMapBuilder& builder, const MapArray& map) {
        if (map.rows() != builder.size.y() || map.cols() != builder.size.x()) {
          throw std::runtime_error("elevation_map must have shape (size[1], size[0])");
        }
        builder.elevation_map = map;
        builder.count_map.resize(0, 0);
      })
    .def_property(
      "color_map",
      [](const glk::ElevationMapBuilder& builder) -> MapArray { return builder.color_map; },
      [](glk::ElevationMapBuilder& builder, const MapArray& map) {
        if (map.size() != 0 && (map.rows() != builder.size.y() || map.cols() != builder.size.x())) {
          throw std::runtime_error("color_map must be empty or have shape (size[1], size[0])");
        }
        builder.color_map = map;
      })
    .def_property_readonly("count_map", [](const glk::ElevationMapBuilder& builder) -> MapArray { return builder.count_map; });

  // glk::ElevationMap
  py::class_<glk::ElevationMap, glk::Drawable, std::shared_ptr<glk::ElevationMap>>(glk_, "ElevationMap")
    .def(py::init<const glk::ElevationMapBuilder&, const glk::MeshRenderingOptions&>(), py::arg("builder"), py::arg("options") = glk::MeshRenderingOptions());

  // glk::GridMap
  py::class_<glk::GridMap, glk::Drawable, std::shared_ptr<glk::GridMap>> gridmap(
    glk_,
    "GridMap",
    R"doc(Drawable that represents a 2D grid map with color values.

values is a (height, width) array ((height, width, 4) for RGBA mode), and values[y, x] is the value of cell (x, y).
The first row (values[0, :]) lies at -y.
The grid map lies on the XY plane, and the center is at the origin (0, 0, 0),
thus it extends from (-resolution * width / 2, -resolution * height / 2) to (resolution * width / 2, resolution * height / 2).
Use guik.TextureColor as shader setting.

uint8 arrays use the uint8 interpretation of ColorMode, and arrays of other dtypes are converted to float32 and use the float interpretation.
)doc");

  py::enum_<glk::GridMap::ColorMode>(
    gridmap,
    "ColorMode",
    R"doc(Defines how the color values are interpreted.

For uint8 values:
  RAW: rgb = values[y, x]
  TURBO: rgb = colormap(TURBO, values[y, x])
  PROB: rgb = 100 - values[y, x]
  PROB_TURBO: rgb = colormap(TURBO, (100 - values[y, x]) * 255.0 / 100.0)
  RGBA: rgba = values[y, x, :]
For float values:
  RAW: rgb = scale * values[y, x]
  TURBO: rgb = colormapf(TURBO, scale * values[y, x])
  PROB: rgb = 1.0 - scale * values[y, x]
  PROB_TURBO: rgb = colormapf(TURBO, 1.0 - scale * values[y, x])
  RGBA: rgba = values[y, x, :] (scale is not applied)
)doc")
    .value("RAW", glk::GridMap::ColorMode::RAW)
    .value("TURBO", glk::GridMap::ColorMode::TURBO)
    .value("PROB", glk::GridMap::ColorMode::PROB)
    .value("PROB_TURBO", glk::GridMap::ColorMode::PROB_TURBO)
    .value("RGBA", glk::GridMap::ColorMode::RGBA)
    .export_values();

  // Validate the shape of grid values ((height, width) or (height, width, 4) for RGBA) and return (width, height)
  const auto check_gridmap_values = [](const py::array& values, glk::GridMap::ColorMode mode) -> std::pair<int, int> {
    if (mode == glk::GridMap::ColorMode::RGBA) {
      if (values.ndim() != 3 || values.shape(2) != 4) {
        throw std::runtime_error("values must have shape (height, width, 4) for RGBA mode");
      }
    } else if (values.ndim() != 2) {
      throw std::runtime_error("values must have shape (height, width) (ndim=" + std::to_string(values.ndim()) + ")");
    }

    if (values.shape(0) == 0 || values.shape(1) == 0) {
      throw std::runtime_error("values must not be empty");
    }

    return {static_cast<int>(values.shape(1)), static_cast<int>(values.shape(0))};
  };

  // Float overloads are defined before uint8 ones so that non-uint8 arrays (e.g., float64) are converted to float rather than truncated to uint8
  gridmap
    .def(
      py::init([=](double resolution, const py::array_t<float, py::array::c_style | py::array::forcecast>& values, float alpha, glk::GridMap::ColorMode mode, float scale) {
        const auto [width, height] = check_gridmap_values(values, mode);
        return std::make_shared<glk::GridMap>(resolution, width, height, scale, values.data(), alpha, mode);
      }),
      py::arg("resolution"),
      py::arg("values"),
      py::arg("alpha") = 1.0f,
      py::arg("mode") = glk::GridMap::ColorMode::PROB,
      py::arg("scale") = 1.0f,
      R"doc(Create a grid map from float values.

Args:
    resolution: Resolution of each grid cell in world units.
    values: Color values of shape (height, width), or (height, width, 4) for RGBA mode. Converted to float32.
    alpha: Alpha value for the entire map. Ignored if mode is RGBA.
    mode: Color mode for interpreting the values.
    scale: Scale factor for the color values. Ignored if mode is RGBA.
)doc")
    .def(
      py::init([=](double resolution, const py::array_t<std::uint8_t, py::array::c_style>& values, int alpha, glk::GridMap::ColorMode mode) {
        const auto [width, height] = check_gridmap_values(values, mode);
        return std::make_shared<glk::GridMap>(resolution, width, height, values.data(), alpha, mode);
      }),
      py::arg("resolution"),
      py::arg("values"),
      py::arg("alpha") = 255,
      py::arg("mode") = glk::GridMap::ColorMode::PROB,
      R"doc(Create a grid map from uint8 values.

Args:
    resolution: Resolution of each grid cell in world units.
    values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
    alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
    mode: Color mode for interpreting the values.
)doc")
    .def(
      "update_color",
      [=](glk::GridMap& self, const py::array_t<float, py::array::c_style | py::array::forcecast>& values, float alpha, glk::GridMap::ColorMode mode, float scale) {
        const auto [width, height] = check_gridmap_values(values, mode);
        self.update_color(width, height, scale, values.data(), alpha, mode);
      },
      py::arg("values"),
      py::arg("alpha") = 1.0f,
      py::arg("mode") = glk::GridMap::ColorMode::PROB,
      py::arg("scale") = 1.0f,
      R"doc(Update the colors with float values.

The map extent given at construction does not change. If the shape of values differs from the original one, the colors are stretched to fit the extent.

Args:
    values: Color values of shape (height, width), or (height, width, 4) for RGBA mode. Converted to float32.
    alpha: Alpha value for the entire map. Ignored if mode is RGBA.
    mode: Color mode for interpreting the values.
    scale: Scale factor for the color values. Ignored if mode is RGBA.
)doc")
    .def(
      "update_color",
      [=](glk::GridMap& self, const py::array_t<std::uint8_t, py::array::c_style>& values, int alpha, glk::GridMap::ColorMode mode) {
        const auto [width, height] = check_gridmap_values(values, mode);
        self.update_color(width, height, values.data(), alpha, mode);
      },
      py::arg("values"),
      py::arg("alpha") = 255,
      py::arg("mode") = glk::GridMap::ColorMode::PROB,
      R"doc(Update the colors with uint8 values.

The map extent given at construction does not change. If the shape of values differs from the original one, the colors are stretched to fit the extent.

Args:
    values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
    alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
    mode: Color mode for interpreting the values.
)doc");

  // glk::DrawableContainer
  // push_back(drawable, shader_setting) is defined in define_guik() because guik.ShaderSetting is not registered yet here
  py::class_<glk::DrawableContainer, glk::Drawable, std::shared_ptr<glk::DrawableContainer>>(
    glk_,
    "DrawableContainer",
    R"doc(Container to store multiple drawables and draw them in order.
It can also store a shader setting for each drawable.

Shader settings stored in the container are applied right before drawing the corresponding drawable and are not restored afterward,
thus they also affect the subsequent drawables in the container that do not have their own shader settings.
)doc")
    .def(
      py::init<bool>(),
      py::arg("skip_model_matrix_setting") = true,
      R"doc(Create an empty container.

Args:
    skip_model_matrix_setting: If True, the model matrices of the shader settings stored in the container are not applied.
)doc")
    .def(
      py::init([](const std::vector<py::object>& drawables, bool skip_model_matrix_setting) {
        auto container = std::make_shared<glk::DrawableContainer>(skip_model_matrix_setting);
        for (const auto& drawable : drawables) {
          container->push_back(keep_python_drawable_alive(drawable));
        }
        return container;
      }),
      py::arg("drawables"),
      py::arg("skip_model_matrix_setting") = true,
      R"doc(Create a container with drawables.

Args:
    drawables: List of drawables to be added to the container.
    skip_model_matrix_setting: If True, the model matrices of the shader settings stored in the container are not applied.
)doc")
    .def("size", &glk::DrawableContainer::size, "Number of drawables in the container.")
    .def("__len__", &glk::DrawableContainer::size)
    .def("clear", &glk::DrawableContainer::clear, "Clear all drawables in the container.")
    .def(
      "push_back",
      [](glk::DrawableContainer& container, const py::object& drawable) { container.push_back(keep_python_drawable_alive(drawable)); },
      py::arg("drawable"),
      R"doc(Add a drawable to the container.

Args:
    drawable: Drawable to be added.
)doc")
    .def_readwrite(
      "skip_model_matrix",
      &glk::DrawableContainer::skip_model_matrix,
      "If True, the model matrices of the shader settings stored in the container are not applied.");

  // glk::Texture
  py::class_<glk::Texture, std::shared_ptr<glk::Texture>>(glk_, "Texture", R"doc(2D OpenGL texture.
All methods must be called in the thread that owns the OpenGL context.
Pixel data are assumed to be tightly packed (no row padding).
)doc")
    .def(
      py::init([](const Eigen::Vector2i& size, unsigned int internal_format, unsigned int format, unsigned int type, const py::buffer& bytes) {
        const py::array arr = py::array::ensure(bytes, py::array::c_style);
        if (!arr) {
          throw std::runtime_error("bytes must be a C-contiguous buffer");
        }
        return create_texture_from_bytes(size, internal_format, format, type, arr.data(), arr.nbytes());
      }),
      py::arg("size"),
      py::arg("internal_format"),
      py::arg("format"),
      py::arg("type"),
      py::arg("bytes"),
      R"doc(Create a texture from raw pixel data.

Args:
    size: Texture size (width, height).
    internal_format: Internal format of the texture (e.g., gl.RGBA8, gl.RGBA32F).
    format: Format of the pixel data (e.g., gl.RGBA, gl.RGB, gl.RED).
    type: Data type of the pixel data (e.g., gl.UNSIGNED_BYTE, gl.FLOAT).
    bytes: Pixel data as an object supporting the buffer protocol (e.g., numpy array, bytes, bytearray).
           The first row is uploaded as the first (v=0) row of the texture. If empty, the texture is created but not initialized.
)doc")
    .def(
      py::init([](const Eigen::Vector2i& size, unsigned int internal_format, unsigned int format, unsigned int type, const std::vector<std::uint8_t>& bytes) {
        return create_texture_from_bytes(size, internal_format, format, type, bytes.data(), bytes.size());
      }),
      py::arg("size"),
      py::arg("internal_format"),
      py::arg("format"),
      py::arg("type"),
      py::arg("bytes"),
      R"doc(Create a texture from raw pixel data given as a list of bytes.

Args:
    size: Texture size (width, height).
    internal_format: Internal format of the texture (e.g., gl.RGBA8, gl.RGBA32F).
    format: Format of the pixel data (e.g., gl.RGBA, gl.RGB, gl.RED).
    type: Data type of the pixel data (e.g., gl.UNSIGNED_BYTE, gl.FLOAT).
    bytes: Pixel data as a list of byte values. If empty, the texture is created but not initialized.
)doc")
    .def("id", &glk::Texture::id, "OpenGL texture ID. Note that it changes when set_size() is called.")
    .def("size", &glk::Texture::size, "Texture size (width, height).")
    .def(
      "set_size",
      &glk::Texture::set_size,
      py::arg("size"),
      R"doc(Change the size of the texture.
The texture is re-created: the texture data are discarded, the texture ID changes, and the filter and clamp modes are reset to the defaults.
Do not call this on the viewer's frame buffer textures (e.g., color_buffer()).
)doc")
    .def("bind", py::overload_cast<>(&glk::Texture::bind, py::const_), "Bind the texture to GL_TEXTURE_2D of the active texture unit.")
    .def(
      "bind",
      py::overload_cast<GLenum>(&glk::Texture::bind, py::const_),
      py::arg("unit"),
      "Bind the texture to the specified texture unit (e.g., gl.TEXTURE0 + 1). The active texture unit is set to gl.TEXTURE0 afterwards.")
    .def("unbind", py::overload_cast<>(&glk::Texture::unbind, py::const_), "Unbind GL_TEXTURE_2D of the active texture unit.")
    .def(
      "unbind",
      py::overload_cast<GLenum>(&glk::Texture::unbind, py::const_),
      py::arg("unit"),
      "Unbind GL_TEXTURE_2D of the specified texture unit (e.g., gl.TEXTURE0 + 1). The active texture unit is set to gl.TEXTURE0 afterwards.")
    .def(
      "set_filter_mode",
      [](const glk::Texture& texture, GLenum mode) { texture.set_filer_mode(mode); },
      py::arg("mode"),
      "Set the min/mag filter mode (gl.LINEAR or gl.NEAREST).")
    .def(
      "set_clamp_mode",
      [](const glk::Texture& texture, GLenum mode) { texture.set_clamp_mode(mode); },
      py::arg("mode"),
      "Set the wrap mode for both S and T (e.g., gl.CLAMP_TO_EDGE, gl.REPEAT).")
    .def(
      "read_pixels",
      [](const glk::Texture& texture, unsigned int format, unsigned int type, int num_components) -> py::array {
        const int format_num_components = gl_format_num_components(format);
        if (num_components <= 0) {
          if (format_num_components <= 0) {
            throw std::runtime_error("unknown format (format=" + std::to_string(format) + "). Specify num_components explicitly");
          }
          num_components = format_num_components;
        } else if (format_num_components > 0 && num_components != format_num_components) {
          throw std::runtime_error(
            "num_components does not match format (num_components=" + std::to_string(num_components) + " expected=" + std::to_string(format_num_components) + ")");
        }

        switch (type) {
          case GL_UNSIGNED_BYTE:
            return read_texture_pixels<unsigned char>(texture, format, type, num_components, "uint8");
          case GL_BYTE:
            return read_texture_pixels<unsigned char>(texture, format, type, num_components, "int8");
          case GL_UNSIGNED_SHORT:
            return read_texture_pixels<std::uint16_t>(texture, format, type, num_components, "uint16");
          case GL_SHORT:
            return read_texture_pixels<std::uint16_t>(texture, format, type, num_components, "int16");
          case GL_HALF_FLOAT:
            return read_texture_pixels<std::uint16_t>(texture, format, type, num_components, "float16");
          case GL_UNSIGNED_INT:
            return read_texture_pixels<unsigned int>(texture, format, type, num_components, "uint32");
          case GL_INT:
            return read_texture_pixels<int>(texture, format, type, num_components, "int32");
          case GL_FLOAT:
            return read_texture_pixels<float>(texture, format, type, num_components, "float32");
          default:
            throw std::runtime_error("unsupported type (type=" + std::to_string(type) + ")");
        }
      },
      py::arg("format") = static_cast<unsigned int>(GL_RGBA),
      py::arg("type") = static_cast<unsigned int>(GL_UNSIGNED_BYTE),
      py::arg("num_components") = -1,
      R"doc(Read the pixel data (mip level 0) from the texture.

Args:
    format: Format of the pixel data to read (e.g., gl.RGBA, gl.RGB, gl.RED, gl.DEPTH_COMPONENT, gl.RGBA_INTEGER).
    type: Data type of the pixel data to read. It determines the dtype of the returned array
          (UNSIGNED_BYTE: uint8, BYTE: int8, UNSIGNED_SHORT: uint16, SHORT: int16, HALF_FLOAT: float16,
           UNSIGNED_INT: uint32, INT: int32, FLOAT: float32).
    num_components: Number of components per pixel. If <= 0, it is derived from format.

Returns:
    Array of shape (height, width, num_components), or (height, width) if num_components is 1.
    The first row is the first (v=0) row of the texture. For the viewer's frame buffers, it is the bottom row of the screen.
)doc");

  glk_.def(
    "create_texture",
    [](py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast> arr) -> std::shared_ptr<glk::Texture> {
      if (arr.ndim() != 2 && arr.ndim() != 3) {
        std::cerr << "ndim must be 2 or 3 (ndim=" << arr.ndim() << ")" << std::endl;
        return nullptr;
      }

      const int height = arr.shape(0);
      const int width = arr.shape(1);

      if (arr.ndim() == 2) {
        return std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, GL_RED, GL_UNSIGNED_BYTE, arr.data());
      }

      GLuint format = GL_BGR;
      switch (arr.shape(2)) {
        default:
          std::cerr << "warning: invalid ch=" << arr.shape(2) << std::endl;
          break;
        case 1:
          format = GL_RED;
          break;
        case 2:
          format = GL_RG;
          break;
        case 3:
          format = GL_BGR;
          break;
        case 4:
          format = GL_BGRA;
          break;
      }

      return std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, format, GL_UNSIGNED_BYTE, arr.data());
    },
    py::arg("image_uint8"));

  glk_.def(
    "create_texture_f",
    [](py::array_t<float, py::array::c_style | py::array::forcecast> arr) -> std::shared_ptr<glk::Texture> {
      if (arr.ndim() != 2 && arr.ndim() != 3) {
        std::cerr << "ndim must be 2 or 3 (ndim=" << arr.ndim() << ")" << std::endl;
        return nullptr;
      }

      const int height = arr.shape(0);
      const int width = arr.shape(1);

      if (arr.ndim() == 2) {
        return std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, GL_RED, GL_FLOAT, arr.data());
      }

      GLuint format = GL_BGR;
      switch (arr.shape(2)) {
        default:
          std::cerr << "warning: invalid ch=" << arr.shape(2) << std::endl;
          break;
        case 1:
          format = GL_RED;
          break;
        case 2:
          format = GL_RG;
          break;
        case 3:
          format = GL_BGR;
          break;
        case 4:
          format = GL_BGRA;
          break;
      }

      return std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, format, GL_FLOAT, arr.data());
    },
    py::arg("image_float"));

  // glk::ScreenEffect
  py::class_<glk::ScreenEffect, std::shared_ptr<glk::ScreenEffect>>(glk_, "ScreenEffect");
  py::class_<glk::NaiveScreenSpaceAmbientOcclusion, glk::ScreenEffect, std::shared_ptr<glk::NaiveScreenSpaceAmbientOcclusion>>(glk_, "NaiveScreenSpaceAmbientOcclusion")
    .def(py::init<>());
  py::class_<glk::ScreenSpaceAmbientOcclusion, glk::ScreenEffect, std::shared_ptr<glk::ScreenSpaceAmbientOcclusion>>(glk_, "ScreenSpaceAmbientOcclusion").def(py::init<>());
  py::class_<glk::ScreenSpaceLighting, glk::ScreenEffect, std::shared_ptr<glk::ScreenSpaceLighting>> ssli(glk_, "ScreenSpaceLighting");

  // enums
  py::enum_<glk::ScreenSpaceLighting::DIFFUSE_MODEL>(ssli, "DIFFUSE_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::DIFFUSE_MODEL::ZERO)
    .value("LAMBERT", glk::ScreenSpaceLighting::DIFFUSE_MODEL::LAMBERT)
    .value("DISNEY", glk::ScreenSpaceLighting::DIFFUSE_MODEL::DISNEY)
    .value("NORMALIZED_DISNEY", glk::ScreenSpaceLighting::DIFFUSE_MODEL::NORMALIZED_DISNEY)
    .value("OREN_NAYAR", glk::ScreenSpaceLighting::DIFFUSE_MODEL::OREN_NAYAR)
    .export_values();

  py::enum_<glk::ScreenSpaceLighting::SPECULAR_MODEL>(ssli, "SPECULAR_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::SPECULAR_MODEL::ZERO)
    .value("PHONG", glk::ScreenSpaceLighting::SPECULAR_MODEL::PHONG)
    .value("BLINN_PHONG", glk::ScreenSpaceLighting::SPECULAR_MODEL::BLINN_PHONG)
    .value("COOK_TORRANCE", glk::ScreenSpaceLighting::SPECULAR_MODEL::COOK_TORRANCE)
    .export_values();

  py::enum_<glk::ScreenSpaceLighting::OCCLUSION_MODEL>(ssli, "OCCLUSION_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::OCCLUSION_MODEL::ZERO)
    .value("AMBIENT_OCCLUSION", glk::ScreenSpaceLighting::OCCLUSION_MODEL::AMBIENT_OCCLUSION)
    .export_values();

  py::enum_<glk::ScreenSpaceLighting::IRIDESCENCE_MODEL>(ssli, "IRIDESCENCE_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::ZERO)
    .value("IRIDESCENCE1", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::IRIDESCENCE1)
    .value("IRIDESCENCE2", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::IRIDESCENCE2)
    .value("IRIDESCENCE3", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::IRIDESCENCE3)
    .export_values();

  ssli.def(py::init<>())
    .def("num_lights", &glk::ScreenSpaceLighting::num_lights, "")
    .def("set_diffuse_model", &glk::ScreenSpaceLighting::set_diffuse_model)
    .def("set_specular_model", &glk::ScreenSpaceLighting::set_specular_model)
    .def("set_occlusion_model", &glk::ScreenSpaceLighting::set_occlusion_model)
    .def("set_iridescence_model", &glk::ScreenSpaceLighting::set_iridescence_model)
    .def(
      "set_light",
      [](glk::ScreenSpaceLighting& effect, int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color) { return effect.set_light(i, pos, color); },
      "")
    .def("set_albedo", &glk::ScreenSpaceLighting::set_albedo, "")
    .def("set_roughness", &glk::ScreenSpaceLighting::set_roughness, "");

  // methods
  glk_.def("get_data_path", glk::get_data_path, "");
  glk_.def("set_data_path", &glk::set_data_path, "");
  glk_.def(
    "create_pointcloud_buffer",
    [](const Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& points, const Eigen::Matrix<float, -1, 4, Eigen::RowMajor>& colors) -> std::shared_ptr<glk::PointCloudBuffer> {
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points.data(), sizeof(float) * 3, points.rows());
      if (colors.rows() == points.rows()) {
        cloud_buffer->add_color(colors.data(), sizeof(Eigen::Vector4f), colors.rows());
      }
      return cloud_buffer;
    },
    "",
    py::arg("points"),
    py::arg("colors") = Eigen::Matrix<float, -1, 4, Eigen::RowMajor>());

  // colormaps
  glk_.def("colormap", &glk::colormap, py::arg("colormap_type"), py::arg("x"));
  glk_.def("colormapf", &glk::colormapf, py::arg("colormap_type"), py::arg("x"));
  glk_.def("colormap_categorical", &glk::colormap_categorical, py::arg("colormap_type"), py::arg("x"), py::arg("num_categories"));
  glk_.def("colormap_categoricalf", &glk::colormap_categoricalf, py::arg("colormap_type"), py::arg("x"), py::arg("num_categories"));

  // primitives
  primitives_.def("sphere", [] { return glk::Primitives::sphere(); });
  primitives_.def("cube", [] { return glk::Primitives::cube(); });
  primitives_.def("cone", [] { return glk::Primitives::cone(); });
  primitives_.def("icosahedron", [] { return glk::Primitives::icosahedron(); });
  primitives_.def("bunny", [] { return glk::Primitives::bunny(); });
  primitives_.def("coordinate_system", [] { return glk::Primitives::coordinate_system(); });
  primitives_.def("frustum", [] { return glk::Primitives::frustum(); });
  primitives_.def("wire_sphere", [] { return glk::Primitives::wire_sphere(); });
  primitives_.def("wire_cube", [] { return glk::Primitives::wire_cube(); });
  primitives_.def("wire_cone", [] { return glk::Primitives::wire_cone(); });
  primitives_.def("wire_icosahedron", [] { return glk::Primitives::wire_icosahedron(); });
  primitives_.def("wire_bunny", [] { return glk::Primitives::wire_bunny(); });
  primitives_.def("wire_frustum", [] { return glk::Primitives::wire_frustum(); });

  // IO
  // glk::PLYPropertyType
  py::enum_<glk::PLYPropertyType>(glk_, "PLYPropertyType")
    .value("CHAR", glk::PLYPropertyType::CHAR)
    .value("UCHAR", glk::PLYPropertyType::UCHAR)
    .value("SHORT", glk::PLYPropertyType::SHORT)
    .value("USHORT", glk::PLYPropertyType::USHORT)
    .value("INT", glk::PLYPropertyType::INT)
    .value("UINT", glk::PLYPropertyType::UINT)
    .value("FLOAT", glk::PLYPropertyType::FLOAT)
    .value("DOUBLE", glk::PLYPropertyType::DOUBLE)
    .export_values();

  // glk::PLYGenericPropertyBuffer
  py::class_<glk::PLYGenericPropertyBuffer, std::shared_ptr<glk::PLYGenericPropertyBuffer>>(glk_, "PLYGenericPropertyBuffer")
    .def_property_readonly("name", [](glk::PLYGenericPropertyBuffer& self) { return self.name; })
    .def_property_readonly("type", [](glk::PLYGenericPropertyBuffer& self) { return self.type(); })
    .def_property_readonly("size", [](glk::PLYGenericPropertyBuffer& self) { return self.size(); })
    .def(
      "as_uint8",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<unsigned char> {
        const unsigned char* data = self.get<unsigned char>();
        return std::vector<unsigned char>(data, data + self.size());
      })
    .def(
      "as_int8",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<char> {
        const char* data = self.get<char>();
        return std::vector<char>(data, data + self.size());
      })
    .def(
      "as_uint16",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<unsigned short> {
        const unsigned short* data = self.get<unsigned short>();
        return std::vector<unsigned short>(data, data + self.size());
      })
    .def(
      "as_int16",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<short> {
        const short* data = self.get<short>();
        return std::vector<short>(data, data + self.size());
      })
    .def(
      "as_uint32",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<unsigned int> {
        const unsigned int* data = self.get<unsigned int>();
        return std::vector<unsigned int>(data, data + self.size());
      })
    .def(
      "as_int32",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<int> {
        const int* data = self.get<int>();
        return std::vector<int>(data, data + self.size());
      })
    .def(
      "as_float",
      [](glk::PLYGenericPropertyBuffer& self) -> std::vector<float> {
        const float* data = self.get<float>();
        return std::vector<float>(data, data + self.size());
      })
    .def("as_double", [](glk::PLYGenericPropertyBuffer& self) -> std::vector<double> {
      const double* data = self.get<double>();
      return std::vector<double>(data, data + self.size());
    });

  // glk::PLYData
  py::class_<glk::PLYData, std::shared_ptr<glk::PLYData>>(glk_, "PLYData")  //
    .def(py::init<>())
    .def_property(
      "vertices",
      [](glk::PLYData& self) -> Eigen::Matrix<float, -1, -1, Eigen::RowMajor> {
        return Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(self.vertices[0].data(), self.vertices.size(), 3);
      },
      [](glk::PLYData& self, const Eigen::MatrixXf& vertices) {
        if (vertices.cols() != 3 && vertices.cols() != 4) {
          std::cerr << "vertices must be 3 or 4 columns (cols=" << vertices.cols() << ")" << std::endl;
          return;
        }

        self.vertices.resize(vertices.rows());
        for (int i = 0; i < vertices.rows(); i++) {
          self.vertices[i] = vertices.row(i).head<3>();
        }
      })
    .def_property(
      "normals",
      [](glk::PLYData& self) -> Eigen::Matrix<float, -1, -1, Eigen::RowMajor> {
        return Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(self.normals[0].data(), self.normals.size(), 3);
      },
      [](glk::PLYData& self, const Eigen::MatrixXf& normals) {
        if (normals.cols() != 3 && normals.cols() != 4) {
          std::cerr << "normals must be 3 or 4 columns (cols=" << normals.cols() << ")" << std::endl;
          return;
        }

        self.normals.resize(normals.rows());
        for (int i = 0; i < normals.rows(); i++) {
          self.normals[i] = normals.row(i).head<3>();
        }
      })
    .def_property(
      "intensities",
      [](glk::PLYData& self) -> Eigen::VectorXf { return Eigen::Map<const Eigen::VectorXf>(self.intensities.data(), self.intensities.size()); },
      [](glk::PLYData& self, const Eigen::VectorXf& intensities) {
        self.intensities.resize(intensities.size());
        for (int i = 0; i < intensities.size(); i++) {
          self.intensities[i] = intensities[i];
        }
      })
    .def_property(
      "colors",
      [](glk::PLYData& self) -> Eigen::Matrix<float, -1, -1, Eigen::RowMajor> {
        return Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(self.colors[0].data(), self.colors.size(), 4);
      },
      [](glk::PLYData& self, const Eigen::MatrixXf& colors) {
        if (colors.cols() != 4) {
          std::cerr << "colors must be 3 or 4 columns (cols=" << colors.cols() << ")" << std::endl;
          return;
        }

        self.colors.resize(colors.rows());
        for (int i = 0; i < colors.rows(); i++) {
          self.colors[i] = colors.row(i);
        }
      })
    .def_property(
      "indices",
      [](glk::PLYData& self) -> Eigen::VectorXi { return Eigen::Map<const Eigen::VectorXi>(self.indices.data(), self.indices.size()); },
      [](glk::PLYData& self, const Eigen::VectorXi& indices) {
        self.indices.resize(indices.size());
        for (int i = 0; i < indices.size(); i++) {
          self.indices[i] = indices[i];
        }
      })
    .def_property_readonly("properties", [](glk::PLYData& self) { return self.properties; })
    .def_property(
      "comments",
      [](glk::PLYData& self) -> std::vector<std::string> { return self.comments; },
      [](glk::PLYData& self, const std::vector<std::string>& comments) { self.comments = comments; })
    .def("get_prop", [](glk::PLYData& self, const std::string& name) -> std::shared_ptr<glk::PLYGenericPropertyBuffer> {
      for (const auto& prop : self.properties) {
        if (prop->name == name) {
          return prop;
        }
      }
      return nullptr;
    });

  glk_.def("load_ply", &glk::load_ply, py::arg("filename"));
  glk_.def("save_ply", &glk::save_ply, py::arg("filename"), py::arg("ply"), py::arg("binary") = true);
  glk_.def("save_ply", [](const std::string& filename, const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points) {
    if (points.cols() != 3 && points.cols() != 4) {
      std::cerr << "points must be 3 or 4 columns (cols=" << points.cols() << ")" << std::endl;
      return false;
    }

    if (points.cols() == 3) {
      return glk::save_ply_binary(filename, reinterpret_cast<const Eigen::Matrix<float, 3, 1>*>(points.data()), points.rows());
    } else {
      return glk::save_ply_binary(filename, reinterpret_cast<const Eigen::Matrix<float, 4, 1>*>(points.data()), points.rows());
    }
  });
}
