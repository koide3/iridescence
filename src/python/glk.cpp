#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <iostream>
#include <glk/path.hpp>
#include <glk/lines.hpp>
#include <glk/texture.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/io/ply_io.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_effect.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>

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
  // glk::Drawable
  py::class_<glk::Drawable, std::shared_ptr<glk::Drawable>>(glk_, "Drawable");

  // glk::ThinLines
  py::class_<glk::ThinLines, glk::Drawable, std::shared_ptr<glk::ThinLines>>(glk_, "ThinLines")
    .def(
      py::init([](const py::array_t<float, py::array::c_style | py::array::forcecast>& points, bool line_strip) {
        if (!check_valid_points(points)) {
          throw std::runtime_error("invalid points array");
        }

        return std::make_shared<glk::ThinLines>(points.data(), points.shape(0), line_strip);
      }),
      py::arg("points"),
      py::arg("line_strip") = false)
    .def(
      py::init([](
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& points,
                 const py::array_t<float, py::array::c_style | py::array::forcecast>& colors,
                 bool line_strip) {
        if (!check_valid_points(points)) {
          throw std::runtime_error("invalid points array");
        }
        if (!check_valid_colors(colors)) {
          throw std::runtime_error("invalid colors array");
        }
        if (points.shape(0) != colors.shape(0)) {
          throw std::runtime_error("points and colors must have the same number of rows");
        }

        return std::make_shared<glk::ThinLines>(points.data(), colors.data(), points.shape(0), line_strip);
      }),
      py::arg("points"),
      py::arg("colors"),
      py::arg("line_strip") = false)
    .def(
      py::init([](const std::vector<Eigen::Vector3f>& points, bool line_strip) { return std::make_shared<glk::ThinLines>(points, line_strip); }),
      "",
      py::arg("points"),
      py::arg("line_strip") = false)
    .def(
      py::init([](const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector4f>& colors, bool line_strip) {
        return std::make_shared<glk::ThinLines>(points, colors, line_strip);
      }),
      "",
      py::arg("points"),
      py::arg("colors"),
      py::arg("line_strip") = false)
    // pyplot-like constructor
    .def(
      py::init([](const Eigen::VectorXf& x, const Eigen::VectorXf& y, const Eigen::VectorXf& z, const Eigen::VectorXf& c, bool line_strip) {
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

        if (c.size()) {
          std::vector<Eigen::Vector4f> colors(size, Eigen::Vector4f::Zero());
          for (int i = 0; i < size && i < c.size(); i++) {
            colors[i] = glk::colormapf(glk::COLORMAP::TURBO, c[i]);
          }

          return std::make_shared<glk::ThinLines>(vertices.data(), colors[0].data(), size, line_strip);
        }

        return std::make_shared<glk::ThinLines>(vertices.data(), size, line_strip);
      }),
      "",
      py::arg("x") = Eigen::VectorXf(),
      py::arg("y") = Eigen::VectorXf(),
      py::arg("z") = Eigen::VectorXf(),
      py::arg("c") = Eigen::VectorXf(),
      py::arg("line_strip") = true)
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

  // glk::Texture
  py::class_<glk::Texture, std::shared_ptr<glk::Texture>>(glk_, "Texture")
    .def(
      py::init([](const Eigen::Vector2i& size, unsigned int internal_format, unsigned int format, unsigned int type, const std::vector<std::uint8_t>& bytes) {
        return std::make_shared<glk::Texture>(size, internal_format, format, type, bytes.data());
      }),
      py::arg("size"),
      py::arg("internal_format"),
      py::arg("format"),
      py::arg("type"),
      py::arg("bytes"));
  ;

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
    .def_property(
      "comments",
      [](glk::PLYData& self) -> std::vector<std::string> { return self.comments; },
      [](glk::PLYData& self, const std::vector<std::string>& comments) { self.comments = comments; });

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
