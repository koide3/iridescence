#ifndef GUIK_SHADER_SETTING_HPP
#define GUIK_SHADER_SETTING_HPP

#include <memory>
#include <optional>
#include <unordered_map>

#include <glk/drawable.hpp>
#include <glk/make_shared.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>

namespace guik {

/// @brief Fragment color mode.
///        RAINBOW         : Color mapped by 3D coordinates of pixels (by default, Z-axis is used for color mapping)
///        FLAT_COLOR      : Single flat color
///        VERTEX_COLOR    : Color from vertex color attribute
///        TEXTURE_COLOR   : Color from texture map
///        VERTEX_COLORMAP : Color from vertex colormap attribute
struct ColorMode {
  enum MODE { RAINBOW = 0, FLAT_COLOR = 1, VERTEX_COLOR = 2, TEXTURE_COLOR = 3, VERTEX_COLORMAP = 4 };
};

/// @brief Point size scale mode.
///        SCREENSPACE : Screen space size
///        METRIC      : Metric size (in meters)
struct PointScaleMode {
  enum MODE { SCREENSPACE = 0, METRIC = 1 };
};

/// @brief Point shape mode.
///        RECTANGLE : Square/rectangle shape
///        CIRCLE    : Circle shape
struct PointShapeMode {
  enum MODE { RECTANGLE = 0, CIRCLE = 1 };
};

/// @brief Data type of shader parameter.
enum class ShaderDataType { INT = 0, INT2, INT3, INT4, FLOAT, FLOAT2, FLOAT3, FLOAT4, MATRIX4, INT_ARRAY, FLOAT_ARRAY, INVALID };

/// @brief  Deduce shader data type from C++ type.
template <typename T>
ShaderDataType deduce_data_type() {
  if constexpr (std::is_same<T, int>::value) {
    return ShaderDataType::INT;
  } else if constexpr (std::is_same<T, Eigen::Vector2i>::value) {
    return ShaderDataType::INT2;
  } else if constexpr (std::is_same<T, Eigen::Vector3i>::value) {
    return ShaderDataType::INT3;
  } else if constexpr (std::is_same<T, Eigen::Vector4i>::value) {
    return ShaderDataType::INT4;
  } else if constexpr (std::is_same<T, float>::value) {
    return ShaderDataType::FLOAT;
  } else if constexpr (std::is_same<T, Eigen::Vector2f>::value) {
    return ShaderDataType::FLOAT2;
  } else if constexpr (std::is_same<T, Eigen::Vector3f>::value) {
    return ShaderDataType::FLOAT3;
  } else if constexpr (std::is_same<T, Eigen::Vector4f>::value) {
    return ShaderDataType::FLOAT4;
  } else if constexpr (std::is_same<T, Eigen::Matrix4f>::value) {
    return ShaderDataType::MATRIX4;
  } else if constexpr (std::is_same<T, std::vector<int>>::value) {
    return ShaderDataType::INT_ARRAY;
  } else if constexpr (std::is_same<T, std::vector<float>>::value) {
    return ShaderDataType::FLOAT_ARRAY;
  } else {
    return ShaderDataType::INVALID;
  }
}

/// @brief Shader parameter class that holds a single parameter for shader setting.
/// @note  The maximum size of the parameter value is 4x4 float matrix.
struct ShaderParameter {
public:
  /// @brief Default constructor
  ShaderParameter() : type(ShaderDataType::INVALID), name(0), mat4f(Eigen::Matrix4f::Zero()) {}

  /// @brief Constructor
  template <typename T>
  ShaderParameter(std::uint64_t name, const T& v) : type(deduce_data_type<T>()),
                                                    name(name) {
    set_value<T>(v);
  }

  /// @brief Check if the parameter is valid.
  bool valid() const { return type != ShaderDataType::INVALID && name != 0; }

  /// @brief Invalidate the parameter.
  void invalidate() {
    type = ShaderDataType::INVALID;
    name = 0;
    mat4f.setZero();
  }

  /// @brief Get the parameter value.
  template <typename T>
  T get_value() const {
    if (type != deduce_data_type<T>()) {
      std::cerr << "error: type mismatch when getting shader parameter " << name << std::endl;
      abort();
    }

    if constexpr (std::is_same<T, std::vector<int>>::value || std::is_same<T, std::vector<float>>::value) {
      const int* ptr = reinterpret_cast<const int*>(&mat4f);
      int num_data = ptr[0];

      T v(num_data);
      memcpy(v.data(), &ptr[1], sizeof(typename T::value_type) * num_data);
      return v;
    } else {
      return *reinterpret_cast<const T*>(&mat4f);
    }
  }

  /// @brief Get the parameter value.
  template <typename T>
  void set_value(const T& value) {
    if (type != deduce_data_type<T>()) {
      std::cerr << "error: type mismatch when getting shader parameter " << name << std::endl;
      abort();
    }

    if constexpr (std::is_same<T, std::vector<int>>::value || std::is_same<T, std::vector<float>>::value) {
      if (value.size() > 15) {
        std::cerr << "error: array size exceeds the maximum limit of 15 for shader parameter " << name << std::endl;
        abort();
      }

      int* ptr = reinterpret_cast<int*>(&mat4f);
      ptr[0] = static_cast<int>(value.size());
      memcpy(&ptr[1], value.data(), sizeof(typename T::value_type) * value.size());
    } else {
      *reinterpret_cast<T*>(&mat4f) = value;
    }
  }

  /// @brief Set the parameter to the shader.
  void set(glk::GLSLShader& shader) const {
    switch (type) {
      case ShaderDataType::INT:
        shader.set_uniform(name, get_value<int>());
        break;
      case ShaderDataType::INT2:
        shader.set_uniform(name, get_value<Eigen::Vector2i>());
        break;
      case ShaderDataType::INT3:
        shader.set_uniform(name, get_value<Eigen::Vector3i>());
        break;
      case ShaderDataType::INT4:
        shader.set_uniform(name, get_value<Eigen::Vector4i>());
        break;
      case ShaderDataType::FLOAT:
        shader.set_uniform(name, get_value<float>());
        break;
      case ShaderDataType::FLOAT2:
        shader.set_uniform(name, get_value<Eigen::Vector2f>());
        break;
      case ShaderDataType::FLOAT3:
        shader.set_uniform(name, get_value<Eigen::Vector3f>());
        break;
      case ShaderDataType::FLOAT4:
        shader.set_uniform(name, get_value<Eigen::Vector4f>());
        break;
      case ShaderDataType::MATRIX4:
        shader.set_uniform(name, get_value<Eigen::Matrix4f>());
        break;
      case ShaderDataType::INT_ARRAY:
        shader.set_uniform(name, get_value<std::vector<int>>());
        break;
      case ShaderDataType::FLOAT_ARRAY:
        shader.set_uniform(name, get_value<std::vector<float>>());
        break;
      default:
        std::cerr << "error: invalid shader parameter type for " << name << std::endl;
        break;
    }
  }

public:
  ShaderDataType type;    ///< Data type of the parameter
  std::uint64_t name;     ///< Name of the parameter (hashed)
  Eigen::Matrix4f mat4f;  ///< Value of the parameter (stored as 4x4 float matrix)
};

/// @brief Shader setting class that holds rendering settings.
struct ShaderSetting {
public:
  using Ptr = std::shared_ptr<ShaderSetting>;
  using ConstPtr = std::shared_ptr<const ShaderSetting>;

  /// @brief  Default constructor
  /// @note   Default parameters are:
  ///         color_mode   : FLAT_COLOR
  ///         point_scale  : 1.0
  ///         model_matrix : Identity matrix
  ShaderSetting();

  /// @brief  Constructor
  /// @param  color_mode  Color mode (ColorMode)
  ShaderSetting(int color_mode);

  /// @brief  Constructor
  /// @param  color_mode  Color mode (ColorMode)
  /// @param  transform   Transform matrix (Eigen::Matrix4f)
  ShaderSetting(int color_mode, const Eigen::Matrix4f& transform);

  /// @brief Constructor
  /// @param color_mode Color mode (ColorMode)
  /// @param transform  Transform matrix (e.g., Eigen::Matrix4(f|d) or Eigen::Isometry3(f|d))
  template <typename Transform>
  ShaderSetting(int color_mode, const Transform& transform) : ShaderSetting(color_mode, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  /// @brief Copy constructor
  ShaderSetting(const ShaderSetting& other);

  /// @brief Copy assignment operator
  ShaderSetting& operator=(const ShaderSetting& other);

  /// @brief Destructor
  virtual ~ShaderSetting() {}

  /// @brief Clone the shader setting.
  ShaderSetting clone() const;

  /// @brief Add a new parameter to the shader setting.
  /// @param name   Parameter name
  /// @param value  Parameter value
  /// @return       This object
  template <typename T>
  ShaderSetting& add(const std::string& name_, const T& value) {
    const std::uint64_t name = glk::hash(name_);

    for (int i = 0; i < params.size() && params[i].valid(); i++) {
      if (params[i].name == name) {
        params[i] = ShaderParameter(name, value);
        return *this;
      }
    }

    params.emplace_back(ShaderParameter(name, value));
    return *this;
  }

  /// @brief Get a parameter value from the shader setting.
  /// @param name   Parameter name
  /// @return       Parameter value if found, otherwise std::nullopt
  template <typename T>
  std::optional<T> get(const std::string& name_) {
    const std::uint64_t name = glk::hash(name_);

    for (const auto& param : params) {
      if (!param.valid() || param.name != name) {
        continue;
      }

      if (param.type != deduce_data_type<T>()) {
        std::cerr << "warning: type mismatch for shader parameter " << name_ << std::endl;
        return std::nullopt;
      }

      return param.get_value<T>();
    }

    return std::nullopt;
  }

  /// @brief Get a parameter value from the shader setting.
  /// @param name   Parameter name
  /// @return       Parameter value if found, otherwise std::nullopt
  template <typename T>
  std::optional<T> get(const std::string& name_) const {
    const std::uint64_t name = glk::hash(name_);

    for (const auto& param : params) {
      if (!param.valid() || param.name != name) {
        continue;
      }

      if (param.type != deduce_data_type<T>()) {
        std::cerr << "warning: type mismatch for shader parameter " << name_ << std::endl;
        return std::nullopt;
      }

      return param.get_value<T>();
    }

    return std::nullopt;
  }

  /// @brief Get a parameter value from the shader setting. This method avoid type checking. Fast but unsafe.
  /// @param name   Parameter name
  /// @return       Parameter value
  template <typename T>
  T cast(const std::string& name_) const {
    const std::uint64_t name = glk::hash(name_);

    for (const auto& param : params) {
      if (!param.valid() || param.name != name) {
        continue;
      }

      if (param.type != deduce_data_type<T>()) {
        std::cerr << "error: type mismatch for shader parameter " << name << std::endl;
        abort();
      }

      return param.get_value<T>();
    }

    std::cerr << "error: " << name << " not found in the param list" << std::endl;
    abort();
  }

  /// @brief Set shader parameters to the shader program.
  /// @param shader  Shader program
  void set(glk::GLSLShader& shader) const;

  // Object type (static/dynamic)
  /// @brief Set static object flag.
  ShaderSetting& static_object();
  /// @brief Set dynamic object flag.
  ShaderSetting& dynamic_object();

  // Color
  /// @brief Get material color (FLAT_COLOR).
  Eigen::Vector4f material_color() const;
  /// @brief Set material color (FLAT_COLOR).
  ShaderSetting& set_color(float r, float g, float b, float a);

  /// @brief Set material color (for FLAT_COLOR).
  /// @tparam Color  Eigen 4d vector type (e.g., Eigen::Vector4f, Eigen::Vector4d)
  template <typename Color>
  ShaderSetting& set_color(const Color& color_) {
    const Eigen::Vector4f color = color_.template cast<float>();
    if (color.w() < 0.999f) {
      transparent = true;
    }
    return add("material_color", color);
  }

  /// @brief  Set alpha value of the material color.
  ShaderSetting& set_alpha(float alpha);
  /// @brief  Make the object transparent (alpha blending).
  ShaderSetting& make_transparent();

  /// @brief Set RAINBOW color range.
  ShaderSetting& set_rainbow_range(const Eigen::Vector2f& minmax_z);
  /// @brief Set RAINBOW coloring axis.
  ShaderSetting& set_rainbow_axis(const Eigen::Vector3f& axis);

  // Point size and scale
  /// @brief Get point size scale factor.
  float point_scale() const;
  /// @brief Set point size scale factor.
  ShaderSetting& set_point_scale(float scaling);

  /// @brief Get point size.
  float point_size() const;
  /// @brief Set point size.
  ShaderSetting& set_point_size(float size);
  /// @brief Set point size offset.
  ShaderSetting& set_point_size_offset(float offset);

  // Point shape mode
  /// @brief Set point shape mode.
  ShaderSetting& set_point_shape_mode(guik::PointShapeMode::MODE mode);
  /// @brief Set point shape to circle.
  ShaderSetting& set_point_shape_circle();
  /// @brief  Set point shape to rectangle.
  ShaderSetting& set_point_shape_rectangle();

  // Point scale mode
  /// @brief Set point size scale mode.
  ShaderSetting& set_point_scale_mode(guik::PointScaleMode::MODE mode);
  /// @brief Set point size scale to screen space.
  ShaderSetting& set_point_scale_screenspace();
  /// @brief Set point size scale to metric.
  ShaderSetting& set_point_scale_metric();

  /// @brief Set point shape with common settings.
  /// @param point_size  Point size in meters (if metric) or in pseudo pixels (if screenspace)
  /// @param metric      If true, point size is in meters. If false, point size is in pixels.
  /// @param circle      If true, point shape is circle. If false, point shape is rectangle.
  ShaderSetting& set_point_shape(float point_size, bool metric, bool circle);

  /// @brief Set old (v0.1.9) default point shape (rectangle + screenspace)
  ShaderSetting& set_old_default_point_shape();

  // Calling `remove_model_matrix` invalidates model matrix operations (translate, rotate, and scale)
  ShaderSetting& remove_model_matrix();

  /// @brief  Get color mode.
  int color_mode() const;
  /// @brief  Set color mode.
  ShaderSetting& set_color_mode(int color_mode);

  /// @brief  Get model matrix.
  Eigen::Matrix4f model_matrix() const;
  /// @brief  Get translation vector from the model matrix.
  Eigen::Vector3f translation() const;
  /// @brief  Get rotation matrix from the model matrix.
  Eigen::Matrix3f rotation() const;

  /// @brief  Set model matrix.
  /// @tparam Transform  Transform matrix type (e.g., Eigen::Matrix4(f|d) or Eigen::Isometry3(f|d))
  template <typename Transform>
  ShaderSetting& set_model_matrix(const Transform& transform) {
    Eigen::Matrix4f mat = (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix();

    auto& p = params[2];
    p.set_value<Eigen::Matrix4f>(mat);
    return *this;
  }

  /// @brief  Apply transformation to the model matrix.
  /// @tparam Transform Transform matrix type (e.g., Eigen::Matrix4(f|d) or Eigen::Isometry3(f|d))
  /// @note   The transformation is applied after the existing transformation. (T_new = T_old * transform)
  template <typename Transform>
  ShaderSetting& transform(const Transform& transform) {
    auto& p = params[2];
    Eigen::Matrix4f current = p.get_value<Eigen::Matrix4f>();
    Eigen::Matrix4f trans_mat = (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix();
    p.set_value<Eigen::Matrix4f>(current * trans_mat);
    return *this;
  }

  /// @brief Apply translation to the model matrix.
  ShaderSetting& translate(float tx, float ty, float tz);
  /// @brief Apply translation to the model matrix.
  ShaderSetting& translate(const Eigen::Vector3f& translation);
  /// @brief  Apply translation to the model matrix.
  /// @tparam Vector  Eigen vector type (e.g., Eigen::Vector3f, Eigen::Vector3d)
  template <typename Vector>
  ShaderSetting& translate(const Vector& translation) {
    auto& p = params[2];
    const auto trans = translation.eval();
    p.mat4f.block<3, 1>(0, 3) += trans.template cast<float>().template head<3>();
    return *this;
  }

  /// @brief Apply rotation to the model matrix.
  ShaderSetting& rotate(const float angle, const Eigen::Vector3f& axis);
  /// @brief Apply rotation to the model matrix.
  template <typename Vector>
  ShaderSetting& rotate(const float angle, const Vector& axis) {
    const Eigen::Vector3f ax = axis.eval().template cast<float>();
    auto& p = params[2];
    p.mat4f = p.mat4f * (Eigen::Isometry3f::Identity() * Eigen::AngleAxisf(angle, ax)).matrix();
    return *this;
  }
  /// @brief Apply rotation to the model matrix.
  /// @param quat  Rotation quaternion (Eigen::Quaternion(f|d))
  template <typename Scalar>
  ShaderSetting& rotate(const Eigen::Quaternion<Scalar>& quat) {
    auto& p = params[2];
    p.mat4f = p.mat4f * (Eigen::Isometry3f::Identity() * quat.template cast<float>()).matrix();
    return *this;
  }
  /// @brief Apply rotation to the model matrix.
  /// @param rot  Rotation matrix (Eigen::Matrix3(f|d))
  template <typename Scalar, int Dim>
  ShaderSetting& rotate(const Eigen::Matrix<Scalar, Dim, Dim>& rot) {
    Eigen::Isometry3f R = Eigen::Isometry3f::Identity();
    R.linear() = rot.template cast<float>().template block<3, 3>(0, 0);

    auto& p = params[2];
    p.mat4f = p.mat4f * R.matrix();
    return *this;
  }

  /// @brief Apply uniform scaling to the model matrix.
  ShaderSetting& scale(float scaling);
  /// @brief Apply scaling to the model matrix (non-uniform).
  ShaderSetting& scale(float sx, float sy, float sz);
  /// @brief Apply scaling to the model matrix (non-uniform).
  ShaderSetting& scale(const Eigen::Vector3f& scaling);
  /// @brief  Apply scaling to the model matrix (non-uniform).
  /// @tparam Vector  Eigen vector type (e.g., Eigen::Vector3f, Eigen::Vector3d)
  template <typename Vector>
  std::enable_if_t<!std::is_arithmetic_v<Vector>, ShaderSetting&> scale(const Vector& scaling) {
    auto& p = params[2];
    const auto s = scaling.eval();
    p.mat4f.col(0) *= s[0];
    p.mat4f.col(1) *= s[1];
    p.mat4f.col(2) *= s[2];
    return *this;
  }

public:
  bool transparent;                     ///< If true, the object is rendered as transparent.
  std::vector<ShaderParameter> params;  ///< Shader parameters
};

template <>
inline ShaderSetting& ShaderSetting::add(const std::string& name, const Eigen::Isometry3f& value) {
  return add<Eigen::Matrix4f>(name, value.matrix());
}

template <>
inline ShaderSetting& ShaderSetting::add(const std::string& name, const Eigen::Affine3f& value) {
  return add<Eigen::Matrix4f>(name, value.matrix());
}

/// @brief Rainbow coloring scheme that assigns colors to the height (z-value) of pixels of the object.
///        The coloring band can be changed by setting `z_range` param.
///        The direction for the color mapping can be changed by setting `colormap_axis` param.
struct Rainbow : public ShaderSetting {
public:
  Rainbow() : ShaderSetting(ColorMode::RAINBOW) {}

  template <typename Transform>
  Rainbow(const Transform& transform) : ShaderSetting(ColorMode::RAINBOW, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~Rainbow() override {}
};

/// @brief Flat coloring scheme that assigns a single color to the object.
///        If the alpha value is less than 0.999f, the object is rendered as transparent.
struct FlatColor : public ShaderSetting {
public:
  explicit FlatColor(float r, float g, float b, float a = 1.0f) : ShaderSetting(ColorMode::FLAT_COLOR) {
    add("material_color", Eigen::Vector4f(r, g, b, a));
    transparent = a < 0.999f;
  }

  explicit FlatColor(const Eigen::Vector4f& color) : ShaderSetting(ColorMode::FLAT_COLOR) {
    add("material_color", color);
    transparent = color.w() < 0.999f;
  }

  template <typename Transform>
  explicit FlatColor(const Eigen::Vector4f& color, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    add("material_color", color);
    transparent = color.w() < 0.999f;
  }

  template <typename Color>
  explicit FlatColor(const Color& color) : ShaderSetting(ColorMode::FLAT_COLOR) {
    const Eigen::Vector4f c = color.eval().template cast<float>();
    add("material_color", c);
    transparent = color.w() < 0.999f;
  }

  template <typename Transform>
  explicit FlatColor(float r, float g, float b, float a, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    add("material_color", Eigen::Vector4f(r, g, b, a));
    transparent = a < 0.999f;
  }

  template <typename Color, typename Transform>
  explicit FlatColor(const Color& color, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    const Eigen::Vector4f c = color.eval().template cast<float>();
    add("material_color", c);
    transparent = color.w() < 0.999f;
  }

  template <typename T>
  explicit FlatColor(std::initializer_list<T> i) : ShaderSetting(ColorMode::FLAT_COLOR) {
    Eigen::Vector4f color = Eigen::Vector4f::Zero();
    std::copy(i.begin(), i.end(), color.data());
    add("material_color", color);
    transparent = color.w() < 0.999f;
  }

  template <typename T, typename Transform>
  explicit FlatColor(std::initializer_list<T> i, const Transform& transform)
  : ShaderSetting(ColorMode::FLAT_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {
    Eigen::Vector4f color = Eigen::Vector4f::Zero();
    std::copy(i.begin(), i.end(), color.data());
    add("material_color", color);
    transparent = color.w() < 0.999f;
  }

  virtual ~FlatColor() override {}
};

/* primitive colors */
/// @brief Flat red color
struct FlatRed : public FlatColor {
  FlatRed() : FlatColor(1.0f, 0.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatRed(const Transform& transform) : FlatColor(1.0f, 0.0f, 0.0f, 1.0f, transform) {}
};

/// @brief Flat green color
struct FlatGreen : public FlatColor {
  FlatGreen() : FlatColor(0.0f, 1.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatGreen(const Transform& transform) : FlatColor(0.0f, 1.0f, 0.0f, 1.0f, transform) {}
};

/// @brief Flat blue color
struct FlatBlue : public FlatColor {
  FlatBlue() : FlatColor(0.0f, 0.0f, 1.0f, 1.0f) {}

  template <typename Transform>
  FlatBlue(const Transform& transform) : FlatColor(0.0f, 0.0f, 1.0f, 1.0f, transform) {}
};

/// @brief Flat orange color
struct FlatOrange : public FlatColor {
  FlatOrange() : FlatColor(1.0f, 0.5f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatOrange(const Transform& transform) : FlatColor(1.0f, 0.5f, 0.0f, 1.0f, transform) {}
};

/// @brief Flat white color
struct FlatWhite : public FlatColor {
  FlatWhite() : FlatColor(1.0f, 1.0f, 1.0f, 1.0f) {}

  template <typename Transform>
  FlatWhite(const Transform& transform) : FlatColor(1.0f, 1.0f, 1.0f, 1.0f, transform) {}
};

/// @brief Flat light gray color
struct FlatGray : public FlatColor {
  FlatGray() : FlatColor(0.7f, 0.7f, 0.7f, 1.0f) {}

  template <typename Transform>
  FlatGray(const Transform& transform) : FlatColor(0.7f, 0.7f, 0.7f, 1.0f, transform) {}
};

/// @brief Flat dark gray color
struct FlatDarkGray : public FlatColor {
  FlatDarkGray() : FlatColor(0.3f, 0.3f, 0.3f, 1.0f) {}

  template <typename Transform>
  FlatDarkGray(const Transform& transform) : FlatColor(0.3f, 0.3f, 0.3f, 1.0f, transform) {}
};

/// @brief Flat black color
struct FlatBlack : public FlatColor {
  FlatBlack() : FlatColor(0.0f, 0.0f, 0.0f, 1.0f) {}

  template <typename Transform>
  FlatBlack(const Transform& transform) : FlatColor(0.0f, 0.0f, 0.0f, 1.0f, transform) {}
};

/// @brief Vertex color scheme that assigns colors based on vertex color attributes.
struct VertexColor : public ShaderSetting {
public:
  VertexColor() : ShaderSetting(ColorMode::VERTEX_COLOR) {}

  template <typename Transform>
  VertexColor(const Transform& transform) : ShaderSetting(ColorMode::VERTEX_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~VertexColor() override {}
};

/// @brief Texture color scheme that assigns colors based on the binded texture.
struct TextureColor : public ShaderSetting {
public:
  TextureColor() : ShaderSetting(ColorMode::TEXTURE_COLOR) {}

  template <typename Transform>
  TextureColor(const Transform& transform) : ShaderSetting(ColorMode::TEXTURE_COLOR, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~TextureColor() override {}
};

/// @brief Vertex colormap scheme that assigns colors based on vertex colormap attributes.
struct VertexColorMap : public ShaderSetting {
public:
  VertexColorMap() : ShaderSetting(ColorMode::VERTEX_COLORMAP) {}

  template <typename Transform>
  VertexColorMap(const Transform& transform) : ShaderSetting(ColorMode::VERTEX_COLORMAP, (transform.template cast<float>() * Eigen::Isometry3f::Identity()).matrix()) {}

  virtual ~VertexColorMap() override {}
};

}  // namespace guik

#endif