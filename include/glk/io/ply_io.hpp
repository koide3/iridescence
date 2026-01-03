#ifndef GLK_PLY_IO_HPP
#define GLK_PLY_IO_HPP

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace glk {

/// @brief PLY property type.
enum class PLYPropertyType { CHAR, UCHAR, SHORT, USHORT, INT, UINT, FLOAT, DOUBLE };

/// @brief Get the PLY property type for a specific type.
/// @tparam T  Primitive type (e.g., std::uint32_t, float).
/// @return    PLY property type.
template <typename T>
PLYPropertyType ply_prop_type();

/// @brief PLY file metadata.
struct PLYMetaData {
  PLYMetaData() : num_vertices(0), num_faces(0) {}

  std::string format;                 // PLY format (ascii or binary_big_endian).
  std::vector<std::string> comments;  // Comments.

  int num_vertices;                                                        // Number of vertices.
  int num_faces;                                                           // Number of faces.
  std::vector<std::pair<std::string, PLYPropertyType>> vertex_properties;  // Property names and types.
  std::vector<PLYPropertyType> face_properties;                            // Face property types
};

/// @brief PLY property buffer.
struct PLYGenericPropertyBuffer {
public:
  using Ptr = std::shared_ptr<PLYGenericPropertyBuffer>;
  using ConstPtr = std::shared_ptr<const PLYGenericPropertyBuffer>;

  PLYGenericPropertyBuffer(const std::string& name) : name(name) {}
  virtual ~PLYGenericPropertyBuffer() = default;

  virtual std::shared_ptr<PLYGenericPropertyBuffer> clone() const = 0;

  virtual PLYPropertyType type() const = 0;

  /// @brief Get the number of elements.
  virtual size_t size() const = 0;

  /// @brief Get the raw pointer to the data.
  virtual void* get() = 0;

  template <typename T>
  T* get() {
    return static_cast<T*>(get());
  }

  virtual void read_from_buffer(char* buffer, int offset, size_t index) = 0;
  virtual void read_from_stream(std::istream& is, int offset, size_t index) = 0;

  virtual void write_to_buffer(char* buffer, int offset, size_t index) const = 0;
  virtual void write_to_stream(std::ostream& os, int offset, size_t index) const = 0;

public:
  const std::string name;
};

/// @brief PLY property buffer for a specific type.
template <typename T>
struct PLYPropertyBuffer : public PLYGenericPropertyBuffer {
public:
  PLYPropertyBuffer(const std::string& name, size_t size) : PLYGenericPropertyBuffer(name), data(size) {}
  PLYPropertyBuffer(const std::string& name, const T* data, size_t size) : PLYGenericPropertyBuffer(name), data(data, data + size) {}

  std::shared_ptr<PLYGenericPropertyBuffer> clone() const override { return std::make_shared<PLYPropertyBuffer<T>>(name, data.data(), data.size()); }

  PLYPropertyType type() const override { return ply_prop_type<T>(); }

  size_t size() const override { return data.size(); }

  void* get() override { return data.data(); }

  void read_from_buffer(char* buffer, int offset, size_t index) override { data[index] = *reinterpret_cast<T*>(buffer + offset); }

  void read_from_stream(std::istream& is, int offset, size_t index) override {
    if constexpr (std::is_integral_v<T>) {
      int value;
      is >> value;
      data[index] = value;
    } else {
      is >> data[index];
    }
  }

  void write_to_buffer(char* buffer, int offset, size_t index) const override { *reinterpret_cast<T*>(buffer + offset) = data[index]; }

  void write_to_stream(std::ostream& os, int offset, size_t index) const override {
    if (offset) {
      os << " ";
    }
    if constexpr (std::is_integral_v<T>) {
      os << static_cast<int>(data[index]);
    } else {
      os << data[index];
    }
  }

public:
  std::vector<T> data;
};  // namespace

/// @brief PLY data.
/// @note  When saving, primary properties (vertices, normals...) are first written, and then the generic properties are written to the file.
///        If a property exists in both generic and primary properties, the primary property is prioritized.
///        (e.g., `vertices[0]` overwrites `properties[name =="x"]`).
struct PLYData {
public:
  template <typename T>
  void add_prop(const std::string& name, const T* data, size_t size) {
    properties.emplace_back(std::make_shared<PLYPropertyBuffer<T>>(name, data, size));
  }

public:
  std::vector<Eigen::Vector3f> vertices;  // Vertex positions (saved as "x", "y", "z" properties).
  std::vector<Eigen::Vector3f> normals;   // Vertex normals (saved as "nx", "ny", "nz" properties).
  std::vector<float> intensities;         // Intensity values (saved as "intensity" property).
  std::vector<Eigen::Vector4f> colors;    // RGBA colors (saved as "r", "g", "b", "a" properties).
  std::vector<int> indices;

  std::vector<std::string> comments;
  std::vector<PLYGenericPropertyBuffer::Ptr> properties;
};

/// @brief            Load a PLY file.
/// @param filename   File path.
/// @return           PLY data if successful, nullptr otherwise.
std::shared_ptr<PLYData> load_ply(const std::string& filename);

/// @brief            Save a PLY file.
/// @param filename   File path.
/// @param ply        PLY data.
/// @param binary     Binary flag.
/// @return           True if successful, false otherwise.
bool save_ply(const std::string& filename, const PLYData& ply, bool binary = true);
bool save_ply_ascii(const std::string& filename, const PLYData& ply);
bool save_ply_binary(const std::string& filename, const PLYData& ply);

template <typename T, int D>
bool save_ply_binary(const std::string& filename, const Eigen::Matrix<T, D, 1>* points, int num_points);

}  // namespace glk

#endif