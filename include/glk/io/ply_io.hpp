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
PLYPropertyType prop_type();

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

  PLYGenericPropertyBuffer(const std::string& name, PLYPropertyType type, int offset) : name(name), type(type), offset(offset) {}
  virtual ~PLYGenericPropertyBuffer() = default;

  /// @brief Get the number of elements.
  virtual size_t size() const = 0;

  /// @brief Get the raw pointer to the data.
  virtual void* get() = 0;

  template <typename T>
  T* get() {
    return static_cast<T*>(get());
  }

  virtual void read_from_buffer(char* buffer, size_t index) = 0;
  virtual void read_from_stream(std::istream& is, size_t index) = 0;

  virtual void write_to_buffer(char* buffer, size_t index) const = 0;
  virtual void write_to_stream(std::ostream& os, size_t index) const = 0;

public:
  const std::string name;
  const PLYPropertyType type;
  int offset;
};

/// @brief PLY property buffer for a specific type.
template <typename T>
struct PLYPropertyBuffer : public PLYGenericPropertyBuffer {
public:
  PLYPropertyBuffer(const std::string& name, size_t size, int offset = -1) : PLYGenericPropertyBuffer(name, prop_type<T>(), offset), data(size) {}
  PLYPropertyBuffer(const std::string& name, const T* data, size_t size, int offset = -1) : PLYGenericPropertyBuffer(name, prop_type<T>(), offset), data(data, data + size) {}

  size_t size() const override { return data.size(); }

  void* get() override { return data.data(); }

  void read_from_buffer(char* buffer, size_t index) override { data[index] = *reinterpret_cast<T*>(buffer + offset); }

  void read_from_stream(std::istream& is, size_t index) override {
    if (type == PLYPropertyType::FLOAT || type == PLYPropertyType::DOUBLE) {
      is >> data[index];
    } else {
      int value;
      is >> value;
      data[index] = value;
    }
  }

  void write_to_buffer(char* buffer, size_t index) const override { *reinterpret_cast<T*>(buffer + offset) = data[index]; }

  void write_to_stream(std::ostream& os, size_t index) const override {
    if (offset) {
      os << " ";
    }
    if (type == PLYPropertyType::FLOAT || type == PLYPropertyType::DOUBLE) {
      os << data[index];
    } else {
      os << static_cast<int>(data[index]);
    }
  }

public:
  std::vector<T> data;
};  // namespace

/// @brief PLY data.
/// @note  When saving, vertices, normals, intensities, and colors are first written to the properties, and then the generic properties are written to the file.
///        If a property exists in both the generic and specific properties, the specific property is used (i.e., `vertices[0]` overwrites `properties[name == "x"]`).
struct PLYData {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<float> intensities;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
  std::vector<int> indices;

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