#include <glk/io/ply_io.hpp>

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <Eigen/Core>

#include <glk/mesh_utils.hpp>
#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

#define DEFINE_PLY_PROP_TYPE(type, proptype) \
  template <>                                \
  PLYPropertyType ply_prop_type<type>() {    \
    return PLYPropertyType::proptype;        \
  }

DEFINE_PLY_PROP_TYPE(std::int8_t, CHAR)
DEFINE_PLY_PROP_TYPE(std::uint8_t, UCHAR)
DEFINE_PLY_PROP_TYPE(std::int16_t, SHORT)
DEFINE_PLY_PROP_TYPE(std::uint16_t, USHORT)
DEFINE_PLY_PROP_TYPE(std::int32_t, INT)
DEFINE_PLY_PROP_TYPE(std::uint32_t, UINT)
DEFINE_PLY_PROP_TYPE(float, FLOAT)
DEFINE_PLY_PROP_TYPE(double, DOUBLE)

namespace {

PLYPropertyType parse_property_type(const std::string& type) {
  if (type == "char" || type == "int8") {
    return PLYPropertyType::CHAR;
  } else if (type == "uchar" || type == "uint8") {
    return PLYPropertyType::UCHAR;
  } else if (type == "short" || type == "int16") {
    return PLYPropertyType::SHORT;
  } else if (type == "ushort" || type == "uint16") {
    return PLYPropertyType::USHORT;
  } else if (type == "int" || type == "int32") {
    return PLYPropertyType::INT;
  } else if (type == "uint" || type == "uint32") {
    return PLYPropertyType::UINT;
  } else if (type == "float" || type == "float32") {
    return PLYPropertyType::FLOAT;
  } else if (type == "double" || type == "float64") {
    return PLYPropertyType::DOUBLE;
  }

  std::cerr << console::bold_red << "error: unknown ply property type " << type << console::reset << std::endl;
  return PLYPropertyType::CHAR;
}

int property_bytes(PLYPropertyType prop) {
  switch (prop) {
    case PLYPropertyType::CHAR:
    case PLYPropertyType::UCHAR:
      return 1;
    case PLYPropertyType::SHORT:
    case PLYPropertyType::USHORT:
      return 2;
    case PLYPropertyType::INT:
    case PLYPropertyType::UINT:
    case PLYPropertyType::FLOAT:
      return 4;
    case PLYPropertyType::DOUBLE:
      return 8;
    default:
      std::cerr << console::bold_red << "error: invalid point property type!!" << console::reset << std::endl;
      return 1;
  }
}

template <typename T>
T property_cast(const char* data, PLYPropertyType type) {
  switch (type) {
    case PLYPropertyType::CHAR:
      return static_cast<T>(*reinterpret_cast<const int8_t*>(data));
    case PLYPropertyType::UCHAR:
      return static_cast<T>(*reinterpret_cast<const uint8_t*>(data));
    case PLYPropertyType::SHORT:
      return static_cast<T>(*reinterpret_cast<const int16_t*>(data));
    case PLYPropertyType::USHORT:
      return static_cast<T>(*reinterpret_cast<const uint16_t*>(data));
    case PLYPropertyType::INT:
      return static_cast<T>(*reinterpret_cast<const int32_t*>(data));
    case PLYPropertyType::UINT:
      return static_cast<T>(*reinterpret_cast<const uint32_t*>(data));
    case PLYPropertyType::FLOAT:
      return static_cast<T>(*reinterpret_cast<const float*>(data));
    case PLYPropertyType::DOUBLE:
      return static_cast<T>(*reinterpret_cast<const double*>(data));
    default:
      std::cerr << console::bold_red << "error: unknown point property!!" << console::reset << std::endl;
      return static_cast<T>(0);
  }
}

std::shared_ptr<PLYData> load_ply_body(std::ifstream& ifs, const PLYMetaData& meta_data) {
  if (meta_data.format.find("big_endian") != std::string::npos) {
    std::cerr << console::bold_red << "error: big endian is not supported!!" << console::reset << std::endl;
    return nullptr;
  }

  auto ply = std::make_shared<PLYData>();
  ply->comments = meta_data.comments;

  // Create property buffers
  int sum_property_offsets = 0;
  std::vector<int> property_offsets;
  for (int i = 0; i < meta_data.vertex_properties.size(); i++) {
    const auto& name = meta_data.vertex_properties[i].first;
    property_offsets.emplace_back(sum_property_offsets);
    sum_property_offsets += property_bytes(meta_data.vertex_properties[i].second);

    switch (meta_data.vertex_properties[i].second) {
      case PLYPropertyType::CHAR:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<int8_t>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::UCHAR:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<uint8_t>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::SHORT:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<int16_t>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::USHORT:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<uint16_t>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::INT:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<int32_t>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::UINT:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<uint32_t>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::FLOAT:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<float>>(name, meta_data.num_vertices));
        break;
      case PLYPropertyType::DOUBLE:
        ply->properties.emplace_back(std::make_shared<PLYPropertyBuffer<double>>(name, meta_data.num_vertices));
        break;
    }
  }

  // Read binary body
  if (meta_data.format.find("binary") != std::string::npos) {
    const int vertex_step = sum_property_offsets;
    std::vector<char> vertex_buffer(vertex_step * meta_data.num_vertices);
    ifs.read(vertex_buffer.data(), vertex_buffer.size());

    // Read vertices
    for (size_t i = 0; i < meta_data.num_vertices; i++) {
      char* buff = vertex_buffer.data() + vertex_step * i;
      for (size_t j = 0; j < ply->properties.size(); j++) {
        ply->properties[j]->read_from_buffer(buff, property_offsets[j], i);
      }
    }

    // Read faces
    if (meta_data.face_properties.size() == 2) {
      ply->indices.resize(meta_data.num_faces * 3);

      const auto count_type = meta_data.face_properties[0];
      const auto index_type = meta_data.face_properties[1];

      const int face_size = property_bytes(count_type) + property_bytes(index_type) * 3;
      std::vector<char> index_buffer(face_size * meta_data.num_faces);
      ifs.read(index_buffer.data(), index_buffer.size());

      const char* data_itr = index_buffer.data();
      for (int i = 0; i < meta_data.num_faces; i++) {
        int num_vertices = property_cast<int>(data_itr, count_type);
        if (num_vertices != 3) {
          std::cerr << console::yellow << "warning: non-triangle faces are not supported!!" << console::reset << std::endl;
          ply->indices.clear();
          break;
        }

        data_itr += property_bytes(count_type);

        for (int j = 0; j < 3; j++) {
          ply->indices[i * 3 + 2 - j] = property_cast<int>(data_itr, index_type);
          data_itr += property_bytes(index_type);
        }
      }
    }
  }
  // Read ascii body
  else {
    // Read vertices
    for (size_t i = 0; i < meta_data.num_vertices; i++) {
      for (size_t j = 0; j < ply->properties.size(); j++) {
        ply->properties[j]->read_from_stream(ifs, property_offsets[j], i);
      }
    }
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Read faces
    ply->indices.resize(meta_data.num_faces * 3);
    for (int i = 0; i < meta_data.num_faces; i++) {
      std::string line;
      std::getline(ifs, line);

      int faces = 0;
      std::stringstream sst(line);
      sst >> faces >> ply->indices[i * 3 + 2] >> ply->indices[i * 3 + 1] >> ply->indices[i * 3];

      if (faces != 3) {
        std::cerr << bold_red << "error : only faces with three vertices are supported!!" << reset << std::endl;
        ply->indices.clear();
        break;
      }
    }
  }

  // Fill primary fields
  const auto populate_property = [&](const std::vector<std::string>& field_names, auto& dst, int element_idx) {
    // Find a generic property that matches the field name
    PLYGenericPropertyBuffer::Ptr prop = nullptr;
    for (const auto& field_name : field_names) {
      const auto found = std::find_if(ply->properties.begin(), ply->properties.end(), [&](const auto& p) { return p->name == field_name; });
      if (found != ply->properties.end()) {
        prop = *found;
        break;
      }
    }

    if (!prop) {
      return;
    }

    // Populate the field
    // Note: The elements in the property buffer are assumed to be floats
    dst.resize(meta_data.num_vertices);
    switch (prop->type()) {
      case PLYPropertyType::FLOAT:
        for (size_t i = 0; i < meta_data.num_vertices; i++) {
          reinterpret_cast<float*>(&dst[i])[element_idx] = prop->get<float>()[i];
        }
        break;
      case PLYPropertyType::DOUBLE:
        for (size_t i = 0; i < meta_data.num_vertices; i++) {
          reinterpret_cast<float*>(&dst[i])[element_idx] = prop->get<double>()[i];
        }
        break;
      case PLYPropertyType::CHAR:
        for (size_t i = 0; i < meta_data.num_vertices; i++) {
          reinterpret_cast<float*>(&dst[i])[element_idx] = prop->get<int8_t>()[i] / 128.0f;
        }
        break;
      case PLYPropertyType::UCHAR:
        for (size_t i = 0; i < meta_data.num_vertices; i++) {
          reinterpret_cast<float*>(&dst[i])[element_idx] = prop->get<uint8_t>()[i] / 255.0f;
        }
        break;
      default:
        std::cerr << console::bold_red << "error: unsupported property type!! (type=" << static_cast<int>(prop->type()) << ")" << console::reset << std::endl;
        dst.clear();
        break;
    }
  };

  populate_property({"x"}, ply->vertices, 0);
  populate_property({"y"}, ply->vertices, 1);
  populate_property({"z"}, ply->vertices, 2);
  populate_property({"nx"}, ply->normals, 0);
  populate_property({"ny"}, ply->normals, 1);
  populate_property({"nz"}, ply->normals, 2);
  populate_property({"r", "red"}, ply->colors, 0);
  populate_property({"g", "green"}, ply->colors, 1);
  populate_property({"b", "blue"}, ply->colors, 2);
  populate_property({"a", "alpha"}, ply->colors, 3);
  populate_property({"intensity", "scalar_Intensity", "scalar_intensity"}, ply->intensities, 0);

  if (ply->normals.empty() && !ply->vertices.empty() && !ply->indices.empty()) {
    NormalEstimater nest(ply->vertices, ply->indices);
    ply->normals = nest.normals;
  }

  return ply;
}

}  // namespace

std::shared_ptr<PLYData> load_ply(const std::string& filename) {
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    std::cerr << bold_red << "error: failed to open " << filename << reset << std::endl;
    return nullptr;
  }

  // Check the magic number
  std::string line;
  std::getline(ifs, line);
  if (line != "ply") {
    std::cerr << bold_red << "error: not a ply file" << reset << std::endl;
    return nullptr;
  }

  const auto starts_with = [](const std::string& str, const std::string& prefix) { return str.size() >= prefix.size() && str.substr(0, prefix.size()) == prefix; };

  // Read header
  PLYMetaData meta_data;
  while (!ifs.eof()) {
    std::getline(ifs, line);

    if (line.empty()) {
      continue;
    }

    std::stringstream sst(line);
    std::string token;

    if (starts_with(line, "format")) {
      sst >> token >> meta_data.format;
      continue;
    }

    if (starts_with(line, "comment")) {
      meta_data.comments.emplace_back(line.substr(8));
      continue;
    }

    if (starts_with(line, "element vertex")) {
      sst >> token >> token >> meta_data.num_vertices;
    }
    if (starts_with(line, "element face")) {
      sst >> token >> token >> meta_data.num_faces;
    }

    if (starts_with(line, "property")) {
      std::string property;
      std::string type;
      sst >> token >> type;

      if (type != "list") {
        sst >> property;
        PLYPropertyType ply_prop_type = parse_property_type(type);
        meta_data.vertex_properties.push_back(std::make_pair(property, ply_prop_type));
      } else {
        std::string num_type;
        std::string index_type;
        sst >> num_type >> index_type;
        meta_data.face_properties.push_back(parse_property_type(num_type));
        meta_data.face_properties.push_back(parse_property_type(index_type));
      }
    }

    if (line.find("end_header") != std::string::npos) {
      break;
    }
  }

  return load_ply_body(ifs, meta_data);
}

struct PLYFace {
  int32_t num_indices;
  int32_t indices[3];
};

bool save_ply(const std::string& filename, const PLYData& ply, bool binary) {
  std::ofstream ofs;
  if (binary) {
    ofs.open(filename, std::ios::binary);
  } else {
    ofs.open(filename);
  }

  if (!ofs) {
    std::cerr << bold_red << "error: failed to open " << filename << std::endl;
    return false;
  }

  std::vector<int> property_offsets;
  std::vector<PLYGenericPropertyBuffer::Ptr> props;

  // Populate primary fields
  size_t sum_prop_offsets = 0;
  const auto populate_field = [&](const std::vector<std::string>& names, auto& data) {
    if (data.empty()) {
      return;
    }

    const int D = sizeof(data[0]) / sizeof(float);
    for (int i = 0; i < D; i++) {
      property_offsets.emplace_back(sum_prop_offsets);
      props.emplace_back(std::make_shared<PLYPropertyBuffer<float>>(names[i], data.size()));

      float* ptr = props.back()->get<float>();
      for (size_t j = 0; j < data.size(); j++) {
        ptr[j] = reinterpret_cast<const float*>(&data[j])[i];
      }

      sum_prop_offsets += sizeof(float);
    }
  };

  populate_field({"x", "y", "z"}, ply.vertices);
  populate_field({"nx", "ny", "nz"}, ply.normals);
  populate_field({"r", "g", "b", "a"}, ply.colors);
  populate_field({"intensity"}, ply.intensities);

  for (const auto& prop : ply.properties) {
    if (std::find_if(props.begin(), props.end(), [&](const auto& p) { return p->name == prop->name; }) != props.end()) {
      // This property is already added
      continue;
    }

    // props.emplace_back(prop->clone());
    props.emplace_back(prop);
    property_offsets.emplace_back(sum_prop_offsets);
    sum_prop_offsets += property_bytes(prop->type());
  }

  const size_t num_vertices = props.size() ? props[0]->size() : 0;

  // Write header
  ofs << "ply" << std::endl;
  ofs << "format " << (binary ? "binary_little_endian" : "ascii") << " 1.0" << std::endl;

  for (const auto& comment : ply.comments) {
    ofs << "comment " << comment << std::endl;
  }
  if (std::find(ply.comments.begin(), ply.comments.end(), "generated with iridescence") == ply.comments.end()) {
    ofs << "comment generated with iridescence" << std::endl;
  }

  ofs << "element vertex " << num_vertices << std::endl;

  const std::vector<std::string> type_names = {"char", "uchar", "short", "ushort", "int", "uint", "float", "double"};
  for (const auto& prop : props) {
    ofs << "property " << type_names[static_cast<int>(prop->type())] << " " << prop->name << std::endl;
  }

  if (ply.indices.size()) {
    ofs << "element face " << ply.indices.size() / 3 << std::endl;
    ofs << "property list int32 int32 vertex_indices" << std::endl;
  }
  ofs << "end_header" << std::endl;

  for (const auto& prop : props) {
    if (prop->size() != num_vertices) {
      std::cerr << bold_red << "error: property size mismatch!! prop=" << prop->name << " size=" << prop->size() << " vs " << num_vertices << reset << std::endl;
      return false;
    }
  }

  // Write body
  if (binary) {
    std::vector<char> buffer(sum_prop_offsets * num_vertices);
    for (size_t i = 0; i < num_vertices; i++) {
      char* data_ptr = buffer.data() + i * sum_prop_offsets;
      for (size_t j = 0; j < props.size(); j++) {
        props[j]->write_to_buffer(data_ptr, property_offsets[j], i);
      }
    }

    ofs.write(buffer.data(), buffer.size());

    std::vector<PLYFace> faces(ply.indices.size() / 3);
    for (int i = 0; i < ply.indices.size() / 3; i++) {
      faces[i].num_indices = 3;
      for (int j = 0; j < 3; j++) {
        faces[i].indices[j] = ply.indices[i * 3 + 2 - j];
      }
    }

    ofs.write(reinterpret_cast<const char*>(faces.data()), sizeof(PLYFace) * faces.size());
  } else {
    for (size_t i = 0; i < num_vertices; i++) {
      for (size_t j = 0; j < props.size(); j++) {
        props[j]->write_to_stream(ofs, property_offsets[j], i);
      }
      ofs << std::endl;
    }

    for (int i = 0; i < ply.indices.size() / 3; i++) {
      ofs << 3 << " " << ply.indices[i * 3 + 2] << " " << ply.indices[i * 3 + 1] << " " << ply.indices[i * 3 + 0] << std::endl;
    }
  }

  return true;
}

bool save_ply_ascii(const std::string& filename, const PLYData& ply) {
  return save_ply(filename, ply, false);
}

bool save_ply_binary(const std::string& filename, const PLYData& ply) {
  return save_ply(filename, ply, true);
}

template <typename T, int D>
bool save_ply_binary(const std::string& filename, const Eigen::Matrix<T, D, 1>* points, int num_points) {
  PLYData ply;
  ply.vertices.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    ply.vertices[i] = points[i].template head<3>().template cast<float>();
  }

  return save_ply_binary(filename, ply);
}

template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<float, 3, 1>* points, int num_points);
template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<float, 4, 1>* points, int num_points);
template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<double, 3, 1>* points, int num_points);
template bool save_ply_binary(const std::string& filename, const Eigen::Matrix<double, 4, 1>* points, int num_points);

}  // namespace glk
