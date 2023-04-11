#include <glk/normal_distributions.hpp>

#include <glk/mesh.hpp>
#include <glk/path.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/shader_storage_buffer.hpp>
#include <glk/primitives/icosahedron.hpp>
#include <glk/async_buffer_copy.hpp>

namespace glk {

std::shared_ptr<glk::GLSLShader> NormalDistributions::vertices_shader;
std::shared_ptr<glk::GLSLShader> NormalDistributions::indices_shader;

NormalDistributions::NormalDistributions(const float* means, const float* covs, int num_points, float scale) {
  // Generate sphere mesh
  glk::Icosahedron icosahedron;
  icosahedron.subdivide();
  icosahedron.subdivide();
  icosahedron.spherize();

  if (!vertices_shader || !indices_shader) {
    // Create transformation shaders if they are not created and cached yet
    const auto data_path = glk::get_data_path();
    vertices_shader = std::make_shared<glk::GLSLShader>();
    vertices_shader->attach_source(data_path + "/shader/transform_normal_dists_vertices.comp", GL_COMPUTE_SHADER);
    vertices_shader->link_program();

    indices_shader = std::make_shared<glk::GLSLShader>();
    indices_shader->attach_source(data_path + "/shader/transform_normal_dists_indices.comp", GL_COMPUTE_SHADER);
    indices_shader->link_program();
  }

  std::vector<Eigen::Vector4f> sphere_vertices(icosahedron.vertices.size());
  std::transform(icosahedron.vertices.begin(), icosahedron.vertices.end(), sphere_vertices.begin(), [](const auto& x) { return Eigen::Vector4f(x[0], x[1], x[2], 1.0f); });

  std::vector<unsigned int> sphere_indices(icosahedron.indices.size());
  std::copy(icosahedron.indices.begin(), icosahedron.indices.end(), sphere_indices.begin());

  const int num_transformed_points = sphere_vertices.size() * num_points;
  const int num_transformed_indices = sphere_indices.size() * num_points;

  // Create buffers
  auto sphere_vertex_buffer = std::make_unique<glk::ShaderStorageBuffer>(sizeof(float) * 4 * sphere_vertices.size(), nullptr, GL_STREAM_DRAW);
  auto sphere_index_buffer = std::make_unique<glk::ShaderStorageBuffer>(sizeof(unsigned int) * sphere_indices.size(), nullptr, GL_STREAM_DRAW);
  auto means_buffer = std::make_unique<glk::ShaderStorageBuffer>(sizeof(float) * 4 * num_points, nullptr, GL_STREAM_DRAW);
  auto covs_buffer = std::make_unique<glk::ShaderStorageBuffer>(sizeof(float) * 16 * num_points, nullptr, GL_STREAM_DRAW);
  auto transformed_vertex_buffer = std::make_unique<glk::ShaderStorageBuffer>(sizeof(float) * 4 * num_transformed_points, nullptr, GL_STREAM_COPY);
  auto transformed_index_buffer = std::make_unique<glk::ShaderStorageBuffer>(sizeof(unsigned int) * num_transformed_indices, nullptr, GL_STREAM_COPY);

  // Host to device copy
  write_named_buffer_async(sphere_vertex_buffer->id(), sizeof(float) * 4 * sphere_vertices.size(), sphere_vertices.data());
  write_named_buffer_async(sphere_index_buffer->id(), sizeof(unsigned int) * sphere_indices.size(), sphere_indices.data());
  write_named_buffer_async(means_buffer->id(), sizeof(float) * 4 * num_points, means);
  write_named_buffer_async(covs_buffer->id(), sizeof(float) * 16 * num_points, covs);

  const int LOCAL_SIZE = 32;

  // Transform elipsoid vertices
  vertices_shader->use();
  vertices_shader->set_uniform("num_means", num_points);
  vertices_shader->set_uniform("num_sphere_vertices", static_cast<int>(sphere_vertices.size()));
  vertices_shader->set_uniform("scale", scale);

  sphere_vertex_buffer->bind(0);
  means_buffer->bind(1);
  covs_buffer->bind(2);
  transformed_vertex_buffer->bind(3);

  const int num_groups_vertices = (num_transformed_points / LOCAL_SIZE) + 1;
  glDispatchCompute(num_groups_vertices, 1, 1);

  vertices_shader->unuse();

  // Transform elipsoid indices
  indices_shader->use();
  indices_shader->set_uniform("num_means", num_points);
  indices_shader->set_uniform("num_sphere_vertices", static_cast<int>(sphere_vertices.size()));
  indices_shader->set_uniform("num_sphere_indices", static_cast<int>(sphere_indices.size()));

  sphere_index_buffer->bind(0);
  transformed_index_buffer->bind(1);

  const int num_groups_indices = (num_transformed_indices / LOCAL_SIZE) + 1;
  glDispatchCompute(num_groups_indices, 1, 1);

  indices_shader->unuse();

  // Create mesh buffers
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * num_transformed_points, nullptr, GL_STATIC_DRAW);
  glCopyNamedBufferSubData(transformed_vertex_buffer->id(), vbo, 0, 0, sizeof(float) * 4 * num_transformed_points);

  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_transformed_indices, nullptr, GL_STATIC_DRAW);
  glCopyNamedBufferSubData(transformed_index_buffer->id(), ebo, 0, 0, sizeof(unsigned int) * num_transformed_indices);
  num_indices = num_transformed_indices;

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

template <>
NormalDistributions::NormalDistributions(const Eigen::Matrix<float, 4, 1>* means, const Eigen::Matrix<float, 4, 4>* covs, int num_points, float scale)
: NormalDistributions(means[0].data(), covs[0].data(), num_points, scale) {}

template <>
NormalDistributions::NormalDistributions(const Eigen::Matrix<float, 3, 1>* means, const Eigen::Matrix<float, 3, 3>* covs, int num_points, float scale)
: NormalDistributions(convert_to_vector<float, 4, 1>(means, num_points), convert_to_vector<float, 4, 4>(covs, num_points), scale) {}

template <>
NormalDistributions::NormalDistributions(const Eigen::Matrix<double, 3, 1>* means, const Eigen::Matrix<double, 3, 3>* covs, int num_points, float scale)
: NormalDistributions(convert_to_vector<float, 4, 1>(means, num_points), convert_to_vector<float, 4, 4>(covs, num_points), scale) {}

template <>
NormalDistributions::NormalDistributions(const Eigen::Matrix<double, 4, 1>* means, const Eigen::Matrix<double, 4, 4>* covs, int num_points, float scale)
: NormalDistributions(convert_to_vector<float, 4, 1>(means, num_points), convert_to_vector<float, 4, 4>(covs, num_points), scale) {}

NormalDistributions::~NormalDistributions() {
  glDeleteBuffers(1, &vbo);
  glDeleteBuffers(1, &ebo);
  glDeleteVertexArrays(1, &vao);
}

void NormalDistributions::draw(glk::GLSLShader& shader) const {
  glBindVertexArray(vao);

  GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableVertexAttribArray(position_loc);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

template <typename T, int D>
NormalDistributions_old::NormalDistributions_old(const Eigen::Matrix<T, D, 1>* means, const Eigen::Matrix<T, D, D>* covs, int num_points, float scale) {
  glk::Icosahedron icosahedron;
  icosahedron.subdivide();
  icosahedron.subdivide();
  icosahedron.spherize();

  Eigen::Matrix<float, 4, -1> sphere_vertices = Eigen::Matrix<float, 4, -1>::Ones(4, icosahedron.vertices.size());
  sphere_vertices.topRows(3) = Eigen::Map<Eigen::Matrix<float, 3, -1>>(icosahedron.vertices[0].data(), 3, icosahedron.vertices.size());
  Eigen::ArrayXi sphere_indices = Eigen::Map<Eigen::ArrayXi>(icosahedron.indices.data(), icosahedron.indices.size());

  Eigen::Matrix<float, 4, -1> vertices(4, num_points * icosahedron.vertices.size());
  Eigen::ArrayXi indices(num_points * icosahedron.indices.size());

  for (int i = 0; i < num_points; i++) {
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();
    model_matrix.block<3, 1>(0, 3) = means[i].template cast<float>().template head<3>();
    model_matrix.block<3, 3>(0, 0) = scale * covs[i].template cast<float>().template block<3, 3>(0, 0);

    vertices.middleCols(icosahedron.vertices.size() * i, icosahedron.vertices.size()) = model_matrix * sphere_vertices;
    indices.middleRows(icosahedron.indices.size() * i, icosahedron.indices.size()) = sphere_indices + icosahedron.vertices.size() * i;
  }

  mesh.reset(new Mesh(vertices.data(), sizeof(float) * 4, nullptr, 0, vertices.cols(), indices.data(), indices.size()));
}

template NormalDistributions_old::NormalDistributions_old(const Eigen::Vector3f*, const Eigen::Matrix3f*, int, float);
template NormalDistributions_old::NormalDistributions_old(const Eigen::Vector4f*, const Eigen::Matrix4f*, int, float);
template NormalDistributions_old::NormalDistributions_old(const Eigen::Vector3d*, const Eigen::Matrix3d*, int, float);
template NormalDistributions_old::NormalDistributions_old(const Eigen::Vector4d*, const Eigen::Matrix4d*, int, float);

NormalDistributions_old::~NormalDistributions_old() {}

void NormalDistributions_old::draw(glk::GLSLShader& shader) const {
  mesh->draw(shader);
}

}  // namespace glk