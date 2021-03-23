#include <glk/normal_distributions.hpp>

#include <glk/mesh.hpp>
#include <glk/primitives/icosahedron.hpp>

namespace glk {

NormalDistributions::NormalDistributions(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& means, const std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>>& covs, float scale) {
  glk::Icosahedron icosahedron;
  icosahedron.subdivide();
  icosahedron.subdivide();
  icosahedron.spherize();

  Eigen::Matrix<float, 4, -1> sphere_vertices = Eigen::Matrix<float, 4, -1>::Ones(4, icosahedron.vertices.size());
  sphere_vertices.topRows(3) = Eigen::Map<Eigen::Matrix<float, 3, -1>>(icosahedron.vertices[0].data(), 3, icosahedron.vertices.size());
  Eigen::ArrayXi sphere_indices = Eigen::Map<Eigen::ArrayXi>(icosahedron.indices.data(), icosahedron.indices.size());

  Eigen::Matrix<float, 4, -1> vertices(4, means.size() * icosahedron.vertices.size());
  Eigen::ArrayXi indices(means.size() * icosahedron.indices.size());

  for(int i = 0; i < means.size(); i++) {
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();
    model_matrix.block<3, 1>(0, 3) = means[i];
    model_matrix.block<3, 3>(0, 0) = scale * covs[i];

    vertices.middleCols(icosahedron.vertices.size() * i, icosahedron.vertices.size()) = model_matrix * sphere_vertices;
    indices.middleRows(icosahedron.indices.size() * i, icosahedron.indices.size()) = sphere_indices + icosahedron.vertices.size() * i;
  }

  mesh.reset(new Mesh(vertices.data(), sizeof(float) * 4, nullptr, 0, vertices.cols(), indices.data(), indices.size()));
}

NormalDistributions::NormalDistributions(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& means, const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& covs, float scale) {
  glk::Icosahedron icosahedron;
  icosahedron.subdivide();
  icosahedron.subdivide();
  icosahedron.spherize();

  Eigen::Matrix<float, 4, -1> sphere_vertices = Eigen::Matrix<float, 4, -1>::Ones(4, icosahedron.vertices.size());
  sphere_vertices.topRows(3) = Eigen::Map<Eigen::Matrix<float, 3, -1>>(icosahedron.vertices[0].data(), 3, icosahedron.vertices.size());
  Eigen::ArrayXi sphere_indices = Eigen::Map<Eigen::ArrayXi>(icosahedron.indices.data(), icosahedron.indices.size());

  Eigen::Matrix<float, 4, -1> vertices(4, means.size() * icosahedron.vertices.size());
  Eigen::ArrayXi indices(means.size() * icosahedron.indices.size());

  for(int i = 0; i < means.size(); i++) {
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();
    model_matrix.block<3, 1>(0, 3) = means[i].cast<float>();
    model_matrix.block<3, 3>(0, 0) = scale * covs[i].cast<float>();

    vertices.middleCols(icosahedron.vertices.size() * i, icosahedron.vertices.size()) = model_matrix * sphere_vertices;
    indices.middleRows(icosahedron.indices.size() * i, icosahedron.indices.size()) = sphere_indices + icosahedron.vertices.size() * i;
  }

  mesh.reset(new Mesh(vertices.data(), sizeof(float) * 4, nullptr, 0, vertices.cols(), indices.data(), indices.size()));
}

NormalDistributions::~NormalDistributions() {}

void NormalDistributions::draw(glk::GLSLShader& shader) const {
  mesh->draw(shader);
}
}  // namespace glk