#include <glk/normal_distributions.hpp>

#include <glk/mesh.hpp>
#include <glk/primitives/icosahedron.hpp>

namespace glk {

template <typename T, int D>
NormalDistributions::NormalDistributions(const Eigen::Matrix<T, D, 1>* means, const Eigen::Matrix<T, D, D>* covs, int num_points, float scale) {
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

template NormalDistributions::NormalDistributions(const Eigen::Vector3f*, const Eigen::Matrix3f*, int, float);
template NormalDistributions::NormalDistributions(const Eigen::Vector4f*, const Eigen::Matrix4f*, int, float);
template NormalDistributions::NormalDistributions(const Eigen::Vector3d*, const Eigen::Matrix3d*, int, float);
template NormalDistributions::NormalDistributions(const Eigen::Vector4d*, const Eigen::Matrix4d*, int, float);

NormalDistributions::~NormalDistributions() {}

void NormalDistributions::draw(glk::GLSLShader& shader) const {
  mesh->draw(shader);
}
}  // namespace glk