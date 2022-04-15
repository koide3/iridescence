#include <glk/normal_distributions.hpp>

#include <glk/mesh.hpp>
#include <glk/primitives/icosahedron.hpp>

namespace glk {

NormalDistributions::NormalDistributions(const Eigen::Vector3f* means, const Eigen::Matrix3f* covs, int num_points, float scale) {
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
    model_matrix.block<3, 1>(0, 3) = means[i];
    model_matrix.block<3, 3>(0, 0) = scale * covs[i];

    vertices.middleCols(icosahedron.vertices.size() * i, icosahedron.vertices.size()) = model_matrix * sphere_vertices;
    indices.middleRows(icosahedron.indices.size() * i, icosahedron.indices.size()) = sphere_indices + icosahedron.vertices.size() * i;
  }

  mesh.reset(new Mesh(vertices.data(), sizeof(float) * 4, nullptr, 0, vertices.cols(), indices.data(), indices.size()));
}

NormalDistributions::NormalDistributions(const Eigen::Matrix<float, 3, -1>& means, const Eigen::Matrix<float, 9, -1>& covs, float scale)
: NormalDistributions(reinterpret_cast<const Eigen::Vector3f*>(means.data()), reinterpret_cast<const Eigen::Matrix3f*>(covs.data()), means.cols(), scale) {}

NormalDistributions::NormalDistributions(const Eigen::Matrix<double, 3, -1>& means, const Eigen::Matrix<double, 9, -1>& covs, float scale)
: NormalDistributions(means.cast<float>().eval(), covs.cast<float>().eval(), scale) {}

NormalDistributions::~NormalDistributions() {}

void NormalDistributions::draw(glk::GLSLShader& shader) const {
  mesh->draw(shader);
}
}  // namespace glk