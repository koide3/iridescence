#include <glk/splatting.hpp>

#include <glk/path.hpp>
#include <glk/texture.hpp>
#include <glk/pointcloud_buffer.hpp>

namespace glk {

std::shared_ptr<glk::GLSLShader> create_splatting_shader() {
  const auto data_path = glk::get_data_path();

  auto shader = std::make_shared<glk::GLSLShader>();
  if (
    !shader->attach_source(data_path + "/shader/splatting.vert", GL_VERTEX_SHADER) || !shader->attach_source(data_path + "/shader/splatting.geom", GL_GEOMETRY_SHADER) ||
    !shader->attach_source(data_path + "/shader/splatting.frag", GL_FRAGMENT_SHADER) || !shader->link_program()) {
    return nullptr;
  }

  shader->use();
  shader->set_uniform("colormap_sampler", 0);
  shader->set_uniform("texture_sampler", 1);
  return shader;
}

Splatting::Splatting(const std::shared_ptr<glk::GLSLShader>& shader) {
  if (shader) {
    this->shader = shader;
  } else {
    this->shader = create_splatting_shader();
  }

  vert_radius_enabled = 0;
  point_radius = 0.1f;
}

Splatting::~Splatting() {}

void Splatting::enable_vertex_radius() {
  vert_radius_enabled = 1;
}

void Splatting::disable_vertex_radius() {
  vert_radius_enabled = 0;
}

void Splatting::set_point_radius(float r) {
  this->point_radius = r;
}

void Splatting::set_texture(const std::shared_ptr<glk::Texture>& texture) {
  this->texture = texture;
}

void Splatting::set_cloud_buffer(const std::shared_ptr<glk::PointCloudBuffer>& cloud_buffer) {
  this->cloud_buffer = cloud_buffer;
}

void Splatting::draw(glk::GLSLShader& shader_) const {
  if (!cloud_buffer) {
    return;
  }

  shader->use();
  if (shader_.get_uniform_cache<int>(glk::hash("info_enabled"))) {
    shader->set_uniform("info_enabled", 1);
    shader->set_uniform("info_values", shader_.get_uniform_cache<Eigen::Vector4i>(glk::hash("info_values")));
  } else {
    shader->set_uniform("info_enabled", 0);
  }

  if (shader_.get_uniform_cache<int>(glk::hash("partial_rendering_enabled"))) {
    shader->set_uniform("partial_rendering_enabled", 1);
    shader->set_uniform("dynamic_object", shader_.get_uniform_cache<int>(glk::hash("dynamic_object")));
  } else {
    shader->set_uniform("partial_rendering_enabled", 0);
  }

  shader->set_uniform("normal_enabled", shader_.get_uniform_cache<int>(glk::hash("normal_enabled")));
  shader->set_uniform("texture_enabled", texture ? 1 : 0);

  shader->set_uniform("vert_radius_enabled", vert_radius_enabled);
  shader->set_uniform("point_radius", point_radius);

  shader->set_uniform("color_mode", shader_.get_uniform_cache<int>(glk::hash("color_mode")));
  shader->set_uniform("z_range", shader_.get_uniform_cache<Eigen::Vector2f>(glk::hash("z_range")));
  shader->set_uniform("cmap_range", shader_.get_uniform_cache<Eigen::Vector2f>(glk::hash("cmap_range")));
  shader->set_uniform("colormap_axis", shader_.get_uniform_cache<Eigen::Vector3f>(glk::hash("colormap_axis")));

  shader->set_uniform("model_matrix", shader_.get_uniform_cache<Eigen::Matrix4f>(glk::hash("model_matrix")));
  shader->set_uniform("view_matrix", shader_.get_uniform_cache<Eigen::Matrix4f>(glk::hash("view_matrix")));
  shader->set_uniform("projection_matrix", shader_.get_uniform_cache<Eigen::Matrix4f>(glk::hash("projection_matrix")));
  shader->set_uniform("material_color", shader_.get_uniform_cache<Eigen::Vector4f>(glk::hash("material_color")));

  const bool cull_was_enabled = glIsEnabled(GL_CULL_FACE);
  glDisable(GL_CULL_FACE);

  if (texture) {
    texture->bind(GL_TEXTURE1);
  }

  cloud_buffer->draw(*shader);

  if (texture) {
    texture->unbind(GL_TEXTURE1);
  }

  shader_.use();

  if (cull_was_enabled) {
    glEnable(GL_CULL_FACE);
  }
}

}  // namespace glk
