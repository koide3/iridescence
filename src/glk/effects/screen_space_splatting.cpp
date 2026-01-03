#include <glk/effects/screen_space_splatting.hpp>

#include <fstream>

#include <glk/path.hpp>
#include <glk/query.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/console_colors.hpp>

#include <glk/profiler.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glk {

using namespace glk::console;

ScreenSpaceSplatting::ScreenSpaceSplatting(const Eigen::Vector2i& size) {
  k_neighbors = 15;
  k_tolerance = 2;
  initial_estimation_grid_size = 32;
  num_iterations = 6;

  query.reset(new glk::Query());

  if (!texture_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/texture.frag")) {
    abort();
  }

  if (!position_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/splat/calc_pos.frag")) {
    abort();
  }

  if (!white_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/one_float.frag")) {
    abort();
  }

  if (!increment_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/one_float.frag")) {
    abort();
  }

  if (
    !point_extraction_shader.attach_source(get_data_path() + "/shader/splat/extract_points.vert", GL_VERTEX_SHADER) ||
    !point_extraction_shader.attach_source(get_data_path() + "/shader/splat/extract_points.geom", GL_GEOMETRY_SHADER) ||
    !point_extraction_shader.attach_source(get_data_path() + "/shader/splat/extract_points.frag", GL_FRAGMENT_SHADER) ||
    !point_extraction_shader.add_feedback_varying("vert_out") || !point_extraction_shader.link_program()) {
    abort();
  }

  if (!initial_radius_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/initial_radius.frag")) {
    abort();
  }

  if (!distribution_shader.init(get_data_path() + "/shader/splat/distribution")) {
    abort();
  }

  if (!gathering_shader.init(get_data_path() + "/shader/splat/gathering")) {
    abort();
  }

  if (!bounds_update_shader.init(get_data_path() + "/shader/splat/tex2screen_test_finalized.vert", get_data_path() + "/shader/splat/update_bounds.frag")) {
    abort();
  }

  if (!farthest_point_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/farthest_point.frag")) {
    abort();
  }

  if (!radius_finalization_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/finalize_radius.frag")) {
    abort();
  }

  if (!gaussian_gathering_shader.init(get_data_path() + "/shader/splat/gathering.vert", get_data_path() + "/shader/splat/gaussian_gathering.frag")) {
    abort();
  }

  if (
    !gaussian_finalization_shader.attach_source(get_data_path() + "/shader/splat/tex2screen.vert", GL_VERTEX_SHADER) ||
    !gaussian_finalization_shader.attach_source(get_data_path() + "/shader/splat/gaussian_finalization.frag", GL_FRAGMENT_SHADER) ||
    !gaussian_finalization_shader.attach_source(get_data_path() + "/shader/splat/eigen.frag", GL_FRAGMENT_SHADER) || !gaussian_finalization_shader.link_program()) {
    abort();
  }

  if (
    !splatting_first_shader.attach_source(get_data_path() + "/shader/splat/splat.vert", GL_VERTEX_SHADER) ||
    !splatting_first_shader.attach_source(get_data_path() + "/shader/splat/splat.geom", GL_GEOMETRY_SHADER) ||
    !splatting_first_shader.attach_source(get_data_path() + "/shader/splat/splat_first.frag", GL_FRAGMENT_SHADER) || !splatting_first_shader.link_program()) {
    abort();
  }

  if (
    !splatting_second_shader.attach_source(get_data_path() + "/shader/splat/splat.vert", GL_VERTEX_SHADER) ||
    !splatting_second_shader.attach_source(get_data_path() + "/shader/splat/splat.geom", GL_GEOMETRY_SHADER) ||
    !splatting_second_shader.attach_source(get_data_path() + "/shader/splat/splat_second.frag", GL_FRAGMENT_SHADER) || !splatting_second_shader.link_program()) {
    abort();
  }

  if (!splatting_finalization_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/splat/finalize_splat.frag")) {
    abort();
  }

  if (
    !debug_shader.attach_source(get_data_path() + "/shader/texture.vert", GL_VERTEX_SHADER) ||
    !debug_shader.attach_source(get_data_path() + "/shader/splat/debug.frag", GL_FRAGMENT_SHADER) || !debug_shader.link_program()) {
    abort();
  }

  point_extraction_shader.use();
  point_extraction_shader.set_uniform("depth_sampler", 0);

  initial_radius_shader.use();
  initial_radius_shader.set_uniform("depth_sampler", 0);
  initial_radius_shader.set_uniform("num_points_grid_sampler", 1);

  initial_radius_shader.set_uniform("grid_area", initial_estimation_grid_size * initial_estimation_grid_size);
  initial_radius_shader.set_uniform("k_neighbors", k_neighbors);

  distribution_shader.use();
  distribution_shader.set_uniform("position_sampler", 0);
  distribution_shader.set_uniform("radius_sampler", 1);
  distribution_shader.set_uniform("finalized_radius_sampler", 2);

  gathering_shader.use();
  gathering_shader.set_uniform("position_sampler", 0);
  gathering_shader.set_uniform("radius_bounds_sampler", 1);
  gathering_shader.set_uniform("feedback_radius_sampler", 2);

  bounds_update_shader.use();
  bounds_update_shader.set_uniform("neighbor_counts_sampler", 0);
  bounds_update_shader.set_uniform("radius_bounds_sampler", 1);
  bounds_update_shader.set_uniform("finalized_radius_sampler", 2);
  bounds_update_shader.set_uniform("k_neighbors", k_neighbors);

  radius_finalization_shader.use();
  radius_finalization_shader.set_uniform("neighbor_counts_sampler", 0);
  radius_finalization_shader.set_uniform("radius_bounds_sampler", 1);
  radius_finalization_shader.set_uniform("k_neighbors", k_neighbors);

  gaussian_gathering_shader.use();
  gaussian_gathering_shader.set_uniform("position_sampler", 0);
  gaussian_gathering_shader.set_uniform("radius_bounds_sampler", 1);
  gaussian_gathering_shader.set_uniform("feedback_radius_sampler", 2);

  gaussian_finalization_shader.use();
  gaussian_finalization_shader.set_uniform("radius_bounds_buffer", 0);
  gaussian_finalization_shader.set_uniform("num_points_sampler", 1);
  gaussian_finalization_shader.set_uniform("sum_points_sampler", 2);
  gaussian_finalization_shader.set_uniform("sum_cross1_sampler", 3);
  gaussian_finalization_shader.set_uniform("sum_cross2_sampler", 4);
  gaussian_finalization_shader.set_uniform("k_neighbors", k_neighbors);
  gaussian_finalization_shader.set_uniform("k_tolerance", k_tolerance);

  splatting_first_shader.use();
  splatting_first_shader.set_uniform("position_sampler", 0);
  splatting_first_shader.set_uniform("radius_sampler", 1);
  splatting_first_shader.set_uniform("normal_sampler", 2);
  splatting_first_shader.set_uniform("minor_tangent_sampler", 3);
  splatting_first_shader.set_uniform("major_tangent_sampler", 4);
  splatting_first_shader.set_uniform("color_sampler", 5);

  splatting_second_shader.use();
  splatting_second_shader.set_uniform("position_sampler", 0);
  splatting_second_shader.set_uniform("radius_sampler", 1);
  splatting_second_shader.set_uniform("normal_sampler", 2);
  splatting_second_shader.set_uniform("minor_tangent_sampler", 3);
  splatting_second_shader.set_uniform("major_tangent_sampler", 4);
  splatting_second_shader.set_uniform("color_sampler", 5);

  splatting_finalization_shader.use();
  splatting_finalization_shader.set_uniform("weight_sampler", 0);
  splatting_finalization_shader.set_uniform("position_sampler", 1);
  splatting_finalization_shader.set_uniform("normal_sampler", 2);
  splatting_finalization_shader.set_uniform("color_sampler", 3);

  debug_shader.use();
  debug_shader.set_uniform("sampler0", 0);
  debug_shader.set_uniform("sampler1", 1);
  debug_shader.set_uniform("sampler2", 2);
  debug_shader.set_uniform("sampler3", 3);

  glUseProgram(0);

  set_size(size);
}

ScreenSpaceSplatting::~ScreenSpaceSplatting() {}

void ScreenSpaceSplatting::set_size(const Eigen::Vector2i& size) {
  const int steps = 4;
  const int subsample_steps = 1;
  const Eigen::Array3d step_size = Eigen::Array3d(1.0 / size[0], 1.0 / size[1], 0.0);

  point_extraction_shader.use();
  point_extraction_shader.set_uniform("step_size", Eigen::Vector2f(step_size[0], step_size[1]));
  point_extraction_shader.unuse();

  position_buffer.reset(new glk::FrameBuffer(size, 0, true));
  position_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);

  std::vector<Eigen::Vector3f> sampling_points;
  for (int y = 0; y < size[1]; y += steps * subsample_steps) {
    for (int x = 0; x < size[0]; x += steps * subsample_steps) {
      Eigen::Array3d pt = step_size * Eigen::Array3d(x, y, 0) + step_size * 0.5;
      sampling_points.push_back(pt.cast<float>());
    }
  }
  sampling_points_buffer.reset(new glk::PointCloudBuffer(sampling_points));

  // It seems the speed of glDrawTransformFeedback largely depends on the buffer size
  // and is almost independent of the number of generated primitives (take it with a grain of solt).
  // Let us assume most of pixels have invalid data and ignore them
  // Note: I found that GL_STATIC_DRAW significantly improves the drawing speed
  int max_num_points = sampling_points.size() * steps * steps;
  int buffer_size = sizeof(Eigen::Vector3f) * max_num_points / 5;
  points_on_screen.reset(new glk::TransformFeedback(buffer_size));

  // finalized radius buffer
  finalized_radius_buffer.reset(new glk::FrameBuffer(size, 0, false));
  finalized_radius_buffer->add_color_buffer(0, GL_RG32F, GL_RG, GL_FLOAT).set_filer_mode(GL_NEAREST);
  finalized_radius_buffer->add_depth_buffer(GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT);

  // Low resolution buffer for initial radius estimation
  const Eigen::Vector2i initial_estimation_buffer_size = (size.cast<double>() / initial_estimation_grid_size).array().ceil().cast<int>();
  initial_estimation_buffer.reset(new glk::FrameBuffer(initial_estimation_buffer_size, 0, false));
  // Because alpha blending doesn't work with integer textures, we use float texture here
  // (Should we use a stencil buffer instead?)
  initial_estimation_buffer->add_color_buffer(0, GL_R32F, GL_RED, GL_FLOAT).set_filer_mode(GL_NEAREST);

  // knn estimation
  // radius buffer
  radius_buffer_ping.reset(new glk::FrameBuffer(size, 0, false));
  radius_buffer_ping->add_color_buffer(0, GL_RG32F, GL_RG, GL_FLOAT).set_filer_mode(GL_NEAREST);
  radius_buffer_ping->bind_ext_depth_buffer(position_buffer->depth());

  radius_buffer_pong.reset(new glk::FrameBuffer(size, 0, false));
  radius_buffer_pong->add_color_buffer(0, GL_RG32F, GL_RG, GL_FLOAT).set_filer_mode(GL_NEAREST);
  radius_buffer_pong->bind_ext_depth_buffer(position_buffer->depth());

  // neighbor counts buffer
  neighbor_counts_buffer.reset(new glk::FrameBuffer(size, 0, false));
  neighbor_counts_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGBA, GL_FLOAT).set_filer_mode(GL_NEAREST);
  neighbor_counts_buffer->bind_ext_depth_buffer(finalized_radius_buffer->depth());

  // feedback radius
  feedback_radius_buffer.reset(new glk::FrameBuffer(size, 0, false));
  feedback_radius_buffer->add_color_buffer(0, GL_R32F, GL_RED, GL_FLOAT).set_filer_mode(GL_NEAREST);  // feedback radius
  feedback_radius_buffer->bind_ext_depth_buffer(position_buffer->depth());

  // gaussian estimation
  // accumulation buffer
  gaussian_accum_buffer.reset(new glk::FrameBuffer(size, 0, false));
  gaussian_accum_buffer->add_color_buffer(0, GL_R32F, GL_RED, GL_FLOAT).set_filer_mode(GL_NEAREST);    //
  gaussian_accum_buffer->add_color_buffer(1, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // mean
  gaussian_accum_buffer->add_color_buffer(2, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // cov1
  gaussian_accum_buffer->add_color_buffer(3, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // cov2
  gaussian_accum_buffer->bind_ext_depth_buffer(position_buffer->depth());

  // gaussian buffer
  gaussian_dists_buffer.reset(new glk::FrameBuffer(size, 0, false));
  gaussian_dists_buffer->add_color_buffer(0, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // length
  gaussian_dists_buffer->add_color_buffer(1, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // normal
  gaussian_dists_buffer->add_color_buffer(2, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // minor tangent
  gaussian_dists_buffer->add_color_buffer(3, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // major tangent
  gaussian_dists_buffer->bind_ext_depth_buffer(position_buffer->depth());

  // splatting buffer
  splatting_buffer.reset(new glk::FrameBuffer(size, 0, true));
  splatting_buffer->add_color_buffer(0, GL_R32F, GL_RED, GL_FLOAT).set_filer_mode(GL_NEAREST);    // weight
  splatting_buffer->add_color_buffer(1, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // position
  splatting_buffer->add_color_buffer(2, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // normal
  splatting_buffer->add_color_buffer(3, GL_RGB32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);  // color

  // result buffer
  result_buffer.reset(new glk::FrameBuffer(size, 0, false));
  result_buffer->add_color_buffer(0, GL_RGB32F, GL_RGBA, GL_FLOAT);  // color
  result_buffer->add_color_buffer(1, GL_RGB32F, GL_RGB, GL_FLOAT);   // position
  result_buffer->add_color_buffer(2, GL_RGB32F, GL_RGB, GL_FLOAT);   // normal
}

const glk::Texture& ScreenSpaceSplatting::position() const {
  return result_buffer->color(0);
}

const glk::Texture& ScreenSpaceSplatting::normal() const {
  return result_buffer->color(1);
}

const glk::Texture& ScreenSpaceSplatting::color() const {
  return result_buffer->color(2);
}

/**
 * @brief Extract valid pixels from sampling_buffer and store their uv coordinates and 3D positions
 *        in #position_buffer and #points_on_screen
 * @param [in]depth_texture, sampling_buffer
 * @param [out]position_buffer, points_on_screen
 */
void ScreenSpaceSplatting::extract_points_on_screen(const glk::Texture& depth_texture) {
  const GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_ALWAYS);

  position_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);
  glClearBufferfv(GL_DEPTH, 0, black);

  point_extraction_shader.use();
  depth_texture.bind();

  // transform feedback
  points_on_screen->bind();
  glBeginTransformFeedback(GL_POINTS);
  sampling_points_buffer->draw(point_extraction_shader);
  glEndTransformFeedback();
  points_on_screen->unbind();

  depth_texture.unbind();
  point_extraction_shader.unuse();

  position_buffer->unbind();

  glDepthFunc(GL_LESS);
  glDisable(GL_DEPTH_TEST);
}

/**
 * @brief Estimate initial knn radius bounds
 * @param [in]depth_texture, points_on_screen
 * @param [out]radius_buffer_ping
 */
void ScreenSpaceSplatting::estimate_initial_radius(const glk::Texture& depth_texture) {
  const GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};

  // count the number of vertices in a low resolution grid
  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_ONE, GL_ONE);

  initial_estimation_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);

  white_shader.use();
  points_on_screen->draw(white_shader);
  white_shader.unuse();

  initial_estimation_buffer->unbind();

  glDisable(GL_BLEND);

  // radius upper bound calculation
  radius_buffer_ping->bind();
  glClearBufferfv(GL_COLOR, 0, black);  // radius

  initial_radius_shader.use();

  depth_texture.bind(GL_TEXTURE0);                       // depth
  initial_estimation_buffer->color().bind(GL_TEXTURE1);  // num points grid

  points_on_screen->draw(initial_radius_shader);

  depth_texture.unbind(GL_TEXTURE0);
  initial_estimation_buffer->color().unbind(GL_TEXTURE1);

  initial_radius_shader.unuse();

  radius_buffer_ping->unbind();
}

/**
 * @brief Estimate knn radius
 * @param [in]points_on_screen, radius_buffer_ping
 * @param [out]finalized_radius_buffer
 */
void ScreenSpaceSplatting::estimate_knn_radius() {
  const GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};

  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE);

  // initialize finalized radius buffer
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_ALWAYS);

  finalized_radius_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);
  glClearBufferfv(GL_DEPTH, 0, black);

  farthest_point_shader.use();
  points_on_screen->draw(farthest_point_shader);
  farthest_point_shader.unuse();

  finalized_radius_buffer->unbind();

  glDepthFunc(GL_LESS);
  glDepthMask(GL_FALSE);

  // ping pong
  for (int i = 0; i < num_iterations; i++) {
    auto& radius_buffer_front = i % 2 == 0 ? radius_buffer_ping : radius_buffer_pong;
    auto& radius_buffer_back = i % 2 == 0 ? radius_buffer_pong : radius_buffer_ping;

    // distribution
    feedback_radius_buffer->bind();
    glClearBufferfv(GL_COLOR, 0, black);

    glBlendEquation(GL_MAX);
    distribution_shader.use();
    position_buffer->color().bind(GL_TEXTURE0);      // position
    radius_buffer_front->color().bind(GL_TEXTURE1);  // radius
    finalized_radius_buffer->color().bind(GL_TEXTURE2);

    points_on_screen->draw(distribution_shader);

    feedback_radius_buffer->unbind();

    // gathering
    neighbor_counts_buffer->bind();
    glClearBufferfv(GL_COLOR, 0, black);

    glBlendEquation(GL_FUNC_ADD);
    gathering_shader.use();
    feedback_radius_buffer->color().bind(GL_TEXTURE2);
    finalized_radius_buffer->color().bind(GL_TEXTURE3);

    points_on_screen->draw(gathering_shader);

    position_buffer->color().unbind(GL_TEXTURE0);
    radius_buffer_front->color().unbind(GL_TEXTURE1);
    feedback_radius_buffer->color().unbind(GL_TEXTURE2);
    finalized_radius_buffer->color().unbind(GL_TEXTURE3);
    gathering_shader.unuse();

    neighbor_counts_buffer->unbind();

    // update radius bounds
    radius_buffer_back->bind();
    glClearBufferfv(GL_COLOR, 0, black);

    glDisable(GL_BLEND);
    bounds_update_shader.use();
    neighbor_counts_buffer->color().bind(GL_TEXTURE0);
    radius_buffer_front->color().bind(GL_TEXTURE1);
    finalized_radius_buffer->color().bind(GL_TEXTURE2);

    points_on_screen->draw(bounds_update_shader);

    finalized_radius_buffer->color().unbind(GL_TEXTURE2);
    radius_buffer_front->color().unbind(GL_TEXTURE1);
    neighbor_counts_buffer->color().unbind(GL_TEXTURE0);
    bounds_update_shader.unuse();

    radius_buffer_back->unbind();

    // finalize
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS);

    finalized_radius_buffer->bind();

    radius_finalization_shader.use();
    neighbor_counts_buffer->color().bind(GL_TEXTURE0);
    radius_buffer_front->color().bind(GL_TEXTURE1);

    points_on_screen->draw(radius_finalization_shader);

    radius_buffer_front->color().unbind(GL_TEXTURE1);
    neighbor_counts_buffer->color().unbind(GL_TEXTURE0);
    radius_finalization_shader.unuse();

    finalized_radius_buffer->unbind();

    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
  }

  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);
  glDisable(GL_DEPTH_TEST);

  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glDisable(GL_BLEND);
}

/**
 * @brief Estimate Gaussian distributions
 */
void ScreenSpaceSplatting::estimate_gaussian(GLProfiler& prof, const TextureRenderer& renderer) {
  const GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};

  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glDepthMask(GL_FALSE);

  // distribution
  prof.add("gauss_dist");
  feedback_radius_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);

  glBlendEquation(GL_MAX);
  distribution_shader.use();

  position_buffer->color().bind(GL_TEXTURE0);
  finalized_radius_buffer->color().bind(GL_TEXTURE1);
  finalized_radius_buffer->color().unbind(GL_TEXTURE2);

  points_on_screen->draw(distribution_shader);

  distribution_shader.unuse();

  feedback_radius_buffer->unbind();

  // gathering
  prof.add("gauss_gather");
  gaussian_accum_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);
  glClearBufferfv(GL_COLOR, 1, black);
  glClearBufferfv(GL_COLOR, 2, black);
  glClearBufferfv(GL_COLOR, 3, black);

  // glDisable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  gaussian_gathering_shader.use();

  position_buffer->color().bind(GL_TEXTURE0);
  finalized_radius_buffer->color().bind(GL_TEXTURE1);
  feedback_radius_buffer->color().bind(GL_TEXTURE2);

  points_on_screen->draw(gaussian_gathering_shader);

  position_buffer->color().unbind(GL_TEXTURE0);
  finalized_radius_buffer->color().unbind(GL_TEXTURE1);
  feedback_radius_buffer->color().unbind(GL_TEXTURE2);

  gaussian_gathering_shader.unuse();
  gaussian_accum_buffer->unbind();

  // finalize
  prof.add("gauss_finalize");
  gaussian_dists_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);
  glClearBufferfv(GL_COLOR, 1, black);
  glClearBufferfv(GL_COLOR, 2, black);
  glClearBufferfv(GL_COLOR, 3, black);

  glDisable(GL_BLEND);
  gaussian_finalization_shader.use();

  finalized_radius_buffer->color().bind(GL_TEXTURE0);
  gaussian_accum_buffer->color(0).bind(GL_TEXTURE1);
  gaussian_accum_buffer->color(1).bind(GL_TEXTURE2);
  gaussian_accum_buffer->color(2).bind(GL_TEXTURE3);
  gaussian_accum_buffer->color(3).bind(GL_TEXTURE4);

  points_on_screen->draw(gaussian_finalization_shader);

  finalized_radius_buffer->color().unbind(GL_TEXTURE0);
  gaussian_accum_buffer->color(0).unbind(GL_TEXTURE1);
  gaussian_accum_buffer->color(1).unbind(GL_TEXTURE2);
  gaussian_accum_buffer->color(2).unbind(GL_TEXTURE3);
  gaussian_accum_buffer->color(3).unbind(GL_TEXTURE4);

  gaussian_finalization_shader.unuse();

  gaussian_dists_buffer->unbind();

  glDepthMask(GL_TRUE);
  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glDisable(GL_DEPTH_TEST);
}

/**
 * Splatting
 */
void ScreenSpaceSplatting::render_splatting(GLProfiler& prof, const glk::Texture& color_texture) {
  const GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};
  const GLfloat white[] = {1.0f, 1.0f, 1.0f, 1.0f};

  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  splatting_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);
  glClearBufferfv(GL_COLOR, 1, black);
  glClearBufferfv(GL_COLOR, 2, black);
  glClearBufferfv(GL_COLOR, 3, black);
  glClearBufferfv(GL_DEPTH, 0, white);

  position_buffer->color().bind(GL_TEXTURE0);
  gaussian_dists_buffer->color(0).bind(GL_TEXTURE1);  // radii (normal, minor, major)
  gaussian_dists_buffer->color(1).bind(GL_TEXTURE2);  // normal
  gaussian_dists_buffer->color(2).bind(GL_TEXTURE3);  // minor_tangent
  gaussian_dists_buffer->color(3).bind(GL_TEXTURE4);  // major_tangent
  color_texture.bind(GL_TEXTURE5);

  // visibility pass
  splatting_first_shader.use();
  points_on_screen->draw(splatting_first_shader);

  // attribute pass
  glEnable(GL_BLEND);
  glDepthMask(GL_FALSE);
  glBlendEquation(GL_FUNC_ADD);
  splatting_second_shader.use();

  points_on_screen->draw(splatting_first_shader);

  splatting_second_shader.unuse();

  position_buffer->color().unbind(GL_TEXTURE0);
  gaussian_dists_buffer->color(0).unbind(GL_TEXTURE1);  // radii
  gaussian_dists_buffer->color(1).unbind(GL_TEXTURE2);  // normal
  gaussian_dists_buffer->color(2).unbind(GL_TEXTURE3);  // minor_tangent
  gaussian_dists_buffer->color(3).unbind(GL_TEXTURE4);  // major_tangent
  color_texture.unbind(GL_TEXTURE5);

  splatting_buffer->unbind();

  glDepthMask(GL_TRUE);
  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
}

void ScreenSpaceSplatting::draw(
  const TextureRenderer& renderer,
  const glk::Texture& color_texture,
  const glk::Texture& depth_texture,
  const TextureRendererInput::Ptr& input,
  glk::FrameBuffer* frame_buffer) {
  glDisable(GL_DEPTH_TEST);

  color_texture.set_filer_mode(GL_NEAREST);
  depth_texture.set_filer_mode(GL_NEAREST);

  // uniform variables
  const auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
  const auto projection_matrix = input->get<Eigen::Matrix4f>("projection_matrix");
  if (!view_matrix || !projection_matrix) {
    std::cerr << bold_red << "error: view and projection matrices must be set" << reset << std::endl;
    return;
  }

  const Eigen::Vector2f screen_size = position_buffer->size().cast<float>();
  const Eigen::Vector2f inv_screen_size(1.0 / position_buffer->size()[0], 1.0 / position_buffer->size()[1]);

  const Eigen::Vector3f view_point = view_matrix->inverse().block<3, 1>(0, 3);
  const Eigen::Matrix4f inv_projection_matrix = (*projection_matrix).inverse();
  const Eigen::Matrix4f projection_view_matrix = (*projection_matrix) * (*view_matrix);
  const Eigen::Matrix4f inv_projection_view_matrix = projection_view_matrix.inverse();

  // set up uniforms
  point_extraction_shader.use();
  point_extraction_shader.set_uniform("inv_projection_view_matrix", inv_projection_view_matrix);

  initial_radius_shader.use();
  initial_radius_shader.set_uniform("inv_screen_size", inv_screen_size);
  initial_radius_shader.set_uniform("inv_projection_matrix", inv_projection_matrix);

  distribution_shader.use();
  distribution_shader.set_uniform("screen_size", screen_size);
  distribution_shader.set_uniform("inv_screen_size", inv_screen_size);
  distribution_shader.set_uniform("view_matrix", *view_matrix);
  distribution_shader.set_uniform("projection_matrix", *projection_matrix);

  gathering_shader.use();
  gathering_shader.set_uniform("screen_size", screen_size);
  gathering_shader.set_uniform("inv_screen_size", inv_screen_size);
  gathering_shader.set_uniform("view_matrix", *view_matrix);
  gathering_shader.set_uniform("projection_matrix", *projection_matrix);

  gaussian_gathering_shader.use();
  gaussian_gathering_shader.set_uniform("screen_size", screen_size);
  gaussian_gathering_shader.set_uniform("inv_screen_size", inv_screen_size);
  gaussian_gathering_shader.set_uniform("view_matrix", *view_matrix);
  gaussian_gathering_shader.set_uniform("projection_matrix", *projection_matrix);

  gaussian_finalization_shader.use();
  gaussian_finalization_shader.set_uniform("view_point", view_point);

  splatting_first_shader.use();
  splatting_first_shader.set_uniform("pass_stage", 0);
  splatting_first_shader.set_uniform("view_matrix", *view_matrix);
  splatting_first_shader.set_uniform("projection_matrix", *projection_matrix);
  splatting_first_shader.set_uniform("projection_view_matrix", projection_view_matrix);

  splatting_second_shader.use();
  splatting_second_shader.set_uniform("pass_stage", 1);
  splatting_second_shader.set_uniform("view_matrix", *view_matrix);
  splatting_second_shader.set_uniform("projection_matrix", *projection_matrix);
  splatting_second_shader.set_uniform("projection_view_matrix", projection_view_matrix);

  glk::GLProfiler prof("splat", false);

  // extract valid points with transform feedback and calc vertex positions
  prof.add("extract_points");
  extract_points_on_screen(depth_texture);

  // initial radius estimation
  prof.add("initial_guess");
  estimate_initial_radius(depth_texture);

  // radius ping pong
  prof.add("estimate knn");
  estimate_knn_radius();

  // estimate gaussian
  prof.add("estimate gaussian");
  estimate_gaussian(prof, renderer);

  prof.add("splatting");
  render_splatting(prof, color_texture);

  prof.add("finalization");
  result_buffer->bind();
  glDisable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT);

  splatting_finalization_shader.use();

  splatting_buffer->color(0).bind(GL_TEXTURE0);
  splatting_buffer->color(1).bind(GL_TEXTURE1);
  splatting_buffer->color(2).bind(GL_TEXTURE2);
  splatting_buffer->color(3).bind(GL_TEXTURE3);

  renderer.draw_plain(splatting_finalization_shader);

  splatting_buffer->color(0).unbind(GL_TEXTURE0);
  splatting_buffer->color(1).unbind(GL_TEXTURE1);
  splatting_buffer->color(2).unbind(GL_TEXTURE2);
  splatting_buffer->color(3).unbind(GL_TEXTURE3);

  splatting_finalization_shader.unuse();

  result_buffer->unbind();

  prof.add("done");

  // render to screen
  /*
  if(frame_buffer) {
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);

    frame_buffer->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    debug_shader.use();
    result_buffer->color(0).bind(GL_TEXTURE0);
    result_buffer->color(1).bind(GL_TEXTURE1);
    result_buffer->color(2).bind(GL_TEXTURE2);

    renderer.draw_plain(debug_shader);

    result_buffer->color(0).unbind(GL_TEXTURE0);
    result_buffer->color(1).unbind(GL_TEXTURE1);
    result_buffer->color(2).unbind(GL_TEXTURE2);
    debug_shader.unuse();

    frame_buffer->unbind();
  }
  */
}
}  // namespace glk