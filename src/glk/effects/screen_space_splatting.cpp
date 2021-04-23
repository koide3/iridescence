#include <glk/effects/screen_space_splatting.hpp>

#include <fstream>

#include <glk/path.hpp>
#include <glk/query.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/pointcloud_buffer.hpp>

#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glk {

ScreenSpaceSplatting::ScreenSpaceSplatting(const Eigen::Vector2i& size) {
  k_neighbors = 10;
  initial_estimation_grid_size = 32;

  if(!texture_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/texture.frag")) {
    return;
  }

  if(!position_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_pos.frag")) {
    return;
  }

  if(!white_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/one_float.frag")) {
    return;
  }

  if(!increment_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/one_float.frag")) {
    return;
  }

  if(!point_extraction_shader.attach_source(get_data_path() + "/shader/splat/extract_points.geom", GL_GEOMETRY_SHADER) ||
     !point_extraction_shader.attach_source(get_data_path() + "/shader/splat/extract_points.vert", GL_VERTEX_SHADER) ||
     !point_extraction_shader.add_feedback_varying("vert_out") || !point_extraction_shader.link_program()) {
    return;
  }

  if(!initial_radius_shader.init(get_data_path() + "/shader/splat/tex2screen.vert", get_data_path() + "/shader/splat/initial_radius.frag")) {
    abort();
    return;
  }

  if(!distribution_shader.init(get_data_path() + "/shader/splat/distribution")) {
    abort();
    return;
  }

  if(!gathering_shader.init(get_data_path() + "/shader/splat/gathering")) {
    abort();
    return;
  }

  point_extraction_shader.use();
  point_extraction_shader.set_uniform("depth_sampler", 0);

  initial_radius_shader.use();
  initial_radius_shader.set_uniform("num_points_grid_sampler", 0);

  initial_radius_shader.set_uniform("grid_area", initial_estimation_grid_size * initial_estimation_grid_size);
  initial_radius_shader.set_uniform("k_neighbors", k_neighbors);

  distribution_shader.use();
  distribution_shader.set_uniform("position_sampler", 0);
  distribution_shader.set_uniform("radius_sampler", 1);

  gathering_shader.use();
  gathering_shader.set_uniform("position_sampler", 0);
  gathering_shader.set_uniform("feedback_radius_sampler", 1);
  gathering_shader.set_uniform("radius_bounds_sampler", 2);

  query.reset(new glk::Query());

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

  position_buffer.reset(new glk::FrameBuffer(size, 0, false));
  position_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT).set_filer_mode(GL_NEAREST);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> sampling_points;
  for(int y = 0; y < size[1]; y += steps * subsample_steps) {
    for(int x = 0; x < size[0]; x += steps * subsample_steps) {
      Eigen::Array3d pt = step_size * Eigen::Array3d(x, y, 0);
      sampling_points.push_back(pt.cast<float>());
    }
  }
  sampling_points_buffer.reset(new glk::PointCloudBuffer(sampling_points));

  // It seems the speed of glDrawTransformFeedback largely depends on the buffer size
  // and is almost independent of the number of generated primitives (take it with a grain of solt).
  // Let us assume most of pixels have invalid data and ignore them
  int max_num_points = sampling_points.size() * steps * steps;
  int buffer_size = sizeof(Eigen::Vector3f) * max_num_points / 10;
  points_on_screen.reset(new glk::TransformFeedback(buffer_size));

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
  radius_buffer_pong.reset(new glk::FrameBuffer(size, 0, false));
  radius_buffer_pong->add_color_buffer(0, GL_RG32F, GL_RG, GL_FLOAT).set_filer_mode(GL_NEAREST);

  // neighbor counts buffer
  neighbor_counts_buffer.reset(new glk::FrameBuffer(size, 0, false));
  neighbor_counts_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGBA, GL_FLOAT).set_filer_mode(GL_NEAREST);

  // feedback radius
  feedback_radius_buffer.reset(new glk::FrameBuffer(size, 0, false));
  feedback_radius_buffer->add_color_buffer(0, GL_R32F, GL_RED, GL_FLOAT).set_filer_mode(GL_NEAREST);  // feedback radius
}

void ScreenSpaceSplatting::draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer) {
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);

  color_texture.set_filer_mode(GL_NEAREST);
  depth_texture.set_filer_mode(GL_NEAREST);

  // uniform variables
  const auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
  const auto projection_matrix = input->get<Eigen::Matrix4f>("projection_matrix");
  if(!view_matrix || !projection_matrix) {
    std::cerr << "view and projection matrices must be set" << std::endl;
    return;
  }

  const GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};
  const Eigen::Matrix4f inv_projection_matrix = (*projection_matrix).inverse();
  const Eigen::Matrix4f projection_view_matrix = (*projection_matrix) * (*view_matrix);
  const Eigen::Matrix4f inv_projection_view_matrix = projection_view_matrix.inverse();

  const Eigen::Vector2f screen_size = position_buffer->size().cast<float>();
  const Eigen::Vector2f inv_screen_size(1.0 / position_buffer->size()[0], 1.0 / position_buffer->size()[1]);

  // calc vertex positions
  position_shader.use();
  position_shader.set_uniform("inv_projection_view_matrix", inv_projection_view_matrix);

  depth_texture.bind();
  position_buffer->bind();
  renderer.draw_plain(position_shader);
  position_buffer->unbind();

  // extract valid points with transform feedback
  point_extraction_shader.use();

  depth_texture.bind();
  points_on_screen->bind();

  glEnable(GL_RASTERIZER_DISCARD);
  glBeginTransformFeedback(GL_POINTS);
  sampling_points_buffer->draw(point_extraction_shader);
  glEndTransformFeedback();
  glDisable(GL_RASTERIZER_DISCARD);

  points_on_screen->unbind();
  point_extraction_shader.unuse();
  depth_texture.unbind();

  // initial radius estimation
  // count the number of vertices in low resolution grid
  white_shader.use();
  initial_estimation_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);

  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_ONE, GL_ONE);

  points_on_screen->draw(white_shader);

  glDisable(GL_BLEND);

  initial_estimation_buffer->unbind();
  white_shader.unuse();

  // initial radius computation
  initial_radius_shader.use();
  initial_radius_shader.set_uniform("inv_screen_size", inv_screen_size);
  initial_radius_shader.set_uniform("inv_projection_matrix", inv_projection_matrix);
  initial_estimation_buffer->color(0).bind();  // num points grid

  radius_buffer_ping->bind();
  glClearBufferfv(GL_COLOR, 0, black);  // radius

  points_on_screen->draw(initial_radius_shader);

  radius_buffer_ping->unbind();

  initial_estimation_buffer->color(0).unbind();
  initial_radius_shader.unuse();

  // distribution
  distribution_shader.use();
  distribution_shader.set_uniform("screen_size", screen_size);
  distribution_shader.set_uniform("inv_screen_size", inv_screen_size);
  distribution_shader.set_uniform("view_matrix", *view_matrix);
  distribution_shader.set_uniform("projection_matrix", *projection_matrix);

  position_buffer->color(0).bind(GL_TEXTURE0);     // position
  radius_buffer_ping->color(0).bind(GL_TEXTURE1);  // radius

  feedback_radius_buffer->bind();
  glClearBufferfv(GL_COLOR, 0, black);

  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glEnable(GL_BLEND);
  glBlendEquation(GL_MAX);
  glBlendFunc(GL_ONE, GL_ONE);

  points_on_screen->draw(distribution_shader);

  glBlendEquation(GL_FUNC_ADD);
  glDisable(GL_BLEND);
  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);

  feedback_radius_buffer->unbind();
  distribution_shader.unuse();

  // gathering

  /*
  std::vector<float> r = vertex_info_buffer->color(0).read_pixels<float>(GL_RED, GL_FLOAT, 1);
  std::cout << "---" << std::endl;
  int count = 0;
  for(int i = 0; i < r.size(); i++) {
    if(std::abs(r[i]) > 1e-6) {
      count++;
      std::cout << i << ":" << r[i] << " ";
    }

    if(count > 100) {
      break;
    }
  }
  std::cout << "num_points:" << count << std::endl;
  */

  guik::LightViewer::instance()->register_ui_callback("buff", [this] {
    ImGui::Image((void*)initial_estimation_buffer->color(0).id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));
    ImGui::Image((void*)feedback_radius_buffer->color(0).id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));
  });

  // put white
  if(frame_buffer) {
    frame_buffer->bind();

    GLfloat black[] = {0.0f, 0.0f, 0.0f, 0.0f};
    glClearBufferfv(GL_COLOR, 0, black);

    white_shader.use();

    // points_on_screen->bind();

    points_on_screen->draw(white_shader);

    // points_on_screen->unbind();

    white_shader.unuse();

    frame_buffer->unbind();
  }

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
}
}  // namespace glk