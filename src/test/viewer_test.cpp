#include <numeric>
#include <gtest/gtest.h>

#include <glk/io/png_io.hpp>
#include <glk/lines.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

bool test_transform(int i) {
  Eigen::Vector3f trans(-10.0f, -10.0f, 0.0f);
  Eigen::AngleAxisf rot(0.6f, Eigen::Vector3f(1.0f, 2.0f, 3.0f).normalized());

  Eigen::Vector3d trans2(-10.0f, 0.0f, 0.0f);
  Eigen::AngleAxisd rot2(0.6f, Eigen::Vector3d(1.0f, 2.0f, 3.0f).normalized());

  guik::ShaderSetting setting;
  guik::ShaderSetting setting2;
  guik::ShaderSetting setting3;

  switch (i) {
    default:
    case 0: {
      Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
      matrix.block<3, 3>(0, 0) = rot.toRotationMatrix();
      matrix.block<3, 1>(0, 3) = trans;
      setting = guik::Rainbow(matrix);

      Eigen::Matrix4d matrix2 = Eigen::Matrix4d::Identity();
      matrix2.block<3, 3>(0, 0) = rot2.toRotationMatrix();
      matrix2.block<3, 1>(0, 3) = trans2;
      setting2 = guik::Rainbow(matrix2);
    } break;
    case 1: {
      Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
      matrix.block<3, 3>(0, 0) = rot.toRotationMatrix();
      matrix.block<3, 1>(0, 3) = trans;
      setting = guik::Rainbow().set_model_matrix(matrix);

      Eigen::Matrix4d matrix2 = Eigen::Matrix4d::Identity();
      matrix2.block<3, 3>(0, 0) = rot2.toRotationMatrix();
      matrix2.block<3, 1>(0, 3) = trans2;
      setting2 = guik::Rainbow().set_model_matrix(matrix2);
    } break;
    case 2: {
      Eigen::Isometry3f matrix = Eigen::Isometry3f::Identity();
      matrix.linear() = rot.toRotationMatrix();
      matrix.translation() = trans;
      setting = guik::Rainbow(matrix);

      Eigen::Isometry3d matrix2 = Eigen::Isometry3d::Identity();
      matrix2.linear() = rot2.toRotationMatrix();
      matrix2.translation() = trans2;
      setting2 = guik::Rainbow(matrix2);
    } break;
    case 3: {
      Eigen::Isometry3f matrix = Eigen::Isometry3f::Identity();
      matrix.linear() = rot.toRotationMatrix();
      matrix.translation() = trans;
      setting = guik::Rainbow().set_model_matrix(matrix);

      Eigen::Isometry3d matrix2 = Eigen::Isometry3d::Identity();
      matrix2.linear() = rot2.toRotationMatrix();
      matrix2.translation() = trans2;
      setting2 = guik::Rainbow().set_model_matrix(matrix2);
    } break;
    case 4: {
      Eigen::Affine3f matrix = Eigen::Isometry3f::Identity();
      matrix.linear() = rot.toRotationMatrix();
      matrix.translation() = trans;
      setting = guik::Rainbow(matrix);

      Eigen::Affine3d matrix2 = Eigen::Affine3d::Identity();
      matrix2.linear() = rot2.toRotationMatrix();
      matrix2.translation() = trans2;
      setting2 = guik::Rainbow(matrix2);
    } break;
    case 5: {
      Eigen::Affine3f matrix = Eigen::Isometry3f::Identity();
      matrix.linear() = rot.toRotationMatrix();
      matrix.translation() = trans;
      setting = guik::Rainbow().set_model_matrix(matrix);

      Eigen::Affine3d matrix2 = Eigen::Affine3d::Identity();
      matrix2.linear() = rot2.toRotationMatrix();
      matrix2.translation() = trans2;
      setting2 = guik::Rainbow().set_model_matrix(matrix2);
    } break;
    case 6: {
      setting = guik::Rainbow(Eigen::Translation3f(trans) * rot);
      setting2 = guik::Rainbow(Eigen::Translation3d(trans2) * rot2);
    } break;
    case 7: {
      setting = guik::Rainbow().set_model_matrix(Eigen::Translation3f(trans) * rot);
      setting2 = guik::Rainbow().set_model_matrix(Eigen::Translation3d(trans2) * rot2);
    } break;
    case 8: {
      setting = guik::Rainbow().translate(trans).rotate(rot.angle(), rot.axis());
      setting2 = guik::Rainbow().translate(trans2).rotate(rot.angle(), rot.axis());
    } break;
    case 9: {
      setting = guik::Rainbow().translate(Eigen::Vector4f(trans.x(), trans.y(), trans.z(), 1.0f)).rotate(rot.toRotationMatrix());
      setting2 = guik::Rainbow().translate(Eigen::Vector4d(trans2.x(), trans2.y(), trans2.z(), 1.0f)).rotate(rot.toRotationMatrix());
    } break;
    case 10: {
      setting = guik::Rainbow().translate(Eigen::Vector4f(trans.x(), trans.y(), trans.z(), 1.0f)).rotate(Eigen::Quaternionf(rot));
      setting2 = guik::Rainbow().translate(Eigen::Vector4d(trans2.x(), trans2.y(), trans2.z(), 1.0f)).rotate(Eigen::Quaterniond(rot2));
    } break;
  }

  auto viewer = guik::viewer();
  viewer->update_icosahedron("transform1", setting);
  viewer->update_icosahedron("transform2", setting2);

  return i <= 10;
}

bool test_primitives(int i) {
  auto viewer = guik::viewer();

  if (i == 0) {
    viewer->update_drawable("sphere1", glk::Primitives::sphere(), guik::FlatRed().translate(-20.0f, -25.0f, 0.0f));
    viewer->update_drawable("sphere2", glk::Primitives::sphere(), guik::FlatGreen().translate(-20.0f, -22.5f, 0.0f));
    viewer->update_drawable("sphere3", glk::Primitives::sphere(), guik::FlatBlue().translate(-20.0f, -20.0f, 0.0f));
    viewer->update_drawable("sphere4", glk::Primitives::sphere(), guik::FlatOrange().translate(-20.0f, -17.5f, 0.0f));
    viewer->update_drawable("sphere5", glk::Primitives::sphere(), guik::Rainbow().translate(-20.0f, -15.0f, 0.0f));
    viewer->update_drawable("sphere6", glk::Primitives::sphere(), guik::Rainbow().translate(-20.0f, -10.0f, 0.0f).scale(2.0f));
    viewer->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow().translate(-20.0f, -5.0f, 0.0f));
    viewer->update_drawable("cone", glk::Primitives::cone(), guik::Rainbow().translate(-20.0f, -2.5f, 0.0f));
    viewer->update_drawable("coord", glk::Primitives::coordinate_system(), guik::VertexColor().translate(-20.0f, 0.0f, 0.0f));
    viewer->update_drawable("frustum", glk::Primitives::frustum(), guik::Rainbow().translate(-20.0f, 2.5f, 0.0f));
    viewer->update_drawable("icosahedron", glk::Primitives::icosahedron(), guik::Rainbow().translate(-20.0f, 5.0f, 0.0f));
    viewer->update_drawable("bunny", glk::Primitives::bunny(), guik::Rainbow().translate(-20.0f, 7.5f, 0.0f).scale(10.0f));
    viewer->update_drawable("trans_cube", glk::Primitives::cube(), guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f).translate(-20.0f, 12.5f, 0.0f).make_transparent());

    viewer->update_drawable("wire_sphere1", glk::Primitives::wire_sphere(), guik::FlatRed().translate(-15.0f, -25.0f, 0.0f));
    viewer->update_drawable("wire_sphere2", glk::Primitives::wire_sphere(), guik::FlatGreen().translate(-15.0f, -22.5f, 0.0f));
    viewer->update_drawable("wire_sphere3", glk::Primitives::wire_sphere(), guik::FlatBlue().translate(-15.0f, -20.0f, 0.0f));
    viewer->update_drawable("wire_sphere4", glk::Primitives::wire_sphere(), guik::FlatOrange().translate(-15.0f, -17.5f, 0.0f));
    viewer->update_drawable("wire_sphere5", glk::Primitives::wire_sphere(), guik::Rainbow().translate(-15.0f, -15.0f, 0.0f));
    viewer->update_drawable("wire_sphere6", glk::Primitives::wire_sphere(), guik::Rainbow().translate(-15.0f, -10.0f, 0.0f).scale(2.0f));
    viewer->update_drawable("wire_cube", glk::Primitives::wire_cube(), guik::Rainbow().translate(-15.0f, -5.0f, 0.0f));
    viewer->update_drawable("wire_cone", glk::Primitives::wire_cone(), guik::Rainbow().translate(-15.0f, -2.5f, 0.0f));
    viewer->update_drawable("wire_coord", glk::Primitives::coordinate_system(), guik::VertexColor().translate(-15.0f, 0.0f, 0.0f));
    viewer->update_drawable("wire_frustum", glk::Primitives::wire_frustum(), guik::Rainbow().translate(-15.0f, 2.5f, 0.0f));
    viewer->update_drawable("wire_icosahedron", glk::Primitives::wire_icosahedron(), guik::Rainbow().translate(-15.0f, 5.0f, 0.0f));
    viewer->update_drawable("wire_bunny", glk::Primitives::wire_bunny(), guik::Rainbow().translate(-15.0f, 7.5f, 0.0f).scale(10.0f));

    return true;
  } else {
    viewer->update_sphere("sphere1", guik::FlatRed().translate(-20.0f, -25.0f, 0.0f));
    viewer->update_sphere("sphere2", guik::FlatGreen().translate(-20.0f, -22.5f, 0.0f));
    viewer->update_sphere("sphere3", guik::FlatBlue().translate(-20.0f, -20.0f, 0.0f));
    viewer->update_sphere("sphere4", guik::FlatOrange().translate(-20.0f, -17.5f, 0.0f));
    viewer->update_sphere("sphere5", guik::Rainbow().translate(-20.0f, -15.0f, 0.0f));
    viewer->update_sphere("sphere6", guik::Rainbow().translate(-20.0f, -10.0f, 0.0f).scale(2.0f));
    viewer->update_cube("cube", guik::Rainbow().translate(-20.0f, -5.0f, 0.0f));
    viewer->update_cone("cone", guik::Rainbow().translate(-20.0f, -2.5f, 0.0f));
    viewer->update_coord("coord", guik::VertexColor().translate(-20.0f, 0.0f, 0.0f));
    viewer->update_frustum("frustum", guik::Rainbow().translate(-20.0f, 2.5f, 0.0f));
    viewer->update_icosahedron("icosahedron", guik::Rainbow().translate(-20.0f, 5.0f, 0.0f));
    viewer->update_drawable("bunny", glk::Primitives::bunny(), guik::Rainbow().translate(-20.0f, 7.5f, 0.0f).scale(10.0f));
    viewer->update_cube("trans_cube", guik::FlatColor(1.0f, 0.5f, 0.0f, 0.5f).translate(-20.0f, 12.5f, 0.0f).make_transparent());

    viewer->update_wire_sphere("wire_sphere1", guik::FlatRed().translate(-15.0f, -25.0f, 0.0f));
    viewer->update_wire_sphere("wire_sphere2", guik::FlatGreen().translate(-15.0f, -22.5f, 0.0f));
    viewer->update_wire_sphere("wire_sphere3", guik::FlatBlue().translate(-15.0f, -20.0f, 0.0f));
    viewer->update_wire_sphere("wire_sphere4", guik::FlatOrange().translate(-15.0f, -17.5f, 0.0f));
    viewer->update_wire_sphere("wire_sphere5", guik::Rainbow().translate(-15.0f, -15.0f, 0.0f));
    viewer->update_wire_sphere("wire_sphere6", guik::Rainbow().translate(-15.0f, -10.0f, 0.0f).scale(2.0f));
    viewer->update_wire_cube("wire_cube", guik::Rainbow().translate(-15.0f, -5.0f, 0.0f));
    viewer->update_wire_cone("wire_cone", guik::Rainbow().translate(-15.0f, -2.5f, 0.0f));
    viewer->update_coord("wire_coord", guik::VertexColor().translate(-15.0f, 0.0f, 0.0f));
    viewer->update_wire_frustum("wire_frustum", guik::Rainbow().translate(-15.0f, 2.5f, 0.0f));
    viewer->update_wire_icosahedron("wire_icosahedron", guik::Rainbow().translate(-15.0f, 5.0f, 0.0f));
    viewer->update_drawable("wire_bunny", glk::Primitives::wire_bunny(), guik::Rainbow().translate(-15.0f, 7.5f, 0.0f).scale(10.0f));

    return false;
  }
}

template <typename T1, int D1, typename T2>
void test_lines() {
  std::vector<Eigen::Matrix<T1, D1, 1>> line;
  std::vector<Eigen::Matrix<T2, 4, 1>> colors;
  std::vector<Eigen::Vector4i> infos;
  for (double y = 15.0; y < 20.0; y += 0.2) {
    Eigen::Matrix<T1, D1, 1> p = Eigen::Matrix<T1, D1, 1>::Ones();
    p.template head<3>() << -20.0 + std::sin(y * 5), y, 0.0;

    line.emplace_back(p);

    const Eigen::Vector4f c = glk::colormapf(glk::COLORMAP::TURBO, (y - 15.0f) / 5.0f);
    colors.emplace_back(c[0], c[1], c[2], c[3]);

    infos.emplace_back(0, 1, 2, 3);
  }

  std::vector<unsigned int> indices(line.size() / 2);
  std::iota(indices.begin(), indices.end(), line.size() / 4);

  auto viewer = guik::viewer();

  viewer->update_drawable("thin_line1", std::make_shared<glk::ThinLines>(line, true), guik::FlatGreen());
  viewer->update_drawable("thin_line2", std::make_shared<glk::ThinLines>(line, colors, true), guik::VertexColor().translate(0.0f, 5.0f, 0.0f));
  viewer->update_drawable("thin_line3", std::make_shared<glk::ThinLines>(line, false), guik::FlatGreen().translate(0.0f, 10.0f, 0.0f));
  viewer->update_drawable("thin_line4", std::make_shared<glk::ThinLines>(line, indices, true), guik::FlatGreen().translate(0.0f, 15.0f, 0.0f));
  viewer->update_drawable("thin_line5", std::make_shared<glk::ThinLines>(line, colors, indices, true), guik::VertexColor().translate(0.0f, 20.5f, 0.0f));

  viewer->update_drawable("thin_line6", std::make_shared<glk::ThinLines>(line.data(), line.size(), true), guik::FlatGreen().translate(5.0f, 0.0f, 0.0f));
  viewer->update_drawable("thin_line7", std::make_shared<glk::ThinLines>(line.data(), colors.data(), line.size(), true), guik::VertexColor().translate(5.0f, 5.0f, 0.0f));
  viewer->update_drawable("thin_line8", std::make_shared<glk::ThinLines>(line.data(), colors.data(), line.size(), false), guik::FlatGreen().translate(5.0f, 10.0f, 0.0f));
  viewer->update_drawable(
    "thin_line9",
    std::make_shared<glk::ThinLines>(line.data(), line.size(), indices.data(), indices.size(), true),
    guik::FlatGreen().translate(5.0f, 15.0f, 0.0f));
  viewer->update_drawable(
    "thin_line10",
    std::make_shared<glk::ThinLines>(line.data(), colors.data(), line.size(), indices.data(), indices.size(), true),
    guik::VertexColor().translate(5.0f, 20.0f, 0.0f));

  viewer->update_drawable("line1", std::make_shared<glk::Lines>(0.1f, line, true), guik::FlatGreen());
  viewer->update_drawable("line2", std::make_shared<glk::Lines>(0.1f, line, colors, true), guik::VertexColor().translate(0.0f, 5.0f, 0.0f));
  viewer->update_drawable("line3", std::make_shared<glk::Lines>(0.1f, line, false), guik::FlatGreen().translate(0.0f, 10.0f, 0.0f));
  viewer->update_drawable("line4", std::make_shared<glk::Lines>(0.1f, line, colors, infos, true), guik::FlatGreen().translate(0.0f, 15.0f, 0.0f));

  viewer->update_drawable("line6", std::make_shared<glk::Lines>(0.1f, line.data(), line.size(), true), guik::FlatGreen().translate(10.0f, 0.0f, 0.0f));
  viewer->update_drawable("line7", std::make_shared<glk::Lines>(0.1f, line.data(), colors.data(), line.size(), true), guik::VertexColor().translate(10.0f, 5.0f, 0.0f));
  viewer->update_drawable("line8", std::make_shared<glk::Lines>(0.1f, line.data(), colors.data(), line.size(), false), guik::FlatGreen().translate(10.0f, 10.0f, 0.0f));
  viewer->update_drawable(
    "line9",
    std::make_shared<glk::Lines>(0.1f, line.data(), colors.data(), infos.data(), line.size(), false),
    guik::FlatGreen().translate(10.0f, 15.0f, 0.0f));
}

bool test_lines(int i) {
  auto viewer = guik::viewer();

  switch (i) {
    case 0:
      test_lines<float, 3, float>();
      return true;
    case 1:
      test_lines<float, 3, double>();
      return true;
    case 2:
      test_lines<float, 4, float>();
      return true;
    case 3:
      test_lines<float, 4, double>();
      return true;
    case 4:
      test_lines<double, 3, float>();
      return true;
    case 5:
      test_lines<double, 3, double>();
      return true;
    case 6:
      test_lines<double, 4, float>();
      return true;
    case 7:
      test_lines<double, 4, double>();
      return true;
  }

  test_lines<float, 3, float>();
  return false;
}

template <typename T1, int D1, typename T2>
void test_points(bool use_ptr) {
  std::vector<Eigen::Matrix<T1, D1, 1>> points;
  std::vector<Eigen::Matrix<T2, 4, 1>> colors;
  std::vector<unsigned int> indices;

  for (double x = -22.0; x <= -18.0; x += 1.0) {
    for (double y = -35.0; y <= -27.5; y += 1.0) {
      for (double z = -2.5; z <= 2.5; z += 0.5) {
        Eigen::Matrix<T1, D1, 1> pt = Eigen::Matrix<T1, D1, 1>::Ones();
        pt.template head<3>() << x, y, z;
        points.emplace_back(pt);

        Eigen::Vector4f color = glk::colormapf(glk::COLORMAP::TURBO, (z + 2.5) / 5.0);
        colors.emplace_back(color.cast<T2>());
      }
    }
  }

  std::vector<T2> intensities(points.size());
  for (int i = 0; i < points.size(); i++) {
    intensities[i] = static_cast<T2>(i) / points.size();
  }

  indices.resize(points.size() / 2);
  std::iota(indices.begin(), indices.end(), points.size() / 4);

  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer2;
  if (use_ptr) {
    cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points.data(), points.size());
    cloud_buffer->add_color(colors.data(), colors.size());

    cloud_buffer2 = std::make_shared<glk::PointCloudBuffer>(points.data(), points.size());
    cloud_buffer2->add_intensity(glk::COLORMAP::TURBO, intensities.data(), intensities.size());
  } else {
    cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
    cloud_buffer->add_color(colors);

    cloud_buffer2 = std::make_shared<glk::PointCloudBuffer>(points);
    cloud_buffer2->add_intensity(glk::COLORMAP::TURBO, intensities);
  }

  auto viewer = guik::viewer();
  viewer->update_drawable("points1", cloud_buffer, guik::VertexColor());
  viewer->update_drawable("points2", cloud_buffer, guik::VertexColor().set_point_scale(2.0f).translate(10.0f, 0.0f, 0.0f));
  viewer->update_drawable("points3", cloud_buffer, guik::VertexColor().set_point_scale(2.0f).translate(20.0f, 0.0f, 0.0f));

  auto indexed_buffer = std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, indices);
  viewer->update_drawable("points4", indexed_buffer, guik::VertexColor().set_point_scale(2.0f).translate(30.0f, 0.0f, 0.0f));
}

bool test_points(int i) {
  switch (i) {
    case 0:
      test_points<float, 3, float>(true);
      return true;
    case 1:
      test_points<float, 3, float>(false);
      return true;
    case 2:
      test_points<float, 3, double>(true);
      return true;
    case 3:
      test_points<float, 3, double>(false);
      return true;
    case 4:
      test_points<float, 4, float>(true);
      return true;
    case 5:
      test_points<float, 4, float>(false);
      return true;
    case 6:
      test_points<float, 4, double>(true);
      return true;
    case 7:
      test_points<float, 4, double>(false);
      return true;
    case 8:
      test_points<double, 3, float>(true);
      return true;
    case 9:
      test_points<double, 3, float>(false);
      return true;
    case 10:
      test_points<double, 3, double>(true);
      return true;
    case 11:
      test_points<double, 3, double>(false);
      return true;
    case 12:
      test_points<double, 4, float>(true);
      return true;
    case 13:
      test_points<double, 4, float>(false);
      return true;
    case 14:
      test_points<double, 4, double>(true);
      return true;
    case 15:
      test_points<double, 4, double>(false);
      return true;
  }

  test_points<float, 3, float>(true);
  return false;
}

double pixel_error(const std::vector<unsigned char>& reference, const std::vector<unsigned char>& pixels) {
  if (reference.size() != pixels.size()) {
    return -1.0;
  }

  double sum = 0.0;
  for (int i = 0; i < reference.size(); i++) {
    sum += std::pow(reference[i] - pixels[i], 2);
  }

  return sum / reference.size();
}

TEST(ViewerTest, Primitives) {
  auto viewer = guik::viewer("viewer_test", {-1, -1}, false);
  viewer->disable_vsync();

  std::vector<unsigned char> reference;

  std::vector<std::pair<std::string, std::function<bool(int)>>> tests = {
    {"primitives", [](int i) { return test_primitives(i); }},
    {"transform", [](int i) { return test_transform(i); }},
    {"lines", [](int i) { return test_lines(i); }},
    {"points", [](int i) { return test_points(i); }},
  };

  for (int i = 0;; i++) {
    viewer->clear();

    bool cont = false;
    for (const auto& [name, test] : tests) {
      cont |= test(i);
    }
    viewer->spin_once();

    if (i == 0) {
      reference = viewer->read_color_buffer();
    } else {
      auto pixels = viewer->read_color_buffer();
      const double error = pixel_error(reference, pixels);
      std::cout << i << " : error=" << error << std::endl;
      EXPECT_LE(error, 0.1) << i << " : error=" << error;

      if (error > 0.1) {
        for (int j = 0; j < tests.size(); j++) {
          viewer->clear();

          for (int k = 0; k < tests.size(); k++) {
            if (j == k) {
              tests[k].second(j);
            } else {
              tests[k].second(0);
            }
          }

          viewer->spin_once();
          auto pixels = viewer->read_color_buffer();
          const double error = pixel_error(reference, pixels);
          EXPECT_LE(error, 0.1) << "failure on trial " << i << " for " << tests[j].first << " : error=" << error;
        }
      }
    }

    if (!cont) {
      break;
    }
  }
}
