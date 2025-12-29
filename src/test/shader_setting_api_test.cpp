#include <gtest/gtest.h>
#include <guik/viewer/light_viewer.hpp>

TEST(ShaderSettingTest, ParameterTest) {
  const Eigen::Vector2f v2f(1, 2);
  const Eigen::Vector3f v3f(1, 2, 3);
  const Eigen::Vector4f v4f(1, 2, 3, 4);
  const Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();

  const Eigen::Vector2i v2i(1, 2);
  const Eigen::Vector3i v3i(1, 2, 3);
  const Eigen::Vector4i v4i(1, 2, 3, 4);

  std::vector<float> vf = {1, 2, 3, 4, 5, 6, 7, 8};
  std::vector<int> vi = {1, 2, 3, 4, 5, 6, 7, 8};

  // cast
  EXPECT_EQ(guik::Rainbow().add("test1", 1).cast<int>("test1"), 1);
  EXPECT_NEAR(guik::Rainbow().add("test1", 1.0f).cast<float>("test1"), 1.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", v2f).cast<Eigen::Vector2f>("test1") - v2f).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", v3f).cast<Eigen::Vector3f>("test1") - v3f).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", v4f).cast<Eigen::Vector4f>("test1") - v4f).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", v2i).cast<Eigen::Vector2i>("test1") - v2i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", v3i).cast<Eigen::Vector3i>("test1") - v3i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", v4i).cast<Eigen::Vector4i>("test1") - v4i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().add("test1", m4f).cast<Eigen::Matrix4f>("test1") - m4f).norm(), 0.0, 1e-6);

  auto vf2 = guik::Rainbow().add("test1", vf).cast<std::vector<float>>("test1");
  for (int i = 0; i < vf.size(); i++) {
    EXPECT_NEAR(vf2[i], vf[i], 1e-6);
  }

  auto vi2 = guik::Rainbow().add("test1", vi).cast<std::vector<int>>("test1");
  for (int i = 0; i < vi.size(); i++) {
    EXPECT_EQ(vi2[i], vi[i]);
  }

  // get
  EXPECT_TRUE(guik::Rainbow().get<int>("test1") == std::nullopt);
  EXPECT_TRUE(guik::Rainbow().add("test1", 1).get<int>("test1") != std::nullopt);

  EXPECT_EQ(*guik::Rainbow().add("test1", 1).get<int>("test1"), 1);
  EXPECT_NEAR(*guik::Rainbow().add("test1", 1.0f).get<float>("test1"), 1.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", v2f).get<Eigen::Vector2f>("test1") - v2f).norm(), 0.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", v3f).get<Eigen::Vector3f>("test1") - v3f).norm(), 0.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", v4f).get<Eigen::Vector4f>("test1") - v4f).norm(), 0.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", v2i).get<Eigen::Vector2i>("test1") - v2i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", v3i).get<Eigen::Vector3i>("test1") - v3i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", v4i).get<Eigen::Vector4i>("test1") - v4i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((*guik::Rainbow().add("test1", m4f).get<Eigen::Matrix4f>("test1") - m4f).norm(), 0.0, 1e-6);

  vf2 = *guik::Rainbow().add("test1", vf).get<std::vector<float>>("test1");
  for (int i = 0; i < vf.size(); i++) {
    EXPECT_NEAR(vf2[i], vf[i], 1e-6);
  }

  vi2 = *guik::Rainbow().add("test1", vi).get<std::vector<int>>("test1");
  for (int i = 0; i < vi.size(); i++) {
    EXPECT_EQ(vi2[i], vi[i]);
  }
}

TEST(ShaderSettingTest, CloneTest) {
  const Eigen::Vector2f v2 = Eigen::Vector2f::Random();
  const Eigen::Vector3f v3 = Eigen::Vector3f::Random();
  const Eigen::Vector4f v4 = Eigen::Vector4f::Random();
  const Eigen::Vector2i v2i = Eigen::Vector2i::Random();
  const Eigen::Vector3i v3i = Eigen::Vector3i::Random();
  const Eigen::Vector4i v4i = Eigen::Vector4i::Random();
  const Eigen::Matrix4f m4 = Eigen::Matrix4f::Random();

  const std::vector<float> vf = {1, 2, 3, 4, 5, 6, 7, 8};
  const std::vector<int> vi = {1, 2, 3, 4, 5, 6, 7, 8};

  auto setting = guik::Rainbow(m4);
  setting.add("int", 1);
  setting.add("float", 1.0f);
  setting.add("v2", v2);
  setting.add("v3", v3);
  setting.add("v4", v4);
  setting.add("v2i", v2i);
  setting.add("v3i", v3i);
  setting.add("v4i", v4i);
  setting.add("m4", m4);
  setting.add("vf", vf);
  setting.add("vi", vi);

  auto cloned = setting.clone();
  EXPECT_EQ(cloned.color_mode(), guik::ColorMode::RAINBOW);
  EXPECT_EQ(cloned.cast<int>("int"), 1);
  EXPECT_NEAR(cloned.cast<float>("float"), 1.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Vector2f>("v2") - v2).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Vector3f>("v3") - v3).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Vector4f>("v4") - v4).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Vector2i>("v2i") - v2i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Vector3i>("v3i") - v3i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Vector4i>("v4i") - v4i).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.cast<Eigen::Matrix4f>("m4") - m4).norm(), 0.0, 1e-6);
  EXPECT_NEAR((cloned.model_matrix() - m4).norm(), 0.0, 1e-6);

  const auto vf2 = cloned.cast<std::vector<float>>("vf");
  for (int i = 0; i < vf.size(); i++) {
    EXPECT_NEAR(vf2[i], vf[i], 1e-6);
  }

  const auto vi2 = cloned.cast<std::vector<int>>("vi");
  for (int i = 0; i < vi.size(); i++) {
    EXPECT_EQ(vi2[i], vi[i]);
  }
}

TEST(ShaderSettingTest, TranslationTest) {
  const Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

  const Eigen::Quaternionf quatf = Eigen::Quaternionf::Identity();
  const Eigen::Isometry3f posef = Eigen::Isometry3f::Identity();
  const Eigen::Matrix4f matf = Eigen::Matrix4f::Identity();

  const Eigen::Vector3d t = Eigen::Vector3d(1, 2, 3);
  const Eigen::Vector3f tf = Eigen::Vector3f(1, 2, 3);

  const Eigen::Vector4d t4 = Eigen::Vector4d(1, 2, 3, 1);
  const Eigen::Vector4f tf4 = Eigen::Vector4f(1, 2, 3, 1);

  EXPECT_NEAR((guik::Rainbow().translate(t).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(t * 1.0).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(quat * t).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(pose * t).translation() - tf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().translate(tf).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(tf * 1.0).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(quatf * tf).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(posef * tf).translation() - tf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().translate(t4).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(t4 * 1.0).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(t4.head<3>()).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(pose * t4).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(mat * t4).translation() - tf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().translate(tf4).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(tf4 * 1.0).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(tf4.head<3>()).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(posef * tf4).translation() - tf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().translate(matf * tf4).translation() - tf).norm(), 0.0, 1e-6);
}

TEST(ShaderSettingTest, ModelMatrixTest) {
  // SE3
  const Eigen::Quaterniond quat = Eigen::Quaterniond::UnitRandom();
  const Eigen::Vector3d trans = Eigen::Vector3d::Random();
  const Eigen::AngleAxisd aa(quat.toRotationMatrix());

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quat.toRotationMatrix();
  pose.translation() = trans;

  const Eigen::Matrix4d mat = pose.matrix();

  const Eigen::Quaternionf quatf = quat.cast<float>();
  const Eigen::Vector3f transf = trans.cast<float>();
  const Eigen::AngleAxisf aaf(quatf.toRotationMatrix());

  Eigen::Isometry3f posef = Eigen::Isometry3f::Identity();
  posef.linear() = quatf.toRotationMatrix();
  posef.translation() = transf;

  const Eigen::Matrix4f matf = posef.matrix();

  // double
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(pose).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(Eigen::Isometry3d::Identity() * pose).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(mat).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(Eigen::Matrix4d::Identity() * mat).model_matrix() - matf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().transform(pose).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().transform(Eigen::Isometry3d::Identity() * pose).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().transform(mat).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().transform(Eigen::Matrix4d::Identity() * mat).model_matrix() - matf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().transform(Eigen::Translation3d(trans) * quat).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().rotate(mat.block<3, 3>(0, 0).eval()).translate(trans).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().rotate(quat).translate(trans).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().rotate(aa.angle(), aa.axis()).translate(trans).model_matrix() - matf).norm(), 0.0, 1e-6);

  // float
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(posef).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(Eigen::Isometry3f::Identity() * posef).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(matf).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_model_matrix(Eigen::Matrix4f::Identity() * matf).model_matrix() - matf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().transform(posef).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().transform(Eigen::Isometry3f::Identity() * posef).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().transform(matf).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().transform(Eigen::Matrix4f::Identity() * matf).model_matrix() - matf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().transform(Eigen::Translation3f(transf) * quatf).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().rotate(matf.block<3, 3>(0, 0).eval()).translate(transf).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().rotate(quatf).translate(transf).model_matrix() - matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().rotate(aaf.angle(), aaf.axis()).translate(transf).model_matrix() - matf).norm(), 0.0, 1e-6);

  // Scale
  const Eigen::Vector3d scale = Eigen::Vector3d::Random();
  const Eigen::Vector3f scalef = scale.cast<float>();
  Eigen::Matrix4d scale_mat = Eigen::Matrix4d::Identity();
  scale_mat.block<3, 3>(0, 0) = scale.asDiagonal();
  const Eigen::Matrix4f scale_matf = scale_mat.cast<float>();

  guik::Rainbow().scale(1.0f);
  guik::Rainbow().scale(1.0);
  guik::Rainbow().scale(2);

  EXPECT_NEAR((guik::Rainbow().scale(scale).model_matrix() - scale_matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().scale(scale * 1.0).model_matrix() - scale_matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().scale(scalef).model_matrix() - scale_matf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().scale(scalef * 1.0).model_matrix() - scale_matf).norm(), 0.0, 1e-6);
}

TEST(ShaderSettingTest, ColorTest) {
  const Eigen::Vector4d color = Eigen::Vector4d::Random();
  const Eigen::Vector4f colorf = color.cast<float>();

  // double
  EXPECT_NEAR((guik::FlatColor({color[0], color[1], color[2], color[3]}).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(color).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(color * 1.0).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(color, Eigen::Isometry3d::Identity()).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(color * 1.0, Eigen::Isometry3d::Identity()).material_color() - colorf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().set_color(color[0], color[1], color[2], color[3]).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_color(color).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_color(color * 1.0).material_color() - colorf).norm(), 0.0, 1e-6);

  // float
  EXPECT_NEAR((guik::FlatColor({colorf[0], colorf[1], colorf[2], colorf[3]}).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(colorf).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(colorf * 1.0).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(colorf, Eigen::Isometry3f::Identity()).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::FlatColor(colorf * 1.0, Eigen::Isometry3f::Identity()).material_color() - colorf).norm(), 0.0, 1e-6);

  EXPECT_NEAR((guik::Rainbow().set_color(colorf).material_color() - colorf).norm(), 0.0, 1e-6);
  EXPECT_NEAR((guik::Rainbow().set_color(colorf * 1.0).material_color() - colorf).norm(), 0.0, 1e-6);
}
