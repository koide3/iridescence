#include <numeric>
#include <gtest/gtest.h>

#include <glk/path.hpp>
#include <glk/io/ply_io.hpp>

void compare_primary_fields(const glk::PLYData& ply1, const glk::PLYData& ply2) {
  ASSERT_EQ(ply1.vertices.size(), ply2.vertices.size());
  for (int i = 0; i < ply1.vertices.size(); i++) {
    EXPECT_NEAR((ply1.vertices[i] - ply2.vertices[i]).cwiseAbs().maxCoeff(), 0.0, 1e-6);
  }

  ASSERT_EQ(ply1.normals.size(), ply2.normals.size());
  for (int i = 0; i < ply1.normals.size(); i++) {
    EXPECT_NEAR((ply1.normals[i] - ply2.normals[i]).cwiseAbs().maxCoeff(), 0.0, 1e-6);
  }

  ASSERT_EQ(ply1.intensities.size(), ply2.intensities.size());
  for (int i = 0; i < ply1.intensities.size(); i++) {
    EXPECT_NEAR(ply1.intensities[i] - ply2.intensities[i], 0.0, 1e-6);
  }

  ASSERT_EQ(ply1.colors.size(), ply2.colors.size());
  for (int i = 0; i < ply1.colors.size(); i++) {
    EXPECT_NEAR((ply1.colors[i] - ply2.colors[i]).cwiseAbs().maxCoeff(), 0.0, 1e-6);
  }

  ASSERT_EQ(ply1.indices.size(), ply2.indices.size());
  for (int i = 0; i < ply1.indices.size(); i++) {
    EXPECT_EQ(ply1.indices[i], ply2.indices[i]);
  }
}

void compare_generic_fields(const glk::PLYData& ply1, const glk::PLYData& ply2) {
  ASSERT_EQ(ply1.properties.size(), ply2.properties.size());
  for (int i = 0; i < ply1.properties.size(); i++) {
    const auto& prop1 = ply1.properties[i];

    const auto found = std::find_if(ply2.properties.begin(), ply2.properties.end(), [&](const auto& prop) { return prop->name == prop1->name; });
    ASSERT_NE(found, ply2.properties.end());

    const auto& prop2 = *found;
    ASSERT_EQ(prop1->name, prop2->name);
    ASSERT_EQ(prop1->type(), prop2->type());
    ASSERT_EQ(prop1->size(), prop2->size());

    switch (prop1->type()) {
      case glk::PLYPropertyType::CHAR:
        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_EQ(prop1->get<std::int8_t>()[j], prop2->get<std::int8_t>()[j]);
        }
        break;
      case glk::PLYPropertyType::UCHAR:
        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_EQ(prop1->get<std::uint8_t>()[j], prop2->get<std::uint8_t>()[j]);
        }
        break;
      case glk::PLYPropertyType::SHORT:
        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_EQ(prop1->get<std::int16_t>()[j], prop2->get<std::int16_t>()[j]);
        }
        break;
      case glk::PLYPropertyType::USHORT:
        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_EQ(prop1->get<std::uint16_t>()[j], prop2->get<std::uint16_t>()[j]);
        }
        break;
      case glk::PLYPropertyType::INT:
        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_EQ(prop1->get<std::int32_t>()[j], prop2->get<std::int32_t>()[j]);
        }
        break;
      case glk::PLYPropertyType::UINT:
        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_EQ(prop1->get<std::uint32_t>()[j], prop2->get<std::uint32_t>()[j]);
        }
        break;
      case glk::PLYPropertyType::FLOAT:

        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_NEAR(prop1->get<float>()[j], prop2->get<float>()[j], 1e-6);
        }
        break;
      case glk::PLYPropertyType::DOUBLE:

        for (int j = 0; j < prop1->size(); j++) {
          EXPECT_NEAR(prop1->get<double>()[j], prop2->get<double>()[j], 1e-6);
        }
        break;
    }
  }
}

TEST(PLYTest, BunnyTest) {
  const std::string filename = glk::get_data_path() + "/models/bunny.ply";

  // Load test
  auto ply = glk::load_ply(filename);
  ASSERT_NE(ply, nullptr);

  EXPECT_NE(ply->vertices.size(), 0);
  EXPECT_EQ(ply->normals.size(), ply->vertices.size());
  EXPECT_EQ(ply->intensities.size(), ply->vertices.size());
  EXPECT_EQ(ply->colors.size(), 0);

  ASSERT_NE(ply->indices.size(), 0);
  EXPECT_EQ(ply->indices.size() % 3, 0);

  // Binary IO test
  ASSERT_TRUE(glk::save_ply_binary("/tmp/bunny_bin.ply", *ply));

  auto ply2 = glk::load_ply("/tmp/bunny_bin.ply");
  ASSERT_NE(ply2, nullptr);
  compare_primary_fields(*ply, *ply2);

  // ASCII IO test
  ASSERT_TRUE(glk::save_ply_ascii("/tmp/bunny_asc.ply", *ply));

  auto ply3 = glk::load_ply("/tmp/bunny_asc.ply");
  ASSERT_NE(ply3, nullptr);
  compare_primary_fields(*ply, *ply3);
}

TEST(PLYTest, EmptyTest) {
  auto ply = std::make_shared<glk::PLYData>();

  ASSERT_TRUE(glk::save_ply_binary("/tmp/empty.ply", *ply));
  auto ply_bin = glk::load_ply("/tmp/empty.ply");
  ASSERT_NE(ply_bin, nullptr);
  compare_primary_fields(*ply, *ply_bin);

  ASSERT_TRUE(glk::save_ply_ascii("/tmp/empty.ply", *ply));
  auto ply_asc = glk::load_ply("/tmp/empty.ply");
  ASSERT_NE(ply_asc, nullptr);
  compare_primary_fields(*ply, *ply_asc);
}

TEST(PLYTest, CommentTest) {
  auto ply = std::make_shared<glk::PLYData>();
  ply->comments = {"comment1", "comment2 abc", "comment3 def ghi"};

  ASSERT_TRUE(glk::save_ply_binary("/tmp/comment.ply", *ply));
  auto ply_bin = glk::load_ply("/tmp/comment.ply");
  ASSERT_NE(ply_bin, nullptr);

  ASSERT_EQ(ply->comments.size() + 1, ply_bin->comments.size());
  for (const auto& comment : ply->comments) {
    EXPECT_NE(std::find(ply_bin->comments.begin(), ply_bin->comments.end(), comment), ply_bin->comments.end());
  }
  EXPECT_TRUE(std::find(ply_bin->comments.begin(), ply_bin->comments.end(), "generated with iridescence") != ply_bin->comments.end());
}

TEST(PLYTest, PrimaryTest) {
  const std::vector<int> num_vertices = {1, 10, 100, 10000};

  for (int N : num_vertices) {
    const auto test_io = [](const glk::PLYData& ply) {
      ASSERT_TRUE(glk::save_ply_binary("/tmp/test_bin.ply", ply));
      auto ply_bin = glk::load_ply("/tmp/test_bin.ply");
      ASSERT_NE(ply_bin, nullptr);
      compare_primary_fields(ply, *ply_bin);

      ASSERT_TRUE(glk::save_ply_ascii("/tmp/test_asc.ply", ply));
      auto ply_asc = glk::load_ply("/tmp/test_asc.ply");
      ASSERT_NE(ply_asc, nullptr);
      compare_primary_fields(ply, *ply_asc);
    };

    for (int mask = 1; mask < 8; mask++) {
      auto ply = std::make_shared<glk::PLYData>();

      if (mask & (1 << 0)) {
        ply->vertices.resize(N);
        std::generate(ply->vertices.begin(), ply->vertices.end(), [] { return Eigen::Vector3f::Random(); });
      }

      if (mask & (1 << 1)) {
        ply->normals.resize(N);
        std::generate(ply->normals.begin(), ply->normals.end(), [] { return Eigen::Vector3f::Random(); });
      }

      if (mask & (1 << 2)) {
        ply->intensities.resize(N);
        std::generate(ply->intensities.begin(), ply->intensities.end(), [] { return static_cast<float>(rand() % 1024) / 1024.0; });
      }

      if (mask & (1 << 3)) {
        ply->colors.resize(N);
        std::generate(ply->colors.begin(), ply->colors.end(), [] { return Eigen::Vector4f::Random(); });
      }

      test_io(*ply);
    }

    auto ply = std::make_shared<glk::PLYData>();
    ply->vertices.resize(N);
    std::generate(ply->vertices.begin(), ply->vertices.end(), [] { return Eigen::Vector3f::Random(); });
    ply->normals.resize(N);
    std::generate(ply->normals.begin(), ply->normals.end(), [] { return Eigen::Vector3f::Random(); });
    ply->indices.resize(N * 3);
    std::generate(ply->indices.begin(), ply->indices.end(), [=] { return rand() % N; });
    test_io(*ply);
  }
}

template <typename T>
std::shared_ptr<glk::PLYPropertyBuffer<T>> generate_buffer(const std::string& name, int N) {
  std::vector<T> data(N);
  std::generate(data.begin(), data.end(), [] { return static_cast<T>(rand() % 100); });
  return std::make_shared<glk::PLYPropertyBuffer<T>>(name, data.data(), data.size());
}

TEST(PLYTest, GeneralTest) {
  const std::vector<int> num_vertices = {1, 10, 100, 1000};
  for (int N : num_vertices) {
    std::vector<glk::PLYGenericPropertyBuffer::Ptr> props;
    props.push_back(generate_buffer<std::int8_t>("char", N));
    props.push_back(generate_buffer<std::uint8_t>("uchar", N));
    props.push_back(generate_buffer<std::int16_t>("short", N));
    props.push_back(generate_buffer<std::uint16_t>("ushort", N));
    props.push_back(generate_buffer<std::int32_t>("int", N));
    props.push_back(generate_buffer<std::uint32_t>("uint", N));
    props.push_back(generate_buffer<float>("float", N));
    props.push_back(generate_buffer<double>("double", N));

    const auto test_io = [](const glk::PLYData& ply) {
      ASSERT_TRUE(glk::save_ply_binary("/tmp/test_bin.ply", ply));
      auto ply_bin = glk::load_ply("/tmp/test_bin.ply");
      ASSERT_NE(ply_bin, nullptr);
      compare_generic_fields(ply, *ply_bin);

      ASSERT_TRUE(glk::save_ply_ascii("/tmp/test_asc.ply", ply));
      auto ply_asc = glk::load_ply("/tmp/test_asc.ply");
      ASSERT_NE(ply_asc, nullptr);
      compare_generic_fields(ply, *ply_asc);
    };

    for (int mask = 1; mask < (1 << props.size()); mask++) {
      auto ply = std::make_shared<glk::PLYData>();

      for (int select = 0; select < props.size(); select++) {
        if (mask & (1 << select)) {
          ply->properties.push_back(props[select]);
        }
      }

      test_io(*ply);
    }
  }
}