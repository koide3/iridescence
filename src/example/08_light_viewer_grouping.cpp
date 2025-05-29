#include <glk/pointcloud_buffer.hpp>
#include <glk/thin_lines.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/drawable_tree.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>
#include <Eigen/Core>
#include <unordered_map>
#include <stdio.h>

struct drawableTreeNode {
  std::string value;
  std::string parentField;
  std::string dataType;
  bool isParent;
  std::unordered_map<std::string, std::shared_ptr<drawableTreeNode>> children;

  drawableTreeNode(std::string value, std::string parentField, bool isParent, std::string dataType = "TopicPath")
  : value(value),
    parentField(parentField),
    isParent(isParent),
    dataType(dataType) {
    // Initialize the children map
    children = std::unordered_map<std::string, std::shared_ptr<drawableTreeNode>>();
  }

  std::shared_ptr<drawableTreeNode> addChild(std::string childName, drawableTreeNode childNode) {
    // check if the childName already exists
    if (children.find(childName) == children.end()) {
      children[childName] = std::make_shared<drawableTreeNode>(childNode);
      return children[childName];
    } else {
      // if the childName already exists, return the existing child node
      return children[childName];
    }
  }

  std::shared_ptr<drawableTreeNode> getChild(std::string childName) {
    if (children.find(childName) != children.end()) {
      return children[childName];
    } else {
      return nullptr;
    }
  }
};

class GroupEditor {
public:
  GroupEditor() {
    auto viewer = guik::LightViewer::instance();
    viewer->register_ui_callback("ui", [this] { ui_callback(); });
    viewer->spin();
  }

private:
  void ui_callback() {
    auto viewer = guik::LightViewer::instance();
    ImGui::SetNextWindowSize(ImVec2(800, 1200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Object List Panel");
    createDrawableTreeDisplay(rootTree_);

    ImGui::Separator();
    if (ImGui::Button("Generate Random Scene")) {
      GenRandomScene();
    }

    if (selectedTopicNode_ != nullptr) {
      ImGui::Separator();

      if (ImGui::Button("Remove Node")) {
        std::string inputDrawableName = selectedTopicNode_->parentField + selectedTopicNode_->value;
        drawableTree_->remove(inputDrawableName);
        removeTreeNodeDisplay(selectedTopicNode_->parentField, selectedTopicNode_->value);
        selectedTopicNode_ = nullptr;  // Clear selection after removal
        drawableTree_->print_tree();   // Print the tree structure after removal
      }
    }
    ImGui::Separator();
    ImGui::Text("Add New DrawableNode");
    static char parentField[128] = "";
    static char drawableName[128] = "";
    static char dataType[128] = "Cube";  // Default data type
    static int dataTypeIndex = 0;        // 0 = Cube, 1 = Sphere, etc.
    ImGui::InputText("Parent Field", parentField, sizeof(parentField));
    ImGui::InputText("Drawable Name", drawableName, sizeof(drawableName));
    // ImGui::InputText("Data Type", dataType, sizeof(dataType));
    ImGui::Combo("Data Type", &dataTypeIndex, "Cube\0Sphere\0PointCloud\0Line\0\0");  // Combo box for data type

    // Update dataType based on selected index
    const char* dataTypeOptions[] = {"Cube", "Sphere", "PointCloud", "Line"};
    if (dataTypeIndex >= 0 && dataTypeIndex < 4) {
      strcpy(dataType, dataTypeOptions[dataTypeIndex]);
    }
    if (ImGui::Button("Add Node")) {
      if (strlen(parentField) > 0 && strlen(drawableName) > 0) {
        std::string inputDrawableName = std::string(parentField) + std::string(drawableName);
        if (dataTypeIndex == 0) {
          auto randomTransform = genRandomTransform();
          auto randomColor = genRandomColor();
          drawableTree_->update_drawable(inputDrawableName, glk::Primitives::cube(), guik::FlatColor(randomColor, randomTransform));
          std::cout << "Adding Cube Node: " << inputDrawableName << " under parent: " << parentField << std::endl;
        } else if (dataTypeIndex == 1) {
          auto randomTransform = genRandomTransform();
          auto randomColor = genRandomColor();
          drawableTree_->update_drawable(inputDrawableName, glk::Primitives::sphere(), guik::FlatColor(randomColor, randomTransform));
          std::cout << "Adding Sphere Node: " << inputDrawableName << " under parent: " << parentField << std::endl;
        } else if (dataTypeIndex == 2) {
          auto randomCloudBuffer = genRandomShperePointCloud(100);
          auto randomColor = genRandomColor();
          drawableTree_->update_drawable(inputDrawableName, randomCloudBuffer, guik::FlatColor(randomColor, Eigen::Matrix4f::Identity()));
          std::cout << "Adding PointCloud Node: " << inputDrawableName << " under parent: " << parentField << std::endl;
        } else if (dataTypeIndex == 3) {
          auto ramdomStart = Eigen::Vector3f::Random() * 3.0f;
          auto ramdomEnd = Eigen::Vector3f::Random() * 3.0f;
          std::vector<Eigen::Vector3f> points = {ramdomStart, ramdomEnd};
          bool linestrip = true;
          auto randomColorStart = genRandomColor();
          auto randomColorEnd = genRandomColor();
          std::vector<Eigen::Vector4f> colors = {randomColorStart, randomColorEnd};
          auto lineBuffer = std::make_shared<glk::ThinLines>(points, colors, linestrip);
          lineBuffer->set_line_width(2.0f);  // Set line width
          drawableTree_->update_drawable(inputDrawableName, lineBuffer, guik::FlatColor(randomColorStart, Eigen::Matrix4f::Identity()));
          std::cout << "Adding Line Node: " << inputDrawableName << " under parent: " << parentField << std::endl;
        }

        addNewTreeNode(parentField, drawableName, dataType);
        // Clear input fields after adding
        parentField[0] = '\0';
        drawableName[0] = '\0';
        dataType[0] = '\0';

        drawableTree_->print_tree();  // Print the tree structure after adding a new node
        viewer->update_drawable("tree", drawableTree_);
      } else {
        std::cerr << "Parent Field and Drawable Name cannot be empty!" << std::endl;
      }
    }
    ImGui::Separator();
    ImGui::Text("Applied Model Matrix");
    // here we only give user to test the x, y, z translation
    static float x = 0.0f, y = 0.0f, z = 0.0f;
    static char parentFieldInput[128] = "";
    ImGui::InputText("Parent Field##modelMatrix", parentFieldInput, sizeof(parentFieldInput));
    ImGui::InputFloat("X", &x);
    ImGui::InputFloat("Y", &y);
    ImGui::InputFloat("Z", &z);

    if (ImGui::Button("Apply Model Matrix")) {
      if (strlen(parentFieldInput) > 0) {
        std::string inputDrawableName = std::string(parentFieldInput);
        auto transform = getTransformFromXYZ(x, y, z);
        drawableTree_->update_setting(inputDrawableName, guik::ShaderSetting().set_model_matrix(transform));
        std::cout << "Applied Model Matrix to: " << inputDrawableName << std::endl;
      } else if (selectedTopicNode_ != nullptr) {
        // If no input, use the selected node's parentField
        std::string inputDrawableName = selectedTopicNode_->parentField + selectedTopicNode_->value;
        auto transform = getTransformFromXYZ(x, y, z);
        drawableTree_->update_setting(inputDrawableName, guik::ShaderSetting().set_model_matrix(transform));
        std::cout << "Applied Model Matrix to: " << inputDrawableName << std::endl;
      } else {
        std::cerr << "Parent Field cannot be empty! Or You need to select a parent field first" << std::endl;
      }
    }

    ImGui::Separator();
    ImGui::Text("Change Color (Mode)");
    const char* colorModes[] = {"Flat", "Rainbow"};
    static int colorModeIndex = 0;  // Default to Flat
    ImGui::Combo("Color Mode", &colorModeIndex, colorModes, IM_ARRAYSIZE(colorModes));
    if (ImGui::Button("Apply Color Mode")) {
      if (selectedTopicNode_ != nullptr) {
        std::string inputDrawableName = selectedTopicNode_->parentField + selectedTopicNode_->value;
        guik::ColorMode::MODE colorMode = (colorModeIndex == 0) ? guik::ColorMode::FLAT_COLOR : guik::ColorMode::RAINBOW;
        drawableTree_->update_tree_color_mode(inputDrawableName, colorMode);
        std::cout << "Applied Color Mode to: " << inputDrawableName << std::endl;
      } else {
        std::cerr << "No node selected to apply color mode!" << std::endl;
      }
    }
    static Eigen::Vector4f color = Eigen::Vector4f::Zero();
    ImGui::ColorEdit4("Color", (float*)&color);

    if (ImGui::Button("Apply Color")) {
      if (selectedTopicNode_ != nullptr) {
        std::string inputDrawableName = selectedTopicNode_->parentField + selectedTopicNode_->value;
        drawableTree_->update_tree_color(inputDrawableName, color);
        std::cout << "Applied Color to: " << inputDrawableName << std::endl;
      } else {
        std::cerr << "No node selected to apply color!" << std::endl;
      }
    }

    ImGui::Separator();
    ImGui::Text("Applied Point Scale");

    static float pointScale = 1.0f;
    ImGui::InputFloat("Point Scale", &pointScale);
    if (ImGui::Button("Apply Point Scale")) {
      if (selectedTopicNode_ != nullptr) {
        std::string inputDrawableName = selectedTopicNode_->parentField + selectedTopicNode_->value;
        drawableTree_->update_tree_setting(inputDrawableName, "point_scale", pointScale);
        std::cout << "Applied Point Scale to: " << inputDrawableName << std::endl;
      } else {
        std::cerr << "No node selected to apply point scale!" << std::endl;
      }
    }

    ImGui::End();
  }

  void createDrawableTreeDisplay(std::shared_ptr<drawableTreeNode> node) {
    if (node->isParent) {
      bool isSelected = (selectedTopicNode_ == node);
      if (ImGui::Selectable((node->value + "##parent").c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns)) {
        selectedTopicNode_ = node;
      }

      // Check if the parent item is being hovered
      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        std::string topicName = node->parentField + node->value;
        ImGui::Text("Topic Name: %s", topicName.c_str());
        ImGui::Text("Data Type: %s", node->dataType.c_str());
        ImGui::EndTooltip();
      }

      if (ImGui::TreeNode(("##" + node->value + "tree").c_str())) {
        for (auto child : node->children) {
          createDrawableTreeDisplay(child.second);
        }
        ImGui::TreePop();
      }
    } else {
      ImGui::Bullet();
      bool isSelected = (selectedTopicNode_ == node);
      if (ImGui::Selectable(node->value.c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns)) {
        selectedTopicNode_ = node;
      }

      // Check if the item is being hovered
      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        std::string topicName = node->parentField + node->value;
        ImGui::Text("Topic Name: %s", topicName.c_str());
        ImGui::Text("Data Type: %s", node->dataType.c_str());
        ImGui::EndTooltip();
      }
    }
  }

  void addNewTreeNode(std::string parentField, std::string drawableName, std::string dataType = "Cube") {
    // given the parentField, split it by /, the pareentField will be like "a/b/c/d/"
    std::vector<std::string> fields;
    // Split the parentField by '/'
    size_t pos = 0;
    while ((pos = parentField.find('/')) != std::string::npos) {
      fields.push_back(parentField.substr(0, pos));
      parentField.erase(0, pos + 1);
    }

    auto curNode = rootTree_;
    std::string curParentField = "";
    for (const auto& field : fields) {
      if (curNode->getChild(field) == nullptr) {
        curNode = curNode->addChild(field, drawableTreeNode(field, curParentField, true, "path"));
      } else {
        curNode = curNode->getChild(field);
      }
      curParentField += field + "/";
    }

    // Add the new drawable node
    curNode->addChild(drawableName, drawableTreeNode(drawableName, curParentField, false, dataType));
    std::cout << "Added drawable node: " << drawableName << " under parent: " << curParentField << std::endl;
  }

  void removeTreeNodeDisplay(std::string parentField, std::string drawableName) {
    std::cout << "Removing drawable node: " << drawableName << " from parent: " << parentField << std::endl;
    // given the parentField, split it by /, the pareentField will be like "a/b/c/d/"
    std::vector<std::string> fields;
    // Split the parentField by '/'
    size_t pos = 0;
    while ((pos = parentField.find('/')) != std::string::npos) {
      fields.push_back(parentField.substr(0, pos));
      parentField.erase(0, pos + 1);
    }

    auto curNode = rootTree_;
    for (const auto& field : fields) {
      if (curNode->getChild(field) == nullptr) {
        std::cerr << "Node not found: " << field << std::endl;
        return;  // Node not found

      } else {
        curNode = curNode->getChild(field);
      }
    }

    // Remove the drawable node
    curNode->children.erase(drawableName);
    std::cout << "Removed drawable node: " << drawableName << std::endl;

    clearEmptyParents(rootTree_);  // Clear empty parents after removal
  }

  void removeTreeNode(std::string parentField, std::string drawableName) {
    std::cout << "Removing drawable node: " << drawableName << " from parent: " << parentField << std::endl;
    // given the parentField, split it by /, the pareentField will be like "a/b/c/d/"
    std::vector<std::string> fields;
    // Split the parentField by '/'
    size_t pos = 0;
    while ((pos = parentField.find('/')) != std::string::npos) {
      fields.push_back(parentField.substr(0, pos));
      parentField.erase(0, pos + 1);
    }

    auto curNode = rootTree_;
    for (const auto& field : fields) {
      if (curNode->getChild(field) == nullptr) {
        std::cerr << "Node not found: " << field << std::endl;
        return;  // Node not found

      } else {
        curNode = curNode->getChild(field);
      }
    }

    // Remove the drawable node
    curNode->children.erase(drawableName);
    std::cout << "Removed drawable node: " << drawableName << std::endl;
  }

  void clearEmptyParents(std::shared_ptr<drawableTreeNode> node) {
    // if node is root, return
    if (node->dataType == "Root") {
      for (auto& child : node->children) {
        clearEmptyParents(child.second);
      }
      for (const auto& child : childrenToRemove) {
        // remove the node from the parent
        std::cout << "Removing child node: " << child.second << " from parent: " << child.first << std::endl;
        removeTreeNode(child.first, child.second);
      }

      return;
    }
    if (!node->isParent) {
      std::cout << "Node is not a parent, cannot clear empty parents: " << node->value << std::endl;
      return;  // Node is not a parent, cannot clear empty parents
    }

    if (node->children.empty()) {
      std::cout << "Node is empty, removing it: " << node->value << std::endl;
      // removeTreeNode(node->parentField, node->value);
      childrenToRemove.push_back({node->parentField, node->value});
      return;  // Node is empty, remove it
    } else {
      // Recursively clear empty parents for children
      std::cout << "Node has children, checking them: " << node->value << std::endl;
      for (auto& child : node->children) {
        clearEmptyParents(child.second);
      }
    }
    std::cout << "After clearing children, checking if node is empty: " << node->value << std::endl;
    for (const auto& child : childrenToRemove) {
      // remove the node from the parent
      std::cout << "Removing child node: " << child.second << " from parent: " << child.first << std::endl;
      removeTreeNode(child.first, child.second);
    }
    if (node->children.empty()) {
      // check if the node is still not empty after clearing children
      std::cout << "Node is empty after clearing children, removing it: " << node->value << std::endl;
      childrenToRemove.push_back({node->parentField, node->value});
    }
  }

  std::shared_ptr<glk::PointCloudBuffer> genRandomShperePointCloud(int num_points) {
    // random select a center of the sphere from -10 to 10
    Eigen::Vector3f center = Eigen::Vector3f::Random() * 5.0f;
    std::vector<Eigen::Vector3f> points;
    points.reserve(num_points);
    for (int i = 0; i < num_points; i++) {
      // generate a random point in the sphere
      Eigen::Vector3f point = Eigen::Vector3f::Random();
      point.normalize();
      point *= static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 5.0f;  // radius of the sphere is 5.0
      point += center;
      points.push_back(point);
    }
    return std::make_shared<glk::PointCloudBuffer>(points);
  }
  Eigen::Vector4f genRandomColor() {
    // Generate a random color in RGBA format
    Eigen::Vector4f randomColor = Eigen::Vector4f::Random();
    randomColor[3] = 1.0f;  // Set alpha to 1.0 for full opacity
    return randomColor;
  }

  Eigen::Matrix4f genRandomTransform() {
    // Generate a random rotation and translation
    Eigen::AngleAxisf rotation(Eigen::Quaternionf::UnitRandom());
    Eigen::Vector3f translation = Eigen::Vector3f::Random() * 5.0f;  // Random translation in [-5, 5]
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = translation;
    return transform;
  }

  Eigen::Matrix4f getTransformFromXYZ(float x, float y, float z) {
    // Create a translation matrix from the given x, y, z coordinates
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    return transform;
  }

  void GenRandomScene() {
    auto viewer = guik::LightViewer::instance();
    std::string randomParentFieldBase = "randomScene" + std::to_string(randomSceneCount_++) + "/";
    // Generate random number of cube, sphere, point cloud, and line nodes, each maxing 3
    int numCubes = rand() % 3 + 1;        // Random number of cubes (1 to 3)
    int numSpheres = rand() % 3 + 1;      // Random number of spheres (1 to 3)
    int numPointClouds = rand() % 3 + 1;  // Random number of point clouds (1 to 3)
    int numLines = rand() % 3 + 1;        // Random number of lines (1 to 3)
    for (int i = 0; i < numCubes; i++) {
      std::string drawableName = "Cube_" + std::to_string(i);
      std::string parentField = randomParentFieldBase + "Cubes/";
      addNewTreeNode(parentField, drawableName, "Cube");
      auto randomTransform = genRandomTransform();
      auto randomColor = genRandomColor();
      drawableTree_->update_drawable(parentField + drawableName, glk::Primitives::cube(), guik::FlatColor(randomColor, randomTransform));
    }
    for (int i = 0; i < numSpheres; i++) {
      std::string drawableName = "Sphere_" + std::to_string(i);
      std::string parentField = randomParentFieldBase + "Spheres/";
      addNewTreeNode(parentField, drawableName, "Sphere");
      auto randomTransform = genRandomTransform();
      auto randomColor = genRandomColor();
      drawableTree_->update_drawable(parentField + drawableName, glk::Primitives::sphere(), guik::FlatColor(randomColor, randomTransform));
    }
    for (int i = 0; i < numPointClouds; i++) {
      std::string drawableName = "PointCloud_" + std::to_string(i);
      std::string parentField = randomParentFieldBase + "PointClouds/";
      addNewTreeNode(parentField, drawableName, "PointCloud");
      auto randomCloudBuffer = genRandomShperePointCloud(100);

      if (i == 0) {
        drawableTree_->update_drawable(parentField + drawableName, randomCloudBuffer, guik::Rainbow());
      } else {
        auto randomColor = genRandomColor();
        drawableTree_->update_drawable(parentField + drawableName, randomCloudBuffer, guik::FlatColor(randomColor, Eigen::Matrix4f::Identity()));
      }
    }
    for (int i = 0; i < numLines; i++) {
      std::string drawableName = "Line_" + std::to_string(i);
      std::string parentField = randomParentFieldBase + "Lines/";
      addNewTreeNode(parentField, drawableName, "Line");
      auto ramdomStart = Eigen::Vector3f::Random() * 3.0f;
      auto ramdomEnd = Eigen::Vector3f::Random() * 3.0f;
      std::vector<Eigen::Vector3f> points = {ramdomStart, ramdomEnd};
      bool linestrip = true;
      auto randomColorStart = genRandomColor();
      auto randomColorEnd = genRandomColor();
      std::vector<Eigen::Vector4f> colors = {randomColorStart, randomColorEnd};
      auto lineBuffer = std::make_shared<glk::ThinLines>(points, colors, linestrip);
      lineBuffer->set_line_width(2.0f);  // Set line width
      drawableTree_->update_drawable(parentField + drawableName, lineBuffer, guik::FlatColor(randomColorStart, Eigen::Matrix4f::Identity()));
    }
    viewer->update_drawable("tree", drawableTree_);
  }

private:
  std::shared_ptr<drawableTreeNode> rootTree_ = std::make_shared<drawableTreeNode>("root", "", true, "Root");
  std::shared_ptr<guik::DrawableTree> drawableTree_ = std::make_shared<guik::DrawableTree>();
  std::shared_ptr<drawableTreeNode> selectedTopicNode_ = nullptr;
  std::vector<std::pair<std::string, std::string>> childrenToRemove;
  int randomSceneCount_ = 0;  // Counter for random scene generation
};

int main(int argc, char** argv) {
  GroupEditor editor;
  return 0;
}