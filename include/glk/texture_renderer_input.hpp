#ifndef GLK_TEXTURE_RENDERER_INPUT_HPP
#define GLK_TEXTURE_RENDERER_INPUT_HPP

#include <array>
#include <optional>
#include <unordered_map>

#include <Eigen/Core>

namespace glk {

struct TextureRendererInput {
  using Ptr = std::shared_ptr<TextureRendererInput>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TextureRendererInput() {}
  ~TextureRendererInput() {}

  template<typename T>
  void set(const std::string& name, const T& value);

  template<typename T>
  std::optional<T> get(const std::string& name) const;

private:
  std::unordered_map<std::string, unsigned int> params_int;
  std::unordered_map<std::string, std::array<float, 16>> params_16f;
};

template<>
inline void TextureRendererInput::set(const std::string& name, const Eigen::Matrix4f& value) {
  std::copy(value.data(), value.data() + 16, params_16f[name].begin());
}

template <>
inline std::optional<Eigen::Matrix4f> TextureRendererInput::get(const std::string& name) const {
  auto found = params_16f.find(name);
  if(found == params_16f.end()) {
    return std::nullopt;
  }

  return Eigen::Map<const Eigen::Matrix4f>(found->second.data()).eval();
}

template<>
inline void TextureRendererInput::set(const std::string& name, const unsigned int& value) {
  params_int[name] = value;
}

template<>
inline std::optional<unsigned int> TextureRendererInput::get(const std::string& name) const {
  auto found = params_int.find(name);
  if(found == params_int.end()) {
    return std::nullopt;
  }

  return found->second;
}
}  // namespace glk

#endif