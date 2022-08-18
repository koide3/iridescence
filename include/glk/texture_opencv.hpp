#ifndef GLK_TEXTURE_OPENCV_HPP
#define GLK_TEXTURE_OPENCV_HPP

#include <glk/texture.hpp>
#include <opencv2/opencv.hpp>

// for old OpenCV
#ifndef CV_16F
#define CV_16F 7
#endif

namespace glk {

static std::shared_ptr<glk::Texture> create_texture(const cv::Mat& image) {
  Eigen::Vector2i size(image.cols, image.rows);

  GLuint format = GL_BGR;
  GLuint type = GL_UNSIGNED_BYTE;

  switch(image.depth()) {
    case CV_8U:
      type = GL_UNSIGNED_BYTE;
      break;
    case CV_16U:
      type = GL_UNSIGNED_SHORT;
      break;
    case CV_8S:
      type = GL_BYTE;
      break;
    case CV_16S:
      type = GL_SHORT;
      break;
    case CV_32S:
      type = GL_INT;
      break;
    case CV_16F:
      type = GL_HALF_FLOAT;
      break;
    case CV_32F:
      type = GL_FLOAT;
      break;
    case CV_64F:
      type = GL_DOUBLE;
      break;
    default:
      std::cerr << "error: unsupported depth " << image.depth() << std::endl;
      break;
  }

  switch(image.channels()) {
    case 1:
      format = GL_RED;
      break;
    case 2:
      format = GL_RG;
      break;
    case 3:
      format = GL_BGR;
      break;
    case 4:
      format = GL_BGRA;
      break;
    default:
      std::cerr << "error: unsupported channels " << image.channels() << std::endl;
      break;
  }

  return std::make_shared<glk::Texture>(size, GL_RGBA, format, type, image.data);
}

}  // namespace glk

#endif