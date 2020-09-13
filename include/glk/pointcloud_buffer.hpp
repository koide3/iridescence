#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawble.hpp>

namespace glk {

class PointCloudBuffer : public glk::Drawable {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<PointCloudBuffer>;

    PointCloudBuffer(int stride, int num_points);
    PointCloudBuffer(const float* data, int stride, int num_points);

    virtual ~PointCloudBuffer() override;

    virtual void draw(glk::GLSLShader& shader) const override;

    GLuint vba_id() const { return vao; }
    GLuint vbo_id() const { return vbo; }

private:
    GLuint vao;
    GLuint vbo;
    int stride;
    int num_points;
};

}

#endif