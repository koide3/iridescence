#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/drawble.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

class PointCloudBuffer : public glk::Drawable {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<PointCloudBuffer>;

    PointCloudBuffer(const std::string& cloud_filename);

    template<typename PointT>
    PointCloudBuffer(const boost::shared_ptr<const pcl::PointCloud<PointT>>& cloud);

    template<typename PointT>
    PointCloudBuffer(const boost::shared_ptr<pcl::PointCloud<PointT>>& cloud);

    virtual ~PointCloudBuffer() override;

    virtual void draw(glk::GLSLShader& shader) const override;

private:
    GLuint vao;
    GLuint vbo;
    int stride;
    int num_points;
};

}

#endif