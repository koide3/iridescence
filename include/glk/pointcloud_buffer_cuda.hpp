#ifndef GLK_POINTCLOUD_BUFFER_CUDA_HPP
#define GLK_POINTCLOUD_BUFFER_CUDA_HPP

#include <iostream>
#include <Eigen/Core>

#include <cuda_gl_interop.h>
#include <thrust/device_vector.h>
#include <glk/pointcloud_buffer.hpp>

namespace glk {

static std::shared_ptr<PointCloudBuffer> create_point_cloud_buffer(const void* src_cuda_ptr, int stride, int num_points) {
  auto buffer = std::make_shared<PointCloudBuffer>(stride, num_points);

  const auto error_check = [](cudaError_t error) {
    if(error != cudaSuccess) {
      // std::cerr << "error : " << cudaGetErrorName(error) << std::endl;
      // std::cerr << "      : " << cudaGetErrorString(error) << std::endl;
    }
  };

  cudaGraphicsResource* vbo_resource = 0;
  error_check(cudaGraphicsGLRegisterBuffer(&vbo_resource, buffer->vbo_id(), cudaGraphicsRegisterFlagsNone));

  float* dst_gl_ptr = nullptr;
  size_t dst_size = 0;

  error_check(cudaGraphicsMapResources(1, &vbo_resource));
  error_check(cudaGraphicsResourceGetMappedPointer((void**)&dst_gl_ptr, &dst_size, vbo_resource));

  error_check(cudaMemcpy(dst_gl_ptr, src_cuda_ptr, stride * num_points, cudaMemcpyDeviceToDevice));

  error_check(cudaGraphicsUnmapResources(1, &vbo_resource));
  error_check(cudaGraphicsUnregisterResource(vbo_resource));

  return buffer;
}

static std::shared_ptr<PointCloudBuffer> create_point_cloud_buffer(const thrust::device_vector<Eigen::Vector3f>& points) {
  return create_point_cloud_buffer(thrust::raw_pointer_cast(points.data()), sizeof(Eigen::Vector3f), points.size());
}
}

#endif