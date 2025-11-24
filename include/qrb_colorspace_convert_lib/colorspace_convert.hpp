// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_COLORSPACE_CONVERT_LIB__COLORSPACE_CONVERT_HPP_
#define QRB_COLORSPACE_CONVERT_LIB__COLORSPACE_CONVERT_HPP_

#ifdef USE_OPENCV_BACKEND
#include <memory>
#include <opencv4/opencv2/opencv.hpp>

#include "lib_mem_dmabuf/dmabuf.hpp"
#else
#include <drm/drm_fourcc.h>

#include "qrb_colorspace_convert_lib/opengles_common.hpp"
#endif

namespace qrb::colorspace_convert_lib
{

class ConvertAccelerator
{
public:
  ~ConvertAccelerator();

  // Always declare both sets of methods, but only implement the active backend
#ifdef USE_OPENCV_BACKEND
  /// Convert NV12 to RGB8 using OpenCV
  /// @param in_buf input DmaBuffer
  /// @param out_buf output DmaBuffer
  /// @param width image width
  /// @param height image height
  /// @param stride aligned width for memory layout
  /// @param slice aligned height for memory layout
  bool nv12_to_rgb8_opencv(const std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & in_buf,
      const std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & out_buf,
      int width,
      int height,
      int stride,
      int slice);

  /// Convert RGB8 to NV12 using OpenCV
  /// @param in_buf input DmaBuffer
  /// @param out_buf output DmaBuffer
  /// @param width image width
  /// @param height image height
  /// @param stride aligned width for memory layout
  /// @param slice aligned height for memory layout
  bool rgb8_to_nv12_opencv(const std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & in_buf,
      const std::shared_ptr<lib_mem_dmabuf::DmaBuffer> & out_buf,
      int width,
      int height,
      int stride,
      int slice);
#else
  /// Convert NV12 to RGB8 using OpenGL ES
  /// @param in_fd input DMA_BUF file descriptor
  /// @param out_fd output DMA_BUF file descriptor
  /// @param width image width, need align with GPU supported size
  /// @param height image height, need align with GPU supported size
  bool nv12_to_rgb8_opengles(int in_fd, int out_fd, int width, int height);

  /// Convert RGB8 to NV12 using OpenGL ES
  /// @param in_fd input DMA_BUF file descriptor
  /// @param out_fd output DMA_BUF file descriptor
  /// @param width image width, need align with GPU supported size
  /// @param height image height, need align with GPU supported size
  bool rgb8_to_nv12_opengles(int in_fd, int out_fd, int width, int height);
#endif

private:
#ifndef USE_OPENCV_BACKEND
  bool egl_init();
  bool egl_deinit();

  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;
  bool initialized_ = false;
#endif
};

}  // namespace qrb::colorspace_convert_lib

#endif  // QRB_COLORSPACE_CONVERT_LIB__COLORSPACE_CONVERT_HPP_
