#include <glk/io/jpeg_io.hpp>

#include <cstdio>
#include <iostream>
#include <jpeglib.h>

namespace glk {

bool load_jpeg(const std::string& filename, int& width, int& height, std::vector<unsigned char>& bytes) {
  jpeg_decompress_struct cinfo;

  FILE* infile;
  if ((infile = fopen(filename.c_str(), "rb")) == nullptr) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return false;
  }

  jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);

  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, infile);
  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);

  width = cinfo.output_width;
  height = cinfo.output_height;

  bytes.resize(cinfo.output_width * cinfo.output_height * 4, 255);

  int row_stride = cinfo.output_width * cinfo.output_components;
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, row_stride, 1);

  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, buffer, 1);

    int y = cinfo.output_scanline - 1;
    for (int x = 0; x < cinfo.output_width; x++) {
      for (int i = 0; i < 3; i++) {
        bytes[y * cinfo.output_width * 4 + x * 4 + i] = (*buffer)[x * 3 + i];
      }
    }
  }

  jpeg_finish_decompress(&cinfo);
  fclose(infile);

  return true;
}

bool save_jpeg(const std::string& filename, int width, int height, const std::vector<unsigned char>& bytes, int quality) {
  std::vector<unsigned char> image_buffer(3 * bytes.size() / 4);
  for (int i = 0; i < bytes.size() / 4; i++) {
    image_buffer[i * 3] = bytes[i * 4];
    image_buffer[i * 3 + 1] = bytes[i * 4 + 1];
    image_buffer[i * 3 + 2] = bytes[i * 4 + 2];
  }

  jpeg_compress_struct cinfo;
  jpeg_error_mgr jerr;

  FILE* outfile;
  JSAMPROW row_pointer[1];

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  if ((outfile = fopen(filename.c_str(), "wb")) == nullptr) {
    std::cerr << "error: failed to open " << filename << " to save a jpeg file" << std::endl;
    return false;
  }
  jpeg_stdio_dest(&cinfo, outfile);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);

  jpeg_start_compress(&cinfo, TRUE);
  int row_stride = width * 3;

  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = &image_buffer[cinfo.next_scanline * row_stride];
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  jpeg_finish_compress(&cinfo);
  fclose(outfile);
  jpeg_destroy_compress(&cinfo);

  return true;
}
}  // namespace glk