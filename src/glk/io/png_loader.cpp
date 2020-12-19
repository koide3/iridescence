#include <glk/io/png_loader.hpp>

#include <iostream>
#include <libpng/png.h>

namespace glk {

PNGLoader::PNGLoader() {}

PNGLoader::~PNGLoader() {}

bool PNGLoader::load(const std::string& filename) {
  width = height = 0;

  FILE* fp = fopen(filename.c_str(), "rb");
  if(fp == nullptr) {
    std::cerr << "failed to open " << filename << std::endl;
    return false;
  }

  if(!load(fp)) {
    std::cerr << "failed to load " << filename << std::endl;
    return false;
  }

  fclose(fp);
  return true;
}

bool PNGLoader::load(FILE* fp) {
  png_colorp palette;

  png_byte sig_bytes[8];
  if(fread(sig_bytes, sizeof(sig_bytes), 1, fp) != 1 || png_sig_cmp(sig_bytes, 0, sizeof(sig_bytes))) {
    return false;
  }

  png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if(png == nullptr) {
    png_destroy_read_struct(&png, nullptr, nullptr);
    return false;
  }

  png_infop info = png_create_info_struct(png);
  if(info == nullptr || setjmp(png_jmpbuf(png))) {
    png_destroy_read_struct(&png, &info, nullptr);
    return false;
  }

  png_init_io(png, fp);
  png_set_sig_bytes(png, sizeof(sig_bytes));
  png_read_png(png, info, PNG_TRANSFORM_PACKING | PNG_TRANSFORM_STRIP_16, nullptr);
  width = png_get_image_width(png, info);
  height = png_get_image_height(png, info);

  png_bytepp rows = png_get_rows(png, info);

  switch(png_get_color_type(png, info)) {
    case PNG_COLOR_TYPE_PALETTE:
      png_destroy_read_struct(&png, &info, nullptr);
      std::cerr << "PNG_COLOR_TYPE_PALETTE not supported" << std::endl;
      return false;

    case PNG_COLOR_TYPE_GRAY:
      bytes.resize(width * height * 4);
      for(int y = 0; y < height; y++) {
        png_bytep row = rows[y];
        for(int x = 0; x < width; x++) {
          bytes[(y * width + x) * 4] = *row;
          bytes[(y * width + x) * 4 + 1] = *row;
          bytes[(y * width + x) * 4 + 2] = *row++;
          bytes[(y * width + x) * 4 + 3] = 255;
        }
      }
      break;

    case PNG_COLOR_TYPE_GRAY_ALPHA:
      bytes.resize(width * height * 4);
      for(int y = 0; y < height; y++) {
        png_bytep row = rows[y];
        for(int x = 0; x < width; x++) {
          bytes[(y * width + x) * 4] = *row;
          bytes[(y * width + x) * 4 + 1] = *row;
          bytes[(y * width + x) * 4 + 2] = *row++;
          bytes[(y * width + x) * 4 + 3] = *row++;
        }
      }
      break;

    case PNG_COLOR_TYPE_RGB:
      bytes.resize(width * height * 4);
      for(int y = 0; y < height; y++) {
        png_bytep row = rows[y];
        for(int x = 0; x < width; x++) {
          bytes[(y * width + x) * 4] = *row++;
          bytes[(y * width + x) * 4 + 1] = *row++;
          bytes[(y * width + x) * 4 + 2] = *row++;
          bytes[(y * width + x) * 4 + 3] = 255;
        }
      }
      break;

    case PNG_COLOR_TYPE_RGB_ALPHA:
      bytes.resize(width * height * 4);
      for(int y = 0; y < height; y++) {
        png_bytep row = rows[y];
        for(int x = 0; x < width; x++) {
          bytes[(y * width + x) * 4] = *row++;
          bytes[(y * width + x) * 4 + 1] = *row++;
          bytes[(y * width + x) * 4 + 2] = *row++;
          bytes[(y * width + x) * 4 + 3] = *row++;
        }
      }
      break;
  }

  png_destroy_read_struct(&png, &info, nullptr);

  return true;
}
}  // namespace glk