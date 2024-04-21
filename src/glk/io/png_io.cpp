#include <glk/io/png_io.hpp>

#include <iostream>
#include <png.h>
#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

namespace {
bool load_png(FILE* fp, int& width, int& height, std::vector<unsigned char>& bytes) {
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
      std::cerr << bold_red << "PNG_COLOR_TYPE_PALETTE not supported" << reset << std::endl;
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
}  // namespace

bool load_png(const std::string& filename, int& width, int& height, std::vector<unsigned char>& bytes) {
  FILE* fp = fopen(filename.c_str(), "rb");
  if(fp == nullptr) {
    std::cerr << bold_red << "failed to open " << filename << reset << std::endl;
    return false;
  }

  if(!load_png(fp, width, height, bytes)) {
    std::cerr << bold_red << "failed to load " << filename << reset << std::endl;
    fclose(fp);
    return false;
  }

  fclose(fp);
  return true;
}

bool save_png(const std::string& filename, int width, int height, const std::vector<unsigned char>& bytes) {
  FILE* fp = fopen(filename.c_str(), "wb");
  if(fp == nullptr) {
    std::cerr << bold_red << "failed to open " << filename << reset << std::endl;
    return false;
  }

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if(png == nullptr) {
    std::cerr << bold_red << "failed to create png write struct" << reset << std::endl;
    return false;
  }

  png_infop info = png_create_info_struct(png);
  if(info == nullptr) {
    std::cerr << bold_red << "failed to create png info struct" << reset << std::endl;
    return false;
  }

  if(setjmp(png_jmpbuf(png))) {
    std::cerr << bold_red << "failed to setjmp" << reset << std::endl;
    return false;
  }

  png_init_io(png, fp);

  png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  png_write_info(png, info);

  std::vector<png_bytep> rows(height);
  for(int y = 0; y < height; y++) {
    rows[y] = (png_bytep)(bytes.data() + (y * width * 4));
  }

  png_write_image(png, rows.data());
  png_write_end(png, nullptr);

  png_destroy_write_struct(&png, &info);

  fclose(fp);
  return true;
}

}  // namespace glk
