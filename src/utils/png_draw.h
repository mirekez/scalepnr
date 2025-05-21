#pragma once

#include <iostream>
#include <png.h>

#include <cstdlib>
//#include <cmath>

struct png_draw
{
    int width = 0;
    int height = 0;
    uint8_t *image_data = nullptr;

    void init(int w, int h)
    {
        width = w;
        height = h;
        if (image_data) {
            free(image_data);
        }
        image_data = (uint8_t *) malloc(width * height * 4);
        clear();
    }

    ~png_draw()
    {
        if (image_data) {
            free(image_data);
        }
    }

    void clear()
    {
        memset(image_data, 0, width * height * 4);
    }

    void write(const std::string& name)
    {
        FILE *fp = fopen(name.c_str(), "wb");
        if (!fp) {
            return;
        }

        png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
        png_infop info = png_create_info_struct(png);

        if (!png || !info) {
            fclose(fp);
            return;
        }

        if (setjmp(png_jmpbuf(png))) {
            fclose(fp);
            return;
        }

        png_init_io(png, fp);

        png_set_IHDR(
            png, info, width, height, 8,
            PNG_COLOR_TYPE_RGBA,
            PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_DEFAULT,
            PNG_FILTER_TYPE_DEFAULT
        );

        png_write_info(png, info);

        for (int y = 0; y < height; ++y) {
            png_write_row(png, &image_data[y * width * 4]);
        }

        png_write_end(png, info);
        fclose(fp);
    }

    void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            int index = (y * width + x) * 4;
            image_data[index + 0] = r;
            image_data[index + 1] = g;
            image_data[index + 2] = b;
            image_data[index + 3] = a;
        }
    }

    void draw_space(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a, int size)
    {
        int x1 = x+rand()*size/10/RAND_MAX;
        int y1 = y+rand()*size/10/RAND_MAX;
        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < width) {
            int index = (y1 * width + x1) * 4;
            image_data[index + 0] = r;
            image_data[index + 1] = g;
            image_data[index + 2] = b;
            image_data[index + 3] = a;
        }
    }

    void draw_line(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            set_pixel(x0, y0, r, g, b, a);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }
};

