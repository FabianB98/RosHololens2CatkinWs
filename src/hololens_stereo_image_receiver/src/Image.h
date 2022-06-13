#pragma once

#include <string>
#include <vector>
#include <stdint.h>

#include "Pixel.h"

class Image
{
public:
    Image(std::string _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool bigEndian);
    Image(std::vector<uint8_t> _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool bigEndian);

    uint32_t valueAt(uint32_t u, uint32_t v) const;
    uint32_t valueAt(const Pixel& p) const;
    void setValueAt(uint32_t u, uint32_t v, uint32_t value);
    void setValueAt(const Pixel& p, uint32_t value);

    bool hasValueForPixel(uint32_t u, uint32_t v) const;
    bool hasValueForPixel(const Pixel& p) const;

public:
    const uint32_t width;
    const uint32_t height;
    const uint32_t pixelStride;
    const bool bigEndian;

private:
    std::vector<uint8_t> data;
};
