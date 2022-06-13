#include "Image.h"

Image::Image(std::string _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool _bigEndian)
    : data(std::vector<uint8_t>(_data.begin(), _data.end())), width(_width), height(_height), pixelStride(_pixelStride), bigEndian(_bigEndian) {}

Image::Image(std::vector<uint8_t> _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool _bigEndian)
    : data(_data), width(_width), height(_height), pixelStride(_pixelStride), bigEndian(_bigEndian) {}

uint32_t Image::valueAt(uint32_t u, uint32_t v) const
{
    // Calculate the index where the value for the given pixel can be found in the data array.
    uint32_t index = (u + v * width) * pixelStride;
    
    // Check whether the data is stored in big or little endian order.
    if (bigEndian)
    {
        // Data is stored in big endian order, i.e. the smallest index will contain the highest byte.
        // Calculate the pixel value accordingly (depending on the amount of bytes per pixel) and return the result.
        if (pixelStride == 1)
            return data[index];
        else if (pixelStride == 2)
            return (data[index] << 8) | data[index + 1];
        else if (pixelStride == 3)
            return (data[index] << 16) | (data[index + 1] << 8) | data[index + 2];
        else
            return (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    }
    else
    {
        // Data is stored in little endian order, i.e. the highest index will contain the highest byte.
        // Calculate the pixel value accordingly (depending on the amount of bytes per pixel) and return the result.
        if (pixelStride == 1)
            return data[index];
        else if (pixelStride == 2)
            return data[index] | (data[index + 1] << 8);
        else if (pixelStride == 3)
            return data[index] | (data[index + 1] << 8) | (data[index + 2] << 16);
        else
            return data[index] | (data[index + 1] << 8) | (data[index + 2] << 16) | (data[index + 3] << 24);
    }
}

uint32_t Image::valueAt(const Pixel& p) const
{
    return valueAt(p.u, p.v);
}

void Image::setValueAt(uint32_t u, uint32_t v, uint32_t value)
{
    // Calculate the index where the value for the given pixel should be stored in the data array.
    uint32_t index = (u + v * width) * pixelStride;

    if (bigEndian)
    {
        // Data is stored in big endian order, i.e. the smallest index will contain the highest byte.
        // Store the pixel value accordingly (depending on the amount of bytes per pixel).
        if (pixelStride == 1)
        {
            data[index] = static_cast<uint8_t>(value);
        }
        else if (pixelStride == 2)
        {
            data[index] = static_cast<uint8_t>(value >> 8);
            data[index + 1] = static_cast<uint8_t>(value);
        }
        else if (pixelStride == 3)
        {
            data[index] = static_cast<uint8_t>(value >> 16);
            data[index + 1] = static_cast<uint8_t>(value >> 8);
            data[index + 2] = static_cast<uint8_t>(value);
        }
        else
        {
            data[index] = static_cast<uint8_t>(value >> 24);
            data[index + 1] = static_cast<uint8_t>(value >> 16);
            data[index + 2] = static_cast<uint8_t>(value >> 8);
            data[index + 3] = static_cast<uint8_t>(value);
        }
    }
    else
    {
        // Data is stored in little endian order, i.e. the highest index will contain the highest byte.
        // Store the pixel value accordingly (depending on the amount of bytes per pixel).
        if (pixelStride == 1)
        {
            data[index] = static_cast<uint8_t>(value);
        }
        else if (pixelStride == 2)
        {
            data[index] = static_cast<uint8_t>(value);
            data[index + 1] = static_cast<uint8_t>(value >> 8);
        }
        else if (pixelStride == 3)
        {
            data[index] = static_cast<uint8_t>(value);
            data[index + 1] = static_cast<uint8_t>(value >> 8);
            data[index + 2] = static_cast<uint8_t>(value >> 16);
        }
        else
        {
            data[index] = static_cast<uint8_t>(value);
            data[index + 1] = static_cast<uint8_t>(value >> 8);
            data[index + 2] = static_cast<uint8_t>(value >> 16);
            data[index + 3] = static_cast<uint8_t>(value >> 24);
        }
    }
}

void Image::setValueAt(const Pixel& p, uint32_t value)
{
    setValueAt(p.u, p.v, value);
}

bool Image::hasValueForPixel(uint32_t u, uint32_t v) const
{
    return u < width && v < height;
}

bool Image::hasValueForPixel(const Pixel& p) const
{
    return hasValueForPixel(p.u, p.v);
}
