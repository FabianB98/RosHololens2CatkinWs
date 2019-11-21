#include "DepthMap.h"

DepthMap::DepthMap(std::string _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool _bigEndian)
    : data(std::vector<uint8_t>(_data.begin(), _data.end())), width(_width), height(_height), pixelStride(_pixelStride), bigEndian(_bigEndian) {}

DepthMap::DepthMap(std::vector<uint8_t> _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool _bigEndian)
    : data(_data), width(_width), height(_height), pixelStride(_pixelStride), bigEndian(_bigEndian) {}

uint32_t DepthMap::valueAt(uint32_t u, uint32_t v) const
{
    uint32_t index = (u + v * width) * pixelStride;
    
    if (bigEndian)
    {
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
