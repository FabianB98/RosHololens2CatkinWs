#include <string>
#include <vector>
#include <stdint.h>

class DepthMap
{
public:
    DepthMap(std::string _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool bigEndian);
    DepthMap(std::vector<uint8_t> _data, uint32_t _width, uint32_t _height, uint32_t _pixelStride, bool bigEndian);

    uint32_t valueAt(uint32_t u, uint32_t v) const;
    void setValueAt(uint32_t u, uint32_t v, uint32_t value);

public:
    const uint32_t width;
    const uint32_t height;
    const uint32_t pixelStride;
    const bool bigEndian;

private:
    std::vector<uint8_t> data;
};
