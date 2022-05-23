#pragma once

#include <stdint.h>
#include <unordered_map>

class Pixel
{
public:
    Pixel(uint32_t _u, uint32_t _v);
    Pixel(const Pixel&) = default;
    Pixel(Pixel&&) = default;

    Pixel& operator+=(const Pixel& other);
    Pixel& operator-=(const Pixel& other);

    bool operator==(const Pixel& other) const;

    uint32_t u;
    uint32_t v;
};

inline Pixel operator+(Pixel lhs, const Pixel& rhs)
{
    // Blatantly copied from https://stackoverflow.com/a/4421719
    lhs += rhs;
    return lhs;
}

inline Pixel operator-(Pixel lhs, const Pixel& rhs)
{
    // Blatantly copied (and slightly modified) from https://stackoverflow.com/a/4421719
    lhs -= rhs;
    return lhs;
}

namespace std {
    template <>
    struct hash<Pixel>
    {
        size_t operator()(const Pixel& pixel) const
        {
            size_t res = 17;
            res = res * 31 + hash<uint32_t>()(pixel.u);
            res = res * 31 + hash<uint32_t>()(pixel.v);
            return res;
        }
    };
}