#include "Pixel.h"

Pixel::Pixel(uint32_t _u, uint32_t _v)
{
    u = _u;
    v = _v;
}

Pixel& Pixel::operator+=(const Pixel& other)
{
    u += other.u;
    v += other.v;

    return *this;
}

Pixel& Pixel::operator-=(const Pixel& other)
{
    u -= other.u;
    v -= other.v;

    return *this;
}

bool Pixel::operator==(const Pixel& other) const
{
    return u == other.u && v == other.v;
}