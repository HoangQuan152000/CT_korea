#pragma once
#include "xvtCV/xvtDefine.h"
#include <cassert>
#include <functional>

namespace xvt {

constexpr double Pixel2MM(double px, double pixelSize)
{
    return px * pixelSize;
}

constexpr double MM2Pixel(double mm, double pixelSize)
{
    assert(pixelSize != 0);
    return mm / pixelSize;
}

//Class that support reference to pixel value
class XVT_EXPORTS PixelRef
{
public:
    explicit PixelRef(double const& vl) : mPixelSize{ std::cref(vl) } {}
    explicit PixelRef(double&& vl) = delete;
    //Convert from pixel to milimeter
    auto ToMilimet(double const& px) const -> double { return px * mPixelSize.get(); }
    //Convert from milimeter to pixel
    auto ToPixel(double const& mm) const -> double { return mm / mPixelSize.get(); }
    auto GetPixelSize() const -> double { return mPixelSize.get(); }
public:
    //Pixel size in mm.
    std::reference_wrapper<const double> mPixelSize;
};

}
