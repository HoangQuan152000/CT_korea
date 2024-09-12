#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>
#include <functional>

namespace xvt {
namespace enhance {
class XVT_EXPORTS GammaCorrector
{
public:
    GammaCorrector(float g = 1);
    void SetGamma(float g)&;

    float GetGamma() const&
    {
        return mGamma;
    }

    void Apply(cv::Mat const& src, cv::Mat& dst) const&;

    std::vector<uchar>const& GetLUT() const& { return mLUT; }
    std::vector<uchar> GetLUT()&& { return std::move(mLUT); }

    void SetGammaFunction(std::function<uchar(int, float)> f)& {if(f) mGammaFunc = f; };

    static uchar GammaCorrectFuncOpencv(int i, float g);
    static uchar GammaCorrectFuncXavis(int i, float g);
    static uchar SigmoidCorrectFunc(int i, float g);
protected:
    void CalLUT()&;

private:
    std::function<uchar(int, float)> mGammaFunc;
    bool mIsLUTCalculated = false;
    float mGamma = 1.0f;
    std::vector<uchar> mLUT;
};
}
}