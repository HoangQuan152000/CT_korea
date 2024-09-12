#pragma once
#include <opencv2/core/types.hpp>
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include <xvtCV/xvtPen.h>
#include <xvtCV/IInspection.h>
#include <string>
#include <vector>
#include <iostream>
#include <numeric>

namespace xvt {

class RoiInfo;
class IntensityAnalyzerResult;

XVT_EXPORTS
// just support uchar becareful when using it
auto GetIntensity(const cv::Mat1b& image, const std::vector<cv::Point2f>const& points)->std::vector<uchar>;

XVT_EXPORTS
// just support 16bit unsign becareful when using it
auto GetIntensity(const cv::Mat1w& image, const std::vector<cv::Point2f>const& points)->std::vector<ushort>;

XVT_EXPORTS
auto GetPoints(cv::Mat const& img, cv::Point2f start, cv::Point2f end, float radiusStep = 1.f)->std::vector<cv::Point2f>;

template<typename T>
auto GetVariance(const std::vector<T>& data)->double;

class XVT_EXPORTS IntensityAnalyzer 
{
public:
    auto Inspect(cv::Mat const& inImg) const->IntensityAnalyzerResult;
    // Return true if load successfully
    auto Load(const std::wstring& settingPath)  &-> bool;

public:
    bool mEnable = true;
    VecRect mRoiList;
    bool mIsUse8Bit = true;
};

struct XVT_EXPORTS RoiInfo
{
    cv::Rect mSetRoi{};
    cv::Rect mUseRoi{};
    double mAvg=0.0;
    double mMin=0.0;
    double mMax=0.0;
    double mStd=0.0;

    auto GetCSVData(VecKeyValueStr& out, std::string prefix = "", bool isRecursive = true)const->void;
};

class XVT_EXPORTS IntensityAnalyzerResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    virtual auto GetCSVData(VecKeyValueStr& out, std::string prefix = "", bool isRecursive = true)const->void override;

    virtual auto GetResultStr() const->std::string override;

public:
    std::vector<RoiInfo> mRois;
};

template<typename T>
auto GetVariance(const std::vector<T>& data)->double
{
    double variance = 0.0;
    if (data.empty()) return variance;

    double mean = 0.0;
    mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    for (auto const& value : data)
    {
        auto error = value - mean;
        variance += error * error;
    }

    variance /= data.size();
    return variance;
}
}
