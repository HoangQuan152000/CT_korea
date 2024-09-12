#pragma once
#include "xvtCV/IInspection.h"
#include "xvtCV/PixelRef.h"
#include "xvtCV/xvtRange.h"

namespace xvt
{
namespace battery
{

struct BeadingItemResult
{
    InspectionResult result;
    double value;
    cv::Point from;
    cv::Point to;
};

class XVT_EXPORTS BatteryInspectorBeadingResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;
    auto CombineAllResult() -> void
    {
        *this &= mCoverHeightResult.result;
        *this &= mInnerDiameterResult.result;
        *this &= mOuterDiameterResult.result;
        *this &= mGrooveDepthResult.result;
        *this &= mGrooveHeightResult.result;
    }

public:
    // Cover Height
    BeadingItemResult mCoverHeightResult;
    // Inner Diameter
    BeadingItemResult mInnerDiameterResult;
    // Max Outer Diameter
    BeadingItemResult mOuterDiameterResult;
    // Groove Depth
    BeadingItemResult mGrooveDepthResult;
    // Groove Height
    BeadingItemResult mGrooveHeightResult;

    cv::Point mBottomLeftContour = cv::Point(-1, -1);
    cv::Point mBottomRightContour = cv::Point(-1, -1);
    cv::Point mGrooveLeftContour = cv::Point(-1, -1);
    cv::Point mGrooveRightContour = cv::Point(-1, -1);
};

class XVT_EXPORTS BatteryInspectorBeading : public PixelRef
{
public:
    explicit BatteryInspectorBeading(double const &pxSize) : PixelRef{pxSize}
    {
    }
    explicit BatteryInspectorBeading(double &&pxSize) = delete;

    auto Inspect(cv::Mat const &src, const std::vector<cv::Point> &contour) const -> BatteryInspectorBeadingResult;

private:
    auto InspectBeading(const cv::Mat &src, BatteryInspectorBeadingResult &BIresult) const -> InspectionResult;

public:
    bool mEnable = true;
    bool mIsCheckCoverHeight = false;
    bool mIsCheckInnerDiameter = false;
    bool mIsCheckOuterDiameter = false;
    bool mIsCheckGrooveHeight = false;
    bool mIsCheckGrooveDepth = false;

    int mBeadingHeightMin = 20;

    // D1 Starting Index[%l]
    float mD1StartPosition = 0.5;

    // Valid COVER_HEIGHT Range[mm]
    Ranged mValidCoverHeightRange = Ranged(0, 10);

    // Valid GROOVE_DEPTH Range[mm]
    Ranged mValidGrooveDepthRange = Ranged(0, 10);

    // Valid GROOVE_HEIGHT Range[mm]
    Ranged mValidGrooveHeightRange = Ranged(0, 10);

    // Valid INNER_DIAMETER Range[mm]
    Ranged mValidInnerDiameterRange = Ranged(0, 10);
};

}; // namespace battery
}; // namespace xvt
