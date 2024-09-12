#pragma once
#include "xvtCV/IInspection.h"
#include "xvtCV/PixelRef.h"

namespace xvt
{
namespace battery
{

class XVT_EXPORTS BatteryInspectorJRResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawOKResult = false) const -> cv::Point override;

public:
    cv::Rect mPoleRoi = cv::Rect(0, 0, 0, 0);
    cv::Point mLeftLeaningPos = cv::Point(0, 0);
    cv::Point mRightLeaningPos = cv::Point(0, 0);
    cv::Point mTopContourPos = cv::Point(0, 0);
    cv::Point mBottomContourPos = cv::Point(0, 0);
    cv::Point mCenterContourPos = cv::Point(0, 0);
};

class XVT_EXPORTS BatteryInspectorJR : public PixelRef
{
public:
    explicit BatteryInspectorJR(double const &pxSize) : PixelRef{pxSize}
    {
    }
    explicit BatteryInspectorJR(double &&pxSize) = delete;

    auto Inspect(cv::Mat const &src, cv::Rect roi) const -> BatteryInspectorJRResult;

    auto FindTopReferenceLine(const cv::Mat &image, cv::Point &center, cv::Point offset) const -> InspectionResult;
    auto FindLeaning(const cv::Mat &image, cv::Point offset) const -> InspectionResult;

public:
    bool mEnable = true;
    bool mEnableCheckLeaning = true;
    bool mEnableCheckNumberOfPoles = true;
    bool mEnableCheckLeaningAuto = false;
    bool mEnableCenterNeglection = true;
    bool mEnableCheckTopReferenceLine = true;

    int mCaseLineOffset = 20;
    // Neglected Center Area Width[pixel]
    int mCenterNeglectionWidth = 200;

    // Neglected Center Area Width[pixel]
    int mNoRegion = 10;

    // JR ROi Height
    int mHeight = 200;

    // Offset x left and right side JR Roi
    int mJROffsetX = 20;

    // Offset y JR Roi
    int mJROffsetY = 20;

    // Threshold find leaning
    int mLeaningThreshold = 0;

    // Number of pole in one side
    int mOneSidePoleNumber = 24;

    double mMinLeaningDistance = 0.0;
};
}; // namespace battery
}; // namespace xvt
