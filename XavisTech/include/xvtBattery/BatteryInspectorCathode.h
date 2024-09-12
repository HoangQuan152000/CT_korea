#pragma once
#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryInspectorPole.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/PixelRef.h"
#include "xvtCV/Peak.h"
#include "xvtCV/xvtRange.h"
#include <memory>

namespace xvt
{
namespace battery
{

class XVT_EXPORTS BatteryInspectorCathodeResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawOKResult = false) const -> cv::Point override;

    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

public:
    VecPoint mContour;
};

class BatteryInspectorCathode : public PixelRef
{
public:
    explicit BatteryInspectorCathode(double const &pxSize, BatteryInspectorPole const &pole, BatteryInspectorJR const &jr)
        : PixelRef{pxSize}, mIspPole{std::cref(pole)}, mIspJR{std::cref(jr)}
    {
    }

    explicit BatteryInspectorCathode(double &&pxSize) = delete;

    auto Inspect(cv::Mat const &src, cv::Rect roi, std::vector<PoleInfo> &poles) const -> BatteryInspectorCathodeResult;

    auto FindCathodeLine(const cv::Mat &srcImg, VecPoint &lstCathodeLine) const -> InspectionResult;

    auto FindCathodeLineByAStar(const cv::Mat &srcImg, VecPoint &lstCathodeLine, int centerNeglectionWidth = 0, int region = 10) const
        -> InspectionResult;

    auto FindCathodeLineByPeak(const cv::Mat &srcImg, VecDouble &lstCathodeLine) const -> InspectionResult;

    auto FindCathodeXPosition(const cv::Mat &src,
                              VecPoint &vtLine,
                              VecPoint &vtContour,
                              std::vector<PoleInfo> &poles) const -> InspectionResult;

    auto GetJR() const -> BatteryInspectorJR
    {
        return mIspJR.get();
    }
    auto GetPole() const -> BatteryInspectorPole
    {
        return mIspPole.get();
    }

private:
    auto GetPoleInfo(const VecPoint &vtLine, VecPeak &peaks, std::vector<PoleInfo> &poles, Rangei &disRange, int cols) const
        -> InspectionResult;

public:
    bool mEnable = true;
    bool mEnableAutoMode = true;
    bool mIsApplyEnhanced = true;

    // Settings for finding Cathode line (Positive pole wave).
    // Cathode Line Inner Threshold
    double mCathodeLineThresholdInner = 0;

    // Cathode Line Middle Threshold
    double mCathodeLineThresholdMiddle = 0;

    // Cathode Line Outer Threshold
    double mCathodeLineThresholdOuter = 0;

    // Cathode Line Window Size[pixel]
    int mCathodeLineWindowSize = 20;

private:
    std::reference_wrapper<const BatteryInspectorJR> mIspJR;
    std::reference_wrapper<const BatteryInspectorPole> mIspPole;
};

}; // namespace battery
}; // namespace xvt
