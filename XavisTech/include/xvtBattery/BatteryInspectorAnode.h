#pragma once
#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryInspectorPole.h"
#include "xvtBattery/PoleInfo.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/PixelRef.h"
#include "xvtCV/xvtRange.h"
#include <memory>

namespace xvt {
namespace battery {

class XVT_EXPORTS BatteryInspectorAnodeResult : public InspectionResult
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
};

class BatteryInspectorAnode : public PixelRef
{
public:
    explicit BatteryInspectorAnode(double const &pxSize, BatteryInspectorPole const &pole, BatteryInspectorJR const &jr)
        : PixelRef{pxSize}, mIspPole{std::cref(pole)}, mIspJR{std::cref(jr)}
    {
    }
    explicit BatteryInspectorAnode(double &&pxSize) = delete;

    auto Inspect(cv::Mat const &src, cv::Rect roi, std::vector<PoleInfo> &poles) const -> BatteryInspectorAnodeResult;

    auto FindAnode(const cv::Mat &inputImg, std::vector<PoleInfo> &poles) const -> InspectionResult;

    auto FindAnodeByLineTracing(const cv::Mat &inputImg,
                                int &anode,
                                cv::Point startPoint,
                                int defaultVal,
                                int stepHorizontal,
                                int stepVertical,
                                float breakThreshold,
                                int borderCheck,
                                int borderDistance,
                                bool restrictMove,
                                int moveAllow) const -> InspectionResult;

    auto FindAnodeAutoByEdge(const cv::Mat &inputImg,
                             int &anode,
                             cv::Point startPoint,
                             int defaultVal,
                             int stepHorizontal,
                             int stepVertical,
                             VecPoint &vtPoint,
                             cv::Point &ending) const -> InspectionResult;

    auto IsPoleAnode(const cv::Mat& inputImg, cv::Point polePos, int &endPos, int stepHorizontal, int stepVerticals) const -> InspectionResult;

    auto GetJR() const -> BatteryInspectorJR
    {
        return mIspJR.get();
    }

    auto GetPole() const -> BatteryInspectorPole
    {
        return mIspPole.get();
    }

public:
    bool mEnable = true;
    bool mEnableAutoMode = true;
    bool mIsApplyEnhanced = true;

    // Anode Threshold for Endpoint Poles
    double mEndPointPosition = 10;

    // Anode Threshold for Inner Poles
    double mAnodeThresholdInner = 0;

    // Anode Threshold for Middle Poles
    double mAnodeThresholdMiddle = 0;

    // Anode Threshold for Outer Poles
    double mAnodeThresholdOuter = 0;

private:
    std::reference_wrapper<const BatteryInspectorJR> mIspJR;
    std::reference_wrapper<const BatteryInspectorPole> mIspPole;
};

}; // namespace battery
}; // namespace xvt
