#pragma once
#include "xvtBattery/BatteryInspectorAnode.h"
#include "xvtBattery/BatteryInspectorBase.h"
#include "xvtBattery/BatteryInspectorBeading.h"
#include "xvtBattery/BatteryInspectorCathode.h"
#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryInspectorPole.h"
//#include "IInspection.h"
//#include "PixelRef.h"
//#include "xvt_Range.h"

namespace xvt
{
namespace battery
{

class XVT_EXPORTS CylinderBatteryUpperResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

    void SetBaseResult(BatteryInspectorBaseResult &&result)
    {
        CombineResult(result.GetResult());
        mBaseResult = result;
    }

    void SetBeadingResult(BatteryInspectorBeadingResult &&result)
    {
        CombineResult(result.GetResult());
        mBeadingResult = result;
    }

    void SetJRResult(BatteryInspectorJRResult &&result)
    {
        CombineResult(result.GetResult());
        mJRResult = result;
    }

    void SetCathodeResult(BatteryInspectorCathodeResult &&result)
    {
        CombineResult(result.GetResult());
        mCathodeResult = result;
    }

    void SetAnodeResult(BatteryInspectorAnodeResult &&result)
    {
        CombineResult(result.GetResult());
        mAnodeResult = result;
    }

public:
    BatteryInspectorBaseResult mBaseResult;
    BatteryInspectorBeadingResult mBeadingResult;
    BatteryInspectorJRResult mJRResult;
    BatteryInspectorCathodeResult mCathodeResult;
    BatteryInspectorAnodeResult mAnodeResult;
    PoleResult mPoleResult;
};

class XVT_EXPORTS CylinderInspectorUpper
{
public:
    auto Inspect(cv::Mat const &src) const -> CylinderBatteryUpperResult;

public:
    //Size of one pixel in mm
    double mPixelSize = 1.0;

    BatteryInspectorBase mIspBase = BatteryInspectorBase(mPixelSize);
    BatteryInspectorBeading mIspBeading = BatteryInspectorBeading(mPixelSize);
    BatteryInspectorJR mIspJR = BatteryInspectorJR(mPixelSize);
    BatteryInspectorPole mIspPole = BatteryInspectorPole(mPixelSize);
    BatteryInspectorCathode mIspCathode = BatteryInspectorCathode(mPixelSize, mIspPole, mIspJR);
    BatteryInspectorAnode mIspAnode = BatteryInspectorAnode(mPixelSize, mIspPole, mIspJR);
};

}; // namespace battery
}; // namespace xvt
