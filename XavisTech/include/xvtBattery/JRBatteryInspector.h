#pragma once
#include "xvtBattery/BatteryInspectorAnode.h"
#include "xvtBattery/BatteryInspectorBase.h"
#include "xvtBattery/BatteryInspectorCathode.h"
#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryInspectorPole.h"

namespace xvt
{
    enum FindStartAnode
    {
        thresholdManuly = 0, // Manully Threshold
        thresholdAuto = 1, // Auto Threshold
        findPeak = 2,  // Find Peak
    };

namespace battery
{

class XVT_EXPORTS JRBatteryInspectResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawOKResult = false) const -> cv::Point override;

    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

    void SetBaseResult(BatteryInspectorBaseResult &&result)
    {
        mBaseResult = std::move(result);
        CombineResult(&mBaseResult);
    }

    void SetCathodeResult(BatteryInspectorCathodeResult &&result)
    {
        mCathodeResult = std::move(result);
        CombineResult(mCathodeResult.GetResult());
    }

    void SetAnodeResult(BatteryInspectorAnodeResult &&result)
    {
        mAnodeResult = std::move(result);
        CombineResult(mAnodeResult.GetResult());
    }

public:
    BatteryInspectorBaseResult mBaseResult;
    BatteryInspectorCathodeResult mCathodeResult;
    BatteryInspectorAnodeResult mAnodeResult;
    PoleResult mAnodePoleResult;
    PoleResult mCathodePoleResult;
    cv::Rect mUncertainRegion;
    cv::Rect mCertainRegion;
    std::vector<float> snrValRoi;

    //Check Uncertain Region
    bool mShowUncertainRegion = false;
};

class XVT_EXPORTS JRBatteryInspector
{
public:
    JRBatteryInspector();

    auto Inspect(const cv::Mat &src) -> JRBatteryInspectResult;

    auto FindStartPointCathode(const cv::Mat &src, const cv::Rect &roi, cv::Point &catPos) const -> InspectionResult;

    auto FindStartPointByPeak(const cv::Mat& src, cv::Rect subRoi, cv::Point &catPos) const -> InspectionResult;

    auto CheckAnodeInBattery(std::vector<PoleInfo>& anoPos, int minPosAno) const -> InspectionResult;

    auto CalculateSNR(const cv::Mat& img, PoleResult anodeInfo, std::vector<float>& snrValRoi, int stepWid = 15, int stepHei = 100) const->InspectionResult;
    
    auto CalculateCTF(const cv::Mat& img, std::vector<float>& ctfValRoi, int sampleWid = 0, int stepHei = 0) const->InspectionResult;

    auto RegionDivision(const cv::Mat& img) const->InspectionResult;
private:
    auto VerifyResult(JRBatteryInspectResult &result, cv::Size imgSize) -> InspectionResult;

public:
    //Size of one pixel in mm
    bool mIsRotated = true;
    double mPixelSize = 1.0;

    //Parameter Calculate Poiny Cathode
    int mOffset = 3;
    int mThresholdCathodeDefault = 5;

    //Select algorithm for Anode
    int mAlgo = 0;
    //If select manully thershold:
    int mManullyThreshold = 5;
    //If wrong position detect using offset;
    int mOffsetAno = 0;

    float mProminenceValue = 40.0;

    BatteryInspectorBase mIspBase = BatteryInspectorBase(mPixelSize);
    BatteryInspectorJR mIspJR = BatteryInspectorJR(mPixelSize);
    BatteryInspectorPole mIspPole = BatteryInspectorPole(mPixelSize);
    BatteryInspectorCathode mIspCathode = BatteryInspectorCathode(mPixelSize, mIspPole, mIspJR);
    BatteryInspectorAnode mIspAnode = BatteryInspectorAnode(mPixelSize, mIspPole, mIspJR);
};

} // namespace battery
} // namespace xvt
