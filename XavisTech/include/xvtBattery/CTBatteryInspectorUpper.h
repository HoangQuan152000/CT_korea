#pragma once
//#include "xvtBattery/CylinderBatteryBase.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/xvtProperty.h"
#include "xvtBattery/CylinderUtils.h"
#include "xvtBattery/CylinderBatteryBase.h"
#include "xvtBattery/CylinderBatteryResult.h"
#include "xvtBattery/PoleInfo.h"
#include "xvtBattery/BatteryInspectorBase.h"
#include "xvtBattery/BatteryInspectorBeading.h"
#include "xvtBattery/BatteryInspectorJR.h"
#include "xvtBattery/BatteryInspectorPole.h"
#include "xvtBattery/BatteryInspectorCathode.h"
#include "xvtBattery/BatteryInspectorAnode.h"
#include "xvtBattery/BatteryUtils.h"

#pragma region Setting Cylinder Battery CT
#define CB_CT_SETTING_SECTION                   L"INSPECT CYLINDER BATTERY CT UPPER"
#define CB_CT_ROI                               L"Battery ROI(px)"
#define CB_CT_THRESHOLD                         L"Threshold"
#define CB_CT_OFFSET_ROI_X                      L"Offset Roi X"
#define CB_CT_OFFSET_ROI_Y                      L"Offset Roi Y"
#define CB_CT_JR_ROI_X                          L"JRROI_X"
#define CB_CT_JR_ROI_Y                          L"JRROI_Y"
#define CB_CT_POLE_REGION_HEIGHT                L"Pole Region Height(px)"
#define CB_CT_CATHODE_LINE_THRESHOLD_INNER      L"Cathode Line Threshold Inner"
#define CB_CT_ANODE_THRESHOLD_INNER             L"Anode Threshold Inner"
#pragma endregion

namespace xvt {
namespace battery {

class XVT_EXPORTS CTBatteryInspectorUpperResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawOKResult = false) const -> cv::Point override;

    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

    void DrawPoleTextResult(cv::Mat& resImg) const;
public:
    cv::Rect mOuterRoi;
    cv::Rect mPoleRegionRoi;
    cv::Point mCenter;
    cv::Rect mCenterDetected;
    int mCenterWidth;
    PoleResult mAnodePoles;
    VecPoint mCathodePos;
    DisplayMode mDisplayMode = DisplayMode::ALL;
    double mPixelSize = 1.0;
    BatteryInspectorBeadingResult mBeadingResult;
    BatteryInspectionResult BIresult;
};

class XVT_EXPORTS CTBatteryInspectorUpper : public PropertyList
{
public:
    CTBatteryInspectorUpper();

    auto Inspect(cv::Mat const &src) const -> CTBatteryInspectorUpperResult;

    ERR_CODE Inspect(const cv::Mat& inImg, BatteryInspectionResult& BIresult);

    ERR_CODE Inspect(const std::vector<cv::Mat>& inImg, BatteryInspectionResult& BIresult);

    auto InspectAnode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorUpperResult& ispResult) const->InspectionResult;

    auto InspectCathode(cv::Mat const& src, cv::Rect subROI, CTBatteryInspectorUpperResult& ispResult) const->InspectionResult;
private:

    ERR_CODE VerifyParameters(BatteryInspectionResult& BIresult);
  
    auto FindAnodeByLineTracing(const cv::Mat &inputImg,
                                VecPoint &anodes,
                                cv::Point startPoint,
                                int limitVal,
                                int stepHorizontal,
                                int stepVertical,
                                float breakThreshold,
                                int borderDistance,
                                bool restrictMove,
                                int moveAllow,
                                int borderCheck = 0) const -> InspectionResult;

    VecPeak FindCathodeXPositionAuto(const VecFloat& signal, xvt::Rangei& polesDistanceRange, int centerNeglectionWidth, xvt::PeakType type) const;

    VecPeak GetPeakResult(xvt::FindPeaks& findPeaks, VecFloat reduceVec, float prominenceTh, float& minDis, float& maxDis, float alpha = 0.1) const;

    int FindMode(VecInt numVector) const;
public:
    double mPixelSize = 1.0;
    InspectingItem mInspectingItems;
    int mSliceNo = 1;
    int mAvgSliceNo = 3;
    double mGamma = 1.0;
    int mBatteryDirection = 0;

    BatteryInspectorBase mIspBase = BatteryInspectorBase(mPixelSize);
    BatteryInspectorBeading mIspBeading = BatteryInspectorBeading(mPixelSize);
    BatteryInspectorJR mIspJR = BatteryInspectorJR(mPixelSize);
    BatteryInspectorPole mIspPole = BatteryInspectorPole(mPixelSize);
    BatteryInspectorCathode mIspCathode = BatteryInspectorCathode(mPixelSize, mIspPole, mIspJR);
    BatteryInspectorAnode mIspAnode = BatteryInspectorAnode(mPixelSize, mIspPole, mIspJR);
    // Debug mode
    // Results display mode [1: Show Outer ROI, 2: Show Pole Grid, 4: Show Text, 7: Show All ]
    DisplayMode mDisplayMode = DisplayMode::POLE;

    // Text result font scale
	double mTextFontScale = 0.7;

	// Text result line space
	int mTextLineSpace = 25;

	// Text result postion
	cv::Point mTextPosition = cv::Point(0, 150);

    CTBatteryInspectorUpperResult mIspResult;
};
}
}