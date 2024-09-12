#pragma once
#include "xvtBattery/PoleInfo.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/PixelRef.h"
#include "xvtCV/xvtRange.h"

namespace xvt
{
namespace battery
{

class XVT_EXPORTS PoleResult : public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat &img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    auto DrawResultStr(cv::Mat &image,
                       std::string const &name = "",
                       CVPen const &pen = CVPen(),
                       cv::Point const &offset = cv::Point(),
                       bool isDrawOKResult = false) const -> cv::Point override;

    auto GetCSVData(CSVOutput& out, std::string prefix = "", bool isRecursive=true)const->void override;

    auto GetCathodes() const -> std::vector<cv::Point>
    {
        std::vector<cv::Point> cPoint;

        for (const auto &p : mPoles)
            cPoint.push_back(p.mCathode);

        return cPoint;
    }

    auto GetAnodes() const -> VecPoint
    {
        std::vector<cv::Point> aPoint;

        for (const auto &p : mPoles)
            aPoint.push_back(p.mAnode);

        return aPoint;
    }

    auto GetLeftPoles() const -> std::vector<PoleInfo>
    {
        return std::vector<PoleInfo>(mPoles.begin(), mPoles.begin() + mLeftNoPole);
    }

    auto GetRightPoles() const -> std::vector<PoleInfo>
    {
        return std::vector<PoleInfo>(mPoles.begin() + mLeftNoPole, mPoles.end());
    }

    auto GetLenghts() const -> VecDouble
    {
        VecDouble pLenghts;

        for (const auto &p : mPoles)
            pLenghts.push_back(p.length() * mPixelSize);

        return pLenghts;
    }

    auto Size() const -> size_t
    {
        return mPoles.size();
    }

    void ClearAll()
    {
        mPoles.clear();
        mCathode2Anode.clear();
        mAnode2Case.clear();
        mCathode2Case.clear();
        mXPoles.clear();
    }
public:
    std::vector<PoleInfo> mPoles = {};
    std::vector<std::pair<EResult, float>> mCathode2Anode = {};
    std::vector<std::pair<EResult, float>> mAnode2Case = {};
    std::vector<std::pair<EResult, float>> mCathode2Case = {};
    VecDouble mXPoles = {};
    cv::Point mCaseRef = cv::Point(0, 0);
    int mLeftNoPole = 0;
    Ranged mValidCathode2AnodeRange = Ranged(0, 10);
    Ranged mValidAnode2CaseRange = Ranged(0, 10);
    Ranged mValidCathode2CaseRange = Ranged(0, 10);
    double mPixelSize = 1.0;
};

class BatteryInspectorPole : public PixelRef
{
public:
    explicit BatteryInspectorPole(double const &pxSize) : PixelRef{pxSize}
    {
    }
    explicit BatteryInspectorPole(double &&pxSize) = delete;

    auto Inspect(std::vector<PoleInfo> &poles, cv::Point caseRef = cv::Point(-1, -1)) const -> PoleResult;

public:
    bool mEnable = true;
    bool mEnableValidCathode2Anode = false;
    bool mEnableValidAnode2Case = false;
    bool mEnableValidCathode2Case = false;

    int mPoleHeight = 50;

    // Settings for detecting Pole position (in horizontal axis)
    // Pole Detection Threshold
    double mPolesProminenceThreshold = 0;

    // Number Remove Pole Auto
    int mNumberRemovePoleAuto = 1;

    // Setting for making output decision.
    // Number of pole in one side
    int mMaximumNumberPoles = 24;
    int mMinimumNumberPoles = 18;

    // Is checking the number of poles
    bool mIsCheckPoleNo = false;

    // Skip Poles Distance[pixel]
    int mSkipPolesDistance = 0;

    // Poles Distance Range[pixel]
    Rangei mPolesDistanceRange = Rangei(0, 25);

    // Valid Cathode to Anode Range[mm]
    Ranged mValidCathode2AnodeRange = Ranged(0, 10);

    Ranged mValidAnode2CaseRange = Ranged(0, 10);

    Ranged mValidCathode2CaseRange = Ranged(0, 10);

    // Variation Anode to Case[mm]
	double mVariationAnode2Case = 10;

    // Anode to Case offset[pixel]
	int mAnode2CaseOffset = 0;

	// Cathode to Anode offset[pixel]
	int mCathode2AnodeOffset = 0;

	// Cathode to Anode offset[pixel]
	int mCathode2CaseOffset = 0;
};

}; // namespace battery
}; // namespace xvt
