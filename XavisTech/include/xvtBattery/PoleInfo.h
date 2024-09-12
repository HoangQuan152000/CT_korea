#pragma once
#include "xvtCV/IInspection.h"
#include "xvtCV/Peak.h"

namespace xvt {
namespace battery {
class PoleInfo;
class PoleCompare;

enum class PoleType : int
{
    Real,            // Real detected pole
    Incorrect,
    Adjusted,        // Adjusted pole by copy the left pole to the right pole
    NotFound,        // Can not find the matched pole in the other side
    Inserted,
    InsertedOutRange // The pole is inserted, but it is out of range
};

enum class PoleCompareStatus
{
    Untrust = 0, // The matched detected pole that has length difference >= MaxDiff
    Trust = 1,   // The matched detected pole that has length difference < MaxDiff
    Adjusted
};

struct PoleRemoveInfo
{
    int pointX;
    int index;
};

XVT_EXPORTS PoleInfo GetRefPole(float x, PoleInfo const &trustedPoleBefore, PoleInfo const &trustedPoleAfter);

XVT_EXPORTS std::vector<PoleCompare> evaluatePoleForRefinement(const VecInt &lstPolePosXAll, const VecInt &anodePos, const VecInt &cathodePos,
                                                               int widthROI, double maxDis, double maxDiff, bool refinement);

class XVT_EXPORTS PoleInfo : public InspectionResult {
public:
    PoleInfo(cv::Point anode = cv::Point(), cv::Point cathode = cv::Point(), PoleType type = PoleType::NotFound);
    PoleInfo(Peak peakValue, cv::Point anode = cv::Point(), cv::Point cathode = cv::Point(), PoleType type = PoleType::NotFound);
    auto length() const -> double;

public:
    //Type of left and right pair pole
    PoleType mType;
    cv::Point mAnode;
    cv::Point mCathode;
    Peak mPeakValue;
};

class PoleCompare {
public:
    PoleCompare();

    PoleCompare(PoleInfo &&leftPole, PoleInfo &&rightPole, double const &maxdiff);

    double GetLengthError();

    void SetPole(PoleInfo &&leftPole, PoleInfo &&rightPole, double const &maxdiff);

    // Adjust the pole that type is untrust
    // Return true if pole adjusted
    bool AdjustUncertainType(double maxdiff);
public:
    PoleCompareStatus mType;
    PoleInfo mLeftPole;
    PoleInfo mLeftReferencePole;
    PoleInfo mRightPole;
    PoleInfo mRightReferencePole;
};

} // namespace battery
} // namespace xvt
