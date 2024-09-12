#pragma once
#pragma once
#include "CylinderBatteryBase.h"

namespace xvt
{
namespace battery
{
struct JRBatteryInspectResult
{
    std::vector<double> lstAnodeLenght;
    std::vector<cv::Point> lstPoint;
};

class XVT_LIB_EXPORTS JRBatteryInspect
{
public:
    void DrawResults(cv::Mat &img, const std::vector<cv::Point> endAnodePos, const std::vector<cv::Point> endCathodePos, cv::Rect subROI);
    JRBatteryInspectResult Inspect(const cv::Mat &src, cv::Mat &res, cv::Rect subROI);
    cv::Mat MaskFilterRoi(cv::Mat &src, std::vector<cv::Point> points, double offset);
    std::vector<cv::Point> FindAnodeLine(const cv::Mat &img, cv::Point offsetAnode, int region = 10);
    cv::Mat ContourMaskFilter(const cv::Mat &imgROI, std::vector<cv::Point> cathodeLine, cv::Rect &subROI);
    cv::Point FindStartPointCathode(const cv::Mat &src, cv::Rect &midSubROI, cv::Rect &subROI);

public:
    const int mMinImageHeight = 15;

    //DiagonalFilter
    int mFilterSize = 7;
    double mSigma = 1.0;
    int mBorderMode = cv::BORDER_REFLECT; //border types for blur, soble

    //Inspect
    int mPoleHeight = -130;
    int mKSize = 3;

    //Parameter Calculate Poiny Cathode
    int mOffset = 7;
    int mThresholdCathodeDefault = 5;

    //Parameter Calculate Point Anode
    double mOffsetThreshold = 20;
    int mSizeW = 2;

    double mProminenceThreshold = 2.0;
    int mProminenceDistanceThreshold = 7;
};

} // namespace battery
} // namespace xvt
