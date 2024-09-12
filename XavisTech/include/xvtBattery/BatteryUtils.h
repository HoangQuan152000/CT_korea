#pragma once

#include "xvtCV/Utils.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/xvtDefine.h"
#include "xvtCV/Peak.h"
#include "xvtCV/xvtRange.h"

#define SECRET_CODE 255
#define BORDER_WIDTH_RATIO 0.012

namespace xvt {
namespace battery {

enum class anodeAlgoType
{
    LineTracing,
    Edge,
    Kmean,
    Projection,
};

struct pointValue
{
    int intensity;
    cv::Point position;

    pointValue(int i = 0, cv::Point p = cv::Point()) : intensity{i}, position{p}{}
};

struct candiEdP
{
    int sumDerivative;
    int position;
    int firstPos;
    int countDerivative;
    int weightedSum;
    int var;
};

template <typename T>
struct AverageCathodeLineResult
{
    std::vector<T> localAverageCathodeLine{};
    std::vector<T> localAverageCathodeLineLeft{};
    std::vector<T> localAverageCathodeLineRight{};
    std::vector<T> lst4MeanStd{};
    int lstCathodeSize = 0;
    double sum = 0;
    bool success = false;
};

XVT_EXPORTS
cv::Point drawText( cv::Mat &resImg,
                    cv::Point const &startPos,
                    std::vector<std::pair<std::string, cv::Scalar>> const &textList,
                    float fontScale,
                    float fontWeight);

XVT_EXPORTS
cv::Point drawText( cv::Mat &resImg,
                    cv::Point const &pos,
                    std::string const &str,
                    cv::Scalar const &color,
                    float fontScale,
                    float fontWeight);

XVT_EXPORTS
int FindRefLine(cv::Mat &blurImg, int leaningThreshold);

XVT_EXPORTS
std::string GetStrCurrentTime();

XVT_EXPORTS
int GetRegionIndex(int xCord, int regions, int imgWidth, int midWidth);

XVT_EXPORTS
int CheckRegion(int xCord, int regions, int imgWidth, int midWidth);

XVT_EXPORTS
VecFloat GetLUT(int a, int b);

XVT_EXPORTS
cv::Mat DiagonalFilter(const cv::Mat &src, int ksize, double sigmaX, int borderMode = cv::BORDER_DEFAULT);

XVT_EXPORTS
cv::Mat LocalHistogramEqualization(const cv::Mat &src, const int region);

/// <summary>
/// Return the most left and right pole region position
/// </summary>
/// <param name="image">input whole battery image</param>
/// <param name="batteryROI">Found betteryROI by FindBatteryROI() function. This will be changed if this function found the new one.</param>
/// <param name="yA2">Turning point of beading</param>
/// <param name="comparedvalue">To find the pole region</param>
/// <param name="padding">Added space to the border of batteryROI</param>
/// <param name="thresholdvalue">Threshold to find the new battery ROI</param>
/// <param name="leftx"> x most left position + xoffset, if can not find return (batteryROI.x + padding + xoffset)</param>
/// <param name="rightx"> x most right position - xoffset, if can not find return (batteryROI.x + batteryROI.width - (padding + xoffset))</param>
XVT_EXPORTS
bool CheckPoleLeaning(const cv::Mat &image, int poleRegionHeight, int leaningThreshold, double &leftx, double &rightx);

XVT_EXPORTS
bool CheckPoleLeaningAuto(const cv::Mat &image, int windowSize, int &leftx, int &rightx);

XVT_EXPORTS
bool CheckPoleLeaningManual(const cv::Mat &image, int leaningThreshold, int &leftx, int &rightx);

XVT_EXPORTS
VecPoint refineMaskingContour(const VecPoint& inContour,
                                            int imageWidth,
                                            int bottomLineShift,
                                            int TopLineShift,
                                            VecPoint outerPoint);

XVT_EXPORTS
int FindClosestPoint(const VecPoint &vtDst, const int xValue, int xAfter, const int maxRange);

XVT_EXPORTS
int FindClosestPoint(const std::vector<int> &vtDst, const int xValue, int xAfter, const int maxRange);

XVT_EXPORTS
std::vector<Peak> FindCathodeXPositionManual(const VecFloat &reduceVec,
                                                             const float &minProminence,
                                                             const float &polesMinDistance);

XVT_EXPORTS
std::vector<Peak> FindCathodeXPositionAuto(const VecFloat &signal,
                                                           Rangei &polesDistanceRange,
                                                           const int &centerNeglectionWidth);

/*
@param src input image; the image can have any number of channels, which are processed
independently, but the depth should be CV_8U, CV_16U, CV_16S, CV_32F or CV_64F.
@param dim dimension index along which the matrix is reduced. 0 means that the matrix is reduced to
a single row. 1 means that the matrix is reduced to a single column.
@param ksize Gaussian kernel ksize can differ but it must be
positive and odd.Or, it can be zero's and then they are computed from sigma.
*/
XVT_EXPORTS
VecFloat ReduceBlackBackgroundMat(const cv::Mat &src, int dim, unsigned int kSize = 5);

XVT_EXPORTS
VecFloat CheckTiltedLine(const VecPoint& line, int radiusPole = 1);

XVT_EXPORTS
auto AverageSliceImage(const std::vector<cv::Mat>& imgs) -> cv::Mat;

XVT_EXPORTS
// maximum number of slice is 11
auto AverageSliceImage(const cv::Mat& imgs, int noSlice = 1) -> cv::Mat;

XVT_EXPORTS
// maximum number of slice is 11
auto GetSliceImages(const cv::Mat& imgs, int noSlice = 1) -> std::vector<cv::Mat>;

XVT_EXPORTS
auto GetPeakAndValley(const VecFloat& signal, float prominence = 0.01, int distance = 0, int ksize = 3) -> VecPeak;

XVT_EXPORTS
auto GetPeakAndValleyImage(const cv::Mat& src, float prominence = 0.01, int distance = 4, int ksize = 3) -> cv::Mat;

XVT_EXPORTS
auto CalculateCTF(cv::Mat const& src, double prominence = 0.01, int distance = 0, int ksize = 3) -> double;

XVT_EXPORTS
auto FindBattery(cv::Mat const& img, int thd, std::array<cv::Point, 4>& corners) -> InspectionResult;

XVT_EXPORTS
auto FindTurnPoint(cv::Mat const& img, int thd, bool isLeft, int min_length = 50) -> std::tuple<cv::Point, cv::Point, VecPoint, bool>;

template <typename T>
AverageCathodeLineResult<T> FindAverageCathodeLine(std::vector<T> &cathodeLineList,
                                                   int neglectionWidth,
                                                   int startNeglectCol,
                                                   int endNeglectCol,
                                                   int rightEndBounder,
                                                   int localAverageWindowSize,
                                                   int wSize);

template <typename T>
void RefineCathodeLine(AverageCathodeLineResult<T> &avgCathodeLineResult,
                       std::vector<T> &lstCathodeLine,
                       int wSize,
                       int centerLeft,
                       int centerRight);
}

template <typename T>
std::pair<double, double> MeanStdDevLite(const std::vector<T>& signal);

XVT_EXPORTS
inline cv::Rect CreateROI(cv::Rect tmp, cv::Size const& imageSize)
{
    RefineROI(tmp, imageSize);
    return tmp;
}

XVT_EXPORTS
bool SaveCSV(std::wstring const& path, std::wstring const& imgName, xvt::CSVOutput const& output);
}

#include "xvtBattery/BatteryUtils.inl.h"