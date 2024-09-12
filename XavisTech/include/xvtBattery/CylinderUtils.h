#pragma once
#include "xvtBattery/BatteryUtils.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtEnumTemplate.h"
#include <xvtCV/xvtRange.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iomanip>
#include <numeric>
#include <execution>

#define XVT_LIB_EXPORTS XVT_EXPORTS

#define USE_POLE_LENGTH_REFINE
#define MAX_HEAD_NUM 64

//#define checkingRoilower
#ifdef _DEBUG
//#define DEBUG_FIND_PEAK_BY_PROMINENCE
//#define DEBUG_POLE
//#define DEBUG_POLE_LENGTH_REFINE
//#define DEBUG_POLE_LEANING 1
//#define DEBUG_FIND_ANODE_X_POS
//#define DEBUG_FIND_LINE_AUTO
//#define DEBUG_FIND_CATHODE_X_POSITION_AUTO
#else
#endif // DEBUG

namespace xvt
{
struct Peak;
namespace battery
{
class BatteryInspectionResult;
class CylinderBatteryBase;

enum class ERR_CODE
{
    NA = -1,
    OK = 0,
    NG = 1,

    errImageEmpty = 10,
    errImageFormat,

    errBatterySize = 100,
    errWidthBattery,
    errBeadingArea,
    errBeadingInsp,
    errBatteryROI,
    errJRFinding,

    errReferenceCase,
    errCheckBlackCloud,
    errCenterPin,
    errCenterPinROI,
    errCenterPinDepth,
    errPixelSize,

    errPoleXDetection = 200,
    errPoleXDetection2,
    errPoleXDetection3,
    errPoleXDetection4,
    errPoleXDetection5,

    errPoleCathodeLine = 210,
    errPoleCathodeLine1,
    errPoleCathodeLine2,
    errPoleCathodeLine3,
    errPoleCathodeLine4,
    errPoleCathodeLine5,
    errPoleCathodeLine6,

    errPoleAnodeLine = 220,

    errPoleNotRefinable = 300,
    errPoleRefinement = 301,
    errROIRefinement,

    errInspection = 400,
};
DEFINE_CONVERT_ENUM_FUNCS(ERR_CODE);

enum DisplayMode : int
{
    NONE = 0,           // No flags
    POLE = 1,           // Show Pole
    GRID = 3,           // Show Pole and Grid
    TEXT = 5,           // Show Pole and Text
    ALL  = GRID | TEXT,  // Show all (includes Grid and Text, which also include Pole)
    DALL = SECRET_CODE, // Show all debug
};

struct listPole
{
    std::vector<pointValue> listMiniInten;
    cv::Point starting;
    cv::Point ending;
};

struct SegmentMeans
{
	float meanLeftInner = 0.0f;
	float meanLeftMiddle = 0.0f;
	float meanLeftOuter = 0.0f;
	float meanRightInner = 0.0f;
	float meanRightMiddle = 0.0f;
	float meanRightOuter = 0.0f;
};

XVT_LIB_EXPORTS void show_window(const std::string &name, const cv::Mat &img, cv::WindowFlags flag = cv::WindowFlags::WINDOW_NORMAL);

XVT_LIB_EXPORTS std::vector<double> weightedAvgSmoothen(std::vector<double> values);

XVT_LIB_EXPORTS std::vector<double> gaussSmoothen(std::vector<double> values, double sigma, int samples);

XVT_LIB_EXPORTS std::vector<cv::Point> rotate180vtPoint(const std::vector<cv::Point> &vtP_In, cv::Point center);

XVT_LIB_EXPORTS cv::Point rotate180Point(const cv::Point &pntIn, cv::Point center);

XVT_LIB_EXPORTS SegmentMeans calc_mean_region(const std::vector<float>& signal, int neglect_center);

XVT_LIB_EXPORTS void MeanCalibration(const cv::Mat& src, cv::Mat& dst, std::vector<cv::Rect>& regions, std::vector<float>& listMean, bool enabled = false);

template <typename T>
void CheckErasedPole(std::vector<T> lstPolePosLeft,
                     std::vector<T> lstPolePosLRight,
                     std::vector<std::pair<int, int>> &idxPoleDistanceErr,
                     Rangei distanceRange,
                     T boundaryList[],
                     bool isCheckPoleNo,
                     unsigned int numberOfMissingPoleIsAllowed = 2)
{
    if (lstPolePosLeft.size() < 3 || lstPolePosLRight.size() < 3)
        return;

    std::sort(lstPolePosLeft.begin(), lstPolePosLeft.end(), [](auto &a, auto &b) { return a < b; });
    std::sort(lstPolePosLRight.begin(), lstPolePosLRight.end(), [](auto &a, auto &b) { return a < b; });

    // Caculator average distance between two real pole
    float averageDistance = 0.0;
    float sumDistance = 0.0;
    int count = 0;

    int leftPolesSize = lstPolePosLeft.size();
    int rightPolesSize = lstPolePosLRight.size();

    for (auto it = lstPolePosLeft.begin() + 1; it != lstPolePosLeft.end(); it++)
    {
        // distance among this and leftside this position
        int distance = *it - *(it - 1);

        if (distanceRange.IsInRange(distance))
        {
            sumDistance = sumDistance + distance;
            count++;
        }
    }
    for (auto it = lstPolePosLRight.begin() + 1; it != lstPolePosLRight.end(); it++)
    {
        // distance among this and leftside this position
        int distance = *it - *(it - 1);

        if (distanceRange.IsInRange(distance))
        {
            sumDistance = sumDistance + distance;
            count++;
        }
    }

    if (count <= 0)
    {
        return;
    }
    //avread distance of real poles.
    averageDistance = sumDistance / (float)count;
    float missingPoleAllowed = (numberOfMissingPoleIsAllowed + 1) * averageDistance;

    if (lstPolePosLeft[0] - boundaryList[0] > missingPoleAllowed)
    {
        idxPoleDistanceErr.push_back(std::make_pair(boundaryList[0], lstPolePosLeft[0]));
    }

    if (!isCheckPoleNo)
    {
        if (boundaryList[1] - lstPolePosLeft.back() > missingPoleAllowed)
        {
            idxPoleDistanceErr.push_back(std::make_pair(lstPolePosLeft.back(), boundaryList[1]));
        }
        if (lstPolePosLRight[0] - boundaryList[2] > missingPoleAllowed)
        {
            idxPoleDistanceErr.push_back(std::make_pair(boundaryList[2], lstPolePosLRight[0]));
        }
    }

    if (boundaryList[3] - lstPolePosLRight.back() > missingPoleAllowed)
    {
        idxPoleDistanceErr.push_back(std::make_pair(lstPolePosLRight.back(), boundaryList[3]));
    }

    for (int i = 0; i < leftPolesSize - 1; i++)
    {
        int j = i + 1;
        int distance = std::abs(lstPolePosLeft[i] - lstPolePosLeft[j]);
        if (distance > missingPoleAllowed)
        {
            idxPoleDistanceErr.push_back(std::make_pair(lstPolePosLeft[i], lstPolePosLeft[j]));
        }
    }
    for (int i = 0; i < rightPolesSize - 1; i++)
    {
        int j = i + 1;
        int distance = std::abs(lstPolePosLRight[i] - lstPolePosLRight[j]);
        if (distance > missingPoleAllowed)
        {
            idxPoleDistanceErr.push_back(std::make_pair(lstPolePosLRight[i], lstPolePosLRight[j]));
        }
    }
}

XVT_LIB_EXPORTS std::string GetFileName(std::string const &path);

XVT_LIB_EXPORTS std::size_t replace_all(std::string &inout, std::string what, std::string with);

XVT_LIB_EXPORTS int getMaxAreaContourId(const std::vector<std::vector<cv::Point>> &contours, int minSize);

XVT_LIB_EXPORTS int getMaxRectangleId(const std::vector<cv::Rect> &listRect);

XVT_LIB_EXPORTS int getPointOnContourId(const std::vector<cv::Point> &contour, cv::Point pt);

//Function for finding  top reference Point
XVT_EXPORTS
auto FindCornerPoints(std::vector<cv::Point> const &contour, cv::Point &tl, cv::Point &tr, cv::Point &br, cv::Point &bl) -> InspectionResult;

XVT_LIB_EXPORTS cv::Mat histDisplay(std::vector<int> histogram, const char *name, int minTh, int maxTh);

XVT_LIB_EXPORTS ERR_CODE
PoleLengthRefinement(std::vector<int> lstPolePosXAll, std::vector<int> &anodePos, std::vector<int> &cathodePos, int widthROI, std::string &strDesc);

XVT_LIB_EXPORTS int findClosestPoint(std::vector<int> vtDst, int value, int maxRange);

XVT_LIB_EXPORTS std::pair<std::vector<int>, std::vector<double>> RemoveCloseLine(std::vector<std::tuple<int, double, double>> poleList, int minDis);

// Add virtual pole position in the losing poles
// lstPolePos: List of pole detected
// maxDis: Maximum distance between two pole
// numPole: number of one side pole
XVT_LIB_EXPORTS std::vector<int> FindLosingPoleLite(std::vector<int> lstPolePos, int maxDis, int numPole);

XVT_LIB_EXPORTS void drawPoleTextResult(cv::Mat &resImg,
                                        cv::Point const &startPos,
                                        std::string const &poleNumStr,
                                        cv::Scalar anodeColor,
                                        std::string const &strAnode2Boundery,
                                        std::string const &strCath2Ano,
                                        cv::Scalar anode2CathodeColor,
                                        std::string const &anode2CathodeLimitStr,
                                        std::string const &strAno2Case,
                                        cv::Scalar anode2CaseColor,
                                        std::string const &anode2CaseLimitStr,
                                        cv::Scalar const &inforColor,
                                        float fontScale,
                                        float fontWeight);


XVT_LIB_EXPORTS int plotPoleWithRect(cv::Mat &resImg,
                                     const cv::Rect &poleRegionROI,
                                     int CenterNeglectionWidth,
                                     int SkipPoleDistance,
                                     int nLeftPole,
                                     int lineType,
                                     bool isShowGrid,
                                     const std::vector<cv::Point> &lstAnodePnt,
                                     const std::vector<cv::Point> &lstCathodePnt,
                                     const std::vector<bool> &listAno2CathodDecision,
                                     const std::vector<bool> &listAno2CaseDecision,
                                     const std::vector<bool> &listCathode2CaseDecision,
                                     const std::vector<std::pair<int, int>> &lstPolePositionErr);

XVT_LIB_EXPORTS int findAnodeAutoByEdge(const cv::Mat &inputImg,
                                        cv::Point startPoint,
                                        int defaultVal,
                                        int stepHorizontal,
                                        int stepVertical,
                                        cv::Mat &drawOuput,
                                        std::vector<pointValue> &listMinimumIntensity,
                                        cv::Point &ending);
XVT_LIB_EXPORTS void refineAnodePos(std::vector<listPole> listAllPoles,
                                    std::vector<int> &vtAnodePos,
                                    std::vector<int> &vtCathodePos,
                                    std::vector<int> &lstPolePosXAll,
                                    cv::Mat image,
                                    cv::Mat drawoutut);
XVT_LIB_EXPORTS std::vector<cv::Point> findCathodeLineByAStar(const cv::Mat &srcImg, int centerNeglectionWidth);

//Line Tracing Added 17/06/2021 (Bach)
XVT_LIB_EXPORTS int findAnodeByLineTracing(const cv::Mat &inputImg,
                                           cv::Point startPoint,
                                           int defaultVal,
                                           int stepHorizontal,
                                           int stepVertical,
                                           float breakThreshold,
                                           cv::Mat &drawOuput,
                                           int borderCheck = 0,
                                           int borderDistance = 0,
                                           bool restrictMove = false,
                                           int moveAllow = 0);

XVT_LIB_EXPORTS std::pair<int, int> findUnstableRegion(const std::vector<int> &lstPolePosXAlLBeforeRemove,
                                                       std::vector<int> &lstPolePosXAll,
                                                       std::vector<int> &anodePosAfterRemove,
                                                       std::vector<int> &cathodePosAfterRemove,
                                                       std::vector<int> &listRemovebyDominant,
                                                       int widthROI,
                                                       int numberPoleChecking,
                                                       cv::Mat &debugImg);
XVT_LIB_EXPORTS ERR_CODE PoleLengthRefinement(const std::vector<int> &lstPolePosXAll,
                                              std::vector<int> &anodePos,
                                              std::vector<int> &cathodePos,
                                              int widthROI,
                                              std::string &strDesc,
                                              cv::Mat debugImg = cv::Mat());

XVT_LIB_EXPORTS float findTiltAngle(const std::vector<cv::Point> &contour, int &start, int &end);

XVT_LIB_EXPORTS bool drawPoleTextResult(cv::Mat &resImg,
                                        const std::vector<double> &listAnode2CathodeDistance,
                                        const std::vector<double> &listAnode2CaseDistance,
                                        std::vector<double> const &listPolePos,
                                        const std::vector<bool> &listAno2CathodDecision,
                                        const std::vector<bool> &listAno2CaseDecision,
                                        double MinCathode2Anode,
                                        double MaxCathode2Anode,
                                        double MinAnode2Case,
                                        double MaxAnode2Case,
                                        double cathode2AnodeOffset,
                                        double anode2CaseOffset,
                                        bool isCheckPoleNo,
                                        int OneSidePoleNumb,
                                        int nLeftPole,
                                        float fontScale = 0.7,
                                        int textLineSpace = 25,
                                        cv::Point position = cv::Point() //Draw top-left start position
);

XVT_LIB_EXPORTS bool drawPoleTextResult(CylinderBatteryBase *BInspect, BatteryInspectionResult &IResult);

XVT_LIB_EXPORTS bool isPntsIn2Side(cv::Point p1, cv::Point p2, std::vector<cv::Point> Ls, std::vector<cv::Point> Rs);

XVT_LIB_EXPORTS void AutoEnhance(cv::Mat &rImg);

XVT_LIB_EXPORTS cv::Mat getHistogramImg(const cv::Mat &src, int minTh = -1, int maxTh = -1);

XVT_LIB_EXPORTS std::vector<cv::Point> Dijkstra_Algorithm(const cv::Mat &src);

XVT_LIB_EXPORTS cv::Mat LocalHistogramEqualization(const cv::Mat &src, const int region);

XVT_LIB_EXPORTS std::pair<std::vector<int>, std::vector<double>> LowerRemoveCloseLine(std::vector<std::tuple<int, double, double>> poleList,
                                                                                      int polesMinDistance);

} // namespace battery
} // namespace xvt
