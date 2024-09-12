#pragma once
#include "xvtCV/GlobalThresholding.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtTypes.h"

namespace xvt {

inline bool CompareArea(VecPoint const& lhs, VecPoint const& rhs)
{
    return (cv::contourArea(lhs) < cv::contourArea(rhs));
}

inline bool CompareHeight(VecPoint const& lhs, VecPoint const& rhs)
{
    return (cv::boundingRect(lhs).height < cv::boundingRect(rhs).height);
}

inline bool CompareWidth(VecPoint const& lhs, VecPoint const& rhs)
{
    return (cv::boundingRect(lhs).width < cv::boundingRect(rhs).width);
}

inline bool CompareRectArea(VecPoint const& lhs, VecPoint const& rhs)
{
    return (cv::boundingRect(lhs).area() < cv::boundingRect(rhs).area());
}

inline bool CompareSize(VecPoint const& lhs, VecPoint const& rhs)
{
    return (lhs.size() < rhs.size());
}

template<class FilterFunc>
auto FindContour(cv::Mat const& binImg
                 , const FilterFunc& func
                 , cv::ContourApproximationModes ApproMode = cv::CHAIN_APPROX_SIMPLE
                 , cv::Point offset=cv::Point()
)->std::vector<std::vector<cv::Point>>
{
    std::vector<std::vector<cv::Point>> outContours;
    if (!binImg.empty() && binImg.type() == CV_8UC1)
    {
        std::vector<std::vector<cv::Point>> contour;
        cv::findContours(binImg, contour, cv::RetrievalModes::RETR_LIST, ApproMode, offset);

        for (auto& c : contour)
        {
            if (func(c))
            {
                outContours.push_back(std::move(c));
            }
        }
    }

    return outContours;
}

/// <summary>
/// Find the set of contour that sastify the filter function
/// </summary>
/// <example>
/// Finding the contour that height of it is between minHeight and maxHeight
/// <code>
/// FindContour(img, thd,
///    [&amp;minHeight, &amp;maxHeight](auto &amp; c)
///    {
///        cv::Rect  boundRect cv::boundingRect(c);
///        return (boundRect.height &gt; minHeight &amp;&amp; boundRect.height &lt; maxHeight);
///    }
/// )
/// </code>
/// </example>
/// <typeparam name="FilterFunc">Function pointer</typeparam>
/// <param name="img">Input image</param>
/// <param name="threshold">Threshold image</param>
/// <param name="func">Function pointer or lamda function that filter the contour</param>
/// <param name="isInvert">Object is black or not</param>
/// <returns>Filtered contours</returns>
template<class FilterFunc>
auto FindContour(cv::Mat const& img
                 , int threshold
                 , bool isInvert
                 , FilterFunc&& func
                 , cv::ContourApproximationModes ApproMode = cv::CHAIN_APPROX_SIMPLE
                 , cv::Point offset = cv::Point()
)->std::vector<std::vector<cv::Point>>
{
    std::vector<std::vector<cv::Point>> outContours;
    if (!img.empty())
    {
        cv::Mat binImg;
        threshold = (std::max)(threshold, 0);
        int thresholdType = isInvert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
        if (threshold == 0)
        {
            thresholdType |= cv::THRESH_OTSU;
        }

        cv::threshold(img, binImg, threshold, 255, thresholdType);
        outContours = FindContour(binImg, std::forward<FilterFunc>(func), ApproMode, offset);
    }

    return outContours;
}

template<class CompareFunc>
auto FindMaxContour(cv::Mat const& binImg
                 , CompareFunc&& func
                 , cv::ContourApproximationModes ApproMode = cv::CHAIN_APPROX_SIMPLE
                 , cv::Point offset = cv::Point()
)->std::vector<cv::Point>
{
    std::vector<cv::Point> outContour;
    if (!binImg.empty() && binImg.type() == CV_8UC1)
    {
        std::vector<std::vector<cv::Point>> contour;
        cv::findContours(binImg, contour, cv::RetrievalModes::RETR_LIST, ApproMode, offset);
        auto maxPtr = std::max_element(contour.begin(), contour.end(), std::forward<CompareFunc>(func));
        
        if (maxPtr != contour.end())
        {
            outContour = std::move(*maxPtr);
        }
    }

    return outContour;
}

template<class CompareFunc>
auto FindMaxContour(cv::Mat const& img
                 , int threshold
                 , bool isInvert
                 , CompareFunc&& func
                 , cv::ContourApproximationModes ApproMode = cv::CHAIN_APPROX_SIMPLE
                 , cv::Point offset = cv::Point()
)->std::vector<cv::Point>
{
    std::vector<cv::Point> outContour;
    if (!img.empty())
    {
        cv::Mat binImg;
        threshold = (std::max)(threshold, 0);
        int thresholdType = isInvert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
        if (threshold == 0)
        {
            thresholdType |= cv::THRESH_OTSU;
        } 

        cv::threshold(img, binImg, threshold, 255, thresholdType);
        outContour = FindMaxContour(binImg, std::forward<CompareFunc>(func), ApproMode, offset);
    }

    return outContour;
}

template<class CompareFunc>
auto FindMaxContour(cv::Mat const& img
                    , int lowThd
                    , int upThd
                    , bool isInvert
                    , CompareFunc&& func
                    , cv::ContourApproximationModes ApproMode = cv::CHAIN_APPROX_SIMPLE
                    , cv::Point offset = cv::Point()
)->std::vector<cv::Point>
{
    std::vector<cv::Point> outContour;
    if (!img.empty())
    {
        cv::Mat binImg;
        lowThd = (std::max)(lowThd, 0);
        upThd = (std::max)(upThd, 0);
        if (lowThd > upThd) std::swap(lowThd, upThd);

        int thresholdType = isInvert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
        if (lowThd == 0)
        {
            thresholdType |= cv::THRESH_OTSU;
            upThd = 255;
        }

        xvt::threshold::Threshold(img, binImg, lowThd, upThd, thresholdType);
        outContour = FindMaxContour(binImg, std::forward<CompareFunc>(func), ApproMode, offset);
    }

    return outContour;
}

template<class FilterFunc>
auto FindContour(cv::Mat const& img
                 , int lowThd, int upThd
                 , bool isInvert
                 , FilterFunc&& func
                 , cv::ContourApproximationModes ApproMode = cv::CHAIN_APPROX_SIMPLE
                 , cv::Point offset = cv::Point()
)->std::vector<std::vector<cv::Point>>
{
    std::vector<std::vector<cv::Point>> outContours;
    if (!img.empty())
    {
        int thresholdType = isInvert ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

        cv::Mat binImg;
        xvt::threshold::Threshold(img, binImg, lowThd, upThd, thresholdType);
        outContours = FindContour(binImg, std::forward<FilterFunc>(func), ApproMode, offset);
    }

    return outContours;
}

//Return the rotated angle in radian
XVT_EXPORTS
double GetRotatedAngle(std::vector<cv::Point> const& ptList
                     , cv::Point2d& mainAxis
                     , cv::Point2d& center
);

//Return true if contour is not empty, find the cloest point on the contour that nearest the pt point
XVT_EXPORTS
bool FindClosestPointOnContour(const std::vector<cv::Point>& contour, cv::Point const& pt, int& index);

//Function for finding  top reference vector index
XVT_EXPORTS
auto FindCornerPointsIdx(std::vector<cv::Point> const& contour)->std::vector<int>;

//Function for finding  top reference vector Point
XVT_EXPORTS
auto FindCornerPoints(std::vector<cv::Point> const& contour)->std::vector<cv::Point>;

//Function for finding  top reference Point
XVT_EXPORTS
auto FindCornerPoints(std::vector<cv::Point> const& contour, cv::Point& tl, cv::Point& tr, cv::Point& br, cv::Point& bl)->bool;

class XVT_EXPORTS Contour
{
public:
    Contour();
    Contour(std::vector<cv::Point2f> const& pointList);
    Contour(std::vector<cv::Point2f>&& pointList);

    Contour(std::vector<cv::Point> const& pointList);

    void SetBoundaryPoints(std::vector<cv::Point> const& pList);
    void SetBoundaryPoints(std::vector<cv::Point2f> const& pList);
    std::vector<cv::Point> GetBoundaryPointList() const&;
    void SetBoundaryPoints(std::vector<cv::Point2f>&& pList);
    std::vector<cv::Point2f> GetBoundaryPoint2fList()&&;
    std::vector<cv::Point2f> const& GetBoundaryPoint2fList() const&;

    /// <summary>
    /// Get the area of the contour
    /// </summary>
    /// <returns>area</returns>
    double GetArea();

    double GetRotatedAngle();

    /// <summary>
    /// Get the closest rectangle bound the contour
    /// </summary>
    /// <returns></returns>
    cv::Rect GetBoundaryRectangle();

    /// <summary>
    /// Get the min rotated rectangle bound the contour
    /// </summary>
    /// <returns></returns>
    cv::RotatedRect GetMinBoundaryRectangle();

    /// <summary>
    /// Get the center of mass of the contour
    /// </summary>
    /// <returns></returns>
    cv::Point2f GetCenter();

    /// <summary>
    /// Find the contour that has the maximum area in the contour list
    /// </summary>
    /// <param name="binImg">Binary image</param>
    /// <param name="rMaxContour">Maxcontour output</param>
    /// <returns>True if finding sucessfully, false if can not find</returns>
    static bool FindMaxContour(cv::Mat const& binImg, Contour& rMaxContour);

    void Draw(cv::Mat& inImg, CVPen pen, cv::Point2f offSet = cv::Point2f()) const;

    void DrawContour(cv::Mat& inImg, CVPen pen, cv::Point2f offSet = cv::Point2f()) const;

private:
    void SetAllCalculatedFlag(bool flag);

public:
    double mRefAngle;

private:
    //Calculated Flags
    bool mIsCenterCalculated;
    bool mIsAreaCalculated;
    bool mIsBoundaryRectCalculated;
    bool mIsRotatedBoundaryRectCalculated;
    bool misRotatedAngleCalculated;

    //Memebers
    double mArea;
    cv::Rect2f mBoundaryRect;
#pragma warning (suppress:4251)
    cv::RotatedRect mRotatedBoundaryRect;
#pragma warning (suppress:4251)
    std::vector<cv::Point2f> mBoundaryPointList;
    cv::Point2f mCenter;
    double mRotatedAngle;
    float mSquareTolerance;
};

}

