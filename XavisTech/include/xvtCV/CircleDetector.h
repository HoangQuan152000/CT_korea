#pragma once
#include "xvtCV/Utils.h"
#include "xvtCV/xvtRange.h"
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/xvtRansac.h"
#include <opencv2/core/types.hpp>
#include <iostream>
#include <string>

namespace xvt {

class XVT_EXPORTS Circle
{
public:
    Circle() = default;
    Circle(cv::Point2d c, double r) :mCenter{ c }, mRadius{ r }{};

    bool operator==(xvt::Circle const& other);

    auto operator+=(cv::Point2d const& p) & ->xvt::Circle&;

    auto operator+(cv::Point2d const& p) const->xvt::Circle;

    auto operator+=(double const& r) & ->xvt::Circle&;

    auto operator+(double const& r) const->xvt::Circle;

    //Get the distance d from a point to line. 
    //d>0: point is outside of circle
    //d<0: point is inside of circle
    auto GetDistance(cv::Point2d p)const->double;

    void Draw(cv::Mat& img, xvt::CVPen pen=xvt::CVPen(), cv::Point2d offset = cv::Point2d(), bool drawCenter = false) const;

public:
    cv::Point2d mCenter;
    double mRadius=0.0;
};

class XVT_EXPORTS CircleResult:
     public Circle
    ,public InspectionResult
{
public:
    CircleResult() = default;
    CircleResult(cv::Point2d c, double r) :Circle(c, r){}

    auto GetRMSE(cv::Point2d p)const->double;
    // Compute mean square error
    template<typename T>
    auto GetRMSE(std::vector<cv::Point_<T>>const& points)const->double;
    auto GetRMSE() const -> double { return GetRMSE(mPoints); }

    void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    //void GetCSVData(CSVOutput& out, std::string prefix = "", bool isResursive = true) const override;
};

class XVT_EXPORTS CircleDetector : public Ransac<CircleResult>
{
public:
    CircleDetector();
    /// <summary>
    /// CircleDetector parameters
    /// </summary>
    /// <param name="radiusRange">Radius range of circle</param>
    /// <param name="dataThreshold">Threshold to consider a sample is belong to model</param>
    /// <param name="modelThreshold">Threshold for number of points that match circle</param>
    /// <param name="maxCoverRate">Max cover rate</param>
    CircleDetector(Ranged radiusRange);

    auto Fit(const std::vector<cv::Point>& pointList, std::vector<cv::Point>& rChoosenPointList) const->ModelResultType;

    auto FitLSQE(xvt::VecPoint const& points) const->ModelResultType override;
    auto IsValidModel(ModelResultType const& res) const->bool override;

    //====================================Static methodes ========================================

    //---------------------------------------------------------------------
    // Fits a circle to a given set of points. There must be at least 2 points
    // The circle equation is of the form: (x-xc)^2 + (y-yc)^2 = r^2
    // Returns true if there is a fit, false in case no circles can be fit
    // rPr: Predict radius
    // rPe: Predict error
    // refer: Least-Squares Circle Fit, Randy Bullock
    static bool FitLeastSquare(const std::vector<cv::Point>& pointList, cv::Point2f& rCenterPoint, double& rPr, double& rPe);
    // Compute mean square error
    static double ComputeRMSE(const std::vector<cv::Point>& pointList, double centerX, double centerY, double R);
    static bool IsOnCircle(double centerX, double centerY, double r, double x, double y, double tolerance);
public:
    //Enable or disable fitting. If disable the center of mass and average of width height will be return as center and radius
    bool mIsEnable = true;

    //Fitting method
    FittingMethod mFitMethod = FittingMethod::RANSAC;

    //Filter the fitted result that has the RMSE in the valid range
    Ranged mValidFitErrorRange = Ranged(0, 1.5, true);

    //Searching radius range
    Ranged mRadiusRange;

    double mMaxCoverRate;

};

template<typename T>
auto CircleResult::GetRMSE(std::vector<cv::Point_<T>> const& points) const -> double
{
    double rms = 0.0;
    size_t size = points.size();
    if (size > 0)
    {
        for (int i = 0; i < size; i++)
        {
            double d = GetDistance(points[i]);
            rms += d * d;
        }
        rms = sqrt(rms / size);
    }
    return rms;
}

inline
auto CircleResult::GetRMSE(cv::Point2d p) const -> double
{
    return abs(GetDistance(p));
}

}
