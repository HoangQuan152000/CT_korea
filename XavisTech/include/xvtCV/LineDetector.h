#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtPen.h"
#include "xvtCV/IInspection.h"
#include "xvtCV/xvtRange.h"
#include "xvtCV/xvtRansac.h"
#include <opencv2/opencv.hpp>
#include <iostream>

namespace xvt {

class XVT_EXPORTS Line
{
public:
    Line() = default;

    //angle is the angle between normal vector of the line and ox in radian
    //distance is distance from original to line
    Line(double angle, double distance);

    Line(double a, double b, double c);

    //Contruct the line from two points p1, p2 on the line
    //isDirectVector=true, p1 will be treated as point on the line and p2 is the line direction vector
    Line(cv::Point2d p1, cv::Point2d p2, bool isDirectVector=false);

    // Compute mean square error
    template<typename T>
    auto GetRMSE(std::vector<cv::Point_<T>>const& points)const->double;

    auto GetRMSE(cv::Point2d p)const->double;

    auto GetIntersectPoint(Line const& l) const->cv::Point2d;

    //Get the distance d from a point to line. 
    //d>0: point in the clockwise direction of normalize vector
    //d<0: point in the counter-clockwise direction of normalize vector
    //clockwise direction is rotation from positive y to positive x direction.
    auto GetDistance(cv::Point2d p = cv::Point2d(0, 0))const->double;

    //Get angle between line and a vector
    //angle > 0 mean it is before the vector 'vec' in the clockwise direction from the vector 'vec'
    //angle < 0 mean it is after the vector 'vec' in the clockwise direction from the vector 'vec'
    //clockwise direction is rotation from positive y to positive x direction.
    auto GetAngle(cv::Point2d vec = (cv::Point2d(1, 0)))const->double;

    auto GetPoint(cv::Point2d p)const->cv::Point2d;

    //Get point when have the x position
    auto GetPointX(cv::Point2d p)const->cv::Point2d;

    //Get point when have the y position
    auto GetPointY(cv::Point2d p)const->cv::Point2d;

    //Vector perpendicular with the line (phap tuyen)
    auto GetNormalVector()const->cv::Point2d;
    auto SetNormalVector(cv::Point2d vec)& ->void;

    //Vector same direction as line (chi phuong) (rotated 90Degree from Noraml vector)
    auto GetNormalizeVector() const ->cv::Point2d const;
    auto SetNormalizeVector(cv::Point2d vec)& ->void;

    //A point on the line
    auto GetPoint0() const ->cv::Point2d const;
    auto SetPoint0(cv::Point2d p) & ->void;

    bool operator==(xvt::Line const& other);

    auto operator+=(cv::Point2d const& p) & ->xvt::Line&;

    auto operator+(cv::Point2d const& p) const->xvt::Line;

    virtual
    //Transform the line base on the tranform matrix m
    auto Transform(cv::Mat const& m)& ->void;

    virtual
    //shift the line by the vector p
    auto Transform(cv::Point2d const& p)& ->void;

    void Draw(cv::Mat& img, xvt::CVPen pen, cv::Point2d offset=cv::Point2d(), bool drawDetail=false) const;

public:
protected:
    double m_wx=0;
    double m_wy=0;
    double m_wr=0;

};

class XVT_EXPORTS LineResult :
      public Line
    , public InspectionResult
{
public:
    LineResult() = default;
    LineResult(xvt::EResult result, std::string const& msg) : Line(), InspectionResult(result, msg) {};
    LineResult(double angle, double distance) : Line(angle, distance), InspectionResult() {};
    LineResult(cv::Point const& p1, cv::Point const& p2) : Line(p1, p2), InspectionResult() {};
    LineResult(Line const& l) : Line(l), InspectionResult() {};
    LineResult(Line && l) : Line(std::move(l)), InspectionResult() {};
    LineResult(InspectionResult const& res) : Line(), InspectionResult(res) {};
    LineResult(InspectionResult && res) : Line(), InspectionResult(std::move(res)) {};

    virtual
    //Transform the line base on the tranform matrix m
    auto Transform(cv::Mat const& m) & ->void override;

    virtual
    //shift the line by the vector p
    auto Transform(cv::Point2d const& p) & ->void override;

    auto GetRMSE() const->double;
    auto GetRMSE(cv::Point2d p)const->double { return Line::GetRMSE(p); }
    auto GetRMSE(xvt::VecPoint points)const->double { return Line::GetRMSE(points); }

    void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;

    void GetCSVData(CSVOutput& out, std::string prefix = "", bool isResursive = true) const override;

public:

};

class XVT_EXPORTS LineDetector:public xvt::Ransac<LineResult>
{
public:
    LineDetector() : RansacType(2, 1.0, 5) {}

    static
    auto FitLineLSQE(std::vector<cv::Point>const& points)->xvt::Line;

    static
    auto FitLineLSQE(std::vector<cv::Point2f>const& points)->xvt::Line;

    static
    auto FitLineLSQE(std::vector<cv::Point2d>const& points)->xvt::Line;

    static
    auto FitLineOpenCV(std::vector<cv::Point>const& points
                        , cv::DistanceTypes disType = cv::DistanceTypes::DIST_L2
                        , double param = 0, double reps = 0.01, double aeps = 0.01
    )->xvt::Line;

    //Find the line on the image, and base on width, height to decide it is horizontal or vertical line
    //it will use the abs sobel x and y image if mUseEdge=true
    auto FindLine(cv::Mat const& img)const->xvt::LineResult;

    //Find the line on the image, it will use the abs sobel x and y image if mUseEdge=true
    auto FindLine(cv::Mat const& img, bool isHorLine=true)const->xvt::LineResult;

    //Find horiztal the line on the image, it will use the sobel y image if mUseEdge=true
    auto FindHorizontalLine(cv::Mat const& img)const->xvt::LineResult;
     
    //Find vertical the line on the image, it will use the sobel x image if mUseEdge=true
    auto FindVerticalLine(cv::Mat const& img)const->xvt::LineResult;

    auto FitLSQE(xvt::VecPoint const& points) const->ModelResultType override;
    auto IsValidModel(ModelResultType const& res) const->bool override;

public:
    bool  mUseEdge=true;    //Apply sobel the image to find the line
    float mThreshold=0;     //Threshold the input or the edge image to find the line
    bool  mIsBlack2White=true;   //Line is between black 2 white region black/white, false is white/black
    xvt::Rangef mValidAngle= xvt::Rangef(0, 0, false);
    xvt::Rangef mValidRMSE = xvt::Rangef(0, 0, false);
};

inline
auto Line::GetNormalVector()const->cv::Point2d
{
    return cv::Point2d(m_wx, m_wy);
}

inline
auto Line::SetNormalVector(cv::Point2d vec) & -> void
{
    double l = cv::norm(vec);
    if (l > 0 && l != 1)
    {
        vec /= l;
    }

    m_wx = vec.x;
    m_wy = vec.y;
}

inline
auto Line::GetNormalizeVector()const->cv::Point2d const
{
    return cv::Point2d(-m_wy, m_wx);
}

inline
auto Line::SetNormalizeVector(cv::Point2d vec) & ->void
{
    auto l = cv::norm(vec);
    if (l > 0 && l != 1)
    {
        vec /= l;
    }
    m_wx = vec.y;
    m_wy = -vec.x;
}

inline
auto Line::GetPoint0()const->cv::Point2d const
{
    return GetPoint(cv::Point2d(0,0));
}

inline
auto Line::SetPoint0(cv::Point2d p) & ->void
{
    m_wr = -m_wx * p.x - m_wy * p.y;
}

inline
bool Line::operator==(xvt::Line const& other)
{
    auto v1 = GetNormalizeVector();
    auto v2 = other.GetNormalizeVector();
    auto p0 = GetPoint0();
    auto dir = abs(abs(v1.ddot(v2)) - 1);
    auto dis = abs(other.GetDistance(p0));
    return  (dir < 2*DBL_EPSILON) && (dis < DBL_EPSILON);
}

inline
auto Line::operator+=(cv::Point2d const& p) & -> xvt::Line&
{
    this->Transform(p);
    return *this;
}

inline
auto Line::operator+(cv::Point2d const& p) const-> xvt::Line
{
    xvt::Line l = { *this };
    l.Transform(p);
    return l;
}

template<typename T>
// Compute mean square error
auto Line::GetRMSE(std::vector<cv::Point_<T>>const& points)const->double
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
        rms= sqrt(rms / size);
    }
    return rms;
}

inline
auto LineResult::GetRMSE() const ->double
{
    return Line::GetRMSE(mPoints);
}

XVT_EXPORTS
auto Transform(Line const& l, cv::Point2d const& p)->xvt::Line;

XVT_EXPORTS
auto Transform(Line&& l, cv::Point2d const& m)->xvt::Line;

XVT_EXPORTS
auto Transform(Line const& l, cv::Mat const& m)->xvt::Line;

XVT_EXPORTS
auto Transform(Line&& l, cv::Mat const& m)->xvt::Line;

XVT_EXPORTS
auto Transform(LineResult const& l, cv::Mat const& m)->xvt::LineResult;

XVT_EXPORTS
auto Transform(LineResult&& l, cv::Mat const& m)->xvt::LineResult;

XVT_EXPORTS
auto Transform(LineResult const& l, cv::Point2d const& p)->xvt::Line;

XVT_EXPORTS
auto Transform(LineResult&& l, cv::Point2d const& m)->xvt::Line;
}

