#pragma once

#include "xvtCV/xvtDefine.h"
#include "xvtCV/IInspection.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace xvt {
namespace ct {

class AngleSelectorResult;

class XVT_EXPORTS AngleSelector{
public:
    void GetCenter(cv::Mat const& img, cv::Point2f center);

    static
    auto GetPoints(cv::Mat const& img, cv::Point2f center, int radius, float angle, float radiusStep = 1.f)->std::vector<cv::Point2f>;

    auto GetAngle(cv::Mat const& img, std::vector<float>& varianceVec) const->std::vector<float>;

    auto GetSliceImage(std::vector<cv::Mat> const& inputImgs, xvt::VecPoint2f const& points) const->cv::Mat;

    auto GetSliceImage(std::vector<cv::Mat>const& inputImgs, float angle1, float angle2)const->cv::Mat;

    auto GetSliceImage(std::vector<cv::Mat>const& inputImgs, cv::Point2f start, cv::Point2f end)const->cv::Mat;

    auto GetSliceImageAvg(std::vector<cv::Mat>const& inputImgs, float angle1, float angle2)const->cv::Mat1b;

    auto GetSliceImageAvg(std::vector<cv::Mat>const& inputImgs, cv::Point2f start, cv::Point2f end)const->cv::Mat1b;

    auto GetSliceImage(std::vector<cv::Mat>const& inputImgs, int zSliceIdx)->AngleSelectorResult;

public:
    int mCenterThreshold = 15;
    int   mAngleNo = 2;
    float mAngleStart= 0.f;
    float mAngleEnd = 360.f;
    float mAngleStep = 1.f;

    float mAvgNo = 3;
    float mAvgStep = 0.5f;

    float mRadius=0;
    float mRadiusStep=1.0f;

    float mVarianceThreshold = 0.f;
    int   mVarianceBlurSize = 15;

    int   mCircleThreshold = 0;

    cv::Point2f mCenter;
};

class XVT_EXPORTS AngleSelectorResult :public InspectionResult
{
public:
    virtual void DrawResult(cv::Mat& img, cv::Point offSetPoint = cv::Point(), CVPen pen = CVPen()) const override;
    void DrawResult(cv::Mat& zImg, cv::Mat& sideImg, int zSliceIdx) const;

public:
    float              mRadius=0;
    float              mAvgNo =0;
    cv::Point2f        mCenter;
    cv::Mat            mSideSliceImage;
    std::vector<float> mAngleList;      //list of angle in degree
    std::vector<float> mVarianceVec;

};
}
}