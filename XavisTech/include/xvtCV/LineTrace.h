#pragma once
#include "xvtCV/xvtDefine.h"
#include "opencv2/core/types.hpp"

namespace xvt {
enum class Direction
{
	Backward = -1, // move backward
	//Stop     = 0,  // not move, stop.
	Forward = 1,  // move forward
};

template <class T1, class T2>
void normalizeVector(T1 v, T2& v_norm) {
    if (v.x == 0 && v.y == 0) return;
    double x2 = (double)v.x * (double)v.x;
    double y2 = (double)v.y * (double)v.y;
    double length = sqrt(x2 + y2);

    v_norm.x = v.x / length;
    v_norm.y = v.y / length;
}
class XVT_EXPORTS LineTrace
{
public:
    void SetLowerRange(int lower)& { mLowerMoveRange = lower; }
    void SetUpperRange(int upper)& { mUpperMoveRange = upper; }
    //lower value will be set as lower limit
    void SetXLimit(int a, int b)& { mXLowerLimit = (std::min)(a, b); mXUpperLimit = (std::max)(a, b); }
    void SetYLimit(int a, int b)& { mYLowerLimit = (std::min)(a, b); mYUpperLimit = (std::max)(a, b); }
    void SetAnchor(cv::Point p)&;
    cv::Point GetWindowPosOffset() const;
    bool Trace(cv::Mat const& inputImg, cv::Point const& startPoint, std::vector<std::pair<cv::Point, int>>& traceList) const;
    void TracePoint2(const cv::Mat& inImg, cv::Point2d startPoint, int side_length, int ahead_length, float breakThreshold, std::vector<cv::Point>& collectedPoints, int polesPos = 0, int borderDistance = 0);
    void Draw(cv::Mat const& img, std::vector<std::pair<cv::Point, int>>const& traceList) const;
public:
    //Move direction
    Direction mDirection = Direction::Backward;
    //Y Step to move the searching window
    cv::Point mStep = cv::Point(0, 1);
    //Searching window
    cv::Size mMoveWindowSize{};
    //Intensity threshold
    int mThreshold = 0;
    //Number of allow discontinue points
    int mContinueThreshold = 5;

private:
    bool IsContinue(cv::Point const& p) const
    {
        return (p.x > mXLowerLimit && p.x < mXUpperLimit&& p.y > mYLowerLimit && p.y < mYUpperLimit);
    }

private:
    int mLowerMoveRange = 0;
    int mUpperMoveRange = INT_MAX;

    int mXLowerLimit = 0;
    int mXUpperLimit = INT_MAX;
    int mYLowerLimit = 0;
    int mYUpperLimit = INT_MAX;

    //if value < 0, anchor will be the center of window
    cv::Point mAnchor = cv::Point(-1, -1);
};
};
