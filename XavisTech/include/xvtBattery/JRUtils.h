#pragma once

#include "xvtBattery/BatteryUtils.h"

namespace xvt {
namespace battery {

XVT_EXPORTS
cv::Point FlipYPoint(cv::Point p1, const cv::Size &size);

XVT_EXPORTS
cv::Point FlipXPoint(cv::Point p1, const cv::Size &size);

XVT_EXPORTS
cv::Point TransposePoint(cv::Point p1);

XVT_EXPORTS
void FlipXPoints(std::vector<cv::Point> &vtPoints, const cv::Size &size);

XVT_EXPORTS
void FlipYPoints(std::vector<cv::Point> &vtPoints, const cv::Size &size);

XVT_EXPORTS
void TransposePoints(std::vector<cv::Point> &vtPoints);

// source image to inspection image
XVT_EXPORTS
cv::Point TranformPoint(cv::Point p, const cv::Size &size);

// inspection image to source image
XVT_EXPORTS
cv::Point TranformPointInvert(cv::Point p, const cv::Size &size);

// source image to inspection image
XVT_EXPORTS
std::vector<cv::Point> TranformPointsInvert(std::vector<cv::Point> vtPoints, const cv::Size &size, cv::Point offset = cv::Point(0, 0));

// source image to inspection image
XVT_EXPORTS
cv::Rect TranformRect(cv::Rect r, const cv::Size &size);

// source image to inspection image
XVT_EXPORTS
cv::Rect TranformRectInvert(cv::Rect r, const cv::Size &size);

//Battery Left or Right
XVT_EXPORTS
bool BatteryPositionIsLeft(cv::Mat src, cv::Rect r);

}
}
