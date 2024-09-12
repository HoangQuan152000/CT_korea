#pragma once
#include "xvtCV/xvtTypes.h"
#include "xvtCV/xvtDefine.h"
#include "opencv2/core/types.hpp"
namespace xvt
{
/// <summary>
/// Find the shortest path using the Pixel value.
/// </summary>
/// <param name="image">The input grayscale 8-bit image.</param>
/// <param name="start">The starting point for the path.</param>
/// <param name="end">The target or end point for the path.</param>
/// <param name="pixelThreshold">The threshold value for considering pixels in the pathfinding process.</param>
/// <returns>A vector of points representing the shortest path.</returns>
XVT_EXPORTS
auto FindPathWithPixelValue(const cv::Mat& image, const cv::Point& start, const cv::Point& end, 
                                std::vector<cv::Point> const& directions,int pixelThreshold=255)->std::vector<cv::Point>;
}
