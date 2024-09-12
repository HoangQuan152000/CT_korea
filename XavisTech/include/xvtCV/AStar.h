#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtTypes.h"
#include "opencv2/core/types.hpp"
#include <vector>

namespace xvt
{
/// <summary>
/// Find the line by using A* algorithm
/// </summary>
/// <param name="src">gray 8bit image</param>
/// <param name="blurSize">blur the output line, 0 means not blur. It should be equal or greater than 0</param>
/// <returns></returns>
XVT_EXPORTS
auto FindPathByAStar(const cv::Mat& src, int blurSize=0) -> VecPoint;
}