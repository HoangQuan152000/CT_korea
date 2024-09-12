#pragma once
#include <opencv2/core/types.hpp>
#include <vector>
#include <utility>

namespace xvt {

using VecInt    = std::vector<int>;     //Vector of int
using VecFloat  = std::vector<float>;   //Vector of float
using VecDouble = std::vector<double>;  //Vector of double

using VecString = std::vector<std::string>;  //Vector of std::string
using VecWString = std::vector<std::wstring>;  //Vector of std::wstring

using VecPoint      = std::vector<cv::Point>;   //Vector of cv::Point
using VecPoint2f    = std::vector<cv::Point2f>; //Vector of cv::Point2f
using VecPoint2d    = std::vector<cv::Point2d>; //Vector of cv::Point2d

using VecVecPoint   = std::vector<VecPoint>;    //Vector of cv::Point Vector
using VecVecPoint2f = std::vector<VecPoint2f>;  //Vector of cv::Point2f Vector
using VecVecPoint2d = std::vector<VecPoint2d>;  //Vector of cv::Point2d Vector

using VecRect       = std::vector<cv::Rect>;  //Vector of cv::Rect
using VecRect2f     = std::vector<cv::Rect2f>;  //Vector of cv::Rect2f
using VecRect2d     = std::vector<cv::Rect2d>;  //Vector of cv::Rect2d

using VecKeyValueStr = std::vector<std::pair<std::string, std::string>>;//header-data vector

template<typename T>
static inline
T SaturateCast(T value, T from, T to)
{
    return (std::min)(cv::saturate_cast<T>(to), (std::max)(cv::saturate_cast<T>(from), value));
}

template<>
static inline
cv::Point SaturateCast<cv::Point>(cv::Point p, cv::Point from, cv::Point to)
{
    assert(from.x <= to.x && from.y <= to.y);
    return cv::Point(SaturateCast<int>(p.x, from.x, to.x), SaturateCast<int>(p.y, from.y, to.y));
}
}
