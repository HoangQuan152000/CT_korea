#pragma once
#include "xvtCV/xvtDefine.h"
#include "xvtCV/xvtFile.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <type_traits>
#include <string>

namespace xvt {

//Check if a ROI is inside the range [0, imgW) and [0, imgH)
constexpr
bool IsValidROI(int x, int y, int w, int h, int imgW, int imgH)
{
    return (x >= 0 && y >= 0 && w >= 0 && h >= 0 && x < imgW&& y < imgH&& x + w <= imgW && y + h <= imgH);
}

//Return true if refined ROI is not empty
inline
bool RefineROI(cv::Rect& selectROI, cv::Size const& imageSize)
{
    if (imageSize.empty())
    {
        selectROI = cv::Rect();
    }
    else
    {
        cv::Point br = selectROI.br();

        selectROI.x = (std::min)((std::max)(0, selectROI.x), imageSize.width);
        selectROI.y = (std::min)((std::max)(0, selectROI.y), imageSize.height);

        br.x = (std::min)((std::max)(0, br.x), imageSize.width);
        br.y = (std::min)((std::max)(0, br.y), imageSize.height);

        selectROI.width = (std::max)(0, br.x - selectROI.x);
        selectROI.height = (std::max)(0, br.y - selectROI.y);

        if (selectROI.x == imageSize.width) selectROI.x -= 1;
        if (selectROI.y == imageSize.height) selectROI.y -= 1;
    }

    return !selectROI.empty();
}

inline
cv::Rect CreateROI(int x, int y, int w, int h, cv::Size const& imageSize)
{
    cv::Rect tmp{ x, y , w, h };
    RefineROI(tmp, imageSize);
    return tmp;
}

inline
cv::Rect CreateROI(cv::Point const& p, cv::Size const& s, cv::Size const& imageSize)
{
    cv::Rect tmp{ p , s };
    RefineROI(tmp, imageSize);
    return tmp;
}

//Refine the ROI and then return the img(roi)
inline
cv::Mat GetImage(cv::Mat const& img, cv::Rect& roi)
{
    xvt::RefineROI(roi, img.size());
    return img(roi);
}

//Refine the ROI and then return the img(roi)
inline
cv::Mat GetImage(cv::Mat const& img, cv::Rect&& roi)
{
    xvt::RefineROI(roi, img.size());
    return img(roi);
}

/// <summary>
/// Calculate the distance between two opencv points.
/// </summary>
/// <typeparam name="T">Type of points</typeparam>
/// <param name="p1"> First point</param>
/// <param name="p2">Second point</param>
/// <returns></returns>
template <class T>
inline double Distance(T const& p1, T const& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

template <class T>
static double GetLength(T const& p1, T const& p2)
{
    T d = p2 - p1;
    return sqrt((double)d.x * d.x + (double)d.y * d.y);
}

template <class T>
static double GetLength(T const& p)
{
    return sqrt((double)p.x * p.x + (double)p.y * p.y);
}

//Return 0 if no error, 1 if empty image, 2 if not support
//when src is U8C1, isClone=true dst=src.clone() otherwise dst=src
XVT_EXPORTS
auto Convert8Bits(const cv::Mat& src, cv::Mat& dst, bool isClone = true)->int;

//Return 0 if no error, 1 if empty image, 2 if not support
//when src is U8C3, isClone=true dst=src.clone() otherwise dst=src
XVT_EXPORTS
auto ConvertRGB(const cv::Mat& src, cv::Mat& dst, bool isClone = true) -> int;

XVT_EXPORTS
auto ReadImage(std::wstring const& filePath, int flags = 2)->cv::Mat;

XVT_EXPORTS
void WriteImage(std::wstring const& filePath, cv::Mat const& image);

XVT_EXPORTS
void RemoveDuplicatePoint(std::vector<cv::Point>& points);

// Function to find points where each point has only one neighbor
XVT_EXPORTS
auto FindPointsWithOneNeighbor(const std::vector<cv::Point>& points, const std::vector<cv::Point>& directions)->std::vector<cv::Point>;

// Lexicographic compare, same as for ordering words in a dictionnary:
// test first 'letter of the word' (x coordinate), if same, test 
// second 'letter' (y coordinate).
XVT_EXPORTS
bool LexicoCompare(const cv::Point& p1, const cv::Point& p2);

//Types of fitting
enum class XVT_EXPORTS FittingMethod : int
{
      RANSAC = 0
    , LEAST_SQUARE
};

inline
double round_f(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

//Convert from Radian to Degree
inline constexpr
double Rad2Deg(double val)
{
    return (180.0 / CV_PI) * val;
}

//Convert from Degree to Radian
inline constexpr
double Deg2Rad(double val)
{
    return (CV_PI / 180.0) * val;
}

template<class T>
inline
auto Transform(cv::Point_<T> const& p, cv::Mat const& m)->cv::Point_<T>
{
    cv::Point_<T> tmp = p;
    if (!m.empty())
    {
        tmp.x = m.at<double>(0, 0) * p.x + m.at<double>(0, 1) * p.y + m.at<double>(0, 2);
        tmp.y = m.at<double>(1, 0) * p.x + m.at<double>(1, 1) * p.y + m.at<double>(1, 2);
    }
    return tmp; 
}

template<class T>
inline
auto Transform(std::vector<cv::Point_<T>> const& src, cv::Mat const& m)->std::vector<cv::Point_<T>>
{
    std::vector<cv::Point_<T>> dst;
    if (m.empty())
    {
        dst = src;
    }
    else if(!src.empty())
    {
        if(m.rows==2)
            cv::transform(src, dst, m);

        if (m.rows == 3)
            cv::perspectiveTransform(src, dst, m);
    }

    return dst;
}

template<class T>
inline
auto Transform(std::vector<cv::Point_<T>> const& src, cv::Point_<T> const& shift)->std::vector<cv::Point_<T>>
{
    std::vector<cv::Point_<T>> dst;
    if (!src.empty())
    {
        dst.reserve(src.size());
        for (auto p : src)
        {
            dst.push_back(p + shift);
        }
    }

    return dst;
}

template<class T>
inline
auto Transform(std::vector<std::vector<cv::Point_<T>>> const& src, cv::Point_<T> const& shift)->std::vector<std::vector<cv::Point_<T>>>
{
    std::vector<std::vector<cv::Point_<T>>> dst;
    for (auto& c : src)
    {
        dst.emplace_back(xvt::Transform(c, shift));
    }
    return dst;
}

template<class T>
inline
auto Transform(std::vector<std::vector<cv::Point_<T>>> const& src, cv::Mat const& m)->std::vector<std::vector<cv::Point_<T>>>
{
    std::vector<std::vector<cv::Point_<T>>> dst;
    for (auto& c : src)
    {
        dst.emplace_back(xvt::Transform(c, m));
    }
    return dst;
}

template<class T>
inline
void RotatePoints(std::vector<cv::Point_<T>>& points, cv::Size imgSize, double angle)
{
    imgSize -= cv::Size(1, 1);
    cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(imgSize.width / 2.0f, imgSize.height / 2.0f), angle, 1.0);
    rotMat.at<double>(0, 2) += -imgSize.width / 2.0f + imgSize.height / 2.0f;
    rotMat.at<double>(1, 2) += imgSize.width / 2.0f - imgSize.height / 2.0f;
    points = Transform(points, rotMat);
}

template<typename ForwardIt, typename Compare >
auto FindAllMax(ForwardIt first, ForwardIt last, Compare com)->std::vector<ForwardIt>
{
    std::vector<ForwardIt> tmp;
    if (first != last)
    {
        ForwardIt largest = first++;
        tmp.emplace_back(largest);
        for (; first != last; ++first)
        {
            if (com(*largest, *first))
            {
                tmp.clear();
                largest = first;
                tmp.emplace_back(first);
            }
            else if (!com(*first, *largest))
            {
                tmp.emplace_back(first);
            }
        }
    }
    return tmp;
}

XVT_EXPORTS
// Return |Sobel(X)|*wx + wy*|Sobel(Y)|
auto Sobel(const cv::Mat& inputImage, float wx = 0.5f, float wy = 0.5f)->cv::Mat;

XVT_EXPORTS
// Return |Sobel(X)|*wx + wy*|Sobel(Y)| and the output is float type
auto Sobel(const cv::Mat& inputImage, float wx, float wy, int kSize, int order)->cv::Mat;

}//xvt
