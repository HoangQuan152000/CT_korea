#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/imgproc.hpp>
#include <utility>
#include <iostream>

namespace xvt {

class XVT_EXPORTS CVPen
{
public:
    CVPen(cv::Scalar color = cv::Scalar(0, 0, 0)
          , int thickness = 1
          , double fontScale = 0.7
          , int space = 10
          , int lineType = cv::LINE_8
          , int fontFace = cv::FONT_HERSHEY_SIMPLEX // cv::FONT_HERSHEY_PLAIN
    );

    //Get the size of tab
    int GetTabSize() const
    {
        return GetTextSize().width * mTabSize;
    }

    //Get the size of a character
    cv::Size GetTextSize() const
    {
        return cv::getTextSize(" ", mFontFace, mFontScale, mThickness, 0);
    }

public:
    int mThickness;
    int mFontFace;
    double mFontScale;
#pragma warning(suppress : 4251)
    cv::Scalar mColor;
    //How to draw the line see cv::LINE
    int mLineType;
    int mSpace;
    int mTabSize = 4;
};

enum class TextAlign
{
    TOP = 1,
    MIDDLE = TOP << 1,
    BOTTOM = MIDDLE << 1,
    LEFT = BOTTOM << 1,
    CENTER = LEFT << 1,
    RIGHT = CENTER << 1,
    TOP_LEFT = TOP + LEFT,
    TOP_CENTER = TOP + CENTER,
    TOP_RIGHT = TOP + RIGHT,
    MIDDLE_LEFT = MIDDLE + LEFT,
    MIDDLE_CENTER = MIDDLE + CENTER,
    MIDDLE_RIGHT = MIDDLE + RIGHT,
    BOTTOM_LEFT = BOTTOM + LEFT,
    BOTTOM_CENTER = BOTTOM + CENTER,
    BOTTOM_RIGHT = BOTTOM + RIGHT,
};

template<TextAlign alginType>
bool IsAlign(TextAlign align)
{
    return (static_cast<int>(align) & static_cast<int>(alginType)) == static_cast<int>(alginType);
}

inline
auto GetTextSize(std::string const& text, CVPen const& pen)->cv::Size
{
    return cv::getTextSize(text, pen.mFontFace, pen.mFontScale, pen.mThickness, 0);
}

XVT_EXPORTS
auto DrawText(cv::Mat& img, std::string const& text, cv::Point p, CVPen const& pen)->cv::Point;

/*** Description:
* src: Input image matrix.
* text: Text string to be printed on the image.
* point: Coordinate to mark.
* offset: Distance from the mark coordinate (Unit: pixels).
* pos: Position of the text: TopLeft  -  TopCenter  -  Top Right
*                            MidLeft  -  MidCenter  -  Mid Right
*                            BotLeft  -  BotCenter  -  Bot Right.
*/
XVT_EXPORTS
void DrawText(cv::Mat& src, const std::string& text, const cv::Point& point, TextAlign pos, CVPen const& pen, cv::Point offset = cv::Point(0, 0));

/*** Description:
* To add the coordinates of the specified point (in green color) and incorporate it for annotating multiple points. 
*/
XVT_EXPORTS
void DrawTextList(cv::Mat& src, const std::vector<std::string>& textList, const std::vector<cv::Point>& points, TextAlign pos, CVPen const& pen, cv::Point offset = cv::Point(0, 0));

inline
void DrawLine(cv::Mat& img, cv::Point p1, cv::Point p2, CVPen const& pen)
{
    cv::line(img, p1, p2, pen.mColor, pen.mThickness, pen.mLineType);
}

template<class PointType>
void DrawPoints(cv::Mat& img, std::vector<PointType> const& pointList, cv::Vec4b const& color, cv::Point const& offset = cv::Point())
{
    if (img.empty()) return;
    if (img.type() == CV_8UC1)
    {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }
    if (img.type() != CV_8UC3) return;

    auto color3 = cv::Vec3b(color[0], color[1], color[2]);
    if (color[3])
    {
        //double const alpha = std::min(std::max(0, color[3]), 100) / 100.0; //cast the alpha to rang 0-1
        //double const beta = 1.0 - alpha;

        for (auto const& p : pointList)
        {
            cv::Point tmp = cv::Point((int)(p.x + offset.x + 0.5), (int)(p.y + offset.y + 0.5));
            if (tmp.x >= 0 && tmp.x < img.cols && tmp.y >= 0 && tmp.y < img.rows)
            {
                auto& c = img.at<cv::Vec3b>(tmp);
                c[0] = c[0] & color3[0];
                c[1] = c[1] & color3[1];
                c[2] = c[2] & color3[2];
            }
        }
    }
    else
    {
        for (auto const& p : pointList)
        {
            cv::Point tmp = cv::Point((int)(p.x + offset.x + 0.5), (int)(p.y + offset.y + 0.5));
            if (tmp.x >= 0 && tmp.x < img.cols && tmp.y >= 0 && tmp.y < img.rows)
                img.at<cv::Vec3b>(tmp) = color3;
        }
    }
}

template<class PointType>
void DrawPoints(cv::Mat& img, std::vector<PointType> const& pointList, cv::Vec3b const& color, cv::Point const& offset = cv::Point())
{
    if (img.empty()) return;
    if (img.type() == CV_8UC1)
    {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }
    if (img.type() != CV_8UC3) return;
    for (auto const& p : pointList)
    {
        cv::Point tmp = cv::Point((int)(p.x + offset.x + 0.5), (int)(p.y + offset.y + 0.5));
        if (tmp.x >= 0 && tmp.x < img.cols && tmp.y >= 0 && tmp.y < img.rows)
            img.at<cv::Vec3b>(tmp) = color;
    }
}

}