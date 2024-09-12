#pragma once
#include "xvtCV/xvtPen.h"
#include "xvtCV/xvtDefine.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace xvt {
enum class DrawType
{
    Line,
    Star,
    Caster,
    Plus,
    Circle,
    Pixel,
};

inline
void DrawPlusSign(cv::Mat& img, cv::Point2f const& p, cv::Scalar color, int thickness = 1, int size = 1)
{
    cv::line(img, p - cv::Point2f(size, 0), p + cv::Point2f(size, 0), color, thickness);
    cv::line(img, p - cv::Point2f(0, size), p + cv::Point2f(0, size), color, thickness);
};

inline
void DrawPlusSign(cv::Mat& img, cv::Point2f const& p, CVPen const& pen, int size = 1)
{
    cv::line(img, p - cv::Point2f(size, 0), p + cv::Point2f(size, 0), pen.mColor, pen.mThickness);
    cv::line(img, p - cv::Point2f(0, size), p + cv::Point2f(0, size), pen.mColor, pen.mThickness);
};

// Draw the data in the image image
class XVT_EXPORTS Drawing
{
public:
    Drawing();

    constexpr int GetDrawHeight(int imgHeight)
    {
        return (imgHeight - 2 * padding_Y - 1);
    }

    constexpr int GetDrawWidth(int imgWidth)
    {
        return (imgWidth - 2 * padding_X - 1);
    }

    cv::Point2d GetScale()
    {
        return cv::Point2d(scale_X, scale_Y);
    }

    template <class T1, class T2>
    void CaculateScale(cv::Size const& imgSize, std::vector<T1> const& xList, std::vector<T2> const& yList)
    {
        // find min max in y-coordination to caculation offset and scale in y-coordinate
        auto minmax_Y = std::minmax_element(yList.begin(), yList.end(), [](const T2& t1, const T2& t2) { return t1 < t2; });
        // find min max in x-coordination to caculation offset and scale in x-coordinate
        auto minmax_X = std::minmax_element(xList.begin(), xList.end(), [](const T1& t1, const T1& t2) { return t1 < t2; });

        CaculateScale(*minmax_X.first, *minmax_X.second, *minmax_Y.first, *minmax_Y.second, imgSize);
    }

    template <class T1, class T2>
    void CaculateScale(T1 const& lower_x, T1 const& upper_x, T2 const& lower_y, T2 const& upper_y, cv::Size const& imgSize)
    {
        int height = GetDrawHeight(imgSize.height);
        int width = GetDrawWidth(imgSize.width);

        limit_X = std::pair<double, double>(lower_x, upper_x);
        limit_Y = std::pair<double, double>(lower_y, upper_y);

        // caculation the scale in x and y coordination
        scale_Y = (static_cast<double>(upper_y) - lower_y);
        scale_Y = scale_Y != 0 || scale_Y != height ? height / scale_Y : 1.0;

        scale_X = (static_cast<double>(upper_x) - lower_x);
        scale_X = scale_X != 0 || scale_X != width ? width / scale_X : 1.0;

        int tmpPaddY = padding_Y;
        if (!yTopOrigin)
        {
            scale_Y *= -1;
            tmpPaddY += height;
        }

        // update the offset in x and y coordination
        offSet_Y = tmpPaddY - (lower_y * scale_Y);
        offSet_X = padding_X - (lower_x * scale_X);
    }

    template <class T>
    constexpr int GetDrawX(T const& x)
    {
        return static_cast<int>(std::round(x * scale_X + offSet_X));
    }

    template <class T>
    constexpr int GetDrawY(T const& y)
    {
        return static_cast<int>(std::round(y * scale_Y + offSet_Y));
    }

    template <class T1, class T2>
    constexpr cv::Point GetDrawPoint(T1 const& x, T2 const& y)
    {
        return cv::Point(GetDrawX(x), GetDrawY(y));
    }

    cv::Point2f GetWorldPoint(cv::Point2f const& drawPoint)
    {
        return cv::Point2f((drawPoint.x - offSet_X) / scale_X, (drawPoint.y - offSet_Y) / scale_Y);
    }

    template<class T1, class T2>
    void Plot(cv::Mat& img, const std::vector<T1>& dataX, const std::vector<T2>& dataY, const bool& isUpdateScale = true)
    {
        int sizeX = (int)dataX.size();
        if (sizeX != dataY.size()) throw std::invalid_argument("Size of x data and y data are not equale!");
        if (img.empty() || dataX.empty()) return;
        if (img.rows <= 2 * padding_X || img.cols <= 2 * padding_X) throw std::invalid_argument("ERROR: img.rows <= 2 * padding_X || img.cols <= 2 * padding_X");
        if (img.channels() == 1)
            cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
        else if (img.channels() != 3) throw std::invalid_argument("Only support cv::Mat channle=1 || channel=3");

        if (isUpdateScale)
        {
            CaculateScale(img.size(), dataX, dataY);
        }

        std::vector<cv::Point> lineList;
        lineList.reserve(sizeX);
        for (int i = 0; i < sizeX; i++)
        {
            lineList.push_back(GetDrawPoint(dataX[i], dataY[i]));
        }

        int lineListSize = (int)lineList.size();
        int symbolSize = thickness * 2;
        switch (type)
        {
        case DrawType::Line:
            if (lineListSize > 1)
            {
                lineListSize--;
                for (int i = 0; i < lineListSize; i++)
                {
                    cv::line(img, lineList[i], lineList[(int64)i + 1], color, thickness, mLineType);
                }
            }
            break;
        case DrawType::Circle:

            for (auto&& p : lineList)
            {
                cv::circle(img, p, symbolSize, color, thickness);
            }
            break;
        case DrawType::Star:
            //break;
        case DrawType::Caster:
            //break;
        case DrawType::Plus:
            for (auto&& p : lineList)
            {
                DrawPlusSign(img, p, color, thickness, symbolSize);
            }
            break;
        case DrawType::Pixel:
        default:
            // Default draw pixel value
            for (auto&& p1: lineList)
            {
                if (p1.x >= 0 && p1.x < img.cols && p1.y >= 0 && p1.y < img.rows)
                    img.at<cv::Vec3b>(p1) = cv::Vec3b(color[0], color[1], color[2]);
            }
            break;
        }
    }

    void Plot(cv::Mat& img, const std::vector<cv::Point2f>& points, const bool isUpdateScale = true);

    void Plot(cv::Mat& img, const std::vector<cv::Point>& points, const bool isUpdateScale = true);

    void Plot(cv::Mat& img, const cv::Point2f& p);

    void Plot(cv::Mat& img, const cv::Point2f& p, const cv::Scalar& c);

    void Plot(cv::Mat& img, std::string const& text, cv::Point const& imgPos, float fontScale = 1, const bool isDrawPoint = false);

    // x is in range [min_x max_x)
    // y is in yList
    template<class T>
    void Plot(cv::Mat& img, int min_x, int max_x, std::vector<T> const& yList, bool isDrawRuler=true)
    {
        size_t ySize = yList.size();
        if (ySize < 1) return;

        size_t xLength = ((int64)max_x - min_x);
        const double step = (double)xLength / ySize;

        std::vector<double> xData(ySize, 0);
        for (int i = 0; i < ySize; i++)
        {
            xData[i] = min_x + i * step;
        }

        double minY = 0, maxY = 0;
        {
            auto minmaxY = std::minmax_element(yList.begin(), yList.end());
            minY = *minmaxY.first;
            maxY = *minmaxY.second;
        }

        if (isDrawRuler)
        {
            DrawRuler(img, min_x, max_x, minY, maxY, true);
        }

        Plot(img, xData, yList, !isDrawRuler);
    }

    template<class T>
    void Plot(cv::Mat& img, std::vector<T> const& xList, int min_y, int max_y, bool isDrawRuler = true)
    {
        size_t xSize = xList.size();
        if (xSize < 1) return;

        size_t yLength = ((int64)max_y - min_y);
        const double step = (double)yLength / xSize;

        std::vector<double> yData(xSize, 0);
        for (int i = 0; i < xSize; i++)
        {
            yData[i] = min_y + i * step;
        }

        double minX = 0, maxX = 0;
        {
            auto minmaxY = std::minmax_element(xList.begin(), xList.end());
            minX = *minmaxY.first;
            maxX = *minmaxY.second;
        }

        if (isDrawRuler)
        {
            DrawRuler(img, minX, maxX, min_y, max_y, true);
        }

        Plot(img, xList, yData, !isDrawRuler);
    }

    void Plot(cv::Mat& img, cv::Rect const& rect);
    void Plot(cv::Mat& img, cv::Rect const& rect, cv::Scalar const& c);
    void Plot(cv::Mat& img, std::vector<cv::Rect> const& rectList, cv::Scalar const& c);

    inline void ResetDefault();
    // Start point is draw point in the image
    void DrawRuler(cv::Mat& img, cv::Point start, cv::Point end);
    // x, y boundary is world coordiante
    void DrawRuler(cv::Mat img, double lower_x, double upper_x, double lower_y, double upper_y, const bool isUpdateScale = false);

    // Static function
    static void ShowImg(const std::string& imgTitle, const cv::Mat& img);
    static void DrawDashedLine(cv::Mat& img, cv::Point pt1, cv::Point pt2, cv::Scalar color, int thickness, std::string style, int gap);
    static void DrawDimention(cv::Mat& img, cv::Point p1, cv::Point p2, cv::Scalar color, int thickness = 1, std::string str = "", int padding = 0, int length = 0);
    static void DrawDimention(cv::Mat& img, cv::Point p1, cv::Point p2, float value, cv::Scalar color, int thickness = 1, int padding = 0, int length = 0);

protected:
    //Direction should be normalize
    static void DrawMaker(cv::Mat& img, cv::Point const& p, cv::Scalar color, cv::Point2f direction = cv::Point2f(1.0f, 0.0f), int len = 5, int thickness = 1)
    {
        cv::Point leng = cv::Point(direction * len);
        cv::line(img, p, p + leng, color, thickness);
    }

    static bool cvt2BGR(cv::Mat& img)
    {
        bool rtn = !img.empty();

        if (rtn)
        {
            int type = img.type();
            switch (type)
            {
            case CV_8UC1:
                break;
            case CV_8UC3:
                break;
            case CV_16UC1:
                img.convertTo(img, CV_8UC1, 1.0 / 256.0);
                break;
            default:
                rtn = false;
                break;
            }

            if (img.type() == CV_8UC1)
            {
                cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
            }
        }

        return rtn;
    }

public:
    //cv::Point2f offset;
    bool		yTopOrigin;
    int			thickness;
    int			padding_X; //pixel
    int			padding_Y; //pixel
    DrawType	type;
    cv::Scalar	color;
    cv::LineTypes mLineType = cv::LineTypes::LINE_AA;
private:
    double scale_X;
    double scale_Y;
    double offSet_X;
    double offSet_Y;

    std::pair<double, double> limit_X;
    std::pair<double, double> limit_Y;
};

XVT_EXPORTS
cv::Mat DrawHistogram(std::vector<int> histogramValue, int minRange, int maxRange, bool isNormalize = true);

XVT_EXPORTS
cv::Mat GetHistoImage(const cv::Mat& src, int minTh, int maxTh);

XVT_EXPORTS
void Draw(cv::Mat& src, cv::RotatedRect const& rect, CVPen pen, cv::Point2f offset = cv::Point2f());

}//namespace xvt