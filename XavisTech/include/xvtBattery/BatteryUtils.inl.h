#pragma once
#include "xvtBattery/BatteryUtils.h"
#include <numeric>

namespace xvt {
namespace battery {

template <typename T> inline
AverageCathodeLineResult<T> FindAverageCathodeLine(std::vector<T> &cathodeLineList,
                                                   int neglectionWidth,
                                                   int startNeglectCol,
                                                   int endNeglectCol,
                                                   int rightEndBounder,
                                                   int localAverageWindowSize,
                                                   int wSize)
{
    AverageCathodeLineResult<T> ret;
    /*20220316-PhuNM-Change the global average to local average*/
    //Refine outlier poles
    int cIdx = 0;
    int xPos = 0;
    int neglectionSize = 0;
    int leftCenterBounder = startNeglectCol - neglectionWidth;
    int rightCenterBounder = endNeglectCol + neglectionWidth;

    for (auto &cathodeLine : cathodeLineList)
    {
        xPos = cIdx * wSize;
        if ((xPos > neglectionWidth && xPos < leftCenterBounder) || (xPos > rightCenterBounder && xPos < rightEndBounder))
        {
            ret.lst4MeanStd.push_back(cathodeLine);
            ret.sum += cathodeLine;
            ret.lstCathodeSize++;
        }

        if (xPos < startNeglectCol)
        {
            ret.localAverageCathodeLineLeft.push_back(cathodeLine);
        }
        else if (xPos > endNeglectCol)
        {
            ret.localAverageCathodeLineRight.push_back(cathodeLine);
        }
        else
        {
            neglectionSize++;
        }
        cIdx++;
    }

    if (ret.lstCathodeSize > 0)
    {
        cv::blur(ret.localAverageCathodeLineLeft,
                 ret.localAverageCathodeLineLeft,
                 cv::Size(localAverageWindowSize, 1),
                 cv::Point(-1, -1),
                 cv::BORDER_REFLECT);
        cv::blur(ret.localAverageCathodeLineRight,
                 ret.localAverageCathodeLineRight,
                 cv::Size(localAverageWindowSize, 1),
                 cv::Point(-1, -1),
                 cv::BORDER_REFLECT);
        ret.localAverageCathodeLine.reserve(cathodeLineList.size());
        ret.localAverageCathodeLine.insert(ret.localAverageCathodeLine.end(),
                                           ret.localAverageCathodeLineLeft.begin(),
                                           ret.localAverageCathodeLineLeft.end());
        ret.localAverageCathodeLine.insert(ret.localAverageCathodeLine.end(),
                                           cathodeLineList.begin() + ret.localAverageCathodeLineLeft.size(),
                                           cathodeLineList.begin() + ret.localAverageCathodeLineLeft.size() + neglectionSize);
        ret.localAverageCathodeLine.insert(ret.localAverageCathodeLine.end(),
                                           ret.localAverageCathodeLineRight.begin(),
                                           ret.localAverageCathodeLineRight.end());
        if (ret.localAverageCathodeLine.size() == cathodeLineList.size())
        {
            ret.success = true;
        }
    }

    return ret;
}

template <typename T> inline
void RefineCathodeLine(AverageCathodeLineResult<T> &avgCathodeLineResult, std::vector<T> &lstCathodeLine, int wSize, int centerLeft, int centerRight)
{
    int listSize = lstCathodeLine.size() - 1;
    if (listSize < 3)
        return;

    double mean = avgCathodeLineResult.sum / (avgCathodeLineResult.lstCathodeSize);
    double sq_sum = std::inner_product(avgCathodeLineResult.lst4MeanStd.begin(),
                                       avgCathodeLineResult.lst4MeanStd.end(),
                                       avgCathodeLineResult.lst4MeanStd.begin(),
                                       0.0);
    double stdev = std::sqrt(sq_sum / avgCathodeLineResult.lstCathodeSize - mean * mean);

    float outlierRange = stdev * 2;

    std::vector<int> outliers;
    outliers.reserve(lstCathodeLine.size());
    int validIdxBefore = -1;
    int validIdxAfter = -1;
    float refineValue = 0;

    int centerLeftIdx = ceil((float)centerLeft / wSize);
    int centerRightIdx = floor((float)centerLeft / wSize);


#ifdef DEBUG_FIND_ANODE_X_POS
    auto minmax = std::minmax_element(lstCathodeLine.begin(), lstCathodeLine.end());
    double h = std::max(abs(*minmax.second - *minmax.first) + outlierRange * 2.5, 100.0);
    cv::Mat debugImg = cv::Mat(h, listSize * wSize, CV_8UC3);
    int wIdxDraw = 0;
    int yOffSet = (h - *minmax.first - *minmax.second) / 2;
    for (int idx = 0; idx <= listSize; idx++)
    {
        cv::line(debugImg,
                 cv::Point(wIdxDraw, avgCathodeLineResult.localAverageCathodeLine[idx] + outlierRange + yOffSet),
                 cv::Point(wIdxDraw + wSize, avgCathodeLineResult.localAverageCathodeLine[idx] + outlierRange + yOffSet),
                 cv::Scalar(0, 255, 255));
        cv::line(debugImg,
                 cv::Point(wIdxDraw, avgCathodeLineResult.localAverageCathodeLine[idx] - outlierRange + yOffSet),
                 cv::Point(wIdxDraw + wSize, avgCathodeLineResult.localAverageCathodeLine[idx] - outlierRange + yOffSet),
                 cv::Scalar(0, 255, 255));
        cv::line(debugImg,
                 cv::Point(wIdxDraw, avgCathodeLineResult.localAverageCathodeLine[idx] + yOffSet),
                 cv::Point(wIdxDraw + wSize, avgCathodeLineResult.localAverageCathodeLine[idx] + yOffSet),
                 cv::Scalar(0, 255, 0));
        cv::line(debugImg,
                 cv::Point(wIdxDraw, lstCathodeLine[idx] + yOffSet),
                 cv::Point(wIdxDraw + wSize, lstCathodeLine[idx] + yOffSet),
                 cv::Scalar(255, 0, 0));
        wIdxDraw += wSize;
    }
#endif // DEBUG_FIND_ANODE_X_POS

    for (int idx = 0; idx <= listSize; idx++)
    {
        if (lstCathodeLine[idx] > avgCathodeLineResult.localAverageCathodeLine[idx] + outlierRange ||
            lstCathodeLine[idx] < avgCathodeLineResult.localAverageCathodeLine[idx] - outlierRange)
        {
            outliers.push_back(idx);
        }
        else
        {
            if (!outliers.empty())
            {
                validIdxAfter = idx;
            }
            else
            {
                validIdxBefore = idx;
            }
        }

        if (!outliers.empty() && (validIdxAfter > 0 || (idx == listSize || idx == centerLeftIdx) && validIdxBefore >= 0))
        {
            for (int oIdx : outliers)
            {
                if (validIdxBefore < 0)
                {
                    refineValue = lstCathodeLine[validIdxAfter];
                }
                else if (validIdxAfter < 0 || validIdxAfter == centerLeftIdx)
                {
                    refineValue = lstCathodeLine[validIdxBefore];
                }
                else
                {
                    float rightRatio = (float)(oIdx - validIdxBefore) / (validIdxAfter - validIdxBefore);
                    refineValue = ((float)lstCathodeLine[validIdxBefore] * (1 - rightRatio) + (float)lstCathodeLine[validIdxAfter] * rightRatio);
                }
                lstCathodeLine[oIdx] = refineValue;
            }

            outliers.clear();
            validIdxBefore = validIdxAfter;
            validIdxAfter = -1;
        }
    }

#ifdef DEBUG_FIND_ANODE_X_POS
    wIdxDraw = 0;
    for (int idx = 0; idx <= listSize; idx++)
    {
        cv::line(debugImg,
                 cv::Point(wIdxDraw, lstCathodeLine[idx] + yOffSet),
                 cv::Point(wIdxDraw + wSize, lstCathodeLine[idx] + yOffSet),
                 cv::Scalar(255, 0, 255));
        wIdxDraw += wSize;
    }
    cv::imshow("CathodeLine Refine", debugImg);
#endif // DEBUG_FIND_ANODE_X_POS
}

template <typename T> inline
std::pair<double, double> MeanStdDevLite(const std::vector<T>& signal)
{
    cv::Scalar m = cv::Scalar(0);
    cv::Scalar stdDev = cv::Scalar(0);
    if (!signal.empty())
    {
        cv::meanStdDev(signal, m, stdDev);
    }
    return std::make_pair(m[0], stdDev[0]);
}

}
}