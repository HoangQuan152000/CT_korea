#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

namespace xvt {
namespace threshold {

std::vector<int> XVT_EXPORTS histogramCalculate(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdHuang(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdIntermodes(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdIsoData(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdLi(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdMaxEntropy(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdMean(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdMinimum(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdPercentile(cv::Mat const& src, float percentile = 0.5, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS MinErrorI(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdRenyiEntropy(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdShanbhag(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdTriangle(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
int  XVT_EXPORTS ThresholdYen(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
std::vector<int> XVT_EXPORTS ThresholdOtsuMulti(cv::Mat const& src);
std::vector<int> XVT_EXPORTS ThresholdOtsuMulti(cv::Mat const& src, int level);
std::vector<int> XVT_EXPORTS ThresholdOtsuThreeLevel(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);
void XVT_EXPORTS ThresholdIntegral(cv::Mat const& inputMat, cv::Mat& outputMat, float T, int part);
int XVT_EXPORTS  ThresholdHuang2(cv::Mat const& src, int ignoreValueLower, int ignoreValueUpper);
void XVT_EXPORTS TryAllGolbalThreshold(cv::Mat const& src, int ignoreValueLower = 0, int ignoreValueUpper = 256);

/// <summary>
/// 
/// </summary>
/// <param name="src"></param>
/// <param name="dst"></param>
/// <param name="thresh"></param>
/// <param name="maxval"></param>
/// <param name="type"></param>
/// <param name="mask"></param>
/// <returns></returns>
double XVT_EXPORTS ThresholdWithMask(cv::Mat const& src, cv::Mat& dst, double thresh, double maxval, int type, const cv::Mat& mask = cv::Mat());

double XVT_EXPORTS Threshold(cv::Mat const& src, cv::Mat& dst, int lower, int upper, int type);

}
}