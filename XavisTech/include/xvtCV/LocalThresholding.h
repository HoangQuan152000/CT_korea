#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>

namespace xvt {
namespace threshold {

/// <summary>
/// Niblack recommends K_VALUE = -0.2 for images with black foreground objects
/// and K_VALUE = +0.2 for images with white foreground objects.
/// Niblack W. (1986) "An introduction to Digital Image Processing" Prentice-Hall.
/// </summary>
/// <param name="src"></param>
/// <param name="dst"></param>
/// <param name="windowsSize"></param>
/// <param name="K"></param>
/// <param name="r"></param>
void XVT_EXPORTS ThresholdNiBlack(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.3, int r = 128);

/// <summary>
/// Sauvola J. and Pietaksinen M. (2000)
/// Adaptive Document Image Binarization
/// Pattern Recognition, 33(2): 225-236
/// <para> Sauvola recommends K_VALUE = 0.5 and R_VALUE = 128. </para>
/// </summary>
/// <param name="src">[in] input image</param>
/// <param name="dst">[out] binary image</param>
/// <param name="windowsSize">[in] window size</param>
/// <param name="K"></param>
/// <param name="r"></param>
/// <created>Lam,Fri, 5, 14, 2020</created>
/// <changed>[Gitlab#24] Tuyen,Fri, 5, 14, 2021</changed>
void XVT_EXPORTS ThresholdSauvola(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.25, int r = 128);

/// <summary>
/// 
/// </summary>
/// <param name="src"></param>
/// <param name="dst"></param>
/// <param name="windowsSize"></param>
/// <param name="K"></param>
/// <param name="r"></param>
void XVT_EXPORTS ThresholdWolf(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.1, int r = 128);

void XVT_EXPORTS ThresholdNick(cv::Mat const& src, cv::Mat& dst, cv::Size windowsSize, double K = -0.1, int r = 128);

/// <summary>
/// 
/// </summary>
/// <param name="prename"></param>
/// <param name="src"></param>
/// <param name="size"></param>
void XVT_EXPORTS TryAllLocalThreshold(cv::Mat const& src, cv::Size size);

}//Threshold
}//cvt