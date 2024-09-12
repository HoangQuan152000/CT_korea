#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>

namespace xvt {
namespace enhance {

enum class ConvolutionType
{
	/* Return the full convolution, including border */
	CONVOLUTION_FULL,
	/* Return only the part that corresponds to the original image */
	CONVOLUTION_SAME,
	/* Return only the submatrix containing elements that were not influenced by the border */
	CONVOLUTION_VALID
};

// This is a OpenCV-based implementation of Conv2 in Matlab.
cv::Mat XVT_EXPORTS Conv2(const cv::Mat& img, const cv::Mat& ikernel, ConvolutionType type);

void XVT_EXPORTS AGCIE(const cv::Mat& src, cv::Mat& dst);
void XVT_EXPORTS IAGCWD(const cv::Mat& src, cv::Mat& dst, double alpha_dimmed = 0.75, double alpha_bright = 0.25, int T_t = 112, double tau_t = 0.3, double tau = 0.5);
void XVT_EXPORTS WTHE(const cv::Mat& src, cv::Mat& dst, float r = 0.5, float v = 0.5);
void XVT_EXPORTS LDR(const cv::Mat& src, cv::Mat& dst, double alpha = 2.5);
void XVT_EXPORTS AGCWD(const cv::Mat& src, cv::Mat& dst, double alpha = 0.5);
void XVT_EXPORTS JHE(const cv::Mat& src, cv::Mat& dst);
void XVT_EXPORTS SEF(const cv::Mat& src, cv::Mat& dst, double alpha = 6.0, double beta = 0.5, double lambda = 0.125);
void XVT_EXPORTS PerformSUACE(cv::Mat& src, cv::Mat& dst, int distance = 20, double sigma = 7);
void XVT_EXPORTS TryImageEnhancement(cv::Mat src);

}
}