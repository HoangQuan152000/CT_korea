#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <functional>

namespace xvt {

enum class ThinningTypes{
    THINNING_ZHANGSUEN    = 0, // Thinning technique of Zhang-Suen
    THINNING_GUOHALL      = 1  // Thinning technique of Guo-Hall
};

/** @brief Applies a binary blob thinning operation, to achieve a skeletization of the input image.
The function transforms a binary blob image into a skeletized form using the technique of Zhang-Suen.
@param src Source 8-bit single-channel image, containing binary blobs, with blobs having 255 pixel values.
@param dst Destination image of the same size and the same type as src. The function can work in-place.
@param thinningType Value that defines which thinning algorithm should be used. See cv::ximgproc::ThinningTypes
 */
XVT_EXPORTS
void Thinning(const cv::Mat& src, cv::Mat& dst, ThinningTypes thinningType = ThinningTypes::THINNING_GUOHALL);

}

