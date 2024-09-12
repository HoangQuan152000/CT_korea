#pragma once
//convert c# to c++ https://github.com/shimat/opencvsharp/blob/ea7c67ff0df7b0bc003b46b245a413c7830ffb16/src/OpenCvSharp.Extensions/BitmapConverter.cs

#ifndef __BITMAP_CONVERTER_H__
#define __BITMAP_CONVERTER_H__

#ifdef __CLR__
#include<opencv2/core/mat.hpp>
using namespace System;
using namespace System::Drawing;
using namespace System::Drawing::Imaging;
using namespace System::Runtime::InteropServices;

namespace xvt {

class BitmapConverter
{
    /// <summary>
    /// Converts System::Drawing::Bitmap to Mat
    /// </summary>
    /// <param name="src">System::Drawing::Bitmap object to be converted</param>
    /// <returns>A Mat object which is converted from System::Drawing::Bitmap</returns>
public: static cv::Mat ToMat(Bitmap^ src);

    /// <summary>
    /// Converts System.Drawing.Bitmap to Mat
    /// </summary>
    /// <param name="src">System.Drawing.Bitmap object to be converted</param>
    /// <param name="dst">A Mat object which is converted from System.Drawing.Bitmap</param>
public: static void ToMat(Bitmap^ src, cv::Mat dst);

    /// <summary>
    /// Converts Mat to System.Drawing.Bitmap
    /// </summary>
    /// <param name="src">Mat</param>
    /// <returns></returns>
public: static Bitmap^ ToBitmap(cv::Mat src);

    /// <summary>
    /// Converts Mat to System.Drawing.Bitmap
    /// </summary>
    /// <param name="src">Mat</param>
    /// <param name="pf">Pixel Depth</param>
    /// <returns></returns>
public: static Bitmap^ ToBitmap(cv::Mat src, PixelFormat pf);

    /// <summary>
    /// Converts Mat to System.Drawing.Bitmap
    /// </summary>
    /// <param name="src">Mat</param>
    /// <param name="dst">Mat</param>
    /// <remarks>Author: shimat, Gummo (ROI support)</remarks>
public: static void ToBitmap(cv::Mat src, Bitmap^ dst);

};
};
#endif
#endif // !__BITMAP_CONVERTER_H__
