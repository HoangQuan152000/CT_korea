#pragma once
#include "xvtCV/xvtDefine.h"
//#pragma warning( push )
//#pragma warning( disable : )
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
//#pragma warning( pop )
#include <iostream>

namespace xvt {
namespace filter {
class XVT_EXPORTS Regions
{

public:
    Regions();
    Regions(int _kernel);
    ~Regions();

    /// <summary>
    /// Copy constructor
    /// </summary>
    /// <param name=""></param>
    Regions(const Regions& src);

    /// <summary>
    /// Assign
    /// </summary>
    /// <param name="src"></param>
    /// <returns></returns>
    Regions& operator = (Regions const& src);

    /// <summary>
    /// Update data, increase the size of the area, update the sum
    /// </summary>
    /// <param name="area"></param>
    /// <param name="data"></param>
    void SendData(int area, int data);

    /// <summary>
    /// Calculate the variance of each area
    /// </summary>
    /// <param name="area"></param>
    /// <returns></returns>
    double GetAreaVariance(int area);

    /// <summary>
    /// Call the above function to calc the variances of all 4 areas
    /// </summary>
    void CalculateVariance();

    /// <summary>
    /// Find out which regions has the least variance
    /// </summary>
    /// <returns></returns>
    int GetMinVariance();

    /// <summary>
    /// Return the mean of that regions
    /// </summary>
    /// <returns></returns>
    uchar GetResult();

private:
    int* Area[4];
    int Size[4];
    unsigned long long Sum[4];
    double Var[4];
    int kernel;

};

/// <summary>
/// 
/// </summary>
class XVT_EXPORTS Kuwahara
{

public:
    //Constructor
    Kuwahara(const cv::Mat& _image, int _kernel);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <returns></returns>
    Regions GetRegions(int x, int y);

    /// <summary>
    /// Create new image and replace its pixels by the results of Kuwahara filter on the original pixels
    /// </summary>
    /// <returns></returns>
    cv::Mat Apply();

private:
    int wid, hei, pad, kernel;

#pragma warning (suppress :4251)
    cv::Mat image;
};
}
}