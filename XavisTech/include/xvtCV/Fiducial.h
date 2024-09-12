#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/base.hpp>
#include <string>

namespace xvt {
namespace feature {

enum class WarpFiducialType : int
{
      NONE = 0 // No need to warp 
    , FIDUCIAL_MANUAL = 1  // Warp by 6 points get by manual template selected by user
    , FIDUCIAL_AUTO = 2  // Warp by autoget rectangle object in image 
    , FIDUCIAL_GRID = 3  // Warp by grid of footprint
};

struct Pair_Fifucial
{
    std::string imgname="";
    cv::Point2f PointREF_warp{};
    cv::Point2f PointCentermatch{};
    cv::Mat img{};
    cv::Mat templateImage{};
    bool bMatchTemplate=false;
};

class XVT_EXPORTS Fiducial
{
public:
    Fiducial();

    // Clone the pointer
    virtual Fiducial* Clone();

    virtual bool WarpImage(std::string path, std::string imgname, cv::Mat img, cv::Mat& outImage);

private:

    void FindSetofPoint(Pair_Fifucial* PairF);
    bool Warp_Manual(std::string path, std::string imgname, cv::Mat img, cv::Mat& outImage, std::vector<cv::Point2f> vtPointREF_warp, std::vector<std::string> vtTemplateImagePath);
    bool Warp_FiducialObject(std::string path, std::string filename, cv::Mat img, cv::Mat& outImage, std::vector<cv::Point2f> warpCorner);
    void Pre_process_Fulloverlap(cv::Mat srcImg, cv::Mat& dst);
    cv::Rect FindLargestContour(cv::Mat BinaryImg, cv::Mat& output, cv::Scalar color);
    bool Matching_template(cv::Mat imgsrc, cv::Mat templateimage, std::string imagename, double& score, int x, int y, cv::Point2f& centerpoint);

public:
    WarpFiducialType warpType;
    std::vector<cv::Point2f> vtPointREF_warp;
    std::vector<std::string> vtTemplateImagePath;
};
}
}
