#pragma once
#include "xvtCV/Utils.h"
#include "xvtCV/xvtDefine.h"
#include <list>

namespace xvt {

struct data_image
{
    cv::Mat src_color;
    cv::Mat src;
    cv::Mat src_edge;
    cv::Mat src_bin;
    cv::Rect roi;
    //cv::Mat src_edge_color;
};

struct point_angle
{
    //cv::Point pt;
    cv::Point2d dp{};
    //int avg_intensity;
    uchar value{ 0 };
    int angle{ 0 };
};

cv::Mat get_bin_img_adaptive_threshold(const cv::Mat& src);
std::vector<cv::Point2d> get_arr_direction_dppoint(cv::Point2d start_point, int angle, int num_of_next_point);
uchar get_intensity_double_point_image(const cv::Mat& src, double x, double y);
double calc_value_direction_edge(const std::vector<cv::Point2d>& direction, const cv::Mat& src, int angle_degree);
double GetDirectionValue(const cv::Point2d& start_point, const cv::Mat& img, int angle, int num_of_next_point);
double GetDirectionValue(const cv::Mat& img, const cv::Point2f& start_point, const cv::Point2f& refVector, int angle, int num_of_next_point, cv::Point2f& newVector);
uchar get_intensity_cv_point_image(const cv::Mat& src, cv::Point dp);
cv::Point double_point_to_cv_point(const cv::Point2d& pt);
cv::Point2d get_next_dpoint_from_angle(const cv::Point2d& pt, int angle_degree);
double calc_distance(const cv::Point2d& start, const cv::Point2d& end);
double calc_distance(const cv::Point2f& start, const cv::Point2f& end);
double push_to_queue_get_num_of_white_pixel(const cv::Mat& src, std::list<cv::Point2d>& list_dp, cv::Point2d dp, int max_size);
bool ReadPointsFromFile(std::string path, std::vector<cv::Point>& start, std::vector<cv::Point>& end);
bool GetSartPoint(const cv::Mat& img, cv::Point2f pt, int size, cv::Point2f& outPoint);

void XVT_EXPORTS process_edge_tracking(const cv::Mat& src,
                                       const cv::Point2d& src_pt,
                                       const int& angle_degree,
                                       double intentThreshold,
                                       std::vector<point_angle>& vt_result,
                                       std::vector<point_angle>& vt_result_filter,
                                       std::vector<point_angle>& vt_result_final
);

}
