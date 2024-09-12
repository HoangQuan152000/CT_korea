#pragma once
#include "xvtBattery/CylinderBatteryUpper.h"

namespace xvt {
namespace battery {

class XVT_LIB_EXPORTS CylinderBatteryUpperMeasurement :
    public CylinderBatteryUpper
{
public:
    CylinderBatteryUpperMeasurement();

    cv::Mat GetCumulativeHorizontalImage(cv::Size size = cv::Size(100, 800), std::string title = "", std::string lable = "");

    void SetCenterROI(cv::Rect subROI);

private:
    ERR_CODE Inspection2(const cv::Mat& inImg, BatteryInspectionResult& BIresult) override;
public:

    cv::Rect poleRegionROI{};

    cv::Rect CenterROI{};

    int leftCenterRefX = 0;

    int rightCenterRefX = 0;

    std::string path = "";
    
    cv::Mat inputImg{};
    
    cv::Mat outImg{};

    bool autoMeasure = false;
};

}
}

