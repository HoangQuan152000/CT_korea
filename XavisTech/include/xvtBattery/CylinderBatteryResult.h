#pragma once
#include "xvtBattery/CylinderUtils.h"
#include "xvtCV/xvtEnumTemplate.h"

namespace xvt
{
namespace battery
{
struct XVT_LIB_EXPORTS BatteryInspectionResult
{
    std::string sHeader;
    std::string sData; // Data
    cv::Mat resImg{};
    cv::Rect outerROI;
    int yA2 = 0;
    int iResult;
    int nLeftPole;
    std::string Description;            // Final Decision
    double corverHeight;                // Cover Height
    double corverDiameter;              // Cover Diameter
    double outerDiameter;               // Max Outer Diameter
    double grooveDepth;                 // Groove Depth
    double grooveHeight;                // Groove Height
    std::vector<double> sCathode2Anode; // Positive to Negative
    std::vector<double> sAnode2Case;    // Negative to Case
    std::vector<double> sCathode2Case;  // Positive to Case
    std::vector<double> sXPos;          // Position of horizontal axis to left border battery
    std::vector<bool> vtAno2CathodDecision;
    std::vector<bool> vtAno2CaseDecision;
    std::vector<bool> vtCathode2CaseDecision;
    double minCathode2Anode;                  // Min Positive to Negative
    double maxCathode2Anode;                  // Max Positive to Negative
    double avgCathode2Anode;                  // Avg Positive to Negative
    double minAnode2Case;                     // Min Negative to Case
    double maxAnode2Case;                     // Max Negative to Case
    double avgAnode2Case;                     // Avg Negative to Case
    double minCathode2Case;                   // Min Positive to Case
    double maxCathode2Case;                   // Max Positive to Case
    double avgCathode2Case;                   // Avg Positive to Case
    double blackCloudHeight;                  // Black Cloud Height
    double blackCloudAreaRatio;               // Black Cloud Area Ratio
    std::vector<cv::Point> vtCathodes;        // Positive points
    std::vector<cv::Point> vtAnodes;          // Negative points
    std::string Anode2CathodeDecision;        // True if no error, otherwise false
    std::string Anodes2CaseDecision;          // True if no error, otherwise false
    std::string Cathode2CaseDecision;         // True if no error, otherwise false
    std::string Anode2CaseVariationDecision;  // True if no error, otherwise false
    std::string BlackCloudDecision;          // True if no error, otherwise false
    std::string BeadingCoverDiameterDecision; // True if no error, otherwise false
    std::string BeadingGrooveDepthDecision;   // True if no error, otherwise false
    std::string BeadingGrooveHeightDecision;  // True if no error, otherwise false
    std::string BeadingInnerDiameterDecision; // True if no error, otherwise false
    std::string finalDecision;                // Final Decision: true if no error, otherwise false
    double centerPinWidth;                    // Center PIN Width
    double centerPinDepth;                    // Center PIN Depth
    bool isPinExist;                          // true if there is pin, otherwise false
    std::string cTime;

    BatteryInspectionResult();
};

XVT_LIB_EXPORTS std::string textWrap(std::string str, int location);

XVT_LIB_EXPORTS void saveErroImage(const cv::Mat &resImg, const std::string &outFileName, std::string errDesc, ERR_CODE errCode);
} // namespace battery
} // namespace xvt