#pragma once
#include "xvtBattery/CylinderUtils.h"
#include "xvtBattery/CylinderBatteryResult.h"
#include "xvtCV/GammaCorrector.h"
#include "xvtCV/xvtRange.h"

#define USE_POLE_REFINEMENT			1

namespace xvt {
namespace battery {

class XVT_LIB_EXPORTS InspectingItem
{
public:
	void SetALL(bool flag = true);
	void SetBeadingALL(bool flag = true);
	void SetPolesInspect(bool flag = true);
	void SetUpperInspectDefail();
	void SetLowerInspectDefail();
	void SetJRInspectDefail();
public:
    // Distance between the top edge of O to the top of battery
	bool COVER_HEIGHT = false;

    // The inner diameter of the plant
	bool INNER_DIAMETER = false;
	
    // Maximum outer diameter
	bool OUTER_DIAMETER = false;

    // The depth of the groove
	bool GROOVE_DEPTH = false;

    // Distance between upper and lower edges.
	bool GROOVE_HEIGHT = false;

    // Distance between negative pole and positive pole.
	bool ANODE_TO_CATHODE_LENGTH = false;;

    // Distance between negative pole and case
	bool ANODE_TO_CASE_GAP = false;

    // Distance between positive pole and case
	bool CATHODE_TO_CASE_GAP = false;

	// Distance between positive pole and case
	bool ANODE_TO_CASE_VARIATION = false;

    // Pole leaning
	bool CHECK_LEANING = false;

	// Check center pin
	bool CHECK_CENTER_PIN = false;

	// Check beading
	bool CHECK_BEADING = false;

    // Find Battery jr
	bool FIND_JR_ROI = false;

	// Check black cloud
	bool CHECK_BLACK_CLOUD = false;

	// Check erase pole
	bool CHECK_ERASE_POLE = false;
};

class XVT_LIB_EXPORTS CylinderBatteryBase
{
public:
	CylinderBatteryBase();

	virtual ERR_CODE Inspection(const cv::Mat& src, BatteryInspectionResult& BIresult);

	virtual cv::Rect FindBatteryROI(const cv::Mat& image) = 0;

	virtual ERR_CODE FindCathodeXPosition(const cv::Mat& src, VecInt& lstPolePosXAll) = 0;

	virtual ERR_CODE FindAllPolesPos(const cv::Mat& Img,
		cv::Rect& poleRegionROI,
		VecInt& lstPolePosXAll,
		VecInt& anodePos,
		VecInt& cathodePos,
		std::string& descript,
		bool isExtraROI = true
	) = 0;
		
	virtual VecDouble FindCathodeLineByPeak(const cv::Mat& src) = 0;
		
	virtual ERR_CODE FindAnode(const cv::Mat& inputImg,
		VecInt& vtAnodePos,
		VecPoint const& vtStartPoints,
		std::vector<listPole>& listAllPoles,
		cv::Mat& drawOuput
	) = 0;

	virtual ERR_CODE FindCathodeLine(const cv::Mat& srcImg, VecPoint& lstCathodeLine, std::string& descript) = 0;

	virtual std::pair<int, int> FindTopReferenceLine(const cv::Mat& image) = 0;

	void setGamma(float _gamma); // _gamma = [0,39]
private:
	virtual ERR_CODE Inspection2(const cv::Mat& src, BatteryInspectionResult& BIresult) = 0;

public:

	// Enable Inspection
	bool mEnable = true;

	// Battery Threshold
	int mThreshold = 0;

	// Gamma(0~39)
	enhance::GammaCorrector mGamma{};

	// Battery Inspecting Items
	InspectingItem mInspectingItems{};

	// Use Auto Mode[0: No, 1: Yes]
	bool mEnableAutoMode = false;

	// Settings for outer ROI - ROI which certainly contain battery image(unit in pixels).
	cv::Rect mRoi{};

	// Settings for identifying pole inspection area.
	// JR ROI X[pixel]
	int JR_ROIX = 30;

	// JR ROI Y[pixel]
	int JR_ROIY = 15;

	// Threshold to determine pole Leaning
	int mPoleLeaningThreshold = 0;

	// Neglected Center Area Width[pixel]
	int mCenterNeglectionWidth = 200;

	// Pole Region Height[pixel]
	int mPoleRegionHeight = 200;

	// Min Leaning Distance[pixel]
	double mLeaningDistanceMin = 0;

	// Settings for finding Cathode line (Positive pole wave).
	// Cathode Line Inner Threshold
	double mCathodeLineThredsholdInner = 0;

	// Cathode Line Middle Threshold
	double mCathodeLineThredsholdMiddle = 0;

	// Cathode Line Outer Threshold
	double mCathodeLineThredsholdOuter = 0;

	// Cathode Line Window Size[pixel]
	int mCathodeLineWindowSize = 20;

	// Anode Threshold for Inner Poles
	double mAnodeThresholdInner = 0;

	// Anode Threshold for Middle Poles
	double mAnodeThresholdMiddle = 0;

	// Anode Threshold for Outer Poles
	double mAnodeThresholdOuter = 0;

	// Case Line Offset[pixel]
	int mCaseLineOffset = 0;

	// Settings for detecting Pole position (in horizontal axis)
	// Pole Detection Threshold
	double mPolesProminenceThreshold = 0;

	// Poles Distance Range[pixel]
	Rangei mPolesDistanceRange = Rangei(0, 25);

	// Setting for making output decision.
	// Number of pole in one side
	double mOneSidePoleNumber = 24;

	// Is checking the number of poles
	bool mIsCheckPoleNo = false;

	// Skip Poles Distance[pixel]
	int mSkipPolesDistance = 0;

	// Valid Cathode to Anode Range[mm]
	Ranged mValidCathode2AnodeRange = Ranged(0, 10);

	// Valid Anode to Case Range[mm]
	Ranged mValidAnode2CaseRange = Ranged(0, 10);

	// Valid Cathode to Case Range[mm]
	Ranged mValidCathode2CaseRange = Ranged(0, 10);

	// Variation Anode to Case[mm]
	double mVariationAnode2Case = 10;

	// Anode to Case offset[pixel]
	int mAnode2CaseOffset = 0;

	// Cathode to Anode offset[pixel]
	int mCathode2AnodeOffset = 0;

	// Cathode to Anode offset[pixel]
	int mCathode2CaseOffset = 0;

	// Line Type(0: Dotted Line, 1: Curved Line)
	int mLineType;

	// Pixel Size[mm]
	double mPixelSize = 1;

	// Valid Cell Width Range[mm]
	Ranged mValidCellWidthRange = Ranged(0, 100);

	// Debug mode
	// Results display mode [1: Show Outer ROI, 2: Show Pole Grid, 4: Show Text, 7: Show All ]
	DisplayMode mDisplayMode = DisplayMode::ALL;

	// Text result font scale
	double mTextFontScale = 0.7;

	// Text result line space
	int mTextLineSpace = 25;

	// Text result postion
	cv::Point mTextPosition = cv::Point(0, 150);

	// Number Remove Pole Auto
	int mNumberRemovePoleAuto = 1;
};
}
}
