#pragma once
#include "xvtBattery/CylinderBatteryBase.h"

namespace xvt {
namespace battery {

class XVT_LIB_EXPORTS CylinderBatteryUpper :
	public CylinderBatteryBase
{
public:
	CylinderBatteryUpper();

	virtual cv::Rect FindBatteryROI(const cv::Mat& image) override final;
			
	virtual ERR_CODE FindCathodeXPosition(const cv::Mat& src, VecInt& lstPolePosXAll) override final;
			
	virtual ERR_CODE FindAllPolesPos(const cv::Mat& Img, cv::Rect& poleRegionROI, VecInt& lstPolePosXAll, VecInt& anodePos, VecInt& cathodePos, std::string& descript, bool isExtraROI = true) override final;
			
	virtual VecDouble FindCathodeLineByPeak(const cv::Mat& src) override final;
			
	virtual ERR_CODE FindAnode(const cv::Mat& inputImg, VecInt& vtAnodePos, VecPoint const& vtStartPoints, std::vector<listPole>& listAllPoles, cv::Mat& drawOuput) final;
	
	virtual ERR_CODE FindCathodeLine(const cv::Mat& srcImg, VecPoint& lstCathodeLine, std::string& descript) override final;

	virtual std::pair<int, int> FindTopReferenceLine(const cv::Mat& image) override final;
protected:
	/// <summary>
	/// findBeadingInspection: inspect beading and get case reference line (yA2)
	/// BIresult[0]: Thickness of top cover
	/// BIresult[1]: The inner diameter of the plant
	/// BIresult[2]: Maximum outer diameter
	/// BIresult[3]: The depth of the groove
	/// BIresult[4]: Distance between upper and lower edges
	/// </summary>
	/// <param name="Img">Original image</param>
	/// <param name="resImage">Result image</param>
	/// <param name="batteryROI">Battery input setting ROI</param>
	/// <param name="BatInsp"></param>
	/// <param name="BIresult"></param>
	/// <returns></returns>
	int InspectBeading(const cv::Mat& Img, cv::Mat& resImage, cv::Rect batteryROI, VecDouble& BIresult);
	
	int FindAnodeByKmean(const cv::Mat& inputImg, cv::Point startPoint, int defaultVal, int stepHorizontal, int stepVertical, cv::Mat& drawOuput);

private:
	virtual ERR_CODE Inspection2(const cv::Mat& src, BatteryInspectionResult& BIresult) override;
public:
	// Min Beading Height[pixel]
	int mBeadingHeightMin = 20;

	// D1 Starting Index[%l]
	float mD1StartPosition = 0.5;

	// Valid COVER_HEIGHT Range[mm]
	Ranged mValidCoverHeightRange = Ranged(0, 10);

	// Valid GROOVE_DEPTH Range[mm]
	Ranged mValidGrooveDepthRange = Ranged(0, 10);

	// Valid GROOVE_HEIGHT Range[mm]
	Ranged mValidGrooveHeightRange = Ranged(0, 10);

	// Valid INNER_DIAMETER Range[mm]
	Ranged mValidInnerDiameterRange = Ranged(0, 10);

private:
	// Is Check Battery Beading
	bool mIsCheckBeading = false;
};
}
}