#pragma once
#include "xvtBattery/CylinderBatteryBase.h"

//#define DEBUG_DOMINANT_POLES

namespace xvt {
namespace battery {

class XVT_LIB_EXPORTS CylinderBatteryLower :
    public CylinderBatteryBase
{
public:
	CylinderBatteryLower();
		
	virtual cv::Rect FindBatteryROI(const cv::Mat& image) override;
		
	virtual ERR_CODE FindCathodeXPosition(const cv::Mat& src, VecInt& lstPolePosXAll) override;
		
	virtual ERR_CODE FindAllPolesPos(const cv::Mat& Img, cv::Rect& poleRegionROI, VecInt& lstPolePosXAll, VecInt& anodePos, VecInt& cathodePos, std::string& descript, bool isExtraROI = true) override;
		
	virtual VecDouble FindCathodeLineByPeak(const cv::Mat& src) override;
	
	virtual ERR_CODE FindAnode(const cv::Mat& inputImg, VecInt& vtAnodePos, VecPoint const& vtStartPoints, std::vector<listPole>& listAllPoles, cv::Mat& drawOuput);
	
	virtual ERR_CODE FindCathodeLine(const cv::Mat& srcImg, VecPoint& lstCathodeLine, std::string& descript) override;
		
	virtual std::pair<int, int> FindTopReferenceLine(const cv::Mat& image) override;
protected:
	int LineRefinement(VecPoint& lstPolePosAll, int widthROI);

	int FindCenterPin(const cv::Mat& rotatedImg, cv::Rect batteryROI, int cp_startingcenter, int thresh_horiwidth, int thresh_verticalwidth, int thresh_st_checkending, int threshending, cv::Point& pointleft, cv::Point& pointright, cv::Point& pointtop);
	
	void RecheckDisconectAnode(const cv::Mat& Roi4Res, cv::Mat& anodeDebugImage, const VecPoint& CathodePnts, VecInt& anodePos, const int& stepVertical);
	
	ERR_CODE CheckBlackCloud(const cv::Mat& rotatedImg, cv::Rect& PoleROI, VecPoint& BlackRegion, int Thd, double& resultHeight, double& resultRatioArea);
	//check tab appearence detected as poles (added 06/28/2021, Bach)
#ifdef DEBUG_DOMINANT_POLES
	std::vector<std::string> debugTabPoles(const cv::Mat& drawImg, VecFloat meanROIs, int center, VecInt xCoords, VecInt yCoords, std::string fileName);
#endif // !DEBUG_DOMINANT_POLES
	
	VecInt CheckDominantPoles(const VecInt& anodeVec, const VecInt& cathodeVec, const VecFloat& avgInt, const VecInt& vectorXPos, int center,
		float firstThresh = 0.8, float leftMidThresh = 0.85, float rightMidThresh = 0.83, float lastThresh = 0.78);
	
	// Check tab issue in boundery
	void CheckBoundaryTab(const VecFloat& poleHeight, VecInt& anodePos, int heightAvg, int numTabPoles = 5);

	/// <summary>
	///Check if cathode line is too high within ROI. 
	///Function return "true" if amount of OK is acceptable, "false" otherwise
	/// </summary>
	///<param name="cathodeList"> = input cathode vector</param>
	///<param name="cathodeYCordThresh"> = minimum height threshold, value less than threshold will be count as defect</param>
	///<param name="OKPercentThresh"> = OK amount percentage</param>
	bool CheckCathodeLine(const VecDouble& cathodeList, int cathodeYCordThresh = 5, float OKPercentThresh = 0.9);

	std::vector<std::pair<int, int>> FindTabs(const std::vector<float>& Signal);
	
	ERR_CODE CheckIsTab(const std::vector<float>& Signal, const std::vector<int> PVIdx, float thresh_inner, float thresh_middle, float thresh_outer);
	
private:
	virtual ERR_CODE Inspection2(const cv::Mat& src, BatteryInspectionResult& BIresult) override;
public:
	bool mIsCheckPoleTAB = false;

	// manual threshold
	int mBlackCloudThreshold = 80;

	// offset to crop Pole ROI
	int mBlackCloudOffset = 0;

	int mBlackCloudCheckCondition = 0;

	double mRatioAreaThreshold = 0.1;

	int mHeightThreshold = 100;
private:
	// Battery Direction
	int	mDirection = 1;
};
}
}
