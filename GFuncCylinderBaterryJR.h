#pragma once
#include <atlstr.h>

//namespace xf 
//{
	enum ERR_CODE2
	{
		OK = 0,
		NG = 1,
		errBatterySize = 100,
		errWidthBattery,
		errBeadingArea,
		errBeadingInsp,
		errJRFinding,
		errReferenceCase,
		errCheckBlackCloud,
		errCenterPin,
		errPixelSize,
		errPoleXDetection = 200,
		errPoleXDetection2,
		errPoleXDetection3,
		errPoleXDetection4,
		errPoleXDetection5,
		errPoleCathodeLine = 210,
		errPoleCathodeLine1,
		errPoleCathodeLine2,
		errPoleCathodeLine3,
		errPoleCathodeLine4,
		errPoleCathodeLine5,
		errPoleCathodeLine6,
		errPoleAnodeLine = 220,
		errPoleNotRefinable = 300,
		errPoleRefinement,
		errImageFormat = 310,
		errInspection = 400,
	};

	enum eInspectionNG_Code // Kumyang 적용
	{
		NG_UpperCat2CaseDistance = 2201,    // 상부 양극과 기준선 거리(단차) 불량
		NG_LowerAno2CathDistance = 2202,    // 하부 음양극간의 거리(단차) 불량,
		NG_LowerAno2CaseDistance = 2203,    // 하부 음극과 기준선(바닥면 or can) 거리 불량 (하부 삽입깊이 NG)

		NG_UpperImageFormat = 2204,    // 상부검사 image foramt이 안맞음
		NG_LowerImageFormat = 2205,    // 하부검사 image foramt이 안맞음
		NG_UpperAno2CaseVariation = 2206,    // 상부 음극과 기준선(비딩부) Max와 Min의 거리차이 불량 (Wave 심함),
		NG_LowerAno2CaseVariation = 2207,    // 하부 음극과 기준선(바닥면) Max와 Min의 거리차이 불량 (Wave 심함),

		NG_UpperEtc = 2208,	 // 상부검사에서 여러 조건이 안맞아 기타 불량 처리
		NG_LowerEtc = 2209,	 // 하부검사에서 여러 조건이 안맞아 기타 불량 처리

		NG_UpperAno2CathDistance = 2208,    // 상부 음양극간의 거리(단차) 불량, 4680 적용안됨
		NG_UpperAno2CaseDistance = 2208,    // 상부 음극과 기준선(비딩부) 거리 불량 (상부 삽입깊이 NG), 4680 적용안됨
		NG_LowerCenterPin = 2209,    // 하부 center pin 없음, 적용안됨 
		NG_UpperCellWidth = 2208,    // 상부검사 Cell의 폭이 설정 상한과 하한치 벗어남, 적용안됨
		NG_LowerCellWidth = 2209,    // 하부검사 Cell의 폭이 설정 상한과 하한치 벗어남, 적용안됨
	};
//}

	struct BatResultStruct {
		std::string finalDecision;           // Final Decision: true if no error, otherwise false
		std::string Description;            // Error Description
		std::vector<double> sCathode2Anode; // Positive to Negative
		std::vector<double> sAnode2Case;    // Negative to Case
		std::vector<double> sCathode2Case;  // Positive to Case
		std::vector<double> sXPos;          // Position of horizontal axis to left border battery

		std::string Anode2CathodeDecision;        // True if no error, otherwise false
		std::string Anodes2CaseDecision;          // True if no error, otherwise false
		std::string Cathode2CaseDecision;         // True if no error, otherwise false

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
		std::string cTime;
		int baselineLoc;
		std::vector<cv::Point> vtCathodes;        // Positive points
		std::vector<cv::Point> vtAnodes;          // Negative points

		int opCode;
		int ngCode;
		int setJudge;
		cv::Mat img8bit;
		cv::Mat resImg;
	};

namespace xvt {
	namespace battery {
		class BatteryInspectionResult;
	}

	struct InspectingItem
	{
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

	typedef struct CylinderJRInspection
	{
		int UseBatteryInspection;
		InspectingItem inspectingItems;
		int BatteryDirection;
		int RoiX;
		int RoiY;
		int RoiWidth;
		int RoiHeight;
		int minBeadingHeight;
		double D1StartPosition;
		int JR_ROIX;
		int JR_ROIY;
		int batteryThreshold;
		int poleRegionHeight;
		int skipPolesDistance;
		int CenterNeglectionWidth;
		double cathodeLineThredsholdInner;
		double cathodeLineThredsholdMiddle;
		double cathodeLineThredsholdOuter;
		int cathodeLineWindowSize;
		int PoleLeaningThreshold;
		int polesMaxDistance;
		int polesMinDistance;
		int tolerenceShift;
		int polesHeight;
		int blackCloudThreshold;
		int blackCloudOffset;
		double MinCathode2Anode;
		double MaxCathode2Anode;
		double MinAnode2Case;
		double MaxAnode2Case;
		double MinCathode2Case;
		double MaxCathode2Case;
		double variationAnode2Case;
		int mAnode2CaseOffset;
		int mCathode2AnodeOffset;
		int mCathode2CaseOffset;
		int displayMode;
		int mLineType; // 0: Dotted Line, 1: Curved Line
		double blackCloudRatioAreaThreshold;
		int blackCloudHeightThreshold;
		double minLeaningDistance;
		int OneSidePoleNumb;
		int isCheckPoleNo;
		double anodeThresholdInner;
		double anodeThresholdMiddle;
		double anodeThresholdOuter;
		double pixelSize;
		double minProminence;
		int isCheckPin;
		int isSaveResults;
		double depthScale;
		int additionalLine;
		int gamma;
		double cellSizeLowBound;
		double cellSizeHighBound;
		CString sCellID;
		bool isCheckBeading = false;
		int Angle;
		int StepNo;
		int InspItem;

		std::string cellID;  // added by shpark
		std::string resultPath; // added by shpark

	} CYLINDERJRINSPECTION;

}