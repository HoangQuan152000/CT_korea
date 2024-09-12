#pragma once
//#include "ImageInfo.h"
//#include "InspectionUnit.h"
//#include "FactoryAlgorithm.h"
//#include "InspectComponent.h"

#include <atlstr.h>
#include <atltime.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//using namespace xvt;
//using namespace xvt::battery;

#define MAX_REPEAT_NO  	200  // 장비 최대 반복 회수
#define MAX_SAMPLE_NO  	10  // 샘플(시료셀) 개수
#define MAX_POLE_NO     110  // GRR check할 폴의 최대 개수
#define GRR_CHECK_RESOL	0.05     // 특정 폴간격의 차이가 중간값과 비교하여 이값보다 작은 경우 기록 
#define RESIZE_SCALE   10   // 배터리외곽 영역 추출시 이미지 축소 크기
#define IMAGE_MAX_SIZE 62500000 // 2500 x 2500
#define MAX_QUEUE_COUNT		10

#define NO_BAT_INSPECTION			0
#define NORMAL_BAT_INSPECTION		1
#define GRR_TEST_INSPECTION1		2    // 10개의 cell을 9회 검사 즉 총 90회 검사후 GRR 데이타 생성 - 원통형, stack형에 적용
#define GRR_TEST_INSPECTION2		3    // 10개의 cell을 50회 검사 즉 총 500회 검사후 GRR 데이타 생성 - 원통형에 적용
#define GRR_TEST_INSPECTION3		4    // 10개의 cell을 200회 검사 즉 총 2000회 검사후 GRR 데이타 생성 - 원통형에 적용
#define NCELL_REPEAT_TEST			5	 // BOSK에서 적용, 1 또는 N cell에 대한 반복성 테스트 목적
#define STAIRS_TYPE_BAT_INSPECTION	10   // 원통형 배터리용 Jig에 적용
#define BALL_TYPE_BAT_INSPECTION	11   // 1 ~ 2 mm의 ball 검출 : GOC, BOSK 적용가능
#define RING_JIG_INSPECTION			12   // 원통형에 적용
#define PIXEL_AUTO_CAL				12   // BOSK에서만 적용
#define MASTER_JIG_INSPECTION		13   // BOSK용 Master jig(ball 2ea, 8 mm steel, 6 mm steel이 stacking cell에 적용되는 경우에 적용

#define GRAYVALUE_ROI				15

#define CELL_IMAGE_CHECK			20
#define CELL_IMAGE_MULTI_CHECK		21
#define NORMAL_MULTI_INSPECTION		30

#define DATA_WRITE_BAT_INSPECTION	101

#define L1R1_DISPLAY_HIDE
#define MAX_HEAD_NUM 64

// ----------------- Added for Upper Gage R&R test ------------------------------

static double dGRR_UpperPoleDiffLength[MAX_REPEAT_NO][MAX_SAMPLE_NO][MAX_POLE_NO];
static std::string dGRR_UpperInspTime[MAX_REPEAT_NO][MAX_SAMPLE_NO];
static std::string dGRR_UpperResult[MAX_REPEAT_NO][MAX_SAMPLE_NO];

// ----------------- Added for Lower Gage R&R test ------------------------------

static double dGRR_LowerPoleDiffLength[MAX_REPEAT_NO][MAX_SAMPLE_NO][MAX_POLE_NO];
static std::string dGRR_LowerInspTime[MAX_REPEAT_NO][MAX_SAMPLE_NO];
static std::string dGRR_LowerResult[MAX_REPEAT_NO][MAX_SAMPLE_NO];

void SaveInspectionResult(cv::Mat orgImg, cv::Mat resImg, BOOL isUpper, std::string nameStr, int cellID, std::string wStr);

class PreprocessLUT
{
public:
	BYTE gammaLUT[100][256];
	int  gammaLUT2[256];
	float logTable[65536];

	PreprocessLUT()
	{
		for (int gamma = 0; gamma < 100; gamma++)
			for (int j = 0; j < 256; j++) gammaLUT[gamma][j] = (BYTE)(pow((1 - (j / 255.0)), (float)gamma/10.0) * 255.0);
		for (int i = 0; i < 256; i++) gammaLUT2[i] = (int)(pow((1 - (i / 255.0)), 2.0) * 255.0*256.0);

		logTable[0] = 0;
		for (int i = 1; i < 65536; i++) 
			logTable[i] = (float)log(i);
	}
	void CreateGammaLUT(float gamma)
	{
		if ((gamma < 1.0) || (gamma > 9.9)) return;
		for (int i = 0; i < 256; i++) gammaLUT[(int)(gamma*10)][i] = (BYTE)(pow((1 - (i / 255.0)), gamma) * 255.0);
		for (int i = 0; i < 256; i++) gammaLUT2[i] = (int)(pow((1 - (i / 255.0)), gamma) * 255.0 * 256.0);
	}
};

static PreprocessLUT DC_Prep;

namespace xf
{
	namespace ImageProcessor
	{
		class CImageProcessorBase
		{
		public:
			CImageProcessorBase();
			virtual ~CImageProcessorBase();

		public:
			/// <summary>
			/// Image Processing function
			/// </summary>
			/// <param name="pGI">Grab Image, Include Average Image and Result Image</param>
			/// <param name="cIU">Inspection Unit, Include Setting Value and Result Value</param>
//			virtual void ImageProcessing(cv::Mat img, xvt::CylinderCTInspection BatLowerInsp, xvt::battery::BatteryInspectionResult& BIresult) PURE;

		public:
			const static int NGRAY = 256;

			static cv::Rect RoiRefinement(cv::Rect selectROI, cv::Size imageSize);
			static std::string to_string_with_precision(double a_value, int n);
			static void plotHist(cv::Mat img, std::string tilte);
			static void fit_circle(std::vector<cv::Point>& pnts, cv::Point2d& centre, double& radius);
			static float getAccHistAtThreshold(cv::Mat image, int threshold);
			std::vector<double> weightedAvgSmoothen(std::vector<double> values);
			static std::vector<double> gaussSmoothen(std::vector<double> values, double sigma, int samples);
			cv::Point rotate180Point(cv::Point pntIn, cv::Point center);
			static void WrProcessingData(std::string& wStr, int manualProcCount, int wrPoleNo, double poleDiffQueue[][MAX_POLE_NO], double poleDiffLength[], double poleDiffLength2[], double poleDiffLength3[], int commaNum);
			static void GRR_Cal_EV_AV(double* poleDiffLen, double &dEV, double &dAV);
			static cv::Rect CellWidthCalcul(cv::Mat Img, cv::Rect Roi, BOOL displayMode);
			static cv::Rect BallOutsideBoundFinding(cv::Mat Img, cv::Mat grayImg, cv::Rect SettingRoi, int displayMode); // master jig ball 찾기
			static cv::Rect RectangleSteelOutsideBoundFinding(cv::Mat Img, cv::Mat grayImg, cv::Rect SettingRoi, int displayMode); // rectangle steel 6 ~ 8mm 찾기
			static std::vector<cv::Rect> MultiBallOutsideBoundFinding(cv::Mat Img, cv::Mat grayImg, cv::Rect SettingRoi, int displayMode, int ballNo); // master jig ball 찾기, 추가 20230410

			static cv::Rect FindBatteryRoi(cv::Mat image, cv::Rect Roi, int displayMode);
			static std::vector<float> FindMasterDistance(cv::Mat srcImg);
			static void GammaCorrection(cv::Mat& Img, int gamma);
			static void GainCorrection(int* ImgArray, cv::Mat& gammaImg8bit, int gamma, int rowNum, int colNum, cv::Rect outputROI, int whiteCalLine = 100, int option = 1, int addionalHeight = 100);  // global Preprocessing table을 참조
			static void GainCorrection2(int* ImgArray, int rowNum, int colNum, cv::Rect outputROI, int whiteCalLine = 10);  // global Preprocessing table을 참조
			static cv::Rect FindRoughBatteryROI(int *Img1, int rowNum, int colNum, cv::Rect Roi, float depthScaleFactor = 4, bool display = false);  // Img1 : 16bit
			static int MedianValueFrom3Value(int* value);
			static int MechaCalibrationDataLoad(double HeadCalibValue[], double refLoc[], double cellWidth[], std::string fileName);
			static cv::Rect FindCTCellOutsideRoi(cv::Mat image, cv::Rect Roi, int displayMode);
		};

		namespace ed
		{
			std::vector<std::list<cv::Point>> detectEdges(const cv::Mat& image,
				const int proposal_thresh = 80,
				const int anchor_interval = 4,
				const int anchor_thresh = 8);
		}
	}
}